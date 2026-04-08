//! HITL Daemon - Hardware-in-the-loop simulator for UAV testing
//!
//! This daemon connects to a Pixhawk flight controller via serial and provides
//! simulated sensor data for hardware-in-the-loop testing.

use clap::Parser;
use crossbeam_channel::{bounded, Receiver, Sender};
use mavlink::ardupilotmega::MavMessage;
use mavlink_io::async_io::{MavlinkIo, NshRequest};
use mavlink_io::serial::find_pixhawk_ports;
use protocol::ActuatorOutputs;
use hitl_physics::throttle_to_omega;
use hitl_sensors::{ImuConfig, SensorsConfig};
use simulation::{SimulationConfig, SimulationLoop, SimulationState};
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use tracing::{debug, error, info, trace, warn};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};
use websocket::{CommandType, StateUpdate, ValidatedNshCommand, WebSocketServer, WebSocketServerConfig};

const VERSION: &str = env!("CARGO_PKG_VERSION");

/// HITL Daemon - Hardware-in-the-loop simulator for UAV testing
#[derive(Parser, Debug)]
#[command(name = "hitl-daemon")]
#[command(version, about, long_about = None)]
struct Args {
    /// Serial port to use (auto-detect Pixhawk if not specified)
    #[arg(short, long)]
    port: Option<String>,

    /// Baud rate for serial communication
    #[arg(short, long, default_value = "921600")]
    baud: u32,

    /// WebSocket port for simulator connection
    #[arg(short, long, default_value = "9876")]
    websocket_port: u16,

    /// Reference latitude for GPS origin (degrees)
    #[arg(long, default_value = "40.015")]
    reference_lat: f64,

    /// Reference longitude for GPS origin (degrees)
    #[arg(long, default_value = "-105.2705")]
    reference_lon: f64,

    /// Reference altitude MSL (meters)
    #[arg(long, default_value = "1655.0")]
    reference_alt: f64,

    /// Simulation tick rate (Hz)
    #[arg(long, default_value = "400")]
    tick_rate: u32,

    /// GPS update rate (Hz)
    #[arg(long, default_value = "10")]
    gps_rate: u32,

    /// Run in simulation-only mode (no Pixhawk required)
    #[arg(long)]
    sim_only: bool,

    /// UDP port for QGroundControl bridge (0 to disable)
    #[arg(long, default_value = "14550")]
    qgc_udp: u16,
}

fn init_tracing() {
    tracing_subscriber::registry()
        .with(EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")))
        .with(tracing_subscriber::fmt::layer())
        .init();
}

fn detect_or_use_port(specified_port: Option<String>) -> Option<String> {
    if let Some(port) = specified_port {
        info!(port = %port, "Using specified serial port");
        return Some(port);
    }

    info!("No port specified, auto-detecting Pixhawk...");
    let ports = find_pixhawk_ports();

    if ports.is_empty() {
        warn!("No Pixhawk devices detected");
        return None;
    }

    if ports.len() > 1 {
        info!("Multiple Pixhawk devices found:");
        for port in &ports {
            info!("  - {}", port);
        }
        info!("Using first detected port");
    }

    let port = ports.into_iter().next().unwrap();
    info!(port = %port, "Auto-detected Pixhawk");
    Some(port)
}

/// Spawn the simulation loop in a dedicated thread
fn spawn_simulation_thread(
    config: SimulationConfig,
    actuator_rx: Receiver<ActuatorOutputs>,
    mav_tx: Sender<MavMessage>,
    _shutdown: Arc<AtomicBool>,
) -> (thread::JoinHandle<()>, SimulationState) {
    let mut sim_loop = SimulationLoop::new(config, actuator_rx, mav_tx);
    let state = sim_loop.state_handle();

    let handle = thread::Builder::new()
        .name("simulation".to_string())
        .spawn(move || {
            sim_loop.run();
            info!("Simulation thread exiting");
        })
        .expect("Failed to spawn simulation thread");

    (handle, state)
}

/// Simulation-only mode: generate fake actuator commands for testing
fn spawn_sim_only_actuator_thread(
    actuator_tx: Sender<ActuatorOutputs>,
    shutdown: Arc<AtomicBool>,
) -> thread::JoinHandle<()> {
    thread::Builder::new()
        .name("sim-actuator".to_string())
        .spawn(move || {
            info!("Simulation-only actuator thread started (generating hover commands)");
            let mut tick = 0u64;

            while !shutdown.load(Ordering::Relaxed) {
                // Generate hover throttle (~50%)
                let actuator = ActuatorOutputs {
                    timestamp_us: tick * 2500,
                    motors: [0.5, 0.5, 0.5, 0.5],
                    mode: protocol::FlightMode::HilArmed,
                    controls: [0.0; 16],
                };

                if actuator_tx.send(actuator).is_err() {
                    break;
                }

                tick += 1;
                thread::sleep(Duration::from_micros(2500));
            }

            info!("Simulation-only actuator thread exiting");
        })
        .expect("Failed to spawn simulation-only actuator thread")
}

/// Create WebSocket state update from simulation state
fn create_state_update(sim_state: &SimulationState) -> StateUpdate {
    let state = sim_state.read();
    let q = &state.quadrotor;

    StateUpdate {
        timestamp_us: state.sim_time_us,
        position_ned: [q.position[0] as f32, q.position[1] as f32, q.position[2] as f32],
        velocity_ned: [q.velocity[0] as f32, q.velocity[1] as f32, q.velocity[2] as f32],
        quaternion_wxyz: [
            q.quaternion.w as f32,
            q.quaternion.i as f32,
            q.quaternion.j as f32,
            q.quaternion.k as f32,
        ],
        angular_velocity: [
            q.angular_velocity[0] as f32,
            q.angular_velocity[1] as f32,
            q.angular_velocity[2] as f32,
        ],
        motor_rpms: if state.armed {
            state.motor_commands.map(|c| {
                let omega = throttle_to_omega(c as f64);
                (omega * 60.0 / (2.0 * std::f64::consts::PI)) as f32
            })
        } else {
            [0.0; 4] // Show 0 RPM when disarmed
        },
        battery_voltage: 16.8,
        battery_percent: 100,
        armed: state.armed,
        flight_mode: state.flight_mode,
    }
}

#[tokio::main]
async fn main() {
    init_tracing();

    let args = Args::parse();

    info!(version = VERSION, "Starting HITL Daemon");

    // Determine operating mode
    let port = if args.sim_only {
        info!("Running in simulation-only mode (no Pixhawk connection)");
        None
    } else {
        detect_or_use_port(args.port)
    };

    let sim_only_mode = port.is_none();
    if sim_only_mode && !args.sim_only {
        warn!("No Pixhawk found, falling back to simulation-only mode");
        warn!("Use --sim-only to suppress this warning");
    }

    // Create simulation configuration with minimal sensor noise for HIL
    // PX4 needs some noise to validate sensors, but too much causes preflight failures
    // Sensor noise tuned for PX4 HIL - EKF2 needs realistic variance but not too much drift
    // HITL sensor noise tuned for PX4's EKF2 sensor validators
    // Key insight: PX4 rejects sensors with variance too low (stuck sensor detection)
    // Use realistic noise levels but disable bias DRIFT (which causes "High Gyro Bias")
    let clean_sensors = SensorsConfig {
        imu: ImuConfig {
            gyro_noise_density: 0.0008,   // Default: realistic noise level
            accel_noise_density: 0.006,   // Default: realistic noise level
            gyro_bias_sigma: 0.0,         // CRITICAL: No bias drift in HITL
            gyro_bias_tau: 1000.0,        // Long time constant (unused since sigma=0)
        },
        baro: hitl_sensors::BaroConfig::default(),
        gps: hitl_sensors::GpsConfig {
            position_drift_tau: 1000.0,      // Very slow drift
            position_drift_sigma: 0.0,       // No position drift for HITL
            horizontal_noise_sigma: 0.1,     // 10cm noise - tight for HITL
            altitude_noise_sigma: 0.3,       // 30cm altitude noise
            velocity_noise_sigma: 0.05,      // 5cm/s velocity noise
            delay_ms: 80.0,                  // Moderate delay
            update_rate_hz: 10.0,            // 10 Hz GPS
        },
        mag: hitl_sensors::MagConfig::default(),
    };

    let sim_config = SimulationConfig {
        reference_lat: args.reference_lat,
        reference_lon: args.reference_lon,
        reference_alt: args.reference_alt,
        tick_rate_hz: args.tick_rate,
        gps_rate_hz: args.gps_rate,
        sensors: clean_sensors,
        ..Default::default()
    };

    info!(
        port = port.as_deref().unwrap_or("none"),
        baud = args.baud,
        websocket_port = args.websocket_port,
        reference_lat = sim_config.reference_lat,
        reference_lon = sim_config.reference_lon,
        reference_alt = sim_config.reference_alt,
        tick_rate = sim_config.tick_rate_hz,
        gps_rate = sim_config.gps_rate_hz,
        sim_only = sim_only_mode,
        "HITL Daemon configuration"
    );

    // Create channels
    // actuator_tx/rx: MAVLink receiver -> Simulation (HIL_ACTUATOR_CONTROLS)
    // sim_mav_tx/rx: Simulation -> MAVLink sender (HIL_SENSOR, HIL_GPS)
    let (actuator_tx, actuator_rx) = bounded::<ActuatorOutputs>(16);
    let (sim_mav_tx, sim_mav_rx) = bounded::<MavMessage>(64);

    // Shutdown signal
    let shutdown = Arc::new(AtomicBool::new(false));

    // Set up Ctrl+C handler
    let shutdown_ctrlc = shutdown.clone();
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.ok();
        info!("Received Ctrl+C, initiating shutdown...");
        shutdown_ctrlc.store(true, Ordering::SeqCst);
    });

    // Spawn simulation thread
    let (sim_handle, sim_state) = spawn_simulation_thread(
        sim_config,
        actuator_rx,
        sim_mav_tx,
        shutdown.clone(),
    );

    // Thread handles to join later
    let mut thread_handles = vec![sim_handle];

    // MAVLink I/O (only if we have a port)
    // Create UDP socket for QGC bridge (if enabled)
    // Bind to a known port so QGC can send commands back to us
    let qgc_local_port = args.qgc_udp + 10; // 14560 by default
    let qgc_target: std::net::SocketAddr = format!("127.0.0.1:{}", args.qgc_udp).parse().unwrap();
    let qgc_socket: Option<Arc<UdpSocket>> = if args.qgc_udp > 0 {
        match UdpSocket::bind(format!("0.0.0.0:{}", qgc_local_port)) {
            Ok(socket) => {
                socket.set_nonblocking(true).ok();
                info!("QGC UDP bridge: local port {} ↔ QGC port {}", qgc_local_port, args.qgc_udp);
                Some(Arc::new(socket))
            }
            Err(e) => {
                warn!(error = %e, "Failed to create UDP socket for QGC bridge");
                None
            }
        }
    } else {
        None
    };

    // NSH command channel (WebSocket handler -> NSH processor)
    let (nsh_cmd_tx, mut nsh_cmd_rx) = tokio::sync::mpsc::channel::<ValidatedNshCommand>(32);

    let mav_io: Option<Arc<MavlinkIo>> = if let Some(ref port_path) = port {
        let (mut mav_io, tx_to_app, rx_from_app, nsh_resp_tx, nsh_req_rx) = MavlinkIo::new();

        // Spawn MAVLink I/O tasks
        if let Err(e) = mav_io.spawn(port_path, args.baud, tx_to_app, rx_from_app, nsh_resp_tx, nsh_req_rx).await {
            error!(error = %e, "Failed to open serial port");
            error!("Falling back to simulation-only mode");

            // Fall back to sim-only
            let handle = spawn_sim_only_actuator_thread(actuator_tx, shutdown.clone());
            thread_handles.push(handle);
            None
        } else {
            let mav_io = Arc::new(mav_io);

            // Spawn receiver thread (Pixhawk -> simulation + QGC)
            let mav_io_recv = mav_io.clone();
            let shutdown_recv = shutdown.clone();
            let qgc_socket_send = qgc_socket.clone();
            let qgc_target_send = qgc_target;
            let recv_handle = thread::Builder::new()
                .name("mav-receiver".to_string())
                .spawn(move || {
                    info!("MAVLink receiver thread started");
                    while !shutdown_recv.load(Ordering::Relaxed) {
                        if let Some((header, msg)) = mav_io_recv.try_recv() {
                            // Forward to QGC via UDP
                            if let Some(ref socket) = qgc_socket_send {
                                let mut buf = Vec::new();
                                if mavlink::write_v2_msg(&mut buf, header, &msg).is_ok() {
                                    let _ = socket.send_to(&buf, qgc_target_send);
                                }
                            }

                            // Process message locally
                            match &msg {
                                MavMessage::HIL_ACTUATOR_CONTROLS(hil) => {
                                    debug!(
                                        motors = ?[hil.controls[0], hil.controls[1], hil.controls[2], hil.controls[3]],
                                        mode = ?hil.mode,
                                        "Received HIL_ACTUATOR_CONTROLS"
                                    );
                                    match ActuatorOutputs::from_mavlink(&msg) {
                                        Ok(actuator) => {
                                            if actuator_tx.send(actuator).is_err() {
                                                error!("Failed to send actuator to simulation");
                                                break;
                                            }
                                        }
                                        Err(e) => {
                                            warn!(error = %e, "Failed to parse HIL_ACTUATOR_CONTROLS");
                                        }
                                    }
                                }
                                MavMessage::HEARTBEAT(hb) => {
                                    debug!(
                                        system_id = header.system_id,
                                        autopilot = ?hb.autopilot,
                                        "Received HEARTBEAT"
                                    );
                                }
                                MavMessage::STATUSTEXT(st) => {
                                    let text = String::from_utf8_lossy(&st.text);
                                    info!(severity = ?st.severity, text = %text.trim_end_matches('\0'), "FC");
                                }
                                other => {
                                    trace!(msg_type = ?std::mem::discriminant(other), "Received MAVLink message");
                                }
                            }
                        } else {
                            thread::sleep(Duration::from_micros(500));
                        }
                    }
                    info!("MAVLink receiver thread exiting");
                })
                .expect("Failed to spawn MAVLink receiver thread");
            thread_handles.push(recv_handle);

            // Spawn sender thread (simulation -> Pixhawk)
            let mav_io_send = mav_io.clone();
            let shutdown_send = shutdown.clone();
            let send_handle = thread::Builder::new()
                .name("mav-sender".to_string())
                .spawn(move || {
                    info!("MAVLink sender thread started");
                    while !shutdown_send.load(Ordering::Relaxed) {
                        match sim_mav_rx.recv_timeout(Duration::from_millis(10)) {
                            Ok(msg) => {
                                if let Err(e) = mav_io_send.send(msg) {
                                    error!(error = %e, "Failed to send to Pixhawk");
                                    break;
                                }
                            }
                            Err(crossbeam_channel::RecvTimeoutError::Timeout) => continue,
                            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
                        }
                    }
                    info!("MAVLink sender thread exiting");
                })
                .expect("Failed to spawn MAVLink sender thread");
            thread_handles.push(send_handle);

            // Spawn QGC -> PX4 forwarder thread (if UDP enabled)
            if let Some(qgc_recv) = qgc_socket.clone() {
                let mav_io_qgc = mav_io.clone();
                let shutdown_qgc = shutdown.clone();
                let qgc_handle = thread::Builder::new()
                    .name("qgc-to-px4".to_string())
                    .spawn(move || {
                        info!("QGC→PX4 forwarder thread started (listening for commands)");
                        let mut buf = [0u8; 280];
                        while !shutdown_qgc.load(Ordering::Relaxed) {
                            match qgc_recv.recv_from(&mut buf) {
                                Ok((n, _src)) if n > 0 => {
                                    // Parse and forward to PX4
                                    use mavlink::peek_reader::PeekReader;
                                    use std::io::Cursor;
                                    let cursor = Cursor::new(&buf[..n]);
                                    let mut reader = PeekReader::new(cursor);
                                    if let Ok((_, msg)) = mavlink::read_v2_msg::<MavMessage, _>(&mut reader) {
                                        trace!("Forwarding QGC command to PX4: {:?}", std::mem::discriminant(&msg));
                                        if let Err(e) = mav_io_qgc.send(msg) {
                                            debug!(error = %e, "Failed to forward QGC message to PX4");
                                        }
                                    }
                                }
                                Ok(_) => thread::sleep(Duration::from_millis(5)),
                                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                                    thread::sleep(Duration::from_millis(5));
                                }
                                Err(_) => thread::sleep(Duration::from_millis(10)),
                            }
                        }
                        info!("QGC→PX4 forwarder thread exiting");
                    })
                    .expect("Failed to spawn QGC forwarder thread");
                thread_handles.push(qgc_handle);
            }

            Some(mav_io)
        }
    } else {
        // Simulation-only mode: generate fake actuator commands
        let handle = spawn_sim_only_actuator_thread(actuator_tx, shutdown.clone());
        thread_handles.push(handle);
        None
    };

    // Broadcast channel for NSH responses (to WebSocket clients)
    let (nsh_resp_broadcast_tx, _) = tokio::sync::broadcast::channel::<websocket::NshResponse>(64);

    // Create WebSocket server
    let ws_config = WebSocketServerConfig {
        port: args.websocket_port,
        update_rate_hz: 30,
        allowed_origins: vec![],
    };
    let mut ws_server = WebSocketServer::new(ws_config);
    let state_tx = ws_server.state_sender();
    let mut command_rx = ws_server.take_command_receiver();

    // Enable NSH support if we have a Pixhawk connection
    if mav_io.is_some() {
        ws_server.set_nsh_sender(nsh_cmd_tx);
        ws_server.set_nsh_response_receiver(nsh_resp_broadcast_tx.subscribe());
    }

    // Spawn WebSocket server task
    let serial_port_label = port.clone().unwrap_or_else(|| "simulation".to_string());
    let ws_handle = tokio::spawn(async move {
        let version_parts: Vec<u8> = VERSION
            .split('.')
            .take(2)
            .filter_map(|s| s.parse().ok())
            .collect();
        let version_major = version_parts.first().copied().unwrap_or(0);
        let version_minor = version_parts.get(1).copied().unwrap_or(1);

        if let Err(e) = ws_server.run(version_major, version_minor, serial_port_label).await {
            error!("WebSocket server error: {}", e);
        }
    });

    // Spawn task to handle WebSocket commands
    let shutdown_ws_cmd = shutdown.clone();
    let sim_state_cmd = sim_state.clone();
    let ws_cmd_handle = tokio::spawn(async move {
        while !shutdown_ws_cmd.load(Ordering::Relaxed) {
            match command_rx.recv().await {
                Some(validated_cmd) => {
                    info!(
                        client_id = validated_cmd.client_id,
                        command_id = validated_cmd.command.command_id,
                        ?validated_cmd.command.command_type,
                        "Received command from WebSocket client"
                    );

                    // Handle commands
                    match validated_cmd.command.command_type {
                        CommandType::Arm => {
                            info!("Arming (simulation ignores, FC controls this)");
                        }
                        CommandType::Disarm => {
                            info!("Disarming (simulation ignores, FC controls this)");
                        }
                        CommandType::Rtl => {
                            info!("RTL command (resetting simulation)");
                            sim_state_cmd.reset();
                        }
                        CommandType::EmergencyStop => {
                            info!("Emergency stop - stopping simulation");
                            sim_state_cmd.stop();
                        }
                        _ => {
                            debug!(cmd = ?validated_cmd.command.command_type, "Command forwarded to FC");
                        }
                    }
                }
                None => break,
            }
        }
    });

    // Spawn task to broadcast state updates to WebSocket clients
    let shutdown_ws_state = shutdown.clone();
    let sim_state_broadcast = sim_state.clone();
    let ws_state_handle = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(33)); // ~30 Hz

        while !shutdown_ws_state.load(Ordering::Relaxed) {
            interval.tick().await;

            let state_update = create_state_update(&sim_state_broadcast);
            let _ = state_tx.send(state_update);
        }
    });

    // Spawn NSH command handler task (if Pixhawk connected)
    let shutdown_nsh = shutdown.clone();
    let nsh_handle = if let Some(ref mav) = mav_io {
        let mav_io_nsh = mav.clone();
        Some(tokio::spawn(async move {
            info!("NSH handler task started");

            // Pending request tracking — serialized: one request at a time,
            // new requests queue behind the current one via the mpsc channel.
            let mut current_request_id: Option<u32> = None;
            let mut response_buffer: Vec<u8> = Vec::new();
            let mut request_deadline: Option<tokio::time::Instant> = None;

            loop {
                if shutdown_nsh.load(Ordering::Relaxed) {
                    break;
                }

                // Use select to handle both commands and response polling
                tokio::select! {
                    // Process incoming NSH commands from WebSocket clients
                    // Only accept a new command when no request is in-flight
                    Some(cmd) = nsh_cmd_rx.recv(), if current_request_id.is_none() => {
                        debug!(
                            request_id = cmd.request_id,
                            client_id = cmd.client_id,
                            cmd = %cmd.command,
                            "Processing NSH command"
                        );

                        // Send command via SERIAL_CONTROL
                        let mut data = cmd.command.into_bytes();
                        data.push(b'\n'); // Add newline to execute command

                        let request = NshRequest {
                            request_id: cmd.request_id,
                            data,
                            timeout_ms: cmd.timeout_ms,
                        };

                        match mav_io_nsh.send_nsh(request) {
                            Ok(_) => {
                                current_request_id = Some(cmd.request_id);
                                response_buffer.clear();
                                let timeout_ms = if cmd.timeout_ms == 0 { 2000 } else { cmd.timeout_ms as u64 };
                                request_deadline = Some(tokio::time::Instant::now() + Duration::from_millis(timeout_ms));
                            }
                            Err(e) => {
                                warn!(error = %e, "Failed to send NSH command");
                                let _ = nsh_resp_broadcast_tx.send(websocket::NshResponse {
                                    request_id: cmd.request_id,
                                    success: false,
                                    complete: true,
                                    output: format!("Failed to send: {}", e),
                                });
                            }
                        }
                    }

                    // Poll for responses (check every 10ms)
                    _ = tokio::time::sleep(Duration::from_millis(10)) => {
                        // Check for NSH responses
                        while let Some(resp) = mav_io_nsh.try_recv_nsh() {
                            response_buffer.extend_from_slice(&resp.data);

                            if resp.complete {
                                // Send complete response to WebSocket clients
                                if let Some(req_id) = current_request_id.take() {
                                    let output = String::from_utf8_lossy(&response_buffer).to_string();
                                    debug!(request_id = req_id, len = output.len(), "NSH response complete");

                                    let _ = nsh_resp_broadcast_tx.send(websocket::NshResponse {
                                        request_id: req_id,
                                        success: true,
                                        complete: true,
                                        output,
                                    });
                                    response_buffer.clear();
                                    request_deadline = None;
                                }
                            }
                        }

                        // Check for request timeout
                        if let (Some(req_id), Some(deadline)) = (current_request_id, request_deadline) {
                            if tokio::time::Instant::now() >= deadline {
                                warn!(request_id = req_id, "NSH request timed out");
                                let partial = String::from_utf8_lossy(&response_buffer).to_string();
                                let _ = nsh_resp_broadcast_tx.send(websocket::NshResponse {
                                    request_id: req_id,
                                    success: false,
                                    complete: true,
                                    output: if partial.is_empty() {
                                        "Request timed out".to_string()
                                    } else {
                                        format!("{}\n[timed out]", partial)
                                    },
                                });
                                current_request_id = None;
                                response_buffer.clear();
                                request_deadline = None;
                            }
                        }
                    }
                }
            }

            info!("NSH handler task exiting");
        }))
    } else {
        None
    };

    info!("HITL Daemon running. Press Ctrl+C to stop.");
    info!(
        websocket_url = format!("ws://localhost:{}/ws", args.websocket_port),
        "WebSocket server listening"
    );

    if sim_only_mode {
        info!("Mode: SIMULATION ONLY (no flight controller)");
    } else {
        info!("Mode: HARDWARE-IN-THE-LOOP (connected to Pixhawk)");
    }

    // Wait for shutdown signal
    while !shutdown.load(Ordering::Relaxed) && sim_state.is_running() {
        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    // Signal shutdown
    shutdown.store(true, Ordering::SeqCst);
    sim_state.stop();

    // MavlinkIo will be dropped when Arc refcount reaches zero
    // The internal shutdown flag prevents hanging
    drop(mav_io);

    // Cancel async tasks
    ws_handle.abort();
    ws_cmd_handle.abort();
    ws_state_handle.abort();
    if let Some(handle) = nsh_handle {
        handle.abort();
    }

    // Wait for threads
    for handle in thread_handles {
        let _ = handle.join();
    }

    info!("HITL Daemon shutdown complete");
}
