//! HITL Daemon - Hardware-in-the-loop simulator for UAV testing
//!
//! This daemon connects to a PX4-compatible flight controller via serial and provides
//! simulated sensor data for hardware-in-the-loop testing.

use clap::Parser;
use crossbeam_channel::{bounded, Receiver, Sender};
use mavlink::ardupilotmega::MavMessage;
use mavlink_io::async_io::{MavlinkIo, NshRequest, reconnect_delay};
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
use websocket::{CommandType, ConnectionStatus, StateUpdate, ValidatedNshCommand, WebSocketServer, WebSocketServerConfig};

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

    info!("No port specified, auto-detecting FC...");
    let ports = find_pixhawk_ports();

    if ports.is_empty() {
        warn!("No PX4 boards detected");
        return None;
    }

    if ports.len() > 1 {
        info!("Multiple PX4 boards found:");
        for port in &ports {
            info!("  - {}", port);
        }
        info!("Using first detected port");
    }

    let port = ports.into_iter().next().unwrap();
    info!(port = %port, "Auto-detected FC");
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
    // sim_only_mode is ONLY true when --sim-only flag is explicitly passed
    let sim_only_mode = args.sim_only;
    let initial_port = if sim_only_mode {
        info!("Running in simulation-only mode (no FC connection)");
        None
    } else {
        let port = detect_or_use_port(args.port.clone());
        if port.is_none() {
            info!("No FC found at startup, will keep scanning...");
        }
        port
    };

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
        port = initial_port.as_deref().unwrap_or("none"),
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

    // Shared MAVLink I/O - can be updated by connection manager
    let mav_io: Arc<tokio::sync::RwLock<Option<Arc<MavlinkIo>>>> = Arc::new(tokio::sync::RwLock::new(None));

    // In sim-only mode, always generate fake actuator commands
    // In normal mode, fake actuators run until FC connects
    if sim_only_mode {
        let handle = spawn_sim_only_actuator_thread(actuator_tx.clone(), shutdown.clone());
        thread_handles.push(handle);
    }

    // Broadcast channel for NSH responses (to WebSocket clients)
    let (nsh_resp_broadcast_tx, _) = tokio::sync::broadcast::channel::<websocket::NshResponse>(64);

    // Broadcast channel for connection status (to WebSocket clients)
    let (conn_status_tx, _) = tokio::sync::broadcast::channel::<ConnectionStatus>(16);

    // Create WebSocket server
    let ws_config = WebSocketServerConfig {
        port: args.websocket_port,
        update_rate_hz: 30,
        allowed_origins: vec![],
    };
    let mut ws_server = WebSocketServer::new(ws_config);
    let state_tx = ws_server.state_sender();
    let mut command_rx = ws_server.take_command_receiver();

    // Always enable NSH support (will be available when Pixhawk connects)
    ws_server.set_nsh_sender(nsh_cmd_tx);
    ws_server.set_nsh_response_receiver(nsh_resp_broadcast_tx.subscribe());

    // Always enable connection status broadcasting
    ws_server.set_connection_status_receiver(conn_status_tx.subscribe());

    // Spawn WebSocket server task
    let serial_port_label = initial_port.clone().unwrap_or_else(|| "scanning".to_string());
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

    // Clone broadcast tx before it's moved into tasks
    let nsh_resp_broadcast_tx_for_reconnect = nsh_resp_broadcast_tx.clone();

    // Spawn NSH command handler task (always runs, checks for mav_io dynamically)
    let shutdown_nsh = shutdown.clone();
    let mav_io_nsh = mav_io.clone();
    let nsh_handle = if !sim_only_mode {
        Some(tokio::spawn(async move {
            info!("NSH handler task started");

            // Pending request tracking — serialized: one request at a time
            let mut current_request_id: Option<u32> = None;
            let mut response_buffer: Vec<u8> = Vec::new();
            let mut request_deadline: Option<tokio::time::Instant> = None;
            let mut cached_mav_io: Option<Arc<MavlinkIo>> = None;

            loop {
                if shutdown_nsh.load(Ordering::Relaxed) {
                    break;
                }

                // Refresh cached mav_io reference
                {
                    let guard = mav_io_nsh.read().await;
                    if cached_mav_io.as_ref().map(|m| m.is_disconnected()).unwrap_or(true) {
                        cached_mav_io = guard.clone();
                    }
                }

                // Use select to handle both commands and response polling
                tokio::select! {
                    // Process incoming NSH commands from WebSocket clients
                    Some(cmd) = nsh_cmd_rx.recv(), if current_request_id.is_none() => {
                        debug!(
                            request_id = cmd.request_id,
                            client_id = cmd.client_id,
                            cmd = %cmd.command,
                            "Processing NSH command"
                        );

                        // Check if we have a connection
                        let Some(ref mav) = cached_mav_io else {
                            let _ = nsh_resp_broadcast_tx.send(websocket::NshResponse {
                                request_id: cmd.request_id,
                                success: false,
                                complete: true,
                                output: "No FC connected".to_string(),
                            });
                            continue;
                        };

                        if mav.is_disconnected() {
                            cached_mav_io = None;
                            let _ = nsh_resp_broadcast_tx.send(websocket::NshResponse {
                                request_id: cmd.request_id,
                                success: false,
                                complete: true,
                                output: "FC disconnected".to_string(),
                            });
                            continue;
                        }

                        // Send command via SERIAL_CONTROL
                        let mut data = cmd.command.into_bytes();
                        data.push(b'\n');

                        let request = NshRequest {
                            request_id: cmd.request_id,
                            data,
                            timeout_ms: cmd.timeout_ms,
                        };

                        match mav.send_nsh(request) {
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
                        if let Some(ref mav) = cached_mav_io {
                            // Check for NSH responses
                            while let Some(resp) = mav.try_recv_nsh() {
                                response_buffer.extend_from_slice(&resp.data);

                                // Check for completion: either count==0 OR nsh> prompt detected
                                let output_str = String::from_utf8_lossy(&response_buffer);
                                let has_prompt = output_str.contains("nsh> \x1b[K") || output_str.trim_end().ends_with("nsh>");
                                let is_complete = resp.complete || has_prompt;

                                if is_complete {
                                    if let Some(req_id) = current_request_id.take() {
                                        let output = output_str.to_string();
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
                        }

                        // Check for request timeout
                        if let (Some(req_id), Some(deadline)) = (current_request_id, request_deadline) {
                            if tokio::time::Instant::now() >= deadline {
                                let partial = String::from_utf8_lossy(&response_buffer).to_string();
                                // If we got output, treat as success (user got the data)
                                let has_output = !partial.trim().is_empty();
                                if has_output {
                                    debug!(request_id = req_id, "NSH request completed with output (timeout fallback)");
                                } else {
                                    warn!(request_id = req_id, "NSH request timed out with no output");
                                }
                                let _ = nsh_resp_broadcast_tx.send(websocket::NshResponse {
                                    request_id: req_id,
                                    success: has_output, // Success if we got output
                                    complete: true,
                                    output: if partial.is_empty() {
                                        "Request timed out".to_string()
                                    } else {
                                        partial // Don't append [timed out] - user got the data
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

    // Spawn connection monitor/reconnection task (if we have or want a FC connection)
    let reconnect_handle = if !sim_only_mode {
        let shutdown_reconnect = shutdown.clone();
        let mav_io_shared = mav_io.clone();
        let conn_status_tx_reconnect = conn_status_tx.clone();
        let _nsh_resp_broadcast_tx_reconnect = nsh_resp_broadcast_tx_for_reconnect;
        let actuator_tx_reconnect = actuator_tx.clone();
        let sim_mav_rx_reconnect = sim_mav_rx.clone();
        let qgc_socket_reconnect = qgc_socket.clone();
        let preferred_port = args.port.clone();
        let baud = args.baud;

        Some(tokio::spawn(async move {
            info!("Connection manager started");

            let mut retry_count: u8 = 0;
            let mut current_mav_io: Option<Arc<MavlinkIo>> = None;
            let mut receiver_handle: Option<tokio::task::JoinHandle<()>> = None;
            let mut sender_handle: Option<tokio::task::JoinHandle<()>> = None;

            // Send initial status - searching
            let _ = conn_status_tx_reconnect.send(ConnectionStatus {
                connected: false,
                reconnecting: true,
                retry_count: 0,
                serial_port: String::new(),
            });

            loop {
                if shutdown_reconnect.load(Ordering::Relaxed) {
                    break;
                }

                // Check if we have an active connection
                let is_connected = current_mav_io.as_ref().map(|m| !m.is_disconnected()).unwrap_or(false);

                if is_connected {
                    // Connection is alive, just check periodically
                    tokio::time::sleep(Duration::from_secs(1)).await;
                    continue;
                }

                // Not connected - clean up old connection if any
                if current_mav_io.is_some() {
                    warn!("FC connection lost");
                    current_mav_io = None;
                    *mav_io_shared.write().await = None;

                    // Abort old tasks
                    if let Some(h) = receiver_handle.take() { h.abort(); }
                    if let Some(h) = sender_handle.take() { h.abort(); }

                    let _ = conn_status_tx_reconnect.send(ConnectionStatus {
                        connected: false,
                        reconnecting: true,
                        retry_count,
                        serial_port: String::new(),
                    });
                }

                // Try to find a Pixhawk
                let port_path = if let Some(ref p) = preferred_port {
                    Some(p.clone())
                } else {
                    find_pixhawk_ports().into_iter().next()
                };

                let Some(port_path) = port_path else {
                    // No FC found, wait and retry
                    let delay = reconnect_delay(retry_count);
                    debug!(retry_count, delay_ms = delay.as_millis(), "No FC found, waiting...");

                    let _ = conn_status_tx_reconnect.send(ConnectionStatus {
                        connected: false,
                        reconnecting: true,
                        retry_count,
                        serial_port: String::new(),
                    });

                    tokio::time::sleep(delay).await;
                    if retry_count < 255 { retry_count += 1; }
                    continue;
                };

                info!(port = %port_path, "Found FC, connecting...");

                // Create MAVLink I/O
                let (mut new_mav_io, tx_to_app, rx_from_app, nsh_resp_tx, nsh_req_rx) = MavlinkIo::new();

                match new_mav_io.spawn(&port_path, baud, tx_to_app, rx_from_app, nsh_resp_tx.clone(), nsh_req_rx).await {
                    Ok(()) => {
                        info!(port = %port_path, "Connected to FC!");
                        retry_count = 0;

                        let new_mav_io = Arc::new(new_mav_io);
                        current_mav_io = Some(new_mav_io.clone());
                        *mav_io_shared.write().await = Some(new_mav_io.clone());

                        // Broadcast connected status
                        let _ = conn_status_tx_reconnect.send(ConnectionStatus {
                            connected: true,
                            reconnecting: false,
                            retry_count: 0,
                            serial_port: port_path.clone(),
                        });

                        // Spawn receiver task (Pixhawk -> simulation + QGC)
                        let mav_io_recv = new_mav_io.clone();
                        let shutdown_recv = shutdown_reconnect.clone();
                        let actuator_tx_recv = actuator_tx_reconnect.clone();
                        let qgc_socket_recv = qgc_socket_reconnect.clone();
                        receiver_handle = Some(tokio::spawn(async move {
                            info!("MAVLink receiver task started");
                            loop {
                                if shutdown_recv.load(Ordering::Relaxed) || mav_io_recv.is_disconnected() {
                                    break;
                                }

                                if let Some((header, msg)) = mav_io_recv.try_recv() {
                                    // Forward to QGC via UDP
                                    if let Some(ref socket) = qgc_socket_recv {
                                        let mut buf = Vec::new();
                                        if mavlink::write_v2_msg(&mut buf, header, &msg).is_ok() {
                                            let _ = socket.send_to(&buf, qgc_target);
                                        }
                                    }

                                    // Process HIL_ACTUATOR_CONTROLS
                                    if let MavMessage::HIL_ACTUATOR_CONTROLS(_) = &msg {
                                        if let Ok(actuator) = ActuatorOutputs::from_mavlink(&msg) {
                                            let _ = actuator_tx_recv.send(actuator);
                                        }
                                    }
                                } else {
                                    tokio::time::sleep(Duration::from_micros(500)).await;
                                }
                            }
                            info!("MAVLink receiver task exiting");
                        }));

                        // Spawn sender task (simulation -> Pixhawk)
                        let mav_io_send = new_mav_io.clone();
                        let shutdown_send = shutdown_reconnect.clone();
                        let sim_mav_rx_send = sim_mav_rx_reconnect.clone();
                        sender_handle = Some(tokio::spawn(async move {
                            info!("MAVLink sender task started");
                            loop {
                                if shutdown_send.load(Ordering::Relaxed) || mav_io_send.is_disconnected() {
                                    break;
                                }

                                match sim_mav_rx_send.recv_timeout(Duration::from_millis(10)) {
                                    Ok(msg) => {
                                        if mav_io_send.send(msg).is_err() {
                                            break;
                                        }
                                    }
                                    Err(crossbeam_channel::RecvTimeoutError::Timeout) => continue,
                                    Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
                                }
                            }
                            info!("MAVLink sender task exiting");
                        }));
                    }
                    Err(e) => {
                        error!(error = %e, port = %port_path, "Failed to connect to FC");

                        let delay = reconnect_delay(retry_count);
                        tokio::time::sleep(delay).await;
                        if retry_count < 255 { retry_count += 1; }
                    }
                }
            }

            info!("Connection manager exiting");
        }))
    } else {
        // In sim-only mode, broadcast that we're not connected and not reconnecting
        let _ = conn_status_tx.send(ConnectionStatus {
            connected: false,
            reconnecting: false,
            retry_count: 0,
            serial_port: "simulation".to_string(),
        });
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
        info!("Mode: HARDWARE-IN-THE-LOOP (scanning for FC...)");
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
    if let Some(handle) = reconnect_handle {
        handle.abort();
    }

    // Wait for threads
    for handle in thread_handles {
        let _ = handle.join();
    }

    info!("HITL Daemon shutdown complete");
}
