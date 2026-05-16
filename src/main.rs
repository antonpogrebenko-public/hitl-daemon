//! HITL Daemon - Hardware-in-the-loop simulator for UAV testing
//!
//! This daemon connects to a PX4-compatible flight controller via serial and provides
//! simulated sensor data for hardware-in-the-loop testing.

use clap::Parser;
use crossbeam_channel::{bounded, Receiver, Sender};
use mavlink::ardupilotmega::MavMessage;
use mavlink_io::async_io::{MavlinkIo, NshRequest, reconnect_delay};
use mavlink_io::serial::find_pixhawk_ports;
use protocol::{ActuatorOutputs, DaemonState, DaemonStatus};
use hitl_physics::{throttle_to_omega_with_config, PhysicsConfig};
use hitl_sensors::{ImuConfig, SensorsConfig};
use simulation::{SimulationConfig, SimulationLoop, SimulationState};
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use tokio::sync::watch;
use tracing::{debug, error, info, trace, warn};
use tracing_appender::non_blocking::WorkerGuard;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};
use websocket::{CommandType, ConnectionStatus, StateUpdate, ValidatedNshCommand, VehicleMessage, WebSocketServer, WebSocketServerConfig};

mod tui;

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

    /// Write logs to this file in addition to stdout (e.g. /tmp/hitl.log)
    #[arg(long)]
    log_file: Option<String>,

    /// UDP port for QGroundControl bridge (0 to disable)
    #[arg(long, default_value = "14550")]
    qgc_udp: u16,
}

enum TracingMode {
    Tui { log_rx: std::sync::mpsc::Receiver<String> },
    Plain,
}

fn init_tracing(log_file: Option<&str>, tui_mode: bool) -> (Option<WorkerGuard>, TracingMode) {
    let env_filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));

    if tui_mode {
        let (log_tx, log_rx) = std::sync::mpsc::sync_channel::<String>(512);
        let tui_layer = tui::TuiLayer::new(log_tx);

        if let Some(path) = log_file {
            let file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(path)
                .expect("Failed to open log file");
            let (file_writer, guard) = tracing_appender::non_blocking(file);
            let file_layer = tracing_subscriber::fmt::layer()
                .with_ansi(false)
                .with_writer(file_writer);
            tracing_subscriber::registry()
                .with(env_filter)
                .with(tui_layer)
                .with(file_layer)
                .init();
            (Some(guard), TracingMode::Tui { log_rx })
        } else {
            tracing_subscriber::registry()
                .with(env_filter)
                .with(tui_layer)
                .init();
            (None, TracingMode::Tui { log_rx })
        }
    } else {
        let stdout_layer = tracing_subscriber::fmt::layer();
        if let Some(path) = log_file {
            let file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(path)
                .expect("Failed to open log file");
            let (file_writer, guard) = tracing_appender::non_blocking(file);
            let file_layer = tracing_subscriber::fmt::layer()
                .with_ansi(false)
                .with_writer(file_writer);
            tracing_subscriber::registry()
                .with(env_filter)
                .with(stdout_layer)
                .with(file_layer)
                .init();
            (Some(guard), TracingMode::Plain)
        } else {
            tracing_subscriber::registry()
                .with(env_filter)
                .with(stdout_layer)
                .init();
            (None, TracingMode::Plain)
        }
    }
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
    config_rx: Receiver<(PhysicsConfig, hitl_physics::BatteryConfig)>,
) -> (thread::JoinHandle<()>, SimulationState) {
    let mut sim_loop = SimulationLoop::new(config, actuator_rx, config_rx, mav_tx);
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
fn create_state_update(sim_state: &SimulationState, packets_per_sec: u16) -> StateUpdate {
    let state = sim_state.read();
    let q = &state.quadrotor;
    let config = sim_state.config();

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
                let omega = throttle_to_omega_with_config(c as f64, &config.physics);
                (omega * 60.0 / (2.0 * std::f64::consts::PI)) as f32
            })
        } else {
            [0.0; 4]
        },
        battery_voltage: state.battery.voltage() as f32,
        battery_percent: state.battery.percent(),
        armed: state.armed,
        flight_mode: state.flight_mode,
        packets_per_sec,
    }
}

#[tokio::main]
async fn main() {
    let args = Args::parse();
    let is_tty = atty::is(atty::Stream::Stdout);
    let (_log_guard, tracing_mode) = init_tracing(args.log_file.as_deref(), is_tty);

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
    // build_config_tx/rx: WebSocket -> Simulation (PhysicsConfig + BatteryConfig updates)
    let (actuator_tx, actuator_rx) = bounded::<ActuatorOutputs>(16);
    let (sim_mav_tx, sim_mav_rx) = bounded::<MavMessage>(64);
    let (build_config_tx, build_config_rx) = bounded::<(PhysicsConfig, hitl_physics::BatteryConfig)>(1);

    // Shutdown signal
    let shutdown = Arc::new(AtomicBool::new(false));

    // Set up Ctrl+C handler
    let shutdown_ctrlc = shutdown.clone();
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.ok();
        info!("Received Ctrl+C, initiating shutdown...");
        shutdown_ctrlc.store(true, Ordering::SeqCst);
    });

    // DaemonStatus watch channel (TUI reads this at 2Hz)
    let (status_tx, status_rx) = watch::channel(DaemonStatus::default());

    // Spawn TUI thread if in TUI mode
    let tui_handle = match tracing_mode {
        TracingMode::Tui { log_rx } => {
            let tui_shutdown = shutdown.clone();
            Some(
                thread::Builder::new()
                    .name("tui".to_string())
                    .spawn(move || {
                        tui::run_tui(status_rx, log_rx, tui_shutdown);
                    })
                    .expect("Failed to spawn TUI thread"),
            )
        }
        TracingMode::Plain => None,
    };

    // Clone the MAVLink output for the WebSocket build-config handler so it
    // can push Phase 6 per-build PIDs via PARAM_SET. In --sim-only mode there
    // is no PX4 attached, so the param push is pointless and we pass None.
    let build_config_mav_tx = if sim_only_mode { None } else { Some(sim_mav_tx.clone()) };

    // Spawn simulation thread
    let (sim_handle, sim_state) = spawn_simulation_thread(
        sim_config,
        actuator_rx,
        sim_mav_tx,
        shutdown.clone(),
        build_config_rx,
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
    let (nsh_cmd_tx, mut nsh_cmd_rx) = tokio::sync::mpsc::channel::<ValidatedNshCommand>(4);

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

    // Broadcast channel for vehicle messages (STATUSTEXT from PX4)
    let (vehicle_msg_tx, _) = tokio::sync::broadcast::channel::<VehicleMessage>(64);

    // Broadcast channel for PARAM_VALUE acks from PX4. BuildConfigHandler
    // subscribes after pushing PARAM_SETs so it can verify each parameter
    // was actually applied before signalling the simulation "ready" stage.
    // Capacity is generous because PX4 may emit unrelated PARAM_VALUE traffic
    // during the wait window (e.g. QGC parameter pull).
    let (param_value_tx, _) =
        tokio::sync::broadcast::channel::<(String, f32)>(256);

    // Create WebSocket server
    let ws_config = WebSocketServerConfig {
        port: args.websocket_port,
        update_rate_hz: 30,
        allowed_origins: vec![],
    };
    let mut ws_server = WebSocketServer::new(ws_config);
    let state_tx = ws_server.state_sender();
    let mut command_rx = ws_server.take_command_receiver();

    // Set up build config handler to send PhysicsConfig updates to simulation
    // Pass NSH sender so it can restart EKF2 after config changes (clone before moving to ws_server)
    let nsh_tx_for_config = if sim_only_mode { None } else { Some(nsh_cmd_tx.clone()) };
    let build_config_param_value_tx = if sim_only_mode { None } else { Some(param_value_tx.clone()) };
    let build_config_handler = std::sync::Arc::new(websocket::BuildConfigHandler::new(
        build_config_tx,
        nsh_tx_for_config,
        build_config_mav_tx,
        build_config_param_value_tx,
    ));
    ws_server.set_build_config_handler(build_config_handler);
    let sim_state_for_recharge = sim_state.clone();
    ws_server.set_recharge_callback(std::sync::Arc::new(move || {
        sim_state_for_recharge.recharge_battery();
    }));

    // Always enable NSH support (will be available when Pixhawk connects)
    ws_server.set_nsh_sender(nsh_cmd_tx);
    ws_server.set_nsh_response_receiver(nsh_resp_broadcast_tx.subscribe());

    // Always enable connection status broadcasting
    ws_server.set_connection_status_receiver(conn_status_tx.subscribe());

    // Always enable vehicle message broadcasting
    ws_server.set_vehicle_message_receiver(vehicle_msg_tx.subscribe());

    // Get browser shutdown signal before ws_server is moved
    let ws_shutdown = ws_server.shutdown_signal();

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

    // Merge browser shutdown with main shutdown
    let shutdown_merge = shutdown.clone();
    tokio::spawn(async move {
        loop {
            if ws_shutdown.load(Ordering::Relaxed) {
                info!("Shutdown triggered from browser");
                shutdown_merge.store(true, Ordering::SeqCst);
                break;
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });

    // Shared FC model (set once when HEARTBEAT identifies the FC)
    let fc_model: Arc<tokio::sync::RwLock<Option<String>>> = Arc::new(tokio::sync::RwLock::new(None));

    // Shared packets_per_sec (updated every second by status task)
    let packets_per_sec_shared = Arc::new(std::sync::atomic::AtomicU32::new(0));

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
    let pps_for_state = packets_per_sec_shared.clone();
    let ws_state_handle = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(33)); // ~30 Hz

        while !shutdown_ws_state.load(Ordering::Relaxed) {
            interval.tick().await;

            let pps = pps_for_state.load(Ordering::Relaxed) as u16;
            let state_update = create_state_update(&sim_state_broadcast, pps);
            let _ = state_tx.send(state_update);
        }
    });

    // Status updater task (updates DaemonStatus for TUI at 2Hz, calculates packets/sec every second)
    let shutdown_status = shutdown.clone();
    let mav_io_status = mav_io.clone();
    let fc_model_status = fc_model.clone();
    let pps_for_status = packets_per_sec_shared.clone();
    let start_time_status = std::time::Instant::now();
    let mut conn_status_rx = conn_status_tx.subscribe();
    let status_handle = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(500));
        let mut tick_count: u8 = 0;
        let mut current_serial_port: Option<String> = None;
        let mut is_reconnecting = false;

        loop {
            if shutdown_status.load(Ordering::Relaxed) {
                break;
            }
            interval.tick().await;

            // Drain connection status updates
            while let Ok(cs) = conn_status_rx.try_recv() {
                current_serial_port = if cs.serial_port.is_empty() { None } else { Some(cs.serial_port) };
                is_reconnecting = cs.reconnecting;
            }

            // Every second (every 2 ticks at 500ms), compute packets/sec
            tick_count = tick_count.wrapping_add(1);
            if tick_count % 2 == 0 {
                if let Some(ref mav) = *mav_io_status.read().await {
                    let count = mav.take_packet_count();
                    pps_for_status.store(count, Ordering::Relaxed);
                } else {
                    pps_for_status.store(0, Ordering::Relaxed);
                }
            }

            // Derive daemon state
            let mav_connected = mav_io_status.read().await.is_some();
            let current_pps = pps_for_status.load(Ordering::Relaxed);
            let state = if shutdown_status.load(Ordering::Relaxed) {
                DaemonState::ShuttingDown
            } else if sim_only_mode {
                DaemonState::Streaming
            } else if mav_connected && current_pps > 0 {
                DaemonState::Streaming
            } else if mav_connected {
                DaemonState::Connected
            } else if is_reconnecting {
                DaemonState::Reconnecting
            } else {
                DaemonState::WaitingForFc
            };

            let model = fc_model_status.read().await.clone();

            let _ = status_tx.send(DaemonStatus {
                state,
                fc_model: model,
                serial_port: current_serial_port.clone(),
                packets_per_sec: current_pps.min(u16::MAX as u32) as u16,
                connected_clients: 0,
                uptime_secs: start_time_status.elapsed().as_secs(),
            });
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
        let fc_model_reconnect = fc_model.clone();
        let sim_state_reconnect = sim_state.clone();
        let param_value_tx_reconnect = param_value_tx.clone();
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
                fc_model: None,
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
                    *fc_model_reconnect.write().await = None;

                    // Abort old tasks and wait for them to release the port
                    if let Some(h) = receiver_handle.take() {
                        h.abort();
                        let _ = tokio::time::timeout(Duration::from_secs(1), h).await;
                    }
                    if let Some(h) = sender_handle.take() {
                        h.abort();
                        let _ = tokio::time::timeout(Duration::from_secs(1), h).await;
                    }

                    if retry_count < 255 { retry_count += 1; }

                    let _ = conn_status_tx_reconnect.send(ConnectionStatus {
                        connected: false,
                        reconnecting: true,
                        retry_count,
                        serial_port: String::new(),
                        fc_model: None,
                    });

                    // Cooldown before reconnect — gives OS time to release port
                    let delay = reconnect_delay(retry_count);
                    tokio::time::sleep(delay).await;
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
                        fc_model: None,
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
                            fc_model: None,
                        });

                        // Spawn receiver task (Pixhawk -> simulation + QGC)
                        let mav_io_recv = new_mav_io.clone();
                        let shutdown_recv = shutdown_reconnect.clone();
                        let actuator_tx_recv = actuator_tx_reconnect.clone();
                        let qgc_socket_recv = qgc_socket_reconnect.clone();
                        let vehicle_msg_tx_recv = vehicle_msg_tx.clone();
                        let fc_model_recv = fc_model_reconnect.clone();
                        let conn_status_tx_recv = conn_status_tx_reconnect.clone();
                        let param_value_tx_recv = param_value_tx_reconnect.clone();
                        let port_path_recv = port_path.clone();
                        let sim_state_recv = sim_state_reconnect.clone();
                        let start_time = std::time::Instant::now();
                        receiver_handle = Some(tokio::spawn(async move {
                            info!("MAVLink receiver task started");
                            let heartbeat_timeout = Duration::from_secs(5);
                            let mut heartbeat_received = false;
                            loop {
                                if shutdown_recv.load(Ordering::Relaxed) || mav_io_recv.is_disconnected() {
                                    break;
                                }

                                // Watchdog: if no heartbeat within timeout, FC is likely in bootloader
                                if !heartbeat_received && start_time.elapsed() > heartbeat_timeout {
                                    warn!("No heartbeat received within {}s — FC may be in bootloader mode", heartbeat_timeout.as_secs());
                                    mav_io_recv.signal_disconnect();
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

                                    // Process PARAM_VALUE (PX4 ack for PARAM_SET). Forward to
                                    // BuildConfigHandler so it can verify each PID parameter
                                    // was applied before transitioning the config to Ready.
                                    if let MavMessage::PARAM_VALUE(pv) = &msg {
                                        let name = std::str::from_utf8(&pv.param_id)
                                            .unwrap_or("")
                                            .trim_end_matches('\0')
                                            .to_string();
                                        if !name.is_empty() {
                                            let _ = param_value_tx_recv.send((name, pv.param_value));
                                        }
                                    }

                                    // Process STATUSTEXT messages for vehicle messages overlay
                                    if let MavMessage::STATUSTEXT(status) = &msg {
                                        let text = std::str::from_utf8(&status.text)
                                            .unwrap_or("")
                                            .trim_end_matches('\0')
                                            .to_string();
                                        if !text.is_empty() {
                                            let severity = status.severity as u8;
                                            let timestamp_ms = start_time.elapsed().as_millis() as u32;
                                            debug!(severity = severity, text = %text, "STATUSTEXT received");
                                            let _ = vehicle_msg_tx_recv.send(VehicleMessage {
                                                severity,
                                                timestamp_ms,
                                                text,
                                            });
                                        }
                                    }

                                    // Extract flight mode and FC model from HEARTBEAT
                                    if let MavMessage::HEARTBEAT(hb) = &msg {
                                        heartbeat_received = true;

                                        // Update flight mode from custom_mode
                                        // PX4 custom_mode is a 32-bit field where main mode is in bits 16-23
                                        let main_mode = ((hb.custom_mode >> 16) & 0xFF) as u8;
                                        sim_state_recv.set_flight_mode(main_mode);

                                        use mavlink::ardupilotmega::{MavAutopilot, MavType};
                                        let mut model = fc_model_recv.write().await;
                                        if model.is_none() {
                                            if hb.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID {
                                                let name = match hb.autopilot {
                                                    MavAutopilot::MAV_AUTOPILOT_PX4 => match hb.mavtype {
                                                        MavType::MAV_TYPE_QUADROTOR => "PX4 Quadrotor",
                                                        MavType::MAV_TYPE_HEXAROTOR => "PX4 Hexarotor",
                                                        MavType::MAV_TYPE_OCTOROTOR => "PX4 Octorotor",
                                                        MavType::MAV_TYPE_FIXED_WING => "PX4 Fixed Wing",
                                                        _ => "PX4 Vehicle",
                                                    },
                                                    MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA => "ArduPilot",
                                                    _ => "Unknown FC",
                                                };
                                                info!(fc_model = name, "Flight controller identified");
                                                *model = Some(name.to_string());
                                                let _ = conn_status_tx_recv.send(ConnectionStatus {
                                                    connected: true,
                                                    reconnecting: false,
                                                    retry_count: 0,
                                                    serial_port: port_path_recv.clone(),
                                                    fc_model: Some(name.to_string()),
                                                });
                                            }
                                        }
                                    }
                                } else {
                                    tokio::time::sleep(Duration::from_millis(2)).await;
                                }
                            }
                            info!("MAVLink receiver task exiting");
                        }));

                        // Spawn sender task (simulation -> Pixhawk + QGC -> Pixhawk)
                        let mav_io_send = new_mav_io.clone();
                        let shutdown_send = shutdown_reconnect.clone();
                        let sim_mav_rx_send = sim_mav_rx_reconnect.clone();
                        let qgc_socket_send = qgc_socket_reconnect.clone();
                        sender_handle = Some(tokio::spawn(async move {
                            info!("MAVLink sender task started");
                            let mut qgc_recv_buf = [0u8; 280]; // MAVLink v2 max frame size
                            loop {
                                if shutdown_send.load(Ordering::Relaxed) || mav_io_send.is_disconnected() {
                                    break;
                                }

                                // Poll for messages from QGC (parameters, commands)
                                if let Some(ref socket) = qgc_socket_send {
                                    while let Ok((len, _addr)) = socket.recv_from(&mut qgc_recv_buf) {
                                        use mavlink::peek_reader::PeekReader;
                                        let cursor = std::io::Cursor::new(&qgc_recv_buf[..len]);
                                        let mut reader = PeekReader::new(cursor);
                                        if let Ok((_header, msg)) = mavlink::read_v2_msg::<MavMessage, _>(&mut reader) {
                                            trace!("QGC -> Pixhawk: {:?}", msg);
                                            if mav_io_send.send(msg).is_err() {
                                                break;
                                            }
                                        }
                                    }
                                }

                                // Forward simulation sensor messages to Pixhawk
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
            fc_model: None,
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
    status_handle.abort();
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

    // Wait for TUI thread
    if let Some(handle) = tui_handle {
        let _ = handle.join();
    }

    info!("HITL Daemon shutdown complete");
}
