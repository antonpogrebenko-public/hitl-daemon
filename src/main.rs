//! HITL Daemon - Hardware-in-the-loop simulator for UAV testing
//!
//! This daemon connects to a PX4-compatible flight controller via serial and provides
//! simulated sensor data for hardware-in-the-loop testing.

use clap::Parser;
use crossbeam_channel::{bounded, Receiver, Sender};
use hitl_physics::{throttle_to_omega_with_config, PhysicsConfig};
use hitl_sensors::{ImuConfig, SensorsConfig};
use mavlink::ardupilotmega::MavMessage;
use mavlink_io::async_io::{reconnect_delay, MavlinkIo, NshRequest, TrySendError};
use mavlink_io::serial::detect_flight_controller;
use protocol::{ActuatorOutputs, DaemonState, DaemonStatus};
use simulation::{SharedOrigin, SimulationConfig, SimulationLoop, SimulationState, TerrainCache};
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use tokio::sync::watch;
use tracing::{debug, error, info, trace, warn};
use tracing_appender::non_blocking::WorkerGuard;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};
use websocket::{
    CommandType, ConnectionStatus, LinkState, StateUpdate, ValidatedNshCommand, VehicleMessage,
    WebSocketServer, WebSocketServerConfig,
};

mod terrain_pack;
mod tui;
mod update;

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

    /// Local terrain tile pack for ground collision, laid out `{z}/{x}/{y}.bin`.
    ///
    /// For headless and CI runs. With a browser attached the tiles arrive over
    /// the WebSocket instead — the daemon never fetches terrain itself, so that
    /// the physics cannot resolve a tile differently from the viewer.
    #[arg(long)]
    terrain_pack: Option<std::path::PathBuf>,

    /// UDP port for QGroundControl bridge (0 to disable)
    #[arg(long, default_value = "14550")]
    qgc_udp: u16,

    /// Update this daemon to the latest release, then exit.
    ///
    /// Running this is the confirmation: nothing replaces the binary on its
    /// own. The previous version is kept alongside as `.previous`.
    #[arg(long)]
    update: bool,
}

enum TracingMode {
    Tui {
        log_rx: std::sync::mpsc::Receiver<String>,
    },
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

/// MAVLink system id PX4 uses for the autopilot.
const PX4_SYSTEM_ID: u8 = 1;
const PX4_COMPONENT_ID: u8 = 1;

/// `MAV_CMD_REQUEST_MESSAGE` asking for `AUTOPILOT_VERSION` (message id 148).
///
/// PX4 does not broadcast this message; it answers on request. The reply
/// carries the board's hardware UID, which is what keys a parameter snapshot to
/// the board it was taken from.
fn make_autopilot_version_request() -> MavMessage {
    const AUTOPILOT_VERSION_MSG_ID: f32 = 148.0;
    MavMessage::COMMAND_LONG(mavlink::ardupilotmega::COMMAND_LONG_DATA {
        target_system: PX4_SYSTEM_ID,
        target_component: PX4_COMPONENT_ID,
        confirmation: 0,
        command: mavlink::ardupilotmega::MavCmd::MAV_CMD_REQUEST_MESSAGE,
        param1: AUTOPILOT_VERSION_MSG_ID,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    })
}

fn detect_or_use_port(specified_port: Option<String>) -> Option<String> {
    if let Some(port) = specified_port {
        info!(port = %port, "Using specified serial port");
        return Some(port);
    }

    info!("No port specified, auto-detecting FC...");
    // Vendor-ID allowlist first, then a read-only heartbeat probe over the
    // remaining USB serial ports. Boards outside the four known vendors are
    // otherwise invisible and force the user to pass --port by hand.
    let outcome = detect_flight_controller();

    if outcome.found.is_empty() {
        if !outcome.bootloader.is_empty() {
            // Not an error and not "no board" — the board is right there,
            // still starting. Opening it now would block forever, so the only
            // correct move is to wait for it to finish.
            info!(
                port = %outcome.bootloader.join(", "),
                "Board is in its bootloader — waiting for it to finish starting"
            );
        } else if outcome.examined.is_empty() {
            warn!("No serial ports to examine — is the board plugged in?");
        } else {
            // "No FC detected" is not actionable; the list of what was looked
            // at is.
            warn!(
                examined = outcome.examined.len(),
                "No flight controller detected. Examined: {}",
                outcome.examined.join(", ")
            );
        }
        return None;
    }

    if outcome.found.len() > 1 {
        info!("Multiple boards found:");
        for port in &outcome.found {
            info!("  - {}", port);
        }
        info!("Using first detected port");
    }

    let port = outcome.found.into_iter().next().unwrap();
    if outcome.adopted_by_probe {
        info!(
            port = %port,
            "Auto-detected FC by heartbeat probe (vendor ID not recognised)"
        );
    } else {
        info!(port = %port, "Auto-detected FC");
    }
    Some(port)
}

/// Spawn the simulation loop in a dedicated thread
fn spawn_simulation_thread(
    config: SimulationConfig,
    actuator_rx: Receiver<ActuatorOutputs>,
    mav_tx: Sender<MavMessage>,
    _shutdown: Arc<AtomicBool>,
    config_rx: Receiver<(
        PhysicsConfig,
        hitl_physics::BatteryConfig,
        hitl_sensors::SensorsConfig,
    )>,
    sim_stats_tx: tokio::sync::watch::Sender<protocol::SimulationStats>,
    origin: Arc<SharedOrigin>,
) -> (thread::JoinHandle<()>, SimulationState) {
    let mut sim_loop = SimulationLoop::new(config, actuator_rx, config_rx, mav_tx, origin)
        .with_stats_publisher(sim_stats_tx);
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

/// Simulation-only mode: generate fake actuator commands for testing.
///
/// Battery awareness: when the simulation battery is depleted, motors are zeroed
/// and mode switches to `Disarmed`. After `AUTO_RECHARGE_SECS` seconds the battery
/// is automatically recharged so the simulation can continue without a daemon restart.
fn spawn_sim_only_actuator_thread(
    actuator_tx: Sender<ActuatorOutputs>,
    shutdown: Arc<AtomicBool>,
    sim_state: simulation::SimulationState,
) -> thread::JoinHandle<()> {
    // Ticks at 400 Hz (2500 µs each). 3 seconds = 1200 ticks.
    const AUTO_RECHARGE_TICKS: u64 = 1200;

    thread::Builder::new()
        .name("sim-actuator".to_string())
        .spawn(move || {
            info!("Simulation-only actuator thread started (generating hover commands)");
            let mut tick = 0u64;
            let mut depleted_since: Option<u64> = None;

            while !shutdown.load(Ordering::Relaxed) {
                let is_depleted = sim_state.read().battery.is_depleted();

                if is_depleted {
                    // Track how long we have been depleted
                    let depleted_tick = *depleted_since.get_or_insert(tick);

                    // Auto-recharge after AUTO_RECHARGE_TICKS ticks (~3 s)
                    if tick - depleted_tick >= AUTO_RECHARGE_TICKS {
                        info!(
                            "sim-only: battery depleted — auto-recharging for continued simulation"
                        );
                        sim_state.recharge_battery();
                        depleted_since = None;
                    }

                    // Send zeroed, disarmed actuator while depleted
                    let actuator = ActuatorOutputs {
                        timestamp_us: tick * 2500,
                        motors: [0.0; 4],
                        mode: protocol::FlightMode::Disarmed,
                        controls: [0.0; 16],
                    };
                    if actuator_tx.send(actuator).is_err() {
                        break;
                    }
                } else {
                    // Battery healthy — clear any stale depletion marker
                    depleted_since = None;

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
    // Read the live config first (clone — no lock held across the inner read).
    // This ensures motor RPM computation uses the current build's physics
    // (max_motor_speed, etc.) rather than the stale config that was set at
    // construction time.
    let config = sim_state.config();
    let state = sim_state.read();
    let q = &state.quadrotor;

    StateUpdate {
        timestamp_us: state.sim_time_us,
        position_ned: [
            q.position[0] as f32,
            q.position[1] as f32,
            q.position[2] as f32,
        ],
        velocity_ned: [
            q.velocity[0] as f32,
            q.velocity[1] as f32,
            q.velocity[2] as f32,
        ],
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
        landed_state: state.landed_state,
    }
}

#[tokio::main]
async fn main() {
    let args = Args::parse();

    if args.update {
        let api_url =
            std::env::var("HITL_API_URL").unwrap_or_else(|_| "https://api.th3seus.net".to_string());
        match update::run_update(&api_url, VERSION).await {
            Ok(()) => {
                println!("Daemon is up to date or was updated successfully.");
                std::process::exit(0);
            }
            Err(e) => {
                eprintln!("Update failed: {e}");
                // Non-zero so a script wrapping this can tell.
                std::process::exit(1);
            }
        }
    }
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
            gyro_noise_density: 0.0008, // Default: realistic noise level
            accel_noise_density: 0.006, // Default: realistic noise level
            gyro_bias_sigma: 0.0,       // CRITICAL: No bias drift in HITL
            gyro_bias_tau: 1000.0,      // Long time constant (unused since sigma=0)
            accel_bias_sigma: 0.0,      // No accel bias drift for HITL — gives EKF clean data
            accel_bias_tau: 1000.0,     // Long time constant, effectively disabled
        },
        baro: hitl_sensors::BaroConfig::default(),
        gps: hitl_sensors::GpsConfig {
            position_drift_tau: 1000.0,  // Very slow drift
            position_drift_sigma: 0.0,   // No position drift for HITL
            horizontal_noise_sigma: 0.1, // 10cm noise - tight for HITL
            altitude_noise_sigma: 0.3,   // 30cm altitude noise
            velocity_noise_sigma: 0.05,  // 5cm/s velocity noise
            delay_ms: 80.0,              // Moderate delay
            update_rate_hz: 10.0,        // 10 Hz GPS
            ..hitl_sensors::GpsConfig::default()
        },
        mag: hitl_sensors::MagConfig::default(),
    };

    // Create shared terrain cache (can be populated via CLI flag or WebSocket)
    let terrain_cache = Arc::new(TerrainCache::new());
    let terrain_ref = (args.reference_lat, args.reference_lon, args.reference_alt);

    // Terrain for a run with no browser. The vertical datum is the elevation
    // at the origin, and ground collision, the barometer and HIL_GPS all adopt
    // it together — keeping the CLI --alt when the terrain says otherwise
    // leaves baro and GPS self-consistent but both offset from true MSL, so the
    // vehicle plots below real terrain in QGC.
    let mut reference_alt = args.reference_alt;
    terrain_cache.set_origin(terrain_ref.0, terrain_ref.1, None);
    if let Some(ref pack) = args.terrain_pack {
        match terrain_pack::load_pack(&terrain_cache, pack) {
            Ok(0) => warn!(
                pack = %pack.display(),
                "Terrain pack held no usable tiles — flying on flat ground"
            ),
            Ok(accepted) => {
                // Re-anchor with the elevation sampled from the pack itself, so
                // the datum matches the ground that is actually loaded.
                if let Some(msl) = terrain_cache
                    .sample_ground_ned(0.0, 0.0)
                    .map(|_| terrain_cache.origin_elevation_msl())
                    .flatten()
                {
                    reference_alt = msl;
                }
                info!(accepted, "Terrain pack ready for ground collision");
            }
            Err(e) => warn!(
                pack = %pack.display(),
                error = %e,
                "Could not read terrain pack — flying on flat ground"
            ),
        }
    } else {
        info!("No terrain pack given; terrain arrives from the browser over the WebSocket");
    }

    let sim_config = SimulationConfig {
        reference_lat: args.reference_lat,
        reference_lon: args.reference_lon,
        reference_alt,
        tick_rate_hz: args.tick_rate,
        gps_rate_hz: args.gps_rate,
        sensors: clean_sensors,
        terrain: Some(terrain_cache.clone()),
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
    // build_config_tx/rx: WebSocket -> Simulation (PhysicsConfig + BatteryConfig + SensorsConfig)
    let (actuator_tx, actuator_rx) = bounded::<ActuatorOutputs>(16);
    let (sim_mav_tx, sim_mav_rx) = bounded::<MavMessage>(512);
    let (build_config_tx, build_config_rx) = bounded::<(
        PhysicsConfig,
        hitl_physics::BatteryConfig,
        hitl_sensors::SensorsConfig,
    )>(1);

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

    // SimulationStats watch channel — populated by the sim loop at 2Hz with
    // a live snapshot of tick rate, position, motor RPMs, battery, etc.
    // Moves the per-window `info!` log out of the scrolling log area and
    // into the TUI header so the log stream stays readable.
    let (sim_stats_tx, sim_stats_rx) = watch::channel(protocol::SimulationStats::default());

    // Spawn TUI thread if in TUI mode
    let tui_handle = match tracing_mode {
        TracingMode::Tui { log_rx } => {
            let tui_shutdown = shutdown.clone();
            Some(
                thread::Builder::new()
                    .name("tui".to_string())
                    .spawn(move || {
                        tui::run_tui(status_rx, sim_stats_rx, log_rx, tui_shutdown);
                    })
                    .expect("Failed to spawn TUI thread"),
            )
        }
        TracingMode::Plain => None,
    };

    // Clone the MAVLink output for the WebSocket build-config handler so it
    // can push Phase 6 per-build PIDs via PARAM_SET. In --sim-only mode there
    // is no PX4 attached, so the param push is pointless and we pass None.
    let build_config_mav_tx = if sim_only_mode {
        None
    } else {
        Some(sim_mav_tx.clone())
    };

    // The shared origin the whole flight is anchored to. Ground contact, the
    // barometer and HIL_GPS all read its datum, so there is one altitude
    // reference rather than three that have to be kept in step. Seeded from the
    // CLI; `ConfigureBuild` replaces it with the browser's choice.
    let flight_origin = Arc::new(SharedOrigin::new(
        sim_config.reference_lat,
        sim_config.reference_lon,
        sim_config.reference_alt,
    ));

    // Spawn simulation thread
    let (sim_handle, sim_state) = spawn_simulation_thread(
        sim_config,
        actuator_rx,
        sim_mav_tx,
        shutdown.clone(),
        build_config_rx,
        sim_stats_tx,
        flight_origin.clone(),
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
                info!(
                    "QGC UDP bridge: local port {} ↔ QGC port {}",
                    qgc_local_port, args.qgc_udp
                );
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
    let mav_io: Arc<tokio::sync::RwLock<Option<Arc<MavlinkIo>>>> =
        Arc::new(tokio::sync::RwLock::new(None));

    // In sim-only mode, always generate fake actuator commands
    // In normal mode, fake actuators run until FC connects
    if sim_only_mode {
        let handle = spawn_sim_only_actuator_thread(
            actuator_tx.clone(),
            shutdown.clone(),
            sim_state.clone(),
        );
        thread_handles.push(handle);
    }

    // Broadcast channel for NSH responses (to WebSocket clients)
    let (nsh_resp_broadcast_tx, _) = tokio::sync::broadcast::channel::<websocket::NshResponse>(64);

    // Broadcast channel for connection status (to WebSocket clients)
    let (conn_status_tx, _) = tokio::sync::broadcast::channel::<ConnectionStatus>(16);

    // Broadcast channel for vehicle messages (STATUSTEXT from PX4)
    let (vehicle_msg_tx, _) = tokio::sync::broadcast::channel::<VehicleMessage>(64);

    // Broadcast channel for terrain origin (GPS_GLOBAL_ORIGIN / HOME_POSITION / GLOBAL_POSITION_INT)
    let (terrain_origin_tx, _) = tokio::sync::broadcast::channel::<websocket::TerrainOrigin>(4);

    // Broadcast channel for PARAM_VALUE acks from PX4. BuildConfigHandler
    // subscribes after pushing PARAM_SETs so it can verify each parameter
    // was actually applied before signalling the simulation "ready" stage.
    // Capacity is generous because PX4 may emit unrelated PARAM_VALUE traffic
    // during the wait window (e.g. QGC parameter pull).
    let (param_value_tx, _) =
        tokio::sync::broadcast::channel::<websocket::param_io::ParamValue>(256);

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
    let nsh_tx_for_config = if sim_only_mode {
        None
    } else {
        Some(nsh_cmd_tx.clone())
    };
    let build_config_param_value_tx = if sim_only_mode {
        None
    } else {
        Some(param_value_tx.clone())
    };
    // Stable per-board key for the parameter snapshot. Derived from
    // AUTOPILOT_VERSION rather than the serial port (which changes on replug)
    // or fc_model (which every PX4 quad shares).
    //
    // Declared here, before the handler, and shared with it: the receiver task
    // populates this same cell. Giving the handler its own would leave it
    // permanently empty, and provisioning would refuse every board for want of
    // an identity it was never told about.
    let board_identity: Arc<tokio::sync::RwLock<Option<websocket::BoardIdentity>>> =
        Arc::new(tokio::sync::RwLock::new(None));

    let preflight_handler = std::sync::Arc::new(websocket::PreflightHandler::with_identity(
        build_config_mav_tx.clone(),
        build_config_param_value_tx.clone(),
        nsh_tx_for_config.clone(),
        sim_state.clone(),
        board_identity.clone(),
    ));
    let build_config_handler = std::sync::Arc::new(websocket::BuildConfigHandler::new(
        build_config_tx,
        nsh_tx_for_config,
        build_config_mav_tx,
        build_config_param_value_tx,
        Some(terrain_cache.clone()),
        terrain_ref,
        Some(flight_origin.clone()),
    ));
    // Keep a clone for the reconnect task so it can re-push PIDs after FC power cycles.
    let build_config_handler_for_reconnect = build_config_handler.clone();
    ws_server.set_build_config_handler(build_config_handler);
    ws_server.set_preflight_handler(preflight_handler);
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

    // Enable terrain origin broadcasting
    ws_server.set_terrain_origin_receiver(terrain_origin_tx.subscribe());

    // The browser is the sole fetcher of elevation data: it pushes decoded
    // tiles in, and the server asks it for whatever the physics still lacks
    // around the vehicle.
    ws_server.set_terrain_cache(terrain_cache.clone());

    // Get browser shutdown signal before ws_server is moved
    let ws_shutdown = ws_server.shutdown_signal();

    // Spawn WebSocket server task
    let serial_port_label = initial_port
        .clone()
        .unwrap_or_else(|| "scanning".to_string());
    let ws_handle = tokio::spawn(async move {
        let version_parts: Vec<u8> = VERSION
            .split('.')
            .take(3)
            .filter_map(|s| s.parse().ok())
            .collect();
        let version_major = version_parts.first().copied().unwrap_or(0);
        let version_minor = version_parts.get(1).copied().unwrap_or(1);
        let version_patch = version_parts.get(2).copied().unwrap_or(0);

        if let Err(e) = ws_server
            .run(
                version_major,
                version_minor,
                version_patch,
                serial_port_label,
            )
            .await
        {
            // A daemon that cannot serve is useless, and a browser waiting on
            // it would spin forever. Fail loudly and exit non-zero rather than
            // appearing to run.
            error!("WebSocket server error: {}", e);
            std::process::exit(1);
        }
    });

    // Check for a newer daemon, detached and time-boxed. Deliberately spawned
    // rather than awaited: a release channel that is down, slow, or blocked by
    // a corporate proxy must never stop someone flying. The result is a
    // notification, not a precondition.
    {
        let api_url =
            std::env::var("HITL_API_URL").unwrap_or_else(|_| "https://api.th3seus.net".to_string());
        tokio::spawn(async move {
            match update::check_for_update(&api_url, VERSION).await {
                Ok(Some(available)) => {
                    info!(
                        current = VERSION,
                        available = %available.version,
                        "A newer daemon is available"
                    );
                }
                Ok(None) => debug!(version = VERSION, "Daemon is up to date"),
                Err(e) => {
                    // Not a startup failure. Logged at debug so an offline
                    // user is not greeted by a warning about something that
                    // does not affect them.
                    debug!(error = %e, "Skipped the update check");
                }
            }
        });
    }

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
    let fc_model: Arc<tokio::sync::RwLock<Option<String>>> =
        Arc::new(tokio::sync::RwLock::new(None));

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
                current_serial_port = if cs.serial_port.is_empty() {
                    None
                } else {
                    Some(cs.serial_port)
                };
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

            // Every 5 seconds (every 10 ticks at 500ms), check serial link quality
            if tick_count % 10 == 0 {
                if let Some(ref mav) = *mav_io_status.read().await {
                    let (successes, failures) = mav.take_parse_stats();
                    let total = successes + failures;
                    if total > 0 {
                        let quality = (successes * 100) / total;
                        if quality < 95 {
                            warn!(
                                quality_pct = quality,
                                failures, "Serial link quality degraded — check USB cable"
                            );
                        }
                    }
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
            // The prompt forms this looks for, as bytes. PROMPT_MAX is the
            // overlap the tail scan needs so a prompt split across two chunks is
            // still found.
            const PROMPT_WITH_CLEAR: &[u8] = b"nsh> \x1b[K";
            const PROMPT_BARE: &[u8] = b"nsh>";
            const PROMPT_MAX: usize = PROMPT_WITH_CLEAR.len();
            let mut request_deadline: Option<tokio::time::Instant> = None;
            let mut cached_mav_io: Option<Arc<MavlinkIo>> = None;

            loop {
                if shutdown_nsh.load(Ordering::Relaxed) {
                    break;
                }

                // Refresh cached mav_io reference
                {
                    let guard = mav_io_nsh.read().await;
                    if cached_mav_io
                        .as_ref()
                        .map(|m| m.is_disconnected())
                        .unwrap_or(true)
                    {
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
                                // Only the tail is rescanned.
                                //
                                // This converted the whole accumulated buffer to
                                // UTF-8 and searched all of it on every 70-byte
                                // chunk: for a K-chunk response that is 70*K^2/2
                                // bytes scanned — a 35 KB `param show` scans about
                                // 8.75 MB. And once the buffer holds invalid UTF-8,
                                // which a degraded cable produces and
                                // `take_parse_stats` exists to instrument,
                                // `from_utf8_lossy` stops borrowing and allocates a
                                // fresh String of the entire buffer per chunk.
                                //
                                // A prompt cannot straddle more than its own length,
                                // so an overlap of PROMPT_MAX is sufficient. Matched
                                // on bytes, so no conversion happens until the
                                // response is actually complete.
                                let scan_from = response_buffer.len().saturating_sub(PROMPT_MAX);
                                response_buffer.extend_from_slice(&resp.data);

                                let tail = &response_buffer[scan_from..];
                                let has_prompt = tail
                                    .windows(PROMPT_WITH_CLEAR.len())
                                    .any(|w| w == PROMPT_WITH_CLEAR)
                                    || {
                                        let trimmed = match tail.iter().rposition(|b| !b.is_ascii_whitespace()) {
                                            Some(end) => &tail[..=end],
                                            None => &tail[..0],
                                        };
                                        trimmed.ends_with(PROMPT_BARE)
                                    };
                                let is_complete = resp.complete || has_prompt;

                                if is_complete {
                                    if let Some(req_id) = current_request_id.take() {
                                        // Converted once, on completion, rather
                                        // than on every chunk.
                                        let output =
                                            String::from_utf8_lossy(&response_buffer).into_owned();
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
        let board_identity_reconnect = board_identity.clone();
        let sim_state_reconnect = sim_state.clone();
        let param_value_tx_reconnect = param_value_tx.clone();
        let preferred_port = args.port.clone();
        let baud = args.baud;
        let build_config_handler_reconnect = build_config_handler_for_reconnect;

        // Shared flag: set true when the heartbeat watchdog fires (bootloader suspected).
        // The connection loop reads this to apply a longer backoff so the port is
        // free for firmware-flashing tools during the 30-60 s bootloader window.
        let bootloader_suspected = Arc::new(AtomicBool::new(false));

        Some(tokio::spawn(async move {
            info!("Connection manager started");

            let mut retry_count: u8 = 0;
            // Distinguishes a first scan from a reconnect. Without it a
            // first-time user is told their board is "reconnecting" to
            // something it was never connected to.
            let mut has_connected = false;
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
                bootloader_suspected: false,
                link_state: LinkState::Searching,
            });

            loop {
                if shutdown_reconnect.load(Ordering::Relaxed) {
                    break;
                }

                // Check if we have an active connection
                let is_connected = current_mav_io
                    .as_ref()
                    .map(|m| !m.is_disconnected())
                    .unwrap_or(false);

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

                    if retry_count < 255 {
                        retry_count += 1;
                    }

                    let is_bootloader = bootloader_suspected.load(Ordering::SeqCst);
                    let _ = conn_status_tx_reconnect.send(ConnectionStatus {
                        connected: false,
                        reconnecting: true,
                        retry_count,
                        serial_port: String::new(),
                        fc_model: None,
                        bootloader_suspected: is_bootloader,
                        link_state: if is_bootloader {
                            LinkState::SuspectedBootloader
                        } else {
                            LinkState::Reconnecting
                        },
                    });

                    // Cooldown before reconnect — gives OS time to release port.
                    // If the heartbeat watchdog fired (bootloader suspected), use a 10s
                    // backoff so firmware-flashing tools have uncontested access to the
                    // serial port during the 30-60 s bootloader window.
                    if bootloader_suspected.load(Ordering::SeqCst) {
                        let bootloader_backoff = Duration::from_secs(10);
                        warn!(
                            "Bootloader suspected — waiting {}s before reconnect to free serial port",
                            bootloader_backoff.as_secs()
                        );
                        bootloader_suspected.store(false, Ordering::SeqCst);
                        tokio::time::sleep(bootloader_backoff).await;
                    } else {
                        let delay = reconnect_delay(retry_count);
                        tokio::time::sleep(delay).await;
                    }
                }

                // Try to find a flight controller. Probing runs here too, so a
                // board that comes back on a different port — or was never
                // recognised by vendor ID — is still picked up on reconnect.
                // Tracks whether this scan saw a board mid-boot, so the user is
                // told "starting up" rather than "not found" — a board that is
                // visibly present but reported missing reads as a broken cable.
                let mut board_in_bootloader = false;
                let port_path = if let Some(ref p) = preferred_port {
                    Some(p.clone())
                } else {
                    let outcome = detect_flight_controller();
                    board_in_bootloader = !outcome.bootloader.is_empty();
                    if board_in_bootloader {
                        debug!(
                            port = %outcome.bootloader.join(", "),
                            "Board is in its bootloader — not opening it"
                        );
                    } else if outcome.found.is_empty() && !outcome.examined.is_empty() {
                        debug!(
                            examined = %outcome.examined.join(", "),
                            "No flight controller among the examined ports"
                        );
                    }
                    outcome.found.into_iter().next()
                };

                let Some(port_path) = port_path else {
                    // No FC found, wait and retry
                    let delay = reconnect_delay(retry_count);
                    debug!(
                        retry_count,
                        delay_ms = delay.as_millis(),
                        "No FC found, waiting..."
                    );

                    let _ = conn_status_tx_reconnect.send(ConnectionStatus {
                        connected: false,
                        reconnecting: true,
                        retry_count,
                        serial_port: String::new(),
                        fc_model: None,
                        bootloader_suspected: board_in_bootloader,
                        link_state: if board_in_bootloader {
                            LinkState::SuspectedBootloader
                        } else if has_connected {
                            LinkState::Reconnecting
                        } else {
                            LinkState::Searching
                        },
                    });

                    tokio::time::sleep(delay).await;
                    if retry_count < 255 {
                        retry_count += 1;
                    }
                    continue;
                };

                info!(port = %port_path, "Found FC, connecting...");

                // Create MAVLink I/O
                let (mut new_mav_io, tx_to_app, rx_from_app, nsh_resp_tx, nsh_req_rx) =
                    MavlinkIo::new();

                match new_mav_io
                    .spawn(
                        &port_path,
                        baud,
                        tx_to_app,
                        rx_from_app,
                        nsh_resp_tx.clone(),
                        nsh_req_rx,
                    )
                    .await
                {
                    Ok(()) => {
                        info!(port = %port_path, "Connected to FC!");
                        let was_reconnect = retry_count > 0;
                        has_connected = true;
                        retry_count = 0;
                        // Successful connection — clear any stale bootloader flag.
                        bootloader_suspected.store(false, Ordering::SeqCst);

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
                            bootloader_suspected: false,
                            link_state: LinkState::Connected,
                        });

                        // After a reconnect (not the initial connection), re-push
                        // PIDs to PX4 — a power cycle resets PX4's RAM parameters.
                        // We wait 3 s for PX4 to boot and EKF2 to start accepting
                        // PARAM_SET before attempting the push.
                        if was_reconnect {
                            // Drop the fingerprint cache *now*, not in 3 s: PX4's
                            // RAM params are already gone, and a browser-initiated
                            // ConfigureBuild (the preflight overlay sends one the
                            // instant its reboot reports Done) can easily land
                            // inside that delay and be skipped as a no-op,
                            // leaving the FC on airframe defaults.
                            build_config_handler_reconnect.invalidate_pid_fingerprint();

                            let handler_clone = build_config_handler_reconnect.clone();
                            tokio::spawn(async move {
                                tokio::time::sleep(Duration::from_secs(3)).await;
                                if let Err(e) = handler_clone.repush_if_configured().await {
                                    warn!(error = %e, "PID re-push after FC reconnect failed — user may need to reconfigure manually");
                                }
                            });
                        }

                        // Spawn receiver task (Pixhawk -> simulation + QGC)
                        let mav_io_recv = new_mav_io.clone();
                        let shutdown_recv = shutdown_reconnect.clone();
                        let actuator_tx_recv = actuator_tx_reconnect.clone();
                        let qgc_socket_recv = qgc_socket_reconnect.clone();
                        let vehicle_msg_tx_recv = vehicle_msg_tx.clone();
                        let terrain_origin_tx_recv = terrain_origin_tx.clone();
                        // The live datum, not a startup snapshot: the browser
                        // must anchor its world to the same altitude reference
                        // the physics is using right now, and ConfigureBuild
                        // can have replaced it since the daemon started.
                        let flight_origin_recv = flight_origin.clone();
                        let fc_model_recv = fc_model_reconnect.clone();
                        let board_identity_recv = board_identity_reconnect.clone();
                        let conn_status_tx_recv = conn_status_tx_reconnect.clone();
                        let param_value_tx_recv = param_value_tx_reconnect.clone();
                        let port_path_recv = port_path.clone();
                        let sim_state_recv = sim_state_reconnect.clone();
                        let bootloader_suspected_recv = bootloader_suspected.clone();
                        let start_time = std::time::Instant::now();
                        receiver_handle = Some(tokio::spawn(async move {
                            // Dropped actuator frames, reported on a divider so a
                            // stalled simulation says so once a second rather than
                            // 400 times.
                            let mut actuator_drops: u64 = 0;
                            // Reusable serialisation buffer for QGC forwarding.
                            let mut qgc_buf: Vec<u8> = Vec::with_capacity(300);
                            info!("MAVLink receiver task started");
                            let heartbeat_timeout = Duration::from_secs(5);
                            let mut heartbeat_received = false;
                            let mut best_origin_source: Option<protocol::OriginSource> = None;
                            let mut last_origin_lat: f64 = 0.0;
                            let mut last_origin_lon: f64 = 0.0;
                            loop {
                                if shutdown_recv.load(Ordering::Relaxed)
                                    || mav_io_recv.is_disconnected()
                                {
                                    break;
                                }

                                // Watchdog: if no heartbeat within timeout, FC is likely in bootloader.
                                // Set the flag so the connection loop uses a long backoff (10s) to keep
                                // the serial port free for firmware-flashing tools. Also broadcast the
                                // status so the frontend can show "FC is booting, please wait...".
                                if !heartbeat_received && start_time.elapsed() > heartbeat_timeout {
                                    warn!("No heartbeat received within {}s — FC may be in bootloader mode", heartbeat_timeout.as_secs());
                                    bootloader_suspected_recv.store(true, Ordering::SeqCst);
                                    let _ = conn_status_tx_recv.send(ConnectionStatus {
                                        connected: false,
                                        reconnecting: true,
                                        retry_count: 0,
                                        serial_port: String::new(),
                                        fc_model: None,
                                        bootloader_suspected: true,
                                        link_state: LinkState::SuspectedBootloader,
                                    });
                                    mav_io_recv.signal_disconnect();
                                    break;
                                }

                                if let Some((header, msg)) = mav_io_recv.try_recv() {
                                    // Forward to QGC via UDP
                                    if let Some(ref socket) = qgc_socket_recv {
                                        // Reused, not allocated per message. QGC
                                        // forwarding is on by default, so this ran
                                        // for every inbound frame — about 410 a
                                        // second — whether or not QGroundControl
                                        // was running.
                                        qgc_buf.clear();
                                        if mavlink::write_v2_msg(&mut qgc_buf, header, &msg).is_ok()
                                        {
                                            let _ = socket.send_to(&qgc_buf, qgc_target);
                                        }
                                    }

                                    // Process HIL_ACTUATOR_CONTROLS
                                    if let MavMessage::HIL_ACTUATOR_CONTROLS(_) = &msg {
                                        if let Ok(actuator) = ActuatorOutputs::from_mavlink(&msg) {
                                            // try_send, not send: this runs on a
                                            // tokio worker, and the blocking form
                                            // parks it until the simulation
                                            // consumes — which, if the sim thread
                                            // has aborted, is never. A full
                                            // 16-deep queue already means the sim
                                            // is not keeping up, so the newest
                                            // command is the one worth keeping.
                                            if actuator_tx_recv.try_send(actuator).is_err() {
                                                actuator_drops += 1;
                                                if actuator_drops % 400 == 1 {
                                                    warn!(
                                                        drops = actuator_drops,
                                                        "actuator queue full — simulation is not consuming"
                                                    );
                                                }
                                            }
                                        }
                                    }

                                    // Process PARAM_VALUE (PX4 ack for PARAM_SET). Forward to
                                    // BuildConfigHandler so it can verify each PID parameter
                                    // was applied before transitioning the config to Ready.
                                    if let MavMessage::PARAM_VALUE(pv) = &msg {
                                        // Decoded into a typed value here rather than
                                        // downstream: PX4's declared param_type is only
                                        // present on this message, and snapshot/restore
                                        // cannot replay a value without it.
                                        if let Some(parsed) =
                                            websocket::param_io::ParamValue::from_mavlink(pv)
                                        {
                                            let _ = param_value_tx_recv.send(parsed);
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
                                            let timestamp_ms =
                                                start_time.elapsed().as_millis() as u32;
                                            debug!(severity = severity, text = %text, "STATUSTEXT received");
                                            let _ = vehicle_msg_tx_recv.send(VehicleMessage {
                                                severity,
                                                timestamp_ms,
                                                text,
                                            });
                                        }
                                    }

                                    // Extract flight mode and FC model from HEARTBEAT
                                    // PX4's land detector verdict. The simulation
                                    // deliberately does not infer this — surfacing
                                    // the FC's own answer is what lets the UI (and
                                    // a human) spot a disagreement between the sim's
                                    // ground contact and what the EKF believes.
                                    if let MavMessage::EXTENDED_SYS_STATE(ess) = &msg {
                                        sim_state_recv.set_landed_state(ess.landed_state as u8);
                                    }

                                    // Board identity for the parameter snapshot. PX4
                                    // sends this only on request, so it arrives once
                                    // per connection after the request below.
                                    if let MavMessage::AUTOPILOT_VERSION(av) = &msg {
                                        let derived =
                                            websocket::board_identity::derive(av, PX4_SYSTEM_ID);
                                        match &derived {
                                            Some(id) => info!(
                                                board_identity = %id,
                                                "Flight controller identity established"
                                            ),
                                            None => warn!(
                                                "Flight controller reports no distinguishing \
                                                 identity — parameter snapshots are unavailable \
                                                 for this board"
                                            ),
                                        }
                                        *board_identity_recv.write().await = derived;
                                    }

                                    if let MavMessage::HEARTBEAT(hb) = &msg {
                                        let first_heartbeat = !heartbeat_received;
                                        heartbeat_received = true;

                                        // Ask for AUTOPILOT_VERSION once the link is
                                        // proven. Requesting before the first heartbeat
                                        // races PX4's own startup and the reply is lost.
                                        if first_heartbeat {
                                            // try_send: this is the async
                                            // receiver task, and the blocking
                                            // form would park a runtime worker
                                            // behind the serial writer. A
                                            // dropped version request costs a
                                            // diagnostic field, not the link.
                                            let _ = mav_io_recv
                                                .try_send(make_autopilot_version_request());
                                        }

                                        // Update flight mode from custom_mode
                                        // PX4 custom_mode is a 32-bit field where main mode is in bits 16-23
                                        let main_mode = ((hb.custom_mode >> 16) & 0xFF) as u8;
                                        sim_state_recv.set_flight_mode(main_mode);

                                        // Preflight gate: cache whether PX4 reports HITL mode and
                                        // quadrotor type so "Launch Simulation" can read this
                                        // synchronously instead of a fresh MAVLink round-trip.
                                        let (hitl_enabled, is_quadrotor) =
                                            websocket::heartbeat_hitl_signals(hb);
                                        sim_state_recv
                                            .set_heartbeat_status(hitl_enabled, is_quadrotor);

                                        use mavlink::ardupilotmega::{MavAutopilot, MavType};
                                        let mut model = fc_model_recv.write().await;
                                        if model.is_none() {
                                            if hb.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID {
                                                let name = match hb.autopilot {
                                                    MavAutopilot::MAV_AUTOPILOT_PX4 => {
                                                        match hb.mavtype {
                                                            MavType::MAV_TYPE_QUADROTOR => {
                                                                "PX4 Quadrotor"
                                                            }
                                                            MavType::MAV_TYPE_HEXAROTOR => {
                                                                "PX4 Hexarotor"
                                                            }
                                                            MavType::MAV_TYPE_OCTOROTOR => {
                                                                "PX4 Octorotor"
                                                            }
                                                            MavType::MAV_TYPE_FIXED_WING => {
                                                                "PX4 Fixed Wing"
                                                            }
                                                            _ => "PX4 Vehicle",
                                                        }
                                                    }
                                                    MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA => {
                                                        "ArduPilot"
                                                    }
                                                    _ => "Unknown FC",
                                                };
                                                info!(
                                                    fc_model = name,
                                                    "Flight controller identified"
                                                );
                                                *model = Some(name.to_string());
                                                let _ =
                                                    conn_status_tx_recv.send(ConnectionStatus {
                                                        connected: true,
                                                        reconnecting: false,
                                                        retry_count: 0,
                                                        serial_port: port_path_recv.clone(),
                                                        fc_model: Some(name.to_string()),
                                                        bootloader_suspected: false,
                                                        link_state: LinkState::Connected,
                                                    });
                                            }
                                        }
                                    }

                                    // Extract terrain origin from GPS messages.
                                    // Priority: GpsGlobalOrigin > HomePosition > GlobalPositionInt.
                                    // Only upgrade source or re-emit if position moved >1m.
                                    let candidate: Option<(f64, f64, f32, protocol::OriginSource)> =
                                        match &msg {
                                            MavMessage::GPS_GLOBAL_ORIGIN(d) => Some((
                                                d.latitude as f64 / 1e7,
                                                d.longitude as f64 / 1e7,
                                                d.altitude as f32 / 1000.0,
                                                protocol::OriginSource::GpsGlobalOrigin,
                                            )),
                                            MavMessage::HOME_POSITION(d) => Some((
                                                d.latitude as f64 / 1e7,
                                                d.longitude as f64 / 1e7,
                                                d.altitude as f32 / 1000.0,
                                                protocol::OriginSource::HomePosition,
                                            )),
                                            MavMessage::GLOBAL_POSITION_INT(d) => {
                                                if best_origin_source.is_none() {
                                                    Some((
                                                        d.lat as f64 / 1e7,
                                                        d.lon as f64 / 1e7,
                                                        d.alt as f32 / 1000.0,
                                                        protocol::OriginSource::GlobalPositionInt,
                                                    ))
                                                } else {
                                                    None
                                                }
                                            }
                                            _ => None,
                                        };

                                    // The message's own altitude is deliberately
                                    // discarded: the viewer needs the same
                                    // vertical datum the physics uses, which is
                                    // the DEM elevation at the origin (adopted
                                    // into reference_alt at terrain load), not
                                    // whatever MSL the FC happens to report.
                                    if let Some((lat, lon, _alt, source)) = candidate {
                                        let dominated = best_origin_source
                                            .map(|best| source < best)
                                            .unwrap_or(false);
                                        if !dominated {
                                            let dlat = (lat - last_origin_lat) * 111_320.0;
                                            let dlon = (lon - last_origin_lon)
                                                * 111_320.0
                                                * (lat.to_radians().cos());
                                            let moved = (dlat * dlat + dlon * dlon).sqrt() > 1.0;
                                            let upgraded = best_origin_source
                                                .map(|best| source > best)
                                                .unwrap_or(true);

                                            if upgraded || moved {
                                                best_origin_source = Some(source);
                                                last_origin_lat = lat;
                                                last_origin_lon = lon;
                                                let _ = terrain_origin_tx_recv.send(
                                                    websocket::TerrainOrigin {
                                                        ref_lat: lat,
                                                        ref_lon: lon,
                                                        ref_alt: flight_origin_recv.get().alt_datum
                                                            as f32,
                                                        source: source as u8,
                                                    },
                                                );
                                                info!(
                                                    lat, lon,
                                                    alt = flight_origin_recv.get().alt_datum,
                                                    source = ?source,
                                                    "Terrain origin updated"
                                                );
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
                            // Outbound frames dropped because the serial writer
                            // was behind, reported on a divider so a stalled
                            // port says so about once a second rather than 400
                            // times.
                            let mut outbound_drops: u64 = 0;

                            // Bridge crossbeam → tokio mpsc so we can use select!
                            let (sim_tx, mut sim_rx) =
                                tokio::sync::mpsc::channel::<MavMessage>(128);
                            // A shutdown flag of this bridge's own, not the
                            // process-wide one.
                            //
                            // Teardown used `bridge_handle.abort()`, and `abort`
                            // cannot cancel a `spawn_blocking` task that has
                            // already started — and on the reconnect path that
                            // line is never reached at all, because the parent
                            // sender task is aborted first and this is the last
                            // statement of its body. The bridge only stopped
                            // because the parent's abort dropped `sim_rx` and
                            // the next `blocking_send` failed, which meant it
                            // first stole a message from the shared crossbeam
                            // receiver that the replacement bridge should have
                            // had. Correct by accident; now correct on purpose.
                            let bridge_shutdown = Arc::new(AtomicBool::new(false));
                            let bridge_stop = Arc::clone(&bridge_shutdown);
                            let bridge_rx = sim_mav_rx_send.clone();
                            let bridge_handle = tokio::task::spawn_blocking(move || {
                                while !bridge_shutdown.load(Ordering::Relaxed) {
                                    match bridge_rx.recv_timeout(Duration::from_millis(5)) {
                                        Ok(msg) => {
                                            if sim_tx.blocking_send(msg).is_err() {
                                                break;
                                            }
                                        }
                                        Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                                            continue
                                        }
                                        Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                                            break
                                        }
                                    }
                                }
                            });

                            // Convert QGC std::net::UdpSocket to tokio async socket
                            let qgc_async_socket = qgc_socket_send.as_ref().and_then(|s| {
                                let std_socket = s.try_clone().ok()?;
                                std_socket.set_nonblocking(true).ok()?;
                                tokio::net::UdpSocket::from_std(std_socket).ok()
                            });

                            let mut qgc_recv_buf = [0u8; 280];

                            loop {
                                if shutdown_send.load(Ordering::Relaxed)
                                    || mav_io_send.is_disconnected()
                                {
                                    break;
                                }

                                let first_msg = if let Some(ref qgc_sock) = qgc_async_socket {
                                    tokio::select! {
                                        biased;
                                        sim_msg = sim_rx.recv() => {
                                            match sim_msg {
                                                Some(m) => m,
                                                None => break,
                                            }
                                        }
                                        qgc_result = qgc_sock.recv_from(&mut qgc_recv_buf) => {
                                            if let Ok((len, _addr)) = qgc_result {
                                                use mavlink::peek_reader::PeekReader;
                                                let cursor = std::io::Cursor::new(&qgc_recv_buf[..len]);
                                                let mut reader = PeekReader::new(cursor);
                                                if let Ok((_header, qgc_msg)) = mavlink::read_v2_msg::<MavMessage, _>(&mut reader) {
                                                    trace!("QGC -> Pixhawk: {:?}", qgc_msg);
                                                    qgc_msg
                                                } else {
                                                    continue;
                                                }
                                            } else {
                                                break;
                                            }
                                        }
                                    }
                                } else {
                                    match sim_rx.recv().await {
                                        Some(m) => m,
                                        None => break,
                                    }
                                };

                                // try_send throughout: this task is on the tokio
                                // runtime, and the blocking form waits for the
                                // serial writer, whose own write timeout is two
                                // seconds. A stalled port would take a runtime
                                // worker with it for that long, which is how
                                // WebSocket handshakes were starved before.
                                match mav_io_send.try_send(first_msg) {
                                    Ok(()) => {}
                                    Err(TrySendError::Disconnected) => break,
                                    Err(TrySendError::Full) => {
                                        outbound_drops += 1;
                                        if outbound_drops % 400 == 1 {
                                            warn!(
                                                drops = outbound_drops,
                                                "serial writer is behind — dropping outbound MAVLink"
                                            );
                                        }
                                    }
                                }

                                // Drain all remaining buffered sim messages without blocking
                                while let Ok(msg) = sim_rx.try_recv() {
                                    match mav_io_send.try_send(msg) {
                                        Ok(()) => {}
                                        Err(TrySendError::Disconnected) => break,
                                        Err(TrySendError::Full) => {
                                            outbound_drops += 1;
                                        }
                                    }
                                }
                            }

                            // Ask it to stop, then wait for it to notice. The
                            // loop polls its flag every 5 ms.
                            bridge_stop.store(true, Ordering::Relaxed);
                            let _ = bridge_handle.await;
                            info!("MAVLink sender task exiting");
                        }));
                    }
                    Err(e) => {
                        error!(error = %e, port = %port_path, "Failed to connect to FC");

                        let delay = reconnect_delay(retry_count);
                        tokio::time::sleep(delay).await;
                        if retry_count < 255 {
                            retry_count += 1;
                        }
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
            bootloader_suspected: false,
            link_state: LinkState::Searching,
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
