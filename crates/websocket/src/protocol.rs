//! Binary message format for WebSocket communication
//!
//! # Outgoing Messages (daemon -> browser)
//!
//! ## 0x01: State Update (sent at 30 Hz)
//! - `[0]`: 0x01 message type
//! - `[1-8]`: timestamp_us (u64 LE)
//! - `[9-20]`: position NED (3x f32 LE)
//! - `[21-32]`: velocity NED (3x f32 LE)
//! - `[33-48]`: quaternion wxyz (4x f32 LE)
//! - `[49-60]`: angular_velocity (3x f32 LE)
//! - `[61-76]`: motor_rpms (4x f32 LE)
//! - `[77-80]`: battery_voltage (f32 LE)
//! - `[81]`: battery_percent (u8)
//! - `[82]`: armed (u8 bool)
//! - `[83]`: flight_mode (u8)
//! - `[84-85]`: packets_per_sec (u16 LE)
//! - `[86]`: landed_state (u8, MAV_LANDED_STATE: 0=undefined, 1=on ground,
//!   2=in air, 3=takeoff, 4=landing)
//! - Total: 87 bytes
//!
//! Readers must accept frames of at least `STATE_UPDATE_MIN_SIZE` (86) bytes so
//! a browser talking to an older daemon still parses; the missing trailing byte
//! decodes as landed_state = 0 (undefined).
//!
//! ## 0x02: Handshake ACK
//! - `[0]`: 0x02 message type
//! - `[1]`: version major (u8)
//! - `[2]`: version minor (u8)
//! - `[3]`: pixhawk_connected (u8 bool)
//! - `[4-N]`: serial_port string (null-terminated)
//!
//! ## 0x03: Command ACK
//! - `[0]`: 0x03 message type
//! - `[1-4]`: command_id (u32 LE)
//! - `[5]`: success (u8 bool)
//! - `[6-N]`: error string (null-terminated, only if !success)
//!
//! # Incoming Messages (browser -> daemon)
//!
//! ## 0x10: Command
//! - `[0]`: 0x10 message type
//! - `[1-4]`: command_id (u32 LE)
//! - `[5]`: command_type (0=Arm, 1=Disarm, 2=Takeoff, 3=Land, 4=RTL, 5=SetMode, 6=EmergencyStop)
//! - `[6+]`: command-specific payload
//!
//! ## 0x11: Handshake
//! - `[0]`: 0x11 message type (no payload)

use serde::{Deserialize, Serialize};
use thiserror::Error;

// Message type constants
pub const MSG_TYPE_STATE_UPDATE: u8 = 0x01;
pub const MSG_TYPE_HANDSHAKE_ACK: u8 = 0x02;
pub const MSG_TYPE_COMMAND_ACK: u8 = 0x03;
pub const MSG_TYPE_NSH_RESPONSE: u8 = 0x04;
pub const MSG_TYPE_CONNECTION_STATUS: u8 = 0x05;
pub const MSG_TYPE_VEHICLE_MESSAGE: u8 = 0x06;
pub const MSG_TYPE_SHUTDOWN: u8 = 0x07;
pub const MSG_TYPE_CONFIG_RESULT: u8 = 0x08;
pub const MSG_TYPE_TERRAIN_ORIGIN: u8 = 0x09;
pub const MSG_TYPE_COMMAND: u8 = 0x10;
pub const MSG_TYPE_HANDSHAKE: u8 = 0x11;
pub const MSG_TYPE_NSH_COMMAND: u8 = 0x12;
pub const MSG_TYPE_CONFIGURE_BUILD: u8 = 0x13;
pub const MSG_TYPE_PREFLIGHT_STATUS: u8 = 0x0A;
pub const MSG_TYPE_PREFLIGHT_APPLY_RESULT: u8 = 0x0B;
pub const MSG_TYPE_REQUEST_PREFLIGHT_CHECK: u8 = 0x14;
pub const MSG_TYPE_APPLY_PREFLIGHT_PARAMS: u8 = 0x15;
/// Daemon -> browser: parameters captured off the board, awaiting persistence.
pub const MSG_TYPE_SNAPSHOT_CAPTURED: u8 = 0x0C;
/// Browser -> daemon: the snapshot is durably stored, writes may proceed.
pub const MSG_TYPE_SNAPSHOT_STORED: u8 = 0x16;
/// Browser -> daemon: write this snapshot back to the board.
pub const MSG_TYPE_RESTORE_SNAPSHOT: u8 = 0x17;
/// Daemon -> browser: progress and outcome of a restore.
pub const MSG_TYPE_RESTORE_RESULT: u8 = 0x0D;
/// Daemon -> browser: what this daemon can do, sent unprompted on connect.
pub const MSG_TYPE_CAPABILITIES: u8 = 0x0E;
/// Daemon -> browser: tile coordinates the physics needs and does not hold.
pub const MSG_TYPE_TERRAIN_NEED: u8 = 0x0F;
/// Browser -> daemon: decoded elevation tiles for the physics to collide against.
pub const MSG_TYPE_TERRAIN_TILES: u8 = 0x18;

// State update size (current wire format)
pub const STATE_UPDATE_SIZE: usize = 87;

/// Smallest state frame a reader must still accept. Frames from daemons older
/// than the landed_state field are 86 bytes; the absent byte decodes as 0
/// (MAV_LANDED_STATE_UNDEFINED).
pub const STATE_UPDATE_MIN_SIZE: usize = 86;

#[derive(Debug, Error)]
pub enum ProtocolError {
    #[error("Message too short: expected at least {expected} bytes, got {actual}")]
    MessageTooShort { expected: usize, actual: usize },

    #[error("Unknown message type: 0x{0:02X}")]
    UnknownMessageType(u8),

    #[error("Invalid command type: {0}")]
    InvalidCommandType(u8),

    #[error("Invalid payload for command type {command_type}: {reason}")]
    InvalidPayload {
        command_type: CommandType,
        reason: String,
    },
}

/// State update sent to browser at 30 Hz
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct StateUpdate {
    pub timestamp_us: u64,
    pub position_ned: [f32; 3],
    pub velocity_ned: [f32; 3],
    pub quaternion_wxyz: [f32; 4],
    pub angular_velocity: [f32; 3],
    pub motor_rpms: [f32; 4],
    pub battery_voltage: f32,
    pub battery_percent: u8,
    pub armed: bool,
    pub flight_mode: u8,
    pub packets_per_sec: u16,
    /// Flight controller's land detector verdict (MAV_LANDED_STATE).
    pub landed_state: u8,
}

impl StateUpdate {
    /// Serialize to binary format (87 bytes)
    pub fn to_bytes(&self) -> [u8; STATE_UPDATE_SIZE] {
        let mut buf = [0u8; STATE_UPDATE_SIZE];

        buf[0] = MSG_TYPE_STATE_UPDATE;
        buf[1..9].copy_from_slice(&self.timestamp_us.to_le_bytes());

        // Position NED
        for (i, &v) in self.position_ned.iter().enumerate() {
            let offset = 9 + i * 4;
            buf[offset..offset + 4].copy_from_slice(&v.to_le_bytes());
        }

        // Velocity NED
        for (i, &v) in self.velocity_ned.iter().enumerate() {
            let offset = 21 + i * 4;
            buf[offset..offset + 4].copy_from_slice(&v.to_le_bytes());
        }

        // Quaternion WXYZ
        for (i, &v) in self.quaternion_wxyz.iter().enumerate() {
            let offset = 33 + i * 4;
            buf[offset..offset + 4].copy_from_slice(&v.to_le_bytes());
        }

        // Angular velocity
        for (i, &v) in self.angular_velocity.iter().enumerate() {
            let offset = 49 + i * 4;
            buf[offset..offset + 4].copy_from_slice(&v.to_le_bytes());
        }

        // Motor RPMs
        for (i, &v) in self.motor_rpms.iter().enumerate() {
            let offset = 61 + i * 4;
            buf[offset..offset + 4].copy_from_slice(&v.to_le_bytes());
        }

        // Battery
        buf[77..81].copy_from_slice(&self.battery_voltage.to_le_bytes());
        buf[81] = self.battery_percent;
        buf[82] = self.armed as u8;
        buf[83] = self.flight_mode;
        buf[84..86].copy_from_slice(&self.packets_per_sec.to_le_bytes());
        buf[86] = self.landed_state;

        buf
    }

    /// Deserialize from binary format
    pub fn from_bytes(data: &[u8]) -> Result<Self, ProtocolError> {
        if data.len() < STATE_UPDATE_MIN_SIZE {
            return Err(ProtocolError::MessageTooShort {
                expected: STATE_UPDATE_MIN_SIZE,
                actual: data.len(),
            });
        }

        if data[0] != MSG_TYPE_STATE_UPDATE {
            return Err(ProtocolError::UnknownMessageType(data[0]));
        }

        let timestamp_us = u64::from_le_bytes(data[1..9].try_into().unwrap());

        let mut position_ned = [0.0f32; 3];
        for (i, v) in position_ned.iter_mut().enumerate() {
            let offset = 9 + i * 4;
            *v = f32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
        }

        let mut velocity_ned = [0.0f32; 3];
        for (i, v) in velocity_ned.iter_mut().enumerate() {
            let offset = 21 + i * 4;
            *v = f32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
        }

        let mut quaternion_wxyz = [0.0f32; 4];
        for (i, v) in quaternion_wxyz.iter_mut().enumerate() {
            let offset = 33 + i * 4;
            *v = f32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
        }

        let mut angular_velocity = [0.0f32; 3];
        for (i, v) in angular_velocity.iter_mut().enumerate() {
            let offset = 49 + i * 4;
            *v = f32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
        }

        let mut motor_rpms = [0.0f32; 4];
        for (i, v) in motor_rpms.iter_mut().enumerate() {
            let offset = 61 + i * 4;
            *v = f32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
        }

        let battery_voltage = f32::from_le_bytes(data[77..81].try_into().unwrap());
        let battery_percent = data[81];
        let armed = data[82] != 0;
        let flight_mode = data[83];
        let packets_per_sec = u16::from_le_bytes(data[84..86].try_into().unwrap());
        // Absent on pre-landed_state daemons; 0 is MAV_LANDED_STATE_UNDEFINED,
        // which is exactly the right meaning for "this daemon cannot tell us".
        let landed_state = data.get(86).copied().unwrap_or(0);

        Ok(Self {
            timestamp_us,
            position_ned,
            velocity_ned,
            quaternion_wxyz,
            angular_velocity,
            motor_rpms,
            battery_voltage,
            battery_percent,
            armed,
            flight_mode,
            packets_per_sec,
            landed_state,
        })
    }
}

/// Handshake acknowledgment sent to browser
///
/// ## Binary format (0x02)
/// - `[0]`: 0x02 message type
/// - `[1]`: version_major (u8)
/// - `[2]`: version_minor (u8)
/// - `[3]`: version_patch (u8)
/// - `[4]`: pixhawk_connected (u8 bool)
/// - `[5-N]`: serial_port string (null-terminated)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HandshakeAck {
    pub version_major: u8,
    pub version_minor: u8,
    pub version_patch: u8,
    pub pixhawk_connected: bool,
    pub serial_port: String,
}

/// Connection status update sent to browser when FC connection changes
///
/// ## Binary format (0x05)
/// - `[0]`: 0x05 message type
/// - `[1]`: connected (u8 bool)
/// - `[2]`: reconnecting (u8 bool)
/// - `[3]`: retry_count (u8)
/// - `[4-N]`: serial_port string (null-terminated, empty if not connected)
/// - `[N+1-M]`: fc_model string (null-terminated, empty if unknown)
/// - `[M+1]`: bootloader_suspected (u8 bool) — appended after fc_model for backwards compat
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectionStatus {
    /// Whether Pixhawk is currently connected
    pub connected: bool,
    /// Whether daemon is actively trying to reconnect
    pub reconnecting: bool,
    /// Number of reconnection attempts so far
    pub retry_count: u8,
    /// Serial port path (empty if not connected)
    pub serial_port: String,
    /// FC model string from HEARTBEAT autopilot version (None if unknown)
    pub fc_model: Option<String>,
    /// True when the heartbeat watchdog timed out — FC is likely in bootloader mode
    pub bootloader_suspected: bool,
    /// Explicit link state.
    ///
    /// The booleans above cannot distinguish a first scan from a reconnect:
    /// both report `connected: false, reconnecting: true`, so the interface
    /// tells a first-time user their board is "reconnecting" to something it
    /// was never connected to.
    pub link_state: LinkState,
}

/// What the daemon is doing about its flight-controller link.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LinkState {
    /// Looking for a board that has not been seen yet this session.
    Searching,
    /// A board is attached and heartbeating.
    Connected,
    /// A previously-connected board went away and is being waited for.
    Reconnecting,
    /// The device is present but silent — PX4 sits in its bootloader for
    /// 3-5s on power-up, and far longer while firmware is being flashed.
    SuspectedBootloader,
}

impl ConnectionStatus {
    /// Serialize to binary format
    pub fn to_bytes(&self) -> Vec<u8> {
        let fc_model_str = self.fc_model.as_deref().unwrap_or("");
        let mut buf =
            Vec::with_capacity(5 + self.serial_port.len() + 1 + fc_model_str.len() + 1 + 1);
        buf.push(MSG_TYPE_CONNECTION_STATUS);
        buf.push(self.connected as u8);
        buf.push(self.reconnecting as u8);
        buf.push(self.retry_count);
        buf.extend_from_slice(self.serial_port.as_bytes());
        buf.push(0); // null terminator for serial_port
        buf.extend_from_slice(fc_model_str.as_bytes());
        buf.push(0); // null terminator for fc_model
        buf.push(self.bootloader_suspected as u8); // appended last for backwards compat
        buf
    }
}

/// Vehicle message from PX4 (STATUSTEXT)
///
/// ## Binary format (0x06)
/// - `[0]`: 0x06 message type
/// - `[1]`: severity (u8, MAVLink MAV_SEVERITY: 0=EMERGENCY, 7=DEBUG)
/// - `[2-5]`: timestamp_ms (u32 LE, daemon timestamp when received)
/// - `[6-N]`: text string (null-terminated UTF-8)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleMessage {
    /// MAVLink MAV_SEVERITY level (0=EMERGENCY, 1=ALERT, 2=CRITICAL, 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG)
    pub severity: u8,
    /// Daemon timestamp when message was received (milliseconds since daemon start)
    pub timestamp_ms: u32,
    /// Message text from PX4
    pub text: String,
}

impl VehicleMessage {
    /// Serialize to binary format
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(7 + self.text.len());
        buf.push(MSG_TYPE_VEHICLE_MESSAGE);
        buf.push(self.severity);
        buf.extend_from_slice(&self.timestamp_ms.to_le_bytes());
        buf.extend_from_slice(self.text.as_bytes());
        buf.push(0); // null terminator
        buf
    }
}

/// Geographic origin of a simulated flight.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub struct FlightLocation {
    pub lat: f64,
    pub lon: f64,
}

impl FlightLocation {
    /// Applied when a browser sends no location. Matches the daemon's own CLI
    /// defaults so the two paths agree on where "nowhere specified" is.
    pub const DEFAULT: Self = Self {
        lat: 40.015,
        lon: -105.2705,
    };

    /// Reject anything that is not a real point on Earth.
    ///
    /// Non-finite values are checked explicitly: NaN fails every comparison, so
    /// a range test alone would let it through and it would then poison every
    /// tile coordinate and sensor reading derived from the origin.
    pub fn validate(&self) -> Result<(), String> {
        if !self.lat.is_finite() {
            return Err("flight_location.lat must be a finite number".to_string());
        }
        if !self.lon.is_finite() {
            return Err("flight_location.lon must be a finite number".to_string());
        }
        if !(-90.0..=90.0).contains(&self.lat) {
            return Err(format!(
                "flight_location.lat {} out of range (-90..=90)",
                self.lat
            ));
        }
        if !(-180.0..=180.0).contains(&self.lon) {
            return Err(format!(
                "flight_location.lon {} out of range (-180..=180)",
                self.lon
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct ConfigureBuild {
    pub motor_slug: String,
    pub prop_slug: Option<String>,
    pub prop_diameter_inches: f64,
    pub frame_weight_g: f64,
    #[serde(default = "default_battery_voltage")]
    pub battery_voltage: f64,
    #[serde(default = "default_battery_capacity_mah")]
    pub battery_capacity_mah: f64,
    #[serde(default = "default_battery_cell_count")]
    pub battery_cell_count: u8,
    #[serde(default)]
    pub esc_slug: Option<String>,
    #[serde(default)]
    pub fc_slug: Option<String>,
    #[serde(default)]
    pub frame_slug: Option<String>,
    #[serde(default)]
    pub battery_slug: Option<String>,
    #[serde(default)]
    pub gps_slug: Option<String>,
    /// 1 = 4-in-1 ESC (weighs once), 4 = individual ESCs (weight × 4).
    #[serde(default = "default_esc_count")]
    pub esc_count: u8,
    /// Where in the world this flight takes place.
    ///
    /// Absent means a browser that predates location selection; the daemon
    /// applies [`FlightLocation::DEFAULT`] rather than failing, so a stale tab
    /// degrades instead of breaking.
    #[serde(default)]
    pub flight_location: Option<FlightLocation>,

    /// Terrain MSL elevation the browser sampled at `flight_location`, in metres.
    ///
    /// This is the vertical datum for the whole flight: ground contact, the
    /// barometer and the altitude reported to the flight controller all adopt
    /// it together. `None` means no elevation data covered the origin, in which
    /// case all three fall back to the configured altitude together.
    ///
    /// Carried in on configuration rather than derived when tiles arrive, so
    /// there is never a window where the datum has changed for some consumers
    /// and not others.
    #[serde(default)]
    pub origin_elevation_msl: Option<f64>,

    // Sensor noise parameters (from API sensor profiles, optional with defaults)
    /// Gyro noise density in rad/s/sqrt(Hz)
    #[serde(default)]
    pub gyro_noise_density: Option<f64>,
    /// Accel noise density in m/s^2/sqrt(Hz)
    #[serde(default)]
    pub accel_noise_density: Option<f64>,
    /// Baro noise sigma in meters
    #[serde(default)]
    pub baro_noise_sigma: Option<f64>,
    /// Mag noise sigma in gauss
    #[serde(default)]
    pub mag_noise_sigma: Option<f64>,
    /// GPS horizontal noise sigma in meters
    #[serde(default)]
    pub gps_horizontal_noise: Option<f64>,
    /// GPS altitude noise sigma in meters
    #[serde(default)]
    pub gps_altitude_noise: Option<f64>,
    /// GPS velocity noise sigma in m/s
    #[serde(default)]
    pub gps_velocity_noise: Option<f64>,
    /// How the sensor profile was matched: "exact", "mcu_family", "average",
    /// or absent when no profile was found (built-in defaults are used).
    /// Informational only — surfaced in the daemon log.
    #[serde(default)]
    pub sensor_match_type: Option<String>,
}

impl ConfigureBuild {
    /// The location this flight runs at.
    ///
    /// A present-but-invalid location is an error and fails configuration — the
    /// caller asked for a specific place and we must not silently fly somewhere
    /// else. An *absent* location is not an error: it means a browser that
    /// predates location selection, and gets the documented default.
    pub fn resolve_flight_location(&self) -> Result<FlightLocation, String> {
        match self.flight_location {
            Some(loc) => {
                loc.validate()?;
                Ok(loc)
            }
            None => Ok(FlightLocation::DEFAULT),
        }
    }
}

fn default_esc_count() -> u8 {
    1
}

fn default_battery_voltage() -> f64 {
    14.8
}
fn default_battery_capacity_mah() -> f64 {
    1500.0
}
fn default_battery_cell_count() -> u8 {
    4
}

impl ConfigureBuild {
    pub fn from_bytes(data: &[u8]) -> Result<Self, ProtocolError> {
        if data.len() < 2 {
            return Err(ProtocolError::MessageTooShort {
                expected: 2,
                actual: data.len(),
            });
        }

        if data[0] != MSG_TYPE_CONFIGURE_BUILD {
            return Err(ProtocolError::UnknownMessageType(data[0]));
        }

        let json_str =
            std::str::from_utf8(&data[1..]).map_err(|_| ProtocolError::InvalidPayload {
                command_type: CommandType::Arm,
                reason: "ConfigureBuild: invalid UTF-8".to_string(),
            })?;

        serde_json::from_str(json_str).map_err(|e| ProtocolError::InvalidPayload {
            command_type: CommandType::Arm,
            reason: format!("ConfigureBuild: {e}"),
        })
    }
}

/// Lifecycle stage of a `ConfigureBuild` request. Two-stage so the UI can show
/// "Configuring…" while we await `PARAM_VALUE` acks from PX4, then unlock the
/// "Continue to simulator" button only on `Ready`.
#[derive(Debug, Clone, Copy, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ConfigState {
    /// Physics + PIDs computed, but `PARAM_SET` acks still pending. Simulation
    /// has NOT been reconfigured yet — frontend must keep the user on the
    /// configure screen.
    Configuring,
    /// All `PARAM_SET` messages acked by PX4 (or skipped in --sim-only). New
    /// physics delivered to the sim loop and EKF2 restarted. Safe to fly.
    Ready,
    /// Something failed (PX4 unreachable, ack timeout, sim channel down).
    /// Sim still runs the previous config; frontend should show the error.
    Error,
}

/// Which part of applying a build is currently running.
///
/// `ConfigState` says only "still working" or "done", which left the interface
/// showing one unchanging line for the whole apply — several seconds of PX4
/// parameter acks and an EKF2 restart, with nothing to distinguish a slow step
/// from a stuck one.
///
/// Every variant corresponds to work the daemon actually performs, and is
/// emitted when that work begins. An earlier interface animated this sequence
/// on fixed timers, which meant the reported stage and the real one could
/// disagree — and added five seconds of delay purely to display it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ConfigStage {
    /// Fetching component specifications from the API.
    FetchingSpecs,
    /// Computing physics and per-build PID gains.
    Computing,
    /// Writing PID parameters to PX4 and awaiting a `PARAM_VALUE` ack for each.
    PushingParams,
    /// Restarting EKF2 so the estimator drops state from the previous build.
    RestartingEkf,
}

#[derive(Debug, Clone, Serialize)]
pub struct ConfigResult {
    pub state: ConfigState,
    pub success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<AppliedConfig>,
    /// The stage in progress. Absent on terminal results, and on any daemon
    /// older than this field — the interface must treat absence as "no detail
    /// available" rather than as an error.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stage: Option<ConfigStage>,
}

impl ConfigResult {
    pub fn to_bytes(&self) -> Vec<u8> {
        let json = serde_json::to_vec(self).expect("ConfigResult serialization cannot fail");
        let mut buf = Vec::with_capacity(1 + json.len());
        buf.push(MSG_TYPE_CONFIG_RESULT);
        buf.extend_from_slice(&json);
        buf
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct AppliedConfig {
    pub mass_kg: f64,
    pub kt: f64,
    pub kq: f64,
    pub arm_length_m: f64,
    pub max_thrust_per_motor_g: f64,
    pub thrust_to_weight_ratio: f64,
    pub motor_kv: f64,
    pub battery_voltage: f64,
    pub max_motor_rpm: f64,
    pub estimated_flight_time_min: f64,
    /// `MPC_THR_HOVER` pushed to PX4 — the `thr_desired` value (0-1, in
    /// PX4's pre-THR_MDL_FAC-inversion units) that produces hover thrust.
    /// Computed as `1/TWR` clamped to PX4's [0.1, 0.8] range. With the
    /// daemon's `THR_MDL_FAC=1` push, PX4 will output `sqrt(thr_desired)`
    /// to the actuator — matching the simulator's linear cmd→ω model and
    /// real ESC behavior.
    pub hover_cmd: f32,
    /// Whether this build can hover at all — `thrust_to_weight_ratio > 1.0`.
    ///
    /// When false, `hover_cmd` is a clamped stand-in that was NOT pushed to
    /// PX4, and `hover_required` states what the build would actually need.
    /// Surfaces to the user so an unflyable build is reported at configure
    /// time rather than discovered on the pad.
    pub can_hover: bool,
    /// Throttle fraction this build needs to hover, unclamped. Equals
    /// `1/sqrt(thrust_to_weight_ratio)`, so it exceeds 1.0 exactly when the
    /// build cannot lift its own weight.
    pub hover_required: f64,
    /// Components whose mass was estimated rather than read from the database,
    /// so the user is never shown a guess presented as a specification. A
    /// `"battery (extrapolated)"` entry additionally means the estimator was
    /// applied outside the capacity range it was fitted over.
    pub estimated_masses: Vec<String>,
    /// Per-build PX4 rate-controller gains pushed via `PARAM_SET` (Phase 6).
    /// Absent when the daemon ran in `--sim-only` mode or the fingerprint
    /// matched the previously-applied build (so we skipped the push).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub applied_pids: Option<Px4PidsView>,
    /// Count of `PARAM_SET` messages confirmed via matching `PARAM_VALUE` ack
    /// from PX4. Zero in --sim-only or on the initial `Configuring` stage.
    /// 15 on a fresh build (12 rate PIDs + `THR_MDL_FAC` + `MPC_THR_HOVER`
    /// + `MPC_THR_MIN`).
    pub verified_params: u32,
}

/// Result of a `RequestPreflightCheck` — an instant, cache-only read of the
/// FC's last-known HITL/quadrotor status (no MAVLink round-trip).
///
/// ## Binary format (0x0A)
/// - `[0]`: 0x0A message type
/// - `[1-N]`: JSON body `{ "connected": bool, "hitl_enabled": bool,
///   "is_quadrotor": bool, "readiness": string, "board_identity"?: string }`
#[derive(Debug, Clone, Serialize)]
pub struct PreflightStatus {
    /// Whether any HEARTBEAT has ever been received (false in --sim-only or
    /// before the first HEARTBEAT arrives).
    pub connected: bool,
    pub hitl_enabled: bool,
    pub is_quadrotor: bool,
    /// Explicit readiness verdict.
    ///
    /// Exists because `connected: false` is ambiguous: it covers both "no FC
    /// is expected" (sim-only) and "an FC is expected but has not reported
    /// yet". Collapsing those let the browser fall straight through the gate
    /// while a board was still booting — the most common first-run state.
    pub readiness: PreflightReadiness,
    /// Which board this is, once it has reported enough to say.
    ///
    /// Sent here rather than only with the snapshot, because the browser needs
    /// to know *which* board it is asking the user about before any write
    /// happens. Carrying it only in the snapshot made identity available only
    /// during provisioning — after the point where consent for that
    /// provisioning is sought, which left approval unable to name its subject.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub board_identity: Option<String>,
}

/// Whether the connected flight controller is ready for HITL.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PreflightReadiness {
    /// An FC is expected but has not reported its state yet. Must be treated
    /// as a wait, never as a pass.
    Unknown,
    /// HITL mode and a quadrotor airframe both confirmed.
    Ready,
    /// The board reported, and one or both signals are wrong.
    NotReady,
    /// No flight controller is part of this session (--sim-only). Nothing to
    /// gate on, so the flow proceeds.
    NotApplicable,
}

impl PreflightStatus {
    pub fn to_bytes(&self) -> Vec<u8> {
        let json = serde_json::to_vec(self).expect("PreflightStatus serialization cannot fail");
        let mut buf = Vec::with_capacity(1 + json.len());
        buf.push(MSG_TYPE_PREFLIGHT_STATUS);
        buf.extend_from_slice(&json);
        buf
    }
}

/// One flight-controller parameter as it stood before provisioning.
///
/// `param_type` is carried because PX4 silently drops a `PARAM_SET` whose type
/// does not match the parameter's declared type. A snapshot that records only
/// name and value cannot be replayed onto the board.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SnapshotParam {
    pub name: String,
    pub value: f32,
    /// "int32" or "real32" — the two shapes PX4's PARAM_SET encoding cares
    /// about, rather than the full MAVLink type enum.
    pub param_type: String,
}

/// Parameters read off the board before any write, sent to the browser for
/// durable storage.
///
/// ## Binary format (0x0C)
/// - `[0]`: 0x0C message type
/// - `[1-N]`: JSON body
#[derive(Debug, Clone, Serialize)]
pub struct SnapshotCaptured {
    /// Stable key for the board these values came from. Restore is refused
    /// when this does not match the connected board.
    pub board_identity: String,
    pub params: Vec<SnapshotParam>,
}

impl SnapshotCaptured {
    pub fn to_bytes(&self) -> Vec<u8> {
        let json = serde_json::to_vec(self).expect("SnapshotCaptured serialization cannot fail");
        let mut buf = Vec::with_capacity(1 + json.len());
        buf.push(MSG_TYPE_SNAPSHOT_CAPTURED);
        buf.extend_from_slice(&json);
        buf
    }
}

/// Browser's confirmation that a captured snapshot is durably stored.
///
/// Provisioning blocks on this. Writing to the board before the restore point
/// is safe would open a window where the board is modified and nothing can put
/// it back, which is the exact state the snapshot exists to prevent.
///
/// ## Binary format (0x16)
/// - `[0]`: 0x16 message type
/// - `[1-N]`: JSON body
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct SnapshotStored {
    pub board_identity: String,
    /// False when the browser could not persist (quota exceeded, storage
    /// disabled). Provisioning must abort rather than proceed unprotected.
    pub stored: bool,
    #[serde(default)]
    pub error: Option<String>,
}

impl SnapshotStored {
    pub fn from_bytes(data: &[u8]) -> Result<Self, ProtocolError> {
        if data.len() < 2 {
            return Err(ProtocolError::MessageTooShort {
                expected: 2,
                actual: data.len(),
            });
        }
        if data[0] != MSG_TYPE_SNAPSHOT_STORED {
            return Err(ProtocolError::UnknownMessageType(data[0]));
        }
        serde_json::from_slice(&data[1..]).map_err(|e| ProtocolError::InvalidPayload {
            command_type: CommandType::Arm,
            reason: format!("SnapshotStored: {e}"),
        })
    }
}

/// Feature names this daemon supports.
///
/// A version number alone forces every client to carry a table of which
/// version gained what. Named features let a mixed fleet degrade per-feature
/// instead of per-version.
pub const DAEMON_FEATURES: &[&str] = &[
    "param_snapshot",
    "param_restore",
    "board_identity",
    "heartbeat_probe",
    "provisioning_broadcast",
    "link_state",
];

/// Current wire-protocol revision.
///
/// Bumped when an existing message's layout or meaning changes, not when a new
/// message type is added — clients ignore message types they do not know.
pub const PROTOCOL_REVISION: u16 = 2;

/// What this daemon is and what it can do, sent unprompted on connect.
///
/// The binary handshake that predates this packs a version into fixed byte
/// positions and already carries a legacy-layout heuristic to disambiguate two
/// past encodings — evidence that byte-packed version fields do not survive
/// protocol evolution. This frame is JSON and additive.
///
/// ## Binary format (0x0E)
/// - `[0]`: 0x0E message type
/// - `[1-N]`: JSON body
#[derive(Debug, Clone, Serialize)]
pub struct Capabilities {
    pub daemon_version: String,
    pub protocol_revision: u16,
    pub features: Vec<String>,
}

impl Capabilities {
    pub fn current(daemon_version: impl Into<String>) -> Self {
        Self {
            daemon_version: daemon_version.into(),
            protocol_revision: PROTOCOL_REVISION,
            features: DAEMON_FEATURES.iter().map(|f| (*f).to_string()).collect(),
        }
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let json = serde_json::to_vec(self).expect("Capabilities serialization cannot fail");
        let mut buf = Vec::with_capacity(1 + json.len());
        buf.push(MSG_TYPE_CAPABILITIES);
        buf.extend_from_slice(&json);
        buf
    }
}

/// Browser's request to write a stored snapshot back to the flight
/// controller.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct RestoreSnapshot {
    /// The board the snapshot was captured from. Refused when it does not
    /// match the connected board: restoring one board's tuning onto another
    /// is the failure the whole snapshot mechanism exists to prevent.
    pub board_identity: String,
    pub params: Vec<SnapshotParam>,
}

impl RestoreSnapshot {
    pub fn from_bytes(data: &[u8]) -> Result<Self, ProtocolError> {
        if data.len() < 2 {
            return Err(ProtocolError::MessageTooShort {
                expected: 2,
                actual: data.len(),
            });
        }
        if data[0] != MSG_TYPE_RESTORE_SNAPSHOT {
            return Err(ProtocolError::UnknownMessageType(data[0]));
        }
        serde_json::from_slice(&data[1..]).map_err(|e| ProtocolError::InvalidPayload {
            command_type: CommandType::Arm,
            reason: format!("RestoreSnapshot: {e}"),
        })
    }
}

/// Lifecycle stage of a restore.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RestoreState {
    Writing,
    Rebooting,
    Reconnecting,
    Verifying,
    Done,
    Error,
}

/// One parameter that did not read back as its snapshotted value.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RestoreMismatch {
    pub name: String,
    pub expected: f32,
    pub actual: f32,
}

/// ## Binary format (0x0D)
/// - `[0]`: 0x0D message type
/// - `[1-N]`: JSON body
#[derive(Debug, Clone, Serialize)]
pub struct RestoreResult {
    pub state: RestoreState,
    pub success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Populated when verification found differences. The board is NOT
    /// reported as restored in that case — the user needs to know exactly
    /// which values differ and what they now hold.
    #[serde(skip_serializing_if = "Vec::is_empty", default)]
    pub mismatches: Vec<RestoreMismatch>,
}

impl RestoreResult {
    pub fn to_bytes(&self) -> Vec<u8> {
        let json = serde_json::to_vec(self).expect("RestoreResult serialization cannot fail");
        let mut buf = Vec::with_capacity(1 + json.len());
        buf.push(MSG_TYPE_RESTORE_RESULT);
        buf.extend_from_slice(&json);
        buf
    }
}

/// Lifecycle stage of an `ApplyPreflightParams` request. Mirrors
/// `ConfigState`'s two-stage shape: the same message type carries both
/// interim progress (`success: true`, non-terminal `state`) and the final
/// result (`Done`/`Error`).
#[derive(Debug, Clone, Copy, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PreflightApplyState {
    /// Reading the board's current parameters and waiting for the browser to
    /// persist them. No write has happened yet at this stage.
    Capturing,
    Applying,
    Rebooting,
    Reconnecting,
    Verifying,
    Done,
    Error,
}

/// ## Binary format (0x0B)
/// - `[0]`: 0x0B message type
/// - `[1-N]`: JSON body `{ "state": ..., "success": bool, "error"?: string }`
#[derive(Debug, Clone, Serialize)]
pub struct PreflightApplyResult {
    pub state: PreflightApplyState,
    pub success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

impl PreflightApplyResult {
    pub fn to_bytes(&self) -> Vec<u8> {
        let json =
            serde_json::to_vec(self).expect("PreflightApplyResult serialization cannot fail");
        let mut buf = Vec::with_capacity(1 + json.len());
        buf.push(MSG_TYPE_PREFLIGHT_APPLY_RESULT);
        buf.extend_from_slice(&json);
        buf
    }
}

/// JSON-friendly view of `hitl_physics::px4_pids::Px4Pids` for transport over
/// the WebSocket. Kept here (rather than re-exporting upstream) so the wire
/// schema is owned by the protocol crate and can evolve independently.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Px4PidsView {
    pub roll_p: f32,
    pub roll_i: f32,
    pub roll_d: f32,
    pub roll_ff: f32,
    pub pitch_p: f32,
    pub pitch_i: f32,
    pub pitch_d: f32,
    pub pitch_ff: f32,
    pub yaw_p: f32,
    pub yaw_i: f32,
    pub yaw_d: f32,
    pub yaw_ff: f32,
}

impl HandshakeAck {
    /// Serialize to binary format
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(6 + self.serial_port.len() + 1);
        buf.push(MSG_TYPE_HANDSHAKE_ACK);
        buf.push(self.version_major);
        buf.push(self.version_minor);
        buf.push(self.version_patch);
        buf.push(self.pixhawk_connected as u8);
        buf.extend_from_slice(self.serial_port.as_bytes());
        buf.push(0); // null terminator
        buf
    }
}

/// Command acknowledgment sent to browser
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandAck {
    pub command_id: u32,
    pub success: bool,
    pub error: Option<String>,
}

/// NSH command request from browser
///
/// ## Binary format (0x12)
/// - `[0]`: 0x12 message type
/// - `[1-4]`: request_id (u32 LE)
/// - `[5-6]`: timeout_ms (u16 LE, 0 = default 2000ms)
/// - `[7-N]`: command string (null-terminated UTF-8)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NshCommand {
    pub request_id: u32,
    pub timeout_ms: u16,
    pub command: String,
}

impl NshCommand {
    /// Deserialize from binary format
    pub fn from_bytes(data: &[u8]) -> Result<Self, ProtocolError> {
        if data.len() < 8 {
            return Err(ProtocolError::MessageTooShort {
                expected: 8,
                actual: data.len(),
            });
        }

        if data[0] != MSG_TYPE_NSH_COMMAND {
            return Err(ProtocolError::UnknownMessageType(data[0]));
        }

        let request_id = u32::from_le_bytes(data[1..5].try_into().unwrap());
        let timeout_ms = u16::from_le_bytes(data[5..7].try_into().unwrap());

        // Find null terminator or end of buffer
        let cmd_start = 7;
        let cmd_end = data[cmd_start..]
            .iter()
            .position(|&b| b == 0)
            .map(|p| cmd_start + p)
            .unwrap_or(data.len());

        let command = String::from_utf8_lossy(&data[cmd_start..cmd_end]).to_string();

        Ok(Self {
            request_id,
            timeout_ms,
            command,
        })
    }

    /// Serialize to binary format
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(8 + self.command.len());
        buf.push(MSG_TYPE_NSH_COMMAND);
        buf.extend_from_slice(&self.request_id.to_le_bytes());
        buf.extend_from_slice(&self.timeout_ms.to_le_bytes());
        buf.extend_from_slice(self.command.as_bytes());
        buf.push(0); // null terminator
        buf
    }
}

/// NSH response sent to browser
///
/// ## Binary format (0x04)
/// - `[0]`: 0x04 message type
/// - `[1-4]`: request_id (u32 LE)
/// - `[5]`: success (u8 bool)
/// - `[6]`: complete (u8 bool) - false if response is chunked
/// - `[7-N]`: output string (null-terminated UTF-8)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NshResponse {
    pub request_id: u32,
    pub success: bool,
    pub complete: bool,
    pub output: String,
}

impl NshResponse {
    /// Serialize to binary format
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(8 + self.output.len());
        buf.push(MSG_TYPE_NSH_RESPONSE);
        buf.extend_from_slice(&self.request_id.to_le_bytes());
        buf.push(self.success as u8);
        buf.push(self.complete as u8);
        buf.extend_from_slice(self.output.as_bytes());
        buf.push(0); // null terminator
        buf
    }
}

impl CommandAck {
    /// Serialize to binary format
    pub fn to_bytes(&self) -> Vec<u8> {
        let error_bytes = self.error.as_deref().unwrap_or("");
        let mut buf = Vec::with_capacity(6 + error_bytes.len() + 1);
        buf.push(MSG_TYPE_COMMAND_ACK);
        buf.extend_from_slice(&self.command_id.to_le_bytes());
        buf.push(self.success as u8);
        if !self.success {
            buf.extend_from_slice(error_bytes.as_bytes());
            buf.push(0); // null terminator
        }
        buf
    }
}

/// Command types that can be sent from browser
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum CommandType {
    Arm = 0,
    Disarm = 1,
    Takeoff = 2,
    Land = 3,
    Rtl = 4,
    SetMode = 5,
    EmergencyStop = 6,
    Recharge = 7,
}

impl std::fmt::Display for CommandType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CommandType::Arm => write!(f, "Arm"),
            CommandType::Disarm => write!(f, "Disarm"),
            CommandType::Takeoff => write!(f, "Takeoff"),
            CommandType::Land => write!(f, "Land"),
            CommandType::Rtl => write!(f, "RTL"),
            CommandType::SetMode => write!(f, "SetMode"),
            CommandType::EmergencyStop => write!(f, "EmergencyStop"),
            CommandType::Recharge => write!(f, "Recharge"),
        }
    }
}

impl TryFrom<u8> for CommandType {
    type Error = ProtocolError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(CommandType::Arm),
            1 => Ok(CommandType::Disarm),
            2 => Ok(CommandType::Takeoff),
            3 => Ok(CommandType::Land),
            4 => Ok(CommandType::Rtl),
            5 => Ok(CommandType::SetMode),
            6 => Ok(CommandType::EmergencyStop),
            7 => Ok(CommandType::Recharge),
            _ => Err(ProtocolError::InvalidCommandType(value)),
        }
    }
}

/// Command received from browser
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Command {
    pub command_id: u32,
    pub command_type: CommandType,
    pub takeoff_altitude: Option<f32>,
    pub set_mode_value: Option<u8>,
}

impl Command {
    /// Deserialize from binary format
    pub fn from_bytes(data: &[u8]) -> Result<Self, ProtocolError> {
        if data.len() < 6 {
            return Err(ProtocolError::MessageTooShort {
                expected: 6,
                actual: data.len(),
            });
        }

        if data[0] != MSG_TYPE_COMMAND {
            return Err(ProtocolError::UnknownMessageType(data[0]));
        }

        let command_id = u32::from_le_bytes(data[1..5].try_into().unwrap());
        let command_type = CommandType::try_from(data[5])?;

        let mut cmd = Command {
            command_id,
            command_type,
            takeoff_altitude: None,
            set_mode_value: None,
        };

        // Parse command-specific payload
        match command_type {
            CommandType::Takeoff => {
                if data.len() < 10 {
                    return Err(ProtocolError::InvalidPayload {
                        command_type,
                        reason: "Takeoff requires altitude parameter (4 bytes)".to_string(),
                    });
                }
                cmd.takeoff_altitude = Some(f32::from_le_bytes(data[6..10].try_into().unwrap()));
            }
            CommandType::SetMode => {
                if data.len() < 7 {
                    return Err(ProtocolError::InvalidPayload {
                        command_type,
                        reason: "SetMode requires mode parameter (1 byte)".to_string(),
                    });
                }
                cmd.set_mode_value = Some(data[6]);
            }
            _ => {}
        }

        Ok(cmd)
    }

    /// Serialize to binary format
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(10);
        buf.push(MSG_TYPE_COMMAND);
        buf.extend_from_slice(&self.command_id.to_le_bytes());
        buf.push(self.command_type as u8);

        match self.command_type {
            CommandType::Takeoff => {
                if let Some(alt) = self.takeoff_altitude {
                    buf.extend_from_slice(&alt.to_le_bytes());
                }
            }
            CommandType::SetMode => {
                if let Some(mode) = self.set_mode_value {
                    buf.push(mode);
                }
            }
            _ => {}
        }

        buf
    }
}

/// Terrain origin sent to browser (event-driven, not periodic).
///
/// ## Binary format (0x09)
/// - `[0]`: 0x09 message type
/// - `[1-8]`: ref_lat (f64 LE, degrees WGS84)
/// - `[9-16]`: ref_lon (f64 LE, degrees WGS84)
/// - `[17-20]`: ref_alt (f32 LE, metres AMSL)
/// - `[21]`: source (u8: 0=GlobalPositionInt, 1=HomePosition, 2=GpsGlobalOrigin)
/// - Total: 22 bytes
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TerrainOrigin {
    pub ref_lat: f64,
    pub ref_lon: f64,
    pub ref_alt: f32,
    pub source: u8,
}

impl TerrainOrigin {
    pub const SIZE: usize = 22;

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(Self::SIZE);
        buf.push(MSG_TYPE_TERRAIN_ORIGIN);
        buf.extend_from_slice(&self.ref_lat.to_le_bytes());
        buf.extend_from_slice(&self.ref_lon.to_le_bytes());
        buf.extend_from_slice(&self.ref_alt.to_le_bytes());
        buf.push(self.source);
        buf
    }
}

/// A slippy/XYZ tile coordinate, as carried on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct WireTileCoord {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

/// Daemon -> browser: tiles the physics needs around the vehicle but does not hold.
///
/// Reconciliation, not RPC. The daemon is the only party that knows what it is
/// missing, so it is the one that asks; unmet needs are simply re-sent, which
/// makes the exchange self-healing across dropped frames, tab reloads and
/// daemon restarts without any acknowledgement bookkeeping. The steady state is
/// an empty list, which is not sent at all.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TerrainNeed {
    pub coords: Vec<WireTileCoord>,
}

impl TerrainNeed {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = vec![MSG_TYPE_TERRAIN_NEED];
        // Serialising a Vec of small structs cannot fail; an empty list on
        // failure would read as "nothing needed" and stall the exchange.
        buf.extend_from_slice(
            serde_json::to_vec(self)
                .expect("TerrainNeed serialises")
                .as_slice(),
        );
        buf
    }
}

/// Header of a `TerrainTiles` frame, describing the payloads that follow it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TerrainTilesHeader {
    /// Origin these tiles are anchored to. A frame anchored to an origin the
    /// daemon has since left is rejected rather than mixed with the current
    /// one -- see `TerrainCache::is_anchored_to`, which the ingress consults
    /// before inserting anything.
    pub origin: FlightLocation,
    /// Samples per tile edge. Checked against the payload length so a mismatch
    /// is caught at the boundary rather than read as garbage heights.
    pub tile_size: u32,
    pub coords: Vec<WireTileCoord>,
    /// Whether each tile came from a non-authoritative source, positionally
    /// matched to `coords`.
    #[serde(default)]
    pub approximate: Vec<bool>,
}

/// Browser -> daemon: decoded elevation tiles.
///
/// Frame layout, following the one-tag-byte convention:
///   `[0]`      `MSG_TYPE_TERRAIN_TILES`
///   `[1..5]`   header length, u32 LE
///   `[5..5+n]` `TerrainTilesHeader` as JSON
///   `[5+n..]`  concatenated tile payloads, each `tile_size^2` f32 LE,
///              row-major from the tile's north-west corner
///
/// The payload is byte-identical to the `.bin` tile format the store serves and
/// the viewer decodes, so there is one elevation representation end to end and
/// no second encoding to keep in step with the file format.
#[derive(Debug, Clone, PartialEq)]
pub struct TerrainTiles {
    pub header: TerrainTilesHeader,
    /// One entry per coord, each `tile_size^2` MSL metres.
    pub tiles: Vec<Vec<f32>>,
}

impl TerrainTiles {
    /// Largest frame accepted, as a backstop against a malicious or runaway
    /// sender. 64 tiles at 256^2 f32 is 16 MiB; the daemon's resident bound is
    /// far below that, so anything approaching it is already wrong.
    pub const MAX_FRAME_BYTES: usize = 16 * 1024 * 1024;

    pub fn from_bytes(data: &[u8]) -> Result<Self, ProtocolError> {
        let invalid = |reason: String| ProtocolError::InvalidPayload {
            command_type: CommandType::Arm,
            reason,
        };

        if data.len() > Self::MAX_FRAME_BYTES {
            return Err(invalid(format!(
                "TerrainTiles: frame of {} bytes exceeds the {} byte limit",
                data.len(),
                Self::MAX_FRAME_BYTES
            )));
        }
        if data.len() < 5 {
            return Err(ProtocolError::MessageTooShort {
                expected: 5,
                actual: data.len(),
            });
        }
        if data[0] != MSG_TYPE_TERRAIN_TILES {
            return Err(ProtocolError::UnknownMessageType(data[0]));
        }

        let header_len = u32::from_le_bytes([data[1], data[2], data[3], data[4]]) as usize;
        let header_end = 5usize
            .checked_add(header_len)
            .ok_or_else(|| invalid("TerrainTiles: header length overflows".to_string()))?;
        if data.len() < header_end {
            return Err(ProtocolError::MessageTooShort {
                expected: header_end,
                actual: data.len(),
            });
        }

        let header: TerrainTilesHeader = serde_json::from_slice(&data[5..header_end])
            .map_err(|e| invalid(format!("TerrainTiles: header {e}")))?;

        let tile_size = header.tile_size as usize;
        if tile_size == 0 {
            return Err(invalid(
                "TerrainTiles: tile_size must be non-zero".to_string(),
            ));
        }
        let samples = tile_size
            .checked_mul(tile_size)
            .ok_or_else(|| invalid("TerrainTiles: tile_size overflows".to_string()))?;
        let bytes_per_tile = samples
            .checked_mul(4)
            .ok_or_else(|| invalid("TerrainTiles: tile_size overflows".to_string()))?;

        if !header.approximate.is_empty() && header.approximate.len() != header.coords.len() {
            return Err(invalid(format!(
                "TerrainTiles: {} approximate flags for {} coords",
                header.approximate.len(),
                header.coords.len()
            )));
        }

        let expected = header
            .coords
            .len()
            .checked_mul(bytes_per_tile)
            .ok_or_else(|| invalid("TerrainTiles: payload size overflows".to_string()))?;

        // Two payload placements are accepted: immediately after the header, and
        // at the next 4-byte boundary after it.
        //
        // The sender writes f32 samples, and a browser can only use
        // `Float32Array.prototype.set` -- one memcpy -- when the payload begins
        // at a multiple of four. Unaligned, it must fall back to a
        // `DataView.setFloat32` per sample, which is 589,824 calls for the nine
        // tiles of a single push. Padding the header to the boundary is what
        // lets the fast path exist.
        //
        // Accepting both is what makes that change deployable at all. A daemon
        // is installed on someone's machine and updates on their schedule, so a
        // browser that padded unconditionally would take terrain away from every
        // daemon older than the change. This lands first, is released, and only
        // then does the sender start padding.
        //
        // The two are told apart by length, not by a flag: padding is 0-3 bytes,
        // so at most one placement can leave exactly `expected` bytes behind.
        // When the header already ends on a boundary the two are the same offset
        // and the first test takes it.
        //
        // The padding bytes themselves are not inspected. They carry nothing, and
        // rejecting a frame over filler would be refusing a well-formed payload
        // for the sake of bytes with no meaning.
        let pad = (4 - (header_end % 4)) % 4;
        let padded_start = header_end
            .checked_add(pad)
            .ok_or_else(|| invalid("TerrainTiles: padded offset overflows".to_string()))?;

        let body_start = if data.len().saturating_sub(header_end) == expected {
            header_end
        } else if data.len().saturating_sub(padded_start) == expected {
            padded_start
        } else {
            return Err(invalid(format!(
                "TerrainTiles: {} payload bytes for {} tiles of {}x{} (expected {} at offset {} \
                 unpadded, or {} padded to the 4-byte boundary)",
                data.len().saturating_sub(header_end),
                header.coords.len(),
                tile_size,
                tile_size,
                expected,
                header_end,
                padded_start
            )));
        };
        let body = &data[body_start..];

        let tiles = body
            .chunks_exact(bytes_per_tile)
            .map(|chunk| {
                chunk
                    .chunks_exact(4)
                    .map(|f| f32::from_le_bytes([f[0], f[1], f[2], f[3]]))
                    .collect()
            })
            .collect();

        Ok(Self { header, tiles })
    }

    /// Encode a frame. Used by tests and by any non-browser producer.
    pub fn to_bytes(&self) -> Vec<u8> {
        let header = serde_json::to_vec(&self.header).expect("TerrainTilesHeader serialises");
        let mut buf = vec![MSG_TYPE_TERRAIN_TILES];
        buf.extend_from_slice(&(header.len() as u32).to_le_bytes());
        buf.extend_from_slice(&header);
        for tile in &self.tiles {
            for h in tile {
                buf.extend_from_slice(&h.to_le_bytes());
            }
        }
        buf
    }
}

/// All possible outgoing messages
#[derive(Debug, Clone)]
pub enum OutgoingMessage {
    StateUpdate(StateUpdate),
    HandshakeAck(HandshakeAck),
    CommandAck(CommandAck),
    NshResponse(NshResponse),
    ConnectionStatus(ConnectionStatus),
    VehicleMessage(VehicleMessage),
    ConfigResult(ConfigResult),
    TerrainOrigin(TerrainOrigin),
    TerrainNeed(TerrainNeed),
    SnapshotCaptured(SnapshotCaptured),
    RestoreResult(RestoreResult),
    Capabilities(Capabilities),
    PreflightStatus(PreflightStatus),
    PreflightApplyResult(PreflightApplyResult),
}

impl OutgoingMessage {
    pub fn to_bytes(&self) -> Vec<u8> {
        match self {
            OutgoingMessage::StateUpdate(s) => s.to_bytes().to_vec(),
            OutgoingMessage::HandshakeAck(h) => h.to_bytes(),
            OutgoingMessage::CommandAck(c) => c.to_bytes(),
            OutgoingMessage::NshResponse(n) => n.to_bytes(),
            OutgoingMessage::ConnectionStatus(c) => c.to_bytes(),
            OutgoingMessage::VehicleMessage(v) => v.to_bytes(),
            OutgoingMessage::ConfigResult(r) => r.to_bytes(),
            OutgoingMessage::TerrainOrigin(t) => t.to_bytes(),
            OutgoingMessage::TerrainNeed(t) => t.to_bytes(),
            OutgoingMessage::SnapshotCaptured(s) => s.to_bytes(),
            OutgoingMessage::RestoreResult(r) => r.to_bytes(),
            OutgoingMessage::Capabilities(c) => c.to_bytes(),
            OutgoingMessage::PreflightStatus(p) => p.to_bytes(),
            OutgoingMessage::PreflightApplyResult(p) => p.to_bytes(),
        }
    }
}

/// All possible incoming messages
#[derive(Debug, Clone)]
pub enum IncomingMessage {
    Command(Command),
    Handshake,
    NshCommand(NshCommand),
    ConfigureBuild(ConfigureBuild),
    Shutdown,
    RequestPreflightCheck,
    ApplyPreflightParams,
    SnapshotStored(SnapshotStored),
    RestoreSnapshot(RestoreSnapshot),
    TerrainTiles(TerrainTiles),
}

impl IncomingMessage {
    /// Parse an incoming message from bytes
    pub fn from_bytes(data: &[u8]) -> Result<Self, ProtocolError> {
        if data.is_empty() {
            return Err(ProtocolError::MessageTooShort {
                expected: 1,
                actual: 0,
            });
        }

        match data[0] {
            MSG_TYPE_COMMAND => Ok(IncomingMessage::Command(Command::from_bytes(data)?)),
            MSG_TYPE_HANDSHAKE => Ok(IncomingMessage::Handshake),
            MSG_TYPE_NSH_COMMAND => Ok(IncomingMessage::NshCommand(NshCommand::from_bytes(data)?)),
            MSG_TYPE_CONFIGURE_BUILD => Ok(IncomingMessage::ConfigureBuild(
                ConfigureBuild::from_bytes(data)?,
            )),
            MSG_TYPE_TERRAIN_TILES => Ok(IncomingMessage::TerrainTiles(TerrainTiles::from_bytes(
                data,
            )?)),
            MSG_TYPE_SHUTDOWN => Ok(IncomingMessage::Shutdown),
            MSG_TYPE_REQUEST_PREFLIGHT_CHECK => Ok(IncomingMessage::RequestPreflightCheck),
            MSG_TYPE_APPLY_PREFLIGHT_PARAMS => Ok(IncomingMessage::ApplyPreflightParams),
            MSG_TYPE_SNAPSHOT_STORED => Ok(IncomingMessage::SnapshotStored(
                SnapshotStored::from_bytes(data)?,
            )),
            MSG_TYPE_RESTORE_SNAPSHOT => Ok(IncomingMessage::RestoreSnapshot(
                RestoreSnapshot::from_bytes(data)?,
            )),
            other => Err(ProtocolError::UnknownMessageType(other)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A browser on the current protocol must still parse frames from a daemon
    /// that predates the landed_state byte, reporting UNDEFINED rather than
    /// rejecting the frame and blanking the whole telemetry view.
    #[test]
    fn state_update_accepts_legacy_86_byte_frame() {
        let state = StateUpdate {
            timestamp_us: 42,
            armed: true,
            flight_mode: 3,
            landed_state: 4,
            ..StateUpdate::default()
        };

        let full = state.to_bytes();
        let legacy = &full[..STATE_UPDATE_MIN_SIZE];

        let parsed = StateUpdate::from_bytes(legacy).expect("legacy frame must parse");
        assert_eq!(parsed.timestamp_us, 42);
        assert_eq!(parsed.flight_mode, 3);
        assert!(parsed.armed);
        assert_eq!(
            parsed.landed_state, 0,
            "a frame without the byte must decode as UNDEFINED, not carry stale data"
        );
    }

    /// Anything shorter than the legacy frame is genuinely malformed.
    #[test]
    fn state_update_rejects_short_frame() {
        let short = [MSG_TYPE_STATE_UPDATE; 40];
        assert!(StateUpdate::from_bytes(&short).is_err());
    }

    #[test]
    fn test_state_update_roundtrip() {
        let state = StateUpdate {
            timestamp_us: 1234567890,
            position_ned: [1.0, 2.0, -3.0],
            velocity_ned: [0.1, 0.2, -0.3],
            quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
            angular_velocity: [0.01, 0.02, 0.03],
            motor_rpms: [5000.0, 5100.0, 5200.0, 5300.0],
            battery_voltage: 16.2,
            battery_percent: 85,
            armed: true,
            flight_mode: 3,
            packets_per_sec: 349,
            landed_state: 2, // MAV_LANDED_STATE_IN_AIR
        };

        let bytes = state.to_bytes();
        assert_eq!(bytes.len(), STATE_UPDATE_SIZE);
        assert_eq!(bytes[0], MSG_TYPE_STATE_UPDATE);
        assert_eq!(bytes[86], 2, "landed_state must occupy the trailing byte");

        let parsed = StateUpdate::from_bytes(&bytes).unwrap();
        assert_eq!(parsed.timestamp_us, state.timestamp_us);
        assert_eq!(parsed.position_ned, state.position_ned);
        assert_eq!(parsed.velocity_ned, state.velocity_ned);
        assert_eq!(parsed.quaternion_wxyz, state.quaternion_wxyz);
        assert_eq!(parsed.angular_velocity, state.angular_velocity);
        assert_eq!(parsed.motor_rpms, state.motor_rpms);
        assert!((parsed.battery_voltage - state.battery_voltage).abs() < 0.001);
        assert_eq!(parsed.battery_percent, state.battery_percent);
        assert_eq!(parsed.armed, state.armed);
        assert_eq!(parsed.flight_mode, state.flight_mode);
        assert_eq!(parsed.landed_state, state.landed_state);
        assert_eq!(parsed.packets_per_sec, state.packets_per_sec);
    }

    #[test]
    fn test_shutdown_message() {
        let bytes = [MSG_TYPE_SHUTDOWN];
        let msg = IncomingMessage::from_bytes(&bytes).unwrap();
        assert!(matches!(msg, IncomingMessage::Shutdown));
    }

    #[test]
    fn test_connection_status_with_fc_model() {
        let status = ConnectionStatus {
            connected: true,
            reconnecting: false,
            retry_count: 0,
            serial_port: "/dev/tty.usb".to_string(),
            fc_model: Some("Pixhawk 6C".to_string()),
            bootloader_suspected: false,
            link_state: LinkState::Connected,
        };
        let bytes = status.to_bytes();
        assert_eq!(bytes[0], MSG_TYPE_CONNECTION_STATUS);
        assert_eq!(bytes[1], 1); // connected
        assert_eq!(bytes[2], 0); // not reconnecting
        assert_eq!(bytes[3], 0); // retry_count

        // Find null terminators
        let port_null = bytes[4..].iter().position(|&b| b == 0).unwrap() + 4;
        let port_str = std::str::from_utf8(&bytes[4..port_null]).unwrap();
        assert_eq!(port_str, "/dev/tty.usb");

        let model_start = port_null + 1;
        let model_null = bytes[model_start..].iter().position(|&b| b == 0).unwrap() + model_start;
        let model_str = std::str::from_utf8(&bytes[model_start..model_null]).unwrap();
        assert_eq!(model_str, "Pixhawk 6C");

        // bootloader_suspected byte is appended after fc_model null terminator
        assert_eq!(bytes[model_null + 1], 0); // not bootloader_suspected
    }

    #[test]
    fn test_connection_status_without_fc_model() {
        let status = ConnectionStatus {
            connected: false,
            reconnecting: true,
            retry_count: 3,
            serial_port: String::new(),
            fc_model: None,
            bootloader_suspected: false,
            link_state: LinkState::Reconnecting,
        };
        let bytes = status.to_bytes();
        assert_eq!(bytes[1], 0); // not connected
        assert_eq!(bytes[2], 1); // reconnecting
        assert_eq!(bytes[3], 3); // retry_count
                                 // Empty serial port: null terminator at index 4
        assert_eq!(bytes[4], 0);
        // Empty fc_model: null terminator at index 5
        assert_eq!(bytes[5], 0);
        // bootloader_suspected at index 6
        assert_eq!(bytes[6], 0);
    }

    #[test]
    fn test_connection_status_bootloader_suspected() {
        let status = ConnectionStatus {
            connected: false,
            reconnecting: true,
            retry_count: 1,
            serial_port: String::new(),
            fc_model: None,
            bootloader_suspected: true,
            link_state: LinkState::SuspectedBootloader,
        };
        let bytes = status.to_bytes();
        assert_eq!(bytes[1], 0); // not connected
        assert_eq!(bytes[2], 1); // reconnecting
                                 // bootloader_suspected byte is at index 6 (after two empty null-terminated strings)
        assert_eq!(bytes[6], 1); // bootloader_suspected = true
    }

    /// Pins the exact bytes of the preflight status frame. The frontend builds
    /// its own fixture bytes independently, so a field rename here would pass
    /// every other test in both repos and only fail on real hardware.
    #[test]
    fn test_preflight_status_to_bytes() {
        let status = PreflightStatus {
            connected: true,
            hitl_enabled: false,
            is_quadrotor: true,
            readiness: PreflightReadiness::NotReady,
            board_identity: Some("uid:0011223344556677".to_string()),
        };
        let bytes = status.to_bytes();
        assert_eq!(bytes[0], MSG_TYPE_PREFLIGHT_STATUS);

        let body: serde_json::Value = serde_json::from_slice(&bytes[1..]).unwrap();
        assert_eq!(body["connected"], serde_json::json!(true));
        assert_eq!(body["hitl_enabled"], serde_json::json!(false));
        assert_eq!(body["is_quadrotor"], serde_json::json!(true));
        // readiness is the field the browser gates on; connected alone is
        // ambiguous between "no FC expected" and "FC has not reported yet".
        assert_eq!(body["readiness"], serde_json::json!("not_ready"));
        // The browser must know which board it is asking the user about
        // before any write, so identity travels with status rather than only
        // with the snapshot - which is captured after consent is sought.
        assert_eq!(
            body["board_identity"],
            serde_json::json!("uid:0011223344556677")
        );
        // Exactly those five keys — no stray or renamed fields.
        assert_eq!(body.as_object().unwrap().len(), 5);
    }

    /// Pins the snake_case rendering of `PreflightApplyState` (the frontend's
    /// TypeScript union matches on these literals) and the omission of
    /// `error` when it is `None`.
    #[test]
    fn test_preflight_apply_result_to_bytes() {
        let result = PreflightApplyResult {
            state: PreflightApplyState::Rebooting,
            success: true,
            error: None,
        };
        let bytes = result.to_bytes();
        assert_eq!(bytes[0], MSG_TYPE_PREFLIGHT_APPLY_RESULT);

        let body: serde_json::Value = serde_json::from_slice(&bytes[1..]).unwrap();
        assert_eq!(body["state"], serde_json::json!("rebooting"));
        assert_eq!(body["success"], serde_json::json!(true));
        assert!(
            body.get("error").is_none(),
            "error must be skipped entirely when None, got: {body}"
        );
    }

    #[test]
    fn test_handshake_ack_to_bytes() {
        let ack = HandshakeAck {
            version_major: 1,
            version_minor: 2,
            version_patch: 3,
            pixhawk_connected: true,
            serial_port: "/dev/ttyACM0".to_string(),
        };

        let bytes = ack.to_bytes();
        assert_eq!(bytes[0], MSG_TYPE_HANDSHAKE_ACK);
        assert_eq!(bytes[1], 1);
        assert_eq!(bytes[2], 2);
        assert_eq!(bytes[3], 3);
        assert_eq!(bytes[4], 1);
        assert_eq!(&bytes[5..17], b"/dev/ttyACM0");
        assert_eq!(bytes[17], 0);
    }

    #[test]
    fn test_command_roundtrip() {
        let cmd = Command {
            command_id: 42,
            command_type: CommandType::Takeoff,
            takeoff_altitude: Some(10.0),
            set_mode_value: None,
        };

        let bytes = cmd.to_bytes();
        let parsed = Command::from_bytes(&bytes).unwrap();

        assert_eq!(parsed.command_id, cmd.command_id);
        assert_eq!(parsed.command_type, cmd.command_type);
        assert!((parsed.takeoff_altitude.unwrap() - 10.0).abs() < 0.001);
    }

    #[test]
    fn test_command_arm() {
        let bytes = [MSG_TYPE_COMMAND, 1, 0, 0, 0, 0]; // command_id=1, type=Arm
        let cmd = Command::from_bytes(&bytes).unwrap();
        assert_eq!(cmd.command_id, 1);
        assert_eq!(cmd.command_type, CommandType::Arm);
    }

    #[test]
    fn test_command_set_mode() {
        let bytes = [MSG_TYPE_COMMAND, 5, 0, 0, 0, 5, 7]; // command_id=5, type=SetMode, mode=7
        let cmd = Command::from_bytes(&bytes).unwrap();
        assert_eq!(cmd.command_id, 5);
        assert_eq!(cmd.command_type, CommandType::SetMode);
        assert_eq!(cmd.set_mode_value, Some(7));
    }

    #[test]
    fn test_incoming_message_handshake() {
        let bytes = [MSG_TYPE_HANDSHAKE];
        let msg = IncomingMessage::from_bytes(&bytes).unwrap();
        assert!(matches!(msg, IncomingMessage::Handshake));
    }

    #[test]
    fn test_invalid_message_type() {
        let bytes = [0xFF];
        let result = IncomingMessage::from_bytes(&bytes);
        assert!(matches!(
            result,
            Err(ProtocolError::UnknownMessageType(0xFF))
        ));
    }

    #[test]
    fn test_command_ack_success() {
        let ack = CommandAck {
            command_id: 42,
            success: true,
            error: None,
        };
        let bytes = ack.to_bytes();
        assert_eq!(bytes[0], MSG_TYPE_COMMAND_ACK);
        assert_eq!(u32::from_le_bytes(bytes[1..5].try_into().unwrap()), 42);
        assert_eq!(bytes[5], 1);
    }

    #[test]
    fn test_command_ack_failure() {
        let ack = CommandAck {
            command_id: 42,
            success: false,
            error: Some("Rate limited".to_string()),
        };
        let bytes = ack.to_bytes();
        assert_eq!(bytes[0], MSG_TYPE_COMMAND_ACK);
        assert_eq!(bytes[5], 0);
        assert_eq!(&bytes[6..18], b"Rate limited");
        assert_eq!(bytes[18], 0);
    }
}

#[cfg(test)]
mod snapshot_protocol_tests {
    use super::*;

    fn params() -> Vec<SnapshotParam> {
        vec![
            SnapshotParam {
                name: "SYS_HITL".to_string(),
                value: 0.0,
                param_type: "int32".to_string(),
            },
            SnapshotParam {
                name: "EKF2_REQ_HDRIFT".to_string(),
                value: 0.3,
                param_type: "real32".to_string(),
            },
        ]
    }

    #[test]
    fn captured_snapshot_encodes_with_its_message_type() {
        let msg = SnapshotCaptured {
            board_identity: "uid:3034510f33323831".to_string(),
            params: params(),
        };
        let bytes = msg.to_bytes();
        assert_eq!(bytes[0], MSG_TYPE_SNAPSHOT_CAPTURED);

        let decoded: serde_json::Value = serde_json::from_slice(&bytes[1..]).unwrap();
        assert_eq!(decoded["board_identity"], "uid:3034510f33323831");
        // The type has to survive the wire: PX4 drops a type-mismatched
        // PARAM_SET, so restore cannot reconstruct it from the value.
        assert_eq!(decoded["params"][0]["param_type"], "int32");
        assert_eq!(decoded["params"][1]["param_type"], "real32");
    }

    #[test]
    fn stored_ack_round_trips() {
        let json = serde_json::json!({
            "board_identity": "uid:3034510f33323831",
            "stored": true,
        });
        let mut bytes = vec![MSG_TYPE_SNAPSHOT_STORED];
        bytes.extend_from_slice(&serde_json::to_vec(&json).unwrap());

        let decoded = SnapshotStored::from_bytes(&bytes).expect("valid ack");
        assert!(decoded.stored);
        assert_eq!(decoded.board_identity, "uid:3034510f33323831");
        assert_eq!(decoded.error, None);
    }

    #[test]
    fn stored_ack_carries_a_persistence_failure() {
        // The browser could not persist (quota, storage disabled). Provisioning
        // must be able to tell this apart from success and abort.
        let json = serde_json::json!({
            "board_identity": "uid:abc",
            "stored": false,
            "error": "QuotaExceededError",
        });
        let mut bytes = vec![MSG_TYPE_SNAPSHOT_STORED];
        bytes.extend_from_slice(&serde_json::to_vec(&json).unwrap());

        let decoded = SnapshotStored::from_bytes(&bytes).expect("valid ack");
        assert!(!decoded.stored);
        assert_eq!(decoded.error.as_deref(), Some("QuotaExceededError"));
    }

    #[test]
    fn malformed_ack_payload_is_rejected() {
        let mut bytes = vec![MSG_TYPE_SNAPSHOT_STORED];
        bytes.extend_from_slice(b"{not json");
        assert!(SnapshotStored::from_bytes(&bytes).is_err());
    }

    #[test]
    fn ack_missing_required_fields_is_rejected() {
        // "stored" is the whole point of the message; defaulting it to false
        // would turn a malformed ack into a silent provisioning abort, and
        // defaulting it to true would be catastrophic.
        let mut bytes = vec![MSG_TYPE_SNAPSHOT_STORED];
        bytes.extend_from_slice(br#"{"board_identity":"uid:abc"}"#);
        assert!(SnapshotStored::from_bytes(&bytes).is_err());
    }

    #[test]
    fn ack_with_wrong_message_type_is_rejected() {
        let mut bytes = vec![MSG_TYPE_CONFIGURE_BUILD];
        bytes.extend_from_slice(br#"{"board_identity":"uid:abc","stored":true}"#);
        assert!(matches!(
            SnapshotStored::from_bytes(&bytes),
            Err(ProtocolError::UnknownMessageType(_))
        ));
    }

    #[test]
    fn empty_ack_is_rejected() {
        assert!(SnapshotStored::from_bytes(&[]).is_err());
        assert!(SnapshotStored::from_bytes(&[MSG_TYPE_SNAPSHOT_STORED]).is_err());
    }

    #[test]
    fn incoming_dispatch_routes_the_ack() {
        let mut bytes = vec![MSG_TYPE_SNAPSHOT_STORED];
        bytes.extend_from_slice(br#"{"board_identity":"uid:abc","stored":true}"#);
        match IncomingMessage::from_bytes(&bytes).expect("dispatches") {
            IncomingMessage::SnapshotStored(ack) => assert!(ack.stored),
            other => panic!("expected SnapshotStored, got {other:?}"),
        }
    }
}

#[cfg(test)]
mod link_state_tests {
    use super::*;

    fn status(link_state: LinkState) -> ConnectionStatus {
        ConnectionStatus {
            connected: matches!(link_state, LinkState::Connected),
            reconnecting: matches!(link_state, LinkState::Reconnecting),
            retry_count: 0,
            serial_port: String::new(),
            fc_model: None,
            bootloader_suspected: matches!(link_state, LinkState::SuspectedBootloader),
            link_state,
        }
    }

    #[test]
    fn a_first_scan_is_distinguishable_from_a_reconnect() {
        // Both report connected: false, reconnecting: true on the wire. Without
        // link_state a first-time user is told their board is "reconnecting" to
        // something it was never connected to.
        assert_ne!(
            status(LinkState::Searching).link_state,
            status(LinkState::Reconnecting).link_state
        );
    }

    #[test]
    fn a_silent_board_is_distinguishable_from_an_absent_one() {
        // Present but quiet means firmware flashing or a bootloader dwell, and
        // the remedy ("wait, do not unplug") is the opposite of the remedy for
        // an absent board ("plug it in").
        assert_ne!(
            status(LinkState::SuspectedBootloader).link_state,
            status(LinkState::Searching).link_state
        );
    }

    #[test]
    fn every_state_serialises_to_the_literal_the_frontend_matches_on() {
        let render = |s: LinkState| serde_json::to_string(&s).unwrap();
        assert_eq!(render(LinkState::Searching), "\"searching\"");
        assert_eq!(render(LinkState::Connected), "\"connected\"");
        assert_eq!(render(LinkState::Reconnecting), "\"reconnecting\"");
        assert_eq!(
            render(LinkState::SuspectedBootloader),
            "\"suspected_bootloader\""
        );
    }
}

#[cfg(test)]
mod capabilities_tests {
    use super::*;

    #[test]
    fn capabilities_encode_with_their_message_type() {
        let caps = Capabilities::current("0.14.0");
        let bytes = caps.to_bytes();
        assert_eq!(bytes[0], MSG_TYPE_CAPABILITIES);

        let body: serde_json::Value = serde_json::from_slice(&bytes[1..]).unwrap();
        assert_eq!(body["daemon_version"], "0.14.0");
        assert_eq!(body["protocol_revision"], PROTOCOL_REVISION);
    }

    #[test]
    fn every_feature_this_change_added_is_advertised() {
        // A client gates on these names, so a feature that ships without its
        // name here is invisible and will never be used.
        let caps = Capabilities::current("0.14.0");
        for expected in [
            "param_snapshot",
            "param_restore",
            "board_identity",
            "heartbeat_probe",
            "provisioning_broadcast",
            "link_state",
        ] {
            assert!(
                caps.features.iter().any(|f| f == expected),
                "{expected} is implemented but not advertised"
            );
        }
    }

    #[test]
    fn feature_names_are_stable_identifiers() {
        // Clients match these literally; renaming one silently disables the
        // feature for every client that has shipped.
        for feature in DAEMON_FEATURES {
            assert!(
                feature.chars().all(|c| c.is_ascii_lowercase() || c == '_'),
                "{feature} is not a stable snake_case identifier"
            );
        }
    }
}

#[cfg(test)]
mod preflight_identity_tests {
    use super::*;

    #[test]
    fn a_board_that_has_not_identified_itself_omits_the_field() {
        // Absent rather than an empty string: the browser distinguishes "no
        // identity yet" from a real one, and an empty string would be treated
        // as a board that could be approved.
        let status = PreflightStatus {
            connected: false,
            hitl_enabled: false,
            is_quadrotor: false,
            readiness: PreflightReadiness::Unknown,
            board_identity: None,
        };
        let body: serde_json::Value = serde_json::from_slice(&status.to_bytes()[1..]).unwrap();
        assert!(body.get("board_identity").is_none());
    }
}

#[cfg(test)]
mod flight_location_tests {
    use super::*;

    /// Minimal valid ConfigureBuild JSON, with `extra` spliced in.
    fn configure_build_json(extra: &str) -> Vec<u8> {
        let json = format!(
            r#"{{"motor_slug":"m","prop_diameter_inches":5.0,"frame_weight_g":250.0{extra}}}"#
        );
        let mut bytes = vec![MSG_TYPE_CONFIGURE_BUILD];
        bytes.extend_from_slice(json.as_bytes());
        bytes
    }

    fn parse(extra: &str) -> ConfigureBuild {
        ConfigureBuild::from_bytes(&configure_build_json(extra)).expect("parses")
    }

    #[test]
    fn a_chosen_location_is_the_one_used() {
        let cfg = parse(r#","flight_location":{"lat":51.5,"lon":-0.12}"#);
        let loc = cfg.resolve_flight_location().expect("valid");
        assert_eq!(loc.lat, 51.5);
        assert_eq!(loc.lon, -0.12);
    }

    #[test]
    fn an_absent_location_falls_back_to_the_default_rather_than_failing() {
        // A browser that predates location selection must degrade, not break.
        let cfg = parse("");
        assert!(cfg.flight_location.is_none());
        assert_eq!(
            cfg.resolve_flight_location().expect("defaults"),
            FlightLocation::DEFAULT
        );
    }

    #[test]
    fn an_out_of_range_latitude_fails_and_names_the_field() {
        let cfg = parse(r#","flight_location":{"lat":91.0,"lon":0.0}"#);
        let err = cfg.resolve_flight_location().expect_err("rejected");
        assert!(err.contains("lat"), "error should name the field: {err}");
    }

    #[test]
    fn an_out_of_range_longitude_fails_and_names_the_field() {
        let cfg = parse(r#","flight_location":{"lat":0.0,"lon":-180.5}"#);
        let err = cfg.resolve_flight_location().expect_err("rejected");
        assert!(err.contains("lon"), "error should name the field: {err}");
    }

    #[test]
    fn the_range_boundaries_are_inclusive() {
        for (lat, lon) in [(90.0, 180.0), (-90.0, -180.0)] {
            let cfg = parse(&format!(
                r#","flight_location":{{"lat":{lat},"lon":{lon}}}"#
            ));
            assert!(
                cfg.resolve_flight_location().is_ok(),
                "({lat}, {lon}) is a real point on Earth"
            );
        }
    }

    #[test]
    fn non_finite_coordinates_are_rejected() {
        // NaN fails every comparison, so a range check alone would admit it and
        // it would then poison every tile coord and sensor reading downstream.
        // JSON has no NaN literal, so this is constructed directly.
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(
                FlightLocation { lat: bad, lon: 0.0 }.validate().is_err(),
                "lat {bad} must be rejected"
            );
            assert!(
                FlightLocation { lat: 0.0, lon: bad }.validate().is_err(),
                "lon {bad} must be rejected"
            );
        }
    }

    #[test]
    fn origin_elevation_is_optional_and_distinguishes_unknown_from_zero() {
        // Sea level is a real datum; "no data covered the origin" is not the
        // same answer and must not collapse into it.
        assert_eq!(parse("").origin_elevation_msl, None);
        assert_eq!(
            parse(r#","origin_elevation_msl":0.0"#).origin_elevation_msl,
            Some(0.0)
        );
        assert_eq!(
            parse(r#","origin_elevation_msl":1655.0"#).origin_elevation_msl,
            Some(1655.0)
        );
    }
}

#[cfg(test)]
mod terrain_transport_tests {
    use super::*;
    /// The exact bytes of a known frame, shared verbatim with the browser
    /// client's test (`apps/web/lib/hitl/__tests__/terrain-transport.test.ts`).
    ///
    /// Both encoders are pinned to this one literal, so a change on either side
    /// that the other does not match fails here rather than in a live session,
    /// where the symptom would be a drone colliding with terrain nobody can see.
    const SHARED_WIRE_FIXTURE_HEX: &str = "186e0000007b226f726967696e223a7b226c6174223a34302e302c226c6f6e223a2d3130352e32377d2c2274696c655f73697a65223a322c22636f6f726473223a5b7b2278223a333430312c2279223a363230322c227a223a31347d5d2c22617070726f78696d617465223a5b747275655d7d0000c03f000010c00000964300000000";

    fn shared_fixture() -> TerrainTiles {
        TerrainTiles {
            header: TerrainTilesHeader {
                origin: FlightLocation {
                    lat: 40.0,
                    lon: -105.27,
                },
                tile_size: 2,
                coords: vec![WireTileCoord {
                    x: 3401,
                    y: 6202,
                    z: 14,
                }],
                approximate: vec![true],
            },
            tiles: vec![vec![1.5f32, -2.25, 300.0, 0.0]],
        }
    }

    fn fixture_bytes() -> Vec<u8> {
        (0..SHARED_WIRE_FIXTURE_HEX.len() / 2)
            .map(|i| u8::from_str_radix(&SHARED_WIRE_FIXTURE_HEX[i * 2..i * 2 + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn the_wire_format_matches_the_shared_fixture() {
        let hex: String = shared_fixture()
            .to_bytes()
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        assert_eq!(hex, SHARED_WIRE_FIXTURE_HEX);
    }

    #[test]
    fn the_shared_fixture_decodes_back_to_its_values() {
        let decoded = TerrainTiles::from_bytes(&fixture_bytes()).expect("decodes");
        assert_eq!(decoded, shared_fixture());
    }

    const T: usize = 4; // tiny tile edge; the format does not care about 256

    fn coord(x: u32, y: u32) -> WireTileCoord {
        WireTileCoord { x, y, z: 14 }
    }

    fn frame(coords: Vec<WireTileCoord>) -> TerrainTiles {
        let tiles = coords
            .iter()
            .enumerate()
            .map(|(i, _)| (0..T * T).map(|s| (i * 100 + s) as f32).collect())
            .collect();
        TerrainTiles {
            header: TerrainTilesHeader {
                origin: FlightLocation::DEFAULT,
                tile_size: T as u32,
                approximate: vec![false; coords.len()],
                coords,
            },
            tiles,
        }
    }

    #[test]
    fn the_new_message_bytes_collide_with_nothing() {
        let used = [
            MSG_TYPE_STATE_UPDATE,
            MSG_TYPE_HANDSHAKE_ACK,
            MSG_TYPE_COMMAND_ACK,
            MSG_TYPE_NSH_RESPONSE,
            MSG_TYPE_CONNECTION_STATUS,
            MSG_TYPE_VEHICLE_MESSAGE,
            MSG_TYPE_SHUTDOWN,
            MSG_TYPE_CONFIG_RESULT,
            MSG_TYPE_TERRAIN_ORIGIN,
            MSG_TYPE_PREFLIGHT_STATUS,
            MSG_TYPE_PREFLIGHT_APPLY_RESULT,
            MSG_TYPE_SNAPSHOT_CAPTURED,
            MSG_TYPE_RESTORE_RESULT,
            MSG_TYPE_CAPABILITIES,
            MSG_TYPE_COMMAND,
            MSG_TYPE_HANDSHAKE,
            MSG_TYPE_NSH_COMMAND,
            MSG_TYPE_CONFIGURE_BUILD,
            MSG_TYPE_REQUEST_PREFLIGHT_CHECK,
            MSG_TYPE_APPLY_PREFLIGHT_PARAMS,
            MSG_TYPE_SNAPSHOT_STORED,
            MSG_TYPE_RESTORE_SNAPSHOT,
        ];
        for existing in used {
            assert_ne!(MSG_TYPE_TERRAIN_NEED, existing, "0x0F already taken");
            assert_ne!(MSG_TYPE_TERRAIN_TILES, existing, "0x18 already taken");
        }
        assert_ne!(MSG_TYPE_TERRAIN_NEED, MSG_TYPE_TERRAIN_TILES);
    }

    #[test]
    fn terrain_need_round_trips_through_its_tag() {
        let need = TerrainNeed {
            coords: vec![coord(3401, 6202), coord(3402, 6202)],
        };
        let bytes = need.to_bytes();
        assert_eq!(bytes[0], MSG_TYPE_TERRAIN_NEED);
        let decoded: TerrainNeed = serde_json::from_slice(&bytes[1..]).unwrap();
        assert_eq!(decoded.coords, need.coords);
    }

    #[test]
    fn an_empty_need_is_representable() {
        // The steady state. It is not normally sent, but must not be malformed.
        let bytes = TerrainNeed { coords: vec![] }.to_bytes();
        let decoded: TerrainNeed = serde_json::from_slice(&bytes[1..]).unwrap();
        assert!(decoded.coords.is_empty());
    }

    #[test]
    fn terrain_tiles_round_trip_exactly() {
        let original = frame(vec![coord(3401, 6202), coord(3402, 6202)]);
        let decoded = TerrainTiles::from_bytes(&original.to_bytes()).expect("round-trips");
        assert_eq!(decoded.header, original.header);
        assert_eq!(decoded.tiles, original.tiles);
        // Heights survive bit-exact: these are the numbers the physics collides
        // against and the viewer draws, so any lossy step here is a divergence.
        assert_eq!(decoded.tiles[1][5], 105.0);
    }

    /// The padded form a browser sends once it can use `Float32Array.set`:
    /// the payload starts at the next 4-byte boundary after the header.
    fn to_bytes_padded(f: &TerrainTiles) -> Vec<u8> {
        let header = serde_json::to_vec(&f.header).unwrap();
        let mut buf = vec![MSG_TYPE_TERRAIN_TILES];
        buf.extend_from_slice(&(header.len() as u32).to_le_bytes());
        buf.extend_from_slice(&header);
        let pad = (4 - (buf.len() % 4)) % 4;
        buf.extend(std::iter::repeat(0u8).take(pad));
        for tile in &f.tiles {
            for h in tile {
                buf.extend_from_slice(&h.to_le_bytes());
            }
        }
        buf
    }

    #[test]
    fn a_padded_payload_decodes_to_the_same_tiles() {
        // The change this exists to make deployable. A browser cannot use
        // `Float32Array.set` unless the payload begins on a 4-byte boundary, and
        // without it pays a `DataView.setFloat32` per sample -- 589,824 of them
        // for one nine-tile push.
        let original = frame(vec![coord(3401, 6202), coord(3402, 6202)]);
        let decoded = TerrainTiles::from_bytes(&to_bytes_padded(&original)).expect("decodes");
        assert_eq!(decoded.header, original.header);
        assert_eq!(decoded.tiles, original.tiles);
        assert_eq!(decoded.tiles[1][5], 105.0, "heights survive bit-exact");
    }

    #[test]
    fn the_unpadded_form_still_decodes() {
        // Consumer-first ordering: this daemon has to keep reading what every
        // already-installed browser sends, or the release takes terrain away
        // from anyone who has not updated both halves at once.
        let original = frame(vec![coord(1, 2)]);
        let bytes = original.to_bytes();
        let pad = (4 - (bytes.len() - original.tiles.len() * T * T * 4) % 4) % 4;
        assert_ne!(pad, 0, "fixture must exercise a header that needs padding");

        let decoded = TerrainTiles::from_bytes(&bytes).expect("decodes");
        assert_eq!(decoded.tiles, original.tiles);
    }

    #[test]
    fn a_payload_of_the_wrong_size_is_still_rejected() {
        // Accepting two placements must not become accepting any length: the
        // padded branch is reachable only for 1-3 extra bytes.
        let original = frame(vec![coord(1, 2)]);
        let mut bytes = original.to_bytes();
        bytes.extend_from_slice(&[0u8; 8]);
        assert!(
            TerrainTiles::from_bytes(&bytes).is_err(),
            "eight trailing bytes is not padding"
        );

        let mut short = original.to_bytes();
        short.truncate(short.len() - 4);
        assert!(
            TerrainTiles::from_bytes(&short).is_err(),
            "a short payload is rejected"
        );
    }

    #[test]
    fn padding_bytes_are_not_inspected() {
        // They carry nothing. Refusing a well-formed payload over filler would
        // be rejecting the tiles for the sake of bytes with no meaning.
        let original = frame(vec![coord(7, 8)]);
        let mut bytes = to_bytes_padded(&original);
        let header_len = u32::from_le_bytes([bytes[1], bytes[2], bytes[3], bytes[4]]) as usize;
        let header_end = 5 + header_len;
        let pad = (4 - (header_end % 4)) % 4;
        assert_ne!(pad, 0, "fixture must exercise real padding");
        for i in 0..pad {
            bytes[header_end + i] = 0xAB;
        }
        let decoded = TerrainTiles::from_bytes(&bytes).expect("decodes despite non-zero padding");
        assert_eq!(decoded.tiles, original.tiles);
    }

    #[test]
    fn terrain_tiles_dispatches_from_incoming_message() {
        let bytes = frame(vec![coord(1, 2)]).to_bytes();
        match IncomingMessage::from_bytes(&bytes).expect("dispatches") {
            IncomingMessage::TerrainTiles(t) => {
                assert_eq!(t.header.coords, vec![coord(1, 2)]);
                assert_eq!(t.tiles.len(), 1);
            }
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn a_payload_that_does_not_match_the_declared_tile_size_is_rejected() {
        let mut bytes = frame(vec![coord(1, 2)]).to_bytes();
        bytes.truncate(bytes.len() - 4); // drop one sample
        let err = TerrainTiles::from_bytes(&bytes).expect_err("rejected");
        assert!(
            format!("{err}").contains("payload bytes"),
            "should name the mismatch: {err}"
        );
    }

    #[test]
    fn a_truncated_header_is_rejected_rather_than_read_as_heights() {
        let bytes = frame(vec![coord(1, 2)]).to_bytes();
        for cut in [1usize, 4, 6] {
            assert!(
                TerrainTiles::from_bytes(&bytes[..cut]).is_err(),
                "a {cut}-byte frame must not parse"
            );
        }
    }

    #[test]
    fn a_zero_tile_size_is_rejected_before_it_divides_anything() {
        let mut f = frame(vec![coord(1, 2)]);
        f.header.tile_size = 0;
        f.tiles = vec![vec![]];
        let err = TerrainTiles::from_bytes(&f.to_bytes()).expect_err("rejected");
        assert!(format!("{err}").contains("tile_size"), "{err}");
    }

    #[test]
    fn mismatched_approximate_flags_are_rejected() {
        // A positional array that does not line up would mislabel provenance,
        // and provenance drives the "approximate terrain" warning.
        let mut f = frame(vec![coord(1, 2), coord(2, 2)]);
        f.header.approximate = vec![true];
        let err = TerrainTiles::from_bytes(&f.to_bytes()).expect_err("rejected");
        assert!(format!("{err}").contains("approximate"), "{err}");
    }

    #[test]
    fn an_oversized_frame_is_refused_without_allocating_it() {
        let bytes = vec![MSG_TYPE_TERRAIN_TILES; TerrainTiles::MAX_FRAME_BYTES + 1];
        let err = TerrainTiles::from_bytes(&bytes).expect_err("rejected");
        assert!(format!("{err}").contains("exceeds"), "{err}");
    }

    #[test]
    fn a_wrong_tag_byte_is_refused() {
        let mut bytes = frame(vec![coord(1, 2)]).to_bytes();
        bytes[0] = MSG_TYPE_COMMAND;
        assert!(TerrainTiles::from_bytes(&bytes).is_err());
    }
}
