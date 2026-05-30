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
//! - Total: 86 bytes
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
pub const MSG_TYPE_COMMAND: u8 = 0x10;
pub const MSG_TYPE_HANDSHAKE: u8 = 0x11;
pub const MSG_TYPE_NSH_COMMAND: u8 = 0x12;
pub const MSG_TYPE_CONFIGURE_BUILD: u8 = 0x13;

// State update size
pub const STATE_UPDATE_SIZE: usize = 86;

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
}

impl StateUpdate {
    /// Serialize to binary format (86 bytes)
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

        buf
    }

    /// Deserialize from binary format
    pub fn from_bytes(data: &[u8]) -> Result<Self, ProtocolError> {
        if data.len() < STATE_UPDATE_SIZE {
            return Err(ProtocolError::MessageTooShort {
                expected: STATE_UPDATE_SIZE,
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
}

impl ConnectionStatus {
    /// Serialize to binary format
    pub fn to_bytes(&self) -> Vec<u8> {
        let fc_model_str = self.fc_model.as_deref().unwrap_or("");
        let mut buf = Vec::with_capacity(5 + self.serial_port.len() + 1 + fc_model_str.len() + 1 + 1);
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
}

fn default_esc_count() -> u8 { 1 }

fn default_battery_voltage() -> f64 { 14.8 }
fn default_battery_capacity_mah() -> f64 { 1500.0 }
fn default_battery_cell_count() -> u8 { 4 }

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

        let json_str = std::str::from_utf8(&data[1..])
            .map_err(|_| ProtocolError::InvalidPayload {
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

#[derive(Debug, Clone, Serialize)]
pub struct ConfigResult {
    pub state: ConfigState,
    pub success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<AppliedConfig>,
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
            MSG_TYPE_CONFIGURE_BUILD => Ok(IncomingMessage::ConfigureBuild(ConfigureBuild::from_bytes(data)?)),
            MSG_TYPE_SHUTDOWN => Ok(IncomingMessage::Shutdown),
            other => Err(ProtocolError::UnknownMessageType(other)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        };

        let bytes = state.to_bytes();
        assert_eq!(bytes.len(), STATE_UPDATE_SIZE);
        assert_eq!(bytes[0], MSG_TYPE_STATE_UPDATE);

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
        };
        let bytes = status.to_bytes();
        assert_eq!(bytes[1], 0); // not connected
        assert_eq!(bytes[2], 1); // reconnecting
        // bootloader_suspected byte is at index 6 (after two empty null-terminated strings)
        assert_eq!(bytes[6], 1); // bootloader_suspected = true
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
        assert!(matches!(result, Err(ProtocolError::UnknownMessageType(0xFF))));
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
