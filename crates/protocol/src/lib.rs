//! Shared types between HITL daemon crates

use mavlink::ardupilotmega::{MavMessage, MavModeFlag, HIL_ACTUATOR_CONTROLS_DATA};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Motor channel mapping from PX4 HIL_ACTUATOR_CONTROLS to simulation.
///
/// Simulation motor numbering matches PX4 Standard Quad X directly — no remapping needed:
/// ```text
///     Front
///   3(CW)   1(CCW)
///      \   /
///        X
///      /   \
///   2(CCW)  4(CW)
///     Back
/// ```
/// ch0 → Motor 1 (FR, CCW), ch1 → Motor 2 (BL, CCW)
/// ch2 → Motor 3 (FL, CW),  ch3 → Motor 4 (BR, CW)
pub const PX4_TO_SIM_MOTOR_MAP: [usize; 4] = [0, 1, 2, 3];

/// Flight mode indicators from HIL_ACTUATOR_CONTROLS
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FlightMode {
    /// Motors disarmed
    Disarmed,
    /// Armed but not in HIL mode
    Armed,
    /// Armed and in HIL mode
    HilArmed,
}

impl Default for FlightMode {
    fn default() -> Self {
        FlightMode::Disarmed
    }
}

/// Daemon operational state for TUI and status reporting
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DaemonState {
    Starting,
    WaitingForFc,
    Connected,
    Streaming,
    FcLost,
    Reconnecting,
    ShuttingDown,
}

impl Default for DaemonState {
    fn default() -> Self {
        DaemonState::Starting
    }
}

impl std::fmt::Display for DaemonState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DaemonState::Starting => write!(f, "Starting"),
            DaemonState::WaitingForFc => write!(f, "Waiting for FC"),
            DaemonState::Connected => write!(f, "Connected"),
            DaemonState::Streaming => write!(f, "Streaming"),
            DaemonState::FcLost => write!(f, "FC Lost"),
            DaemonState::Reconnecting => write!(f, "Reconnecting"),
            DaemonState::ShuttingDown => write!(f, "Shutting Down"),
        }
    }
}

/// Daemon status for TUI display and web status widget
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaemonStatus {
    pub state: DaemonState,
    pub fc_model: Option<String>,
    pub serial_port: Option<String>,
    pub packets_per_sec: u16,
    pub connected_clients: u8,
    pub uptime_secs: u64,
}

impl Default for DaemonStatus {
    fn default() -> Self {
        Self {
            state: DaemonState::Starting,
            fc_model: None,
            serial_port: None,
            packets_per_sec: 0,
            connected_clients: 0,
            uptime_secs: 0,
        }
    }
}

/// Actuator outputs from the flight controller
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActuatorOutputs {
    /// Timestamp in microseconds since boot
    pub timestamp_us: u64,
    /// Motor outputs (normalized 0.0 to 1.0)
    pub motors: [f32; 4],
    /// Current flight mode
    pub mode: FlightMode,
    /// Raw controls array (all 16 channels)
    pub controls: [f32; 16],
}

impl Default for ActuatorOutputs {
    fn default() -> Self {
        Self {
            timestamp_us: 0,
            motors: [0.0; 4],
            mode: FlightMode::default(),
            controls: [0.0; 16],
        }
    }
}

#[derive(Debug, Error)]
pub enum ProtocolError {
    #[error("Invalid message type, expected HIL_ACTUATOR_CONTROLS")]
    InvalidMessageType,

    #[error("Invalid actuator value: {value} at index {index}")]
    InvalidActuatorValue { index: usize, value: f32 },
}

impl ActuatorOutputs {
    /// Create ActuatorOutputs from a HIL_ACTUATOR_CONTROLS MAVLink message
    pub fn from_mavlink(msg: &MavMessage) -> Result<Self, ProtocolError> {
        match msg {
            MavMessage::HIL_ACTUATOR_CONTROLS(data) => Self::from_hil_actuator_controls(data),
            _ => Err(ProtocolError::InvalidMessageType),
        }
    }

    /// Create ActuatorOutputs from HIL_ACTUATOR_CONTROLS data
    pub fn from_hil_actuator_controls(data: &HIL_ACTUATOR_CONTROLS_DATA) -> Result<Self, ProtocolError> {
        let mut motors = [0.0f32; 4];
        let mut controls = [0.0f32; 16];

        // Copy all 16 control channels
        for (i, &control) in data.controls.iter().enumerate() {
            controls[i] = control;
        }

        // Extract motor outputs with PX4 → Simulation remapping
        // PX4 sends motor values in [0, 1] range (0 = off, 1 = full throttle)
        for (sim_idx, &px4_idx) in PX4_TO_SIM_MOTOR_MAP.iter().enumerate() {
            motors[sim_idx] = data.controls[px4_idx].clamp(0.0, 1.0);
        }

        // Determine flight mode from mode flags
        let mode = Self::decode_mode(data.mode);

        Ok(Self {
            timestamp_us: data.time_usec,
            motors,
            mode,
            controls,
        })
    }

    /// Decode the mode flags from HIL_ACTUATOR_CONTROLS
    fn decode_mode(mode: MavModeFlag) -> FlightMode {
        let armed = mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
        let hil = mode.contains(MavModeFlag::MAV_MODE_FLAG_HIL_ENABLED);

        match (armed, hil) {
            (true, true) => FlightMode::HilArmed,
            (true, false) => FlightMode::Armed,
            (false, _) => FlightMode::Disarmed,
        }
    }

    /// Check if motors are armed
    pub fn is_armed(&self) -> bool {
        !matches!(self.mode, FlightMode::Disarmed)
    }

    /// Check if in HIL mode
    pub fn is_hil_active(&self) -> bool {
        matches!(self.mode, FlightMode::HilArmed)
    }

    /// Get average motor output
    pub fn average_throttle(&self) -> f32 {
        self.motors.iter().sum::<f32>() / 4.0
    }
}

// WebSocket message type constants — Outgoing
pub const MSG_TYPE_STATE_UPDATE: u8 = 0x01;
pub const MSG_TYPE_HANDSHAKE_ACK: u8 = 0x02;
pub const MSG_TYPE_COMMAND_ACK: u8 = 0x03;
pub const MSG_TYPE_NSH_RESPONSE: u8 = 0x04;
pub const MSG_TYPE_CONNECTION_STATUS: u8 = 0x05;
pub const MSG_TYPE_VEHICLE_MESSAGE: u8 = 0x06;
pub const MSG_TYPE_CONFIG_RESULT: u8 = 0x08;

// WebSocket message type constants — Incoming
pub const MSG_TYPE_HANDSHAKE: u8 = 0x10;
pub const MSG_TYPE_COMMAND: u8 = 0x11;
pub const MSG_TYPE_NSH_COMMAND: u8 = 0x12;
pub const MSG_TYPE_CONFIGURE_BUILD: u8 = 0x13;
pub const MSG_TYPE_SHUTDOWN: u8 = 0x14;

#[derive(Debug, Error)]
pub enum WsProtocolError {
    #[error("Unknown message type: {0}")]
    UnknownMessageType(u8),

    #[error("Invalid payload: {0}")]
    InvalidPayload(String),

    #[error("Empty message")]
    EmptyMessage,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ConfigureBuild {
    pub motor_slug: String,
    pub prop_diameter_inches: f64,
    pub frame_weight_g: f64,
    #[serde(default)]
    pub prop_slug: Option<String>,
    #[serde(default = "default_battery_voltage")]
    pub battery_voltage: f64,
    #[serde(default = "default_battery_capacity_mah")]
    pub battery_capacity_mah: f64,
    #[serde(default = "default_battery_cell_count")]
    pub battery_cell_count: u8,
}

fn default_battery_voltage() -> f64 { 14.8 }
fn default_battery_capacity_mah() -> f64 { 1500.0 }
fn default_battery_cell_count() -> u8 { 4 }

#[derive(Debug, Clone, Serialize)]
pub struct ConfigResult {
    pub success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<AppliedConfig>,
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
}

#[derive(Debug, Clone, Deserialize)]
pub struct Command {
    pub action: String,
    #[serde(default)]
    pub params: serde_json::Value,
}

#[derive(Debug, Clone, Deserialize)]
pub struct NshCommand {
    pub command: String,
}

#[derive(Debug, Clone)]
pub enum IncomingMessage {
    Handshake,
    Command(Command),
    NshCommand(NshCommand),
    ConfigureBuild(ConfigureBuild),
    Shutdown,
}

impl IncomingMessage {
    pub fn from_bytes(data: &[u8]) -> Result<Self, WsProtocolError> {
        if data.is_empty() {
            return Err(WsProtocolError::EmptyMessage);
        }

        let msg_type = data[0];
        match msg_type {
            MSG_TYPE_HANDSHAKE => Ok(IncomingMessage::Handshake),
            MSG_TYPE_COMMAND => {
                let json_str = std::str::from_utf8(&data[1..])
                    .map_err(|_| WsProtocolError::InvalidPayload("Command: invalid UTF-8".into()))?;
                let cmd: Command = serde_json::from_str(json_str)
                    .map_err(|e| WsProtocolError::InvalidPayload(format!("Command: {e}")))?;
                Ok(IncomingMessage::Command(cmd))
            }
            MSG_TYPE_NSH_COMMAND => {
                let json_str = std::str::from_utf8(&data[1..])
                    .map_err(|_| WsProtocolError::InvalidPayload("NshCommand: invalid UTF-8".into()))?;
                let cmd: NshCommand = serde_json::from_str(json_str)
                    .map_err(|e| WsProtocolError::InvalidPayload(format!("NshCommand: {e}")))?;
                Ok(IncomingMessage::NshCommand(cmd))
            }
            MSG_TYPE_CONFIGURE_BUILD => {
                let json_str = std::str::from_utf8(&data[1..])
                    .map_err(|_| WsProtocolError::InvalidPayload("ConfigureBuild: invalid UTF-8".into()))?;
                let build: ConfigureBuild = serde_json::from_str(json_str)
                    .map_err(|e| WsProtocolError::InvalidPayload(format!("ConfigureBuild: {e}")))?;
                Ok(IncomingMessage::ConfigureBuild(build))
            }
            MSG_TYPE_SHUTDOWN => Ok(IncomingMessage::Shutdown),
            _ => Err(WsProtocolError::UnknownMessageType(msg_type)),
        }
    }
}

#[derive(Debug, Clone)]
pub enum OutgoingMessage {
    StateUpdate(Vec<u8>),
    HandshakeAck,
    CommandAck { success: bool, message: Option<String> },
    NshResponse(String),
    ConnectionStatus { connected: bool },
    VehicleMessage(String),
    ConfigResult(ConfigResult),
}

impl OutgoingMessage {
    pub fn to_bytes(&self) -> Vec<u8> {
        match self {
            OutgoingMessage::StateUpdate(payload) => {
                let mut buf = Vec::with_capacity(1 + payload.len());
                buf.push(MSG_TYPE_STATE_UPDATE);
                buf.extend_from_slice(payload);
                buf
            }
            OutgoingMessage::HandshakeAck => vec![MSG_TYPE_HANDSHAKE_ACK],
            OutgoingMessage::CommandAck { success, message } => {
                let json = serde_json::json!({
                    "success": success,
                    "message": message
                });
                let json_bytes = serde_json::to_vec(&json).expect("CommandAck serialization cannot fail");
                let mut buf = Vec::with_capacity(1 + json_bytes.len());
                buf.push(MSG_TYPE_COMMAND_ACK);
                buf.extend_from_slice(&json_bytes);
                buf
            }
            OutgoingMessage::NshResponse(text) => {
                let mut buf = Vec::with_capacity(1 + text.len());
                buf.push(MSG_TYPE_NSH_RESPONSE);
                buf.extend_from_slice(text.as_bytes());
                buf
            }
            OutgoingMessage::ConnectionStatus { connected } => {
                let json = serde_json::json!({ "connected": connected });
                let json_bytes = serde_json::to_vec(&json).expect("ConnectionStatus serialization cannot fail");
                let mut buf = Vec::with_capacity(1 + json_bytes.len());
                buf.push(MSG_TYPE_CONNECTION_STATUS);
                buf.extend_from_slice(&json_bytes);
                buf
            }
            OutgoingMessage::VehicleMessage(text) => {
                let mut buf = Vec::with_capacity(1 + text.len());
                buf.push(MSG_TYPE_VEHICLE_MESSAGE);
                buf.extend_from_slice(text.as_bytes());
                buf
            }
            OutgoingMessage::ConfigResult(result) => {
                let json = serde_json::to_vec(result).expect("ConfigResult serialization cannot fail");
                let mut buf = Vec::with_capacity(1 + json.len());
                buf.push(MSG_TYPE_CONFIG_RESULT);
                buf.extend_from_slice(&json);
                buf
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::ardupilotmega::MavModeFlag;

    #[test]
    fn test_default_actuator_outputs() {
        let outputs = ActuatorOutputs::default();
        assert_eq!(outputs.timestamp_us, 0);
        assert_eq!(outputs.motors, [0.0; 4]);
        assert_eq!(outputs.mode, FlightMode::Disarmed);
    }

    #[test]
    fn test_from_hil_actuator_controls() {
        let mut controls = [0.0f32; 16];
        // PX4 channels match sim motors 1-4 directly (Standard Quad X)
        controls[0] = 0.1;  // ch0 = Motor 1 (FR, CCW)
        controls[1] = 0.2;  // ch1 = Motor 2 (BL, CCW)
        controls[2] = 0.3;  // ch2 = Motor 3 (FL, CW)
        controls[3] = 0.4;  // ch3 = Motor 4 (BR, CW)

        let data = HIL_ACTUATOR_CONTROLS_DATA {
            time_usec: 1000000,
            controls,
            mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED | MavModeFlag::MAV_MODE_FLAG_HIL_ENABLED,
            flags: 0,
        };

        let outputs = ActuatorOutputs::from_hil_actuator_controls(&data).unwrap();

        assert_eq!(outputs.timestamp_us, 1000000);
        assert_eq!(outputs.mode, FlightMode::HilArmed);
        assert!(outputs.is_armed());
        assert!(outputs.is_hil_active());

        // Identity mapping: sim motors = px4 channels directly
        assert!((outputs.motors[0] - 0.1).abs() < 0.01);  // Motor 1 (FR)
        assert!((outputs.motors[1] - 0.2).abs() < 0.01);  // Motor 2 (BL)
        assert!((outputs.motors[2] - 0.3).abs() < 0.01);  // Motor 3 (FL)
        assert!((outputs.motors[3] - 0.4).abs() < 0.01);  // Motor 4 (BR)
    }

    #[test]
    fn test_decode_mode() {
        assert_eq!(ActuatorOutputs::decode_mode(MavModeFlag::empty()), FlightMode::Disarmed);
        assert_eq!(ActuatorOutputs::decode_mode(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED), FlightMode::Armed);
        assert_eq!(
            ActuatorOutputs::decode_mode(
                MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED | MavModeFlag::MAV_MODE_FLAG_HIL_ENABLED
            ),
            FlightMode::HilArmed
        );
        assert_eq!(
            ActuatorOutputs::decode_mode(MavModeFlag::MAV_MODE_FLAG_HIL_ENABLED),
            FlightMode::Disarmed
        ); // HIL but not armed
    }

    #[test]
    fn test_average_throttle() {
        let mut outputs = ActuatorOutputs::default();
        outputs.motors = [0.25, 0.5, 0.75, 1.0];
        assert!((outputs.average_throttle() - 0.625).abs() < 0.01);
    }

    #[test]
    fn test_invalid_message_type() {
        let msg = MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_GCS,
            autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: mavlink::ardupilotmega::MavModeFlag::empty(),
            system_status: mavlink::ardupilotmega::MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        });

        let result = ActuatorOutputs::from_mavlink(&msg);
        assert!(matches!(result, Err(ProtocolError::InvalidMessageType)));
    }
}
