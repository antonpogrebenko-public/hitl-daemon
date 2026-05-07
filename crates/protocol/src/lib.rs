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
