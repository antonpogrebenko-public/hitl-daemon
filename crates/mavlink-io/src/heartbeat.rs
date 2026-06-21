//! HEARTBEAT management and connection state

use mavlink::ardupilotmega::{
    MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, HEARTBEAT_DATA,
};
use std::time::{Duration, Instant};
use tracing::{debug, info, warn};

/// Heartbeat timeout duration (5 seconds)
const HEARTBEAT_TIMEOUT: Duration = Duration::from_secs(5);

/// Connection state with the flight controller
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    /// No heartbeat received, not connected
    Disconnected,
    /// Heartbeat received, connected but not armed
    Connected,
    /// Heartbeat received and vehicle is armed
    Armed,
}

impl std::fmt::Display for ConnectionState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConnectionState::Disconnected => write!(f, "Disconnected"),
            ConnectionState::Connected => write!(f, "Connected"),
            ConnectionState::Armed => write!(f, "Armed"),
        }
    }
}

/// Manages heartbeat state and connection detection
pub struct HeartbeatManager {
    /// Current connection state
    state: ConnectionState,
    /// Last time a heartbeat was received
    last_heartbeat: Option<Instant>,
    /// Remote system ID (from flight controller)
    remote_system_id: Option<u8>,
    /// Remote component ID (from flight controller)
    remote_component_id: Option<u8>,
    /// Last received base mode flags
    last_base_mode: Option<MavModeFlag>,
}

impl HeartbeatManager {
    /// Create a new heartbeat manager
    pub fn new() -> Self {
        Self {
            state: ConnectionState::Disconnected,
            last_heartbeat: None,
            remote_system_id: None,
            remote_component_id: None,
            last_base_mode: None,
        }
    }

    /// Get the current connection state
    pub fn state(&self) -> ConnectionState {
        self.state
    }

    /// Get the remote system ID if known
    pub fn remote_system_id(&self) -> Option<u8> {
        self.remote_system_id
    }

    /// Get the remote component ID if known
    pub fn remote_component_id(&self) -> Option<u8> {
        self.remote_component_id
    }

    /// Check if we're connected (received a recent heartbeat)
    pub fn is_connected(&self) -> bool {
        matches!(
            self.state,
            ConnectionState::Connected | ConnectionState::Armed
        )
    }

    /// Check if the vehicle is armed
    pub fn is_armed(&self) -> bool {
        matches!(self.state, ConnectionState::Armed)
    }

    /// Process a received heartbeat message
    pub fn on_heartbeat_received(
        &mut self,
        system_id: u8,
        component_id: u8,
        heartbeat: &HEARTBEAT_DATA,
    ) {
        let now = Instant::now();
        let was_disconnected = self.state == ConnectionState::Disconnected;

        self.last_heartbeat = Some(now);
        self.remote_system_id = Some(system_id);
        self.remote_component_id = Some(component_id);
        self.last_base_mode = Some(heartbeat.base_mode);

        // Determine new state based on armed flag
        let is_armed = heartbeat
            .base_mode
            .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
        let new_state = if is_armed {
            ConnectionState::Armed
        } else {
            ConnectionState::Connected
        };

        if was_disconnected {
            info!(
                system_id,
                component_id,
                autopilot = ?heartbeat.autopilot,
                mav_type = ?heartbeat.mavtype,
                "Connection established with flight controller"
            );
        }

        if self.state != new_state {
            debug!(
                old_state = %self.state,
                new_state = %new_state,
                "Connection state changed"
            );
            self.state = new_state;
        }
    }

    /// Check for heartbeat timeout and update state accordingly
    ///
    /// Returns true if a timeout was detected (state changed to Disconnected)
    pub fn check_timeout(&mut self) -> bool {
        if let Some(last) = self.last_heartbeat {
            if last.elapsed() > HEARTBEAT_TIMEOUT {
                if self.state != ConnectionState::Disconnected {
                    warn!(
                        timeout_secs = HEARTBEAT_TIMEOUT.as_secs(),
                        "Heartbeat timeout - connection lost"
                    );
                    self.state = ConnectionState::Disconnected;
                    return true;
                }
            }
        }
        false
    }

    /// Create an outgoing HEARTBEAT message for the simulator
    pub fn make_heartbeat() -> MavMessage {
        MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GCS,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        })
    }
}

impl Default for HeartbeatManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_heartbeat(armed: bool) -> HEARTBEAT_DATA {
        let base_mode = if armed {
            MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED
        } else {
            MavModeFlag::empty()
        };

        HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode,
            system_status: MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        }
    }

    #[test]
    fn test_initial_state() {
        let manager = HeartbeatManager::new();
        assert_eq!(manager.state(), ConnectionState::Disconnected);
        assert!(!manager.is_connected());
        assert!(!manager.is_armed());
    }

    #[test]
    fn test_heartbeat_connects() {
        let mut manager = HeartbeatManager::new();
        let heartbeat = make_test_heartbeat(false);

        manager.on_heartbeat_received(1, 1, &heartbeat);

        assert_eq!(manager.state(), ConnectionState::Connected);
        assert!(manager.is_connected());
        assert!(!manager.is_armed());
        assert_eq!(manager.remote_system_id(), Some(1));
    }

    #[test]
    fn test_armed_state() {
        let mut manager = HeartbeatManager::new();
        let heartbeat = make_test_heartbeat(true);

        manager.on_heartbeat_received(1, 1, &heartbeat);

        assert_eq!(manager.state(), ConnectionState::Armed);
        assert!(manager.is_connected());
        assert!(manager.is_armed());
    }

    #[test]
    fn test_make_heartbeat() {
        let msg = HeartbeatManager::make_heartbeat();
        match msg {
            MavMessage::HEARTBEAT(hb) => {
                assert_eq!(hb.mavtype, MavType::MAV_TYPE_GCS);
                assert_eq!(hb.mavlink_version, 3);
            }
            _ => panic!("Expected HEARTBEAT message"),
        }
    }
}
