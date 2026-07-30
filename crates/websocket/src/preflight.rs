//! Preflight HITL-mode / quadrotor-frame gate.
//!
//! Before `ConfigureBuild` runs, the browser checks whether the connected FC
//! reports HITL-enabled and quadrotor type. Detection reuses the HEARTBEAT
//! already flowing into `SimulationState` for flight-mode extraction — no new
//! MAVLink read round-trip. If either flag is false, `PreflightHandler::apply`
//! pushes the correct params, saves, reboots the FC, and waits for it to come
//! back before re-verifying.

use mavlink::ardupilotmega::{MavModeFlag, MavType, HEARTBEAT_DATA};

/// Derive `(hitl_enabled, is_quadrotor)` from a HEARTBEAT. Pure function so
/// the bit-flag logic is testable without a live MAVLink connection.
pub fn heartbeat_hitl_signals(hb: &HEARTBEAT_DATA) -> (bool, bool) {
    let hitl_enabled = hb.base_mode.contains(MavModeFlag::MAV_MODE_FLAG_HIL_ENABLED);
    let is_quadrotor = hb.mavtype == MavType::MAV_TYPE_QUADROTOR;
    (hitl_enabled, is_quadrotor)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::ardupilotmega::{MavAutopilot, MavState};

    fn heartbeat(base_mode: MavModeFlag, mavtype: MavType) -> HEARTBEAT_DATA {
        HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype,
            autopilot: MavAutopilot::MAV_AUTOPILOT_PX4,
            base_mode,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        }
    }

    #[test]
    fn hitl_and_quadrotor_both_true() {
        let hb = heartbeat(MavModeFlag::MAV_MODE_FLAG_HIL_ENABLED, MavType::MAV_TYPE_QUADROTOR);
        assert_eq!(heartbeat_hitl_signals(&hb), (true, true));
    }

    #[test]
    fn hitl_flag_absent_reads_false() {
        let hb = heartbeat(MavModeFlag::empty(), MavType::MAV_TYPE_QUADROTOR);
        assert_eq!(heartbeat_hitl_signals(&hb), (false, true));
    }

    #[test]
    fn non_quadrotor_type_reads_false() {
        let hb = heartbeat(MavModeFlag::MAV_MODE_FLAG_HIL_ENABLED, MavType::MAV_TYPE_FIXED_WING);
        assert_eq!(heartbeat_hitl_signals(&hb), (true, false));
    }

    #[test]
    fn hil_flag_combined_with_other_flags_still_reads_true() {
        // Real PX4 heartbeats set multiple base_mode bits at once (e.g.
        // SAFETY_ARMED | CUSTOM_MODE_ENABLED | HIL_ENABLED). Must not
        // require an exact match, only that the HIL bit is present.
        let flags = MavModeFlag::MAV_MODE_FLAG_HIL_ENABLED
            | MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            | MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        let hb = heartbeat(flags, MavType::MAV_TYPE_QUADROTOR);
        assert_eq!(heartbeat_hitl_signals(&hb), (true, true));
    }
}
