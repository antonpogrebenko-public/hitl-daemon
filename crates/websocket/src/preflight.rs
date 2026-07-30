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

use crate::build_config::{
    make_param_save, make_param_set, wait_for_param_ack, PARAM_RETRY_COUNT, PX4_TARGET_COMPONENT,
    PX4_TARGET_SYSTEM,
};
use crate::protocol::{OutgoingMessage, PreflightApplyResult, PreflightApplyState, PreflightStatus};
use crossbeam_channel::Sender;
use mavlink::ardupilotmega::{MavCmd, MavMessage, COMMAND_LONG_DATA};
use simulation::SimulationState;
use std::time::Duration;
use tokio::sync::{broadcast, mpsc};
use tracing::warn;

/// PX4 SYS_AUTOSTART id for "Generic Quadcopter X" — the fixed target this
/// feature applies. No per-build airframe selection.
const QUADROTOR_AUTOSTART_ID: f32 = 4001.0;

/// Total time budget for the FC to come back after a preflight-triggered
/// reboot. Bounded so a cable/power issue surfaces as an explicit error
/// instead of an indefinite spinner.
const PREFLIGHT_RECONNECT_TIMEOUT: Duration = Duration::from_secs(30);
const PREFLIGHT_RECONNECT_POLL_INTERVAL: Duration = Duration::from_millis(250);

/// Minimum continuous silence (no HEARTBEAT) required after clearing the
/// cache before a subsequent HEARTBEAT is trusted as a genuine post-reboot
/// reconnect rather than a straggler the FC sent before it actually
/// processed the reboot command. PX4 takes 3-5s to clear its bootloader on
/// power-up; a straggler heartbeat lands within about one HEARTBEAT period
/// (~1s) of the reboot command being sent. 2s cleanly separates the two.
const PREFLIGHT_QUIET_PERIOD: Duration = Duration::from_secs(2);

pub struct PreflightHandler {
    mav_tx: Option<Sender<MavMessage>>,
    param_value_tx: Option<broadcast::Sender<(String, f32)>>,
    sim_state: SimulationState,
}

impl PreflightHandler {
    pub fn new(
        mav_tx: Option<Sender<MavMessage>>,
        param_value_tx: Option<broadcast::Sender<(String, f32)>>,
        sim_state: SimulationState,
    ) -> Self {
        Self {
            mav_tx,
            param_value_tx,
            sim_state,
        }
    }

    /// Instant, cache-only read of the FC's last-known HITL/quadrotor status.
    pub async fn check(&self) -> PreflightStatus {
        let (connected, hitl_enabled, is_quadrotor) = self.sim_state.heartbeat_status();
        PreflightStatus {
            connected,
            hitl_enabled,
            is_quadrotor,
        }
    }

    /// Push SYS_HITL=1 + SYS_AUTOSTART=4001, save, reboot the FC, and wait for
    /// it to reconnect before re-verifying. Fail-closed: any ack failure
    /// aborts before the reboot is sent.
    pub async fn apply(&self, progress_tx: mpsc::Sender<OutgoingMessage>) -> PreflightApplyResult {
        let (Some(mav_tx), Some(param_value_tx)) =
            (self.mav_tx.as_ref(), self.param_value_tx.as_ref())
        else {
            return PreflightApplyResult {
                state: PreflightApplyState::Error,
                success: false,
                error: Some("No flight controller connected (sim-only mode)".to_string()),
            };
        };

        send_progress(&progress_tx, PreflightApplyState::Applying).await;

        for (name, value) in [("SYS_HITL", 1.0f32), ("SYS_AUTOSTART", QUADROTOR_AUTOSTART_ID)] {
            let mut acked = false;
            for attempt in 1..=PARAM_RETRY_COUNT {
                let mut rx = param_value_tx.subscribe();
                match mav_tx.try_send(make_param_set(name, value)) {
                    Ok(()) => {}
                    Err(crossbeam_channel::TrySendError::Full(_)) => {
                        warn!(param = name, attempt, "MAVLink tx channel full — retrying PARAM_SET");
                        tokio::time::sleep(Duration::from_millis(50)).await;
                        continue;
                    }
                    Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                        return PreflightApplyResult {
                            state: PreflightApplyState::Error,
                            success: false,
                            error: Some(format!("MAVLink tx disconnected while sending {name}")),
                        };
                    }
                }

                if wait_for_param_ack(&mut rx, name, value).await.is_some() {
                    acked = true;
                    break;
                }
                warn!(param = name, attempt, "Preflight PARAM_VALUE ack timed out — retrying");
            }

            if !acked {
                return PreflightApplyResult {
                    state: PreflightApplyState::Error,
                    success: false,
                    error: Some(format!(
                        "Failed to verify {name} after {PARAM_RETRY_COUNT} retries"
                    )),
                };
            }
        }

        match mav_tx.try_send(make_param_save()) {
            Ok(()) => {}
            Err(e) => warn!(error = ?e, "Failed to send PARAM_SAVE before preflight reboot"),
        }

        send_progress(&progress_tx, PreflightApplyState::Rebooting).await;
        if let Err(e) = mav_tx.try_send(make_reboot_autopilot()) {
            return PreflightApplyResult {
                state: PreflightApplyState::Error,
                success: false,
                error: Some(format!("Failed to send reboot command: {e}")),
            };
        }

        // Invalidate the cache now, before the FC actually drops — otherwise
        // the reconnect wait below would see the stale pre-reboot "seen=true"
        // and return immediately.
        self.sim_state.clear_heartbeat_status();

        send_progress(&progress_tx, PreflightApplyState::Reconnecting).await;
        if !self
            .wait_for_reconnect(
                PREFLIGHT_RECONNECT_TIMEOUT,
                PREFLIGHT_RECONNECT_POLL_INTERVAL,
                PREFLIGHT_QUIET_PERIOD,
            )
            .await
        {
            return PreflightApplyResult {
                state: PreflightApplyState::Error,
                success: false,
                error: Some("FC did not reconnect after reboot".to_string()),
            };
        }

        send_progress(&progress_tx, PreflightApplyState::Verifying).await;
        let (_, hitl_enabled, is_quadrotor) = self.sim_state.heartbeat_status();
        if !hitl_enabled || !is_quadrotor {
            return PreflightApplyResult {
                state: PreflightApplyState::Error,
                success: false,
                error: Some("Settings did not take effect after reboot".to_string()),
            };
        }

        PreflightApplyResult {
            state: PreflightApplyState::Done,
            success: true,
            error: None,
        }
    }

    /// Poll the cached heartbeat status until a HEARTBEAT arrives that is
    /// trustworthy as a genuine post-reboot reconnect, or `timeout` elapses.
    /// A HEARTBEAT is only trusted once `quiet_period` of continuous silence
    /// has been observed first — anything arriving before that is treated
    /// as a stale pre-reboot straggler (cleared, and the quiet-period
    /// countdown restarts), because the physical FC keeps heartbeating with
    /// its old pre-fix state for a while after the reboot command is sent,
    /// well before it has actually rebooted.
    async fn wait_for_reconnect(
        &self,
        timeout: Duration,
        poll_interval: Duration,
        quiet_period: Duration,
    ) -> bool {
        let deadline = tokio::time::Instant::now() + timeout;
        let mut quiet_start = tokio::time::Instant::now();
        let mut confirmed_quiet = false;

        loop {
            if tokio::time::Instant::now() >= deadline {
                return false;
            }

            let (seen, _, _) = self.sim_state.heartbeat_status();
            if seen {
                if confirmed_quiet {
                    // Arrived only after we confirmed real silence — genuine.
                    return true;
                }
                // Straggler during the quiet-confirmation phase. Clear it
                // and restart the countdown; we haven't confirmed the FC
                // actually went quiet yet.
                self.sim_state.clear_heartbeat_status();
                quiet_start = tokio::time::Instant::now();
            } else if !confirmed_quiet && tokio::time::Instant::now() >= quiet_start + quiet_period {
                confirmed_quiet = true;
            }

            tokio::time::sleep(poll_interval).await;
        }
    }
}

async fn send_progress(progress_tx: &mpsc::Sender<OutgoingMessage>, state: PreflightApplyState) {
    let _ = progress_tx
        .send(OutgoingMessage::PreflightApplyResult(PreflightApplyResult {
            state,
            success: true,
            error: None,
        }))
        .await;
}

fn make_reboot_autopilot() -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: PX4_TARGET_SYSTEM,
        target_component: PX4_TARGET_COMPONENT,
        confirmation: 0,
        command: MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        param1: 1.0, // 1 = reboot autopilot
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    })
}

#[cfg(test)]
mod apply_tests {
    use super::*;
    use crossbeam_channel::bounded;
    use simulation::SimulationConfig;
    use std::sync::Mutex;

    fn make_handler(
        mav_tx: Option<Sender<MavMessage>>,
        param_value_tx: Option<broadcast::Sender<(String, f32)>>,
        sim_state: SimulationState,
    ) -> PreflightHandler {
        PreflightHandler::new(mav_tx, param_value_tx, sim_state)
    }

    #[derive(Debug, Clone)]
    enum CapturedMsg {
        ParamSet(String, f32),
        CommandLong(u32),
    }

    fn spawn_fake_px4(
        mav_rx: crossbeam_channel::Receiver<MavMessage>,
        param_value_tx: broadcast::Sender<(String, f32)>,
    ) -> (
        tokio::task::JoinHandle<()>,
        std::sync::Arc<Mutex<Vec<CapturedMsg>>>,
    ) {
        let captured = std::sync::Arc::new(Mutex::new(Vec::<CapturedMsg>::new()));
        let captured_clone = captured.clone();
        let handle = tokio::task::spawn_blocking(move || {
            while let Ok(msg) = mav_rx.recv() {
                match msg {
                    MavMessage::PARAM_SET(p) => {
                        let name = std::str::from_utf8(&p.param_id)
                            .unwrap_or("")
                            .trim_end_matches('\0')
                            .to_string();
                        captured_clone
                            .lock()
                            .unwrap()
                            .push(CapturedMsg::ParamSet(name.clone(), p.param_value));
                        let _ = param_value_tx.send((name, p.param_value));
                    }
                    MavMessage::COMMAND_LONG(c) => {
                        captured_clone
                            .lock()
                            .unwrap()
                            .push(CapturedMsg::CommandLong(c.command as u32));
                    }
                    _ => {}
                }
            }
        });
        (handle, captured)
    }

    #[tokio::test]
    async fn sim_only_mode_returns_error_immediately() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        let handler = make_handler(None, None, sim_state);
        let (progress_tx, _progress_rx) = mpsc::channel(8);
        let result = handler.apply(progress_tx).await;
        assert!(!result.success);
        assert!(matches!(result.state, PreflightApplyState::Error));
        assert!(result.error.unwrap().contains("sim-only"));
    }

    #[tokio::test]
    async fn happy_path_applies_reboots_and_verifies() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        let (_px4, captured) = spawn_fake_px4(mav_rx, pv_tx.clone());

        let sim_state = SimulationState::new(SimulationConfig::default());
        let sim_state_for_reboot = sim_state.clone();
        // Simulate the FC coming back post-reboot: a real HEARTBEAT would
        // call SimulationState::set_heartbeat_status via main.rs; here we do
        // it directly after modeling genuine silence past the real
        // PREFLIGHT_QUIET_PERIOD (2s) — anything sooner would be (correctly)
        // treated as a stale pre-reboot straggler and cleared.
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(2200)).await;
            sim_state_for_reboot.set_heartbeat_status(true, true);
        });

        let handler = make_handler(Some(mav_tx), Some(pv_tx), sim_state);
        let (progress_tx, mut progress_rx) = mpsc::channel(8);
        let apply_task = tokio::spawn(async move { handler.apply(progress_tx).await });

        let mut stages = Vec::new();
        while let Some(OutgoingMessage::PreflightApplyResult(r)) = progress_rx.recv().await {
            stages.push(format!("{:?}", r.state));
            if matches!(r.state, PreflightApplyState::Verifying) {
                break;
            }
        }

        let result = apply_task.await.unwrap();
        assert!(result.success, "apply() failed: {:?}", result.error);
        assert!(matches!(result.state, PreflightApplyState::Done));

        let snapshot = captured.lock().unwrap().clone();
        let param_names: Vec<&str> = snapshot
            .iter()
            .filter_map(|m| match m {
                CapturedMsg::ParamSet(n, _) => Some(n.as_str()),
                _ => None,
            })
            .collect();
        assert!(param_names.contains(&"SYS_HITL"));
        assert!(param_names.contains(&"SYS_AUTOSTART"));

        const MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: u32 = 246;
        assert!(snapshot
            .iter()
            .any(|m| matches!(m, CapturedMsg::CommandLong(c) if *c == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)));
    }

    #[tokio::test]
    async fn ack_failure_returns_error_without_reboot() {
        let (mav_tx, _mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        // No fake PX4 draining mav_rx — every PARAM_SET times out.
        let sim_state = SimulationState::new(SimulationConfig::default());
        let handler = make_handler(Some(mav_tx), Some(pv_tx), sim_state);
        let (progress_tx, _progress_rx) = mpsc::channel(8);

        let result = handler.apply(progress_tx).await;
        assert!(!result.success);
        assert!(matches!(result.state, PreflightApplyState::Error));
        assert!(result.error.unwrap().contains("SYS_HITL"));
    }

    #[tokio::test]
    async fn wait_for_reconnect_times_out_when_heartbeat_never_seen() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        let handler = make_handler(None, None, sim_state);
        let ok = handler
            .wait_for_reconnect(
                Duration::from_millis(100),
                Duration::from_millis(20),
                Duration::from_millis(30),
            )
            .await;
        assert!(!ok);
    }

    #[tokio::test]
    async fn wait_for_reconnect_rejects_straggler_before_quiet_period() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        let sim_state_for_straggler = sim_state.clone();
        // A single stale heartbeat lands early — well before the 150ms quiet
        // period could have elapsed — and nothing genuine ever follows it.
        // wait_for_reconnect must reject it as a straggler (clear it,
        // restart the countdown) rather than accept it, and since no real
        // reconnect ever arrives, it must time out.
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(10)).await;
            sim_state_for_straggler.set_heartbeat_status(true, true);
        });

        let handler = make_handler(None, None, sim_state);
        let ok = handler
            .wait_for_reconnect(
                Duration::from_millis(500),
                Duration::from_millis(20),
                Duration::from_millis(150),
            )
            .await;
        assert!(!ok, "a lone straggler heartbeat must not be trusted as a genuine reconnect");
    }

    #[tokio::test]
    async fn check_reflects_cached_heartbeat_status() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        sim_state.set_heartbeat_status(false, true);
        let handler = make_handler(None, None, sim_state);
        let status = handler.check().await;
        assert!(status.connected);
        assert!(!status.hitl_enabled);
        assert!(status.is_quadrotor);
    }
}
