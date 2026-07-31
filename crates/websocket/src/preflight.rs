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
///
/// The budget has to cover the *daemon's* whole reconnect ladder, not just
/// the FC's boot: the serial link only drops once PX4 actually resets, then
/// the daemon spends 5s on its heartbeat watchdog and 10s on the bootloader
/// backoff (`main.rs`) before it even reopens the port, PX4 itself sits in
/// its bootloader for another 3-5s, and only then can a first HEARTBEAT
/// arrive — which this function still discounts until
/// `PREFLIGHT_QUIET_PERIOD` (2s) of silence has been confirmed. That is
/// ~20-22s in the good case, and a single extra bootloader cycle adds
/// another ~15s. 30s left no headroom at all and reported "FC did not
/// reconnect" for perfectly healthy hardware; 60s absorbs one full extra
/// cycle plus margin.
const PREFLIGHT_RECONNECT_TIMEOUT: Duration = Duration::from_secs(60);
const PREFLIGHT_RECONNECT_POLL_INTERVAL: Duration = Duration::from_millis(250);

/// Number of times the post-reboot verification re-reads the cached
/// HEARTBEAT flags before declaring the settings did not take effect. The
/// very first HEARTBEAT after a reboot can plausibly race ahead of PX4's
/// own internal state publication, and a single false reading would
/// otherwise be a hard, unrecoverable failure.
const VERIFY_SETTLE_ATTEMPTS: u8 = 3;
/// Gap between verification re-reads (~1s of extra worst-case latency
/// against the 60s reconnect budget).
const VERIFY_SETTLE_INTERVAL: Duration = Duration::from_millis(500);

/// Minimum continuous silence (no HEARTBEAT counter increments) required
/// after the reboot command goes out before a subsequent HEARTBEAT is
/// trusted as a genuine post-reboot reconnect rather than a straggler the FC
/// sent before it actually processed the reboot command. PX4 takes 3-5s to
/// clear its bootloader on power-up; a straggler heartbeat lands within about
/// one HEARTBEAT period (~1s) of the reboot command being sent. 2s cleanly
/// separates the two.
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
    ///
    /// `connected` is derived from the monotonic HEARTBEAT counter, which is
    /// never reset — so a client that checks while another client's `apply()`
    /// is mid-reboot still correctly sees `connected: true` and is gated,
    /// rather than reading a cleared "no FC" and silently skipping the gate.
    pub async fn check(&self) -> PreflightStatus {
        let (count, hitl_enabled, is_quadrotor) = self.sim_state.heartbeat_status();
        PreflightStatus {
            connected: count > 0,
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

        // Snapshot the HEARTBEAT counter before anything else can happen, so
        // the reconnect wait below has a watermark to compare against. Taken
        // instead of clearing shared state: a destructive clear would make a
        // concurrent `check()` report `connected: false` for the whole reboot
        // window, which the preflight gate reads as "no FC to misconfigure"
        // and silently skips.
        let baseline_count = self.sim_state.heartbeat_status().0;

        send_progress(&progress_tx, PreflightApplyState::Rebooting).await;
        if let Err(e) = mav_tx.try_send(make_reboot_autopilot()) {
            return PreflightApplyResult {
                state: PreflightApplyState::Error,
                success: false,
                error: Some(format!("Failed to send reboot command: {e}")),
            };
        }

        send_progress(&progress_tx, PreflightApplyState::Reconnecting).await;
        if !self
            .wait_for_reconnect(
                baseline_count,
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

        // Re-read the flags a few times instead of trusting the single
        // HEARTBEAT that ended the reconnect wait: PX4's first post-reboot
        // HEARTBEAT can plausibly precede its own internal state
        // publication, and a lone false reading here is otherwise a hard
        // failure with no automatic recovery.
        send_progress(&progress_tx, PreflightApplyState::Verifying).await;
        let (mut hitl_enabled, mut is_quadrotor) = (false, false);
        for attempt in 0..VERIFY_SETTLE_ATTEMPTS {
            let (_, h, q) = self.sim_state.heartbeat_status();
            hitl_enabled = h;
            is_quadrotor = q;
            if hitl_enabled && is_quadrotor {
                break;
            }
            if attempt + 1 < VERIFY_SETTLE_ATTEMPTS {
                tokio::time::sleep(VERIFY_SETTLE_INTERVAL).await;
            }
        }
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

    /// Poll the monotonic HEARTBEAT counter until it advances past a
    /// HEARTBEAT that is trustworthy as a genuine post-reboot reconnect, or
    /// `timeout` elapses.
    ///
    /// `baseline` is the count the caller snapshotted immediately before
    /// sending the reboot — this function mutates no shared state, so a
    /// concurrent `check()` keeps seeing an accurate `connected` for the
    /// whole window. A HEARTBEAT is only trusted once `quiet_period` of
    /// continuous silence (no further increments) has been observed first:
    /// the physical FC keeps heartbeating its old pre-fix state for a while
    /// after the reboot command goes out, well before it has actually
    /// rebooted, so each such straggler just moves the watermark up and
    /// restarts the countdown.
    ///
    /// Trustworthiness is checked *before* the deadline, so a heartbeat that
    /// becomes trustworthy on the same tick the deadline expires is still
    /// accepted rather than discarded as a timeout.
    async fn wait_for_reconnect(
        &self,
        baseline: u64,
        timeout: Duration,
        poll_interval: Duration,
        quiet_period: Duration,
    ) -> bool {
        let deadline = tokio::time::Instant::now() + timeout;
        let mut last_seen_count = baseline;
        let mut quiet_start = tokio::time::Instant::now();
        let mut confirmed_quiet = false;

        loop {
            let (count, ..) = self.sim_state.heartbeat_status();
            if count > last_seen_count {
                if confirmed_quiet {
                    // Arrived only after we confirmed real silence — genuine.
                    return true;
                }
                // Straggler: bump our watermark and restart the quiet countdown.
                last_seen_count = count;
                quiet_start = tokio::time::Instant::now();
            } else if !confirmed_quiet && tokio::time::Instant::now() >= quiet_start + quiet_period {
                confirmed_quiet = true;
            }

            if tokio::time::Instant::now() >= deadline {
                return false;
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
        let baseline = sim_state.heartbeat_status().0;
        let handler = make_handler(None, None, sim_state);
        let ok = handler
            .wait_for_reconnect(
                baseline,
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

        let baseline = sim_state.heartbeat_status().0;
        let handler = make_handler(None, None, sim_state);
        let ok = handler
            .wait_for_reconnect(
                baseline,
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

    #[tokio::test]
    async fn check_reports_connected_during_reconnect_wait() {
        // The exact hole the monotonic counter closes. While apply() waits out
        // the reboot, a concurrent RequestPreflightCheck (page reload, second
        // browser tab) must still report connected: true. Under the old
        // destructive clear it read connected: false, which the gate treats as
        // "no FC to misconfigure, skip silently" — letting a build be pushed
        // to a flight controller that is mid-reboot.
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        let (_px4, _captured) = spawn_fake_px4(mav_rx, pv_tx.clone());

        let sim_state = SimulationState::new(SimulationConfig::default());
        // An FC is connected but misconfigured — exactly what triggers apply().
        sim_state.set_heartbeat_status(false, false);

        let handler =
            std::sync::Arc::new(make_handler(Some(mav_tx), Some(pv_tx), sim_state.clone()));
        let apply_handler = handler.clone();
        let (progress_tx, mut progress_rx) = mpsc::channel(8);
        let apply_task = tokio::spawn(async move { apply_handler.apply(progress_tx).await });

        // Wait until apply() has actually entered the reconnect wait.
        loop {
            let msg = progress_rx
                .recv()
                .await
                .expect("progress channel closed before Reconnecting");
            if let OutgoingMessage::PreflightApplyResult(r) = msg {
                if matches!(r.state, PreflightApplyState::Reconnecting) {
                    break;
                }
            }
        }

        let status = handler.check().await;
        assert!(
            status.connected,
            "a concurrent check during the reboot wait must still see the FC as connected"
        );

        // Let apply() finish so nothing outlives the test: model the FC coming
        // back after genuine silence past PREFLIGHT_QUIET_PERIOD.
        tokio::time::sleep(Duration::from_millis(2200)).await;
        sim_state.set_heartbeat_status(true, true);
        let result = apply_task.await.unwrap();
        assert!(result.success, "apply() failed: {:?}", result.error);
    }

    #[tokio::test]
    async fn check_before_and_after_apply_reflects_reboot_outcome() {
        // The public pair exercised together rather than in isolation:
        // check() reports the pre-apply reality, apply() runs the whole
        // reboot cycle, and check() then reports the post-reboot reality.
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        let (_px4, _captured) = spawn_fake_px4(mav_rx, pv_tx.clone());

        let sim_state = SimulationState::new(SimulationConfig::default());
        let handler =
            std::sync::Arc::new(make_handler(Some(mav_tx), Some(pv_tx), sim_state.clone()));

        let before = handler.check().await;
        assert!(!before.connected, "nothing has heartbeated yet");
        assert!(!before.hitl_enabled);
        assert!(!before.is_quadrotor);

        let sim_state_for_reboot = sim_state.clone();
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(2200)).await;
            sim_state_for_reboot.set_heartbeat_status(true, true);
        });

        let apply_handler = handler.clone();
        let (progress_tx, mut progress_rx) = mpsc::channel(8);
        let apply_task = tokio::spawn(async move { apply_handler.apply(progress_tx).await });
        // Drain progress so a full channel can never stall the apply task.
        tokio::spawn(async move { while progress_rx.recv().await.is_some() {} });

        let result = apply_task.await.unwrap();
        assert!(result.success, "apply() failed: {:?}", result.error);

        let after = handler.check().await;
        assert!(after.connected);
        assert!(after.hitl_enabled);
        assert!(after.is_quadrotor);
    }
}
