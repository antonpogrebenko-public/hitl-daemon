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

    /// Root-cause regression test: SYS_HITL/SYS_AUTOSTART are genuinely
    /// INT32 params on PX4. Sending a numeric `1.0f32` with
    /// `param_type = MAV_PARAM_TYPE_REAL32` (what `make_param_set` — correct
    /// for every other param this daemon pushes — would send) makes real
    /// PX4 silently reject the PARAM_SET: no PARAM_VALUE reply at all,
    /// observed on real hardware as a plain ack timeout, not a value
    /// mismatch. The correct encoding bit-reinterprets the int's bytes into
    /// the wire float slot and declares MAV_PARAM_TYPE_INT32.
    #[test]
    fn make_param_set_i32_bit_encodes_value_and_declares_int32_type() {
        let msg = make_param_set_i32("SYS_HITL", 1);
        match msg {
            MavMessage::PARAM_SET(p) => {
                assert_eq!(p.param_type, MavParamType::MAV_PARAM_TYPE_INT32);
                assert_eq!(
                    p.param_value.to_bits() as i32,
                    1,
                    "value must be bit-encoded, not numerically cast"
                );
                assert_ne!(
                    p.param_value, 1.0,
                    "a literal 1.0f32 is exactly the wire encoding PX4 silently rejects for an INT32 param"
                );
            }
            other => panic!("expected PARAM_SET, got {other:?}"),
        }
    }
}

use crate::build_config::{
    make_param_save, PARAM_ACK_TIMEOUT, PARAM_RETRY_COUNT, PX4_TARGET_COMPONENT, PX4_TARGET_SYSTEM,
};
use crate::handler::ValidatedNshCommand;
use crate::protocol::{OutgoingMessage, PreflightApplyResult, PreflightApplyState, PreflightStatus};
use crossbeam_channel::Sender;
use mavlink::ardupilotmega::{MavMessage, MavParamType, PARAM_SET_DATA};
use simulation::SimulationState;
use std::time::Duration;
use tokio::sync::{broadcast, mpsc};
use tracing::warn;

/// PX4 SYS_AUTOSTART id for "Generic Quadcopter X" — the fixed target this
/// feature applies. No per-build airframe selection.
///
/// INT32, not f32: SYS_AUTOSTART (and SYS_HITL below) are genuinely INT32
/// parameters in PX4's metadata, unlike every other param this daemon pushes
/// (PID gains, CAL offsets — all genuinely REAL32). See `make_param_set_i32`.
const QUADROTOR_AUTOSTART_ID: i32 = 4001;

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

/// Delay between the fire-and-forget PARAM_SAVE (flash write) and the reboot
/// that follows it. `MAV_CMD_PREFLIGHT_STORAGE`'s flash commit on PX4 is
/// asynchronous and best-effort-acked (~100ms typical, per `make_param_save`'s
/// doc comment) — writing the command's bytes to the serial port is not the
/// same as PX4 finishing the write. Without this gap, the reboot used to be
/// sent essentially back-to-back with the save (both queue onto the same
/// serial writer, which drains the NSH queue before the MAVLink queue each
/// tick, so ordering on the wire wasn't even guaranteed to match program
/// order), racing an in-flight flash commit against a hard MCU reset.
/// Observed on real hardware as both non-deterministic loss of the
/// just-applied SYS_HITL/SYS_AUTOSTART params (the write never landed before
/// the reset) and a wedged FC that a daemon restart alone could not recover
/// (an interrupted flash erase/write left the parameter store corrupted,
/// requiring a physical power cycle). 500ms is 5x the typical commit time.
const PARAM_SAVE_SETTLE_DELAY: Duration = Duration::from_millis(500);

pub struct PreflightHandler {
    mav_tx: Option<Sender<MavMessage>>,
    param_value_tx: Option<broadcast::Sender<(String, f32)>>,
    /// Used to trigger the post-apply reboot via PX4's own NSH `reboot`
    /// command rather than `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`. On at least
    /// one real board this command construction (`COMMAND_LONG`, correct
    /// per the MAVLink spec, byte-identical in shape to the working
    /// `make_param_save`) got zero `COMMAND_ACK` — not even a denial — while
    /// NSH's `reboot` reliably triggers a real reset every time. NSH goes
    /// through PX4's own shell rather than its MAVLink command router, so it
    /// isn't affected by whatever is silently dropping this one command.
    nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>,
    sim_state: SimulationState,
    /// Guards against two concurrent `apply()` runs against the same
    /// physical FC. `ApplyPreflightParams` is now dispatched onto a
    /// detached task rather than awaited inline (so the WebSocket receive
    /// loop is never blocked for the 20-60s reboot window), which removed
    /// the incidental serialization a blocking await used to provide. This
    /// handler is shared (via `Arc`) across every connected browser client,
    /// so the guard has to live here, not on any one connection — a double
    /// click on one tab and a second tab hitting "Apply" both dispatch
    /// through this same instance.
    applying: std::sync::atomic::AtomicBool,
}

impl PreflightHandler {
    pub fn new(
        mav_tx: Option<Sender<MavMessage>>,
        param_value_tx: Option<broadcast::Sender<(String, f32)>>,
        nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>,
        sim_state: SimulationState,
    ) -> Self {
        Self {
            mav_tx,
            param_value_tx,
            nsh_tx,
            applying: std::sync::atomic::AtomicBool::new(false),
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
    ///
    /// Rejects a second call while one is already running (`self.applying`),
    /// checked before anything else so a concurrent call never sends a
    /// duplicate PARAM_SET/reboot to the FC. Since `ApplyPreflightParams` is
    /// dispatched onto a detached task rather than awaited inline (the
    /// receive loop can't afford to block for the 20-60s reboot window), the
    /// blocking-await serialization that used to make this a non-issue is
    /// gone — this guard replaces it.
    pub async fn apply(&self, progress_tx: mpsc::Sender<OutgoingMessage>) -> PreflightApplyResult {
        if self
            .applying
            .swap(true, std::sync::atomic::Ordering::SeqCst)
        {
            return PreflightApplyResult {
                state: PreflightApplyState::Error,
                success: false,
                error: Some("A preflight apply is already in progress".to_string()),
            };
        }
        // Releases the guard on every return path below, including early
        // returns, without needing to touch each one individually.
        let _guard = ApplyGuard(&self.applying);

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

        for (name, value) in [("SYS_HITL", 1i32), ("SYS_AUTOSTART", QUADROTOR_AUTOSTART_ID)] {
            let mut acked = false;
            for attempt in 1..=PARAM_RETRY_COUNT {
                let mut rx = param_value_tx.subscribe();
                match mav_tx.try_send(make_param_set_i32(name, value)) {
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

                if wait_for_int_param_ack(&mut rx, name, value).await {
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

        // Let PX4's async flash commit finish before the reboot resets the
        // MCU. See PARAM_SAVE_SETTLE_DELAY.
        tokio::time::sleep(PARAM_SAVE_SETTLE_DELAY).await;

        // Snapshot the HEARTBEAT counter before anything else can happen, so
        // the reconnect wait below has a watermark to compare against. Taken
        // instead of clearing shared state: a destructive clear would make a
        // concurrent `check()` report `connected: false` for the whole reboot
        // window, which the preflight gate reads as "no FC to misconfigure"
        // and silently skips.
        let baseline_count = self.sim_state.heartbeat_status().0;

        send_progress(&progress_tx, PreflightApplyState::Rebooting).await;
        if let Err(e) = self.send_reboot_via_nsh().await {
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

    /// Trigger a reboot via PX4's own NSH `reboot` command rather than
    /// `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`. See the doc comment on the
    /// `nsh_tx` field for why: the MAVLink command path got zero
    /// `COMMAND_ACK` on real hardware despite correct construction, while
    /// NSH's `reboot` reliably triggers a real reset. Fire-and-forget, like
    /// `BuildConfigHandler::restart_ekf2`'s internal NSH commands — a reboot
    /// makes any response moot.
    async fn send_reboot_via_nsh(&self) -> Result<(), String> {
        let Some(ref nsh_tx) = self.nsh_tx else {
            return Err("NSH channel not available".to_string());
        };
        let cmd = ValidatedNshCommand {
            request_id: 0xFFFF_FF04, // internal-use id, distinct from build_config.rs's 0xFFFF_FF01/02
            command: "reboot".to_string(),
            timeout_ms: 2000,
            client_id: 0, // system client
        };
        nsh_tx
            .send(cmd)
            .await
            .map_err(|e| format!("Failed to send reboot via NSH: {e}"))
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

/// Resets `PreflightHandler::applying` to `false` on drop, so `apply()`
/// releases its in-flight guard on every return path — including early
/// returns and panics — without each one needing to reset it explicitly.
struct ApplyGuard<'a>(&'a std::sync::atomic::AtomicBool);

impl Drop for ApplyGuard<'_> {
    fn drop(&mut self) {
        self.0.store(false, std::sync::atomic::Ordering::SeqCst);
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

/// Build a `PARAM_SET` for a genuinely INT32 PX4 parameter (`SYS_HITL`,
/// `SYS_AUTOSTART`). MAVLink's `PARAM_SET` always carries `param_value` as a
/// raw 4-byte wire field regardless of the parameter's real type; the
/// correct encoding for a non-float param is to bit-reinterpret the int's
/// bytes into that slot and declare `param_type` accordingly. Every other
/// param this daemon pushes (PID gains, CAL offsets) is genuinely REAL32,
/// so `make_param_set` sending a numeric `1.0f32` with `param_type =
/// MAV_PARAM_TYPE_REAL32` had always been correct there — but for an INT32
/// param PX4 silently rejects the mismatched-type message: no PARAM_VALUE
/// reply at all, not even an incorrect one, which is why this surfaced as a
/// plain ack timeout rather than a value mismatch.
fn make_param_set_i32(name: &str, value: i32) -> MavMessage {
    let mut param_id = [0u8; 16];
    let bytes = name.as_bytes();
    let copy_len = bytes.len().min(param_id.len());
    param_id[..copy_len].copy_from_slice(&bytes[..copy_len]);
    MavMessage::PARAM_SET(PARAM_SET_DATA {
        param_value: f32::from_bits(value as u32),
        target_system: PX4_TARGET_SYSTEM,
        target_component: PX4_TARGET_COMPONENT,
        param_id,
        param_type: MavParamType::MAV_PARAM_TYPE_INT32,
    })
}

/// Drain `rx` until a `PARAM_VALUE` arrives whose name matches `name` and
/// whose bit-reinterpreted value equals `expected`. PX4 echoes INT32 params
/// by bit-packing them into the same 4-byte wire slot every param uses, not
/// by numeric cast — mirrors `wait_for_param_ack` but compares as bits, not
/// as a float value within an epsilon.
async fn wait_for_int_param_ack(
    rx: &mut broadcast::Receiver<(String, f32)>,
    name: &str,
    expected: i32,
) -> bool {
    let deadline = tokio::time::Instant::now() + PARAM_ACK_TIMEOUT;
    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            return false;
        }
        match tokio::time::timeout(remaining, rx.recv()).await {
            Ok(Ok((got_name, got_value))) => {
                if got_name == name && got_value.to_bits() as i32 == expected {
                    return true;
                }
                // Unrelated PARAM_VALUE (QGC pull, other params) — keep draining.
            }
            Ok(Err(broadcast::error::RecvError::Lagged(n))) => {
                warn!(
                    param = name,
                    lagged = n,
                    "PARAM_VALUE receiver lagged — continuing to wait"
                );
            }
            Ok(Err(broadcast::error::RecvError::Closed)) => return false,
            Err(_) => return false,
        }
    }
}

#[cfg(test)]
mod apply_tests {
    use super::*;
    use crossbeam_channel::bounded;
    use simulation::SimulationConfig;
    use std::sync::Mutex;
    use std::time::Instant;

    fn make_handler(
        mav_tx: Option<Sender<MavMessage>>,
        param_value_tx: Option<broadcast::Sender<(String, f32)>>,
        sim_state: SimulationState,
    ) -> PreflightHandler {
        // A fire-and-forget NSH channel with a task draining it forever, so
        // send_reboot_via_nsh's send succeeds without needing a real NSH
        // handler task — apply() never awaits a response to the reboot.
        let (nsh_tx, mut nsh_rx) = mpsc::channel::<ValidatedNshCommand>(4);
        tokio::spawn(async move { while nsh_rx.recv().await.is_some() {} });
        PreflightHandler::new(mav_tx, param_value_tx, Some(nsh_tx), sim_state)
    }

    #[derive(Debug, Clone)]
    enum CapturedMsg {
        ParamSet(String, f32),
        CommandLong(u32, Instant),
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
                            .push(CapturedMsg::CommandLong(c.command as u32, Instant::now()));
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

        // Capture NSH commands (with arrival time) instead of silently
        // draining them, so this test can confirm both that the reboot went
        // out as an NSH "reboot" (rather than the unacked
        // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) and that it didn't race the
        // preceding PARAM_SAVE's flash commit (PARAM_SAVE_SETTLE_DELAY).
        let (nsh_tx, mut nsh_rx) = mpsc::channel::<ValidatedNshCommand>(4);
        let nsh_captured = std::sync::Arc::new(Mutex::new(Vec::<(String, Instant)>::new()));
        let nsh_captured_clone = nsh_captured.clone();
        tokio::spawn(async move {
            while let Some(cmd) = nsh_rx.recv().await {
                nsh_captured_clone
                    .lock()
                    .unwrap()
                    .push((cmd.command, Instant::now()));
            }
        });

        let sim_state = SimulationState::new(SimulationConfig::default());
        let sim_state_for_reboot = sim_state.clone();
        // Simulate the FC coming back post-reboot: a real HEARTBEAT would
        // call SimulationState::set_heartbeat_status via main.rs; here we do
        // it directly after modeling genuine silence past the real
        // PREFLIGHT_QUIET_PERIOD (2s) — anything sooner would be (correctly)
        // treated as a stale pre-reboot straggler and cleared.
        tokio::spawn(async move {
            // Comfortably past PARAM_SAVE_SETTLE_DELAY (500ms, now part of
            // apply()'s pre-reboot sequence) + PREFLIGHT_QUIET_PERIOD (2s)
            // from whenever wait_for_reconnect actually starts polling, plus
            // the same ~200ms margin the original 2200ms value carried.
            tokio::time::sleep(Duration::from_millis(2700)).await;
            sim_state_for_reboot.set_heartbeat_status(true, true);
        });

        let handler = PreflightHandler::new(Some(mav_tx), Some(pv_tx), Some(nsh_tx), sim_state);
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

        let param_save_at = snapshot
            .iter()
            .find_map(|m| match m {
                CapturedMsg::CommandLong(245, t) => Some(*t),
                _ => None,
            })
            .expect("PARAM_SAVE (MAV_CMD_PREFLIGHT_STORAGE) was not sent");

        let nsh_snapshot = nsh_captured.lock().unwrap().clone();
        assert_eq!(
            nsh_snapshot
                .iter()
                .map(|(cmd, _)| cmd.as_str())
                .collect::<Vec<_>>(),
            ["reboot"],
            "reboot must go out via NSH, not MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN"
        );
        let reboot_at = nsh_snapshot[0].1;
        assert!(
            reboot_at.duration_since(param_save_at) >= PARAM_SAVE_SETTLE_DELAY,
            "reboot fired only {:?} after PARAM_SAVE, need >= {:?} so PX4's flash \
             commit isn't interrupted by the reset",
            reboot_at.duration_since(param_save_at),
            PARAM_SAVE_SETTLE_DELAY
        );
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
            // Comfortably past PARAM_SAVE_SETTLE_DELAY (500ms, now part of
            // apply()'s pre-reboot sequence) + PREFLIGHT_QUIET_PERIOD (2s)
            // from whenever wait_for_reconnect actually starts polling, plus
            // the same ~200ms margin the original 2200ms value carried.
            tokio::time::sleep(Duration::from_millis(2700)).await;
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

    #[tokio::test]
    async fn concurrent_apply_is_rejected_without_touching_the_fc() {
        // ApplyPreflightParams is dispatched onto a detached task rather than
        // awaited inline (so the WebSocket receive loop is never blocked for
        // the real 20-60s reboot window), which removed the incidental
        // serialization a blocking await used to provide. Without a guard, a
        // second concurrent call would send a duplicate PARAM_SET/reboot to
        // the same FC. No fake PX4 drains mav_rx here, so the first apply()
        // sits in its PARAM_SET retry loop (~2.4s) for the whole test.
        let (mav_tx, _mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        let sim_state = SimulationState::new(SimulationConfig::default());
        let handler = std::sync::Arc::new(make_handler(Some(mav_tx), Some(pv_tx), sim_state));

        let first_handler = handler.clone();
        let (first_tx, _first_rx) = mpsc::channel(8);
        let first = tokio::spawn(async move { first_handler.apply(first_tx).await });

        // Give the first call a moment to acquire the guard.
        tokio::time::sleep(Duration::from_millis(20)).await;

        let (second_tx, _second_rx) = mpsc::channel(8);
        let second = handler.apply(second_tx).await;
        assert!(!second.success);
        assert!(matches!(second.state, PreflightApplyState::Error));
        assert!(second
            .error
            .as_deref()
            .unwrap_or_default()
            .contains("already in progress"));

        let first_result = first.await.unwrap();
        assert!(!first_result.success, "unrelated ack-timeout failure expected: {first_result:?}");

        // The guard must release once the first call finishes, so a
        // subsequent apply() is not permanently locked out.
        let (third_tx, _third_rx) = mpsc::channel(8);
        let third = handler.apply(third_tx).await;
        assert!(
            !third
                .error
                .as_deref()
                .unwrap_or_default()
                .contains("already in progress"),
            "guard should have released after the first apply() finished, got: {third:?}"
        );
    }
}
