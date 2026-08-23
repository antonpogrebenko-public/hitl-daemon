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
    PARAM_ACK_EPSILON,
    make_param_save, make_param_set, wait_for_param_ack, PARAM_ACK_TIMEOUT, PARAM_RETRY_COUNT,
    PX4_TARGET_COMPONENT, PX4_TARGET_SYSTEM,
};
use crate::handler::ValidatedNshCommand;
use crate::board_identity::BoardIdentity;
use crate::param_io::{read_params_with, ParamReadPolicy, ParamValue};
use crate::snapshot::{SessionSnapshot, StoredSnapshot};
use std::sync::Arc;
use crate::protocol::{
    OutgoingMessage, PreflightApplyResult, PreflightApplyState, PreflightReadiness,
    PreflightStatus, RestoreMismatch, RestoreResult, RestoreSnapshot, RestoreState,
    SnapshotCaptured, SnapshotParam, SnapshotStored,
};
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

/// PX4 params — beyond SYS_HITL/SYS_AUTOSTART — required for a genuinely
/// armable HITL session, not just a mode switch. SYS_HITL alone tells PX4
/// to accept HIL_SENSOR/HIL_GPS instead of real hardware; it does nothing
/// about the real-hardware preflight/EKF gates that check for a real IMU,
/// baro, compass, and SD card, so arming failed with "Accel/Gyro/Compass/
/// Baro ... missing/uncalibrated", "Missing FMU SD Card", and "Strong
/// magnetic interference" even with SYS_HITL=1 verified. Values sourced
/// from this repo's own prior-working `params-hitl-backup.txt` snapshot
/// for this board — a known-good HITL config, not a guess.
///
/// INT32: confirmed by the backup file's own `param show` formatting (no
/// decimal point in what PX4 itself printed) — the same signal that caught
/// SYS_HITL/SYS_AUTOSTART's type mismatch (see `make_param_set_i32`), and
/// cross-checked 3-for-3 against ground truth already established this
/// session (SYS_HITL, RC1_MIN, RC_CHAN_CNT).
const HITL_SUPPORT_PARAMS_I32: &[(&str, i32)] = &[
    ("SYS_HAS_BARO", 0),
    ("CBRK_SUPPLY_CHK", 894281),
    ("COM_ARM_HFLT_CHK", 0),
    ("COM_ARM_MAG_ANG", 180),
    ("COM_ARM_MAG_STR", 0),
    ("COM_ARM_SDCARD", 0),
    ("EKF2_GPS_CHECK", 0),
    ("EKF2_MAG_CHECK", 0),
    ("EKF2_MULTI_IMU", 1),
    ("GPS_1_CONFIG", 0),
    ("HIL_ACT_FUNC1", 101),
    ("HIL_ACT_FUNC2", 102),
    ("HIL_ACT_FUNC3", 103),
    ("HIL_ACT_FUNC4", 104),
    ("SENS_IMU_MODE", 0),
    ("SENS_MAG_AUTOCAL", 0),
];

/// REAL32 counterparts of the above (backup file shows decimals for these).
/// Loosens EKF innovation-drift gates that a simulated IMU with zero
/// physical mounting bias can otherwise trip.
const HITL_SUPPORT_PARAMS_F32: &[(&str, f32)] = &[
    ("EKF2_ABL_LIM", 0.0),
    ("EKF2_REQ_HDRIFT", 3.0),
    ("EKF2_REQ_VDRIFT", 3.0),
];

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
/// asynchronous and best-effort-acked (~100ms typical for a single dirty
/// param, per `make_param_save`'s doc comment) — writing the command's bytes
/// to the serial port is not the same as PX4 finishing the write. Without
/// this gap, the reboot used to be sent essentially back-to-back with the
/// save (both queue onto the same serial writer, which drains the NSH queue
/// before the MAVLink queue each tick, so ordering on the wire wasn't even
/// guaranteed to match program order), racing an in-flight flash commit
/// against a hard MCU reset. Observed on real hardware as both
/// non-deterministic loss of the just-applied params (the write never
/// landed before the reset) and a wedged FC that a daemon restart alone
/// could not recover (an interrupted flash erase/write left the parameter
/// store corrupted — the FC's USB descriptor reports it stuck in bootloader,
/// "PX4 BL FMU ...", indefinitely instead of the normal 3-5s dwell —
/// requiring a physical power cycle).
///
/// `apply()` now pushes `HITL_SUPPORT_PARAMS_I32`/`_F32` alongside
/// SYS_HITL/SYS_AUTOSTART (~21 dirty params total, not 2), and PX4 commits
/// each dirty param as its own small flash write, so the total commit time
/// scales with the count. 500ms (sized for a 2-param save) reproduced this
/// exact bootloader-stuck failure once the push grew; 2s gives roughly the
/// same 5x margin over a 21-param commit that 500ms gave over a 2-param one.
const PARAM_SAVE_SETTLE_DELAY: Duration = Duration::from_millis(2000);

/// How long to wait for the browser to confirm the snapshot is stored.
///
/// Generous: the browser may also be replicating to the user's account over a
/// slow link. Bounded anyway, because a tab that closed mid-hand-off must not
/// leave provisioning waiting forever.
const SNAPSHOT_ACK_TIMEOUT: Duration = Duration::from_secs(20);

/// Wait for the browser's persistence acknowledgement for `board_identity`.
async fn await_snapshot_ack(
    rx: &mut broadcast::Receiver<SnapshotStored>,
    board_identity: &str,
    ack_timeout: Duration,
) -> Result<(), String> {
    let deadline = tokio::time::Instant::now() + ack_timeout;
    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            return Err(
                "The browser did not confirm the restore point was saved. Nothing was changed."
                    .to_string(),
            );
        }
        match tokio::time::timeout(remaining, rx.recv()).await {
            Ok(Ok(ack)) if ack.board_identity == board_identity => {
                return if ack.stored {
                    Ok(())
                } else {
                    Err(format!(
                        "The browser could not save the restore point ({}). Nothing was changed.",
                        ack.error.unwrap_or_else(|| "no reason given".to_string())
                    ))
                };
            }
            // An acknowledgement for a different board belongs to another
            // session; keep waiting for ours.
            Ok(Ok(_)) => {}
            Ok(Err(broadcast::error::RecvError::Lagged(n))) => {
                warn!(lagged = n, "Snapshot ack receiver lagged");
            }
            Ok(Err(broadcast::error::RecvError::Closed)) => {
                return Err("Lost the browser connection before the restore point was \
                            confirmed. Nothing was changed."
                    .to_string());
            }
            Err(_) => {
                return Err(
                    "The browser did not confirm the restore point was saved. Nothing was changed."
                        .to_string(),
                );
            }
        }
    }
}

/// How many write-save-reboot-verify cycles to run before giving up.
///
/// Bounded deliberately: each cycle commits parameters to flash, so an
/// unbounded retry on a board that will never verify would wear it out.
const PROVISION_ATTEMPTS: u8 = 2;

/// How long the flight controller is left alone after a write-and-reboot cycle.
///
/// Covers PX4's flash commit (`PARAM_SAVE_SETTLE_DELAY`, 2s) plus its 3-5s
/// bootloader dwell, with margin. Starting a second cycle inside this window
/// can interrupt the commit and leave the parameter store corrupted — the
/// board then reports `PX4 BL FMU` indefinitely and needs a physical power
/// cycle, which no amount of retrying from here can undo.
const CYCLE_COOLDOWN: Duration = Duration::from_secs(15);

/// Why one provisioning cycle failed, and whether repeating it could help.
struct ApplyFailure {
    message: String,
    /// True only for a verification failure. An unacked PARAM_SET or a board
    /// that never came back are not fixed by pushing the same values again.
    retryable: bool,
}

impl ApplyFailure {
    fn fatal(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            retryable: false,
        }
    }

    fn retryable(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            retryable: true,
        }
    }
}

pub struct PreflightHandler {
    mav_tx: Option<Sender<MavMessage>>,
    param_value_tx: Option<broadcast::Sender<ParamValue>>,
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
    /// Stable key for the connected board, populated from AUTOPILOT_VERSION.
    /// Provisioning refuses to run without it: a snapshot that cannot be tied
    /// to a board cannot be safely restored onto one.
    board_identity: Arc<tokio::sync::RwLock<Option<BoardIdentity>>>,
    /// Browser acknowledgements that a captured snapshot is durably stored.
    snapshot_ack_tx: broadcast::Sender<SnapshotStored>,
    /// Session-scoped copy of the snapshot, for a restore issued before the
    /// browser has to hand one back.
    session_snapshot: Arc<SessionSnapshot>,
    /// Read budget for snapshot capture. Production default; tests shorten it
    /// so the failure paths do not cost a minute of real waiting.
    read_policy: ParamReadPolicy,
    /// How long to wait for the browser's persistence acknowledgement.
    ack_timeout: Duration,
    /// When the last write-and-reboot cycle finished.
    ///
    /// PX4 commits parameters to flash and then reboots; a second cycle
    /// starting on the heels of the first can interrupt that commit and leave
    /// the parameter store corrupted, with the board stuck in its bootloader
    /// until it is physically power-cycled. Observed on real hardware.
    last_cycle_finished: std::sync::Mutex<Option<std::time::Instant>>,
    /// How long to leave the board alone after a write cycle. Shortened in
    /// tests; production always uses `CYCLE_COOLDOWN`.
    cycle_cooldown: Duration,
    /// Provisioning lifecycle, fanned out to every connected client.
    ///
    /// One path rather than a per-connection channel: a reloaded page, a
    /// second tab, and the tab that started the operation all need the same
    /// stream, and mirroring into both would deliver every frame twice to
    /// whoever asked.
    provisioning_tx: broadcast::Sender<OutgoingMessage>,
}

impl PreflightHandler {
    pub fn new(
        mav_tx: Option<Sender<MavMessage>>,
        param_value_tx: Option<broadcast::Sender<ParamValue>>,
        nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>,
        sim_state: SimulationState,
    ) -> Self {
        Self::with_identity(
            mav_tx,
            param_value_tx,
            nsh_tx,
            sim_state,
            Arc::new(tokio::sync::RwLock::new(None)),
        )
    }

    pub fn with_identity(
        mav_tx: Option<Sender<MavMessage>>,
        param_value_tx: Option<broadcast::Sender<ParamValue>>,
        nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>,
        sim_state: SimulationState,
        board_identity: Arc<tokio::sync::RwLock<Option<BoardIdentity>>>,
    ) -> Self {
        let (snapshot_ack_tx, _) = broadcast::channel(8);
        Self {
            mav_tx,
            param_value_tx,
            nsh_tx,
            applying: std::sync::atomic::AtomicBool::new(false),
            sim_state,
            board_identity,
            snapshot_ack_tx,
            session_snapshot: Arc::new(SessionSnapshot::new()),
            read_policy: ParamReadPolicy::default(),
            ack_timeout: SNAPSHOT_ACK_TIMEOUT,
            last_cycle_finished: std::sync::Mutex::new(None),
            cycle_cooldown: CYCLE_COOLDOWN,
            provisioning_tx: {
                let (tx, _) = broadcast::channel(64);
                tx
            },
        }
    }

    /// Subscribe to provisioning and restore progress.
    pub fn subscribe_provisioning(&self) -> broadcast::Receiver<OutgoingMessage> {
        self.provisioning_tx.subscribe()
    }

    /// Shorten the capture budgets. Test-only.
    #[cfg(test)]
    pub fn with_fast_capture(mut self) -> Self {
        self.read_policy = ParamReadPolicy {
            timeout: Duration::from_millis(30),
            retries: 1,
        };
        self.ack_timeout = Duration::from_millis(200);
        self
    }

    /// Shorten the post-cycle cooldown. Test-only.
    #[cfg(test)]
    pub fn with_cooldown(mut self, cooldown: Duration) -> Self {
        self.cycle_cooldown = cooldown;
        self
    }

    /// The channel the WebSocket handler delivers browser acknowledgements on.
    pub fn snapshot_ack_sender(&self) -> broadcast::Sender<SnapshotStored> {
        self.snapshot_ack_tx.clone()
    }

    /// Session-scoped snapshot, for restore during this run.
    pub fn session_snapshot(&self) -> Arc<SessionSnapshot> {
        self.session_snapshot.clone()
    }

    /// Every parameter provisioning will overwrite.
    ///
    /// Derived from the same tables `apply()` writes, so the snapshot cannot
    /// drift out of step with what gets modified — a parameter written but not
    /// captured would be unrecoverable.
    fn provisioned_param_names() -> Vec<&'static str> {
        let mut names = vec!["SYS_HITL", "SYS_AUTOSTART"];
        names.extend(HITL_SUPPORT_PARAMS_I32.iter().map(|(n, _)| *n));
        names.extend(HITL_SUPPORT_PARAMS_F32.iter().map(|(n, _)| *n));
        names
    }

    /// Read every parameter provisioning will overwrite, hand it to the
    /// browser, and wait for confirmation that it is durably stored.
    ///
    /// Ordering is the whole point: read, send, persist, acknowledge, and only
    /// then write. Writing first would open a window in which the board is
    /// modified and nothing can put it back, which is the exact state the
    /// snapshot exists to prevent. Every failure path here therefore aborts
    /// before a single PARAM_SET is sent.
    async fn capture_and_confirm_snapshot(&self) -> Result<(), String> {
        let (Some(mav_tx), Some(param_value_tx)) =
            (self.mav_tx.as_ref(), self.param_value_tx.as_ref())
        else {
            return Err("No flight controller connected (sim-only mode)".to_string());
        };

        let identity = match self.board_identity.read().await.clone() {
            Some(id) => id,
            None => {
                return Err(
                    "This flight controller reports no identifying serial, so a restore \
                     point cannot be tied to it. Provisioning was not started."
                        .to_string(),
                )
            }
        };

        let names = Self::provisioned_param_names();
        let (values, failed) =
            read_params_with(mav_tx, param_value_tx, &names, self.read_policy).await;

        if !failed.is_empty() {
            return Err(format!(
                "Could not read {} of {} parameters from the flight controller ({}). \
                 Nothing was changed.",
                failed.len(),
                names.len(),
                failed.join(", ")
            ));
        }

        let params: Vec<SnapshotParam> = values
            .iter()
            .map(|v| SnapshotParam {
                name: v.name.clone(),
                value: v.decoded_value(),
                param_type: if v.is_integer() { "int32" } else { "real32" }.to_string(),
            })
            .collect();

        // Subscribe before sending: the browser can acknowledge faster than
        // this task reaches the wait, and a missed ack would stall a
        // provisioning that actually succeeded.
        let mut ack_rx = self.snapshot_ack_tx.subscribe();

        self.session_snapshot.store(StoredSnapshot {
            board_identity: identity.as_str().to_string(),
            params: params.clone(),
        });

        if self
            .provisioning_tx
            .send(OutgoingMessage::SnapshotCaptured(SnapshotCaptured {
                board_identity: identity.as_str().to_string(),
                params,
            }))
            .is_err()
        {
            return Err("No browser is connected to save the restore point. Nothing was \
                        changed."
                .to_string());
        }

        await_snapshot_ack(&mut ack_rx, identity.as_str(), self.ack_timeout).await
    }

    /// Instant, cache-only read of the FC's last-known HITL/quadrotor status.
    ///
    /// `connected` is derived from the monotonic HEARTBEAT counter, which is
    /// never reset — so a client that checks while another client's `apply()`
    /// is mid-reboot still correctly sees `connected: true` and is gated,
    /// rather than reading a cleared "no FC" and silently skipping the gate.
    pub async fn check(&self) -> PreflightStatus {
        let (count, hitl_enabled, is_quadrotor) = self.sim_state.heartbeat_status();
        // Sim-only has no board to gate on. Everything else with no heartbeat
        // yet is a board that has not reported — a wait, not a pass.
        let readiness = if self.mav_tx.is_none() {
            PreflightReadiness::NotApplicable
        } else if count == 0 {
            PreflightReadiness::Unknown
        } else if hitl_enabled && is_quadrotor {
            PreflightReadiness::Ready
        } else {
            PreflightReadiness::NotReady
        };

        PreflightStatus {
            connected: count > 0,
            hitl_enabled,
            is_quadrotor,
            readiness,
            // Read here rather than derived: identity comes from
            // AUTOPILOT_VERSION as soon as the board reports, independently of
            // whether it has ever been provisioned.
            board_identity: self
                .board_identity
                .read()
                .await
                .as_ref()
                .map(|id| id.as_str().to_string()),
        }
    }

    /// Push SYS_HITL=1 + SYS_AUTOSTART=4001 plus the supporting params in
    /// `HITL_SUPPORT_PARAMS_I32`/`_F32` that disable real-hardware preflight
    /// gates (SD card, sensor-presence, mag consistency, EKF drift checks),
    /// save, reboot the FC, and wait for it to reconnect before
    /// re-verifying. Fail-closed: any ack failure aborts before the reboot
    /// is sent.
    ///
    /// Rejects a second call while one is already running (`self.applying`),
    /// checked before anything else so a concurrent call never sends a
    /// duplicate PARAM_SET/reboot to the FC. Since `ApplyPreflightParams` is
    /// dispatched onto a detached task rather than awaited inline (the
    /// receive loop can't afford to block for the 20-60s reboot window), the
    /// blocking-await serialization that used to make this a non-issue is
    /// gone — this guard replaces it.
    pub async fn apply(&self) -> PreflightApplyResult {
        if let Some(remaining) = self.cooldown_remaining() {
            let result = PreflightApplyResult {
                state: PreflightApplyState::Error,
                success: false,
                error: Some(format!(
                    "The flight controller is still restarting from the previous change. \
                     Wait {}s and try again — writing now risks corrupting its settings.",
                    remaining.as_secs() + 1
                )),
            };
            let _ = self
                .provisioning_tx
                .send(OutgoingMessage::PreflightApplyResult(result.clone()));
            return result;
        }

        let result = self.apply_inner().await;
        // Any cycle that got as far as writing leaves the board settling,
        // whether it succeeded or not.
        self.mark_cycle_finished();
        // Terminal outcome reaches every client, not just whoever asked: a
        // reloaded page or a second tab needs to learn how this ended.
        let _ = self
            .provisioning_tx
            .send(OutgoingMessage::PreflightApplyResult(result.clone()));
        result
    }

    async fn apply_inner(&self) -> PreflightApplyResult {
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

        // Snapshot first. No PARAM_SET is sent until the browser confirms a
        // restore point exists.
        self.broadcast_progress(PreflightApplyState::Capturing);
        if let Err(error) = self.capture_and_confirm_snapshot().await {
            return PreflightApplyResult {
                state: PreflightApplyState::Error,
                success: false,
                error: Some(error),
            };
        }

        // Provisioning proper. Retried on a verification failure: PX4 can
        // report the old flags on the first HEARTBEAT after a reboot, and a
        // parameter save that did not land is recoverable by pushing again.
        // Bounded, because retrying forever would keep writing flash.
        let mut last_error = String::new();
        for attempt in 1..=PROVISION_ATTEMPTS {
            match self.apply_once(mav_tx, param_value_tx).await {
                Ok(()) => {
                    return PreflightApplyResult {
                        state: PreflightApplyState::Done,
                        success: true,
                        error: None,
                    }
                }
                Err(failure) if failure.retryable && attempt < PROVISION_ATTEMPTS => {
                    warn!(
                        attempt,
                        error = %failure.message,
                        "Provisioning did not verify — re-applying"
                    );
                    last_error = failure.message;
                }
                Err(failure) => {
                    return PreflightApplyResult {
                        state: PreflightApplyState::Error,
                        success: false,
                        error: Some(failure.message),
                    }
                }
            }
        }

        PreflightApplyResult {
            state: PreflightApplyState::Error,
            success: false,
            error: Some(format!(
                "{last_error} (after {PROVISION_ATTEMPTS} attempts)"
            )),
        }
    }

    /// Write a snapshot back to the board, save, reboot, and confirm every
    /// value actually took.
    ///
    /// Refuses when the snapshot belongs to a different board than the one
    /// connected. That check is the whole reason board identity exists:
    /// writing one aircraft's tuning onto another produces a vehicle that
    /// looks configured and flies wrong.
    pub async fn restore(&self, request: RestoreSnapshot) -> RestoreResult {
        if let Some(remaining) = self.cooldown_remaining() {
            let result = restore_error(format!(
                "The flight controller is still restarting from the previous change. \
                 Wait {}s and try again — writing now risks corrupting its settings.",
                remaining.as_secs() + 1
            ));
            let _ = self
                .provisioning_tx
                .send(OutgoingMessage::RestoreResult(result.clone()));
            return result;
        }

        let result = self.restore_inner(request).await;
        self.mark_cycle_finished();
        let _ = self
            .provisioning_tx
            .send(OutgoingMessage::RestoreResult(result.clone()));
        result
    }

    async fn restore_inner(&self, request: RestoreSnapshot) -> RestoreResult {
        if self
            .applying
            .swap(true, std::sync::atomic::Ordering::SeqCst)
        {
            return restore_error("Another flight-controller operation is already in progress");
        }
        let _guard = ApplyGuard(&self.applying);

        let (Some(mav_tx), Some(param_value_tx)) =
            (self.mav_tx.as_ref(), self.param_value_tx.as_ref())
        else {
            return restore_error("No flight controller connected (sim-only mode)");
        };

        match self.board_identity.read().await.clone() {
            Some(connected) if connected.as_str() == request.board_identity => {}
            Some(connected) => {
                return restore_error(format!(
                    "That snapshot was taken from a different flight controller ({}), not the \
                     one connected ({}). Nothing was changed.",
                    request.board_identity,
                    connected.as_str()
                ));
            }
            None => {
                return restore_error(
                    "The connected flight controller reports no identifying serial, so this \
                     snapshot cannot be confirmed to belong to it. Nothing was changed.",
                );
            }
        }

        if request.params.is_empty() {
            return restore_error("That snapshot contains no parameters to restore.");
        }

        self.broadcast_restore(RestoreState::Writing);
        for param in &request.params {
            // Written with the type the snapshot recorded: PX4 silently drops
            // a PARAM_SET whose type does not match, so guessing from the
            // value would leave parameters quietly unrestored.
            let is_int = param.param_type == "int32";
            let message = if is_int {
                make_param_set_i32(&param.name, param.value as i32)
            } else {
                make_param_set(&param.name, param.value)
            };

            let mut acked = false;
            for attempt in 1..=PARAM_RETRY_COUNT {
                let mut rx = param_value_tx.subscribe();
                // A full channel is backpressure, not a lost link, and it has
                // its own budget. A board that is already provisioned is being
                // streamed HIL sensors at 400Hz, so the tx queue is saturated
                // continuously — and that is exactly when a restore is wanted.
                // Sharing the ack-retry budget would exhaust it on queue
                // pressure before a single write ever went out.
                match send_with_backpressure(mav_tx, message.clone(), &param.name).await {
                    Ok(()) => {}
                    Err(e) => return restore_error(e),
                }
                // An integer ack comes back as the value's bit pattern, so it
                // has to be compared as bits rather than within a float epsilon.
                let matched = if is_int {
                    wait_for_int_param_ack(&mut rx, &param.name, param.value as i32).await
                } else {
                    wait_for_param_ack(&mut rx, &param.name, param.value)
                        .await
                        .is_some()
                };
                if matched {
                    acked = true;
                    break;
                }
                warn!(param = %param.name, attempt, "Restore ack timed out — retrying");
            }
            if !acked {
                return restore_error(format!(
                    "The flight controller did not confirm {}. Restore is incomplete.",
                    param.name
                ));
            }
        }

        // Never reboot without knowing the save was at least handed to the
        // writer. A dropped PARAM_SAVE means the values live only in PX4's RAM,
        // so the reboot below would silently discard everything just written —
        // and rebooting while the flash state is uncertain is how a board ends
        // up stuck in its bootloader.
        if let Err(e) = send_with_backpressure(mav_tx, make_param_save(), "PARAM_SAVE").await {
            return restore_error(format!(
                "{e} The values were written but not saved, and the flight controller was \
                 not rebooted."
            ));
        }
        tokio::time::sleep(PARAM_SAVE_SETTLE_DELAY).await;

        let baseline_count = self.sim_state.heartbeat_status().0;
        self.broadcast_restore(RestoreState::Rebooting);
        if let Err(e) = self.send_reboot_via_nsh().await {
            return restore_error(format!("Failed to send reboot command: {e}"));
        }

        self.broadcast_restore(RestoreState::Reconnecting);
        if !self
            .wait_for_reconnect(
                baseline_count,
                PREFLIGHT_RECONNECT_TIMEOUT,
                PREFLIGHT_RECONNECT_POLL_INTERVAL,
                PREFLIGHT_QUIET_PERIOD,
            )
            .await
        {
            return restore_error(
                "The flight controller did not come back after the restore reboot. The \
                 parameters were written, but could not be confirmed.",
            );
        }

        self.broadcast_restore(RestoreState::Verifying);
        let names: Vec<&str> = request.params.iter().map(|p| p.name.as_str()).collect();
        let (values, failed) =
            read_params_with(mav_tx, param_value_tx, &names, self.read_policy).await;

        if !failed.is_empty() {
            return restore_error(format!(
                "Could not read back {} parameters after the restore ({}).",
                failed.len(),
                failed.join(", ")
            ));
        }

        let mut mismatches = Vec::new();
        for expected in &request.params {
            if let Some(actual) = values.iter().find(|v| v.name == expected.name) {
                let actual_value = actual.decoded_value();
                if (actual_value - expected.value).abs() > PARAM_ACK_EPSILON {
                    mismatches.push(RestoreMismatch {
                        name: expected.name.clone(),
                        expected: expected.value,
                        actual: actual_value,
                    });
                }
            }
        }

        if !mismatches.is_empty() {
            // Deliberately not reported as restored: the user needs to know
            // exactly which values differ before trusting the aircraft.
            return RestoreResult {
                state: RestoreState::Error,
                success: false,
                error: Some(format!(
                    "{} parameter(s) did not read back as expected. The flight controller is \
                     not fully restored.",
                    mismatches.len()
                )),
                mismatches,
            };
        }

        RestoreResult {
            state: RestoreState::Done,
            success: true,
            error: None,
            mismatches: Vec::new(),
        }
    }

    /// Whether the board is still settling from the previous write cycle.
    ///
    /// Returns the remaining wait, or `None` when it is safe to proceed.
    fn cooldown_remaining(&self) -> Option<Duration> {
        let guard = self
            .last_cycle_finished
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let finished = (*guard)?;
        let elapsed = finished.elapsed();
        if elapsed >= self.cycle_cooldown {
            None
        } else {
            Some(self.cycle_cooldown - elapsed)
        }
    }

    /// Stamp the end of a write cycle, opening the cooldown window.
    fn mark_cycle_finished(&self) {
        *self
            .last_cycle_finished
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(std::time::Instant::now());
    }

    fn broadcast_progress(&self, state: PreflightApplyState) {
        let _ = self
            .provisioning_tx
            .send(OutgoingMessage::PreflightApplyResult(PreflightApplyResult {
                state,
                success: true,
                error: None,
            }));
    }

    fn broadcast_restore(&self, state: RestoreState) {
        let _ = self
            .provisioning_tx
            .send(OutgoingMessage::RestoreResult(RestoreResult {
                state,
                success: true,
                error: None,
                mismatches: Vec::new(),
            }));
    }

    /// One write-save-reboot-verify cycle.
    ///
    /// Split out of `apply()` so a verification failure can be retried without
    /// re-capturing the snapshot: the restore point is already confirmed, and
    /// re-reading the board after it has been provisioned would capture the
    /// modified values.
    async fn apply_once(
        &self,
        mav_tx: &Sender<MavMessage>,
        param_value_tx: &broadcast::Sender<ParamValue>,
    ) -> Result<(), ApplyFailure> {
        self.broadcast_progress(PreflightApplyState::Applying);

        for (name, value) in [("SYS_HITL", 1i32), ("SYS_AUTOSTART", QUADROTOR_AUTOSTART_ID)]
            .into_iter()
            .chain(HITL_SUPPORT_PARAMS_I32.iter().copied())
        {
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
                        return Err(ApplyFailure::fatal(format!("MAVLink tx disconnected while sending {name}")));
                    }
                }

                if wait_for_int_param_ack(&mut rx, name, value).await {
                    acked = true;
                    break;
                }
                warn!(param = name, attempt, "Preflight PARAM_VALUE ack timed out — retrying");
            }

            if !acked {
                return Err(ApplyFailure::fatal(format!(
                        "Failed to verify {name} after {PARAM_RETRY_COUNT} retries"
                    )));
            }
        }

        for &(name, value) in HITL_SUPPORT_PARAMS_F32 {
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
                        return Err(ApplyFailure::fatal(format!("MAVLink tx disconnected while sending {name}")));
                    }
                }

                if wait_for_param_ack(&mut rx, name, value).await.is_some() {
                    acked = true;
                    break;
                }
                warn!(param = name, attempt, "Preflight PARAM_VALUE ack timed out — retrying");
            }

            if !acked {
                return Err(ApplyFailure::fatal(format!(
                        "Failed to verify {name} after {PARAM_RETRY_COUNT} retries"
                    )));
            }
        }

        // A warning here used to be the whole response, and the reboot went
        // ahead regardless. That reboots a board whose flash state is unknown,
        // which is exactly the condition that leaves one stuck in its
        // bootloader — and at best silently discards the parameters just
        // pushed, since PARAM_SET only writes RAM.
        if let Err(e) = send_with_backpressure(mav_tx, make_param_save(), "PARAM_SAVE").await {
            return Err(ApplyFailure::fatal(format!(
                "{e} The parameters were written but not saved, and the flight controller \
                 was not rebooted."
            )));
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

        self.broadcast_progress(PreflightApplyState::Rebooting);
        if let Err(e) = self.send_reboot_via_nsh().await {
            return Err(ApplyFailure::fatal(format!("Failed to send reboot command: {e}")));
        }

        self.broadcast_progress(PreflightApplyState::Reconnecting);
        if !self
            .wait_for_reconnect(
                baseline_count,
                PREFLIGHT_RECONNECT_TIMEOUT,
                PREFLIGHT_RECONNECT_POLL_INTERVAL,
                PREFLIGHT_QUIET_PERIOD,
            )
            .await
        {
            return Err(ApplyFailure::fatal("FC did not reconnect after reboot".to_string()));
        }

        // Re-read the flags a few times instead of trusting the single
        // HEARTBEAT that ended the reconnect wait: PX4's first post-reboot
        // HEARTBEAT can plausibly precede its own internal state
        // publication, and a lone false reading here is otherwise a hard
        // failure with no automatic recovery.
        self.broadcast_progress(PreflightApplyState::Verifying);
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
            return Err(ApplyFailure::retryable(
                "Settings did not take effect after reboot",
            ));
        }

        Ok(())
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

/// Attempts before giving up on a saturated MAVLink tx queue.
///
/// Generous because the queue drains continuously: the daemon is writing HIL
/// sensors at 400Hz, so a slot appears within milliseconds. The cap exists to
/// bound a genuinely wedged writer, not to ration normal contention.
const RESTORE_SEND_ATTEMPTS: u8 = 40;
const RESTORE_SEND_BACKOFF: Duration = Duration::from_millis(25);

/// Push one message onto the MAVLink queue, waiting out backpressure.
async fn send_with_backpressure(
    mav_tx: &Sender<MavMessage>,
    message: MavMessage,
    param_name: &str,
) -> Result<(), String> {
    for attempt in 1..=RESTORE_SEND_ATTEMPTS {
        match mav_tx.try_send(message.clone()) {
            Ok(()) => return Ok(()),
            Err(crossbeam_channel::TrySendError::Full(_)) => {
                if attempt % 10 == 0 {
                    warn!(param = param_name, attempt, "MAVLink tx still full during restore");
                }
                tokio::time::sleep(RESTORE_SEND_BACKOFF).await;
            }
            Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                return Err(format!(
                    "Lost the link to the flight controller while restoring {param_name}"
                ));
            }
        }
    }
    Err(format!(
        "The flight controller is not accepting writes ({param_name}). Restore is incomplete."
    ))
}

fn restore_error(message: impl Into<String>) -> RestoreResult {
    RestoreResult {
        state: RestoreState::Error,
        success: false,
        error: Some(message.into()),
        mismatches: Vec::new(),
    }
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
    rx: &mut broadcast::Receiver<ParamValue>,
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
            Ok(Ok(pv)) => {
                // PX4 carries an INT32 parameter as the raw bit pattern of the
                // float field, so the comparison reinterprets rather than casts.
                if pv.name == name && pv.value.to_bits() as i32 == expected {
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
        param_value_tx: Option<broadcast::Sender<ParamValue>>,
        sim_state: SimulationState,
    ) -> PreflightHandler {
        // A fire-and-forget NSH channel with a task draining it forever, so
        // send_reboot_via_nsh's send succeeds without needing a real NSH
        // handler task — apply() never awaits a response to the reboot.
        let (nsh_tx, mut nsh_rx) = mpsc::channel::<ValidatedNshCommand>(4);
        tokio::spawn(async move { while nsh_rx.recv().await.is_some() {} });
        PreflightHandler::with_identity(
            mav_tx,
            param_value_tx,
            Some(nsh_tx),
            sim_state,
            Arc::new(tokio::sync::RwLock::new(Some(BoardIdentity::from_raw_for_test(
                "uid:testboard",
            )))),
        )
        .with_fast_capture()
    }

    /// Drain progress and acknowledge the snapshot, standing in for a browser
    /// that persisted it. Tests exercising the write path need this: without
    /// an acknowledgement provisioning correctly refuses to write at all.
    fn auto_ack_snapshots(handler: &PreflightHandler) -> tokio::task::JoinHandle<()> {
        let ack_tx = handler.snapshot_ack_sender();
        let mut progress_rx = handler.subscribe_provisioning();
        tokio::spawn(async move {
            while let Ok(msg) = progress_rx.recv().await {
                if let OutgoingMessage::SnapshotCaptured(captured) = msg {
                    let _ = ack_tx.send(SnapshotStored {
                        board_identity: captured.board_identity,
                        stored: true,
                        error: None,
                    });
                }
            }
        })
    }

    #[derive(Debug, Clone)]
    enum CapturedMsg {
        ParamSet(String, f32, MavParamType),
        CommandLong(u32, Instant),
    }

    fn spawn_fake_px4(
        mav_rx: crossbeam_channel::Receiver<MavMessage>,
        param_value_tx: broadcast::Sender<ParamValue>,
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
                            .push(CapturedMsg::ParamSet(name.clone(), p.param_value, p.param_type));
                        let _ = param_value_tx.send(ParamValue {
                        name,
                        value: p.param_value,
                        param_type: p.param_type,
                        index: 0,
                    });
                    }
                    MavMessage::COMMAND_LONG(c) => {
                        captured_clone
                            .lock()
                            .unwrap()
                            .push(CapturedMsg::CommandLong(c.command as u32, Instant::now()));
                    }
                    // Snapshot capture reads every parameter provisioning will
                    // write before any write is allowed, so the fake board has
                    // to answer reads or nothing downstream is reachable.
                    MavMessage::PARAM_REQUEST_READ(req) => {
                        let name = crate::param_io::decode_param_id(&req.param_id);
                        if !name.is_empty() {
                            let _ = param_value_tx.send(ParamValue {
                                name,
                                value: 0.0,
                                param_type: MavParamType::MAV_PARAM_TYPE_INT32,
                                index: 0,
                            });
                        }
                    }
                    _ => {}
                }
            }
        });
        (handle, captured)
    }

    /// Answers PARAM_REQUEST_READ but never acks a PARAM_SET, so snapshot
    /// capture succeeds and the write path is the thing under test.
    fn spawn_read_only_px4(
        mav_rx: crossbeam_channel::Receiver<MavMessage>,
        param_value_tx: broadcast::Sender<ParamValue>,
    ) -> tokio::task::JoinHandle<()> {
        tokio::task::spawn_blocking(move || {
            while let Ok(msg) = mav_rx.recv() {
                if let MavMessage::PARAM_REQUEST_READ(req) = msg {
                    let name = crate::param_io::decode_param_id(&req.param_id);
                    if !name.is_empty() {
                        let _ = param_value_tx.send(ParamValue {
                            name,
                            value: 0.0,
                            param_type: MavParamType::MAV_PARAM_TYPE_INT32,
                            index: 0,
                        });
                    }
                }
            }
        })
    }

    #[tokio::test]
    async fn sim_only_mode_returns_error_immediately() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        let handler = make_handler(None, None, sim_state);
        let result = handler.apply().await;
        assert!(!result.success);
        assert!(matches!(result.state, PreflightApplyState::Error));
        assert!(result.error.unwrap().contains("sim-only"));
    }

    #[tokio::test]
    async fn happy_path_applies_reboots_and_verifies() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
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
            // Comfortably past PARAM_SAVE_SETTLE_DELAY (2s, now part of
            // apply()'s pre-reboot sequence) + PREFLIGHT_QUIET_PERIOD (2s)
            // from whenever wait_for_reconnect actually starts polling, plus
            // the same ~200ms margin the original 2200ms value carried.
            tokio::time::sleep(Duration::from_millis(4200)).await;
            sim_state_for_reboot.set_heartbeat_status(true, true);
        });

        let handler = PreflightHandler::with_identity(
            Some(mav_tx),
            Some(pv_tx),
            Some(nsh_tx),
            sim_state,
            Arc::new(tokio::sync::RwLock::new(Some(BoardIdentity::from_raw_for_test(
                "uid:testboard",
            )))),
        )
        .with_fast_capture();
        let ack_tx = handler.snapshot_ack_sender();
        let mut progress_rx = handler.subscribe_provisioning();
        let apply_task = tokio::spawn(async move { handler.apply().await });

        let mut stages = Vec::new();
        loop {
            let Ok(msg) = progress_rx.recv().await else {
                break;
            };
            match msg {
                // Stand in for a browser that persisted the restore point.
                // Without this the apply correctly refuses to write anything.
                OutgoingMessage::SnapshotCaptured(captured) => {
                    let _ = ack_tx.send(SnapshotStored {
                        board_identity: captured.board_identity,
                        stored: true,
                        error: None,
                    });
                }
                OutgoingMessage::PreflightApplyResult(r) => {
                    stages.push(format!("{:?}", r.state));
                    if matches!(r.state, PreflightApplyState::Verifying) {
                        break;
                    }
                }
                _ => {}
            }
        }

        let result = apply_task.await.unwrap();
        assert!(result.success, "apply() failed: {:?}", result.error);
        assert!(matches!(result.state, PreflightApplyState::Done));

        let snapshot = captured.lock().unwrap().clone();
        let param_names: Vec<&str> = snapshot
            .iter()
            .filter_map(|m| match m {
                CapturedMsg::ParamSet(n, _, _) => Some(n.as_str()),
                _ => None,
            })
            .collect();
        assert!(param_names.contains(&"SYS_HITL"));
        assert!(param_names.contains(&"SYS_AUTOSTART"));

        // Every HITL support param must go out, and with the type the
        // backup file's own formatting says PX4 expects — the same
        // int-vs-real mismatch silently dropped SYS_HITL/SYS_AUTOSTART
        // earlier this session (see make_param_set_i32's doc comment).
        for &(name, _) in HITL_SUPPORT_PARAMS_I32 {
            let ty = snapshot.iter().find_map(|m| match m {
                CapturedMsg::ParamSet(n, _, t) if n == name => Some(*t),
                _ => None,
            });
            assert_eq!(
                ty,
                Some(MavParamType::MAV_PARAM_TYPE_INT32),
                "{name} must be pushed as INT32"
            );
        }
        for &(name, _) in HITL_SUPPORT_PARAMS_F32 {
            let ty = snapshot.iter().find_map(|m| match m {
                CapturedMsg::ParamSet(n, _, t) if n == name => Some(*t),
                _ => None,
            });
            assert_eq!(
                ty,
                Some(MavParamType::MAV_PARAM_TYPE_REAL32),
                "{name} must be pushed as REAL32"
            );
        }

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
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
        // Reads are answered so the snapshot completes; PARAM_SETs are never
        // acked, which is the failure under test.
        let _px4 = spawn_read_only_px4(mav_rx, pv_tx.clone());
        let sim_state = SimulationState::new(SimulationConfig::default());
        let handler = make_handler(Some(mav_tx), Some(pv_tx), sim_state);
        let _acker = auto_ack_snapshots(&handler);

        let result = handler.apply().await;
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
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
        let (_px4, _captured) = spawn_fake_px4(mav_rx, pv_tx.clone());

        let sim_state = SimulationState::new(SimulationConfig::default());
        // An FC is connected but misconfigured — exactly what triggers apply().
        sim_state.set_heartbeat_status(false, false);

        let handler =
            std::sync::Arc::new(make_handler(Some(mav_tx), Some(pv_tx), sim_state.clone()));
        let apply_handler = handler.clone();
        let mut progress_rx = handler.subscribe_provisioning();
        let ack_tx = handler.snapshot_ack_sender();
        let apply_task = tokio::spawn(async move { apply_handler.apply().await });
        // Wait until apply() has actually entered the reconnect wait,
        // acknowledging the snapshot on the way so the write path is reachable.
        loop {
            let msg = progress_rx
                .recv()
                .await
                .expect("progress broadcast closed before Reconnecting");
            if let OutgoingMessage::SnapshotCaptured(captured) = &msg {
                let _ = ack_tx.send(SnapshotStored {
                    board_identity: captured.board_identity.clone(),
                    stored: true,
                    error: None,
                });
                continue;
            }
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
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
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
            // Comfortably past PARAM_SAVE_SETTLE_DELAY (2s, now part of
            // apply()'s pre-reboot sequence) + PREFLIGHT_QUIET_PERIOD (2s)
            // from whenever wait_for_reconnect actually starts polling, plus
            // the same ~200ms margin the original 2200ms value carried.
            tokio::time::sleep(Duration::from_millis(4200)).await;
            sim_state_for_reboot.set_heartbeat_status(true, true);
        });

        let apply_handler = handler.clone();
        let apply_task = tokio::spawn(async move { apply_handler.apply().await });
        // Drain progress so a full channel can never stall the apply task, and
        // acknowledge the snapshot: without it the apply correctly refuses to
        // write anything at all.
        let _acker = auto_ack_snapshots(&handler);

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
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
        let sim_state = SimulationState::new(SimulationConfig::default());
        let handler = std::sync::Arc::new(make_handler(Some(mav_tx), Some(pv_tx), sim_state));

        let first_handler = handler.clone();
        let first = tokio::spawn(async move { first_handler.apply().await });

        // Give the first call a moment to acquire the guard.
        tokio::time::sleep(Duration::from_millis(20)).await;
        let second = handler.apply().await;
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
        let third = handler.apply().await;
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

#[cfg(test)]
mod readiness_tests {
    use super::*;
    use crossbeam_channel::bounded;
    use simulation::{SimulationConfig, SimulationState};

    fn handler_with_fc(sim_state: SimulationState) -> PreflightHandler {
        let (mav_tx, _mav_rx) = bounded::<MavMessage>(8);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(8);
        let (nsh_tx, _nsh_rx) = mpsc::channel(8);
        PreflightHandler::new(Some(mav_tx), Some(pv_tx), Some(nsh_tx), sim_state)
    }

    fn handler_sim_only(sim_state: SimulationState) -> PreflightHandler {
        PreflightHandler::new(None, None, None, sim_state)
    }

    #[tokio::test]
    async fn a_board_that_has_not_reported_is_unknown_not_not_ready() {
        // The regression this exists to prevent: "no heartbeat yet" used to be
        // indistinguishable from "no FC", and the browser fell straight through
        // the gate while the board was still booting.
        let sim_state = SimulationState::new(SimulationConfig::default());
        let status = handler_with_fc(sim_state).check().await;
        assert_eq!(status.readiness, PreflightReadiness::Unknown);
        assert!(!status.connected);
    }

    #[tokio::test]
    async fn sim_only_is_not_applicable_rather_than_unknown() {
        // Nothing to wait for: there is no board in this session at all, so
        // holding the flow would hang it forever.
        let sim_state = SimulationState::new(SimulationConfig::default());
        let status = handler_sim_only(sim_state).check().await;
        assert_eq!(status.readiness, PreflightReadiness::NotApplicable);
    }

    #[tokio::test]
    async fn both_signals_correct_reports_ready() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        sim_state.set_heartbeat_status(true, true);
        let status = handler_with_fc(sim_state).check().await;
        assert_eq!(status.readiness, PreflightReadiness::Ready);
    }

    #[tokio::test]
    async fn hitl_disabled_reports_not_ready() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        sim_state.set_heartbeat_status(false, true);
        let status = handler_with_fc(sim_state).check().await;
        assert_eq!(status.readiness, PreflightReadiness::NotReady);
    }

    #[tokio::test]
    async fn non_quadrotor_reports_not_ready() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        sim_state.set_heartbeat_status(true, false);
        let status = handler_with_fc(sim_state).check().await;
        assert_eq!(status.readiness, PreflightReadiness::NotReady);
    }

    #[tokio::test]
    async fn unknown_is_never_equal_to_ready() {
        // Guards the collapse this whole field exists to prevent.
        assert_ne!(PreflightReadiness::Unknown, PreflightReadiness::Ready);
        assert_ne!(PreflightReadiness::NotApplicable, PreflightReadiness::Ready);
    }
}

#[cfg(test)]
mod snapshot_gate_tests {
    use super::*;
    use crossbeam_channel::bounded;
    use simulation::{SimulationConfig, SimulationState};
    use std::sync::Mutex;

    /// Counts PARAM_SET messages reaching the FC. The assertion every test here
    /// makes is that this stays at zero when the snapshot does not complete.
    fn spawn_counting_px4(
        mav_rx: crossbeam_channel::Receiver<MavMessage>,
        param_value_tx: broadcast::Sender<ParamValue>,
        answer_reads: bool,
    ) -> Arc<Mutex<usize>> {
        let writes = Arc::new(Mutex::new(0usize));
        let writes_clone = writes.clone();
        tokio::task::spawn_blocking(move || {
            while let Ok(msg) = mav_rx.recv() {
                match msg {
                    MavMessage::PARAM_SET(_) => {
                        *writes_clone.lock().unwrap() += 1;
                    }
                    MavMessage::PARAM_REQUEST_READ(req) if answer_reads => {
                        let name = crate::param_io::decode_param_id(&req.param_id);
                        let pv = mavlink::ardupilotmega::PARAM_VALUE_DATA {
                            param_value: 0.0,
                            param_count: 1,
                            param_index: 0,
                            param_id: req.param_id,
                            param_type: mavlink::ardupilotmega::MavParamType::MAV_PARAM_TYPE_INT32,
                        };
                        if !name.is_empty() {
                            let _ = param_value_tx.send(ParamValue::from_mavlink(&pv).unwrap());
                        }
                    }
                    _ => {}
                }
            }
        });
        writes
    }

    fn handler(
        mav_tx: Sender<MavMessage>,
        pv_tx: broadcast::Sender<ParamValue>,
        identity: Option<&str>,
    ) -> PreflightHandler {
        let (nsh_tx, _nsh_rx) = mpsc::channel(8);
        let id = identity.map(|s| BoardIdentity::from_raw_for_test(s));
        PreflightHandler::with_identity(
            Some(mav_tx),
            Some(pv_tx),
            Some(nsh_tx),
            SimulationState::new(SimulationConfig::default()),
            Arc::new(tokio::sync::RwLock::new(id)),
        )
        .with_fast_capture()
    }

    #[tokio::test]
    async fn no_write_happens_when_parameters_cannot_be_read() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(256);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(256);
        // answer_reads = false: every read times out, so capture fails.
        let writes = spawn_counting_px4(mav_rx, pv_tx.clone(), false);

        let h = handler(mav_tx, pv_tx, Some("uid:aaaa"));
        let result = h.apply().await;

        assert!(!result.success);
        assert_eq!(
            *writes.lock().unwrap(),
            0,
            "the board must not be modified when its parameters could not be captured"
        );
    }

    #[tokio::test]
    async fn no_write_happens_when_the_browser_never_acknowledges() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(256);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(256);
        let writes = spawn_counting_px4(mav_rx, pv_tx.clone(), true);

        let h = handler(mav_tx, pv_tx, Some("uid:aaaa"));
        // A browser is listening — so the snapshot is delivered — but it never
        // confirms it stored anything.
        let mut listening = h.subscribe_provisioning();
        tokio::spawn(async move { while listening.recv().await.is_ok() {} });

        let result = tokio::time::timeout(Duration::from_secs(40), h.apply())
            .await
            .unwrap();

        assert!(!result.success);
        assert!(result.error.unwrap().contains("did not confirm"));
        assert_eq!(*writes.lock().unwrap(), 0);
    }

    #[tokio::test]
    async fn no_write_happens_when_the_browser_reports_a_storage_failure() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(256);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(256);
        let writes = spawn_counting_px4(mav_rx, pv_tx.clone(), true);

        let h = Arc::new(handler(mav_tx, pv_tx, Some("uid:aaaa")));
        let ack_tx = h.snapshot_ack_sender();

        let mut progress_rx = h.subscribe_provisioning();
        let responder = tokio::spawn(async move {
            while let Ok(msg) = progress_rx.recv().await {
                if let OutgoingMessage::SnapshotCaptured(captured) = msg {
                    let _ = ack_tx.send(SnapshotStored {
                        board_identity: captured.board_identity,
                        stored: false,
                        error: Some("QuotaExceededError".to_string()),
                    });
                    return;
                }
            }
        });

        let result = h.apply().await;
        let _ = responder.await;

        assert!(!result.success);
        assert!(result.error.unwrap().contains("QuotaExceededError"));
        assert_eq!(
            *writes.lock().unwrap(),
            0,
            "a browser that could not save the restore point must stop provisioning"
        );
    }

    #[tokio::test]
    async fn no_write_happens_when_the_client_disconnects_mid_handoff() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(256);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(256);
        let writes = spawn_counting_px4(mav_rx, pv_tx.clone(), true);

        let h = handler(mav_tx, pv_tx, Some("uid:aaaa"));
        // No browser is subscribed, standing in for the tab going away before
        // the snapshot can be delivered.
        let result = h.apply().await;
        assert!(!result.success);
        assert_eq!(*writes.lock().unwrap(), 0);
    }

    #[tokio::test]
    async fn the_default_constructor_leaves_identity_unwired() {
        // Regression: main.rs built the handler with `new`, which creates its
        // own empty identity cell, while the receiver task populated a
        // different one. Provisioning therefore refused every board on real
        // hardware for want of an identity it was never told about — and every
        // unit test passed, because they all call `with_identity` directly.
        let (mav_tx, _rx) = bounded::<MavMessage>(8);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(8);
        let (nsh_tx, _nsh_rx) = mpsc::channel(8);
        let handler = PreflightHandler::new(
            Some(mav_tx),
            Some(pv_tx),
            Some(nsh_tx),
            SimulationState::new(SimulationConfig::default()),
        );
        // `new` is only safe where nothing needs identity. Anything that
        // provisions must use `with_identity` and share main.rs's cell.
        assert!(
            handler.board_identity.read().await.is_none(),
            "new() must not fabricate an identity"
        );
    }

    #[tokio::test]
    async fn a_shared_identity_cell_reaches_the_handler() {
        // The property the production wiring depends on: whatever the receiver
        // task writes into the shared cell is what provisioning reads.
        let cell = Arc::new(tokio::sync::RwLock::new(None));
        let (mav_tx, _rx) = bounded::<MavMessage>(8);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(8);
        let (nsh_tx, _nsh_rx) = mpsc::channel(8);
        let handler = PreflightHandler::with_identity(
            Some(mav_tx),
            Some(pv_tx),
            Some(nsh_tx),
            SimulationState::new(SimulationConfig::default()),
            cell.clone(),
        );

        *cell.write().await = Some(BoardIdentity::from_raw_for_test("uid:3034510f33323831"));

        assert_eq!(
            handler.board_identity.read().await.as_ref().map(|b| b.as_str().to_string()),
            Some("uid:3034510f33323831".to_string())
        );
    }

    #[tokio::test]
    async fn no_write_happens_without_a_board_identity() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(256);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(256);
        let writes = spawn_counting_px4(mav_rx, pv_tx.clone(), true);

        // A board with no distinguishing serial: a snapshot could not be tied
        // back to it, so provisioning must not start.
        let h = handler(mav_tx, pv_tx, None);
        let result = h.apply().await;

        assert!(!result.success);
        assert!(result.error.unwrap().contains("identifying serial"));
        assert_eq!(*writes.lock().unwrap(), 0);
    }

    #[tokio::test]
    async fn the_captured_snapshot_carries_every_parameter_provisioning_writes() {
        // A parameter written but not captured would be unrecoverable, so the
        // two lists have to stay in step.
        let names = PreflightHandler::provisioned_param_names();
        assert!(names.contains(&"SYS_HITL"));
        assert!(names.contains(&"SYS_AUTOSTART"));
        for (name, _) in HITL_SUPPORT_PARAMS_I32 {
            assert!(names.contains(name), "{name} is written but not captured");
        }
        for (name, _) in HITL_SUPPORT_PARAMS_F32 {
            assert!(names.contains(name), "{name} is written but not captured");
        }
    }
}

#[cfg(test)]
mod reapply_tests {
    use super::*;
    use crossbeam_channel::bounded;
    use simulation::{SimulationConfig, SimulationState};
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// Answers reads and acks writes. Counts reboots so a test can make the
    /// board "fail to take" the first time and succeed the second.
    fn spawn_px4(
        mav_rx: crossbeam_channel::Receiver<MavMessage>,
        param_value_tx: broadcast::Sender<ParamValue>,
    ) -> Arc<AtomicUsize> {
        let writes = Arc::new(AtomicUsize::new(0));
        let writes_clone = writes.clone();
        tokio::task::spawn_blocking(move || {
            while let Ok(msg) = mav_rx.recv() {
                match msg {
                    MavMessage::PARAM_SET(p) => {
                        writes_clone.fetch_add(1, Ordering::SeqCst);
                        let name = crate::param_io::decode_param_id(&p.param_id);
                        let _ = param_value_tx.send(ParamValue {
                            name,
                            value: p.param_value,
                            param_type: p.param_type,
                            index: 0,
                        });
                    }
                    MavMessage::PARAM_REQUEST_READ(req) => {
                        let name = crate::param_io::decode_param_id(&req.param_id);
                        if !name.is_empty() {
                            let _ = param_value_tx.send(ParamValue {
                                name,
                                value: 0.0,
                                param_type: MavParamType::MAV_PARAM_TYPE_INT32,
                                index: 0,
                            });
                        }
                    }
                    _ => {}
                }
            }
        });
        writes
    }

    fn fast_handler(sim_state: SimulationState) -> (PreflightHandler, Arc<AtomicUsize>) {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(512);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(512);
        let writes = spawn_px4(mav_rx, pv_tx.clone());
        let (nsh_tx, mut nsh_rx) = mpsc::channel::<ValidatedNshCommand>(8);
        tokio::spawn(async move { while nsh_rx.recv().await.is_some() {} });

        let handler = PreflightHandler::with_identity(
            Some(mav_tx),
            Some(pv_tx),
            Some(nsh_tx),
            sim_state,
            Arc::new(tokio::sync::RwLock::new(Some(BoardIdentity::from_raw_for_test(
                "uid:testboard",
            )))),
        )
        .with_fast_capture();
        (handler, writes)
    }

    #[test]
    fn only_a_verification_failure_is_retryable() {
        // Pushing the same values again cannot fix an unacked PARAM_SET or a
        // board that never came back, and retrying would just write flash.
        assert!(ApplyFailure::retryable("did not take effect").retryable);
        assert!(!ApplyFailure::fatal("FC did not reconnect after reboot").retryable);
    }

    #[test]
    fn the_retry_budget_is_bounded() {
        // Each cycle commits parameters to flash; an unbounded retry on a board
        // that will never verify would wear it out.
        assert!(PROVISION_ATTEMPTS >= 2, "one attempt would defeat the point");
        assert!(PROVISION_ATTEMPTS <= 3, "flash wear grows with every cycle");
    }

    #[tokio::test]
    async fn a_failed_verification_is_re_applied_and_can_then_succeed() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        let (handler, writes) = fast_handler(sim_state.clone());
        let handler = Arc::new(handler);
        let ack_tx = handler.snapshot_ack_sender();
        let sim_for_reboots = sim_state.clone();

        // Models a board that comes back wrong the first time and correct the
        // second. Each Reconnecting stage bumps the heartbeat counter so the
        // reconnect wait completes.
        let mut progress_rx = handler.subscribe_provisioning();
        let driver = tokio::spawn(async move {
            let mut reconnects = 0;
            while let Ok(msg) = progress_rx.recv().await {
                match msg {
                    OutgoingMessage::SnapshotCaptured(captured) => {
                        let _ = ack_tx.send(SnapshotStored {
                            board_identity: captured.board_identity,
                            stored: true,
                            error: None,
                        });
                    }
                    OutgoingMessage::PreflightApplyResult(r)
                        if matches!(r.state, PreflightApplyState::Reconnecting) =>
                    {
                        reconnects += 1;
                        let took_effect = reconnects >= 2;
                        let sim = sim_for_reboots.clone();
                        tokio::spawn(async move {
                            tokio::time::sleep(Duration::from_millis(4200)).await;
                            sim.set_heartbeat_status(took_effect, took_effect);
                        });
                    }
                    _ => {}
                }
            }
            reconnects
        });

        let result = handler.apply().await;
        drop(handler);
        let reconnects = driver.await.unwrap();

        assert!(result.success, "re-apply should have succeeded: {:?}", result.error);
        assert_eq!(reconnects, 2, "the board should have been provisioned twice");
        // Two full parameter pushes, not one: the second cycle re-wrote them.
        assert!(
            writes.load(Ordering::SeqCst) > PreflightHandler::provisioned_param_names().len(),
            "the second attempt must actually re-push the parameters"
        );
    }
}

#[cfg(test)]
mod restore_tests {
    use super::*;
    use crossbeam_channel::bounded;
    use simulation::{SimulationConfig, SimulationState};
    use std::collections::HashMap;
    use std::sync::Mutex;

    /// A board that remembers what was written to it, so a restore can be
    /// checked against what it actually now holds.
    fn spawn_stateful_px4(
        mav_rx: crossbeam_channel::Receiver<MavMessage>,
        param_value_tx: broadcast::Sender<ParamValue>,
        // Parameters this board refuses to actually change, modelling a value
        // that does not survive the write.
        stuck: Vec<&'static str>,
    ) -> Arc<Mutex<HashMap<String, f32>>> {
        let state = Arc::new(Mutex::new(HashMap::<String, f32>::new()));
        let state_clone = state.clone();
        tokio::task::spawn_blocking(move || {
            while let Ok(msg) = mav_rx.recv() {
                match msg {
                    MavMessage::PARAM_SET(p) => {
                        let name = crate::param_io::decode_param_id(&p.param_id);
                        if !stuck.contains(&name.as_str()) {
                            state_clone.lock().unwrap().insert(name.clone(), p.param_value);
                        }
                        // PX4 acks with the value it was asked to set, even
                        // when the stored value ends up different.
                        let _ = param_value_tx.send(ParamValue {
                            name,
                            value: p.param_value,
                            param_type: p.param_type,
                            index: 0,
                        });
                    }
                    MavMessage::PARAM_REQUEST_READ(req) => {
                        let name = crate::param_io::decode_param_id(&req.param_id);
                        if name.is_empty() {
                            continue;
                        }
                        let value = *state_clone.lock().unwrap().get(&name).unwrap_or(&0.0);
                        let _ = param_value_tx.send(ParamValue {
                            name,
                            value,
                            param_type: MavParamType::MAV_PARAM_TYPE_INT32,
                            index: 0,
                        });
                    }
                    _ => {}
                }
            }
        });
        state
    }

    fn handler_for(
        identity: Option<&str>,
        stuck: Vec<&'static str>,
        sim_state: SimulationState,
    ) -> (Arc<PreflightHandler>, Arc<Mutex<HashMap<String, f32>>>) {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(512);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(512);
        let board = spawn_stateful_px4(mav_rx, pv_tx.clone(), stuck);
        let (nsh_tx, mut nsh_rx) = mpsc::channel::<ValidatedNshCommand>(8);
        tokio::spawn(async move { while nsh_rx.recv().await.is_some() {} });

        let handler = PreflightHandler::with_identity(
            Some(mav_tx),
            Some(pv_tx),
            Some(nsh_tx),
            sim_state,
            Arc::new(tokio::sync::RwLock::new(
                identity.map(BoardIdentity::from_raw_for_test),
            )),
        )
        .with_fast_capture();
        (Arc::new(handler), board)
    }

    fn request(board: &str) -> RestoreSnapshot {
        RestoreSnapshot {
            board_identity: board.to_string(),
            params: vec![
                SnapshotParam {
                    name: "SYS_HITL".to_string(),
                    value: 0.0,
                    param_type: "int32".to_string(),
                },
                SnapshotParam {
                    name: "COM_ARM_SDCARD".to_string(),
                    value: 1.0,
                    param_type: "int32".to_string(),
                },
            ],
        }
    }

    /// Advance the fake board through the reboot so the reconnect wait ends.
    fn drive_reconnect(sim_state: SimulationState, mut rx: broadcast::Receiver<OutgoingMessage>) {
        tokio::spawn(async move {
            while let Ok(msg) = rx.recv().await {
                if let OutgoingMessage::RestoreResult(r) = msg {
                    if matches!(r.state, RestoreState::Reconnecting) {
                        let sim = sim_state.clone();
                        tokio::spawn(async move {
                            tokio::time::sleep(Duration::from_millis(4200)).await;
                            sim.set_heartbeat_status(false, true);
                        });
                    }
                }
            }
        });
    }

    #[tokio::test]
    async fn restore_writes_every_parameter_and_confirms_it() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        let (handler, board) = handler_for(Some("uid:aaaa"), vec![], sim_state.clone());
        drive_reconnect(sim_state, handler.subscribe_provisioning());

        let result = handler.restore(request("uid:aaaa")).await;

        assert!(result.success, "restore failed: {:?}", result.error);
        assert!(matches!(result.state, RestoreState::Done));
        assert!(result.mismatches.is_empty());
        // The board stores what arrived on the wire, and PX4 carries an
        // integer as the bit pattern of the int32 inside the float field.
        let decode = |v: f32| v.to_bits() as i32;
        let held = board.lock().unwrap();
        assert_eq!(held.get("SYS_HITL").copied().map(decode), Some(0));
        assert_eq!(held.get("COM_ARM_SDCARD").copied().map(decode), Some(1));
    }

    #[tokio::test]
    async fn a_parameter_that_did_not_take_is_reported_and_the_board_is_not_called_restored() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        // COM_ARM_SDCARD acks but never actually changes.
        let (handler, _board) =
            handler_for(Some("uid:aaaa"), vec!["COM_ARM_SDCARD"], sim_state.clone());
        drive_reconnect(sim_state, handler.subscribe_provisioning());

        let result = handler.restore(request("uid:aaaa")).await;

        assert!(!result.success, "a board with a stuck parameter is not restored");
        assert_eq!(result.mismatches.len(), 1);
        let mismatch = &result.mismatches[0];
        assert_eq!(mismatch.name, "COM_ARM_SDCARD");
        assert_eq!(mismatch.expected, 1.0);
        assert_eq!(mismatch.actual, 0.0);
    }

    #[tokio::test]
    async fn a_snapshot_from_another_board_is_refused_without_writing() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        let (handler, board) = handler_for(Some("uid:aaaa"), vec![], sim_state);

        let result = handler.restore(request("uid:bbbb")).await;

        assert!(!result.success);
        assert!(result.error.unwrap().contains("different flight controller"));
        assert!(
            board.lock().unwrap().is_empty(),
            "nothing may be written when the snapshot belongs to another board"
        );
    }

    #[tokio::test]
    async fn restore_is_refused_when_the_board_has_no_identity() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        let (handler, board) = handler_for(None, vec![], sim_state);

        let result = handler.restore(request("uid:aaaa")).await;

        assert!(!result.success);
        assert!(result.error.unwrap().contains("no identifying serial"));
        assert!(board.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn restore_is_refused_in_sim_only_mode() {
        let handler = PreflightHandler::new(
            None,
            None,
            None,
            SimulationState::new(SimulationConfig::default()),
        );
        let result = handler.restore(request("uid:aaaa")).await;
        assert!(!result.success);
        assert!(result.error.unwrap().contains("sim-only"));
    }

    #[tokio::test]
    async fn an_empty_snapshot_is_refused() {
        let sim_state = SimulationState::new(SimulationConfig::default());
        let (handler, board) = handler_for(Some("uid:aaaa"), vec![], sim_state);

        let result = handler
            .restore(RestoreSnapshot {
                board_identity: "uid:aaaa".to_string(),
                params: vec![],
            })
            .await;

        assert!(!result.success);
        assert!(board.lock().unwrap().is_empty());
    }
}

#[cfg(test)]
mod broadcast_tests {
    use super::*;
    use crossbeam_channel::bounded;
    use simulation::{SimulationConfig, SimulationState};

    #[tokio::test]
    async fn every_subscribed_client_sees_the_same_progress() {
        // A second tab, or a page reloaded mid-provisioning, has to converge on
        // the same state. Previously progress went only to the connection that
        // asked, so anyone else saw nothing at all.
        let (mav_tx, _mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
        let handler = PreflightHandler::with_identity(
            Some(mav_tx),
            Some(pv_tx),
            None,
            SimulationState::new(SimulationConfig::default()),
            // No identity: apply fails immediately, which is enough to prove
            // both subscribers receive the same terminal frame.
            Arc::new(tokio::sync::RwLock::new(None)),
        )
        .with_fast_capture();

        let mut first = handler.subscribe_provisioning();
        let mut second = handler.subscribe_provisioning();

        let result = handler.apply().await;
        assert!(!result.success);

        let first_msg = first.recv().await.expect("first client receives");
        let second_msg = second.recv().await.expect("second client receives");

        let describe = |msg: &OutgoingMessage| match msg {
            OutgoingMessage::PreflightApplyResult(r) => format!("{:?}", r.state),
            other => format!("{other:?}"),
        };
        assert_eq!(describe(&first_msg), describe(&second_msg));
    }

    #[tokio::test]
    async fn a_client_that_subscribes_late_still_receives_the_terminal_result() {
        let (mav_tx, _mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
        let handler = Arc::new(
            PreflightHandler::with_identity(
                Some(mav_tx),
                Some(pv_tx),
                None,
                SimulationState::new(SimulationConfig::default()),
                Arc::new(tokio::sync::RwLock::new(None)),
            )
            .with_fast_capture(),
        );

        // Subscribes before the terminal frame but after the run began.
        let mut late = handler.subscribe_provisioning();
        let result = handler.apply().await;
        assert!(!result.success);

        let mut saw_terminal = false;
        while let Ok(msg) = late.try_recv() {
            if let OutgoingMessage::PreflightApplyResult(r) = msg {
                if matches!(r.state, PreflightApplyState::Error) {
                    saw_terminal = true;
                }
            }
        }
        assert!(saw_terminal, "a late subscriber must still learn how it ended");
    }
}

#[cfg(test)]
mod cooldown_tests {
    use super::*;
    use crossbeam_channel::bounded;
    use simulation::{SimulationConfig, SimulationState};

    /// A handler with no identity: apply and restore both refuse quickly, so
    /// these tests exercise the cooldown gate rather than a full write cycle.
    fn handler(cooldown: Duration) -> PreflightHandler {
        let (mav_tx, _rx) = bounded::<MavMessage>(8);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(8);
        let (nsh_tx, _nsh_rx) = mpsc::channel(8);
        PreflightHandler::with_identity(
            Some(mav_tx),
            Some(pv_tx),
            Some(nsh_tx),
            SimulationState::new(SimulationConfig::default()),
            Arc::new(tokio::sync::RwLock::new(None)),
        )
        .with_fast_capture()
        .with_cooldown(cooldown)
    }

    fn restore_request() -> RestoreSnapshot {
        RestoreSnapshot {
            board_identity: "uid:aaaa".to_string(),
            params: vec![SnapshotParam {
                name: "SYS_HITL".to_string(),
                value: 0.0,
                param_type: "int32".to_string(),
            }],
        }
    }

    #[tokio::test]
    async fn a_second_apply_is_refused_while_the_board_is_settling() {
        // The hardware failure this prevents: two write-and-reboot cycles in
        // quick succession interrupted a flash commit and left the board in
        // its bootloader, recoverable only by a physical power cycle.
        let h = handler(Duration::from_secs(30));
        let _first = h.apply().await;

        let second = h.apply().await;
        assert!(!second.success);
        let error = second.error.unwrap();
        assert!(error.contains("still restarting"), "{error}");
        // Tells the user how long to wait rather than just refusing.
        assert!(error.contains('s'), "{error}");
    }

    #[tokio::test]
    async fn a_restore_is_refused_inside_the_window_an_apply_opened() {
        // Both operations write and reboot, so the window is shared: a restore
        // landing on a settling board is just as damaging as a second apply.
        let h = handler(Duration::from_secs(30));
        let _ = h.apply().await;

        let restore = h.restore(restore_request()).await;
        assert!(!restore.success);
        assert!(restore.error.unwrap().contains("still restarting"));
    }

    #[tokio::test]
    async fn an_apply_is_refused_inside_the_window_a_restore_opened() {
        let h = handler(Duration::from_secs(30));
        let _ = h.restore(restore_request()).await;

        let apply = h.apply().await;
        assert!(!apply.success);
        assert!(apply.error.unwrap().contains("still restarting"));
    }

    #[tokio::test]
    async fn both_are_allowed_again_once_the_window_has_elapsed() {
        let h = handler(Duration::from_millis(50));
        let _ = h.apply().await;
        tokio::time::sleep(Duration::from_millis(80)).await;

        // Still fails for want of a board identity, but no longer for the
        // cooldown — which is what this asserts.
        let second = h.apply().await;
        let error = second.error.unwrap();
        assert!(!error.contains("still restarting"), "{error}");
        assert!(error.contains("identifying serial"), "{error}");
    }

    #[tokio::test]
    async fn the_first_operation_is_never_blocked() {
        let h = handler(Duration::from_secs(30));
        let first = h.apply().await;
        // Fails on identity, not on a cooldown that nothing has opened yet.
        assert!(!first.error.unwrap().contains("still restarting"));
    }

    #[test]
    fn the_cooldown_covers_flash_commit_plus_boot() {
        // PARAM_SAVE_SETTLE_DELAY (2s) + PX4's 3-5s bootloader dwell, with
        // margin. Shorter would reopen the window this exists to close.
        assert!(CYCLE_COOLDOWN >= Duration::from_secs(10));
    }
}

#[cfg(test)]
mod param_save_tests {
    use super::*;
    use crossbeam_channel::bounded;

    #[tokio::test]
    async fn a_permanently_full_queue_is_reported_rather_than_dropped() {
        // The condition that matters: nothing is draining, so the message can
        // never go out. Previously PARAM_SAVE was fire-and-forget here, and the
        // reboot went ahead regardless - rebooting a board whose flash state is
        // unknown, having silently discarded the parameters just pushed, since
        // PARAM_SET only writes RAM.
        let (mav_tx, mav_rx) = bounded::<MavMessage>(1);
        mav_tx.try_send(make_param_save()).expect("fills the queue");
        // Held, never read: the queue stays full for the whole attempt.
        let _held = mav_rx;

        let started = tokio::time::Instant::now();
        let result = send_with_backpressure(&mav_tx, make_param_save(), "PARAM_SAVE").await;

        assert!(result.is_err(), "a queue that never drains must be reported");
        assert!(result.unwrap_err().contains("not accepting writes"));
        // Bounded: it gives up rather than blocking provisioning forever.
        assert!(started.elapsed() < Duration::from_secs(10));
    }

    #[tokio::test]
    async fn a_queue_that_drains_lets_the_message_through() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(1);
        mav_tx.try_send(make_param_save()).expect("fills the queue");

        // A consumer appears shortly after, as the real writer does.
        tokio::task::spawn_blocking(move || {
            std::thread::sleep(std::time::Duration::from_millis(60));
            while mav_rx.recv().is_ok() {}
        });

        assert!(send_with_backpressure(&mav_tx, make_param_save(), "PARAM_SAVE")
            .await
            .is_ok());
    }

    #[tokio::test]
    async fn a_disconnected_writer_is_reported_as_a_lost_link_not_backpressure() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(1);
        drop(mav_rx);
        let err = send_with_backpressure(&mav_tx, make_param_save(), "SYS_HITL")
            .await
            .unwrap_err();
        assert!(err.contains("Lost the link"), "{err}");
    }

    #[test]
    fn the_save_settle_delay_covers_a_multi_parameter_commit() {
        // PX4 commits each dirty parameter as its own flash write, and
        // provisioning dirties ~21. Sized for that, not for a single write.
        assert!(PARAM_SAVE_SETTLE_DELAY >= Duration::from_millis(1500));
    }
}
