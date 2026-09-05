//! Timebase command-line surfaces: tick-rate validation, the browser
//! annunciator, and the `--selftest-timebase` fitness check.
//!
//! Kept out of `main.rs` so each piece is small enough to read and so the
//! validator has somewhere to be unit-tested.

use protocol::{SimulationStats, TimebaseHealth};
use simulation::timebase::GAP_THRESHOLD_US;
use simulation::{SharedOrigin, SimulationConfig, SimulationLoop};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, watch};
use tracing::{error, info, warn};
use websocket::VehicleMessage;

/// Nanoseconds in a second — the grid the loop's sleep period is built on
/// (`Duration::from_nanos(1_000_000_000 / tick_rate_hz)`).
const NANOS_PER_SEC: u64 = 1_000_000_000;

/// Microseconds in a second — the grid the simulation clock advances on
/// (`(dt * 1_000_000.0) as u64`).
const MICROS_PER_SEC: u64 = 1_000_000;

/// Highest `--tick-rate` the daemon accepts.
///
/// The rates this project documents are 400, 250 and 200 Hz, so 1000 Hz is
/// generous headroom. There has to be a ceiling because the catch-up budget is
/// DERIVED from the step (`GAP_THRESHOLD_US / step_us`), and the two compose
/// badly at rates that divide the grids but are absurd: `--tick-rate 100000`
/// was accepted, giving a 10 us step and a 2000-step allowance — one iteration
/// running 2000 RK4 sub-steps and 2000 state-write-lock acquisitions. Even a
/// plausible typo like `--tick-rate 2000` bought a 40-step allowance, ~3.8 ms
/// of physics against a 500 us period, which is a permanently gapping daemon
/// rather than a rejected argument. At 1000 Hz the allowance is 20 steps, and
/// [`SIM_THREAD_COMPUTATION`]'s reasoning — written around a worst case of 8 —
/// stays within a factor the doc comment can honestly describe.
const MAX_TICK_RATE_HZ: u32 = 1000;

/// MAVLink `MAV_SEVERITY_CRITICAL`.
const SEVERITY_CRITICAL: u8 = 2;
/// MAVLink `MAV_SEVERITY_WARNING`.
const SEVERITY_WARNING: u8 = 4;
/// MAVLink `MAV_SEVERITY_INFO`.
const SEVERITY_INFO: u8 = 6;

/// Prefix on every annunciator message.
///
/// The browser overlay renders PX4's STATUSTEXT stream through the same path,
/// so without this a daemon fault reads as something the flight controller
/// said.
const ANNUNCIATOR_PREFIX: &str = "DAEMON TIMEBASE";

/// `rt_outcome` for a run that could not observe one — the loop thread
/// panicked, or no health window closed.
///
/// Distinct from `"unsupported"` (this target has no mechanism) and from
/// `"unavailable:..."` (the host refused), both of which are things the run
/// actually established.
const RT_OUTCOME_UNKNOWN: &str = "unknown";

/// Worst-case work inside one loop iteration, declared to the host's scheduler.
///
/// The loop's own 5 s window measures `max_latency_us` at about 95 us, so this
/// is roughly 2.6x the observed worst case. Declaring a budget near the period
/// instead would describe a CPU hog on a 4-P-core part, and the kernel prices
/// the declaration accordingly.
///
/// It deliberately does NOT cover a full catch-up iteration. A recovering
/// iteration runs up to `GAP_THRESHOLD_US / step_us` physics sub-steps — 8 at
/// the 400 Hz default, and 20 at the [`MAX_TICK_RATE_HZ`] ceiling, which is
/// what that ceiling is for — so it can cost 8-20x this figure, and a run of
/// late wake-ups therefore produces genuine overruns of the declaration even
/// though the loop's own work is fine. Sizing the budget for the catch-up case
/// would declare a ~30% duty cycle for a thread that is idle 96% of the time,
/// which is a different and worse lie to tell the scheduler. The consequence is
/// that an overrun-driven revocation cannot be read as "the loop's work is
/// slow" — see the warning text in `SimulationLoop::check_realtime_contract`,
/// which names both causes.
///
/// Note also that macOS does not take this number as given: XNU raises the
/// computation field to a 500 us floor (measured on a Mac14,2), so the contract
/// actually in force already carries 2x what is declared here.
const SIM_THREAD_COMPUTATION: Duration = Duration::from_micros(250);

/// Health string for a run that produced no measurement at all.
///
/// Distinct from `"ok"` on purpose: the default `SimulationStats` reads `Ok`,
/// so a run whose loop never closed a health window — or whose thread died —
/// would otherwise print an all-zero report carrying the word "ok" and exit 0.
const HEALTH_UNKNOWN: &str = "unknown";

/// Minimum `--selftest-timebase` duration, in seconds.
///
/// One full 1 s health window plus margin. Below this the run can end before
/// any window closes, and a fitness check that grades an empty measurement is
/// worse than no fitness check.
const SELFTEST_MIN_SECS: u64 = 2;

/// Drift rate above which `--selftest-timebase` fails regardless of the
/// health verdict, in milliseconds of clock per second of wall.
///
/// 10 ms/s is 1% of real time, the same band as the 0.99 realised ratio the
/// health classifier calls Ok. The 2026-09-05 session measured 148.3 ms/s, and
/// a run whose own JSON says that must never exit 0.
///
/// NOW A BACKSTOP, NOT THE LIVE GATE — say so, because a green drift figure
/// must not read as evidence this check did any work. Wall-anchoring bounds
/// `wall − sim` at `GAP_THRESHOLD_US` (20 ms) by construction, so the largest
/// attainable rate is `20 / secs`: exactly 10.0 at the [`SELFTEST_MIN_SECS`]
/// floor, which the strict `>` does not trip, and unreachable for any run of
/// 3 s or longer. What grades a post-anchoring run is `timebase_worst_health`
/// and the `discarded_ms` that forces it to `Faulted`. This is kept because
/// its firing would mean the `Timebase` invariant itself had broken, which is
/// worth its own line rather than a silent pass.
const SELFTEST_MAX_DRIFT_MS_PER_S: f64 = 10.0;

/// Minimum interval between two annunciator messages that are NOT an
/// escalation.
///
/// The verdict is recomputed every second, so a host hovering on a threshold
/// flips health once per second forever. Unthrottled, that is a WARNING and an
/// INFO "recovered" banner every second at the person flying, and — because
/// the forwarder logs one line per message per connected client into the same
/// lossy 512-slot TUI channel the log path's rate limit protects — it defeats
/// that limit through a side channel.
const ANNUNCIATOR_MIN_INTERVAL: Duration = Duration::from_secs(5);

/// Consecutive windows at a lower severity required before announcing it.
///
/// A single Ok window after a fault is not evidence the host recovered, and
/// telling the pilot it is safe on that basis is the wrong-direction error.
/// Escalations bypass this entirely — they are announced on the first window.
const ANNUNCIATOR_DEESCALATE_WINDOWS: u32 = 3;

/// Escalation order for [`TimebaseHealth`]. Only an increase bypasses the
/// annunciator's rate limit.
fn severity_rank(health: TimebaseHealth) -> u8 {
    match health {
        TimebaseHealth::Ok => 0,
        TimebaseHealth::Degraded => 1,
        TimebaseHealth::Faulted => 2,
    }
}

/// When a health observation should reach the person flying.
///
/// Pure and separate from the task so the policy — which is what actually
/// governs how noisy the overlay is — is a unit test rather than a hope. This
/// mirrors `TimebaseReporter`'s shape in the simulation crate; the log path
/// had this throttling from the start and the browser path had none.
struct AnnunciatorPolicy {
    /// Health as of the last message actually sent.
    announced: TimebaseHealth,
    /// Health observed in the current run of windows.
    candidate: TimebaseHealth,
    /// How many consecutive windows have reported `candidate`.
    consecutive: u32,
    last_sent: Instant,
}

impl AnnunciatorPolicy {
    fn new(now: Instant) -> Self {
        Self {
            announced: TimebaseHealth::Ok,
            candidate: TimebaseHealth::Ok,
            consecutive: 0,
            // Backdated so a genuine de-escalation early in a session is not
            // held for five seconds by the process start alone. `checked_sub`
            // because `Instant` is monotonic since boot and subtracting past
            // its origin panics.
            last_sent: now.checked_sub(ANNUNCIATOR_MIN_INTERVAL).unwrap_or(now),
        }
    }

    /// Returns the health to announce, and the one it replaces, or `None`.
    fn observe(
        &mut self,
        observed: TimebaseHealth,
        now: Instant,
    ) -> Option<(TimebaseHealth, TimebaseHealth)> {
        if observed == self.candidate {
            self.consecutive = self.consecutive.saturating_add(1);
        } else {
            self.candidate = observed;
            self.consecutive = 1;
        }

        if observed == self.announced {
            return None;
        }

        let escalating = severity_rank(observed) > severity_rank(self.announced);
        let settled = self.consecutive >= ANNUNCIATOR_DEESCALATE_WINDOWS
            && now.duration_since(self.last_sent) >= ANNUNCIATOR_MIN_INTERVAL;

        if !escalating && !settled {
            return None;
        }

        let previous = self.announced;
        self.announced = observed;
        self.last_sent = now;
        Some((observed, previous))
    }
}

/// Accept only tick rates that divide BOTH the microsecond and the nanosecond
/// grid exactly.
///
/// The loop sleeps on a nanosecond period (`1_000_000_000 / hz`) and advances
/// its clock on a microsecond one (`(1.0 / hz) * 1_000_000`). A rate that is
/// not an exact divisor of both makes those two disagree, and the daemon then
/// runs a permanent deficit that the new timebase metric would faithfully
/// report — and blame on the host's scheduler, which had nothing to do with
/// it. 333 Hz is the worked example: 3_003_003 ns of wall period against
/// 3_003_000 ns of simulated advance.
///
/// `--tick-rate` had no validation at all, and `--tick-rate 0` reached a
/// divide by zero in the loop.
pub fn validate_tick_rate(hz: u32) -> Result<u32, String> {
    if hz == 0 {
        return Err("tick rate must be at least 1 Hz".to_string());
    }
    if hz > MAX_TICK_RATE_HZ {
        let step_us = MICROS_PER_SEC / u64::from(hz);
        return Err(format!(
            "{hz} Hz is above the {MAX_TICK_RATE_HZ} Hz ceiling. The catch-up budget is \
             derived as GAP_THRESHOLD_US / step, so a {step_us} us step would let ONE \
             iteration run {} physics sub-steps, each taking the state write lock — and \
             SIM_THREAD_COMPUTATION, the budget declared to the host scheduler, is sized \
             around a worst case of 8. Use 400, 250 or 200 Hz.",
            GAP_THRESHOLD_US / step_us.max(1)
        ));
    }

    let hz64 = u64::from(hz);
    let wall_period_ns = NANOS_PER_SEC / hz64;
    let sim_advance_ns = (MICROS_PER_SEC / hz64) * 1000;

    if NANOS_PER_SEC % hz64 != 0 || MICROS_PER_SEC % hz64 != 0 {
        return Err(format!(
            "{hz} Hz does not divide the microsecond and nanosecond grids exactly: \
             the loop would sleep {wall_period_ns} ns per tick while advancing the \
             simulation clock {sim_advance_ns} ns, a permanent deficit the timebase \
             metric would report and blame on the host. Use a rate that divides \
             1000000 exactly, e.g. 400, 250 or 200 Hz."
        ));
    }

    Ok(hz)
}

/// `--tick-rate` parser for clap.
pub fn parse_tick_rate(raw: &str) -> Result<u32, String> {
    let hz: u32 = raw
        .parse()
        .map_err(|_| format!("`{raw}` is not a whole number of hertz"))?;
    validate_tick_rate(hz)
}

/// Ask the host to schedule the CALLING thread as a periodic real-time task,
/// and say once what came back.
///
/// Must be called from INSIDE the simulation thread's closure: every mechanism
/// `rtsched` uses is self-directed and cannot be applied by the spawner.
///
/// EXPOSURE REDUCTION, NOT THE TIMEBASE FIX. The 2026-09-05 shortfall was about
/// 435 us per iteration against about 95 us of work — late wake-ups, not slow
/// compute — and a scheduling class fixed when the thread starts cannot produce
/// a step change at t=36 s that then never recovers. The leading suspect there
/// is Selective Forced Idle on a fanless Mac14,2. SFI is QoS-tiered, but what
/// this asks for on macOS is a mach time-constraint contract, which SUPERSEDES
/// the QoS tier rather than raising it (measured; see `rtsched::macos`), so the
/// mitigation on offer is `TH_MODE_REALTIME` and its relevance to SFI is not
/// established. `--selftest-timebase --no-realtime` is how to test whether it
/// helps on a given host; nothing here asserts that it does.
pub fn promote_sim_thread(tick_rate_hz: u32, realtime: bool) -> rtsched::Promotion {
    if !realtime {
        info!(
            "Simulation thread runs at normal priority (--no-realtime). Pacing is reported by \
             --selftest-timebase either way."
        );
        return rtsched::Promotion::NotRequested;
    }

    let period = Duration::from_micros(MICROS_PER_SEC / u64::from(tick_rate_hz.max(1)));
    let promotion = rtsched::promote_current_thread(period, SIM_THREAD_COMPUTATION);
    // Once, at info!, whichever way it went. A refusal is the ORDINARY path on
    // an unprivileged Linux host, so it must never be a warning; and a grant
    // must not be silent, because "did the promotion actually happen" is the
    // first question the A/B asks.
    info!(outcome = %promotion.token(), "{}", promotion.describe());
    promotion
}

/// How long `--sim-only` lets the battery sit depleted before recharging it.
///
/// Wall seconds, not iterations: the point is that a person watching a sim-only
/// run does not have to restart the daemon, and their patience is measured in
/// seconds regardless of the tick rate.
const SIM_ONLY_AUTO_RECHARGE_SECS: u64 = 3;

/// Cadence for the synthetic actuator generator that stands in for a flight
/// controller under `--sim-only`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SimOnlyCadence {
    /// Interval between synthetic actuator frames.
    pub step: Duration,
    /// Microseconds each frame stamps its timestamp forward.
    pub step_us: u64,
    /// Frames of continuous depletion before the battery is auto-recharged.
    pub auto_recharge_ticks: u64,
}

/// Derive the sim-only actuator cadence from the declared tick rate.
///
/// The generator had 2500 us hardcoded in four places and ignored
/// `--tick-rate` entirely, so `--sim-only --tick-rate 200` fed a 400 Hz stream
/// into a 200 Hz loop: the actuator channel took twice the input it could
/// consume, and the frame timestamps advanced at twice the simulation clock.
/// Deriving both from the rate removes the second timebase nobody declared.
///
/// BEHAVIOUR CHANGE for anyone already running a non-400 Hz rate — called out
/// separately in the CHANGELOG for that reason.
pub fn sim_only_cadence(tick_rate_hz: u32) -> SimOnlyCadence {
    // `.max(1)` mirrors the loop's own guard at the point it computes
    // `step_us`. `validate_tick_rate` already rejects 0 at the CLI, so this is
    // a second line of defence against a future caller, not input handling.
    let hz = u64::from(tick_rate_hz.max(1));
    let step_us = MICROS_PER_SEC / hz;
    SimOnlyCadence {
        step: Duration::from_micros(step_us),
        step_us,
        auto_recharge_ticks: SIM_ONLY_AUTO_RECHARGE_SECS * hz,
    }
}

/// Severity and text for one timebase health transition.
///
/// Pure, so the wording — which is what a person flying actually reads — is
/// covered by a test rather than by hoping.
fn annunciation(stats: &SimulationStats, previous: TimebaseHealth) -> (u8, String) {
    let drift_ms = stats.timebase_drift_us / 1000;
    match stats.timebase_health {
        TimebaseHealth::Faulted => (
            SEVERITY_CRITICAL,
            format!(
                "{ANNUNCIATOR_PREFIX}: FAULTED — loop at {:.0} Hz, sim clock {drift_ms} ms \
                 behind wall. Do not fly: the flight controller is being fed more motion \
                 than was simulated.",
                stats.tick_rate_hz
            ),
        ),
        TimebaseHealth::Degraded => (
            SEVERITY_WARNING,
            format!(
                "{ANNUNCIATOR_PREFIX}: degraded — loop at {:.0} Hz, sim clock {drift_ms} ms \
                 behind wall. Close other applications before flying.",
                stats.tick_rate_hz
            ),
        ),
        // A recovery line is graded against the SESSION latches, not against
        // the window that produced it. `timebase_arming_gate` refuses on
        // `session_invalid` / `timebase_discarded_ms` / `timebase_worst_health`,
        // all of which are session-scoped and none of which a good window
        // clears — so a bare "recovered" here told the person flying the fault
        // had cleared, and the very next arming was still refused with "this
        // does not clear until the daemon is restarted". Told it cleared, then
        // told it did not.
        TimebaseHealth::Ok if session_is_compromised(stats) => (
            SEVERITY_WARNING,
            format!(
                "{ANNUNCIATOR_PREFIX}: cadence recovered from {previous:?} — loop at {:.0} Hz — \
                 but this session lost {} ms of wall time and will NOT arm again until the \
                 daemon is restarted.",
                stats.tick_rate_hz, stats.timebase_discarded_ms
            ),
        ),
        TimebaseHealth::Ok => (
            SEVERITY_INFO,
            format!(
                "{ANNUNCIATOR_PREFIX}: recovered from {previous:?} — loop at {:.0} Hz.",
                stats.tick_rate_hz
            ),
        ),
    }
}

/// Whether this SESSION is one the arming gate will refuse, whatever the last
/// window says.
///
/// Deliberately the same three inputs as `websocket::preflight::timebase_arming_gate`,
/// because the overlay must not tell the pilot something the gate is about to
/// contradict. If that predicate ever gains a fourth input, this is the other
/// place it has to be added.
fn session_is_compromised(stats: &SimulationStats) -> bool {
    stats.session_invalid
        || stats.timebase_discarded_ms > 0
        || stats.timebase_worst_health == TimebaseHealth::Faulted
}

/// Publish a `VehicleMessage` to the browser on every timebase health
/// transition.
///
/// This reaches the person actually flying through the channel the overlay
/// already renders (`MSG_TYPE_VEHICLE_MESSAGE`), so it costs no wire-format
/// change and no work in `th3seus-web`.
pub fn spawn_timebase_annunciator(
    mut stats_rx: watch::Receiver<SimulationStats>,
    vehicle_msg_tx: broadcast::Sender<VehicleMessage>,
    daemon_start: std::time::Instant,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        let mut policy = AnnunciatorPolicy::new(Instant::now());

        while stats_rx.changed().await.is_ok() {
            let stats = stats_rx.borrow_and_update().clone();

            let Some((_health, previous)) = policy.observe(stats.timebase_health, Instant::now())
            else {
                continue;
            };

            let (severity, text) = annunciation(&stats, previous);

            // A send with no receivers means nobody has the simulator page
            // open. That is not an error and must not be logged as one — the
            // daemon log already carries the same transition.
            let _ = vehicle_msg_tx.send(VehicleMessage {
                severity,
                timestamp_ms: daemon_start.elapsed().as_millis() as u32,
                text,
            });
        }

        info!("Timebase annunciator exiting");
    })
}

/// Run the loop headless for `secs` seconds and print one line of JSON.
///
/// This is the pre-flight fitness check, the A/B harness for any scheduling
/// work, and the single command to hand a user reporting a problem. Exits
/// non-zero when the timebase is anything but healthy and when the run
/// produced no measurement at all.
///
/// Since wall-anchoring, the conditions that actually fail a run are
/// `timebase_worst_health` and the `discarded_ms` that drives it to `Faulted`.
/// The [`SELFTEST_MAX_DRIFT_MS_PER_S`] check below is a backstop on the
/// `Timebase` invariant, not a live threshold — see its doc comment.
///
/// The JSON goes to stdout and nothing else does; diagnostics are routed to
/// stderr for the duration, so `--selftest-timebase 5 | jq` works.
pub async fn run_selftest(config: SimulationConfig, secs: u64, realtime: bool) -> i32 {
    let secs = secs.max(SELFTEST_MIN_SECS);
    let target_hz = config.tick_rate_hz;
    info!(
        secs,
        target_hz, "Timebase selftest starting (no flight controller, no browser)"
    );

    let (_actuator_tx, actuator_rx) = crossbeam_channel::bounded(16);
    let (_config_tx, config_rx) = crossbeam_channel::bounded(1);
    let (mav_tx, mav_rx) = crossbeam_channel::bounded(512);
    let (stats_tx, stats_rx) = watch::channel(SimulationStats::default());

    let origin = Arc::new(SharedOrigin::new(
        config.reference_lat,
        config.reference_lon,
        config.reference_alt,
    ));
    let mut sim = SimulationLoop::new(config, actuator_rx, config_rx, mav_tx, origin)
        .with_stats_publisher(stats_tx);
    let state = sim.state_handle();

    // Drain the outbound MAVLink channel. With no consumer the 512-slot
    // channel fills in about 1.3 s and the run then measures backpressure
    // instead of pacing — the opposite of what this check is for.
    let drain = std::thread::Builder::new()
        .name("selftest-drain".to_string())
        .spawn(move || while mav_rx.recv().is_ok() {})
        .expect("failed to spawn selftest drain thread");

    // The selftest promotes its thread exactly as the daemon does, because it
    // is the A/B harness for that promotion: a check that measured an
    // unpromoted thread would answer a different question from the one the user
    // is asking.
    let sim_thread = std::thread::Builder::new()
        .name("selftest-sim".to_string())
        .spawn(move || {
            let promotion = promote_sim_thread(target_hz, realtime);
            sim.set_realtime(&promotion);
            sim.run();
            rtsched::revert_current_thread();
            promotion.token()
        })
        .expect("failed to spawn selftest simulation thread");

    tokio::time::sleep(Duration::from_secs(secs)).await;
    state.stop();

    // Joining a `std::thread` from an async fn parks a tokio worker. Nothing
    // else is scheduled during a selftest, but `spawn_blocking` is where the
    // panic below has to be caught anyway.
    let sim_join = tokio::task::spawn_blocking(move || sim_thread.join()).await;
    let rt_outcome = match sim_join {
        Ok(Ok(token)) => token,
        Ok(Err(panic)) => {
            // A crash in the thing being tested is not a pass. The watch
            // channel still holds `SimulationStats::default()`, whose health
            // reads Ok, so falling through here would print an all-zero
            // report and exit 0.
            error!(?panic, "Timebase selftest: the simulation thread panicked");
            println!("{}", inconclusive_report(target_hz));
            return 2;
        }
        Err(join_err) => {
            error!(%join_err, "Timebase selftest: could not join the simulation thread");
            println!("{}", inconclusive_report(target_hz));
            return 2;
        }
    };
    if let Err(panic) = drain.join() {
        // Less consequential — the drain only keeps the outbound channel from
        // filling — but it must not be swallowed either.
        warn!(?panic, "Timebase selftest: the drain thread panicked");
    }

    let stats = stats_rx.borrow().clone();

    // No health window ever closed, so there is nothing to grade. Reported as
    // its own outcome rather than allowed to inherit the `Ok` default.
    if stats.tick_rate_hz == 0.0 {
        error!(
            secs,
            "Timebase selftest: no health window closed — the run measured nothing"
        );
        println!("{}", inconclusive_report(target_hz));
        return 2;
    }

    // NOT published as a rate. `timebase_drift_us` is `wall − sim` at the LAST
    // window close, an instantaneous offset; dividing it by the run length
    // produced a key called `drift_ms_per_s` that measures nothing per second
    // and that two identical hosts report differently purely as a function of
    // the N passed on the command line. It is kept as a local because the
    // backstop below is expressed in those units.
    let drift_ms_per_s = f64::from(stats.timebase_drift_us) / 1000.0 / secs as f64;
    let report = serde_json::json!({
        "achieved_rate_hz": stats.tick_rate_hz,
        "target_rate_hz": target_hz,
        "steps_per_sec": stats.steps_per_sec,
        // The raw figure, named for what it is: the clock offset at the end of
        // the run. A correctly paced loop rests slightly NEGATIVE — up to one
        // step ahead of the wall value it was last given.
        "drift_us_at_end": stats.timebase_drift_us,
        "min_realised": stats.timebase_min_realised,
        "wake_late_p99_us": stats.wake_late_p99_us,
        "wake_late_max_us": stats.wake_late_max_us,
        "wake_late_avg_us": stats.wake_late_avg_us,
        // The other half of the wake-lateness diagnosis: lateness with no
        // overrun is a host that will not schedule the thread, overrun with no
        // lateness is work that does not fit in the period.
        "deadline_overrun_us": stats.deadline_overrun_us,
        "discarded_ms": stats.timebase_discarded_ms,
        "gap_count": stats.timebase_gap_count,
        // The SESSION's worst verdict, not the last second's. A host that
        // stalled and recovered before the run ended reported "ok" here while
        // its own drift figure sat beside it at 250 ms/s.
        "health": format!("{:?}", stats.timebase_worst_health).to_lowercase(),
        "last_window_health": format!("{:?}", stats.timebase_health).to_lowercase(),
        // Prefer what the LOOP published: the thread's own token says what was
        // granted at the start, while the loop's copy also carries a
        // revocation the kernel issued during the run. `is_empty` covers a run
        // too short for a stats publish.
        "rt_outcome": if stats.realtime_policy.is_empty() {
            rt_outcome.clone()
        } else {
            stats.realtime_policy.clone()
        },
    });
    println!("{report}");

    if drift_ms_per_s > SELFTEST_MAX_DRIFT_MS_PER_S {
        error!(
            drift_ms_per_s,
            threshold = SELFTEST_MAX_DRIFT_MS_PER_S,
            "Timebase selftest: FAILED — the simulation clock fell behind wall time faster \
             than the threshold. Wall-anchoring bounds this at GAP_THRESHOLD_US / secs, so \
             reaching it means the Timebase invariant itself is broken, not merely that the \
             host is slow"
        );
        return 1;
    }

    match stats.timebase_worst_health {
        TimebaseHealth::Ok => 0,
        TimebaseHealth::Degraded => {
            warn!(
                "Timebase selftest: degraded — this host will not hold {target_hz} Hz under load"
            );
            1
        }
        TimebaseHealth::Faulted => {
            error!("Timebase selftest: FAULTED — this host is not fit to fly HITL as configured");
            1
        }
    }
}

/// The report shape for a run that measured nothing.
///
/// Same keys as a real report so anything parsing the output does not have to
/// branch, with `health` set to [`HEALTH_UNKNOWN`] rather than the `Ok` that a
/// default-constructed `SimulationStats` would otherwise supply.
fn inconclusive_report(target_hz: u32) -> String {
    serde_json::json!({
        "achieved_rate_hz": 0.0,
        "target_rate_hz": target_hz,
        "steps_per_sec": 0.0,
        "drift_us_at_end": 0,
        "min_realised": 0.0,
        "wake_late_p99_us": 0,
        "wake_late_max_us": 0,
        "wake_late_avg_us": 0,
        "deadline_overrun_us": 0,
        "discarded_ms": 0,
        "gap_count": 0,
        "health": HEALTH_UNKNOWN,
        "last_window_health": HEALTH_UNKNOWN,
        "rt_outcome": RT_OUTCOME_UNKNOWN,
    })
    .to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tick_rate_must_divide_both_the_microsecond_and_nanosecond_grids() {
        for hz in [400, 250, 200] {
            assert_eq!(
                validate_tick_rate(hz),
                Ok(hz),
                "{hz} Hz divides both grids exactly and must be accepted"
            );
        }

        assert!(
            validate_tick_rate(0).is_err(),
            "`--tick-rate 0` reached a divide by zero in the sim loop with no \
             validation between the CLI and the divide"
        );

        let too_fast = validate_tick_rate(100_000)
            .expect_err("100 kHz divides both grids exactly and must still be rejected");
        assert!(
            too_fast.contains("2000") && too_fast.contains("sub-steps"),
            "the ceiling exists because the catch-up budget is derived from the \
             step, so the message has to name the sub-step count the rate would \
             buy — 20 000 / 10 = 2000 here. Got: {too_fast}"
        );
        assert!(
            validate_tick_rate(2000).is_err(),
            "`--tick-rate 2000` is a plausible typo that divides both grids and \
             buys a 40-step allowance — 3.8 ms of physics against a 500 us period"
        );
        assert_eq!(
            validate_tick_rate(MAX_TICK_RATE_HZ),
            Ok(MAX_TICK_RATE_HZ),
            "the ceiling itself is accepted; the bound is on what is ABOVE it"
        );

        let rejected = validate_tick_rate(333).expect_err("333 Hz must be rejected");
        assert!(
            rejected.contains("3003003") || rejected.contains("3_003_003"),
            "the message must name the actual mismatch — 3_003_003 ns of wall \
             period against 3_003_000 ns of simulated advance — or the operator \
             cannot tell a rejected rate from a typo. Got: {rejected}"
        );
        assert!(
            rejected.contains("3003000") || rejected.contains("3_003_000"),
            "and the other half of it. Got: {rejected}"
        );
    }

    #[test]
    fn the_sim_only_cadence_at_400_hz_is_what_was_hardcoded() {
        let c = sim_only_cadence(400);
        assert_eq!(c.step_us, 2500, "the 400 Hz default must not move");
        assert_eq!(c.step, Duration::from_micros(2500));
        assert_eq!(
            c.auto_recharge_ticks, 1200,
            "3 s at 400 Hz, the literal the generator carried"
        );
    }

    #[test]
    fn the_sim_only_cadence_follows_the_declared_tick_rate() {
        let c = sim_only_cadence(200);
        assert_eq!(
            c.step_us, 5000,
            "the generator ignored --tick-rate and fed a 400 Hz stream into a 200 Hz loop"
        );
        assert_eq!(
            c.auto_recharge_ticks, 600,
            "auto-recharge is 3 s of WALL time, so halving the rate halves the tick count \
             rather than doubling the wait"
        );

        let c = sim_only_cadence(250);
        assert_eq!(c.step_us, 4000);
        assert_eq!(c.auto_recharge_ticks, 750);
    }

    #[test]
    fn the_sim_only_cadence_never_divides_by_zero() {
        // Unreachable from the CLI (`validate_tick_rate` rejects 0), and
        // therefore exactly the kind of input a future caller supplies.
        let c = sim_only_cadence(0);
        assert_eq!(c.step_us, MICROS_PER_SEC);
        assert_eq!(c.auto_recharge_ticks, SIM_ONLY_AUTO_RECHARGE_SECS);
    }

    #[test]
    fn the_annunciator_names_itself_so_the_overlay_does_not_blame_px4() {
        let faulted = SimulationStats {
            tick_rate_hz: 341.0,
            timebase_drift_us: 148_300,
            timebase_health: TimebaseHealth::Faulted,
            ..Default::default()
        };

        let (severity, text) = annunciation(&faulted, TimebaseHealth::Ok);
        assert_eq!(severity, SEVERITY_CRITICAL);
        assert!(
            text.starts_with(ANNUNCIATOR_PREFIX),
            "the browser renders PX4's STATUSTEXT stream through this same \
             channel, so an unprefixed line reads as something the flight \
             controller said. Got: {text}"
        );
        assert!(
            text.contains("341"),
            "the achieved rate belongs in the banner: {text}"
        );
        assert!(text.contains("148"), "so does the drift: {text}");

        let degraded = SimulationStats {
            timebase_health: TimebaseHealth::Degraded,
            ..faulted.clone()
        };
        assert_eq!(
            annunciation(&degraded, TimebaseHealth::Ok).0,
            SEVERITY_WARNING
        );

        let recovered = SimulationStats {
            timebase_health: TimebaseHealth::Ok,
            timebase_worst_health: TimebaseHealth::Ok,
            ..faulted.clone()
        };
        let (severity, text) = annunciation(&recovered, TimebaseHealth::Faulted);
        assert_eq!(severity, SEVERITY_INFO);
        assert!(
            text.contains("Faulted"),
            "a recovery line has to say what it recovered FROM, or the operator \
             cannot tell it apart from a routine notice: {text}"
        );
    }

    /// The overlay must not retract a fault the arming gate still enforces.
    ///
    /// `annunciation` grades the LAST window; `timebase_arming_gate` refuses on
    /// three SESSION latches. A bare "recovered" told the person flying the
    /// fault had cleared, and their next arming was refused with "this does not
    /// clear until the daemon is restarted".
    #[test]
    fn a_recovery_cannot_contradict_the_arming_gate() {
        let latched = |stats: SimulationStats| annunciation(&stats, TimebaseHealth::Faulted);

        for (label, stats) in [
            (
                "session_invalid",
                SimulationStats {
                    tick_rate_hz: 400.0,
                    timebase_health: TimebaseHealth::Ok,
                    session_invalid: true,
                    ..Default::default()
                },
            ),
            (
                "discarded_ms",
                SimulationStats {
                    tick_rate_hz: 400.0,
                    timebase_health: TimebaseHealth::Ok,
                    timebase_discarded_ms: 17,
                    ..Default::default()
                },
            ),
            (
                "worst_health",
                SimulationStats {
                    tick_rate_hz: 400.0,
                    timebase_health: TimebaseHealth::Ok,
                    timebase_worst_health: TimebaseHealth::Faulted,
                    ..Default::default()
                },
            ),
        ] {
            let (severity, text) = latched(stats);
            assert_ne!(
                severity, SEVERITY_INFO,
                "{label} still refuses the next arming, so this must not be an \
                 all-clear: {text}"
            );
            assert!(
                text.contains("NOT arm"),
                "{label}: the line has to say the gate is still shut, or the \
                 pilot is told it cleared and then told it did not: {text}"
            );
        }

        // And a genuinely clean session still gets the plain recovery.
        let clean = SimulationStats {
            tick_rate_hz: 400.0,
            timebase_health: TimebaseHealth::Ok,
            ..Default::default()
        };
        let (severity, text) = annunciation(&clean, TimebaseHealth::Degraded);
        assert_eq!(
            severity, SEVERITY_INFO,
            "a session that never latched anything must still be able to \
             announce a recovery, or the message means nothing: {text}"
        );
    }

    #[test]
    fn a_flapping_verdict_does_not_flood_the_person_flying() {
        let t0 = Instant::now();
        let mut policy = AnnunciatorPolicy::new(t0);

        // A host hovering on the 0.99 threshold: Degraded, Ok, Degraded, Ok…
        // once per second. Unthrottled that is a banner every second, forever,
        // through a 64-slot broadcast shared with PX4's STATUSTEXT relay.
        let mut sent = 0;
        for second in 0..60 {
            let now = t0 + Duration::from_secs(second);
            let observed = if second % 2 == 0 {
                TimebaseHealth::Degraded
            } else {
                TimebaseHealth::Ok
            };
            if policy.observe(observed, now).is_some() {
                sent += 1;
            }
        }

        assert_eq!(
            sent, 1,
            "only the first escalation is announced: a verdict alternating \
             every second never holds a lower severity for {ANNUNCIATOR_DEESCALATE_WINDOWS} \
             consecutive windows, so it can never announce a recovery"
        );
    }

    #[test]
    fn an_escalation_is_immediate_and_a_recovery_has_to_settle() {
        let t0 = Instant::now();
        let mut policy = AnnunciatorPolicy::new(t0);

        assert_eq!(
            policy.observe(TimebaseHealth::Faulted, t0),
            Some((TimebaseHealth::Faulted, TimebaseHealth::Ok)),
            "a fault reaches the pilot on the first window that sees it — the \
             rate limit must never delay bad news"
        );

        // One good window is not a recovery.
        let t1 = t0 + Duration::from_secs(1);
        assert_eq!(policy.observe(TimebaseHealth::Ok, t1), None);
        let t2 = t0 + Duration::from_secs(2);
        assert_eq!(policy.observe(TimebaseHealth::Ok, t2), None);

        // Three consecutive good windows, but only 3 s since the fault line.
        let t3 = t0 + Duration::from_secs(3);
        assert_eq!(
            policy.observe(TimebaseHealth::Ok, t3),
            None,
            "settled, but inside the {ANNUNCIATOR_MIN_INTERVAL:?} floor"
        );

        let t6 = t0 + Duration::from_secs(6);
        assert_eq!(
            policy.observe(TimebaseHealth::Ok, t6),
            Some((TimebaseHealth::Ok, TimebaseHealth::Faulted)),
            "a recovery announced once the host has actually held it"
        );

        // And a steady state says nothing at all.
        let t7 = t0 + Duration::from_secs(7);
        assert_eq!(policy.observe(TimebaseHealth::Ok, t7), None);
    }

    #[test]
    fn a_reconfigure_cannot_announce_a_recovery_that_did_not_happen() {
        // `SimulationLoop` used to reset `self.stats` wholesale on
        // ConfigureBuild, defaulting `timebase_health` back to Ok. The loop
        // now preserves the timebase group; this pins the annunciator's half
        // of the same guarantee — a single Ok observation cannot retract a
        // CRITICAL banner.
        let t0 = Instant::now();
        let mut policy = AnnunciatorPolicy::new(t0);
        assert!(policy.observe(TimebaseHealth::Faulted, t0).is_some());

        let t10 = t0 + Duration::from_secs(10);
        assert_eq!(
            policy.observe(TimebaseHealth::Ok, t10),
            None,
            "one Ok window, even long after the fault, is not evidence the \
             host recovered — and telling the pilot it is safe is the \
             wrong-direction error"
        );
    }
}
