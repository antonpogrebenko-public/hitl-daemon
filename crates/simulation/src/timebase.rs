//! Wall-anchored simulation timebase — the pure pacing planner.
//!
//! This module owns the two monotone counters the simulation loop paces
//! against. It has NO clock and NO I/O: wall time arrives as a parameter, so
//! every pacing decision the loop makes is reproducible in a unit test at
//! whatever cadence the test wants, including cadences no real host produces.
//!
//! # Why it exists
//!
//! PX4 on real hardware ignores `HIL_SENSOR.time_usec` —
//! `MavlinkReceiver::handle_message_hil_sensor` stamps every sample with its
//! own `hrt_absolute_time()` — so it integrates each emitted rate over the
//! WALL interval between arrivals. A loop that advances physics by a fixed
//! 2.5 ms per emission while wall time advances 2.93 ms therefore hands the
//! EKF ~17% more motion than the simulation actually performed, continuously,
//! with zero dropped messages and nothing in a gap-based guard to see. That is
//! the 2026-09-05 injury: the EKF diverged to 42 m / 21.5 m/s while still
//! reporting position valid with eph 1.86 m.
//!
//! The repair is arithmetic, not heuristics. `sim_us` counts microseconds of
//! WALL time since the loop epoch, and the loop's sleep deadline is literally
//! `epoch + sim_us`, so lateness is retained as owed simulation instead of
//! being deleted. Physics is advanced in whole fixed-`step_us` sub-steps to
//! reach it — fixed dt because the motor mode has `tau = 0.025 s`, giving
//! `|lambda·h| = 11.9` and an RK4 amplification around 610 at the 296.7 ms
//! hole this code has to survive.
//!
//! # The two counters
//!
//! - **`sim_us`** — microseconds of wall time since the loop epoch. Drives the
//!   sleep deadline, `HIL_SENSOR.time_usec`, the GPS rate limiter and the TUI
//!   stopwatch.
//! - **`integrated_us = sim_us − discarded_us`** — microseconds of physics
//!   actually simulated. Equal to `sim_us` on a healthy run; used only for
//!   `HIL_GPS.time_usec`, so the ULog cross-check
//!   `(hrt elapsed) − (time_utc_usec elapsed)` reads out exactly the time this
//!   module admits it lost.
//!
//! Both are monotone non-decreasing for the life of the loop. Monotonicity is
//! load-bearing, not tidiness: `hitl-sensors/src/gps.rs` holds `last_update_time`
//! and `last_output_time` in the same timebase, and a backward jump would
//! silence GPS for the rest of the session ("ekf2 missing data" on the FC).
//!
//! # Invariants, enforced on every call
//!
//! 1. `sim_us == steps_done × step_us + discarded_us`, exact by construction.
//! 2. `wall − sim_us <= gap_threshold_us` after `plan()` — the clock never
//!    lags the wall by more than the PX4 IMU clamp window; any excess is
//!    folded into `discarded_us` and reported.
//! 3. `sim_us` and `integrated_us` never decrease.
//!
//! Note the exact shape of invariant 2. `plan()` always returns at least one
//! step, so immediately after a call the clock may be up to one `step_us`
//! AHEAD of the wall value it was given — that is the quantum it cannot
//! subdivide. The lower bound `0 <= wall − sim_us` is a property of the
//! caller, not of this module: a loop that sleeps to `deadline_us()` before
//! each call re-establishes it every iteration. A caller that ignores the
//! deadline gets sim time running ahead of wall time, which is the correct
//! answer to being asked to pace against a wall clock that is not advancing.

/// Microseconds of wall time a single iteration may absorb by sub-stepping
/// before the excess is declared lost instead of simulated.
///
/// Set to PX4's own `imu_max_dt`, which is `2 × EKF2_PREDICT_US`
/// (`src/modules/ekf2/EKF/estimator_interface.cpp`): the EKF clamps any IMU
/// integration interval longer than this. Keeping the catch-up budget at the
/// same value means an absorbed catch-up never crosses the clamp — which is
/// the mechanism that produced "position valid, eph 1.86 m" against a true
/// 42 m error on 2026-09-05. Tightening this constant past PX4's own clamp
/// reintroduces that overconfidence; `a_twenty_millisecond_stall_is_absorbed_and_is_not_a_gap`
/// exists to stop that happening by accident.
pub const GAP_THRESHOLD_US: u64 = 20_000;

/// Maximum physics sub-steps a single iteration may take.
///
/// `8 × 2500 µs` is exactly [`GAP_THRESHOLD_US`] at the 400 Hz default, so the
/// catch-up budget and the PX4 clamp are the same window by construction.
///
/// The floor on the other side is PX4's sensor voter:
/// `voted_sensors_update.cpp:61-62` declares accel/gyro dead after 500 ms
/// without a sample (mag/baro after 300 ms). Catch-up never skips an emission
/// — one HIL_SENSOR still leaves per iteration, carrying the mean over the
/// sub-steps — so emission spacing stays the wall iteration period, two orders
/// of magnitude inside that budget.
pub const MAX_CATCHUP_STEPS: u32 = 8;

/// At or above this fraction of the target iteration rate the loop is healthy.
///
/// 1% of the 2500 µs budget is 25 µs per iteration, an order of magnitude
/// above the ~2 ns quantisation of the counters and well below the ~95 µs of
/// work a tick is documented to cost, so the band is measurable without being
/// jittery.
pub const HEALTH_OK_RATIO: f64 = 0.99;

/// Below this fraction of the target iteration rate the loop is faulted.
///
/// The 2026-09-05 session held 341/400 = 0.8525 for the 27 s that ended in the
/// crash, and `src/tui.rs` rendered it as an acceptable yellow because its
/// caution floor was 320 Hz. This threshold is what makes that session an
/// error rather than a colour.
pub const HEALTH_FAULTED_RATIO: f64 = 0.90;

/// What a single iteration should do, decided from wall time alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TickPlan {
    /// Fixed-`step_us` physics sub-steps to run. Always at least 1: a
    /// zero-step plan would stall the loop and starve the FC of samples,
    /// which is the one outcome no pacing fault is allowed to cause.
    pub steps: u32,
    /// Microseconds of wall time this iteration gives up as unsimulated.
    /// Non-zero only on a gap.
    pub discarded_us: u64,
    /// True when the deficit exceeded [`Timebase::gap_threshold_us`]. Rare by
    /// construction, so the caller logs every occurrence.
    pub gap: bool,
    /// The deficit the decision was made from, carried so a log line can
    /// describe itself without the caller recomputing it.
    pub deficit_us: u64,
}

/// Health of the timebase over one observation window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Health {
    /// Keeping cadence, nothing lost.
    #[default]
    Ok,
    /// Behind cadence but still simulating every microsecond of wall time.
    Degraded,
    /// Either far behind cadence, or wall time has been lost outright.
    Faulted,
}

impl Health {
    /// Short lowercase name for log lines.
    pub fn as_str(self) -> &'static str {
        match self {
            Health::Ok => "ok",
            Health::Degraded => "degraded",
            Health::Faulted => "faulted",
        }
    }
}

impl From<Health> for protocol::TimebaseHealth {
    /// The wire enum lives in `protocol` because `SimulationStats` travels to
    /// the TUI and the status surfaces; this one stays here so the pure
    /// planner does not have to be versioned alongside a wire format.
    fn from(health: Health) -> Self {
        match health {
            Health::Ok => protocol::TimebaseHealth::Ok,
            Health::Degraded => protocol::TimebaseHealth::Degraded,
            Health::Faulted => protocol::TimebaseHealth::Faulted,
        }
    }
}

/// Classify a window's timebase health.
///
/// `realised_ratio` is the achieved ITERATION rate over the target rate — not
/// the step rate. The distinction is the whole diagnostic: after this fix a
/// struggling host reports ~400 steps/s against ~341 iterations/s, meaning the
/// clock is correct and the host is slow, which is a complete diagnosis in one
/// line. Rating it on steps would call that healthy and say nothing.
///
/// Any discarded time is Faulted regardless of rate: time the simulation could
/// not account for is not a matter of degree. There is deliberately NO
/// cumulative-skew budget — a fault budget without decay faults any host
/// holding a steady sub-1% shortfall after a couple of minutes, which is
/// denial of service on the shipped fanless target.
pub fn timebase_health(realised_ratio: f64, discarded_us: u64) -> Health {
    if discarded_us > 0 {
        return Health::Faulted;
    }
    // A window with no measurable duration yields a non-finite ratio. That is
    // an absence of evidence, not evidence of a fault: reporting it as Faulted
    // would make the very first window of every session an alarm.
    if !realised_ratio.is_finite() {
        return Health::Ok;
    }
    if realised_ratio >= HEALTH_OK_RATIO {
        Health::Ok
    } else if realised_ratio >= HEALTH_FAULTED_RATIO {
        Health::Degraded
    } else {
        Health::Faulted
    }
}

/// Raw counters accumulated over one observation window.
#[derive(Debug, Clone, Copy, Default)]
pub struct WindowSample {
    /// Wall duration of the window.
    pub wall_us: u64,
    /// Loop iterations executed in the window.
    pub iterations: u64,
    /// Physics sub-steps executed in the window.
    pub steps: u64,
    /// Target iteration rate the loop was configured for.
    pub target_hz: u32,
    /// Sum of `wake − deadline` over the iterations that actually slept.
    pub wake_late_total_us: u64,
    /// Iterations that actually slept, i.e. the sample count behind
    /// `wake_late_total_us`.
    ///
    /// An iteration whose own work overran the deadline never slept: the
    /// interval between its scheduled deadline and the next clock read is work
    /// time, not wake latency. Averaging the two populations together makes a
    /// compute-bound loop and a descheduled one produce the same number, which
    /// is precisely the misdirection the `latency_us` trace used to supply.
    /// The overrun population is reported separately, as [`WindowSample::overrun_us`].
    pub wake_late_samples: u64,
    /// Largest single `wake − deadline` among those iterations.
    pub wake_late_max_us: u64,
    /// Sum of `deadline − now` over the iterations that were ALREADY past the
    /// deadline when they reached the sleep, i.e. the other population.
    ///
    /// This is the discriminator, and reading `steps_per_sec` against
    /// `iterations_per_sec` is not. Wall-anchoring pins `sim_us` to the wall,
    /// so `steps_per_sec` sits at the target on any host that is not
    /// discarding time — compute-bound or not — and time that IS discarded
    /// forces `Faulted` on its own. Zero wake lateness with a non-zero overrun
    /// is compute-bound; the reverse is descheduled.
    pub overrun_us: u64,
    /// `wall − sim_us` at the end of the window.
    pub drift_us: i64,
    /// Wall time discarded within the window.
    pub discarded_us: u64,
    /// Gaps recorded within the window.
    pub gap_count: u32,
}

/// A window's counters reduced to the numbers a human reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WindowVerdict {
    pub health: Health,
    pub drift_us: i64,
    pub iterations_per_sec: f64,
    pub steps_per_sec: f64,
    pub realised: f64,
    pub wake_late_avg_us: u32,
    pub wake_late_max_us: u32,
    /// Window total of [`WindowSample::overrun_us`].
    pub deadline_overrun_us: u32,
    pub discarded_ms: u32,
    pub gap_count: u32,
}

/// Clamp a `u64` into a `u32` field without wrapping.
fn saturate_u32(value: u64) -> u32 {
    value.min(u64::from(u32::MAX)) as u32
}

impl WindowSample {
    /// Reduce the window to a verdict.
    ///
    /// `steps_per_sec` and `iterations_per_sec` are reported side by side
    /// because the pair says whether the CLOCK is still correct while the loop
    /// is behind: during the 2026-09-05 incident that reads 400 against 341,
    /// which excludes the physics and terrain code from the first log line
    /// anyone looks at.
    ///
    /// It does NOT separate compute-bound from descheduled, and it must not be
    /// read as if it did. Wall-anchoring holds `steps_per_sec` at the target on
    /// any host that is not discarding time, and discarded time forces
    /// `Faulted` by itself — so "both low" collapses to "there were gaps".
    /// `wake_late_avg_us` against `deadline_overrun_us` is the pair that
    /// separates the two populations.
    pub fn verdict(&self) -> WindowVerdict {
        let secs = self.wall_us as f64 / 1_000_000.0;
        let (iterations_per_sec, steps_per_sec) = if secs > 0.0 {
            (self.iterations as f64 / secs, self.steps as f64 / secs)
        } else {
            (f64::NAN, f64::NAN)
        };
        let realised = if self.target_hz > 0 {
            iterations_per_sec / f64::from(self.target_hz)
        } else {
            f64::NAN
        };
        // Divided by the iterations that SLEPT, not by every iteration: an
        // overrunning loop sleeps rarely, and dividing its few genuine wake
        // latencies by the full iteration count would understate the tail it
        // exists to expose.
        let wake_late_avg_us = if self.wake_late_samples > 0 {
            saturate_u32(self.wake_late_total_us / self.wake_late_samples)
        } else {
            0
        };

        WindowVerdict {
            health: timebase_health(realised, self.discarded_us),
            drift_us: self.drift_us,
            iterations_per_sec,
            steps_per_sec,
            realised,
            wake_late_avg_us,
            wake_late_max_us: saturate_u32(self.wake_late_max_us),
            deadline_overrun_us: saturate_u32(self.overrun_us),
            discarded_ms: saturate_u32(self.discarded_us / 1000),
            gap_count: self.gap_count,
        }
    }
}

/// The wall-anchored pacing planner.
///
/// Construct once per loop, call [`Timebase::plan`] once per iteration with
/// the wall time elapsed since the loop epoch, run that many fixed-`step_us`
/// physics sub-steps, then sleep until `epoch + deadline_us()`.
#[derive(Debug, Clone)]
pub struct Timebase {
    step_us: u64,
    gap_threshold_us: u64,
    max_catchup_steps: u32,
    sim_us: u64,
    discarded_us: u64,
    steps_done: u64,
    catchup_steps: u64,
    gap_count: u32,
    max_gap_us: u64,
    session_invalid: bool,
}

impl Timebase {
    /// Create a timebase.
    ///
    /// `step_us` is clamped to at least 1: a zero step would divide by zero on
    /// the very first decision, and this type is what paces the sim thread —
    /// it must not be the thing that panics it.
    pub fn new(step_us: u64, gap_threshold_us: u64, max_catchup_steps: u32) -> Self {
        Self {
            step_us: step_us.max(1),
            gap_threshold_us,
            max_catchup_steps: max_catchup_steps.max(1),
            sim_us: 0,
            discarded_us: 0,
            steps_done: 0,
            catchup_steps: 0,
            gap_count: 0,
            max_gap_us: 0,
            session_invalid: false,
        }
    }

    /// A timebase that never catches up and never discards — one step per
    /// iteration, exactly the pre-2026-09-05 loop expressed in this type.
    ///
    /// Kept so the drift metric can be exercised against the recorded
    /// incident cadence, and so the regression that motivated this module is
    /// demonstrated rather than asserted from arithmetic.
    pub fn without_catchup(step_us: u64) -> Self {
        Self::new(step_us, u64::MAX, 1)
    }

    /// Microseconds of wall time since the loop epoch. Monotone.
    pub fn sim_time_us(&self) -> u64 {
        self.sim_us
    }

    /// Microseconds of physics actually simulated. Monotone, and equal to
    /// [`Timebase::sim_time_us`] on a healthy run.
    pub fn integrated_time_us(&self) -> u64 {
        self.sim_us.saturating_sub(self.discarded_us)
    }

    /// Wall offset of the next iteration's deadline. The caller sleeps until
    /// `epoch + deadline_us()`; there is no separate deadline to re-base,
    /// which is the whole of the fix.
    pub fn deadline_us(&self) -> u64 {
        self.sim_us
    }

    /// `wall − sim_us`. Positive when the clock is behind the wall.
    pub fn drift_us(&self, wall_elapsed_us: u64) -> i64 {
        let drift = i128::from(wall_elapsed_us) - i128::from(self.sim_us);
        drift.clamp(i128::from(i64::MIN), i128::from(i64::MAX)) as i64
    }

    /// Total wall time this session could not simulate.
    pub fn discarded_us(&self) -> u64 {
        self.discarded_us
    }

    /// Physics sub-steps executed this session.
    pub fn steps_done(&self) -> u64 {
        self.steps_done
    }

    /// Sub-steps executed beyond one per iteration.
    pub fn catchup_steps(&self) -> u64 {
        self.catchup_steps
    }

    /// Number of gaps recorded this session.
    pub fn gap_count(&self) -> u32 {
        self.gap_count
    }

    /// Largest single deficit recorded this session.
    pub fn max_gap_us(&self) -> u64 {
        self.max_gap_us
    }

    /// Latched on the first gap and never cleared for the life of the loop.
    /// A session that lost wall time must not be able to hide it by
    /// reconfiguring the build.
    pub fn session_invalid(&self) -> bool {
        self.session_invalid
    }

    /// Fixed physics sub-step, microseconds.
    pub fn step_us(&self) -> u64 {
        self.step_us
    }

    /// Decide and apply one iteration's plan.
    ///
    /// Wall time is the ONLY input. Catch-up used to be suppressible — clamped
    /// to a single sub-step while the ground-impact impulse was live, or while
    /// the outbound MAVLink channel was over half full — and both suppressions
    /// were removed rather than tuned. Neither bought anything and both
    /// reintroduced, on the exact schedule this module exists to remove, the
    /// wall-versus-sim divergence that injured the aircraft on 2026-09-05.
    /// See [`Timebase::decide`] for the measurements.
    pub fn plan(&mut self, wall_elapsed_us: u64) -> TickPlan {
        let plan = self.decide(wall_elapsed_us);
        self.apply(&plan);
        plan
    }

    /// Sub-steps the gap threshold can pay for, at this step size.
    ///
    /// [`MAX_CATCHUP_STEPS`] and `GAP_THRESHOLD_US / STEP_US` coincide at the
    /// 400 Hz default (8 × 2500 == 20 000) and nothing enforced it, so at any
    /// other accepted rate the allowance outran the budget. At `--tick-rate
    /// 250` a 31 ms deficit bought 7 × 4000 µs of replay PLUS the 11 ms
    /// declared lost: 39 ms of clock against 31 ms of wall, the clock leading
    /// the wall by two whole steps and 28 ms of physics replayed under a held
    /// actuator command — past the very PX4 IMU clamp [`GAP_THRESHOLD_US`]
    /// exists to stay inside.
    fn budget_steps(&self) -> u64 {
        Self::budget_steps_u64(self.step_us, self.gap_threshold_us)
    }

    /// The single arithmetic both budget accessors share.
    fn budget_steps_u64(step_us: u64, gap_threshold_us: u64) -> u64 {
        (gap_threshold_us / step_us.max(1)).max(1)
    }

    /// [`Timebase::budget_steps`] as a free calculation, for a caller choosing
    /// the `max_catchup_steps` to construct with.
    ///
    /// [`MAX_CATCHUP_STEPS`] is the 400 Hz figure and nothing more. It bounds
    /// the allowance from ABOVE as well as below, so passing it at a FASTER
    /// accepted rate caps catch-up short of the threshold: `--tick-rate 800`
    /// (an exact divisor of 1 000 000, so `validate_tick_rate` accepts it) has
    /// a 1250 µs step and a 16-step budget, and the constant would absorb only
    /// 8 × 1250 = 10 ms of a 20.1 ms stall — declaring the other 10 ms lost,
    /// latching `session_invalid`, and refusing the NEXT ARMING on a host that
    /// lost nothing and that the PX4 IMU clamp would have covered in full.
    pub fn budget_steps_at(step_us: u64, gap_threshold_us: u64) -> u32 {
        saturate_u32(Self::budget_steps_u64(step_us, gap_threshold_us))
    }

    /// The pure decision. Takes nothing but the current counters and the wall.
    ///
    /// - Regime A, `deficit < step_us`: one step. Bit-identical to the pre-fix
    ///   loop, except that the residue is retained in `wall − sim_us` instead
    ///   of being deleted. That retention is the entire behavioural change.
    /// - Regime B, `step_us <= deficit <= gap_threshold_us`: sub-step to close
    ///   the deficit, up to the allowance. Lateness is paid off, never deleted.
    /// - Regime C, `deficit > gap_threshold_us`: do not try to replay it. A
    ///   held actuator command stops being a defensible input past the FC's
    ///   rate-loop bandwidth, so the excess is declared lost — loudly, and
    ///   permanently for the session.
    ///
    /// # Two suppressions that used to live here, and why neither does
    ///
    /// The allowance was clamped to a single sub-step while the ground-impact
    /// impulse was live, and again while the outbound MAVLink channel was over
    /// half full. Both are gone. They are recorded here because both look
    /// obviously prudent and both were measured to be the opposite.
    ///
    /// **Backpressure.** The stated rationale was that catch-up is upstream
    /// work which turns a scheduling problem into a downstream backlog. It is
    /// not: `SimulationLoop::run` calls `sample_and_send_sensors` exactly ONCE
    /// per iteration whatever `steps` says, and `step_physics` sends nothing at
    /// all, so catch-up adds zero outbound messages. `HIL_GPS` is rate-limited
    /// on the sim clock, so catch-up brings its cadence UP TO the configured
    /// rate and never past it. Nor does catch-up add physics work in
    /// aggregate: `sim_us` can never lead the wall by more than one step, so
    /// the sub-step rate is bounded by `1 / step_us` however the steps are
    /// distributed between iterations. What the clamp DID buy was a deficit
    /// that grew unpaid at exactly the loop's lateness until it crossed
    /// [`Timebase::gap_threshold_us`] — at which point a host that had lost
    /// nothing it could not replay latched `session_invalid` and had its next
    /// arming refused with a message about mains power. An FC reboot stops
    /// draining `mav_tx` for 20-60 s, which is enough on a marginally late
    /// host.
    ///
    /// **The impact impulse.** The stated rationale was that compressing the
    /// 100 ms deceleration window into one averaged sample destroys the event.
    /// It cannot: `SimulationLoop::ground_override` SUBSTITUTES a constant for
    /// the whole emission while the impulse is live, discarding the sub-step
    /// mean entirely, so the emitted accelerations are bit-identical at one
    /// sub-step or eight. Only `interval_dt` — the IMU noise sigma, and the
    /// span PX4 integrates over — changes. And PX4 integrates each emitted
    /// sample over the WALL interval between arrivals, so clamping catch-up
    /// there stretched the impulse's wall span by exactly the loop's lateness
    /// ratio: 40 emissions at the recorded 2935 µs incident cadence span
    /// 117.4 ms of wall against 100 ms of declared impulse, handing the EKF
    /// 1.174x the velocity the ground actually removed. That is the same
    /// over-integration mechanism, and the same magnitude, as the injury this
    /// module exists to prevent — injected at touchdown, the one moment
    /// CLAUDE.md documents the EKF as most fragile.
    fn decide(&self, wall_elapsed_us: u64) -> TickPlan {
        let deficit_us = wall_elapsed_us.saturating_sub(self.sim_us);
        let gap = deficit_us > self.gap_threshold_us;

        // At least one step, always: a zero-step iteration would stall the
        // loop and starve the FC of the samples its voters are counting.
        // At most the allowance, so one iteration can never replay an
        // unbounded backlog of physics under a held actuator command.
        let allowance = u64::from(self.max_catchup_steps).min(self.budget_steps());
        let steps = (deficit_us / self.step_us).min(allowance).max(1);

        // What is declared lost is everything this iteration did NOT absorb —
        // `deficit − steps × step_us` — never `deficit − gap_threshold_us`.
        //
        // The two coincide only when the step divides the threshold exactly —
        // 8 × 2500 == 20 000 at 400 Hz, and likewise at 250, 200, 500 and
        // 1000 Hz. They do NOT at every accepted rate: `--tick-rate 625` has a
        // 1600 µs step and a 12-step budget, so 19.2 ms is absorbed against a
        // 20 ms threshold and 800 µs would go unaccounted under the threshold
        // form. They parted company much more violently for the two
        // suppressions described above, which is how this was found: the
        // threshold form absorbed 2.5 ms while claiming 20 ms, so the
        // unabsorbed 17.5 ms was neither simulated nor discarded, `wall − sim`
        // parked at `gap_threshold_us − step_us`, and every later iteration
        // re-entered the gap branch for the rest of the session.
        let absorbed_us = steps.saturating_mul(self.step_us);
        let discarded_us = if gap {
            deficit_us.saturating_sub(absorbed_us)
        } else {
            0
        };

        TickPlan {
            steps: saturate_u32(steps),
            discarded_us,
            gap,
            deficit_us,
        }
    }

    /// Fold a decision into the counters.
    fn apply(&mut self, plan: &TickPlan) {
        let advance = u64::from(plan.steps)
            .saturating_mul(self.step_us)
            .saturating_add(plan.discarded_us);
        let new_sim = self.sim_us.saturating_add(advance);
        // Discarded time may only be recorded for time the clock actually
        // moved through. Without this, a saturated `sim_us` with a still-growing
        // `discarded_us` would make `integrated_us` run BACKWARDS, which is the
        // one thing that permanently silences the GPS rate limiter.
        let moved = new_sim - self.sim_us;
        let recorded_discard = plan.discarded_us.min(moved);

        self.sim_us = new_sim;
        self.discarded_us = self.discarded_us.saturating_add(recorded_discard);
        self.steps_done = self.steps_done.saturating_add(u64::from(plan.steps));
        self.catchup_steps = self
            .catchup_steps
            .saturating_add(u64::from(plan.steps.saturating_sub(1)));

        if plan.gap {
            self.gap_count = self.gap_count.saturating_add(1);
            self.max_gap_us = self.max_gap_us.max(plan.deficit_us);
            self.session_invalid = true;
        }

        debug_assert!(
            self.identity_holds(),
            "sim_us must equal steps_done × step_us + discarded_us"
        );
    }

    /// Invariant 1. Returns true when the identity cannot be checked because
    /// the counters have saturated, which only happens after ~584 000 years
    /// of wall time or a deliberately adversarial input.
    fn identity_holds(&self) -> bool {
        match self
            .steps_done
            .checked_mul(self.step_us)
            .and_then(|s| s.checked_add(self.discarded_us))
        {
            Some(expected) => expected == self.sim_us,
            None => true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 400 Hz.
    const STEP_US: u64 = 2500;

    /// Wall microseconds per loop iteration measured on 2026-09-05: 341 Hz
    /// against a 400 Hz target.
    const INCIDENT_ITERATION_US: u64 = 2935;

    /// Iterations at the incident cadence that make up ~63 s of wall — the
    /// length of the session that ended in the crash.
    const INCIDENT_ITERATIONS: u64 = 21_479;

    fn fixed() -> Timebase {
        Timebase::new(STEP_US, GAP_THRESHOLD_US, MAX_CATCHUP_STEPS)
    }

    /// Drive a timebase through a constant-cadence wall sequence.
    fn drive(tb: &mut Timebase, iteration_us: u64, iterations: u64) -> u64 {
        for i in 1..=iterations {
            tb.plan(i * iteration_us);
        }
        iterations * iteration_us
    }

    #[test]
    fn sim_time_tracks_wall_clock_when_every_tick_is_late() {
        let mut tb = fixed();
        let wall_us = drive(&mut tb, INCIDENT_ITERATION_US, INCIDENT_ITERATIONS);

        assert!(
            wall_us.saturating_sub(tb.sim_time_us()) < STEP_US,
            "wall-anchored clock must track wall time to within one step: \
             wall {wall_us} µs, sim {} µs, short by {} µs",
            tb.sim_time_us(),
            wall_us - tb.sim_time_us()
        );
        assert_eq!(
            tb.discarded_us(),
            0,
            "a host that is merely slow loses no time — every microsecond is \
             simulated, just in fewer, longer iterations"
        );

        // The same wall sequence through the pre-fix pacing: one step per
        // iteration, so the clock counts ticks instead of wall time.
        let mut pre_fix = Timebase::without_catchup(STEP_US);
        drive(&mut pre_fix, INCIDENT_ITERATION_US, INCIDENT_ITERATIONS);
        let short_by = wall_us - pre_fix.sim_time_us();
        // Exact, not a lower bound: there is no RNG and no clock anywhere on
        // this path, so the shortfall is fully determined by the cadence. A
        // `>` bound would let a regression that halved it still pass.
        assert_eq!(
            short_by,
            INCIDENT_ITERATIONS * (INCIDENT_ITERATION_US - STEP_US),
            "this assertion IS the bug: the pre-fix clock advances {} µs \
             against {wall_us} µs of wall, and PX4 integrates the emitted \
             rates over the wall interval, not this one",
            pre_fix.sim_time_us()
        );
    }

    #[test]
    fn the_2026_09_05_window_is_an_error_not_a_caution() {
        let table = [
            // The incident: 341 iterations/s against a 400 Hz target.
            (0.8525_f64, 0_u64, Health::Faulted),
            (0.96, 0, Health::Degraded),
            (1.0, 0, Health::Ok),
            // A single lost microsecond outranks a perfect rate.
            (1.0, 1, Health::Faulted),
        ];

        for (realised, discarded_us, expected) in table {
            assert_eq!(
                timebase_health(realised, discarded_us),
                expected,
                "realised {realised}, discarded {discarded_us} µs must classify as \
                 {expected:?}. src/tui.rs:174-176 carried a 320.0 Hz caution floor, so \
                 it rendered the incident's 341 Hz as an acceptable YELLOW through a \
                 session in which the aircraft must not have been flown."
            );
        }
    }

    #[test]
    fn drift_metric_reports_the_incident_magnitude() {
        // One second of wall at the incident cadence with catch-up disabled,
        // which is precisely what the pre-fix loop did.
        let mut tb = Timebase::without_catchup(STEP_US);
        let wall_us = drive(&mut tb, INCIDENT_ITERATION_US, 341);

        let drift_us = tb.drift_us(wall_us);
        assert!(
            (drift_us - 148_300).abs() <= 500,
            "the ULog measured +148.3 ms of skew per second of wall; this drift \
             metric reports {drift_us} µs over {wall_us} µs of wall"
        );
    }

    /// The exact `gyro_integral_dt` recorded in log101.ulg on 2026-09-05 —
    /// the hole the loop has to survive without lying about it.
    const INCIDENT_GAP_US: u64 = 296_704;

    #[test]
    fn catchup_is_bounded_to_the_px4_imu_clamp_window() {
        let mut tb = fixed();
        let plan = tb.plan(INCIDENT_GAP_US);

        assert_eq!(
            plan.steps, MAX_CATCHUP_STEPS,
            "a 296.7 ms hole may be absorbed only up to PX4's own imu_max_dt"
        );
        assert!(
            plan.gap,
            "past the clamp window it is a gap, not a catch-up"
        );
        assert_eq!(
            plan.discarded_us,
            INCIDENT_GAP_US - GAP_THRESHOLD_US,
            "everything past the clamp window is declared lost, by name"
        );
        assert_eq!(
            tb.sim_time_us(),
            INCIDENT_GAP_US,
            "the loss goes in the counter, never in the clock: sim_us stays \
             anchored to wall so the sleep deadline and the GPS limiter do not rewind"
        );
        assert_eq!(
            tb.integrated_time_us(),
            GAP_THRESHOLD_US,
            "HIL_GPS reads this, so (FC hrt elapsed) − (time_utc_usec elapsed) \
             equals discarded_us exactly — the ULog cross-check that produced \
             this diagnosis stays alive after the fix"
        );
        assert!(tb.session_invalid(), "a gap latches for the whole session");
        assert_eq!(tb.gap_count(), 1);
        assert_eq!(tb.max_gap_us(), INCIDENT_GAP_US);
    }

    #[test]
    fn a_twenty_millisecond_stall_is_absorbed_and_is_not_a_gap() {
        let mut tb = fixed();
        let plan = tb.plan(GAP_THRESHOLD_US);

        assert_eq!(plan.steps, MAX_CATCHUP_STEPS);
        assert!(
            !plan.gap,
            "GAP_THRESHOLD_US is PX4's imu_max_dt = 2 × EKF2_PREDICT_US. \
             Tightening the catch-up budget past PX4's own clamp reintroduces \
             the eph-1.86 m-against-42 m-error overconfidence this boundary exists \
             to stay inside."
        );
        assert_eq!(plan.discarded_us, 0);
        assert_eq!(tb.sim_time_us(), GAP_THRESHOLD_US);
        assert_eq!(tb.integrated_time_us(), GAP_THRESHOLD_US);
        assert!(!tb.session_invalid());
    }

    /// Sim time in an impulse window. 100 ms, matching
    /// `loop_runner::IMPACT_DURATION_US`, which this module cannot see.
    const IMPACT_DURATION_US: u64 = 100_000;

    /// The impulse must cover the WALL interval it declares, not 17% more.
    ///
    /// `ground_override` substitutes a constant for the whole emission while
    /// the impulse is live — the sub-step mean is discarded — so the emitted
    /// accelerations do not depend on the sub-step count at all. What DOES
    /// depend on it is how much wall time the 100 ms window spans, and PX4
    /// integrates each emitted sample over the wall interval between arrivals.
    /// Suppressing catch-up here therefore over-delivered the landing impulse
    /// to the EKF by exactly the loop's lateness ratio.
    #[test]
    fn the_impact_impulse_covers_the_wall_interval_it_declares() {
        let mut tb = fixed();
        let mut wall_us = 0_u64;
        while tb.sim_time_us() < IMPACT_DURATION_US {
            wall_us += INCIDENT_ITERATION_US;
            tb.plan(wall_us);
        }

        assert!(
            wall_us.abs_diff(IMPACT_DURATION_US) < INCIDENT_ITERATION_US,
            "with catch-up the impulse's 100 ms of sim time is 100 ms of wall \
             time to within one iteration; PX4's integral of the substituted \
             constant then equals the delta-v the ground actually removed. \
             Saw {wall_us} µs of wall for {} µs of sim",
            tb.sim_time_us()
        );

        // The suppressed shape, for comparison: one step per iteration is what
        // clamping the allowance to 1 produced, and it is what
        // `Timebase::without_catchup` still expresses.
        let mut suppressed = Timebase::without_catchup(STEP_US);
        let mut suppressed_wall_us = 0_u64;
        while suppressed.sim_time_us() < IMPACT_DURATION_US {
            suppressed_wall_us += INCIDENT_ITERATION_US;
            suppressed.plan(suppressed_wall_us);
        }
        assert_eq!(
            suppressed_wall_us,
            40 * INCIDENT_ITERATION_US,
            "40 emissions of 2.5 ms at the incident cadence"
        );
        assert_eq!(
            suppressed_wall_us, 117_400,
            "117.4 ms of wall for a 100 ms impulse — the EKF is handed 1.174x \
             the velocity the ground removed, which is the same over-integration \
             mechanism and the same magnitude as the 2026-09-05 injury"
        );
    }

    /// Backpressure is not a reason to stop simulating wall time.
    ///
    /// Catch-up emits nothing extra — `sample_and_send_sensors` runs once per
    /// ITERATION whatever the sub-step count — so withholding it while the FC
    /// is not draining bought no channel depth at all. What it bought was a
    /// deficit growing unpaid until it crossed the gap threshold.
    #[test]
    fn catchup_does_not_yield_to_channel_pressure() {
        let mut tb = fixed();
        let plan = tb.plan(4 * STEP_US);

        assert_eq!(
            plan.steps, 4,
            "a four-step deficit is paid off in four sub-steps: catch-up adds \
             zero outbound messages, so a full mav_tx channel is not a reason \
             to leave wall time unsimulated"
        );
        assert_eq!(plan.discarded_us, 0, "backpressure is not lost time");
        assert!(
            !tb.session_invalid(),
            "a busy FC must not invalidate a session"
        );
    }

    #[test]
    fn a_gap_absorbs_the_whole_budget_and_records_the_rest() {
        let mut tb = fixed();
        let plan = tb.plan(INCIDENT_GAP_US);

        assert_eq!(plan.steps, MAX_CATCHUP_STEPS);
        assert!(
            plan.gap,
            "time past the clamp window is still lost, and still named"
        );
        assert_eq!(
            plan.discarded_us,
            INCIDENT_GAP_US - GAP_THRESHOLD_US,
            "what is lost is what this iteration did not absorb, which now that \
             nothing clamps the allowance is exactly `deficit − gap_threshold`"
        );
        assert_eq!(
            tb.sim_time_us(),
            INCIDENT_GAP_US,
            "the clock re-anchors to the wall on every gap iteration"
        );
        assert!(tb.session_invalid());
    }

    /// A permanently back-pressured, marginally late host must finish clean.
    ///
    /// This is the shape a routine FC reboot produces: `mav_tx` stops draining
    /// for 20-60 s while the host runs 0.4% behind. With catch-up suppressed
    /// the deficit grew 10 µs per iteration until it crossed the 20 ms
    /// threshold, and the session latched `session_invalid` — refusing the
    /// NEXT ARMING, permanently, with a message about mains power, on a host
    /// that had lost nothing it could not replay.
    #[test]
    fn a_backpressured_but_healthy_host_never_latches_the_session() {
        const LATE_ITERATION_US: u64 = 2510;
        const SESSION_US: u64 = 10 * 1_000_000;

        let mut tb = fixed();
        let mut wall_us = 0_u64;

        while wall_us < SESSION_US {
            wall_us += LATE_ITERATION_US;
            tb.plan(wall_us);
        }

        assert_eq!(
            tb.gap_count(),
            0,
            "0.4% late is 10 µs of deficit per iteration, and catch-up pays it \
             off every second iteration once it reaches one step. Nothing here \
             is a gap; saw {}",
            tb.gap_count()
        );
        assert_eq!(tb.discarded_us(), 0);
        assert!(
            !tb.session_invalid(),
            "the arming gate refuses on this latch and does not clear until the \
             daemon is restarted, so a downstream stall the daemon caused \
             nothing of must never set it"
        );
    }

    /// Deterministic jitter. A dependency-free LCG so the adversarial
    /// sequence is identical on every host and in every rerun — a pacing
    /// hardening test that only sometimes exercises the bad input is not a
    /// hardening test.
    struct Lcg(u64);

    impl Lcg {
        fn next(&mut self) -> u64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            self.0
        }
    }

    #[test]
    fn sim_time_is_monotone_under_an_adversarial_wall_sequence() {
        const ITERATIONS: u32 = 10_000;
        let mut tb = fixed();
        let mut rng = Lcg(0x5EED_1234_5678_9ABC);
        let mut wall_us: u64 = 0;
        let mut last_sim = 0_u64;
        let mut last_integrated = 0_u64;

        for i in 0..ITERATIONS {
            wall_us = match i % 3 {
                // Jitter around the nominal cadence.
                0 => wall_us.saturating_add(2000 + rng.next() % 2000),
                // A wall clock that does not advance at all.
                1 => wall_us,
                // A hole.
                _ => wall_us.saturating_add(rng.next() % 400_000),
            };

            let plan = tb.plan(wall_us);

            assert!(plan.steps >= 1, "a plan must never stall the loop");
            assert!(
                plan.steps <= MAX_CATCHUP_STEPS,
                "a plan must never exceed the PX4 clamp window"
            );
            assert!(
                tb.sim_time_us() >= last_sim,
                "sim_us must never rewind: hitl-sensors/src/gps.rs holds \
                 last_update_time and last_output_time in this timebase, and a \
                 backward jump silences GPS for the rest of the session"
            );
            assert!(
                tb.integrated_time_us() >= last_integrated,
                "integrated_us must never rewind either — HIL_GPS.time_usec reads it"
            );
            assert!(
                wall_us.saturating_sub(tb.sim_time_us()) <= GAP_THRESHOLD_US,
                "the clock may never lag the wall by more than PX4's imu_max_dt; \
                 any excess is folded into discarded_us and reported"
            );

            last_sim = tb.sim_time_us();
            last_integrated = tb.integrated_time_us();
        }

        // The far end of the number line, probed on its own timebase: pushed
        // into the shared sequence it pins `wall_us` at the maximum and every
        // later iteration becomes degenerate, so the sequence above would stop
        // testing anything.
        let mut saturating = fixed();
        saturating.plan(u64::MAX);
        let saturated_sim = saturating.sim_time_us();
        let saturated_integrated = saturating.integrated_time_us();
        assert_eq!(saturated_integrated, GAP_THRESHOLD_US);

        // Asked to advance past the end of the range, the counters stop rather
        // than wrap. `integrated_us = sim_us − discarded_us` is the fragile
        // one: a saturated sim_us with a still-growing discarded_us would run
        // it backwards.
        for _ in 0..8 {
            saturating.plan(u64::MAX);
            assert!(saturating.sim_time_us() >= saturated_sim);
            assert!(saturating.integrated_time_us() >= saturated_integrated);
        }
    }

    #[test]
    fn a_loop_that_honours_the_deadline_stays_anchored_on_both_sides() {
        const ITERATIONS: u32 = 10_000;
        // Jitter wider than one step on purpose. A 0-800 µs overshoot never
        // reaches `deficit >= STEP_US`, so catch-up never engages and both
        // assertions below hold by construction — the test would have proved
        // nothing about the Regime A/B boundary it is named for.
        const JITTER_US: u64 = 6_000;
        let mut tb = fixed();
        let mut rng = Lcg(0xC0FF_EE00_1234_5678);
        let mut wall_us: u64 = 0;
        let mut caught_up = false;

        for _ in 0..ITERATIONS {
            // What the real loop does: sleep to the deadline, then measure.
            wall_us = tb
                .deadline_us()
                .max(wall_us)
                .saturating_add(rng.next() % JITTER_US);

            let before = wall_us.saturating_sub(tb.sim_time_us());
            assert!(
                before <= GAP_THRESHOLD_US,
                "a deadline-honouring caller never enters plan() past the clamp"
            );

            let plan = tb.plan(wall_us);
            assert!(
                (1..=3).contains(&plan.steps),
                "a {JITTER_US} µs overshoot buys at most two extra steps; \
                 {} means the allowance is not bounded by the jitter",
                plan.steps
            );
            caught_up |= plan.steps > 1;

            assert!(
                tb.sim_time_us() <= wall_us + STEP_US,
                "the clock may lead the wall only by the step it cannot subdivide"
            );
        }

        assert!(
            caught_up,
            "the jitter has to cross the Regime A/B boundary or this test only \
             ever exercises the one-step path"
        );
        assert_eq!(tb.discarded_us(), 0);
        assert!(!tb.session_invalid());
    }

    #[test]
    fn catchup_never_outruns_the_budget_the_gap_threshold_affords() {
        // 250 Hz is an accepted, documented rate. `MAX_CATCHUP_STEPS` and
        // `GAP_THRESHOLD_US / STEP_US` coincide only at 400 Hz, and nothing
        // enforced it: at a 4000 µs step the allowance of 8 bought 7 replay
        // steps plus an 11 ms discard against a 31 ms deficit, putting the
        // clock 8 ms AHEAD of the wall and replaying 28 ms of physics under a
        // held actuator command.
        const STEP_250HZ_US: u64 = 4_000;
        const WALL_US: u64 = 31_000;

        let mut tb = Timebase::new(STEP_250HZ_US, GAP_THRESHOLD_US, MAX_CATCHUP_STEPS);
        let plan = tb.plan(WALL_US);

        assert_eq!(
            plan.steps, 5,
            "the budget at a 4000 µs step is 20 000 / 4000 = 5 steps, not the \
             400 Hz allowance of 8"
        );
        assert_eq!(tb.sim_time_us(), WALL_US, "5 × 4000 + 11 000 == 31 000");
        assert!(
            tb.sim_time_us() <= WALL_US + STEP_250HZ_US,
            "the clock may lead the wall only by the step it cannot subdivide, \
             at every accepted tick rate — not only at 400 Hz"
        );
        assert_eq!(tb.discarded_us(), WALL_US - GAP_THRESHOLD_US);
    }

    /// The mirror of the test above, on the other side of 400 Hz.
    ///
    /// `MAX_CATCHUP_STEPS` bounds the allowance from ABOVE as well as below, so
    /// at a step SHORTER than 2500 µs the constant caps catch-up short of the
    /// threshold. `--tick-rate 800` is accepted (`1_000_000 % 800 == 0`): with
    /// the constant, a 20.1 ms stall absorbed 8 × 1250 = 10 ms and declared
    /// 10.1 ms lost — latching `session_invalid` and refusing the next arming
    /// on a host the PX4 IMU clamp would have covered in full.
    #[test]
    fn catchup_uses_the_whole_budget_at_rates_faster_than_400_hz() {
        const STEP_800HZ_US: u64 = 1_250;
        const WALL_US: u64 = 20_100;

        let cap = Timebase::budget_steps_at(STEP_800HZ_US, GAP_THRESHOLD_US);
        assert_eq!(
            cap, 16,
            "20 000 / 1250 == 16; the 400 Hz constant of {MAX_CATCHUP_STEPS} would halve it"
        );

        let mut tb = Timebase::new(STEP_800HZ_US, GAP_THRESHOLD_US, cap);
        let plan = tb.plan(WALL_US);

        assert_eq!(plan.steps, 16, "the full 20 ms budget, not 8 steps of it");
        assert_eq!(
            plan.discarded_us,
            WALL_US - GAP_THRESHOLD_US,
            "only the 100 µs past the PX4 clamp is lost; with the 400 Hz constant this \
             was 10.1 ms"
        );
        assert_eq!(tb.sim_time_us(), WALL_US, "16 × 1250 + 100 == 20 100");
    }

    /// At the 400 Hz default the derived cap must be exactly the documented
    /// constant, or this change moved the healthy path.
    #[test]
    fn the_derived_catchup_cap_is_the_documented_constant_at_400_hz() {
        assert_eq!(
            Timebase::budget_steps_at(STEP_US, GAP_THRESHOLD_US),
            MAX_CATCHUP_STEPS,
            "8 × 2500 µs is exactly GAP_THRESHOLD_US; if these ever disagree the constant's \
             own doc comment is wrong"
        );
        // And it never returns zero, whatever it is handed.
        assert_eq!(Timebase::budget_steps_at(0, 0), 1);
        assert_eq!(Timebase::budget_steps_at(u64::MAX, GAP_THRESHOLD_US), 1);
    }

    #[test]
    fn catchup_cannot_diverge() {
        // Every iteration is assumed to consume 25 ms of work against a 20 ms
        // catch-up budget: the loop can never win.
        const WORK_US: u64 = 25_000;
        const ITERATIONS: u64 = 10_000;
        let mut tb = fixed();

        for i in 1..=ITERATIONS {
            let wall_us = i * WORK_US;
            tb.plan(wall_us);
            assert!(
                wall_us - tb.sim_time_us() <= GAP_THRESHOLD_US,
                "each iteration either reduces the deficit or discards the \
                 excess; neither branch can grow it, so no catch-up spiral is \
                 representable"
            );
        }

        assert!(
            tb.session_invalid(),
            "the loop settles loud and bounded, not silent"
        );
        assert_eq!(tb.gap_count(), ITERATIONS as u32);
    }

    #[test]
    fn a_healthy_session_emits_no_events_and_never_flags() {
        // 0.995 of the target cadence, held for ten minutes.
        const ITERATION_US: u64 = 2513;
        const SESSION_US: u64 = 600 * 1_000_000;
        const WINDOW_US: u64 = 1_000_000;

        let mut tb = fixed();
        let mut wall_us = 0_u64;
        let mut window_start_us = 0_u64;
        let mut window_iterations = 0_u64;
        let mut window_steps = 0_u64;

        while wall_us < SESSION_US {
            wall_us += ITERATION_US;
            let plan = tb.plan(wall_us);
            window_iterations += 1;
            window_steps += u64::from(plan.steps);

            if wall_us - window_start_us >= WINDOW_US {
                let verdict = WindowSample {
                    wall_us: wall_us - window_start_us,
                    iterations: window_iterations,
                    steps: window_steps,
                    target_hz: 400,
                    drift_us: tb.drift_us(wall_us),
                    discarded_us: 0,
                    ..Default::default()
                }
                .verdict();

                assert_eq!(
                    verdict.health,
                    Health::Ok,
                    "a host holding 99.5% of cadence is a normal laptop, not a \
                     fault. A metric that cries wolf here is one nobody reads \
                     when it finally matters."
                );
                window_start_us = wall_us;
                window_iterations = 0;
                window_steps = 0;
            }
        }

        assert_eq!(tb.gap_count(), 0);
        assert_eq!(tb.discarded_us(), 0);
        assert!(!tb.session_invalid());
    }

    #[test]
    fn a_degenerate_configuration_cannot_panic_the_sim_thread() {
        // `--tick-rate 0` reached the divide at loop_runner.rs:306 with no
        // validation at all. This type is what paces the sim thread; it must
        // not be the thing that kills it.
        let mut tb = Timebase::new(0, 0, 0);
        for wall_us in [0, 1, GAP_THRESHOLD_US, u64::MAX] {
            let plan = tb.plan(wall_us);
            assert!(plan.steps >= 1);
        }
        assert!(tb.sim_time_us() >= tb.integrated_time_us());
    }

    #[test]
    fn a_window_with_no_measurable_duration_is_not_a_fault() {
        let verdict = WindowSample {
            target_hz: 400,
            ..Default::default()
        }
        .verdict();

        assert_eq!(
            verdict.health,
            Health::Ok,
            "the first window of every session has no duration yet; classifying \
             absence of evidence as a fault makes the metric an alarm nobody reads"
        );
    }

    #[test]
    fn the_verdict_separates_a_slow_host_from_a_slow_simulation() {
        // The post-fix shape of the incident: the clock is correct (400
        // steps/s) and the host is slow (341 iterations/s).
        const INCIDENT_WAKE_LATE_US: u64 = 435;
        let descheduled = WindowSample {
            wall_us: 1_000_000,
            iterations: 341,
            steps: 400,
            target_hz: 400,
            wake_late_total_us: 341 * INCIDENT_WAKE_LATE_US,
            wake_late_samples: 341,
            wake_late_max_us: 900,
            overrun_us: 0,
            drift_us: 0,
            discarded_us: 0,
            gap_count: 0,
        }
        .verdict();

        assert_eq!(descheduled.health, Health::Faulted);
        assert!((descheduled.steps_per_sec - 400.0).abs() < 1.0);
        assert!((descheduled.iterations_per_sec - 341.0).abs() < 1.0);
        assert_eq!(
            descheduled.wake_late_avg_us, INCIDENT_WAKE_LATE_US as u32,
            "wake lateness of about a sixth of the tick budget, with no deadline \
             overrun at all, is the descheduled diagnosis"
        );
        assert_eq!(descheduled.deadline_overrun_us, 0);

        // A COMPUTE-bound host at the same iteration rate. Note what does NOT
        // distinguish it: steps_per_sec is 400 here too, because catch-up pins
        // the clock to the wall whatever the loop is short of. The two windows
        // are told apart by wake lateness against deadline overrun, and by
        // nothing else the verdict carries.
        let compute_bound = WindowSample {
            wall_us: 1_000_000,
            iterations: 341,
            steps: 400,
            target_hz: 400,
            // No iteration slept: every one was already past its deadline.
            wake_late_total_us: 0,
            wake_late_samples: 0,
            wake_late_max_us: 0,
            overrun_us: 341 * INCIDENT_WAKE_LATE_US,
            drift_us: 0,
            discarded_us: 0,
            gap_count: 0,
        }
        .verdict();

        assert_eq!(compute_bound.health, Health::Faulted);
        assert_eq!(
            (
                compute_bound.steps_per_sec.round(),
                compute_bound.iterations_per_sec.round()
            ),
            (
                descheduled.steps_per_sec.round(),
                descheduled.iterations_per_sec.round()
            ),
            "the rate pair is IDENTICAL for a compute-bound and a descheduled \
             host, so a log line telling the reader to compare steps_per_sec \
             against tick_rate_hz to tell them apart names a rule that cannot \
             fire"
        );
        assert_eq!(compute_bound.wake_late_avg_us, 0);
        assert_eq!(
            compute_bound.deadline_overrun_us,
            (341 * INCIDENT_WAKE_LATE_US) as u32,
            "zero wake lateness with a non-zero deadline overrun is the \
             compute-bound diagnosis, and this figure has to leave the loop for \
             it to be readable at all"
        );
    }
}
