//! Main simulation loop running at 400 Hz

use crate::state::{SimulationConfig, SimulationState};
use crate::timebase::{Health, Timebase, WindowSample, WindowVerdict, GAP_THRESHOLD_US};
use crossbeam_channel::{Receiver, Sender, TryRecvError};
use hitl_physics::{
    rk4_step, throttle_to_omega_with_config, total_motor_current, BatteryConfig, PhysicsConfig,
    QuadrotorState,
};
use mavlink::ardupilotmega::{HilSensorUpdatedFlags, MavMessage, HIL_GPS_DATA, HIL_SENSOR_DATA};
use protocol::ActuatorOutputs;
pub use protocol::SimulationStats;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::watch;
use tracing::{debug, error, info, trace, warn};

/// How often the loop pushes a snapshot to `stats_tx`. The TUI redraws at
/// ~2 Hz so anything faster is wasted, anything slower lags the header.
const STATS_PUBLISH_INTERVAL: Duration = Duration::from_millis(500);

/// How often the rolling tick-rate / latency window resets. The window is
/// independent of the publish cadence — between window resets the snapshot
/// carries the previously rolled-up values.
const STATS_WINDOW_INTERVAL: Duration = Duration::from_secs(5);

/// Minimum interval between sensor-drop warning log lines. Prevents log spam
/// at 400 Hz when the FC is slow to consume HIL_SENSOR messages.
const DROP_WARN_INTERVAL: Duration = Duration::from_secs(5);

/// How long without an actuator command before we consider the FC disconnected
/// and zero all motor commands. At 400 Hz, 100 ms = 40 missed commands.
const ACTUATOR_STALE_TIMEOUT: Duration = Duration::from_millis(100);

/// Minimum interval between stale-actuator warning log lines.
const STALE_WARN_INTERVAL: Duration = Duration::from_secs(5);

/// Minimum interval between out-of-terrain-coverage warning log lines.
const TERRAIN_MISS_WARN_INTERVAL: Duration = Duration::from_secs(5);

/// If the HIL_SENSOR channel stays full continuously for this long, escalate
/// to `error!` — the FC has likely disconnected without the serial layer
/// detecting it yet.
const DROP_ERROR_THRESHOLD: Duration = Duration::from_secs(2);

/// Length of the ground-impact deceleration impulse, in microseconds of SIM
/// time.
///
/// Sim time, not iterations. A catch-up iteration advances the clock by up to
/// eight fixed steps at once, so a counter of iterations compresses the
/// impulse by exactly the factor the loop is behind — and the impulse is the
/// only thing that tells the EKF what removed the vehicle's velocity. Without
/// it the EKF rejects GPS innovations and dead-reckons underground: 37 m of
/// divergence at a 6.4 m/s impact (CLAUDE.md, "ground impact needs
/// deceleration impulse"). Compressing it is the same failure, arrived at from
/// the other side.
const IMPACT_DURATION_US: u64 = 100_000;

/// Minimum downward speed (m/s) that generates an impact impulse. Below this a
/// landing is gentle enough that the clamp alone does not confuse the EKF.
const IMPACT_VEL_THRESHOLD: f64 = 0.5;

/// Surface normal of level ground in NED. Down is positive, so "up" is -1.
const LEVEL_NORMAL_NED: [f32; 3] = [0.0, 0.0, -1.0];

/// How far the surface normal may tilt from vertical while still counting as
/// level. Below this the flat-ground accelerometer path is used verbatim: an
/// exact `[0, 0, -g]`, never a rotation of the NED gravity vector. That exact
/// path exists because even a fraction of a degree of spurious tilt integrates
/// into tens of metres of EKF position drift over a flight.
const LEVEL_NORMAL_EPS: f64 = 1e-4;

/// Static friction coefficient between the airframe and the ground. tan(theta)
/// above this and a parked drone starts to slide, i.e. slopes steeper than
/// about 31 degrees.
const GROUND_FRICTION_COEFF: f64 = 0.6;

/// Per-tick lateral damping while the drone is stuck to the ground. At 400 Hz
/// this bleeds off horizontal speed almost immediately, which is what keeps a
/// landed drone from wandering.
const GROUND_STATIC_DAMPING: f64 = 0.9;

/// Per-tick lateral damping while sliding downhill. Light enough that gravity
/// along the surface wins, so a drone on a steep slope actually slides.
const GROUND_SLIDING_DAMPING: f64 = 0.999;

/// True when the surface is level to within `LEVEL_NORMAL_EPS`.
fn is_level(normal: [f32; 3]) -> bool {
    (normal[0] as f64).abs() < LEVEL_NORMAL_EPS && (normal[1] as f64).abs() < LEVEL_NORMAL_EPS
}

/// Attitude of a body resting on a surface with the given NED normal, keeping
/// its current heading. Body Z (down in FRD) is planted into the surface and
/// the nose is the current yaw projected onto the surface plane.
fn attitude_on_surface(normal: [f32; 3], yaw: f64) -> nalgebra::UnitQuaternion<f64> {
    if is_level(normal) {
        // Exact level attitude — no matrix round-trip, so no residual tilt.
        return nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw);
    }

    let up = nalgebra::Vector3::new(normal[0] as f64, normal[1] as f64, normal[2] as f64);
    let z_body = -up; // FRD: body Z points down, into the surface.
    let heading = nalgebra::Vector3::new(yaw.cos(), yaw.sin(), 0.0);

    // Project the heading onto the surface plane. Degenerate only if the
    // surface is vertical and faces along the heading, which real terrain
    // sampled from a heightfield cannot produce.
    let x_body = heading - z_body * heading.dot(&z_body);
    let Some(x_body) = x_body.try_normalize(1e-9) else {
        return nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw);
    };
    let y_body = z_body.cross(&x_body);

    nalgebra::UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(
        nalgebra::Matrix3::from_columns(&[x_body, y_body, z_body]),
    ))
}

/// Component of gravity along the surface, in NED. Zero on level ground.
fn downslope_accel(normal: [f32; 3], gravity: f64) -> [f64; 3] {
    let n = nalgebra::Vector3::new(normal[0] as f64, normal[1] as f64, normal[2] as f64);
    let g = nalgebra::Vector3::new(0.0, 0.0, gravity);
    let tangential = g - n * g.dot(&n);
    [tangential.x, tangential.y, tangential.z]
}

/// Mag and baro cadence, in microseconds of SIM time. 20 000 µs is 50 Hz.
///
/// A sim-time schedule, not a tick divider. Their timestamps already come from
/// the simulation clock, so once an iteration can carry several physics steps
/// a divider decouples the cadence from the stamp: the loop would emit a
/// "50 Hz" mag whose stamps advance 5 ms apart.
///
/// The budget on the other side is PX4's voter. `VehicleMagnetometer.cpp:46`
/// and `VehicleAirData.cpp:47` declare a mag or baro dead after 300 ms without
/// a sample, and MAG #0 / BARO #0 TIMEOUT were the exact alarms the FC raised
/// on 2026-09-05. A schedule that can only ever run early keeps a factor of
/// fifteen between this cadence and that timeout.
const SENSOR_50HZ_PERIOD_US: u64 = 20_000;

/// Start of the next fixed-period slot strictly after `now_us`.
///
/// A fixed grid rather than `now + period`, for two reasons. The schedule
/// cannot drift with the iteration cadence; and a catch-up that crosses
/// several slots fires once instead of emitting a burst to make up the slots
/// it passed — a burst PX4 would fuse as several distinct measurements of the
/// same instant.
///
/// The bound this buys is on the SCHEDULE, and the emitted cadence is one
/// iteration looser: an update fires on the first emission at or after its
/// slot, and that emission can overshoot the slot by the whole sub-step span
/// it carries. The real bound on observed spacing is therefore `period +
/// steps × step_us`, at most `period + GAP_THRESHOLD_US` — 40 ms against
/// PX4's 300 ms voter, a factor of seven and a half. Do not tighten the period
/// on the strength of the `period` figure alone.
fn next_slot_us(now_us: u64, period_us: u64) -> u64 {
    let period = period_us.max(1);
    (now_us / period).saturating_add(1).saturating_mul(period)
}

/// IMU flags: accel + gyro (updated every tick at 400 Hz)
const IMU_FLAGS: HilSensorUpdatedFlags = HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_XACC
    .union(HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_YACC)
    .union(HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_ZACC)
    .union(HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_XGYRO)
    .union(HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_YGYRO)
    .union(HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_ZGYRO);

/// Mag flags (updated at ~50 Hz)
const MAG_FLAGS: HilSensorUpdatedFlags = HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_XMAG
    .union(HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_YMAG)
    .union(HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_ZMAG);

/// Baro flags (updated at ~50 Hz)
const BARO_FLAGS: HilSensorUpdatedFlags = HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_ABS_PRESSURE
    .union(HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_DIFF_PRESSURE)
    .union(HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_PRESSURE_ALT)
    .union(HilSensorUpdatedFlags::HIL_SENSOR_UPDATED_TEMPERATURE);

/// How often the timebase health window rolls up.
///
/// One second, not the 5 s the other stats window uses. At the 2026-09-05
/// cadence a 5 s window puts the first error line at t≈40 s — one second AFTER
/// the ground impact it exists to prevent. A 1 s window reaches it by t≈37 s.
const TIMEBASE_HEALTH_WINDOW: Duration = Duration::from_secs(1);

/// Shortest truncated window the shutdown path will still grade.
///
/// The window a stop interrupts is worth publishing — otherwise a run shorter
/// than one full window reports nothing and a fitness check grades the
/// nothing. Below about a tenth of a second the realised ratio is dominated by
/// the quantisation of a handful of iterations, so it is noise rather than a
/// measurement and is discarded instead.
const MIN_FINAL_WINDOW: Duration = Duration::from_millis(100);

/// Minimum interval between repeated timebase fault lines while the fault
/// persists. Matches `DROP_WARN_INTERVAL` / `STALE_WARN_INTERVAL` /
/// `TERRAIN_MISS_WARN_INTERVAL` above, for one reason: TUI log delivery is a
/// lossy 512-slot `try_send` (`src/main.rs`, `src/tui.rs`), so the rate limit
/// has to live at the emit site. Backpressure must never be what decides which
/// diagnostic the operator sees.
///
/// A health *transition* bypasses this — the first line of a fault, and the
/// line saying it cleared, are always emitted.
const TIMEBASE_WARN_INTERVAL: Duration = Duration::from_secs(5);

/// How often the healthy path says so out loud.
///
/// Without it a clean log is ambiguous: it could mean the timebase is fine, or
/// it could mean the metric is dead. One line a minute makes silence mean
/// something.
const TIMEBASE_HEARTBEAT_INTERVAL: Duration = Duration::from_secs(60);

/// Fractional deviation from the target rate above which the 5 s window line
/// is promoted from `debug!` to `info!`. 1% of 400 Hz is 4 Hz.
const REALISED_DEVIATION_REPORT: f64 = 0.01;

/// Width of one wake-lateness histogram bucket, microseconds.
const WAKE_LATE_BUCKET_US: u64 = 100;

/// Number of wake-lateness buckets. 64 x 100 µs covers 0-6.4 ms, which spans
/// the 2.5 ms tick budget with room for the overshoots that matter; anything
/// beyond lands in the overflow bucket.
const WAKE_LATE_BUCKETS: usize = 64;

/// The airborne sample one physics sub-step produced.
///
/// "Airborne" is the whole point: this carries the specific force the airframe
/// itself transmits (thrust plus drag over mass) and nothing else. The
/// ground-contact and impact overrides never reach it — see
/// [`SimulationLoop::ground_override`], which cannot see this type at all.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct SubStepSample {
    /// Body-frame specific force, m/s².
    accel_body: [f64; 3],
    /// Body-frame angular rate, rad/s.
    gyro_body: [f64; 3],
}

/// Running mean of the airborne sub-step samples in one loop iteration.
///
/// One `HIL_SENSOR` leaves per iteration whatever the sub-step count, carrying
/// the MEAN rate over `steps × step_us`. That is not a smoothing choice: PX4
/// on real hardware ignores `HIL_SENSOR.time_usec` and integrates each emitted
/// rate over the WALL interval between arrivals, so the mean is the only value
/// whose integral over that interval equals the motion actually simulated.
#[derive(Debug, Clone, Copy, Default)]
struct SubStepAccumulator {
    sum: SubStepSample,
    count: u32,
}

impl SubStepAccumulator {
    /// Fold one sub-step in, returning a new accumulator.
    ///
    /// The first sample is stored VERBATIM rather than added to a zeroed sum.
    /// `0.0 + (-0.0)` is `+0.0` in IEEE 754, which is a different bit pattern
    /// and therefore a different `f32` on the wire — and the single-sub-step
    /// case has to be byte-identical to the loop this replaces.
    fn plus(self, sample: SubStepSample) -> Self {
        if self.count == 0 {
            return Self {
                sum: sample,
                count: 1,
            };
        }
        Self {
            sum: SubStepSample {
                accel_body: std::array::from_fn(|i| self.sum.accel_body[i] + sample.accel_body[i]),
                gyro_body: std::array::from_fn(|i| self.sum.gyro_body[i] + sample.gyro_body[i]),
            },
            count: self.count + 1,
        }
    }

    /// Mean over the sub-steps folded in.
    ///
    /// With one sub-step this returns that sub-step's own values bit for bit —
    /// no addition, no division — which is what makes the healthy path
    /// unchanged.
    ///
    /// An empty accumulator would return `SubStepSample::default()`, a
    /// `[0,0,0]` specific force indistinguishable from free fall, and it would
    /// leave as a perfectly valid `HIL_SENSOR`. `Timebase::decide` ends with
    /// `.max(1)` so it cannot happen today, but that guarantee lives in a
    /// separately versioned file and the plan reserves the right to add a rate
    /// governor to `plan()`. Assert it here, where the consequence is.
    fn mean(self) -> SubStepSample {
        debug_assert!(
            self.count > 0,
            "an iteration must run at least one sub-step: a zero-sub-step \
             emission is a free-fall accelerometer reading on the wire"
        );
        if self.count <= 1 {
            return self.sum;
        }
        let n = f64::from(self.count);
        SubStepSample {
            accel_body: std::array::from_fn(|i| self.sum.accel_body[i] / n),
            gyro_body: std::array::from_fn(|i| self.sum.gyro_body[i] / n),
        }
    }
}

/// Seconds of simulation one iteration's emission covers.
///
/// `steps × dt`, and exactly `dt` when `steps == 1`: `f64::from(1u32) * dt`
/// is `dt`. Handing `imu.sample` this larger interval is correct rather than a
/// hazard — the reading IS the mean of n sub-steps, and averaging n
/// independent samples reduces sigma by √n, which is precisely what the IMU's
/// `density / sqrt(dt)` computes.
fn interval_dt(steps: u32, dt: f64) -> f64 {
    f64::from(steps) * dt
}

/// One iteration's emission: the interval it covers, the two clocks it is
/// stamped with, and the mean airborne sample over its sub-steps.
///
/// The two clocks are deliberately separate. `sim_time_us` is wall-anchored
/// and drives `HIL_SENSOR`; `integrated_time_us` is `sim − discarded` and
/// drives `HIL_GPS`, so the ULog cross-check
/// `(hrt elapsed) − (time_utc_usec elapsed)` reads out exactly the wall time
/// the daemon admits it could not simulate.
#[derive(Debug, Clone, Copy)]
struct IterationEmission {
    /// Seconds of simulation this emission covers: `steps × dt`.
    interval_dt: f64,
    /// `HIL_SENSOR.time_usec`, and the timebase the GPS rate limiter runs on.
    sim_time_us: u64,
    /// `HIL_GPS.time_usec`.
    integrated_time_us: u64,
    /// Mean airborne specific force over the sub-steps, body frame.
    mean_accel_body: [f64; 3],
    /// Mean angular rate over the sub-steps, body frame.
    mean_gyro_body: [f64; 3],
}

/// Specific force an accelerometer reads while airborne, in the body frame.
///
/// Thrust plus drag over mass — the only forces the airframe transmits.
/// Gravity is deliberately absent: an accelerometer in free fall reads zero.
///
/// Computed while the physics write lock is already held, once per sub-step,
/// so a catch-up costs no extra lock acquisitions and the healthy path costs
/// exactly what it did before.
fn airborne_specific_force_body(q: &QuadrotorState, physics: &PhysicsConfig) -> [f64; 3] {
    let (thrust_body, _) = q.compute_motor_forces(physics);
    let drag_body = q.compute_drag(physics);
    let force_body = thrust_body + drag_body;
    [
        force_body[0] / physics.mass_kg,
        force_body[1] / physics.mass_kg,
        force_body[2] / physics.mass_kg,
    ]
}

/// The `SimulationStats` a `ConfigureBuild` should leave behind.
///
/// Cumulative message counts restart with the new build — the header is meant
/// to describe the build in force, not an aggregate across builds. The
/// timebase group deliberately SURVIVES: a session that lost wall time must
/// not be able to hide it by changing the build, and defaulting
/// `timebase_health` back to `Ok` told the browser annunciator the fault had
/// cleared — a green "recovered" banner to the person flying, retracted by a
/// fresh CRITICAL up to a second later.
///
/// Extracted as a free function so the rule is a unit test rather than a
/// comment inside a 400 Hz loop.
fn stats_after_reconfigure(previous: &SimulationStats) -> SimulationStats {
    SimulationStats {
        timebase_drift_us: previous.timebase_drift_us,
        timebase_discarded_ms: previous.timebase_discarded_ms,
        timebase_gap_count: previous.timebase_gap_count,
        timebase_max_gap_ms: previous.timebase_max_gap_ms,
        catchup_steps: previous.catchup_steps,
        // The whole pacing group, not just the p99. These four measure the
        // HOST, not the build, and `tb_session` — the thing they are computed
        // from — is not touched by a reconfigure. Carrying `wake_late_p99_us`
        // while zeroing `wake_late_max_us` published, for up to the second
        // until the next roll-up, exactly the "window max under a session p99"
        // that both this field's own documentation and the assignment in
        // `roll_up_timebase_window` go out of their way to prevent.
        tick_rate_hz: previous.tick_rate_hz,
        steps_per_sec: previous.steps_per_sec,
        wake_late_avg_us: previous.wake_late_avg_us,
        wake_late_max_us: previous.wake_late_max_us,
        wake_late_p99_us: previous.wake_late_p99_us,
        deadline_overrun_us: previous.deadline_overrun_us,
        timebase_step_us: previous.timebase_step_us,
        session_invalid: previous.session_invalid,
        timebase_health: previous.timebase_health,
        timebase_worst_health: previous.timebase_worst_health,
        timebase_min_realised: previous.timebase_min_realised,
        ..Default::default()
    }
}

/// The text of a Regime C gap line.
///
/// A free function returning a `String` rather than a format string buried in
/// a macro inside a 400 Hz loop, because the WORDING is the whole value of
/// this line and it therefore has to be a unit test. Gaps are rare by
/// construction, so the allocation costs nothing that matters.
///
/// It predicts the FC symptom by name on purpose. On 2026-09-05 the flight
/// controller reported MAG #0 and BARO #0 TIMEOUT, and days went into looking
/// for an I2C fault on a board that has no I2C bus — those alarms were PX4's
/// 300 ms sensor voters firing as a CONSEQUENCE of the daemon stalling.
fn gap_message(deficit_us: u64, absorbed_us: u64, discarded_us: u64) -> String {
    format!(
        "DAEMON TIMEBASE GAP — the host stalled the simulation thread for {} ms. \
         {} ms were absorbed by sub-stepping (PX4's own imu_max_dt, \
         2 x EKF2_PREDICT_US) and {} ms could not be simulated at all and are \
         gone from this session. The FC will report MAG #0 / BARO #0 TIMEOUT as \
         a CONSEQUENCE of this gap (PX4 VehicleMagnetometer.cpp:46 / \
         VehicleAirData.cpp:47, 300 ms voter) — there is no I2C bus and nothing \
         is wrong with the hardware. This session is marked invalid. Remedy: \
         close the browser's terrain view, run on mains power, stop background \
         builds, then re-check with --selftest-timebase 30.",
        deficit_us / 1000,
        absorbed_us / 1000,
        discarded_us / 1000,
    )
}

/// Clamp a `u64` into a published `u32` field without wrapping.
///
/// Wrapping a millisecond counter would report a session that lost four
/// billion milliseconds as one that lost none — the failure mode the whole
/// metric exists to make impossible.
fn saturating_u32(value: u64) -> u32 {
    value.min(u64::from(u32::MAX)) as u32
}

/// Non-finite guard for a published `f32` metric.
///
/// A NaN serialises as JSON `null`, which then fails to deserialise back into
/// an `f32`. A metric that breaks the snapshot it travels in is worse than one
/// that reads zero.
fn finite_f32(value: f64) -> f32 {
    if value.is_finite() {
        value as f32
    } else {
        0.0
    }
}

/// Fixed-bucket histogram of wake lateness, in microseconds.
///
/// A percentile needs a distribution, and the sim thread must neither allocate
/// nor sort at 400 Hz. Buckets are O(1) per sample and about half a kilobyte
/// in total.
struct WakeLateHistogram {
    buckets: [u64; WAKE_LATE_BUCKETS],
    overflow: u64,
    count: u64,
    /// Largest value ever recorded, exactly.
    ///
    /// Kept so a quantile — which can only name a bucket's upper edge — is
    /// never reported above a value that was actually observed. A p99 above
    /// the max reads as a broken metric, and this number's whole job is to be
    /// believed on its first real incident.
    max_seen: u64,
}

impl WakeLateHistogram {
    fn new() -> Self {
        Self {
            buckets: [0; WAKE_LATE_BUCKETS],
            overflow: 0,
            count: 0,
            max_seen: 0,
        }
    }

    fn record(&mut self, wake_late_us: u64) {
        let bucket = (wake_late_us / WAKE_LATE_BUCKET_US) as usize;
        match self.buckets.get_mut(bucket) {
            Some(slot) => *slot += 1,
            None => self.overflow += 1,
        }
        self.count += 1;
        self.max_seen = self.max_seen.max(wake_late_us);
    }

    /// Upper edge of the bucket holding the given quantile, never above the
    /// largest value actually observed.
    ///
    /// A quantile landing past the last bucket returns the overflow floor, so
    /// an out-of-range answer reads as "at least this much" instead of
    /// silently clamping to the top in-range value and understating the tail.
    fn quantile_us(&self, q: f64) -> u64 {
        self.bucket_edge_us(q).min(self.max_seen)
    }

    fn bucket_edge_us(&self, q: f64) -> u64 {
        if self.count == 0 {
            return 0;
        }
        let target = (self.count as f64 * q).ceil() as u64;
        let target = target.clamp(1, self.count);

        let mut seen = 0_u64;
        for (index, samples) in self.buckets.iter().enumerate() {
            seen += samples;
            if seen >= target {
                return (index as u64 + 1) * WAKE_LATE_BUCKET_US;
            }
        }
        WAKE_LATE_BUCKETS as u64 * WAKE_LATE_BUCKET_US
    }

    /// Samples that fell past the last bucket.
    ///
    /// Reported beside the quantile so the quantile can be read honestly: a
    /// p99 pinned at the top edge means something different when a hundred
    /// samples were off the end than when one was.
    fn over_range(&self) -> u64 {
        self.overflow
    }
}

/// One second of timebase observations.
#[derive(Default, Clone, Copy)]
struct TimebaseWindow {
    iterations: u64,
    steps: u64,
    wake_late_total_us: u64,
    /// Iterations that actually slept. Only those carry wake latency; an
    /// iteration that overran its own deadline never slept, and its lateness
    /// is work time.
    wake_late_samples: u64,
    wake_late_max_us: u64,
    /// Wall time the loop was ALREADY past its deadline by when it reached the
    /// sleep, summed over the window.
    ///
    /// Not lost — owed. The deadline is `epoch + sim_us` and nothing re-bases
    /// it, so this interval is paid off as sub-steps on the following
    /// iterations. It is kept because it separates the two ways of being
    /// behind: overrun is our own work taking too long, wake lateness is the
    /// host refusing to schedule us. Published as
    /// `SimulationStats::deadline_overrun_us` and printed beside
    /// `wake_late_avg_us` in the fault lines, which is the only pair that
    /// actually tells those two apart.
    overrun_us: u64,
    /// `sensor_drops` at the window's start, so a fault line can report the
    /// window's delta. Drops rising alongside the fault means the FC stopped
    /// consuming; drops flat means the host would not schedule us. Those have
    /// nothing in common but their symptom.
    drops_at_start: u64,
    /// Session discarded time at the window's start, so the verdict grades
    /// THIS window rather than re-faulting for ever on one old gap.
    discarded_at_start: u64,
    /// Session gap count at the window's start, for the same reason.
    gaps_at_start: u32,
}

impl TimebaseWindow {
    /// `wake_late_us` is `Some` only when this iteration actually slept.
    fn record(&mut self, steps: u64, wake_late_us: Option<u64>, overrun_us: u64) {
        self.iterations += 1;
        self.steps += steps;
        if let Some(wake_late_us) = wake_late_us {
            self.wake_late_total_us += wake_late_us;
            self.wake_late_samples += 1;
            self.wake_late_max_us = self.wake_late_max_us.max(wake_late_us);
        }
        self.overrun_us += overrun_us;
    }

    fn restart(&mut self, drops: u64, discarded_us: u64, gap_count: u32) {
        *self = Self {
            drops_at_start: drops,
            discarded_at_start: discarded_us,
            gaps_at_start: gap_count,
            ..Default::default()
        };
    }
}

/// Session totals, kept for the shutdown post-mortem.
struct TimebaseSession {
    /// Sum of the per-iteration deadline overruns. See
    /// [`TimebaseWindow::overrun_us`].
    overrun_us: u64,
    wake_late_max_us: u64,
    min_realised: f64,
    /// Worst verdict any window produced. Sticky for the session.
    ///
    /// `SimulationStats::timebase_health` is overwritten every second, so a
    /// host that stalls and recovers finishes a run reporting `Ok`. Anything
    /// grading a whole session — `--selftest-timebase` above all — has to read
    /// a latch, not the last second.
    worst_health: Health,
    wake_late: WakeLateHistogram,
}

impl TimebaseSession {
    fn new() -> Self {
        Self {
            overrun_us: 0,
            wake_late_max_us: 0,
            // Nothing has been observed yet; any real window beats this.
            min_realised: f64::INFINITY,
            worst_health: Health::Ok,
            wake_late: WakeLateHistogram::new(),
        }
    }
}

/// What a window's verdict should produce. Decided without touching the log,
/// so the throttling is a unit test rather than a hope.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TimebaseAnnouncement {
    Silent,
    Heartbeat,
    Recovered { from: Health },
    Degraded,
    Faulted,
}

/// Escalation order. Only an increase bypasses the rate limit.
fn severity_rank(health: Health) -> u8 {
    match health {
        Health::Ok => 0,
        Health::Degraded => 1,
        Health::Faulted => 2,
    }
}

/// Emits the timebase health lines, at default log level and rate-limited at
/// the emit site.
struct TimebaseReporter {
    last_warning: Instant,
    last_heartbeat: Instant,
    /// Health as of the last line actually emitted — NOT the last window
    /// observed. Throttling against the last observation lets a verdict
    /// flapping across a threshold count as a fresh transition every second.
    reported: Health,
}

impl TimebaseReporter {
    fn new(now: Instant) -> Self {
        Self {
            last_warning: now,
            last_heartbeat: now,
            reported: Health::Ok,
        }
    }

    /// Decide what this window should say. Pure but for the reporter's own
    /// bookkeeping; `now` is a parameter so the intervals are testable.
    ///
    /// Two rules, in this order:
    /// 1. An escalation is never delayed.
    /// 2. Everything else waits out `TIMEBASE_WARN_INTERVAL`.
    fn decide(&mut self, health: Health, now: Instant) -> TimebaseAnnouncement {
        let escalated = severity_rank(health) > severity_rank(self.reported);
        let throttled =
            !escalated && now.saturating_duration_since(self.last_warning) < TIMEBASE_WARN_INTERVAL;

        if health == Health::Ok {
            if self.reported != Health::Ok {
                if throttled {
                    return TimebaseAnnouncement::Silent;
                }
                let from = self.reported;
                self.reported = Health::Ok;
                self.last_warning = now;
                self.last_heartbeat = now;
                return TimebaseAnnouncement::Recovered { from };
            }

            // Silence is ambiguous: it could mean healthy, or it could mean
            // the metric itself is dead. One line a minute settles it.
            if now.saturating_duration_since(self.last_heartbeat) >= TIMEBASE_HEARTBEAT_INTERVAL {
                self.last_heartbeat = now;
                return TimebaseAnnouncement::Heartbeat;
            }
            return TimebaseAnnouncement::Silent;
        }

        if throttled {
            return TimebaseAnnouncement::Silent;
        }
        self.reported = health;
        self.last_warning = now;

        match health {
            Health::Faulted => TimebaseAnnouncement::Faulted,
            _ => TimebaseAnnouncement::Degraded,
        }
    }

    /// Report one window.
    ///
    /// `steps_per_sec` beside `tick_rate_hz` says whether the CLOCK is still
    /// correct while the loop is behind — on 2026-09-05 that reads 400 against
    /// 341, which excludes the physics and terrain code in the first line
    /// anyone sees. It does NOT separate compute-bound from descheduled:
    /// catch-up holds `steps_per_sec` at the target on any host that is not
    /// discarding time. `wake_late_avg_us` against `deadline_overrun_us` is
    /// the pair that does, so both are in every fault line below.
    fn report(
        &mut self,
        verdict: &WindowVerdict,
        drops_delta: u64,
        channel_full_secs: Option<u64>,
    ) {
        // Signed, and reported as an OFFSET rather than as "behind".
        //
        // Once the clock is wall-anchored a small NEGATIVE drift is the
        // healthy resting state: `plan()` always returns at least one step, so
        // the clock sits up to one `step_us` ahead of the wall value it was
        // last given. A line reading "the clock is -2 ms behind wall time" on
        // a correctly paced loop is the kind of sentence that sends an
        // investigator the wrong way, which is the failure this whole change
        // exists to stop.
        let drift_ms = verdict.drift_us / 1000;

        match self.decide(verdict.health, Instant::now()) {
            TimebaseAnnouncement::Silent => {}
            TimebaseAnnouncement::Heartbeat => info!(
                tick_rate_hz = verdict.iterations_per_sec,
                steps_per_sec = verdict.steps_per_sec,
                drift_us = verdict.drift_us,
                wake_late_avg_us = verdict.wake_late_avg_us,
                "DAEMON TIMEBASE: ok"
            ),
            TimebaseAnnouncement::Recovered { from } => info!(
                was = from.as_str(),
                tick_rate_hz = verdict.iterations_per_sec,
                drift_us = verdict.drift_us,
                "DAEMON TIMEBASE: recovered — the loop is holding its cadence again"
            ),
            TimebaseAnnouncement::Degraded => warn!(
                health = verdict.health.as_str(),
                tick_rate_hz = verdict.iterations_per_sec,
                steps_per_sec = verdict.steps_per_sec,
                realised = verdict.realised,
                drift_us = verdict.drift_us,
                wake_late_avg_us = verdict.wake_late_avg_us,
                wake_late_max_us = verdict.wake_late_max_us,
                deadline_overrun_us = verdict.deadline_overrun_us,
                sensor_drops_window = drops_delta,
                channel_full_secs,
                "DAEMON TIMEBASE degraded — the loop is not holding its iteration \
                 cadence. The simulation clock is still tracking wall time \
                 (offset {drift_ms} ms; positive means behind, and up to one \
                 step ahead is normal), so nothing has been lost — but read \
                 wake_late_avg_us against deadline_overrun_us: wake lateness \
                 with no overrun means the host will not schedule the thread on \
                 time, overrun with no wake lateness means the iteration's own \
                 work does not fit in the period."
            ),
            TimebaseAnnouncement::Faulted => error!(
                health = verdict.health.as_str(),
                tick_rate_hz = verdict.iterations_per_sec,
                steps_per_sec = verdict.steps_per_sec,
                realised = verdict.realised,
                drift_us = verdict.drift_us,
                wake_late_avg_us = verdict.wake_late_avg_us,
                wake_late_max_us = verdict.wake_late_max_us,
                deadline_overrun_us = verdict.deadline_overrun_us,
                discarded_ms = verdict.discarded_ms,
                gap_count = verdict.gap_count,
                sensor_drops_window = drops_delta,
                channel_full_secs,
                "DAEMON TIMEBASE FAULTED — this host is not fit to fly HITL as \
                 configured. Clock offset {drift_ms} ms (positive means behind). \
                 Read wake_late_avg_us against deadline_overrun_us: wake lateness \
                 with no overrun is a host that will not schedule the thread, \
                 overrun with no wake lateness is work that does not fit the \
                 period. \
                 discarded_ms above zero means wall time the simulation could not \
                 account for at all: PX4 ignores HIL_SENSOR.time_usec and \
                 integrates every emitted rate over the wall interval between \
                 arrivals, so time the daemon could not simulate is motion the EKF \
                 fuses anyway. Do not fly. Remedy: close the browser's terrain \
                 view, run on mains power, stop background builds, then re-check \
                 with --selftest-timebase 30."
            ),
        }
    }
}

/// Main simulation loop
/// Per-phase tick timings, accumulated over the stats window.
///
/// The loop reported one aggregate latency, and an aggregate cannot say
/// which phase spent it: a terrain sampler taking eight lock acquisitions
/// and six coordinate resolutions per tick is invisible inside a total that
/// is otherwise healthy. Each phase is accumulated separately and reported
/// beside the total.
///
/// Nanoseconds, not microseconds. A whole RK4 step measures 264 ns on the
/// development host, so a microsecond accumulator would truncate every
/// physics tick to zero and report the phase as free.
#[derive(Default, Clone, Copy)]
struct PhaseAccumulator {
    actuator_ns: u64,
    terrain_ns: u64,
    physics_ns: u64,
    sensors_ns: u64,
    /// Nanoseconds spent BLOCKED acquiring the state lock, subtracted from
    /// whichever phase the acquisition sat in.
    ///
    /// A tick takes up to six acquisitions of the RwLock it shares with the
    /// 30 Hz WebSocket broadcaster. Blocking on it is not work, but it lands
    /// inside a phase's span and presents as work — which is exactly how the
    /// old aggregate `latency_us` misled the 2026-09-05 investigation.
    lock_wait_ns: u64,
}

impl PhaseAccumulator {
    /// Mean nanoseconds per tick for each phase over the window.
    fn per_tick(&self, ticks: u64) -> (u64, u64, u64, u64, u64) {
        let ticks = ticks.max(1);
        (
            self.actuator_ns / ticks,
            self.terrain_ns / ticks,
            self.physics_ns / ticks,
            self.sensors_ns / ticks,
            self.lock_wait_ns / ticks,
        )
    }

    /// The phase that consumed most of the window, so an overrun report can
    /// name something more useful than the total.
    fn dominant(&self) -> &'static str {
        let mut name = "actuator";
        let mut best = self.actuator_ns;
        for (candidate, value) in [
            ("terrain", self.terrain_ns),
            ("physics", self.physics_ns),
            ("sensors", self.sensors_ns),
            ("lock_wait", self.lock_wait_ns),
        ] {
            if value > best {
                best = value;
                name = candidate;
            }
        }
        name
    }
}

pub struct SimulationLoop {
    state: SimulationState,
    config: SimulationConfig,
    /// Where the flight is and what zero altitude means. Shared with the
    /// WebSocket task, which sets it from `ConfigureBuild`.
    origin: Arc<crate::origin::SharedOrigin>,
    actuator_rx: Receiver<ActuatorOutputs>,
    config_rx: Receiver<(PhysicsConfig, BatteryConfig, hitl_sensors::SensorsConfig)>,
    mav_tx: Sender<MavMessage>,
    stats: SimulationStats,
    /// Watch channel used by the TUI / web status widget to render live
    /// loop + drone state. `None` when nothing subscribes (tests, benches).
    stats_tx: Option<watch::Sender<SimulationStats>>,
    /// Nanoseconds THIS ITERATION spent sampling terrain, summed over its
    /// sub-steps. Zeroed by the loop once per iteration and accumulated by
    /// `step_physics`, which is the only place that touches the terrain cache,
    /// so the terrain cost is reported apart from the integration it sits
    /// inside and stays comparable to the physics span beside it whatever the
    /// sub-step count.
    tick_terrain_ns: u64,
    /// Nanoseconds the last tick spent BLOCKED on the state lock. Written by
    /// every phase that acquires it, read back by the loop and charged to the
    /// phase it occurred in, exactly as `tick_terrain_ns` is.
    tick_lock_wait_ns: u64,
    /// Total ticks executed since startup — used to identify the first
    /// tick for sensor-value logging, not surfaced in `SimulationStats`
    /// because the cumulative HIL counts already convey progress.
    total_ticks: u64,
    /// Tracks whether `ConfigureBuild` has been applied at least once so the
    /// header can show "no build configured" vs default values.
    build_configured: bool,
    /// Cached mag reading (refreshed on the `SENSOR_50HZ_PERIOD_US` schedule)
    last_mag: Option<hitl_sensors::MagReading>,
    /// Cached baro reading (refreshed on the `SENSOR_50HZ_PERIOD_US` schedule)
    last_baro: Option<hitl_sensors::BaroReading>,
    /// Sim time at which the magnetometer is next due. Zero so the first
    /// sample refreshes it, matching the old tick-zero divider.
    next_mag_us: u64,
    /// Sim time at which the barometer is next due.
    next_baro_us: u64,
    /// Last time a sensor-drop warning was emitted (rate-limits log spam).
    last_drop_warning: Instant,
    /// Number of drops accumulated since the last warning was emitted.
    drop_count_since_last_warning: u64,
    /// When the HIL_SENSOR channel first became full in the current
    /// contiguous run. `None` when the channel is not full.
    channel_full_since: Option<Instant>,
    /// Last time an actuator command was received from the FC. Used to detect
    /// FC disconnection: if no command arrives within ACTUATOR_STALE_TIMEOUT
    /// while motors are active, commands are zeroed and armed is cleared.
    last_actuator_time: Instant,
    /// Last time a stale-actuator warning was emitted (rate-limits log spam).
    last_stale_warning: Instant,
    /// Last time an out-of-terrain-coverage warning was emitted. Rate-limits
    /// what would otherwise be a 400 Hz log stream once the drone flies past
    /// the edge of the cached tile block.
    last_terrain_miss_warning: Instant,
    /// Last time a Regime C gap line was emitted. Rate-limits what a host in a
    /// chronic stall would otherwise turn into one allocating, synchronously
    /// written `error!` per iteration on the 400 Hz sim thread.
    last_gap_report: Instant,
    /// Gaps folded into `suppressed_gaps` since the last emitted gap line.
    /// Reported on the next line that emits, so a storm is named rather than
    /// silently swallowed.
    gaps_suppressed: u64,
    /// Set by step_physics when the ground clamp fires or the quad is at z=0.
    /// Read by sample_and_send_sensors to produce clean [0,0,-g] accel on ground.
    ground_contact_active: bool,
    /// Sim time at which the ground-impact deceleration impulse stops being
    /// reported. `None` before the first impact of the session.
    ///
    /// A deadline in sim time, not a countdown of iterations: see
    /// [`IMPACT_DURATION_US`]. It is self-expiring — nothing has to clear it —
    /// so a catch-up cannot end the impulse early and a stall cannot extend it.
    ground_impact_until_us: Option<u64>,
    /// Body-frame deceleration acceleration to report during ground impact (m/s²).
    ground_impact_accel: [f64; 3],
    /// Body-frame specific force to report while resting on the ground (m/s²).
    /// Exactly `[0, 0, -g]` on level ground; on a slope it is the gravity
    /// reaction rotated into the drone's resting attitude, which is what a real
    /// accelerometer on a tilted airframe reads.
    ground_rest_accel_body: [f64; 3],
    /// What the host said when this thread asked to be scheduled as a periodic
    /// real-time task, as `rtsched::Promotion::token()`.
    ///
    /// Set by [`SimulationLoop::set_realtime`] from INSIDE the spawned closure,
    /// because every mechanism `rtsched` uses is self-directed. Empty when
    /// nothing asked — `--no-realtime`, or a test constructing a loop it never
    /// runs on a promoted thread.
    ///
    /// EXPOSURE REDUCTION, NOT THE FIX. A `granted:` token here is not evidence
    /// the loop is keeping cadence; `timebase_health` is the field that says
    /// that.
    realtime_policy: String,
    /// Latched the first time the kernel is seen to have taken a granted
    /// contract back. Sticky for the session, and the reason the revocation is
    /// logged once rather than once per stats window.
    realtime_revoked: bool,
}

impl SimulationLoop {
    /// Create a new simulation loop
    pub fn new(
        config: SimulationConfig,
        actuator_rx: Receiver<ActuatorOutputs>,
        config_rx: Receiver<(PhysicsConfig, BatteryConfig, hitl_sensors::SensorsConfig)>,
        mav_tx: Sender<MavMessage>,
        origin: Arc<crate::origin::SharedOrigin>,
    ) -> Self {
        let state = SimulationState::new(config.clone());
        // Seed from the config actually in force, not the default: relying on
        // them matching is an invariant across two crates that nothing enforces.
        let gravity = config.physics.gravity;

        Self {
            state,
            config,
            origin,
            actuator_rx,
            config_rx,
            mav_tx,
            stats: SimulationStats::default(),
            tick_terrain_ns: 0,
            tick_lock_wait_ns: 0,
            stats_tx: None,
            total_ticks: 0,
            build_configured: false,
            last_mag: None,
            last_baro: None,
            next_mag_us: 0,
            next_baro_us: 0,
            last_drop_warning: Instant::now(),
            drop_count_since_last_warning: 0,
            channel_full_since: None,
            last_actuator_time: Instant::now(),
            last_stale_warning: Instant::now(),
            last_terrain_miss_warning: Instant::now(),
            last_gap_report: Instant::now(),
            gaps_suppressed: 0,
            ground_contact_active: true,
            ground_impact_until_us: None,
            ground_impact_accel: [0.0; 3],
            ground_rest_accel_body: [0.0, 0.0, -gravity],
            realtime_policy: String::new(),
            realtime_revoked: false,
        }
    }

    /// Attach a `watch::Sender` so the loop publishes live stats every
    /// `STATS_PUBLISH_INTERVAL`. Call once before `run()`.
    pub fn with_stats_publisher(mut self, tx: watch::Sender<SimulationStats>) -> Self {
        self.stats_tx = Some(tx);
        self
    }

    /// Record what the host did with this thread's request for a real-time
    /// scheduling class, so it reaches `SimulationStats` and the selftest JSON.
    ///
    /// Takes `&mut self` rather than being a builder ON PURPOSE. Every
    /// `rtsched` mechanism is self-directed — it can only be applied by the
    /// thread it applies to — so the promotion happens as the first statement
    /// inside the spawned closure, at which point the loop has already been
    /// moved into it and a `self`-consuming builder is no longer callable.
    pub fn set_realtime(&mut self, promotion: &rtsched::Promotion) {
        self.realtime_policy = promotion.token();
    }

    /// Get shared state handle for other threads
    pub fn state_handle(&self) -> SimulationState {
        self.state.clone()
    }

    /// Run the simulation loop (blocking)
    pub fn run(&mut self) {
        let dt = 1.0 / self.config.tick_rate_hz as f64;
        let target_hz = f64::from(self.config.tick_rate_hz);
        // The fixed physics sub-step, in microseconds. `validate_tick_rate`
        // accepts only rates that divide the microsecond and nanosecond grids
        // exactly, so this is `dt` expressed in the timebase's own units with
        // no residue — no phantom deficit for the new metric to blame on the
        // host's scheduler.
        let step_us = 1_000_000 / u64::from(self.config.tick_rate_hz.max(1));

        info!(
            tick_rate_hz = self.config.tick_rate_hz,
            gps_rate_hz = self.config.gps_rate_hz,
            ref_lat = self.config.reference_lat,
            ref_lon = self.config.reference_lon,
            "Starting simulation loop"
        );

        // Window-based stats — reset every interval so reported values reflect recent behaviour.
        let mut window_start = Instant::now();
        let mut window_ticks: u64 = 0;
        let mut window_latency_us: u64 = 0;
        let mut window_max_latency_us: u64 = 0;
        let mut window_phases = PhaseAccumulator::default();

        // Last time we pushed a snapshot to `stats_tx`.
        let mut last_stats_publish = Instant::now();

        // The loop epoch. Every timebase number is referenced to it, so a
        // reconfigure cannot erase the session's pacing history.
        let epoch = Instant::now();

        // The wall-anchored clock. `sim_us` counts microseconds of WALL time
        // since `epoch`, and the sleep deadline is literally `epoch + sim_us`,
        // so there is no separate schedule to re-base and nothing that can
        // delete lateness. The pre-fix loop did `next_tick = now` on every
        // late tick: 435 µs deleted per iteration, 4.9 s over the 2026-09-05
        // session, and nothing counted a microsecond of it.
        //
        // The catch-up cap is DERIVED from the step, not taken as
        // `MAX_CATCHUP_STEPS`. That constant is the 400 Hz figure (8 × 2500 µs
        // == GAP_THRESHOLD_US) and caps catch-up short of the threshold at any
        // faster accepted rate — see `Timebase::budget_steps_at`.
        let mut tb = Timebase::new(
            step_us,
            GAP_THRESHOLD_US,
            Timebase::budget_steps_at(step_us, GAP_THRESHOLD_US),
        );

        // The 1 s timebase health window, its reporter and the session totals
        // the shutdown post-mortem prints.
        let mut tb_window_start = epoch;
        let mut tb_window = TimebaseWindow::default();
        let mut tb_reporter = TimebaseReporter::new(epoch);
        let mut tb_session = TimebaseSession::new();

        // `sleep_until`, not `sleep(deadline - now)`.
        //
        // The free function recomputes its deadline from `Instant::now()`
        // *inside* the call, so whatever time passed between reading `now` and
        // entering it was added back on top of the interval. An absolute
        // deadline cannot drift that way.
        //
        // The sleeper keeps its default spin accuracy. Tightening it to 60 µs
        // was tried and reverted: the review costed the default's busy-spin at
        // 5% of a core (125 µs x 400 Hz), but the daemon's *entire* CPU usage
        // measures 1.4% of a core, and a same-session A/B of the two
        // accuracies came back at 1.4% either way. There is no 5% to reclaim
        // on this platform, and the tighter setting moved `max_latency_us`
        // from about 95 µs to 160 µs — worse jitter for nothing.
        let sleeper = spin_sleep::SpinSleeper::default();

        loop {
            // A third state-lock acquisition per iteration, and the only one
            // taken outside every phase span — so its blocking time used to be
            // attributed nowhere at all. Charged straight to `lock_wait_ns`
            // below rather than subtracted from a phase, because it belongs to
            // none of them.
            let running_check = Instant::now();
            let running = self.state.is_running();
            let running_lock_ns = running_check.elapsed().as_nanos() as u64;
            if !running {
                break;
            }

            // Sleep to the wall-anchored deadline. `deadline_us()` IS `sim_us`,
            // so a loop that is behind finds the deadline already past, sleeps
            // for nothing, and starts the next iteration carrying the whole
            // deficit — which the plan below then pays off in physics.
            let deadline = epoch + Duration::from_micros(tb.deadline_us());
            let before_sleep = Instant::now();

            // Two distinct populations, and conflating them is what made the
            // old `latency_us` trace point away from the cause.
            //
            // On the slept branch the interval between the deadline and the
            // next clock read IS wake latency — the host would not schedule
            // us. On the overrun branch no sleep happened, so that interval is
            // our own work taking too long and nothing to do with the
            // scheduler. Feeding it into the wake-latency histogram would make
            // a compute-bound loop and a descheduled one report identical
            // numbers.
            let (wake_late_us, overrun_us) = if before_sleep < deadline {
                sleeper.sleep_until(deadline);
                // One `Instant::now()` (~25 ns) against a 2500 µs budget.
                let late = Instant::now()
                    .saturating_duration_since(deadline)
                    .as_micros() as u64;
                (Some(late), 0)
            } else {
                (None, (before_sleep - deadline).as_micros() as u64)
            };

            let tick_start = Instant::now();

            match self.config_rx.try_recv() {
                Ok((new_physics, new_battery, new_sensors)) => {
                    info!("Reconfiguring simulation");
                    self.config.physics = new_physics;
                    self.config.battery = new_battery;
                    self.config.sensors = new_sensors;
                    // Pass the freshly-updated config so SimulationState stores the
                    // live values (battery capacity, cell count, sensors) before it
                    // resets inner state. Without this, the stale Arc inside
                    // SimulationState would be used for the battery reset, producing
                    // wrong SoC/voltage for any non-default build.
                    self.state.reconfigure(self.config.clone());
                    self.last_mag = None;
                    self.last_baro = None;
                    self.build_configured = true;
                    self.stats = stats_after_reconfigure(&self.stats);
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => break,
            }

            // Phase boundaries. Three extra `Instant::now()` calls per tick,
            // about 75 ns against a 2500 us budget — the price of being able
            // to say which phase moved when one does.
            let phase_start = Instant::now();
            self.tick_lock_wait_ns = 0;

            // Process any pending actuator commands (non-blocking)
            self.process_actuator_commands();
            let after_actuator = Instant::now();
            let lock_after_actuator = self.tick_lock_wait_ns;

            // Pace. How much physics this iteration owes is decided from wall
            // time alone, by a type with no clock of its own — so every pacing
            // decision the loop makes is reproducible in a unit test.
            let wall_us = epoch.elapsed().as_micros() as u64;
            let plan = tb.plan(wall_us);
            let sim_after_us = tb.sim_time_us();

            // Step physics. Records its terrain sampling into
            // `self.tick_terrain_ns` so that cost is reported separately from
            // the integration it is nested inside.
            //
            // Fixed dt, always — never a variable step sized to the deficit.
            // The motor mode has tau = 0.025 s, so a 296.7 ms step gives
            // |lambda·h| = 11.9 and an RK4 amplification around 610: the
            // integrator would detonate on precisely the stall it has to
            // survive.
            self.tick_terrain_ns = 0;
            let mut accumulator = SubStepAccumulator::default();
            for step in 1..=plan.steps {
                // Sub-step k ends at the iteration's end minus the steps still
                // to come, so the LAST sub-step is stamped with exactly the
                // time the emission carries and the sequence stays monotone
                // across a gap.
                let substep_end_us =
                    sim_after_us.saturating_sub(u64::from(plan.steps - step) * step_us);
                accumulator = accumulator.plus(self.step_physics(dt, substep_end_us));
            }
            let after_physics = Instant::now();
            let lock_after_physics = self.tick_lock_wait_ns;

            // Reported AFTER the physics phase boundary is taken. A gap line
            // allocates and writes synchronously; charged inside the physics
            // span it would make the daemon blame `hitl-physics` for the cost
            // of its own diagnostics, which is precisely the misattribution
            // `lock_wait_ns` and the phase split exist to remove.
            if plan.gap {
                let absorbed_us = u64::from(plan.steps).saturating_mul(step_us);
                self.report_gap(&plan, &tb, absorbed_us);
            }

            // Sample sensors and send HIL messages. ONE HIL_SENSOR leaves per
            // iteration whatever the sub-step count, carrying the mean rate
            // over `steps × step_us` — see `SubStepAccumulator`.
            let mean = accumulator.mean();
            self.sample_and_send_sensors(IterationEmission {
                interval_dt: interval_dt(plan.steps, dt),
                sim_time_us: sim_after_us,
                integrated_time_us: tb.integrated_time_us(),
                mean_accel_body: mean.accel_body,
                mean_gyro_body: mean.gyro_body,
            });
            let after_sensors = Instant::now();

            let terrain_ns = self.tick_terrain_ns;
            let lock_total_ns = self.tick_lock_wait_ns;
            // Each phase's span contains whatever time it spent BLOCKED on the
            // state lock. Charge that to `lock_wait_ns` and remove it from the
            // phase, or contention on the 30 Hz broadcaster reads as physics
            // getting slower.
            window_phases.actuator_ns += ((after_actuator - phase_start).as_nanos() as u64)
                .saturating_sub(lock_after_actuator);
            window_phases.terrain_ns += terrain_ns;
            // Terrain is measured inside the physics span, so remove it rather
            // than counting it twice. Saturating because the two readings come
            // from separate clock samples and can disagree by a few ns.
            window_phases.physics_ns += ((after_physics - after_actuator).as_nanos() as u64)
                .saturating_sub(terrain_ns)
                .saturating_sub(lock_after_physics.saturating_sub(lock_after_actuator));
            window_phases.sensors_ns += ((after_sensors - after_physics).as_nanos() as u64)
                .saturating_sub(lock_total_ns.saturating_sub(lock_after_physics));
            window_phases.lock_wait_ns += lock_total_ns + running_lock_ns;

            self.total_ticks += 1;
            window_ticks += 1;

            let latency_us = tick_start.elapsed().as_micros() as u64;
            window_latency_us += latency_us;
            if latency_us > window_max_latency_us {
                window_max_latency_us = latency_us;
            }

            // Roll up window stats every STATS_WINDOW_INTERVAL.
            if window_start.elapsed() >= STATS_WINDOW_INTERVAL {
                // Re-read the kernel's view of this thread's scheduling
                // contract. macOS withdraws a time-constraint contract SILENTLY
                // once a thread chronically overruns its declared budget, and a
                // real-time guarantee that quietly stopped applying is this
                // incident's shape a second time. One syscall per 5 s.
                self.check_realtime_contract();

                let window_secs = window_start.elapsed().as_secs_f64();
                // `tick_rate_hz` belongs to the 1 s health window below, which
                // is also what the verdict is computed from — a header that
                // disagrees with the colour beside it is the failure this
                // whole change exists to fix. This 5 s figure is local to the
                // lines that follow.
                let window_tick_rate_hz = (window_ticks as f64 / window_secs) as f32;
                self.stats.avg_latency_us = (window_latency_us as f64 / window_ticks as f64) as u32;
                self.stats.max_latency_us = window_max_latency_us as u32;

                // Keep the formatted log line as `debug!` for users who tail
                // logs at debug level. The TUI gets the same data via watch.
                let (actuator_ns, terrain_ns, physics_ns, sensors_ns, lock_wait_ns) =
                    window_phases.per_tick(window_ticks);

                // Promote the headline numbers to `info!` once the achieved
                // rate leaves the target's neighbourhood. The per-phase
                // breakdown below stays at `debug!` — it is for someone who
                // has already been told there is something to look at.
                let realised = f64::from(window_tick_rate_hz) / target_hz;
                if (realised - 1.0).abs() > REALISED_DEVIATION_REPORT {
                    info!(
                        tick_rate_hz = window_tick_rate_hz,
                        target_hz,
                        realised,
                        drift_us = tb.drift_us(epoch.elapsed().as_micros() as u64),
                        avg_latency_us = self.stats.avg_latency_us,
                        max_latency_us = self.stats.max_latency_us,
                        lock_wait_ns,
                        dominant_phase = window_phases.dominant(),
                        "sim window off target rate"
                    );
                }

                debug!(
                    tick_rate_hz = window_tick_rate_hz,
                    avg_latency_us = self.stats.avg_latency_us,
                    max_latency_us = self.stats.max_latency_us,
                    hil_sensor = self.stats.hil_sensor_count,
                    hil_gps = self.stats.hil_gps_count,
                    actuators = self.stats.actuator_count,
                    // Mean nanoseconds per tick, by phase. These sum to the
                    // work portion of avg_latency_us; the remainder is the
                    // config poll and the loop's own bookkeeping.
                    actuator_ns,
                    terrain_ns,
                    physics_ns,
                    sensors_ns,
                    lock_wait_ns,
                    "sim window stats"
                );

                window_start = Instant::now();
                window_ticks = 0;
                window_latency_us = 0;
                window_max_latency_us = 0;
                window_phases = PhaseAccumulator::default();
            }

            // Publish a live snapshot to the TUI / status subscribers.
            if last_stats_publish.elapsed() >= STATS_PUBLISH_INTERVAL {
                self.publish_stats();
                last_stats_publish = Instant::now();
            }

            tb_window.record(u64::from(plan.steps), wake_late_us, overrun_us);
            tb_session.overrun_us += overrun_us;
            if let Some(wake_late_us) = wake_late_us {
                tb_session.wake_late_max_us = tb_session.wake_late_max_us.max(wake_late_us);
                tb_session.wake_late.record(wake_late_us);
            }

            let tb_elapsed = tb_window_start.elapsed();
            if tb_elapsed >= TIMEBASE_HEALTH_WINDOW {
                self.roll_up_timebase_window(
                    tb_elapsed,
                    epoch.elapsed(),
                    &tb,
                    &tb_window,
                    &mut tb_session,
                    Some(&mut tb_reporter),
                );
                tb_window_start = Instant::now();
                tb_window.restart(self.stats.sensor_drops, tb.discarded_us(), tb.gap_count());
            }
        }

        // Close the window the shutdown interrupted, so a run shorter than one
        // full window still produces a measurement. Without this the watch
        // channel still held `SimulationStats::default()` — achieved rate 0.0,
        // health Ok — and `--selftest-timebase 1` graded that as a PASS having
        // measured nothing.
        //
        // The reporter is deliberately not passed: a truncated window is not
        // an event worth a fault line, only a number worth publishing.
        let final_elapsed = tb_window_start.elapsed();
        if tb_window.iterations > 0 && final_elapsed >= MIN_FINAL_WINDOW {
            self.roll_up_timebase_window(
                final_elapsed,
                epoch.elapsed(),
                &tb,
                &tb_window,
                &mut tb_session,
                None,
            );
        }
        // Publish once more so whatever reads the watch channel after `run()`
        // returns sees the session's real numbers rather than a snapshot up to
        // `STATS_PUBLISH_INTERVAL` stale.
        self.publish_stats();

        // Post-mortem. This is the best forensic artefact the daemon produces
        // and it prints at default level, because the population that hit the
        // 2026-09-05 incident was running headless with no TUI to read.
        let wall_us = epoch.elapsed().as_micros() as u64;
        let min_realised = if tb_session.min_realised.is_finite() {
            tb_session.min_realised
        } else {
            f64::NAN
        };
        info!(
            wall_s = wall_us as f64 / 1_000_000.0,
            sim_s = tb.sim_time_us() as f64 / 1_000_000.0,
            integrated_s = tb.integrated_time_us() as f64 / 1_000_000.0,
            steps = tb.steps_done(),
            catchup_steps = tb.catchup_steps(),
            // `wall − sim`, measured from the clock itself. Computed the same
            // way here and in the health window, so the post-mortem and the
            // published snapshot cannot disagree.
            drift_ms = tb.drift_us(wall_us) / 1000,
            // Our own overruns, kept apart from wake latency. Both being small
            // while the drift is large would mean the clock stopped tracking
            // the wall, which is the regression this whole change guards.
            deadline_overrun_ms = tb_session.overrun_us / 1000,
            worst_health = ?tb_session.worst_health,
            // The cross-check to run against the ULog:
            // `(FC hrt elapsed) − (sensor_gps.time_utc_usec elapsed)` should
            // equal this number.
            discarded_ms = tb.discarded_us() / 1000,
            gap_count = tb.gap_count(),
            max_gap_ms = tb.max_gap_us() / 1000,
            session_invalid = tb.session_invalid(),
            min_realised,
            wake_late_p99_us = tb_session.wake_late.quantile_us(0.99),
            wake_late_max_us = tb_session.wake_late_max_us,
            wake_late_over_range = tb_session.wake_late.over_range(),
            "Simulation loop stopped — timebase post-mortem"
        );
    }

    /// Close one timebase health window.
    ///
    /// Folds the window into the session totals and the published snapshot,
    /// and — when a reporter is supplied — into the log. Extracted so the
    /// window the shutdown interrupts can be closed the same way the periodic
    /// ones are, rather than being thrown away.
    fn roll_up_timebase_window(
        &mut self,
        window_elapsed: Duration,
        session_elapsed: Duration,
        tb: &Timebase,
        tb_window: &TimebaseWindow,
        tb_session: &mut TimebaseSession,
        tb_reporter: Option<&mut TimebaseReporter>,
    ) {
        let verdict = WindowSample {
            wall_us: window_elapsed.as_micros() as u64,
            iterations: tb_window.iterations,
            steps: tb_window.steps,
            target_hz: self.config.tick_rate_hz,
            wake_late_total_us: tb_window.wake_late_total_us,
            wake_late_samples: tb_window.wake_late_samples,
            wake_late_max_us: tb_window.wake_late_max_us,
            // The other population. Accumulated every iteration since this
            // window type was written and read by nothing until now, which
            // left the only figure that separates compute-bound from
            // descheduled inside the loop.
            overrun_us: tb_window.overrun_us,
            // `wall − sim` read straight off the clock, so the post-mortem
            // and the published snapshot cannot disagree.
            //
            // Deriving this from a deadline-reset accumulator put it one
            // window out of phase with the fault it describes: the reset that
            // deleted an overshoot happened on the NEXT iteration, so the
            // window that lost the time closed reporting `drift_us = 0`.
            drift_us: tb.drift_us(session_elapsed.as_micros() as u64),
            // Window DELTAS, not session totals. The verdict grades this
            // window; a session total here would re-fault for ever on one old
            // gap and the "recovered" line could never be reached.
            discarded_us: tb
                .discarded_us()
                .saturating_sub(tb_window.discarded_at_start),
            gap_count: tb.gap_count().saturating_sub(tb_window.gaps_at_start),
        }
        .verdict();

        if verdict.realised.is_finite() {
            tb_session.min_realised = tb_session.min_realised.min(verdict.realised);
        }
        if severity_rank(verdict.health) > severity_rank(tb_session.worst_health) {
            tb_session.worst_health = verdict.health;
        }

        // Carry the verdict into the published snapshot.
        //
        // The verdict's `discarded_ms` / `gap_count` are window deltas because
        // that is what a verdict grades; the published fields are SESSION
        // totals, which is what their documentation promises and what
        // `--selftest-timebase` and the browser annunciator read.
        self.stats.tick_rate_hz = finite_f32(verdict.iterations_per_sec);
        self.stats.timebase_drift_us = verdict
            .drift_us
            .clamp(i64::from(i32::MIN), i64::from(i32::MAX))
            as i32;
        self.stats.timebase_discarded_ms = saturating_u32(tb.discarded_us() / 1000);
        self.stats.timebase_gap_count = tb.gap_count();
        self.stats.timebase_max_gap_ms = saturating_u32(tb.max_gap_us() / 1000);
        self.stats.catchup_steps = tb.catchup_steps();
        // Never cleared for the life of the loop, and deliberately read from
        // the clock rather than from a field a reconfigure could reset.
        self.stats.session_invalid = tb.session_invalid();
        self.stats.steps_per_sec = finite_f32(verdict.steps_per_sec);
        self.stats.wake_late_avg_us = verdict.wake_late_avg_us;
        // Window total, like `wake_late_avg_us` beside it and unlike the
        // session latches below. Read the two together: zero wake lateness
        // with a non-zero overrun is compute-bound, the reverse is
        // descheduled.
        self.stats.deadline_overrun_us = verdict.deadline_overrun_us;
        // Session-wide, so it shares a scope with the p99 published beside it.
        // A window max under a session p99 reads as a broken metric, and the
        // pair's whole job is to be believed on its first real incident. The
        // window's own max still reaches the log through the reporter.
        self.stats.wake_late_max_us = tb_session.wake_late_max_us.min(u64::from(u32::MAX)) as u32;
        // Session-wide, not per window: the tail is what
        // `--selftest-timebase` is asked about, and a one-second slice of it
        // is not an answer.
        self.stats.wake_late_p99_us = tb_session
            .wake_late
            .quantile_us(0.99)
            .min(u64::from(u32::MAX)) as u32;
        // The unit the drift beside it is quantised in. Published from the
        // clock itself rather than recomputed from `tick_rate_hz`, so a reader
        // grading drift can never be grading it against a different step from
        // the one the loop actually ran.
        self.stats.timebase_step_us = saturating_u32(tb.step_us());
        self.stats.timebase_health = verdict.health.into();
        self.stats.timebase_worst_health = tb_session.worst_health.into();
        self.stats.timebase_min_realised = finite_f32(tb_session.min_realised);

        if let Some(reporter) = tb_reporter {
            let drops_delta = self
                .stats
                .sensor_drops
                .saturating_sub(tb_window.drops_at_start);
            let channel_full_secs = self.channel_full_since.map(|t| t.elapsed().as_secs());
            reporter.report(&verdict, drops_delta, channel_full_secs);
        }
    }

    /// Process pending actuator commands from the flight controller
    fn process_actuator_commands(&mut self) {
        // Drain all pending messages, use the latest
        let mut latest: Option<ActuatorOutputs> = None;

        while let Ok(actuator) = self.actuator_rx.try_recv() {
            latest = Some(actuator);
            self.stats.actuator_count += 1;
        }

        if let Some(actuator) = latest {
            self.last_actuator_time = Instant::now();
            // Two separate write acquisitions of the state lock, on the branch
            // that runs on essentially every tick with a real FC attached.
            // Uninstrumented, they read as actuator "work" — which is exactly
            // the misattribution `lock_wait_ns` exists to remove, and it would
            // have been invisible in every real HITL session because only the
            // no-command branch below was measured.
            let lock_start = Instant::now();
            self.state.set_motor_commands(actuator.motors);
            self.state.set_armed(actuator.is_armed());
            self.tick_lock_wait_ns += lock_start.elapsed().as_nanos() as u64;

            if actuator.is_hil_active() {
                trace!(
                    motors = ?actuator.motors,
                    armed = actuator.is_armed(),
                    "Received actuator commands"
                );
            }
        } else {
            // No command received this tick — check for staleness while motors are active.
            let lock_start = Instant::now();
            let motors_active = {
                let state = self.state.read();
                self.tick_lock_wait_ns += lock_start.elapsed().as_nanos() as u64;
                state.motor_commands.iter().any(|&c| c > 0.01)
            };
            if motors_active && self.last_actuator_time.elapsed() > ACTUATOR_STALE_TIMEOUT {
                // Zero motors and disarm: FC is gone, let gravity take over.
                let lock_start = Instant::now();
                self.state.set_motor_commands([0.0; 4]);
                self.state.set_armed(false);
                self.tick_lock_wait_ns += lock_start.elapsed().as_nanos() as u64;

                if self.last_stale_warning.elapsed() >= STALE_WARN_INTERVAL {
                    warn!(
                        stale_ms = self.last_actuator_time.elapsed().as_millis(),
                        "Actuator commands stale for >100ms — zeroing motors (FC disconnected?)"
                    );
                    self.last_stale_warning = Instant::now();
                }
            }
        }
    }

    /// True while the ground-impact deceleration impulse is still being
    /// reported, measured in SIM time.
    ///
    /// Read once per emission, by [`SimulationLoop::ground_override`], which
    /// substitutes the impulse for the airborne sample while it is true.
    ///
    /// It is deliberately NOT an input to `Timebase::plan` any more. Clamping
    /// catch-up while the impulse was live left sim time behind wall time for
    /// the duration of the impulse, and PX4 integrates each emitted sample over
    /// the WALL interval between arrivals — so the 100 ms window was delivered
    /// to the EKF over 117.4 ms at the recorded incident cadence. The emitted
    /// accelerations never depended on the sub-step count in the first place:
    /// [`SimulationLoop::ground_override`] substitutes a constant and cannot
    /// see the sub-step mean.
    fn impact_active(&self, sim_time_us: u64) -> bool {
        self.ground_impact_until_us
            .is_some_and(|until| sim_time_us < until)
    }

    /// The ground-contact accelerometer override for this instant, if one
    /// applies.
    ///
    /// This function CANNOT see the airborne mean — that is the point. The
    /// override substitutes the sample wholesale; there is no arithmetic
    /// anywhere that could combine the two, so a blend is unrepresentable
    /// rather than merely avoided. CLAUDE.md ends the three-way branch this
    /// replaces with "Do not merge the branches": a 0.25° residual tilt is a
    /// 0.043 m/s² lateral bias that the EKF integrates into 76 m of drift over
    /// a 77 s flight, and an averaged `[0, 0, −g]` is exactly such a residual.
    ///
    /// Known, named approximation: if contact RELEASES part-way through a
    /// catch-up, the mean this replaces would have included sub-steps that
    /// were still on the ground. That error is bounded by the catch-up budget,
    /// 20 ms.
    fn ground_override(&self, sim_time_us: u64) -> Option<[f64; 3]> {
        if self.impact_active(sim_time_us) {
            return Some(self.ground_impact_accel);
        }
        if self.ground_contact_active {
            debug_assert!(
                self.ground_rest_accel_body.iter().any(|c| *c != 0.0)
                    || self.config.physics.gravity == 0.0,
                "resting accel must be initialised before ground contact"
            );
            return Some(self.ground_rest_accel_body);
        }
        None
    }

    /// Report a Regime C gap.
    ///
    /// At `error!`, and rate-limited at the emit site exactly as
    /// [`TIMEBASE_WARN_INTERVAL`], [`DROP_WARN_INTERVAL`] and
    /// [`STALE_WARN_INTERVAL`] are. "Gaps are rare by construction" is a
    /// property of the HOST, not of this code: a chronic stall — every
    /// iteration past [`GAP_THRESHOLD_US`], which is the sustained thermal /
    /// Selective-Forced-Idle scenario this whole change exists to diagnose —
    /// produces one gap per iteration. Unthrottled that is an allocating,
    /// synchronously-written log line on the 400 Hz sim thread, which makes
    /// the loop later still, and it would evict the health-window and drop
    /// warnings from the TUI's lossy 512-slot `try_send`.
    ///
    /// The FIRST gap of a session always emits — that is the line worth
    /// having — and suppressed gaps are counted and named on the next line
    /// that does emit, so nothing is silently swallowed. `tb.gap_count()` in
    /// the structured fields carries the total regardless.
    ///
    /// `absorbed_us` is `steps × step_us`, computed by the caller from the
    /// plan actually run, never [`GAP_THRESHOLD_US`]. The two coincide only
    /// when the step divides the threshold exactly, which the documented rates
    /// happen to do and other ACCEPTED ones do not: at `--tick-rate 625` the
    /// step is 1600 µs, the budget is 12 whole steps and 19.2 ms is absorbed
    /// against a 20 ms threshold. A line claiming the threshold breaks the
    /// `deficit == absorbed + discarded` identity the operator reads it for.
    fn report_gap(&mut self, plan: &crate::timebase::TickPlan, tb: &Timebase, absorbed_us: u64) {
        let first_of_session = tb.gap_count() <= 1;
        if !first_of_session && self.last_gap_report.elapsed() < TIMEBASE_WARN_INTERVAL {
            self.gaps_suppressed = self.gaps_suppressed.saturating_add(1);
            return;
        }

        error!(
            deficit_us = plan.deficit_us,
            absorbed_us = absorbed_us,
            discarded_us = plan.discarded_us,
            gap_count = tb.gap_count(),
            max_gap_us = tb.max_gap_us(),
            session_discarded_ms = tb.discarded_us() / 1000,
            suppressed_gaps = self.gaps_suppressed,
            sensor_drops = self.stats.sensor_drops,
            channel_full_secs = self.channel_full_since.map(|t| t.elapsed().as_secs()),
            "{}",
            gap_message(plan.deficit_us, absorbed_us, plan.discarded_us)
        );

        self.last_gap_report = Instant::now();
        self.gaps_suppressed = 0;
    }

    /// Step the physics simulation by one fixed sub-step.
    ///
    /// `sim_time_us` is the simulation time at the END of this sub-step. It is
    /// a parameter rather than a field the function increments: the clock
    /// belongs to [`Timebase`] now, and a sub-step needs to know where it sits
    /// only so the ground-impact impulse can expire on sim time.
    ///
    /// Returns the AIRBORNE sample this sub-step produced — thrust and drag
    /// only. Ground-contact substitution happens once per iteration, on the
    /// post-substep state, in [`SimulationLoop::ground_override`].
    fn step_physics(&mut self, dt: f64, sim_time_us: u64) -> SubStepSample {
        let lock_start = Instant::now();
        let mut state = self.state.write();
        self.tick_lock_wait_ns += lock_start.elapsed().as_nanos() as u64;

        // Ground height in NED (Z > ground_z means below ground). `None` means
        // the height is *unknown*: terrain is configured but the drone has left
        // the cached tile block. Unknown must not be conflated with "flat at the
        // origin datum" — doing so teleports a drone flying below the origin
        // straight up to it, a discontinuity the EKF cannot reconcile.
        // Flat ground at Z=0 applies only when no terrain is configured at all.
        // `self.config.terrain` is `Some` from the moment the daemon starts
        // (main.rs always wires in a shared Arc<TerrainCache> so a later
        // ConfigureBuild can populate it in place) — it does NOT mean tiles
        // have actually been loaded. Gate on `is_loaded()` too, so an empty
        // cache is treated the same as "no terrain configured" (flat Z=0)
        // rather than "configured but outside coverage" (clamp disabled).
        // Without this, a fresh daemon free-falls from the origin with no
        // floor until it hits conservation::check_position_bounds's 10km
        // debug-assert, permanently killing the simulation thread.
        let terrain = self.config.terrain.as_ref().filter(|t| t.is_loaded());

        // Everything from here to the end of the ground-normal lookup is the
        // terrain phase, timed and reported apart from the integration it sits
        // inside. It takes one exclusive lock and seven shared ones per tick,
        // and resolves the vehicle's coordinate to a tile six separate times.
        let terrain_start = Instant::now();

        // Tell the cache where the vehicle is. It orders eviction by distance
        // from this point — so the tile underfoot is the last thing dropped —
        // and derives which tiles to ask the browser for.
        if let Some(terrain) = self.config.terrain.as_ref() {
            terrain.set_vehicle_ned(state.quadrotor.position[0], state.quadrotor.position[1]);
        }

        let ground_z: Option<f64> = match terrain {
            Some(terrain) => terrain
                .sample_ground_ned(
                    state.quadrotor.position[0], // north
                    state.quadrotor.position[1], // east
                )
                .map(|z| z as f64),
            None => Some(0.0),
        };

        // The surface normal is NOT sampled here.
        //
        // It is read in exactly one place — the ground-contact branch below —
        // and only when the vehicle is touching the ground, which is most of a
        // flight's duration spent not being true. Sampling it costs five height
        // lookups behind five more lock acquisitions: measured at 617 ns of the
        // terrain phase's 753 ns, so four fifths of that phase was computed and
        // discarded while airborne.
        //
        // It cannot simply be guarded by the contact test up here, because that
        // test uses the position *after* this tick's integration and this point
        // is before it. So it moves to where it is used.

        // Accumulated, not assigned: an iteration may run several sub-steps and
        // the loop zeroes this once per ITERATION, before the sub-step loop.
        // Assigning here reported only the last sub-step's terrain cost while
        // the physics span still covered them all, so a catch-up would
        // under-report terrain and over-report physics — a lie in exactly the
        // per-phase breakdown this change exists to make trustworthy.
        self.tick_terrain_ns += terrain_start.elapsed().as_nanos() as u64;

        if ground_z.is_none()
            && self.last_terrain_miss_warning.elapsed() >= TERRAIN_MISS_WARN_INTERVAL
        {
            // Two distinct faults reach here and they need different names.
            // Reporting a missing datum as "outside coverage" sent a whole
            // debugging session after tile coordinates that were in fact
            // resident and correct.
            let has_datum = self.config.terrain.as_ref().is_some_and(|t| t.has_datum());
            if has_datum {
                warn!(
                    north = state.quadrotor.position[0],
                    east = state.quadrotor.position[1],
                    detail = %self
                        .config
                        .terrain
                        .as_ref()
                        .map(|t| t.describe_lookup(
                            state.quadrotor.position[0],
                            state.quadrotor.position[1]
                        ))
                        .unwrap_or_default(),
                    "Outside terrain coverage — ground collision disabled until the drone returns"
                );
            } else {
                warn!(
                    north = state.quadrotor.position[0],
                    east = state.quadrotor.position[1],
                    "Terrain has no altitude datum — ground collision disabled everywhere, \
                     the vehicle will fall through the terrain. The flight origin was never \
                     anchored into the terrain cache."
                );
            }
            self.last_terrain_miss_warning = Instant::now();
        }

        // Skip physics only when disarmed on the ground (gyro calibration).
        // Unknown ground is never "on the ground".
        let on_ground = ground_z.is_some_and(|g| state.quadrotor.position[2] >= g);
        let motors_active = state.motor_commands.iter().any(|&c| c > 0.01);

        if motors_active || !on_ground {
            // Convert motor commands (0-1) to angular velocities using config-aware max speed.
            // Initial pass uses nominal voltage — needed to estimate current before we know sag.
            let mut motor_omegas: [f64; 4] = std::array::from_fn(|i| {
                throttle_to_omega_with_config(state.motor_commands[i] as f64, &self.config.physics)
            });

            // Discharge battery based on motor current draw
            if motors_active && !state.battery.is_depleted() {
                let current = total_motor_current(&motor_omegas, &self.config.physics);
                state.battery.discharge(current, dt);

                // Scale motor speeds by the voltage-sag ratio so thrust reflects
                // the terminal voltage the ESCs actually see, not nominal.
                // V_terminal = V_OCV(soc) - I·R_internal; ratio < 1 under load or at low SoC.
                let v_nominal = self.config.physics.battery_voltage;
                let v_terminal = state
                    .battery
                    .v_terminal(current, self.config.battery.internal_resistance_ohm);
                let voltage_ratio = if v_nominal > 0.0 {
                    (v_terminal / v_nominal).clamp(0.0, 1.0)
                } else {
                    1.0
                };
                motor_omegas = std::array::from_fn(|i| motor_omegas[i] * voltage_ratio);
            } else if state.battery.is_depleted() {
                motor_omegas = [0.0; 4];
            }

            // Step physics using RK4 integration
            state.quadrotor = rk4_step(&state.quadrotor, &self.config.physics, motor_omegas, dt);

            // Debug-only invariant checks: zero cost in release builds because
            // every assertion inside check_all is gated on `#[cfg(debug_assertions)]`.
            hitl_physics::conservation::check_all(&state.quadrotor, &self.config.physics);
        }

        // Ground contact model with impact deceleration.
        //
        // Three concerns:
        // 1. Physics clamp (z >= 0): hard constraint preventing penetration,
        //    zeroes roll/pitch rates, forces level attitude.
        // 2. Impact deceleration: when the drone hits the ground with
        //    significant velocity, generate an accelerometer impulse over
        //    IMPACT_DURATION_US — 100 ms of SIM time, not a tick count — so the
        //    EKF can reconcile the velocity change.
        //    Without this, the EKF dead-reckons underground.
        // 3. Sensor ground mode (hysteresis): keeps accel at [0,0,-g] until
        //    the quad is clearly airborne (z < -0.10m). This prevents EKF
        //    contamination during the liftoff bounce phase.
        let velocity_before_clamp = [
            state.quadrotor.velocity[0],
            state.quadrotor.velocity[1],
            state.quadrotor.velocity[2],
        ];

        let physics_on_ground = ground_z.is_some_and(|g| state.quadrotor.position[2] >= g);
        if let Some(ground_z) = ground_z.filter(|_| physics_on_ground) {
            state.quadrotor.position[2] = ground_z;

            let gravity = self.config.physics.gravity;
            // Sampled here rather than every tick — see the note above.
            let normal_start = Instant::now();
            let normal = match terrain {
                Some(terrain) => terrain
                    .sample_ground_normal_ned(
                        state.quadrotor.position[0],
                        state.quadrotor.position[1],
                    )
                    .unwrap_or(LEVEL_NORMAL_NED),
                None => LEVEL_NORMAL_NED,
            };
            self.tick_terrain_ns += normal_start.elapsed().as_nanos() as u64;
            // Decided up-front because the impact impulse must agree with it:
            // the accelerometer may not claim the drone stopped horizontally if
            // the contact model is about to let it slide.
            let will_slide = {
                let a = downslope_accel(normal, gravity);
                let pull = (a[0] * a[0] + a[1] * a[1]).sqrt();
                pull > GROUND_FRICTION_COEFF * gravity * (-normal[2] as f64).max(0.0)
            };

            // Trigger impact deceleration if hitting ground with significant downward velocity.
            // Deceleration is spread over IMPACT_DURATION_US — 100 ms of sim
            // time, never a count of iterations — to give the EKF
            // a physical impulse it can fuse (instead of an instantaneous velocity
            // discontinuity with no corresponding acceleration).
            if velocity_before_clamp[2] > IMPACT_VEL_THRESHOLD && !self.impact_active(sim_time_us) {
                let impact_dt = IMPACT_DURATION_US as f64 / 1_000_000.0;
                // Deceleration needed to bring velocity to zero over impact_dt,
                // in NED. vz>0 means downward, so this is negative-Z (upward).
                let v = nalgebra::Vector3::new(
                    velocity_before_clamp[0],
                    velocity_before_clamp[1],
                    velocity_before_clamp[2],
                );
                // What the impact actually removes. On ground the drone will
                // hold onto, that is the whole velocity — static damping bleeds
                // the horizontal part away inside the impulse window anyway. On
                // ground it will slide down, only the component *into* the
                // surface is arrested; the tangential part keeps going, so
                // reporting it as stopped would desync the accelerometer from
                // the simulated truth by the full sliding speed.
                let arrested = if will_slide {
                    let n = nalgebra::Vector3::new(
                        normal[0] as f64,
                        normal[1] as f64,
                        normal[2] as f64,
                    );
                    n * v.dot(&n)
                } else {
                    v
                };
                let decel_ned = -arrested / impact_dt;
                // The accelerometer measures specific force = a - g, and it does
                // so in the BODY frame. `decel_ned` is in NED, so it must be
                // rotated by the impact attitude before being reported — a yawed
                // drone otherwise reports its lateral impact on the wrong body
                // axis and the EKF integrates a phantom sideways acceleration.
                // Rotating the whole (a - g) vector reduces to the old
                // [decel_x, decel_y, decel_z - g] when the drone is level.
                let gravity = self.config.physics.gravity;
                let specific_force_ned = decel_ned - nalgebra::Vector3::new(0.0, 0.0, gravity);
                let specific_force_body = state.quadrotor.quaternion.inverse() * specific_force_ned;
                self.ground_impact_accel = [
                    specific_force_body.x,
                    specific_force_body.y,
                    specific_force_body.z,
                ];
                // A deadline in sim time. A catch-up iteration advances the
                // clock by several steps at once, so a countdown of iterations
                // would compress this window by exactly the factor the loop is
                // behind — and this window is the only thing that tells the
                // EKF what removed the vehicle's velocity.
                self.ground_impact_until_us = Some(sim_time_us.saturating_add(IMPACT_DURATION_US));
                debug!(
                    vz = velocity_before_clamp[2],
                    impact_ms = IMPACT_DURATION_US / 1000,
                    until_us = sim_time_us.saturating_add(IMPACT_DURATION_US),
                    decel_z = decel_ned.z,
                    "Ground impact: generating deceleration impulse"
                );
            }

            if state.quadrotor.velocity[2] > 0.0 {
                state.quadrotor.velocity[2] = 0.0;
            }

            state.quadrotor.angular_velocity[0] = 0.0;
            state.quadrotor.angular_velocity[1] = 0.0;
            state.quadrotor.angular_velocity[2] *= 0.95;

            // Rest on the surface, not on an imaginary level plane. Forcing
            // (0, 0, yaw) made every landing look identical regardless of the
            // ground under it: a drone could not tip, could not slide, and sat
            // perfectly flat on a 30-degree hillside.
            let (_, _, yaw) = state.quadrotor.quaternion.euler_angles();
            state.quadrotor.quaternion = attitude_on_surface(normal, yaw);

            let slope_accel = downslope_accel(normal, gravity);
            // Coulomb-style threshold: the drone only creeps once the pull
            // along the surface beats static friction. Below that it stays put,
            // which is what the old blanket 0.9-per-tick damping achieved for
            // flat ground and must keep achieving.
            if will_slide {
                state.quadrotor.velocity[0] += slope_accel[0] * dt;
                state.quadrotor.velocity[1] += slope_accel[1] * dt;
                state.quadrotor.velocity[0] *= GROUND_SLIDING_DAMPING;
                state.quadrotor.velocity[1] *= GROUND_SLIDING_DAMPING;
            } else {
                state.quadrotor.velocity[0] *= GROUND_STATIC_DAMPING;
                state.quadrotor.velocity[1] *= GROUND_STATIC_DAMPING;
            }

            // Specific force a resting accelerometer reads. On level ground
            // this is the exact [0, 0, -g] the EKF depends on; on a slope it is
            // that vector in the tilted airframe's own frame, which is a real
            // physical reading rather than the integration artifact the
            // flat-ground path exists to suppress.
            self.ground_rest_accel_body = if is_level(normal) {
                [0.0, 0.0, -gravity]
            } else {
                let g_ned = nalgebra::Vector3::new(0.0, 0.0, gravity);
                let sf = state.quadrotor.quaternion.inverse() * (-g_ned);
                [sf.x, sf.y, sf.z]
            };
        }

        // No impact counter to decrement: `ground_impact_until_us` is a sim-time
        // deadline and expires on its own.

        // Sensor ground mode with hysteresis: latches ON when physics clamp
        // fires, only releases when clearly airborne. This ensures the EKF
        // gets clean [0,0,-g] accel through the entire liftoff transition.
        if physics_on_ground {
            self.ground_contact_active = true;
        } else {
            let airborne = match ground_z {
                Some(g) => state.quadrotor.position[2] < g - 0.10,
                // Outside terrain coverage there is no surface to be in contact
                // with, so the drone is by definition airborne.
                None => true,
            };
            if airborne {
                self.ground_contact_active = false;
            }
        }

        // The simulation clock is NOT advanced here. It belongs to `Timebase`,
        // which counts wall time and is mirrored into `state.sim_time_us` once
        // per iteration. Incrementing it per step was the defect: a loop
        // running 341 iterations of a 400 Hz clock advances its clock
        // 341 × 2.5 ms = 852.5 ms per second of wall, i.e. 148.3 ms/s of skew.
        // The 2026-09-05 session held that rate for roughly 33 s of its 63 s,
        // which is the 4.9 s of skew that left 58.1 s of simulation reported
        // against 63 s of wall time — while PX4, which restamps every HIL
        // sample with its own `hrt_absolute_time()`, integrated the emitted
        // rates over the real 63 s.
        SubStepSample {
            accel_body: airborne_specific_force_body(&state.quadrotor, &self.config.physics),
            gyro_body: [
                state.quadrotor.angular_velocity[0],
                state.quadrotor.angular_velocity[1],
                state.quadrotor.angular_velocity[2],
            ],
        }
    }

    /// Sample sensors and send HIL messages for one loop iteration.
    ///
    /// Exactly one `HIL_SENSOR` leaves per iteration whatever the sub-step
    /// count, carrying the MEAN airborne rate over the interval the iteration
    /// simulated. PX4 on real hardware ignores `HIL_SENSOR.time_usec` —
    /// `MavlinkReceiver::handle_message_hil_sensor` restamps every sample with
    /// its own `hrt_absolute_time()` — and integrates each emitted rate over
    /// the WALL interval between arrivals, so the mean over that same interval
    /// is the only value whose integral equals the motion actually simulated.
    ///
    /// KNOWN, BOUNDED TORN-READ WINDOW. `accel_body` / `gyro_body` arrive in
    /// `emission`, read under `step_physics`'s write lock; altitude, position,
    /// velocity and attitude are read from a SECOND, later acquisition below.
    /// Before sub-stepping all six came from one `state.read()`. `reset()` (the
    /// WebSocket RTL path) and `recharge_battery()` run on tokio tasks and can
    /// land between the two windows, so one `HIL_SENSOR`/`HIL_GPS` pair can
    /// carry pre-reset rates against post-reset pose. It is accepted rather
    /// than fixed here: it is one sample, only on a path that is resetting the
    /// EKF anyway, and closing it means threading the pose snapshot out of the
    /// last sub-step. Do not widen it — nothing else may move between the
    /// physics write lock and this read.
    fn sample_and_send_sensors(&mut self, emission: IterationEmission) {
        // Compute sensor inputs from physics state
        // One read per tick: the barometer and GPS derive altitude from the
        // same datum, so they must not straddle a change to it.
        let origin = self.origin.get();

        // The ground-contact override is decided ONCE, on the post-substep
        // state, and it SUBSTITUTES the sample. It never averages with the
        // airborne mean, and `ground_override` cannot even see that mean.
        let accel_body = match self.ground_override(emission.sim_time_us) {
            Some(substitute) => substitute,
            None => emission.mean_accel_body,
        };
        let gyro_body = emission.mean_gyro_body;

        let (altitude_m, position_ned, velocity_ned, attitude) = {
            let lock_start = Instant::now();
            let state = self.state.read();
            self.tick_lock_wait_ns += lock_start.elapsed().as_nanos() as u64;

            let q = &state.quadrotor;

            // Altitude is negative of NED down position, plus the shared
            // datum. Ground contact, this barometer and HIL_GPS all read the
            // same `alt_datum`; when they did not, the EKF absorbed the
            // difference as a standing altitude error.
            (
                origin.alt_datum - q.position[2],
                [q.position[0], q.position[1], q.position[2]],
                [q.velocity[0], q.velocity[1], q.velocity[2]],
                q.quaternion,
            )
        };

        // Mag and baro run on a sim-time schedule, not a tick divider: their
        // timestamps come from the simulation clock, so a divider would
        // decouple cadence from stamp the moment an iteration carries more
        // than one step.
        let update_mag = emission.sim_time_us >= self.next_mag_us;
        let update_baro = emission.sim_time_us >= self.next_baro_us;

        // Sample sensors selectively to avoid jittery data on non-update ticks.
        // IMU always sampled at full rate; mag/baro only on their update ticks.
        //
        // The GPS rate limiter reads `sim_time_us` — the wall-anchored clock —
        // so GPS holds a true 10 Hz of WALL time. It also never rewinds, which
        // is load-bearing: `hitl-sensors` holds `last_update_time` and
        // `last_output_time` in this timebase and a backward jump would
        // silence GPS for the session.
        let time_s = emission.sim_time_us as f64 / 1_000_000.0;
        let (imu_reading, mag_reading, baro_reading, gps_reading) = {
            let lock_start = Instant::now();
            let mut state = self.state.write();
            self.tick_lock_wait_ns += lock_start.elapsed().as_nanos() as u64;

            // Mirror the timebase into the shared state, once per iteration.
            // The browser's `StateUpdate.timestamp_us` and the TUI stopwatch
            // read it from here.
            state.sim_time_us = emission.sim_time_us;

            // The IMU is handed the interval this emission covers, which is
            // `steps × dt`. That is correct rather than a hazard: the reading
            // IS the mean of n sub-steps, and averaging n independent samples
            // reduces sigma by exactly the √n that `density / sqrt(dt)`
            // computes.
            let imu = state
                .sensors
                .imu
                .sample(&accel_body, &gyro_body, emission.interval_dt);

            // Mag: only sample on update ticks, otherwise use cached value
            let mag = if update_mag {
                state.sensors.mag.sample(&attitude)
            } else {
                self.last_mag
                    .unwrap_or_else(|| state.sensors.mag.sample(&attitude))
            };

            // Baro: only sample on update ticks, otherwise use cached value
            let baro = if update_baro {
                state.sensors.baro.sample(altitude_m)
            } else {
                self.last_baro
                    .unwrap_or_else(|| state.sensors.baro.sample(altitude_m))
            };

            // GPS has internal rate limiting (returns None when not time to update)
            let gps = state.sensors.gps.sample(
                &position_ned,
                &velocity_ned,
                origin.lat,
                origin.lon,
                time_s,
            );

            (imu, mag, baro, gps)
        };

        // Cache the readings and advance their schedules.
        if update_mag {
            self.last_mag = Some(mag_reading);
            self.next_mag_us = next_slot_us(emission.sim_time_us, SENSOR_50HZ_PERIOD_US);
        }
        if update_baro {
            self.last_baro = Some(baro_reading);
            self.next_baro_us = next_slot_us(emission.sim_time_us, SENSOR_50HZ_PERIOD_US);
        }

        // Compute fields_updated bitmask — only flag sensors that have new data this tick.
        // IMU (accel + gyro) updates every tick at 400 Hz.
        // Mag and baro update at ~50 Hz to match PX4's expected sensor rates.
        // On first tick, always include all flags so PX4 sees all sensors immediately.
        let first_tick = self.total_ticks == 0;
        let mut fields_updated = IMU_FLAGS;
        if update_mag || first_tick {
            fields_updated = fields_updated.union(MAG_FLAGS);
        }
        if update_baro || first_tick {
            fields_updated = fields_updated.union(BARO_FLAGS);
        }

        // Log sensor values on first tick for debugging
        if first_tick {
            info!(
                accel = ?[imu_reading.accel[0], imu_reading.accel[1], imu_reading.accel[2]],
                gyro = ?[imu_reading.gyro[0], imu_reading.gyro[1], imu_reading.gyro[2]],
                mag = ?mag_reading.field,
                baro_pa = baro_reading.pressure_pa,
                baro_alt = baro_reading.altitude_m,
                "First tick sensor values"
            );
        }

        // Build and send HIL_SENSOR message, stamped with the wall-anchored
        // clock.
        let hil_sensor = self.build_hil_sensor(
            &imu_reading,
            &baro_reading,
            &mag_reading,
            emission.sim_time_us,
            fields_updated,
        );
        match self.mav_tx.try_send(MavMessage::HIL_SENSOR(hil_sensor)) {
            Ok(()) => {
                self.stats.hil_sensor_count += 1;
                // Channel drained — reset the contiguous-full timer.
                self.channel_full_since = None;
            }
            Err(crossbeam_channel::TrySendError::Full(_)) => {
                self.stats.sensor_drops += 1;
                self.drop_count_since_last_warning += 1;

                // Track how long the channel has been continuously full.
                let full_since = self.channel_full_since.get_or_insert_with(Instant::now);

                // Escalate to error if full for more than DROP_ERROR_THRESHOLD —
                // this typically means the FC is disconnected but the serial
                // layer hasn't timed out yet.
                if full_since.elapsed() >= DROP_ERROR_THRESHOLD {
                    error!(
                        drops_total = self.stats.sensor_drops,
                        full_secs = full_since.elapsed().as_secs(),
                        "HIL sensor channel full for >{}s — FC likely disconnected (serial has not timed out)",
                        DROP_ERROR_THRESHOLD.as_secs(),
                    );
                    // Reset full_since so we error at most once per threshold period.
                    self.channel_full_since = Some(Instant::now());
                } else if self.last_drop_warning.elapsed() >= DROP_WARN_INTERVAL {
                    // Rate-limited warning: at most once every DROP_WARN_INTERVAL.
                    warn!(
                        drops = self.drop_count_since_last_warning,
                        "HIL sensor channel full — {} messages dropped in last {}s (FC may not be consuming fast enough)",
                        self.drop_count_since_last_warning,
                        DROP_WARN_INTERVAL.as_secs(),
                    );
                    self.last_drop_warning = Instant::now();
                    self.drop_count_since_last_warning = 0;
                }
            }
            Err(crossbeam_channel::TrySendError::Disconnected(_)) => {}
        }

        // Send HIL_GPS when sensor provides it (sensor handles rate limiting).
        //
        // Stamped with `integrated_us = sim_us − discarded_us` — the time
        // actually simulated — NOT with the wall-anchored clock HIL_SENSOR
        // carries. The divergence is deliberate: PX4 uses neither for timing
        // (`sensor_gps.time_utc_usec` reaches the EKF only through the PPS
        // path, gated off with no PPS hardware), so the pair costs nothing and
        // buys an independent measurement. The ULog cross-check
        // `(hrt elapsed) − (time_utc_usec elapsed)` then reads out exactly the
        // wall time this daemon admits it could not simulate: zero on a
        // healthy run, the fault magnitude on a bad one. That is what keeps
        // the detector which produced this diagnosis alive after the fix.
        if let Some(gps) = gps_reading {
            let hil_gps = self.build_hil_gps(&gps, emission.integrated_time_us);
            match self.mav_tx.try_send(MavMessage::HIL_GPS(hil_gps)) {
                Ok(()) => {
                    self.stats.hil_gps_count += 1;
                }
                Err(crossbeam_channel::TrySendError::Full(_)) => {
                    self.stats.sensor_drops += 1;
                    self.drop_count_since_last_warning += 1;
                }
                Err(crossbeam_channel::TrySendError::Disconnected(_)) => {}
            }
        }
    }

    /// Build HIL_SENSOR MAVLink message
    fn build_hil_sensor(
        &self,
        imu: &hitl_sensors::ImuReading,
        baro: &hitl_sensors::BaroReading,
        mag: &hitl_sensors::MagReading,
        time_us: u64,
        fields_updated: HilSensorUpdatedFlags,
    ) -> HIL_SENSOR_DATA {
        HIL_SENSOR_DATA {
            time_usec: time_us,
            xacc: imu.accel[0] as f32,
            yacc: imu.accel[1] as f32,
            zacc: imu.accel[2] as f32,
            xgyro: imu.gyro[0] as f32,
            ygyro: imu.gyro[1] as f32,
            zgyro: imu.gyro[2] as f32,
            xmag: mag.field[0] as f32,
            ymag: mag.field[1] as f32,
            zmag: mag.field[2] as f32,
            abs_pressure: baro.pressure_pa as f32 / 100.0, // Convert to hPa (mbar)
            diff_pressure: 0.0,                            // No airspeed sensor
            pressure_alt: baro.altitude_m as f32,
            temperature: baro.temperature_c as f32,
            fields_updated,
        }
    }

    /// Build HIL_GPS MAVLink message
    fn build_hil_gps(&self, gps: &hitl_sensors::GpsReading, time_us: u64) -> HIL_GPS_DATA {
        // Compute ground speed and course over ground from velocity components
        let ground_speed = ((gps.vel_n * gps.vel_n + gps.vel_e * gps.vel_e) as f64).sqrt();
        let cog = if ground_speed > 0.1 {
            (gps.vel_e as f64).atan2(gps.vel_n as f64).to_degrees()
        } else {
            0.0
        };
        let cog_positive = if cog < 0.0 { cog + 360.0 } else { cog };

        // gps.alt is AGL (height above launch point = -ned_down, no datum).
        // HIL_GPS requires MSL in millimetres, so the shared datum is added
        // here. This is NOT double-counting: the GPS sensor deliberately omits
        // it so the sensor library stays free of daemon-specific config.
        let alt_msl = gps.alt as f64 + self.origin.get().alt_datum;

        HIL_GPS_DATA {
            time_usec: time_us,
            lat: (gps.lat * 1e7) as i32,
            lon: (gps.lon * 1e7) as i32,
            alt: (alt_msl * 1000.0) as i32,     // mm MSL
            eph: (gps.hdop * 100.0) as u16,     // cm (using HDOP as horizontal accuracy proxy)
            epv: 200,                           // cm (fixed vertical accuracy estimate)
            vel: (ground_speed * 100.0) as u16, // cm/s
            vn: (gps.vel_n * 100.0) as i16,     // cm/s
            ve: (gps.vel_e * 100.0) as i16,     // cm/s
            vd: (gps.vel_d * 100.0) as i16,     // cm/s
            cog: (cog_positive * 100.0) as u16, // cdeg
            fix_type: 3,                        // 3D fix
            satellites_visible: gps.satellites,
        }
    }

    /// Re-read this thread's scheduling contract and report a withdrawal.
    ///
    /// Called from the 5 s stats window, so the cost is one syscall per five
    /// seconds on the sim thread. Nothing here can fail the loop: a read that
    /// does not succeed reports `NotApplicable`, which is not a revocation.
    fn check_realtime_contract(&mut self) {
        let (policy, newly_revoked) = crate::realtime::fold_realtime_verdict(
            &self.realtime_policy,
            rtsched::verify_current_thread(),
            self.realtime_revoked,
        );
        if newly_revoked {
            self.realtime_revoked = true;
            warn!(
                granted = %self.realtime_policy,
                "this thread's real-time scheduling contract is no longer the one it was \
                 granted. Two things produce this verdict and they are different: the kernel \
                 DEMOTED the thread to the default policy (chronic overrun of the declared \
                 computation budget), or the contract in force no longer matches the period and \
                 constraint promote() declared — a REPLACEMENT, which is not a demotion and not \
                 an overrun. If it is a demotion, note that it does not single out slow work: a \
                 catch-up iteration runs up to MAX_CATCHUP_STEPS physics sub-steps, so sustained \
                 LATE WAKE-UPS overrun the declaration just as surely, and this release exists \
                 for exactly that scenario. Read deadline_overrun_us, catchup_steps and \
                 wake_late_p99_us together to tell them apart, and run --selftest-timebase on \
                 this host. Reported once per session; it is not re-granted."
            );
        }
        self.realtime_policy = policy;
    }

    /// Snapshot the loop's current state + windowed stats and push it onto
    /// `stats_tx`. Cheap to skip when no subscriber is attached.
    fn publish_stats(&mut self) {
        let Some(tx) = self.stats_tx.as_ref() else {
            return;
        };

        let state = self.state.read();
        let physics = &self.config.physics;

        // Motor RPM = ω · 60 / (2π). We surface the *actual* simulated rotor
        // speed (which trails the command through tau_motor), not the
        // commanded one — that's what the user sees in the 3D viewer and
        // what matters for diagnosing trembling.
        let rpm_scale = 60.0 / (2.0 * std::f64::consts::PI);
        let omegas = state.quadrotor.motor_speeds;
        let motor_rpms = [
            (omegas[0] * rpm_scale) as f32,
            (omegas[1] * rpm_scale) as f32,
            (omegas[2] * rpm_scale) as f32,
            (omegas[3] * rpm_scale) as f32,
        ];

        // TWR snapshot — derived per-publish so it stays consistent with the
        // currently-applied physics config without touching the reconfigure
        // channel signature.
        let max_omega = physics.max_motor_speed_from_voltage();
        let max_thrust_n = 4.0 * physics.kt * max_omega * max_omega;
        let weight_n = physics.mass_kg * physics.gravity;
        let twr = if weight_n > 0.0 {
            (max_thrust_n / weight_n) as f32
        } else {
            0.0
        };

        // Roll/pitch/yaw of the sim quaternion in degrees. The TUI lights up
        // the attitude row red when |roll|+|pitch| is large while disarmed —
        // that's the inverted-on-ground state we want to catch *before* the
        // user arms and sees motor thrash.
        let (roll, pitch, yaw) = state.quadrotor.quaternion.euler_angles();
        let attitude_rpy_deg = [
            roll.to_degrees() as f32,
            pitch.to_degrees() as f32,
            yaw.to_degrees() as f32,
        ];

        let snapshot = SimulationStats {
            // From the 1 s health window, so the rate agrees with the verdict
            // and the colour beside it.
            tick_rate_hz: self.stats.tick_rate_hz,
            // Latency — from the 5 s roll-up.
            avg_latency_us: self.stats.avg_latency_us,
            max_latency_us: self.stats.max_latency_us,
            // Cumulative counts since last reconfigure.
            hil_sensor_count: self.stats.hil_sensor_count,
            hil_gps_count: self.stats.hil_gps_count,
            actuator_count: self.stats.actuator_count,
            sensor_drops: self.stats.sensor_drops,
            // Live values.
            sim_time_s: (state.sim_time_us as f64 / 1_000_000.0) as f32,
            position_ned: [
                state.quadrotor.position[0] as f32,
                state.quadrotor.position[1] as f32,
                state.quadrotor.position[2] as f32,
            ],
            attitude_rpy_deg,
            armed: state.armed,
            flight_mode: state.flight_mode,
            motor_rpms,
            battery_voltage: state.battery.voltage() as f32,
            battery_percent: f32::from(state.battery.percent()),
            build_configured: self.build_configured,
            mass_kg: physics.mass_kg as f32,
            thrust_to_weight: twr,
            // Timebase — carried verbatim from the last 1 s health window.
            timebase_drift_us: self.stats.timebase_drift_us,
            timebase_discarded_ms: self.stats.timebase_discarded_ms,
            timebase_gap_count: self.stats.timebase_gap_count,
            timebase_max_gap_ms: self.stats.timebase_max_gap_ms,
            catchup_steps: self.stats.catchup_steps,
            steps_per_sec: self.stats.steps_per_sec,
            wake_late_avg_us: self.stats.wake_late_avg_us,
            wake_late_max_us: self.stats.wake_late_max_us,
            wake_late_p99_us: self.stats.wake_late_p99_us,
            deadline_overrun_us: self.stats.deadline_overrun_us,
            timebase_step_us: self.stats.timebase_step_us,
            session_invalid: self.stats.session_invalid,
            timebase_health: self.stats.timebase_health,
            timebase_worst_health: self.stats.timebase_worst_health,
            timebase_min_realised: self.stats.timebase_min_realised,
            realtime_policy: self.realtime_policy.clone(),
        };

        // send_replace silently drops the previous value — no subscriber lag
        // and the TUI always sees the latest snapshot.
        let _ = tx.send(snapshot);
    }

    /// Get current statistics
    pub fn stats(&self) -> SimulationStats {
        self.stats.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // The 400 Hz catch-up figure. Production derives its cap from the step
    // size (`Timebase::budget_steps_at`); these tests pin the constant itself.
    use crate::timebase::MAX_CATCHUP_STEPS;
    use nalgebra::{UnitQuaternion, Vector3};
    use std::sync::Arc;
    use terrain::{TerrainCache, TileCoord, TILE_SIZE};

    const DT: f64 = 1.0 / 400.0;
    /// The same step in the timebase's own units.
    const STEP_US: u64 = 2500;
    const TEST_LAT: f64 = 40.015;
    const TEST_LON: f64 = -105.2705;
    const TEST_ZOOM: u32 = 14;

    /// Build a loop with idle channels — enough to drive `step_physics`
    /// directly without a flight controller or a running `run()` loop. The
    /// senders are kept alive for the test's lifetime so the receivers never
    /// report `Disconnected`.
    fn test_loop(config: SimulationConfig) -> (SimulationLoop, TestChannels) {
        let (actuator_tx, actuator_rx) = crossbeam_channel::unbounded();
        let (config_tx, config_rx) = crossbeam_channel::unbounded();
        let (mav_tx, mav_rx) = crossbeam_channel::unbounded();
        let origin = Arc::new(crate::origin::SharedOrigin::new(
            config.reference_lat,
            config.reference_lon,
            config.reference_alt,
        ));
        let sim = SimulationLoop::new(config, actuator_rx, config_rx, mav_tx, origin);
        (
            sim,
            TestChannels {
                _actuator_tx: actuator_tx,
                _config_tx: config_tx,
                mav_rx,
            },
        )
    }

    #[allow(dead_code)]
    struct TestChannels {
        _actuator_tx: Sender<ActuatorOutputs>,
        _config_tx: Sender<(PhysicsConfig, BatteryConfig, hitl_sensors::SensorsConfig)>,
        mav_rx: Receiver<MavMessage>,
    }

    /// The measured 2026-09-05 overshoot: 2935 µs of wall per iteration
    /// against a 2500 µs period.
    const INCIDENT_OVERSHOOT_US: u64 = 435;

    #[test]
    fn a_flapping_timebase_does_not_flood_a_lossy_log_channel() {
        let t0 = Instant::now();
        let mut reporter = TimebaseReporter::new(t0);

        assert_eq!(
            reporter.decide(Health::Degraded, t0 + Duration::from_secs(1)),
            TimebaseAnnouncement::Degraded,
            "the first degradation is announced at once"
        );

        // A host hovering on the threshold flips Ok/Degraded every window.
        // TUI log delivery is a lossy 512-slot `try_send`, so a line per
        // second here decides which diagnostic the operator never sees.
        for second in 2..5 {
            let health = if second % 2 == 0 {
                Health::Ok
            } else {
                Health::Degraded
            };
            assert_eq!(
                reporter.decide(health, t0 + Duration::from_secs(second)),
                TimebaseAnnouncement::Silent,
                "second {second} of a flap must stay quiet"
            );
        }

        assert_eq!(
            reporter.decide(Health::Faulted, t0 + Duration::from_secs(5)),
            TimebaseAnnouncement::Faulted,
            "but an escalation is never delayed by the rate limit — that is the \
             one direction where a late line is worse than a noisy one"
        );
    }

    #[test]
    fn a_healthy_loop_says_so_once_a_minute() {
        let t0 = Instant::now();
        let mut reporter = TimebaseReporter::new(t0);

        assert_eq!(
            reporter.decide(Health::Ok, t0 + Duration::from_secs(30)),
            TimebaseAnnouncement::Silent
        );
        assert_eq!(
            reporter.decide(Health::Ok, t0 + Duration::from_secs(60)),
            TimebaseAnnouncement::Heartbeat,
            "a clean log has to PROVE the metric is alive; ambiguous silence is \
             how the 2026-09-05 session looked healthy"
        );
        assert_eq!(
            reporter.decide(Health::Ok, t0 + Duration::from_secs(61)),
            TimebaseAnnouncement::Silent
        );
    }

    #[test]
    fn a_recovery_names_what_it_recovered_from() {
        let t0 = Instant::now();
        let mut reporter = TimebaseReporter::new(t0);

        assert_eq!(
            reporter.decide(Health::Faulted, t0),
            TimebaseAnnouncement::Faulted
        );
        assert_eq!(
            reporter.decide(Health::Ok, t0 + Duration::from_secs(6)),
            TimebaseAnnouncement::Recovered {
                from: Health::Faulted
            },
            "\"recovered\" without saying from what is indistinguishable from a \
             routine notice"
        );
    }

    #[test]
    fn wake_lateness_quantile_reads_the_tail_not_the_mean() {
        let mut hist = WakeLateHistogram::new();
        // Ninety-nine healthy wake-ups and one at the incident's overshoot.
        for _ in 0..99 {
            hist.record(10);
        }
        hist.record(INCIDENT_OVERSHOOT_US);

        assert_eq!(
            hist.quantile_us(0.99),
            WAKE_LATE_BUCKET_US,
            "one late wake-up in a hundred does not move p99"
        );
        assert_eq!(
            hist.quantile_us(1.0),
            INCIDENT_OVERSHOOT_US,
            "but the tail is exactly what the post-mortem has to name — a mean \
             would report 14 µs and say nothing. The answer is the value that \
             was observed, not its bucket's 500 µs upper edge: a quantile \
             reported above the max printed beside it reads as a broken metric"
        );
        assert!(
            hist.quantile_us(1.0) <= INCIDENT_OVERSHOOT_US,
            "no quantile may exceed the largest value actually recorded"
        );

        // An empty histogram is the shape the post-mortem sees when the loop
        // never ran; it must answer, not divide by zero.
        assert_eq!(WakeLateHistogram::new().quantile_us(0.99), 0);
    }

    /// 3x3 tiles of flat terrain at `elevation_msl` around the reference point.
    fn flat_terrain(elevation_msl: f32) -> Arc<TerrainCache> {
        let center = TileCoord::from_lon_lat(TEST_LON, TEST_LAT, TEST_ZOOM);
        let mut tiles = Vec::new();
        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                let coord = TileCoord {
                    x: (center.x as i32 + dx) as u32,
                    y: (center.y as i32 + dy) as u32,
                    z: TEST_ZOOM,
                };
                tiles.push((coord, vec![elevation_msl; TILE_SIZE * TILE_SIZE], false));
            }
        }

        let cache = TerrainCache::new();
        cache.set_origin(TEST_LAT, TEST_LON, Some(elevation_msl as f64));
        let report = cache.insert_tiles(TILE_SIZE as u32, tiles);
        assert_eq!(report.accepted, 9, "rejected: {:?}", report.rejected);
        Arc::new(cache)
    }

    /// The accelerometer reports specific force in the **body** frame, but the
    /// impact deceleration is derived from NED velocity. With any non-zero yaw
    /// the two disagree, so the lateral impulse must be rotated into the body
    /// frame before it reaches the IMU — otherwise a northward impact on an
    /// east-facing drone lands on the wrong axis and the EKF integrates a
    /// phantom sideways acceleration.
    #[test]
    fn impact_impulse_is_reported_in_the_body_frame() {
        let (mut sim, _ch) = test_loop(SimulationConfig::default());

        // Nose east (yaw +90 deg), travelling north and descending fast.
        let yaw = std::f64::consts::FRAC_PI_2;
        let velocity_ned = [3.0, 0.0, 2.0];
        {
            let mut state = sim.state.write();
            state.quadrotor.quaternion = UnitQuaternion::from_euler_angles(0.0, 0.0, yaw);
            state.quadrotor.position = [0.0, 0.0, -0.001];
            state.quadrotor.velocity = velocity_ned;
        }

        sim.step_physics(DT, STEP_US);

        assert!(
            sim.impact_active(STEP_US),
            "a 2 m/s descent onto the ground must trigger the impact impulse"
        );

        // Expected: NED velocity rotated into the body frame, then divided by
        // the impulse window. Yaw +90 deg puts 3 m/s north onto body -Y.
        let impact_dt = IMPACT_DURATION_US as f64 / 1_000_000.0;
        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, yaw);
        let v_body = q.inverse() * Vector3::new(velocity_ned[0], velocity_ned[1], 0.0);
        let expected_x = -v_body.x / impact_dt;
        let expected_y = -v_body.y / impact_dt;

        let got = sim.ground_impact_accel;
        assert!(
            (got[0] - expected_x).abs() < 1.0,
            "body-X impact accel: expected ~{expected_x:.2}, got {:.2}",
            got[0]
        );
        assert!(
            (got[1] - expected_y).abs() < 1.0,
            "body-Y impact accel: expected ~{expected_y:.2}, got {:.2}",
            got[1]
        );
    }

    /// When terrain is loaded but the drone has flown outside the cached tile
    /// block, the ground height is *unknown*. Treating unknown as "flat at the
    /// origin datum" teleports a drone that is below the origin straight up to
    /// it, zeroing velocity and levelling attitude — a discontinuity the EKF
    /// cannot reconcile. Unknown ground must mean "do not clamp".
    #[test]
    fn missing_terrain_sample_does_not_teleport_the_drone() {
        let config = SimulationConfig {
            terrain: Some(flat_terrain(1655.0)),
            ..SimulationConfig::default()
        };
        let (mut sim, _ch) = test_loop(config);

        // A zoom-14 tile is ~1.9 km tall at this latitude, so the 3x3 block
        // reaches at most ~2.8 km from the origin: 4 km north is outside
        // coverage while staying inside the simulation's 10 km position bound.
        // 50 m down puts the drone *below* the origin ground datum — a valley
        // the loaded tiles do not cover.
        {
            let mut state = sim.state.write();
            state.quadrotor.position = [4_000.0, 0.0, 50.0];
            state.quadrotor.velocity = [0.0, 0.0, 0.0];
        }
        assert!(
            sim.config
                .terrain
                .as_ref()
                .unwrap()
                .sample_ground_ned(4_000.0, 0.0)
                .is_none(),
            "fixture precondition: 4 km north must be outside the tile block"
        );

        sim.step_physics(DT, STEP_US);

        let down = sim.state.read().quadrotor.position[2];
        assert!(
            down > 49.0,
            "drone must not be clamped to the origin datum when the ground \
             height is unknown; expected to stay near 50 m down, got {down}"
        );
    }

    /// Root-cause regression: `main.rs` always wires in `Some(Arc<TerrainCache>)`
    /// from daemon startup (so a later ConfigureBuild can populate it in
    /// place), even before any tiles are loaded. An unloaded cache must be
    /// treated as "no terrain configured" (flat ground clamp at Z=0), not as
    /// "configured but outside coverage" (clamp disabled) — the latter free-
    /// falls the drone from the origin with no floor until it trips
    /// conservation::check_position_bounds's 10km debug-assert, permanently
    /// killing the simulation thread (observed on real hardware: ~5 minutes
    /// after every daemon start, before any ConfigureBuild had run).
    #[test]
    fn unloaded_terrain_cache_clamps_to_flat_ground_instead_of_free_falling() {
        let config = SimulationConfig {
            terrain: Some(Arc::new(TerrainCache::new())),
            ..SimulationConfig::default()
        };
        let (mut sim, _ch) = test_loop(config);
        assert!(
            !sim.config.terrain.as_ref().unwrap().is_loaded(),
            "fixture precondition: cache must have zero tiles"
        );

        {
            let mut state = sim.state.write();
            state.quadrotor.position = [0.0, 0.0, 0.01];
            state.quadrotor.velocity = [0.0, 0.0, 1.0];
        }

        sim.step_physics(DT, STEP_US);

        let down = sim.state.read().quadrotor.position[2];
        assert!(
            (down - 0.0).abs() < 1e-6,
            "an unloaded terrain cache must clamp to flat ground at Z=0, \
             not free-fall as if the ground height were unknown; got {down}"
        );
    }

    /// Inside coverage the clamp must still fire, at the sampled terrain height.
    #[test]
    fn clamp_uses_sampled_terrain_height() {
        let config = SimulationConfig {
            terrain: Some(flat_terrain(1655.0)),
            ..SimulationConfig::default()
        };
        let (mut sim, _ch) = test_loop(config);

        let ground_z = sim
            .config
            .terrain
            .as_ref()
            .and_then(|t| t.sample_ground_ned(0.0, 0.0))
            .map(|z| z as f64)
            .expect("origin is inside the loaded tiles");
        assert!(
            ground_z.abs() < 1e-3,
            "flat terrain at the origin must put ground at NED 0, got {ground_z}"
        );

        {
            let mut state = sim.state.write();
            state.quadrotor.position = [0.0, 0.0, 5.0]; // 5 m below ground
        }
        sim.step_physics(DT, STEP_US);

        let down = sim.state.read().quadrotor.position[2];
        assert!(
            (down - ground_z).abs() < 1e-6,
            "inside coverage the drone must be clamped to the sampled ground \
             height {ground_z}, got {down}"
        );
    }

    /// Level ground must keep producing an exactly level attitude. Any residual
    /// tilt here becomes a lateral accelerometer bias that the EKF integrates
    /// into tens of metres of position drift.
    #[test]
    fn level_ground_produces_an_exactly_level_attitude() {
        for yaw in [0.0, 0.7, -2.1, std::f64::consts::PI] {
            let q = attitude_on_surface(LEVEL_NORMAL_NED, yaw);
            let (roll, pitch, got_yaw) = q.euler_angles();
            assert_eq!(roll, 0.0, "roll must be exactly zero on level ground");
            assert_eq!(pitch, 0.0, "pitch must be exactly zero on level ground");
            assert!((got_yaw - yaw).abs() < 1e-12, "yaw must be preserved");
        }
    }

    /// Resting on a slope must tilt the airframe with the surface rather than
    /// leaving it flat, and must preserve heading.
    #[test]
    fn resting_attitude_follows_the_surface_and_keeps_heading() {
        // 30-degree slope falling to the north: normal leans north.
        let theta: f64 = 30_f64.to_radians();
        let normal = [theta.sin() as f32, 0.0, -theta.cos() as f32];
        let yaw = 0.0;

        let q = attitude_on_surface(normal, yaw);
        let (roll, pitch, got_yaw) = q.euler_angles();

        assert!(
            (pitch.abs() - theta).abs() < 1e-6,
            "expected {theta} rad of pitch on a 30-degree slope, got {pitch}"
        );
        assert!(
            roll.abs() < 1e-6,
            "no roll expected for a pure north slope, got {roll}"
        );
        assert!((got_yaw - yaw).abs() < 1e-6, "heading must be preserved");

        // Body "down" must plant into the surface, i.e. oppose the normal.
        let z_body_ned = q * nalgebra::Vector3::new(0.0, 0.0, 1.0);
        let up = nalgebra::Vector3::new(normal[0] as f64, normal[1] as f64, normal[2] as f64);
        assert!(
            (z_body_ned.dot(&up) + 1.0).abs() < 1e-6,
            "body down should be antiparallel to the surface normal"
        );
    }

    #[test]
    fn downslope_accel_is_zero_on_level_ground_and_points_downhill_otherwise() {
        let g = 9.81;
        let flat = downslope_accel(LEVEL_NORMAL_NED, g);
        assert!(flat[0].abs() < 1e-12 && flat[1].abs() < 1e-12);

        // Ground falling towards the north: normal tilts north, so the pull
        // along the surface must also be northward.
        let theta: f64 = 30_f64.to_radians();
        let normal = [theta.sin() as f32, 0.0, -theta.cos() as f32];
        let a = downslope_accel(normal, g);
        assert!(a[0] > 0.0, "expected northward downhill pull, got {}", a[0]);
        // |a_tangential| = g sin(theta) cos(theta)... check the magnitude of
        // the full 3-vector instead: g*sin(theta).
        let mag = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
        assert!(
            (mag - g * theta.sin()).abs() < 1e-6,
            "expected g*sin(theta) = {}, got {mag}",
            g * theta.sin()
        );
    }

    /// A drone parked on gentle ground must stay parked: static friction has to
    /// beat the downhill pull, or landed vehicles creep forever.
    #[test]
    fn gentle_slopes_do_not_make_a_parked_drone_creep() {
        let g = 9.81;
        // 10 degrees: tan(10 deg) = 0.18, well under the 0.6 friction floor.
        let theta: f64 = 10_f64.to_radians();
        let normal = [theta.sin() as f32, 0.0, -theta.cos() as f32];

        let a = downslope_accel(normal, g);
        let pull = (a[0] * a[0] + a[1] * a[1]).sqrt();
        let normal_load = g * (-normal[2] as f64);
        assert!(
            pull < GROUND_FRICTION_COEFF * normal_load,
            "a 10-degree slope must not overcome static friction"
        );
    }

    /// Steep ground must overcome friction, otherwise the sim can never show a
    /// drone sliding off a hillside.
    #[test]
    fn steep_slopes_overcome_static_friction() {
        let g = 9.81;
        // 45 degrees: tan = 1.0, above the 0.6 friction floor.
        let theta: f64 = 45_f64.to_radians();
        let normal = [theta.sin() as f32, 0.0, -theta.cos() as f32];

        let a = downslope_accel(normal, g);
        let pull = (a[0] * a[0] + a[1] * a[1]).sqrt();
        let normal_load = g * (-normal[2] as f64);
        assert!(
            pull > GROUND_FRICTION_COEFF * normal_load,
            "a 45-degree slope must overcome static friction"
        );
    }

    /// The impact impulse tells the EKF how much velocity the ground removed.
    /// On terrain the drone will slide down, the ground does *not* remove the
    /// tangential component — friction bleeds it away over seconds, not over
    /// the 100 ms impulse window. Reporting a full stop there would desync the
    /// accelerometer from the simulated truth by the entire sliding speed.
    #[test]
    fn impact_impulse_only_arrests_the_normal_component_when_sliding() {
        // 45 degrees: tan = 1.0, comfortably past the friction threshold.
        let theta: f64 = 45_f64.to_radians();
        let normal = [theta.sin(), 0.0, -theta.cos()];
        let gravity = SimulationConfig::default().physics.gravity;

        // Confirm the fixture really is in the sliding regime.
        let a = downslope_accel([normal[0] as f32, 0.0, normal[2] as f32], gravity);
        let pull = (a[0] * a[0] + a[1] * a[1]).sqrt();
        assert!(pull > GROUND_FRICTION_COEFF * gravity * -normal[2]);

        // Descending fast with lateral motion across the slope.
        let v = nalgebra::Vector3::new(3.0, 0.0, 2.0);
        let n = nalgebra::Vector3::new(normal[0], normal[1], normal[2]);
        let arrested = n * v.dot(&n);

        // Only the into-surface component is removed, so the arrested vector
        // must be shorter than the full velocity and parallel to the normal.
        assert!(
            arrested.norm() < v.norm(),
            "sliding impact must arrest less than the whole velocity"
        );
        let cross = arrested.cross(&n).norm();
        assert!(
            cross < 1e-9,
            "arrested component must lie along the surface normal, got {arrested:?}"
        );

        // And the part left over is genuinely tangential — the drone keeps it.
        let residual = v - arrested;
        assert!(
            residual.dot(&n).abs() < 1e-9,
            "residual velocity must be tangential to the surface"
        );
        assert!(residual.norm() > 1.0, "a real sliding speed must survive");
    }

    /// Sensor ground mode latches on contact and only releases once the drone
    /// is clearly airborne, so the EKF sees clean [0,0,-g] through liftoff.
    #[test]
    fn ground_contact_latches_until_clearly_airborne() {
        let (mut sim, _ch) = test_loop(SimulationConfig::default());

        sim.state.write().quadrotor.position = [0.0, 0.0, 0.0];
        sim.step_physics(DT, STEP_US);
        assert!(sim.ground_contact_active, "resting on ground must latch on");

        // 5 cm up is inside the hysteresis band — still latched.
        {
            let mut state = sim.state.write();
            state.quadrotor.position = [0.0, 0.0, -0.05];
            state.quadrotor.velocity = [0.0, 0.0, 0.0];
        }
        sim.step_physics(DT, STEP_US);
        assert!(
            sim.ground_contact_active,
            "5 cm above ground is inside the 10 cm hysteresis band"
        );

        // 50 cm up is clearly airborne — latch releases.
        {
            let mut state = sim.state.write();
            state.quadrotor.position = [0.0, 0.0, -0.5];
            state.quadrotor.velocity = [0.0, 0.0, 0.0];
        }
        sim.step_physics(DT, STEP_US);
        assert!(
            !sim.ground_contact_active,
            "50 cm above ground must release the ground latch"
        );
    }

    // ────────────────────────────────────────────────────────────────────
    // Timebase: sub-stepping, the ground override, and the split clock
    // ────────────────────────────────────────────────────────────────────

    /// The `gyro_integral_dt` recorded in log101.ulg on 2026-09-05.
    const INCIDENT_GAP_US: u64 = 296_704;

    /// Everything a `HIL_SENSOR` puts on the wire, in MAVLink field order.
    ///
    /// A byte comparison rather than a tolerance: the healthy path must not
    /// move at all, and "close enough" cannot distinguish an unchanged
    /// emission from one whose noise happens to land nearby.
    fn hil_sensor_payload(d: &HIL_SENSOR_DATA) -> Vec<u8> {
        let mut out = Vec::with_capacity(60);
        out.extend_from_slice(&d.time_usec.to_le_bytes());
        for v in [
            d.xacc,
            d.yacc,
            d.zacc,
            d.xgyro,
            d.ygyro,
            d.zgyro,
            d.xmag,
            d.ymag,
            d.zmag,
            d.abs_pressure,
            d.diff_pressure,
            d.pressure_alt,
            d.temperature,
        ] {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out.extend_from_slice(&d.fields_updated.bits().to_le_bytes());
        out
    }

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    /// Take whatever the loop emitted since the last drain.
    fn drain(rx: &Receiver<MavMessage>) -> (Option<HIL_SENSOR_DATA>, Option<HIL_GPS_DATA>) {
        let (mut sensor, mut gps) = (None, None);
        while let Ok(msg) = rx.try_recv() {
            match msg {
                MavMessage::HIL_SENSOR(d) => sensor = Some(d),
                MavMessage::HIL_GPS(d) => gps = Some(d),
                _ => {}
            }
        }
        (sensor, gps)
    }

    /// One iteration's emission for a plan of `steps` sub-steps ending at
    /// `sim_time_us`, with no discarded time. Exactly what `run()` builds.
    fn emission(steps: u32, sim_time_us: u64, mean: SubStepSample) -> IterationEmission {
        IterationEmission {
            interval_dt: interval_dt(steps, DT),
            sim_time_us,
            integrated_time_us: sim_time_us,
            mean_accel_body: mean.accel_body,
            mean_gyro_body: mean.gyro_body,
        }
    }

    /// A seeded loop in a fixed pose, so an emission is reproducible.
    ///
    /// Seeded rather than noiseless: the point of the byte-identity guard is
    /// that the RNG stream is consumed in the same order and with the same
    /// `dt` as before, and a noiseless sensor consumes the stream while
    /// producing values that could not show a change.
    fn golden_loop(airborne: bool) -> (SimulationLoop, TestChannels) {
        let config = SimulationConfig::default();
        let (sim, ch) = test_loop(config.clone());
        {
            let mut state = sim.state.write();
            state.sensors = hitl_sensors::Sensors::with_config_and_seed(config.sensors.clone(), 42);
            state.quadrotor.quaternion = UnitQuaternion::from_euler_angles(0.05, -0.03, 0.7);
            if airborne {
                state.quadrotor.position = [1.0, 2.0, -50.0];
                state.quadrotor.velocity = [3.0, -1.0, 0.5];
                state.quadrotor.angular_velocity = [0.11, -0.07, 0.23];
                state.quadrotor.motor_speeds = [1100.0, 1105.0, 1095.0, 1102.0];
                state.motor_commands = [0.55, 0.56, 0.54, 0.55];
            } else {
                state.quadrotor.position = [0.0, 0.0, 0.0];
                state.quadrotor.velocity = [0.0, 0.0, 0.0];
            }
        }
        (sim, ch)
    }

    /// The healthy path must not move by one bit.
    ///
    /// `plan.steps == 1` is the identity case of the whole change: the mean of
    /// one sub-step is that sub-step and `interval_dt(1, dt)` is `dt`. If
    /// either drifts, every HITL session pays for a fault that affects almost
    /// none of them.
    ///
    /// The mag/baro schedule fires on the FIRST sample exactly as the old tick
    /// divider did, which is the only tick this golden covers. From the second
    /// update on it runs on a sim-time grid by design (step 12) and fires one
    /// iteration earlier per 20 ms — `sim >= next` at iterations 0, 7, 15, 23
    /// where `total_ticks % 8 == 0` fired at 0, 8, 16, 24. That is intended and
    /// harmless: each sensor owns its own `StdRng`, so the IMU stream this
    /// golden pins is untouched, and the cadence stays inside PX4's 300 ms
    /// voter by a factor of fifteen.
    ///
    /// The two arrays below were captured by RUNNING the pre-sub-stepping loop
    /// (`step_physics(dt)` then `sample_and_send_sensors(dt)`, working tree at
    /// commit ffee032 plus the Stage 1 observability commit) against this exact
    /// fixture, and pasted here. They cannot be regenerated from the current
    /// code — that is the point of a golden. The sensor chain is RNG-driven, so
    /// "compare against the old path" stops being runnable the moment the old
    /// path is deleted.
    #[test]
    fn single_step_emission_is_unchanged_byte_for_byte() {
        const GOLDEN_AIRBORNE: &str =
            "c40900000000000067ad09bf6673ce3e02764ac08e14e63d3bea6bbd9963973e5\
214513e3848b0bdf8dcfd3e7c204e4400000000ca1cd54401c37a40ff1f0000";
        const GOLDEN_ON_GROUND: &str =
            "c409000000000000070204bf5c3ab13ef7d807c195751e3be50d563c1a4f863d1\
bdb413ebaece2bde231fe3ea0664f4400000000d5dcce44d5c78740ff1f0000";

        for (airborne, golden) in [(true, GOLDEN_AIRBORNE), (false, GOLDEN_ON_GROUND)] {
            let (mut sim, ch) = golden_loop(airborne);
            let sample = sim.step_physics(DT, STEP_US);
            sim.sample_and_send_sensors(emission(1, STEP_US, sample));

            let (sensor, _) = drain(&ch.mav_rx);
            let sensor = sensor.expect("one HIL_SENSOR per iteration, always");
            assert_eq!(
                hex(&hil_sensor_payload(&sensor)),
                golden,
                "single-step emission changed (airborne={airborne}). This path runs \
                 on every healthy host at 400 Hz and must be byte-identical to the \
                 pre-sub-stepping loop; anything else means the fix moved the \
                 sensor stream for sessions that never had the fault."
            );
        }
    }

    /// A catch-up emission must carry the MEAN rate over its sub-steps.
    ///
    /// PX4 ignores `HIL_SENSOR.time_usec` and integrates each emitted rate over
    /// the wall interval between arrivals, so the quantity that has to be right
    /// is `emitted × interval`. The mean is the only value for which that
    /// equals the integral the sub-steps actually performed.
    #[test]
    fn gap_averaged_sample_delivers_the_correct_delta_angle() {
        let rates = [0.10_f64, 0.40, 0.70];
        let accumulated = rates.iter().fold(SubStepAccumulator::default(), |acc, r| {
            acc.plus(SubStepSample {
                accel_body: [0.0; 3],
                gyro_body: [*r, 0.0, 0.0],
            })
        });
        let mean = accumulated.mean();

        assert!(
            (mean.gyro_body[0] - 0.40).abs() < 1e-15,
            "expected the mean of {rates:?}, got {}",
            mean.gyro_body[0]
        );

        // `mean × 3·dt` is what PX4 forms from one sample stamped with a
        // three-step interval; it must equal `Σ ω·dt` over those three steps.
        let integral: f64 = rates.iter().map(|r| r * DT).sum();
        let px4_delta_angle = mean.gyro_body[0] * interval_dt(3, DT);
        assert!(
            (px4_delta_angle - integral).abs() < 1e-15,
            "PX4 would integrate {px4_delta_angle} rad where the simulation \
             turned {integral} rad"
        );

        assert_eq!(interval_dt(3, DT), 3.0 * DT);
        assert_eq!(
            interval_dt(1, DT),
            DT,
            "the identity case must be bit-exact, not merely close"
        );
    }

    /// The interval really does reach the IMU, at the VALUE it should be.
    ///
    /// The IMU scales its noise by `density / sqrt(dt)`, so two emissions with
    /// identical true inputs and different intervals cannot produce the same
    /// reading. Inequality alone would pass equally if `hitl-sensors` were
    /// handed 2 × dt, or any other wrong multiple, so the test pins the number
    /// that actually crosses the crate boundary: a hand-built emission
    /// carrying literally `3 × STEP_US` in seconds must be BIT-IDENTICAL to
    /// `emission(3, ..)`, and both must differ from `steps == 2`.
    #[test]
    fn the_emission_interval_reaches_the_imu() {
        /// `3 × STEP_US` written out in seconds, independent of `interval_dt`.
        const THREE_STEPS_S: f64 = (3 * STEP_US) as f64 / 1_000_000.0;

        let reading_for = |built: IterationEmission| {
            let (mut sim, ch) = golden_loop(true);
            let sample = sim.step_physics(DT, STEP_US);
            sim.sample_and_send_sensors(IterationEmission {
                mean_accel_body: sample.accel_body,
                mean_gyro_body: sample.gyro_body,
                ..built
            });
            let (sensor, _) = drain(&ch.mav_rx);
            sensor.expect("HIL_SENSOR").xacc
        };

        let blank = SubStepSample::default();
        let via_helper = reading_for(emission(3, STEP_US, blank));
        let via_literal = reading_for(IterationEmission {
            interval_dt: THREE_STEPS_S,
            ..emission(3, STEP_US, blank)
        });
        let one_step = reading_for(emission(1, STEP_US, blank));
        let two_steps = reading_for(emission(2, STEP_US, blank));

        assert_eq!(
            via_helper, via_literal,
            "the interval that reached hitl-sensors must be exactly \
             {THREE_STEPS_S} s — 3 × STEP_US — not merely some interval other \
             than dt"
        );
        assert_ne!(
            one_step, via_helper,
            "a three-step emission must be handed 3 × dt; identical readings \
             mean the sub-step count never reached hitl-sensors and PX4 would \
             integrate the noise of a 2.5 ms sample over 7.5 ms"
        );
        assert_ne!(
            two_steps, via_helper,
            "and the value has to be the right multiple, not just a larger one"
        );
    }

    /// The ground-contact override SUBSTITUTES; it never blends.
    ///
    /// CLAUDE.md ends the branch this replaces with "Do not merge the
    /// branches": on level ground the accelerometer must read an exact
    /// `[0, 0, −g]`, because a 0.25° residual is a 0.043 m/s² lateral bias that
    /// the EKF integrates into 76 m of drift over a 77 s flight. Averaging a
    /// contact sample with airborne sub-steps manufactures exactly such a
    /// residual, and a catch-up is where it would appear.
    #[test]
    fn catchup_never_blends_the_ground_contact_override() {
        let gravity = SimulationConfig::default().physics.gravity;
        let (mut sim, ch) = test_loop(noiseless());
        sim.total_ticks = 1;

        // Contact latched, and a loud airborne mean standing in for four
        // sub-steps of which only the last two touched down.
        sim.ground_contact_active = true;
        sim.ground_rest_accel_body = [0.0, 0.0, -gravity];
        let loud = SubStepSample {
            accel_body: [7.5, -3.25, -19.0],
            gyro_body: [0.0; 3],
        };

        assert_eq!(
            sim.ground_override(4 * STEP_US),
            Some([0.0, 0.0, -gravity]),
            "the override is decided from contact state alone — it cannot see \
             the airborne mean, so a blend is unrepresentable rather than \
             merely avoided"
        );

        sim.sample_and_send_sensors(emission(4, 4 * STEP_US, loud));
        let (sensor, _) = drain(&ch.mav_rx);
        let sensor = sensor.expect("HIL_SENSOR");
        assert_eq!(sensor.xacc, 0.0_f32, "body X must be exactly zero");
        assert_eq!(sensor.yacc, 0.0_f32, "body Y must be exactly zero");
        assert_eq!(
            sensor.zacc,
            (-gravity) as f32,
            "body Z must be exactly −g, not −g averaged with {}",
            loud.accel_body[2]
        );
    }

    /// The same substitution for the impact impulse, which outranks contact.
    #[test]
    fn the_impact_impulse_substitutes_for_the_whole_emission() {
        let (mut sim, ch) = test_loop(noiseless());
        sim.total_ticks = 1;
        sim.ground_contact_active = true;
        sim.ground_rest_accel_body = [0.0, 0.0, -9.80665];
        sim.ground_impact_accel = [1.5, -2.5, -30.0];
        sim.ground_impact_until_us = Some(4 * STEP_US + IMPACT_DURATION_US);

        assert_eq!(
            sim.ground_override(4 * STEP_US),
            Some([1.5, -2.5, -30.0]),
            "an active impulse outranks resting contact"
        );

        let loud = SubStepSample {
            accel_body: [7.5, -3.25, -19.0],
            gyro_body: [0.0; 3],
        };
        sim.sample_and_send_sensors(emission(4, 4 * STEP_US, loud));
        let (sensor, _) = drain(&ch.mav_rx);
        let sensor = sensor.expect("HIL_SENSOR");
        assert_eq!(sensor.xacc, 1.5_f32);
        assert_eq!(sensor.yacc, -2.5_f32);
        assert_eq!(sensor.zacc, -30.0_f32);
    }

    /// Catch-up runs THROUGH the impulse, and the impulse does not notice.
    ///
    /// The clamp that used to sit here was justified by "compressing it into
    /// one averaged sample destroys the event". It cannot:
    /// [`SimulationLoop::ground_override`] substitutes a constant for the whole
    /// emission while the impulse is live and never sees the sub-step mean, so
    /// the emitted accelerations are bit-identical at one sub-step or four.
    /// What the clamp DID change is how much WALL time the 100 ms window
    /// spans, and PX4 integrates each sample over the wall interval between
    /// arrivals — which over-delivered the landing delta-v by exactly the
    /// loop's lateness. `the_impact_impulse_covers_the_wall_interval_it_declares`
    /// in `timebase` is the planner half of this.
    #[test]
    fn catchup_runs_during_the_impulse_and_does_not_change_what_it_emits() {
        let (mut sim, ch) = test_loop(noiseless());
        sim.total_ticks = 1;

        {
            let mut state = sim.state.write();
            state.quadrotor.position = [0.0, 0.0, -0.001];
            state.quadrotor.velocity = [0.0, 0.0, 2.0];
        }
        sim.step_physics(DT, STEP_US);
        assert!(
            sim.impact_active(STEP_US),
            "a 2 m/s descent must arm the impulse"
        );

        let mut tb = Timebase::new(STEP_US, GAP_THRESHOLD_US, MAX_CATCHUP_STEPS);
        let plan = tb.plan(4 * STEP_US);
        assert_eq!(
            plan.steps, 4,
            "a four-step deficit is paid off in four sub-steps while the impulse \
             is live: leaving it unpaid stretches the impulse's wall span by the \
             factor the loop is behind, and the EKF integrates over wall time"
        );

        // Same instant, same impulse, two different sub-step counts: the wire
        // values must be identical, which is why the clamp bought nothing.
        let loud = SubStepSample {
            accel_body: [1.5, -2.5, -30.0],
            gyro_body: [0.25, -0.5, 0.75],
        };
        sim.sample_and_send_sensors(emission(1, 2 * STEP_US, loud));
        let one = drain(&ch.mav_rx).0.expect("HIL_SENSOR");
        sim.sample_and_send_sensors(emission(4, 3 * STEP_US, SubStepSample::default()));
        let four = drain(&ch.mav_rx).0.expect("HIL_SENSOR");

        assert_eq!(
            (one.xacc, one.yacc, one.zacc),
            (four.xacc, four.yacc, four.zacc),
            "the impulse is SUBSTITUTED for the emission, so neither the \
             sub-step count nor the airborne mean can reach the wire while it \
             is live"
        );
    }

    /// The impulse lasts 100 ms of SIM time, not forty iterations.
    ///
    /// Driven at TWO sub-steps per iteration for the whole fixture, so the two
    /// rules genuinely disagree inside it: a 40-iteration counter expires at
    /// 40 iterations, which here is 200 ms of sim time, while the sim-time
    /// deadline expires at 100 ms — iteration 20. Every iteration is checked
    /// against the sim-time rule, so a counter of iterations fails at the
    /// twentieth rather than being let through by a fixture that only ever
    /// samples two synthetic instants.
    #[test]
    fn impact_impulse_lasts_100ms_of_sim_time_not_40_iterations() {
        /// Sub-steps per iteration. Exactly what a Regime B catch-up produces.
        const STEPS_PER_ITERATION: u64 = 2;
        /// Long enough to pass both the sim-time deadline (iteration 20) and
        /// the iteration count that used to hold it (iteration 40).
        const ITERATIONS: u64 = 60;

        let (mut sim, _ch) = test_loop(SimulationConfig::default());
        {
            let mut state = sim.state.write();
            state.quadrotor.position = [0.0, 0.0, -0.001];
            state.quadrotor.velocity = [0.0, 0.0, 2.0];
        }

        sim.step_physics(DT, STEP_US);
        let armed_at = STEP_US;
        assert!(
            sim.impact_active(armed_at),
            "a 2 m/s descent arms the impulse"
        );

        let mut cleared_at_iteration = None;
        for i in 1..=ITERATIONS {
            let iteration_end_us = armed_at + i * STEPS_PER_ITERATION * STEP_US;
            for step in (0..STEPS_PER_ITERATION).rev() {
                sim.step_physics(DT, iteration_end_us - step * STEP_US);
            }

            let expected = iteration_end_us < armed_at + IMPACT_DURATION_US;
            assert_eq!(
                sim.impact_active(iteration_end_us),
                expected,
                "at iteration {i}, sim time {} µs after the impact: the \
                 impulse must follow the 100 ms sim-time deadline, not a count \
                 of iterations",
                iteration_end_us - armed_at
            );
            if !expected && cleared_at_iteration.is_none() {
                cleared_at_iteration = Some(i);
            }
        }

        assert_eq!(
            cleared_at_iteration,
            Some(20),
            "at two sub-steps per iteration the impulse must clear on the \
             TWENTIETH iteration, which is where the two rules part company — \
             a 40-iteration counter would hold it for 200 ms of sim time, \
             double the window the EKF is given to reconcile the velocity the \
             ground removed"
        );
    }

    /// The one place the healthy path DID move, pinned deliberately.
    ///
    /// At one sub-step per iteration — a loop keeping perfect cadence — the
    /// impulse is now emitted 40 times, where the pre-change loop emitted it
    /// 39. The old code set `ground_impact_ticks_remaining = 40` and then
    /// decremented it to 39 within the SAME call, so the arming emission saw
    /// 39. The new `sim_time_us < until` test is true on the arming emission
    /// itself, giving T0, T0+2500 … T0+97500.
    ///
    /// 40 is the correct count: `IMPACT_DURATION_US` is 100 ms and 40 × 2.5 ms
    /// is exactly 100 ms. The extra 2.5 ms tick is intended, and this test is
    /// what says so — the golden covers a single emission, so nothing else
    /// pins it.
    #[test]
    fn the_impulse_spans_forty_emissions_at_one_substep_per_iteration() {
        let (mut sim, _ch) = test_loop(SimulationConfig::default());
        {
            let mut state = sim.state.write();
            state.quadrotor.position = [0.0, 0.0, -0.001];
            state.quadrotor.velocity = [0.0, 0.0, 2.0];
        }

        sim.step_physics(DT, STEP_US);
        let armed_at = STEP_US;

        let mut emissions: u64 = 0;
        // Well past the 100 ms window, so the count is the window's and not
        // the loop bound's.
        for i in 0..80 {
            let sim_time_us = armed_at + i * STEP_US;
            if sim.impact_active(sim_time_us) {
                emissions += 1;
            }
        }

        assert_eq!(
            emissions, 40,
            "40 × 2500 µs is exactly IMPACT_DURATION_US. The pre-change loop emitted 39 \
             because it decremented its counter inside the arming call; if this reads 39 \
             again, the deadline has been made exclusive of its own first tick"
        );
        assert_eq!(
            emissions * STEP_US,
            IMPACT_DURATION_US,
            "the emission count and the declared duration must be the same window"
        );
    }

    /// A schedule that can only ever run early.
    ///
    /// PX4 declares a mag or baro dead after 300 ms without a sample
    /// (`VehicleMagnetometer.cpp:46` / `VehicleAirData.cpp:47`) and those were
    /// the exact alarms the FC raised on 2026-09-05. The SCHEDULE's slots are
    /// bounded above by the period whatever the loop does; the observed
    /// spacing is one iteration looser, and
    /// `mag_and_baro_hold_50_hz_in_sim_time_across_a_catchup` pins that side.
    #[test]
    fn mag_and_baro_cadence_stays_above_50_hz() {
        for now_us in [
            0_u64,
            1,
            2500,
            19_999,
            20_000,
            20_001,
            296_704,
            u64::MAX / 2,
        ] {
            let next = next_slot_us(now_us, SENSOR_50HZ_PERIOD_US);
            assert!(next > now_us, "the schedule must advance from {now_us}");
            assert!(
                next - now_us <= SENSOR_50HZ_PERIOD_US,
                "at {now_us} the next update is {} µs away, past the 20 000 µs \
                 budget; drifting toward PX4's 300 ms voter is how MAG #0 \
                 TIMEOUT happens as a side effect of a timebase change",
                next - now_us
            );
        }
        assert_eq!(
            next_slot_us(0, 0),
            1,
            "a degenerate period must not divide by zero in the sim thread"
        );
    }

    /// Held at a true 50 Hz of sim time even when iterations and sim time
    /// diverge.
    #[test]
    fn mag_and_baro_hold_50_hz_in_sim_time_across_a_catchup() {
        let (mut sim, ch) = test_loop(SimulationConfig::default());
        // Past the first tick, whose flags are forced to include every sensor.
        sim.total_ticks = 1;

        // Mixed sub-step counts, so the iteration cadence and the sim clock
        // disagree the way they do during a catch-up.
        let pattern = [1_u32, 2, 3, 8, 5, 1, 4, 2];
        let mut sim_us = 0_u64;
        let mut cursor = 0_usize;
        // Sim times at which a mag update was actually EMITTED, so the test
        // can grade observed spacing and not only the count.
        let mut mag_emitted_at: Vec<u64> = Vec::new();
        let mut drive = |sim: &mut SimulationLoop,
                         sim_us: &mut u64,
                         target_us: u64,
                         mag_emitted_at: &mut Vec<u64>| {
            let (mut mag, mut baro) = (0_usize, 0_usize);
            while *sim_us < target_us {
                let remaining = (target_us - *sim_us) / STEP_US;
                let steps = u64::from(pattern[cursor % pattern.len()])
                    .min(remaining)
                    .max(1) as u32;
                cursor += 1;
                *sim_us += u64::from(steps) * STEP_US;
                sim.sample_and_send_sensors(emission(steps, *sim_us, SubStepSample::default()));
                let (sensor, _) = drain(&ch.mav_rx);
                let flags = sensor.expect("HIL_SENSOR").fields_updated;
                if flags.contains(MAG_FLAGS) {
                    mag += 1;
                    mag_emitted_at.push(*sim_us);
                }
                if flags.contains(BARO_FLAGS) {
                    baro += 1;
                }
            }
            (mag, baro)
        };

        // First 100 ms primes the schedule (the very first sample is due at
        // once, matching the old tick-zero divider).
        drive(&mut sim, &mut sim_us, 100_000, &mut mag_emitted_at);
        assert_eq!(sim_us, 100_000, "the fixture must land exactly on 100 ms");

        let (mag, baro) = drive(&mut sim, &mut sim_us, 200_000, &mut mag_emitted_at);
        assert_eq!(
            (mag, baro),
            (5, 5),
            "100 ms of sim time is exactly five 50 Hz updates, however many \
             iterations it took"
        );

        // The count alone does not bound the WORST spacing, and the worst
        // spacing is what PX4's 300 ms voter grades. An update fires on the
        // first emission at or after its slot, and that emission can carry a
        // whole catch-up's worth of sub-steps, so the real bound is `period +
        // steps × step_us` — never the bare period.
        let worst = mag_emitted_at
            .windows(2)
            .map(|w| w[1] - w[0])
            .max()
            .expect("the fixture emits several mag updates");
        assert!(
            worst <= SENSOR_50HZ_PERIOD_US + GAP_THRESHOLD_US,
            "observed mag spacing reached {worst} µs; the bound is {} µs, and \
             it has to stay far inside PX4's 300 ms voter \
             (VehicleMagnetometer.cpp:46)",
            SENSOR_50HZ_PERIOD_US + GAP_THRESHOLD_US
        );
        assert!(
            worst > SENSOR_50HZ_PERIOD_US,
            "the pattern must actually make an emission overshoot a slot, or \
             this assertion proves nothing about the catch-up case it is named \
             for; saw {worst} µs"
        );
    }

    /// The split clock, stated as the post-fix ULog cross-check.
    ///
    /// `HIL_SENSOR` carries wall-anchored sim time; `HIL_GPS` carries the time
    /// actually simulated. Their difference is the wall time the daemon admits
    /// it lost, so `(FC hrt elapsed) − (sensor_gps.time_utc_usec elapsed)` in
    /// the log reads out the same number the daemon reports — zero on a healthy
    /// run, the fault magnitude on a bad one.
    #[test]
    fn hil_gps_time_usec_lags_hil_sensor_by_exactly_the_discarded_time() {
        let (mut sim, ch) = test_loop(SimulationConfig::default());
        sim.total_ticks = 1;

        let mut tb = Timebase::new(STEP_US, GAP_THRESHOLD_US, MAX_CATCHUP_STEPS);
        let gap = tb.plan(INCIDENT_GAP_US);
        assert!(gap.gap, "296.7 ms is past PX4's own imu_max_dt");
        assert_eq!(tb.discarded_us(), INCIDENT_GAP_US - GAP_THRESHOLD_US);
        assert_eq!(tb.sim_time_us(), INCIDENT_GAP_US);
        assert_eq!(tb.integrated_time_us(), GAP_THRESHOLD_US);
        let discarded = tb.discarded_us();

        // Drive on until the GPS delay buffer produces a reading, so the
        // assertion is made on a real pair rather than on the builders.
        let mut wall_us = INCIDENT_GAP_US;
        let mut checked = false;
        for _ in 0..800 {
            wall_us += STEP_US;
            let plan = tb.plan(wall_us);
            sim.sample_and_send_sensors(IterationEmission {
                interval_dt: interval_dt(plan.steps, DT),
                sim_time_us: tb.sim_time_us(),
                integrated_time_us: tb.integrated_time_us(),
                mean_accel_body: [0.0, 0.0, -9.80665],
                mean_gyro_body: [0.0; 3],
            });
            let (sensor, gps) = drain(&ch.mav_rx);
            let sensor = sensor.expect("HIL_SENSOR");
            assert_eq!(sensor.time_usec, tb.sim_time_us());
            if let Some(gps) = gps {
                assert_eq!(gps.time_usec, tb.integrated_time_us());
                assert_eq!(
                    sensor.time_usec - gps.time_usec,
                    discarded,
                    "the two stamps must differ by exactly the discarded time — \
                     that difference IS the post-fix ULog cross-check"
                );
                checked = true;
                break;
            }
        }
        assert!(checked, "the GPS never produced a reading to compare");
        assert_eq!(
            discarded, 276_704,
            "20 ms absorbed, the rest declared lost and named"
        );
    }

    /// The gap line has to be readable by someone who has never seen this code.
    ///
    /// Its job is to shorten the next investigation, and the specific thing it
    /// has to do is name the FC alarms as a consequence rather than a cause.
    /// On 2026-09-05 MAG #0 / BARO #0 TIMEOUT sent days into hunting an I2C
    /// fault on a board with no I2C bus.
    #[test]
    fn a_gap_line_predicts_the_fc_symptom_by_name() {
        let message = gap_message(
            INCIDENT_GAP_US,
            GAP_THRESHOLD_US,
            INCIDENT_GAP_US - GAP_THRESHOLD_US,
        );

        assert!(
            message.contains("296 ms"),
            "the line must state the stall it observed: {message}"
        );
        assert!(
            message.contains("20 ms were absorbed"),
            "and how much of it was replayed: {message}"
        );
        assert!(
            message.contains("276 ms could not be simulated"),
            "and how much is simply gone: {message}"
        );
        assert!(
            message.contains("MAG #0 / BARO #0 TIMEOUT"),
            "it must name the FC alarms this gap will cause: {message}"
        );
        assert!(
            message.contains("there is no I2C bus"),
            "and rule out the hardware the last investigation chased: {message}"
        );
        assert!(
            message.contains("--selftest-timebase"),
            "and carry the remedy: {message}"
        );

        // A rate whose step does not divide GAP_THRESHOLD_US: `--tick-rate 625`
        // is accepted (it divides both the microsecond and the nanosecond
        // grid), its step is 1600 µs, and the budget is therefore 12 whole
        // steps — 19.2 ms of the 20 ms threshold. Passing GAP_THRESHOLD_US
        // unconditionally would put a figure in the line that the plan did not
        // run.
        const STEP_625HZ_US: u64 = 1_600;
        let budget = crate::timebase::Timebase::budget_steps_at(STEP_625HZ_US, GAP_THRESHOLD_US);
        assert_eq!(budget, 12, "20 000 / 1600 == 12 whole steps");
        let absorbed = u64::from(budget) * STEP_625HZ_US;
        assert_ne!(
            absorbed, GAP_THRESHOLD_US,
            "this case only tests anything if the two genuinely differ"
        );

        let partial = gap_message(INCIDENT_GAP_US, absorbed, INCIDENT_GAP_US - absorbed);
        assert!(
            partial.contains("19 ms were absorbed"),
            "12 × 1600 µs was replayed, not the 20 ms threshold: {partial}"
        );
        assert!(
            partial.contains("277 ms could not be simulated"),
            "and `deficit == absorbed + discarded` has to still hold: {partial}"
        );
    }

    /// Changing the build must not launder a compromised session.
    #[test]
    fn a_reconfigure_does_not_erase_a_recorded_gap() {
        let before = SimulationStats {
            hil_sensor_count: 12_345,
            actuator_count: 999,
            timebase_discarded_ms: 276,
            timebase_gap_count: 1,
            timebase_max_gap_ms: 296,
            catchup_steps: 4_096,
            tick_rate_hz: 341.2,
            steps_per_sec: 400.1,
            wake_late_avg_us: 190,
            wake_late_max_us: 4_812,
            wake_late_p99_us: 435,
            deadline_overrun_us: 61,
            timebase_step_us: 2500,
            session_invalid: true,
            timebase_health: protocol::TimebaseHealth::Faulted,
            timebase_worst_health: protocol::TimebaseHealth::Faulted,
            ..Default::default()
        };

        let after = stats_after_reconfigure(&before);

        assert_eq!(after.hil_sensor_count, 0, "counts belong to the new build");
        assert_eq!(after.actuator_count, 0);
        assert_eq!(after.timebase_discarded_ms, 276);
        assert_eq!(after.timebase_gap_count, 1);
        assert_eq!(after.timebase_max_gap_ms, 296);
        assert_eq!(after.catchup_steps, 4_096);
        assert!(
            after.session_invalid,
            "a session that lost wall time must not be able to hide it by \
             changing the build"
        );
        assert_eq!(
            after.timebase_health,
            protocol::TimebaseHealth::Faulted,
            "defaulting this back to Ok told the browser annunciator the fault \
             had cleared — a green banner to the person flying, retracted by a \
             fresh CRITICAL a second later"
        );
        assert_eq!(
            after.timebase_worst_health,
            protocol::TimebaseHealth::Faulted
        );

        // The pacing group measures the HOST, not the build. `tb_session` —
        // the thing all four are computed from — is untouched by a
        // reconfigure, so zeroing any of them publishes a metric that
        // contradicts the ones beside it.
        assert_eq!(after.tick_rate_hz, 341.2);
        assert_eq!(after.steps_per_sec, 400.1);
        assert_eq!(after.wake_late_avg_us, 190);
        assert_eq!(
            after.deadline_overrun_us, 61,
            "wake lateness and deadline overrun are the pair that separates a \
             descheduled host from a compute-bound one; carrying one and \
             zeroing the other publishes half a diagnosis"
        );
        assert_eq!(after.timebase_step_us, 2500);
        assert_eq!(
            after.wake_late_max_us, 4_812,
            "carrying the p99 while zeroing the max publishes, for up to the \
             second until the next roll-up, exactly the 'window max under a \
             session p99' that this field's own documentation calls a broken \
             metric"
        );
        assert_eq!(after.wake_late_p99_us, 435);
        assert!(
            after.wake_late_p99_us <= after.wake_late_max_us,
            "a p99 above the max is the shape of a metric nobody believes on \
             its first real incident"
        );
    }

    /// A mean of one sub-step is that sub-step, bit for bit — including the
    /// sign of a zero, which `0.0 + x` would have destroyed.
    #[test]
    fn a_single_sub_step_mean_is_the_sub_step_itself() {
        let sample = SubStepSample {
            accel_body: [-0.0, 1.25, -9.80665],
            gyro_body: [0.0, -0.0, 0.5],
        };
        let mean = SubStepAccumulator::default().plus(sample).mean();
        assert_eq!(mean, sample);
        assert!(
            mean.accel_body[0].is_sign_negative(),
            "a negative zero must survive: it is a different f32 on the wire"
        );
        assert!(mean.gyro_body[1].is_sign_negative());
    }

    /// Catch-up adds no outbound traffic, which is why backpressure is not a
    /// reason to withhold it.
    ///
    /// This is the measurement the deleted `channel_pressure()` clamp was
    /// justified by and which nobody made: ONE `HIL_SENSOR` leaves per
    /// `sample_and_send_sensors` call whatever the sub-step count, and the loop
    /// calls it exactly once per iteration.
    #[test]
    fn an_eight_substep_iteration_emits_exactly_one_hil_sensor() {
        let (mut sim, ch) = test_loop(noiseless());
        sim.total_ticks = 1;

        for steps in [1_u32, 8] {
            sim.sample_and_send_sensors(emission(
                steps,
                u64::from(steps) * STEP_US,
                SubStepSample::default(),
            ));
            let sensors = ch
                .mav_rx
                .try_iter()
                .filter(|m| matches!(m, MavMessage::HIL_SENSOR(_)))
                .count();
            assert_eq!(
                sensors, 1,
                "an iteration running {steps} sub-steps must still put exactly \
                 one HIL_SENSOR on the channel — catch-up is upstream work that \
                 produces no downstream message, so withholding it while the FC \
                 is not draining buys no channel depth at all"
            );
        }
    }

    /// A config whose IMU adds no noise, so an emitted value can be compared
    /// exactly against the specific force that went in.
    fn noiseless() -> SimulationConfig {
        SimulationConfig {
            sensors: hitl_sensors::SensorsConfig {
                imu: hitl_sensors::ImuConfig {
                    gyro_noise_density: 0.0,
                    accel_noise_density: 0.0,
                    gyro_bias_sigma: 0.0,
                    accel_bias_sigma: 0.0,
                    ..hitl_sensors::ImuConfig::default()
                },
                ..hitl_sensors::SensorsConfig::default()
            },
            ..SimulationConfig::default()
        }
    }
}
