//! Main simulation loop running at 400 Hz

use crate::state::{SimulationConfig, SimulationState};
use crossbeam_channel::{Receiver, Sender, TryRecvError};
use hitl_physics::{
    rk4_step, throttle_to_omega_with_config, total_motor_current, BatteryConfig, PhysicsConfig,
};
use mavlink::ardupilotmega::{HilSensorUpdatedFlags, MavMessage, HIL_GPS_DATA, HIL_SENSOR_DATA};
use protocol::ActuatorOutputs;
pub use protocol::SimulationStats;
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

/// Length of the ground-impact deceleration impulse, in ticks (40 @ 400 Hz = 100 ms).
const IMPACT_DURATION_TICKS: u32 = 40;

/// Minimum downward speed (m/s) that generates an impact impulse. Below this a
/// landing is gentle enough that the clamp alone does not confuse the EKF.
const IMPACT_VEL_THRESHOLD: f64 = 0.5;

/// Mag sensor update divider (400 Hz / 8 = 50 Hz)
const MAG_UPDATE_DIVIDER: u64 = 8;
/// Baro sensor update divider (400 Hz / 8 = 50 Hz)
const BARO_UPDATE_DIVIDER: u64 = 8;

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

/// Main simulation loop
pub struct SimulationLoop {
    state: SimulationState,
    config: SimulationConfig,
    actuator_rx: Receiver<ActuatorOutputs>,
    config_rx: Receiver<(PhysicsConfig, BatteryConfig, hitl_sensors::SensorsConfig)>,
    mav_tx: Sender<MavMessage>,
    stats: SimulationStats,
    /// Watch channel used by the TUI / web status widget to render live
    /// loop + drone state. `None` when nothing subscribes (tests, benches).
    stats_tx: Option<watch::Sender<SimulationStats>>,
    /// Total ticks executed since startup — used to identify the first
    /// tick for sensor-value logging, not surfaced in `SimulationStats`
    /// because the cumulative HIL counts already convey progress.
    total_ticks: u64,
    /// Tracks whether `ConfigureBuild` has been applied at least once so the
    /// header can show "no build configured" vs default values.
    build_configured: bool,
    /// Cached mag reading (only updated at MAG_UPDATE_DIVIDER rate)
    last_mag: Option<hitl_sensors::MagReading>,
    /// Cached baro reading (only updated at BARO_UPDATE_DIVIDER rate)
    last_baro: Option<hitl_sensors::BaroReading>,
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
    /// Set by step_physics when the ground clamp fires or the quad is at z=0.
    /// Read by sample_and_send_sensors to produce clean [0,0,-g] accel on ground.
    ground_contact_active: bool,
    /// Remaining ticks of ground impact deceleration impulse. When > 0, the
    /// accelerometer reports the deceleration force instead of gravity-only.
    ground_impact_ticks_remaining: u32,
    /// Body-frame deceleration acceleration to report during ground impact (m/s²).
    ground_impact_accel: [f64; 3],
}

impl SimulationLoop {
    /// Create a new simulation loop
    pub fn new(
        config: SimulationConfig,
        actuator_rx: Receiver<ActuatorOutputs>,
        config_rx: Receiver<(PhysicsConfig, BatteryConfig, hitl_sensors::SensorsConfig)>,
        mav_tx: Sender<MavMessage>,
    ) -> Self {
        let state = SimulationState::new(config.clone());

        Self {
            state,
            config,
            actuator_rx,
            config_rx,
            mav_tx,
            stats: SimulationStats::default(),
            stats_tx: None,
            total_ticks: 0,
            build_configured: false,
            last_mag: None,
            last_baro: None,
            last_drop_warning: Instant::now(),
            drop_count_since_last_warning: 0,
            channel_full_since: None,
            last_actuator_time: Instant::now(),
            last_stale_warning: Instant::now(),
            last_terrain_miss_warning: Instant::now(),
            ground_contact_active: true,
            ground_impact_ticks_remaining: 0,
            ground_impact_accel: [0.0; 3],
        }
    }

    /// Attach a `watch::Sender` so the loop publishes live stats every
    /// `STATS_PUBLISH_INTERVAL`. Call once before `run()`.
    pub fn with_stats_publisher(mut self, tx: watch::Sender<SimulationStats>) -> Self {
        self.stats_tx = Some(tx);
        self
    }

    /// Get shared state handle for other threads
    pub fn state_handle(&self) -> SimulationState {
        self.state.clone()
    }

    /// Run the simulation loop (blocking)
    pub fn run(&mut self) {
        let tick_duration = Duration::from_nanos(1_000_000_000 / self.config.tick_rate_hz as u64);
        let dt = 1.0 / self.config.tick_rate_hz as f64;

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

        // Last time we pushed a snapshot to `stats_tx`.
        let mut last_stats_publish = Instant::now();

        // Absolute scheduling: advance next_tick by one period each iteration so overruns
        // don't accumulate (a single 8s spike won't cause 8s of catch-up busy-looping).
        let mut next_tick = Instant::now();

        while self.state.is_running() {
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
                    // Reset cumulative counters so the header reflects the new build,
                    // not aggregate counts across builds.
                    self.stats = SimulationStats::default();
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => break,
            }

            // Process any pending actuator commands (non-blocking)
            self.process_actuator_commands();

            // Step physics
            self.step_physics(dt);

            // Sample sensors and send HIL messages
            self.sample_and_send_sensors(dt);

            self.total_ticks += 1;
            window_ticks += 1;

            let latency_us = tick_start.elapsed().as_micros() as u64;
            window_latency_us += latency_us;
            if latency_us > window_max_latency_us {
                window_max_latency_us = latency_us;
            }

            // Roll up window stats every STATS_WINDOW_INTERVAL.
            if window_start.elapsed() >= STATS_WINDOW_INTERVAL {
                let window_secs = window_start.elapsed().as_secs_f64();
                self.stats.tick_rate_hz = (window_ticks as f64 / window_secs) as f32;
                self.stats.avg_latency_us = (window_latency_us as f64 / window_ticks as f64) as u32;
                self.stats.max_latency_us = window_max_latency_us as u32;

                // Keep the formatted log line as `debug!` for users who tail
                // logs at debug level. The TUI gets the same data via watch.
                debug!(
                    tick_rate_hz = self.stats.tick_rate_hz,
                    avg_latency_us = self.stats.avg_latency_us,
                    max_latency_us = self.stats.max_latency_us,
                    hil_sensor = self.stats.hil_sensor_count,
                    hil_gps = self.stats.hil_gps_count,
                    actuators = self.stats.actuator_count,
                    "sim window stats"
                );

                window_start = Instant::now();
                window_ticks = 0;
                window_latency_us = 0;
                window_max_latency_us = 0;
            }

            // Publish a live snapshot to the TUI / status subscribers.
            if last_stats_publish.elapsed() >= STATS_PUBLISH_INTERVAL {
                self.publish_stats();
                last_stats_publish = Instant::now();
            }

            // Absolute-deadline sleep: skip ticks we're already past rather than catching up.
            next_tick += tick_duration;
            let now = Instant::now();
            if next_tick > now {
                spin_sleep::sleep(next_tick - now);
            } else {
                // We're behind; reset deadline to now to avoid a burst of catch-up ticks.
                next_tick = now;
                trace!(
                    latency_us,
                    target_us = tick_duration.as_micros(),
                    "Tick overrun — deadline reset"
                );
            }
        }

        info!("Simulation loop stopped");
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
            self.state.set_motor_commands(actuator.motors);
            self.state.set_armed(actuator.is_armed());

            if actuator.is_hil_active() {
                trace!(
                    motors = ?actuator.motors,
                    armed = actuator.is_armed(),
                    "Received actuator commands"
                );
            }
        } else {
            // No command received this tick — check for staleness while motors are active.
            let motors_active = self.state.read().motor_commands.iter().any(|&c| c > 0.01);
            if motors_active && self.last_actuator_time.elapsed() > ACTUATOR_STALE_TIMEOUT {
                // Zero motors and disarm: FC is gone, let gravity take over.
                self.state.set_motor_commands([0.0; 4]);
                self.state.set_armed(false);

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

    /// Step the physics simulation
    fn step_physics(&mut self, dt: f64) {
        let mut state = self.state.write();

        // Ground height in NED (Z > ground_z means below ground). `None` means
        // the height is *unknown*: terrain is configured but the drone has left
        // the cached tile block. Unknown must not be conflated with "flat at the
        // origin datum" — doing so teleports a drone flying below the origin
        // straight up to it, a discontinuity the EKF cannot reconcile.
        // Flat ground at Z=0 applies only when no terrain is configured at all.
        let ground_z: Option<f64> = match self.config.terrain.as_ref() {
            Some(terrain) => terrain
                .sample_ground_ned(
                    state.quadrotor.position[0], // north
                    state.quadrotor.position[1], // east
                )
                .map(|z| z as f64),
            None => Some(0.0),
        };

        if ground_z.is_none() && self.last_terrain_miss_warning.elapsed() >= TERRAIN_MISS_WARN_INTERVAL
        {
            warn!(
                north = state.quadrotor.position[0],
                east = state.quadrotor.position[1],
                "Outside terrain coverage — ground collision disabled until the drone returns"
            );
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
        //    IMPACT_DURATION_TICKS so the EKF can reconcile the velocity change.
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

            // Trigger impact deceleration if hitting ground with significant downward velocity.
            // Deceleration is spread over IMPACT_DURATION_TICKS to give the EKF
            // a physical impulse it can fuse (instead of an instantaneous velocity
            // discontinuity with no corresponding acceleration).
            if velocity_before_clamp[2] > IMPACT_VEL_THRESHOLD
                && self.ground_impact_ticks_remaining == 0
            {
                let impact_dt = IMPACT_DURATION_TICKS as f64 * dt;
                // Deceleration needed to bring velocity to zero over impact_dt,
                // in NED. vz>0 means downward, so this is negative-Z (upward).
                let decel_ned = nalgebra::Vector3::new(
                    -velocity_before_clamp[0] / impact_dt,
                    -velocity_before_clamp[1] / impact_dt,
                    -velocity_before_clamp[2] / impact_dt,
                );
                // The accelerometer measures specific force = a - g, and it does
                // so in the BODY frame. `decel_ned` is in NED, so it must be
                // rotated by the impact attitude before being reported — a yawed
                // drone otherwise reports its lateral impact on the wrong body
                // axis and the EKF integrates a phantom sideways acceleration.
                // Rotating the whole (a - g) vector reduces to the old
                // [decel_x, decel_y, decel_z - g] when the drone is level.
                let gravity = self.config.physics.gravity;
                let specific_force_ned =
                    decel_ned - nalgebra::Vector3::new(0.0, 0.0, gravity);
                let specific_force_body =
                    state.quadrotor.quaternion.inverse() * specific_force_ned;
                self.ground_impact_accel = [
                    specific_force_body.x,
                    specific_force_body.y,
                    specific_force_body.z,
                ];
                self.ground_impact_ticks_remaining = IMPACT_DURATION_TICKS;
                debug!(
                    vz = velocity_before_clamp[2],
                    impact_ms = (impact_dt * 1000.0) as u32,
                    decel_z = decel_ned.z,
                    "Ground impact: generating deceleration impulse"
                );
            }

            if state.quadrotor.velocity[2] > 0.0 {
                state.quadrotor.velocity[2] = 0.0;
            }

            state.quadrotor.velocity[0] *= 0.9;
            state.quadrotor.velocity[1] *= 0.9;

            state.quadrotor.angular_velocity[0] = 0.0;
            state.quadrotor.angular_velocity[1] = 0.0;
            state.quadrotor.angular_velocity[2] *= 0.95;

            let (_, _, yaw) = state.quadrotor.quaternion.euler_angles();
            state.quadrotor.quaternion = nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw);
        }

        // Decrement impact counter (ticks down whether on ground or not)
        if self.ground_impact_ticks_remaining > 0 {
            self.ground_impact_ticks_remaining -= 1;
        }

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

        // Update simulation time
        state.sim_time_us += (dt * 1_000_000.0) as u64;
    }

    /// Sample sensors and send HIL messages
    fn sample_and_send_sensors(&mut self, dt: f64) {
        let sim_time_us;

        // Compute sensor inputs from physics state
        let (accel_body, gyro_body, altitude_m, position_ned, velocity_ned, attitude) = {
            let state = self.state.read();
            sim_time_us = state.sim_time_us;

            // Get physics state
            let q = &state.quadrotor;

            // Get attitude from quaternion
            let attitude = q.quaternion;

            // Compute body-frame specific force (what accelerometer measures)
            // Specific force = all non-gravitational forces / mass
            // This is what the accelerometer actually measures
            let (thrust_body, _) = q.compute_motor_forces(&self.config.physics);
            let drag_body = q.compute_drag(&self.config.physics);
            let mut force_body = thrust_body + drag_body;

            // Ground contact accelerometer model:
            // - During impact deceleration: report the impact force so the EKF
            //   can reconcile the velocity change with a physical acceleration.
            // - At rest on ground: report [0, 0, -g] (gravity reaction only).
            if self.ground_impact_ticks_remaining > 0 {
                let mass = self.config.physics.mass_kg;
                force_body = nalgebra::Vector3::new(
                    self.ground_impact_accel[0] * mass,
                    self.ground_impact_accel[1] * mass,
                    self.ground_impact_accel[2] * mass,
                );
            } else if self.ground_contact_active {
                let gravity = self.config.physics.gravity;
                force_body =
                    nalgebra::Vector3::new(0.0, 0.0, -self.config.physics.mass_kg * gravity);
            }

            // Accelerometer reading = specific force = non-gravitational acceleration
            let accel_body = [
                force_body[0] / self.config.physics.mass_kg,
                force_body[1] / self.config.physics.mass_kg,
                force_body[2] / self.config.physics.mass_kg,
            ];

            let gyro_body = [
                q.angular_velocity[0],
                q.angular_velocity[1],
                q.angular_velocity[2],
            ];

            // Altitude is negative of NED down position, plus reference altitude
            let altitude_m = self.config.reference_alt - q.position[2];

            let position_ned = [q.position[0], q.position[1], q.position[2]];
            let velocity_ned = [q.velocity[0], q.velocity[1], q.velocity[2]];

            (
                accel_body,
                gyro_body,
                altitude_m,
                position_ned,
                velocity_ned,
                attitude,
            )
        };

        // Compute which sensors to update this tick
        let tick = self.total_ticks;
        let update_mag = tick % MAG_UPDATE_DIVIDER == 0;
        let update_baro = tick % BARO_UPDATE_DIVIDER == 0;

        // Sample sensors selectively to avoid jittery data on non-update ticks.
        // IMU always sampled at full rate; mag/baro only on their update ticks.
        let time_s = sim_time_us as f64 / 1_000_000.0;
        let (imu_reading, mag_reading, baro_reading, gps_reading) = {
            let mut state = self.state.write();

            // IMU always sampled at 400 Hz
            let imu = state.sensors.imu.sample(&accel_body, &gyro_body, dt);

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
                self.config.reference_lat,
                self.config.reference_lon,
                time_s,
            );

            (imu, mag, baro, gps)
        };

        // Cache the readings for non-update ticks
        if update_mag {
            self.last_mag = Some(mag_reading);
        }
        if update_baro {
            self.last_baro = Some(baro_reading);
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

        // Build and send HIL_SENSOR message
        let hil_sensor = self.build_hil_sensor(
            &imu_reading,
            &baro_reading,
            &mag_reading,
            sim_time_us,
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

        // Send HIL_GPS when sensor provides it (sensor handles rate limiting)
        if let Some(gps) = gps_reading {
            let hil_gps = self.build_hil_gps(&gps, sim_time_us);
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

        // gps.alt is AGL (height above launch point = -ned_down, no reference_alt).
        // HIL_GPS requires MSL in millimeters, so we add reference_alt here.
        // This is NOT double-counting: the GPS sensor deliberately omits reference_alt
        // so that the sensor library stays free of daemon-specific config.
        let alt_msl = gps.alt as f64 + self.config.reference_alt;

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
            // Window stats — carried verbatim from the last 5 s roll-up.
            tick_rate_hz: self.stats.tick_rate_hz,
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
    use nalgebra::{UnitQuaternion, Vector3};
    use std::collections::HashMap;
    use std::sync::Arc;
    use terrain::{BBox, ElevationMeta, TerrainCache, TileCoord, TileMeta, TILE_SIZE};

    const DT: f64 = 1.0 / 400.0;
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
        let sim = SimulationLoop::new(config, actuator_rx, config_rx, mav_tx);
        (
            sim,
            TestChannels {
                _actuator_tx: actuator_tx,
                _config_tx: config_tx,
                _mav_rx: mav_rx,
            },
        )
    }

    #[allow(dead_code)]
    struct TestChannels {
        _actuator_tx: Sender<ActuatorOutputs>,
        _config_tx: Sender<(PhysicsConfig, BatteryConfig, hitl_sensors::SensorsConfig)>,
        _mav_rx: Receiver<MavMessage>,
    }

    /// 3x3 tiles of flat terrain at `elevation_msl` around the reference point.
    fn flat_terrain(elevation_msl: f32) -> Arc<TerrainCache> {
        let center = TileCoord::from_lon_lat(TEST_LON, TEST_LAT, TEST_ZOOM);
        let mut tiles = HashMap::new();
        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                let coord = TileCoord {
                    x: (center.x as i32 + dx) as u32,
                    y: (center.y as i32 + dy) as u32,
                    z: TEST_ZOOM,
                };
                tiles.insert(coord, vec![elevation_msl; TILE_SIZE * TILE_SIZE]);
            }
        }

        let meta = TileMeta {
            schema_version: 1,
            provider: "test".to_string(),
            zoom: TEST_ZOOM,
            tile_size: TILE_SIZE as u32,
            bbox: BBox {
                west: -180.0,
                south: -85.0,
                east: 180.0,
                north: 85.0,
            },
            elevation: ElevationMeta {
                units: "meters".to_string(),
                datum: "test".to_string(),
            },
        };

        let cache = TerrainCache::new();
        assert!(cache.load_from_tiles(meta, tiles, TEST_LAT, TEST_LON, 0.0));
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

        sim.step_physics(DT);

        assert!(
            sim.ground_impact_ticks_remaining > 0,
            "a 2 m/s descent onto the ground must trigger the impact impulse"
        );

        // Expected: NED velocity rotated into the body frame, then divided by
        // the impulse window. Yaw +90 deg puts 3 m/s north onto body -Y.
        let impact_dt = IMPACT_DURATION_TICKS as f64 * DT;
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

        sim.step_physics(DT);

        let down = sim.state.read().quadrotor.position[2];
        assert!(
            down > 49.0,
            "drone must not be clamped to the origin datum when the ground \
             height is unknown; expected to stay near 50 m down, got {down}"
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
        sim.step_physics(DT);

        let down = sim.state.read().quadrotor.position[2];
        assert!(
            (down - ground_z).abs() < 1e-6,
            "inside coverage the drone must be clamped to the sampled ground \
             height {ground_z}, got {down}"
        );
    }

    /// Sensor ground mode latches on contact and only releases once the drone
    /// is clearly airborne, so the EKF sees clean [0,0,-g] through liftoff.
    #[test]
    fn ground_contact_latches_until_clearly_airborne() {
        let (mut sim, _ch) = test_loop(SimulationConfig::default());

        sim.state.write().quadrotor.position = [0.0, 0.0, 0.0];
        sim.step_physics(DT);
        assert!(sim.ground_contact_active, "resting on ground must latch on");

        // 5 cm up is inside the hysteresis band — still latched.
        {
            let mut state = sim.state.write();
            state.quadrotor.position = [0.0, 0.0, -0.05];
            state.quadrotor.velocity = [0.0, 0.0, 0.0];
        }
        sim.step_physics(DT);
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
        sim.step_physics(DT);
        assert!(
            !sim.ground_contact_active,
            "50 cm above ground must release the ground latch"
        );
    }
}
