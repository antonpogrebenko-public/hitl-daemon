//! Main simulation loop running at 400 Hz

use crate::state::{SimulationConfig, SimulationState};
use crossbeam_channel::{Receiver, Sender, TryRecvError};
use hitl_physics::{rk4_step, throttle_to_omega, PhysicsConfig};
use mavlink::ardupilotmega::{
    MavMessage, HIL_GPS_DATA, HIL_SENSOR_DATA, HilSensorUpdatedFlags,
};
use protocol::ActuatorOutputs;
use std::time::{Duration, Instant};
use tracing::{info, trace, warn};

/// Simulation statistics
#[derive(Debug, Clone, Default)]
pub struct SimulationStats {
    /// Total ticks executed
    pub total_ticks: u64,
    /// Actual tick rate (Hz)
    pub actual_tick_rate: f64,
    /// Average loop latency (microseconds)
    pub avg_latency_us: f64,
    /// Maximum loop latency (microseconds)
    pub max_latency_us: u64,
    /// HIL_SENSOR messages sent
    pub hil_sensor_count: u64,
    /// HIL_GPS messages sent
    pub hil_gps_count: u64,
    /// Actuator messages received
    pub actuator_count: u64,
    /// Sensor messages dropped (channel full)
    pub sensor_drops: u64,
}

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
    config_rx: Receiver<PhysicsConfig>,
    mav_tx: Sender<MavMessage>,
    stats: SimulationStats,
    /// Cached mag reading (only updated at MAG_UPDATE_DIVIDER rate)
    last_mag: Option<hitl_sensors::MagReading>,
    /// Cached baro reading (only updated at BARO_UPDATE_DIVIDER rate)
    last_baro: Option<hitl_sensors::BaroReading>,
}

impl SimulationLoop {
    /// Create a new simulation loop
    pub fn new(
        config: SimulationConfig,
        actuator_rx: Receiver<ActuatorOutputs>,
        config_rx: Receiver<PhysicsConfig>,
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
            last_mag: None,
            last_baro: None,
        }
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

        let stats_interval = Duration::from_secs(5);

        // Window-based stats — reset every interval so reported values reflect recent behaviour.
        let mut window_start = Instant::now();
        let mut window_ticks: u64 = 0;
        let mut window_latency_us: u64 = 0;
        let mut window_max_latency_us: u64 = 0;

        // Absolute scheduling: advance next_tick by one period each iteration so overruns
        // don't accumulate (a single 8s spike won't cause 8s of catch-up busy-looping).
        let mut next_tick = Instant::now();

        while self.state.is_running() {
            let tick_start = Instant::now();

            match self.config_rx.try_recv() {
                Ok(new_physics) => {
                    info!("Reconfiguring simulation");
                    self.config.physics = new_physics;
                    self.state.reconfigure();
                    self.last_mag = None;
                    self.last_baro = None;
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

            self.stats.total_ticks += 1;
            window_ticks += 1;

            let latency_us = tick_start.elapsed().as_micros() as u64;
            window_latency_us += latency_us;
            if latency_us > window_max_latency_us {
                window_max_latency_us = latency_us;
            }

            // Print per-window stats every 5 seconds
            if window_start.elapsed() >= stats_interval {
                let window_secs = window_start.elapsed().as_secs_f64();
                self.stats.actual_tick_rate = window_ticks as f64 / window_secs;
                self.stats.avg_latency_us = window_latency_us as f64 / window_ticks as f64;
                self.stats.max_latency_us = window_max_latency_us;

                self.print_stats();

                window_start = Instant::now();
                window_ticks = 0;
                window_latency_us = 0;
                window_max_latency_us = 0;
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
            self.state.set_motor_commands(actuator.motors);
            self.state.set_armed(actuator.is_armed());

            if actuator.is_hil_active() {
                trace!(
                    motors = ?actuator.motors,
                    armed = actuator.is_armed(),
                    "Received actuator commands"
                );
            }
        }
    }

    /// Step the physics simulation
    fn step_physics(&mut self, dt: f64) {
        let mut state = self.state.write();

        // Check if motors are active (any throttle > 1%)
        let motors_active = state.motor_commands.iter().any(|&c| c > 0.01);

        if motors_active {
            // Convert motor commands (0-1) to angular velocities
            let motor_omegas: [f64; 4] = std::array::from_fn(|i| {
                throttle_to_omega(state.motor_commands[i] as f64)
            });

            // Step physics using RK4 integration
            state.quadrotor = rk4_step(&state.quadrotor, &self.config.physics, motor_omegas, dt);
        }
        // When disarmed (motors off), keep drone stationary for gyro calibration

        // Ground contact constraint: in NED, Z >= 0 means at or below ground.
        // Clamp position, kill downward velocity, apply friction.
        // Uses >= so that damping applies even when sitting exactly at Z=0.
        if state.quadrotor.position[2] >= 0.0 {
            state.quadrotor.position[2] = 0.0;

            // Kill downward velocity (positive Z in NED = moving down)
            if state.quadrotor.velocity[2] > 0.0 {
                state.quadrotor.velocity[2] = 0.0;
            }

            // Ground friction: dampen horizontal velocity and angular rates.
            // Strong roll/pitch damping prevents tipping over on the ground.
            state.quadrotor.velocity[0] *= 0.9;
            state.quadrotor.velocity[1] *= 0.9;
            state.quadrotor.angular_velocity[0] *= 0.8;
            state.quadrotor.angular_velocity[1] *= 0.8;
            state.quadrotor.angular_velocity[2] *= 0.9;
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

            // Ground contact: when on ground (Z >= 0) and not ascending significantly,
            // the accelerometer measures only the ground normal force (gravity reaction),
            // independent of motor thrust. Replace (not add) the force to avoid
            // double-counting thrust + gravity when motors are running on the ground.
            let on_ground = q.position[2] >= -0.01 && q.velocity[2] >= -0.1;
            if on_ground {
                let gravity = self.config.physics.gravity;
                let gravity_force_ned = nalgebra::Vector3::new(0.0, 0.0, -self.config.physics.mass_kg * gravity);
                force_body = q.quaternion.inverse() * gravity_force_ned;
            }

            // Accelerometer reading = specific force = non-gravitational acceleration
            let accel_body = [
                force_body[0] / self.config.physics.mass_kg,
                force_body[1] / self.config.physics.mass_kg,
                force_body[2] / self.config.physics.mass_kg,
            ];

            let gyro_body = [q.angular_velocity[0], q.angular_velocity[1], q.angular_velocity[2]];

            // Altitude is negative of NED down position, plus reference altitude
            let altitude_m = self.config.reference_alt - q.position[2];

            let position_ned = [q.position[0], q.position[1], q.position[2]];
            let velocity_ned = [q.velocity[0], q.velocity[1], q.velocity[2]];

            (accel_body, gyro_body, altitude_m, position_ned, velocity_ned, attitude)
        };

        // Compute which sensors to update this tick
        let tick = self.stats.total_ticks;
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
                self.last_mag.unwrap_or_else(|| state.sensors.mag.sample(&attitude))
            };

            // Baro: only sample on update ticks, otherwise use cached value
            let baro = if update_baro {
                state.sensors.baro.sample(altitude_m)
            } else {
                self.last_baro.unwrap_or_else(|| state.sensors.baro.sample(altitude_m))
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
        let first_tick = self.stats.total_ticks == 0;
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
        let hil_sensor = self.build_hil_sensor(&imu_reading, &baro_reading, &mag_reading, sim_time_us, fields_updated);
        match self.mav_tx.try_send(MavMessage::HIL_SENSOR(hil_sensor)) {
            Ok(()) => { self.stats.hil_sensor_count += 1; }
            Err(crossbeam_channel::TrySendError::Full(_)) => {
                self.stats.sensor_drops += 1;
                if self.stats.sensor_drops == 1 || self.stats.sensor_drops % 1000 == 0 {
                    warn!(drops = self.stats.sensor_drops, "Sensor message channel full — FC not consuming");
                }
            }
            Err(crossbeam_channel::TrySendError::Disconnected(_)) => {}
        }

        // Send HIL_GPS when sensor provides it (sensor handles rate limiting)
        if let Some(gps) = gps_reading {
            let hil_gps = self.build_hil_gps(&gps, sim_time_us);
            match self.mav_tx.try_send(MavMessage::HIL_GPS(hil_gps)) {
                Ok(()) => { self.stats.hil_gps_count += 1; }
                Err(crossbeam_channel::TrySendError::Full(_)) => {
                    self.stats.sensor_drops += 1;
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
            diff_pressure: 0.0, // No airspeed sensor
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

        // GPS altitude needs to be MSL, not AGL
        let alt_msl = gps.alt as f64 + self.config.reference_alt;

        HIL_GPS_DATA {
            time_usec: time_us,
            lat: (gps.lat * 1e7) as i32,
            lon: (gps.lon * 1e7) as i32,
            alt: (alt_msl * 1000.0) as i32, // mm MSL
            eph: (gps.hdop * 100.0) as u16,        // cm (using HDOP as horizontal accuracy proxy)
            epv: 200,                               // cm (fixed vertical accuracy estimate)
            vel: (ground_speed * 100.0) as u16,    // cm/s
            vn: (gps.vel_n * 100.0) as i16,        // cm/s
            ve: (gps.vel_e * 100.0) as i16,        // cm/s
            vd: (gps.vel_d * 100.0) as i16,        // cm/s
            cog: (cog_positive * 100.0) as u16,    // cdeg
            fix_type: 3,                            // 3D fix
            satellites_visible: gps.satellites,
        }
    }

    /// Print current simulation statistics
    fn print_stats(&self) {
        let state = self.state.read();
        info!(
            tick_rate = format!("{:.1} Hz", self.stats.actual_tick_rate),
            avg_latency = format!("{:.1} us", self.stats.avg_latency_us),
            max_latency = format!("{} us", self.stats.max_latency_us),
            hil_sensor = self.stats.hil_sensor_count,
            hil_gps = self.stats.hil_gps_count,
            actuators = self.stats.actuator_count,
            sim_time = format!("{:.1} s", state.sim_time_us as f64 / 1_000_000.0),
            pos_ned = format!("[{:.2}, {:.2}, {:.2}]",
                state.quadrotor.position[0],
                state.quadrotor.position[1],
                state.quadrotor.position[2]
            ),
            "Simulation stats"
        );
    }

    /// Get current statistics
    pub fn stats(&self) -> SimulationStats {
        self.stats.clone()
    }
}
