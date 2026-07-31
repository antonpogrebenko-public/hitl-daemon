//! Shared simulation state between threads

use hitl_physics::{BatteryConfig, BatteryState, PhysicsConfig, QuadrotorState};
use hitl_sensors::{Sensors, SensorsConfig};
use parking_lot::RwLock;
use std::sync::Arc;
use terrain::TerrainCache;

/// Configuration for the simulation
#[derive(Clone)]
pub struct SimulationConfig {
    /// Physics configuration
    pub physics: PhysicsConfig,
    /// Battery configuration
    pub battery: BatteryConfig,
    /// Sensor configuration
    pub sensors: SensorsConfig,
    /// Reference latitude for GPS (degrees)
    pub reference_lat: f64,
    /// Reference longitude for GPS (degrees)
    pub reference_lon: f64,
    /// Reference altitude MSL (meters)
    pub reference_alt: f64,
    /// Simulation tick rate (Hz)
    pub tick_rate_hz: u32,
    /// GPS update rate (Hz)
    pub gps_rate_hz: u32,
    /// Terrain cache for ground collision (optional)
    pub terrain: Option<Arc<TerrainCache>>,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            physics: PhysicsConfig::default(),
            battery: BatteryConfig::default(),
            sensors: SensorsConfig::default(),
            // Default to Boulder, CO
            reference_lat: 40.015,
            reference_lon: -105.2705,
            reference_alt: 1655.0,
            tick_rate_hz: 400,
            gps_rate_hz: 10,
            terrain: None,
        }
    }
}

impl std::fmt::Debug for SimulationConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SimulationConfig")
            .field("physics", &self.physics)
            .field("battery", &self.battery)
            .field("sensors", &self.sensors)
            .field("reference_lat", &self.reference_lat)
            .field("reference_lon", &self.reference_lon)
            .field("reference_alt", &self.reference_alt)
            .field("tick_rate_hz", &self.tick_rate_hz)
            .field("gps_rate_hz", &self.gps_rate_hz)
            .field("terrain", &self.terrain.as_ref().map(|t| t.is_loaded()))
            .finish()
    }
}

/// Core simulation state
pub struct SimulationStateInner {
    /// Quadrotor physics state
    pub quadrotor: QuadrotorState,
    /// Battery discharge state
    pub battery: BatteryState,
    /// Sensor suite
    pub sensors: Sensors,
    /// Simulation time in microseconds
    pub sim_time_us: u64,
    /// Current motor commands (normalized 0-1)
    pub motor_commands: [f32; 4],
    /// Whether simulation is running
    pub running: bool,
    /// Armed state from flight controller
    pub armed: bool,
    /// Flight mode from flight controller (PX4 custom_mode)
    pub flight_mode: u8,
    /// Landed state reported by the flight controller's land detector, as
    /// MAV_LANDED_STATE (0=undefined, 1=on ground, 2=in air, 3=takeoff,
    /// 4=landing). The simulation never infers this — it is PX4's own verdict,
    /// which is what makes it useful: comparing it against the sim's ground
    /// contact reveals when synthesized sensors have misled the EKF.
    pub landed_state: u8,
    /// Monotonic count of HEARTBEATs received from the FC, incremented on
    /// every one. Zero in --sim-only mode or before the first HEARTBEAT
    /// arrives.
    ///
    /// Nothing ever resets this — not `reset()`, not `reconfigure()`, and
    /// deliberately not a preflight-triggered reboot. `heartbeat_count > 0`
    /// is therefore a fact that stays true for the daemon's whole lifetime
    /// once the first HEARTBEAT lands, so a concurrent reader (a second
    /// browser tab, a page reload) always sees an accurate "is an FC
    /// physically connected" signal — even while a reboot-wait is in
    /// progress elsewhere. A destructive clear would make `connected` read
    /// false mid-reboot, which the preflight gate interprets as "no FC to
    /// misconfigure, skip the check" and would silently bypass the gate.
    /// The reboot-wait instead works by comparing counter *snapshots*: the
    /// caller records the count before sending the reboot and watches for
    /// it to increase, mutating no shared state.
    pub heartbeat_count: u64,
    /// Cached from the latest HEARTBEAT's `base_mode`: whether PX4 reports
    /// MAV_MODE_FLAG_HIL_ENABLED. Used by the preflight gate ahead of
    /// ConfigureBuild — HITL cannot function with this false.
    pub hitl_enabled: bool,
    /// Cached from the latest HEARTBEAT's `mavtype`: whether PX4 reports
    /// MAV_TYPE_QUADROTOR (any quad airframe variant, not tied to a specific
    /// SYS_AUTOSTART id).
    pub is_quadrotor: bool,
}

impl SimulationStateInner {
    /// Create new simulation state at rest on ground
    pub fn new(config: &SimulationConfig) -> Self {
        Self {
            quadrotor: QuadrotorState::default(),
            battery: BatteryState::fully_charged(&config.battery),
            sensors: Sensors::with_config(config.sensors.clone()),
            sim_time_us: 0,
            motor_commands: [0.0; 4],
            running: true,
            armed: false,
            flight_mode: 0,
            landed_state: 0,
            heartbeat_count: 0,
            hitl_enabled: false,
            is_quadrotor: false,
        }
    }

    /// Reset simulation to initial state
    pub fn reset(&mut self, config: &SimulationConfig) {
        self.quadrotor = QuadrotorState::default();
        self.battery = BatteryState::fully_charged(&config.battery);
        self.sensors = Sensors::with_config(config.sensors.clone());
        self.sim_time_us = 0;
        self.motor_commands = [0.0; 4];
        self.armed = false;
        self.flight_mode = 0;
        self.landed_state = 0;
    }
}

/// Thread-safe wrapper for simulation state
#[derive(Clone)]
pub struct SimulationState {
    inner: Arc<RwLock<SimulationStateInner>>,
    /// Swappable config: written by `reconfigure`, read by all callers via `config()`.
    config: Arc<RwLock<SimulationConfig>>,
}

impl SimulationState {
    /// Create new thread-safe simulation state
    pub fn new(config: SimulationConfig) -> Self {
        let inner = SimulationStateInner::new(&config);
        Self {
            inner: Arc::new(RwLock::new(inner)),
            config: Arc::new(RwLock::new(config)),
        }
    }

    /// Get read access to inner state
    pub fn read(&self) -> parking_lot::RwLockReadGuard<'_, SimulationStateInner> {
        self.inner.read()
    }

    /// Get write access to inner state
    pub fn write(&self) -> parking_lot::RwLockWriteGuard<'_, SimulationStateInner> {
        self.inner.write()
    }

    /// Get a snapshot clone of the current simulation configuration.
    ///
    /// Returns a clone so callers hold no lock across the slow path. For the
    /// hot 400 Hz loop, `SimulationLoop` owns its own `self.config` copy and
    /// never calls this — only the WebSocket broadcast path (30 Hz) uses it.
    pub fn config(&self) -> SimulationConfig {
        self.config.read().clone()
    }

    /// Update motor commands from actuator outputs
    pub fn set_motor_commands(&self, motors: [f32; 4]) {
        self.inner.write().motor_commands = motors;
    }

    /// Update armed state from flight controller
    pub fn set_armed(&self, armed: bool) {
        self.inner.write().armed = armed;
    }

    /// Update flight mode from flight controller
    pub fn set_flight_mode(&self, mode: u8) {
        self.inner.write().flight_mode = mode;
    }

    /// Get current armed state
    pub fn is_armed(&self) -> bool {
        self.inner.read().armed
    }

    /// Get current flight mode
    pub fn flight_mode(&self) -> u8 {
        self.inner.read().flight_mode
    }

    /// Update the landed state reported by the flight controller
    /// (MAV_LANDED_STATE from EXTENDED_SYS_STATE).
    pub fn set_landed_state(&self, landed_state: u8) {
        self.inner.write().landed_state = landed_state;
    }

    /// Get the flight controller's current landed state (MAV_LANDED_STATE).
    pub fn landed_state(&self) -> u8 {
        self.inner.read().landed_state
    }

    /// Record a HEARTBEAT: bump the monotonic counter and store the latest
    /// HITL/quadrotor flags.
    pub fn set_heartbeat_status(&self, hitl_enabled: bool, is_quadrotor: bool) {
        let mut inner = self.inner.write();
        inner.heartbeat_count = inner.heartbeat_count.saturating_add(1);
        inner.hitl_enabled = hitl_enabled;
        inner.is_quadrotor = is_quadrotor;
    }

    /// Read the cached HEARTBEAT status: `(heartbeat_count, hitl_enabled,
    /// is_quadrotor)`. `heartbeat_count == 0` means no HEARTBEAT has ever
    /// been received. The count only ever increases, so callers watching for
    /// a post-reboot reconnect compare it against a snapshot they took
    /// themselves rather than clearing shared state.
    pub fn heartbeat_status(&self) -> (u64, bool, bool) {
        let inner = self.inner.read();
        (inner.heartbeat_count, inner.hitl_enabled, inner.is_quadrotor)
    }

    /// Get current simulation time in microseconds
    pub fn sim_time_us(&self) -> u64 {
        self.inner.read().sim_time_us
    }

    /// Check if simulation is running
    pub fn is_running(&self) -> bool {
        self.inner.read().running
    }

    /// Stop the simulation
    pub fn stop(&self) {
        self.inner.write().running = false;
    }

    /// Reset simulation state using the current live config.
    pub fn reset(&self) {
        let cfg = self.config.read().clone();
        self.inner.write().reset(&cfg);
    }

    /// Reconfigure: store the new config as the live config, then reset inner
    /// state so battery capacity, sensor noise, etc. reflect the new build.
    pub fn reconfigure(&self, new_config: SimulationConfig) {
        // First, store the new config so `reset` (and all readers) see the
        // live values. Then reset inner state using the freshly-stored config.
        *self.config.write() = new_config;
        let cfg = self.config.read().clone();
        self.inner.write().reset(&cfg);
    }

    /// Recharge the battery to full
    pub fn recharge_battery(&self) {
        self.inner.write().battery.recharge();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hitl_physics::BatteryConfig;

    /// After `reconfigure` with a different battery spec, the inner battery
    /// state must reflect the NEW capacity and cell count — not the stale ones.
    #[test]
    fn reconfigure_updates_battery_state() {
        // Build the initial state with a 1000 mAh, 4-cell battery.
        let initial_battery = BatteryConfig {
            capacity_mah: 1000.0,
            cell_count: 4,
            ..BatteryConfig::default()
        };
        let initial_config = SimulationConfig {
            battery: initial_battery,
            ..SimulationConfig::default()
        };
        let sim_state = SimulationState::new(initial_config);

        // Verify initial values.
        {
            let inner = sim_state.read();
            // A fully-charged 4S pack is ~4.2 V × 4 = 16.8 V.
            assert!(
                inner.battery.voltage() > 16.0,
                "Initial voltage should be >16 V (4S full), got {}",
                inner.battery.voltage()
            );
        }

        // Reconfigure with a larger 2000 mAh, 6-cell battery.
        let new_battery = BatteryConfig {
            capacity_mah: 2000.0,
            cell_count: 6,
            ..BatteryConfig::default()
        };
        let new_config = SimulationConfig {
            battery: new_battery,
            ..SimulationConfig::default()
        };
        sim_state.reconfigure(new_config);

        // After reconfigure the battery state must reflect 6S, not the stale 4S.
        {
            let inner = sim_state.read();
            // A fully-charged 6S pack is ~4.2 V × 6 = 25.2 V.
            assert!(
                inner.battery.voltage() > 24.0,
                "After reconfigure, voltage should be >24 V (6S full), got {}",
                inner.battery.voltage()
            );
        }

        // The live config stored inside SimulationState must also reflect the
        // new cell count and capacity.
        let live = sim_state.config();
        assert_eq!(live.battery.cell_count, 6);
        assert!((live.battery.capacity_mah - 2000.0).abs() < 0.1);
    }

    #[test]
    fn heartbeat_status_starts_unseen() {
        let state = SimulationState::new(SimulationConfig::default());
        assert_eq!(state.heartbeat_status(), (0, false, false));
    }

    #[test]
    fn set_heartbeat_status_marks_seen_and_stores_flags() {
        let state = SimulationState::new(SimulationConfig::default());
        state.set_heartbeat_status(true, false);
        assert_eq!(state.heartbeat_status(), (1, true, false));
    }

    #[test]
    fn heartbeat_count_increases_monotonically_and_keeps_latest_flags() {
        // The counter is the whole reason the preflight reboot-wait needs no
        // destructive clear: it only ever goes up, so "an FC is connected"
        // stays true for a concurrent reader even mid-reboot, while the flags
        // always describe the most recent HEARTBEAT.
        let state = SimulationState::new(SimulationConfig::default());
        state.set_heartbeat_status(false, false);
        assert_eq!(state.heartbeat_status(), (1, false, false));
        state.set_heartbeat_status(true, true);
        assert_eq!(state.heartbeat_status(), (2, true, true));
    }

    #[test]
    fn reset_does_not_clear_heartbeat_status() {
        // Reconfiguring the sim (new build) does not mean the FC's boot-time
        // HITL/frame config changed, so reset() must leave the counter and
        // the cached flags alone.
        let state = SimulationState::new(SimulationConfig::default());
        state.set_heartbeat_status(true, true);
        state.reset();
        assert_eq!(state.heartbeat_status(), (1, true, true));
    }
}
