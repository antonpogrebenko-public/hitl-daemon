//! Shared simulation state between threads

use hitl_physics::{BatteryConfig, BatteryState, PhysicsConfig, QuadrotorState};
use hitl_sensors::{Sensors, SensorsConfig};
use parking_lot::RwLock;
use std::sync::atomic::{AtomicU64, Ordering};
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
    /// Simulation time in microseconds — the mirror of
    /// [`crate::timebase::Timebase::sim_time_us`], written once per loop
    /// iteration.
    ///
    /// Microseconds of WALL time since the loop epoch, NOT a count of physics
    /// steps. It is monotone non-decreasing for the life of the session:
    /// neither [`SimulationStateInner::reset`] nor
    /// [`SimulationState::reconfigure`] rewinds it.
    ///
    /// | Consumer | Reads | Why |
    /// |---|---|---|
    /// | `HIL_SENSOR.time_usec` (`loop_runner::build_hil_sensor`) | this | Inert in PX4 on NuttX — `MavlinkReceiver::handle_message_hil_sensor` restamps with `hrt_absolute_time()` — but it must stay monotone and wall-referenced. |
    /// | `HIL_GPS.time_usec` (`loop_runner::build_hil_gps`) | `integrated_us = sim − discarded` | Deliberate divergence. The ULog cross-check `(hrt elapsed) − (time_utc_usec elapsed)` then equals the daemon's discarded time exactly: 0 on a healthy run, the fault magnitude on a bad one. |
    /// | `hitl-sensors` GPS rate limiter and delay buffer | this | Neither `last_update_time` nor `last_output_time` rewinds, so a backward jump silences GPS permanently and PX4 reports "ekf2 missing data". |
    /// | Browser `StateUpdate.timestamp_us` (`src/main.rs`) | this | Session-monotone, so an RTL reset no longer rewinds the viewer's clock. |
    /// | TUI `sim_time_s` | this | Reads as a stopwatch instead of understating elapsed time by the drift. |
    ///
    /// Monotonicity is load-bearing rather than tidy. Today's rewind was benign
    /// only by accident: `reset()` rebuilds `Sensors::with_config`, and
    /// `hitl-sensors/src/gps.rs` initialises both limiter fields to `-1000.0`,
    /// so the limiter re-locks immediately against the larger time. A rewind
    /// with *retained* sensor state would silence GPS for the session.
    pub sim_time_us: u64,
    /// Current motor commands (normalized 0-1)
    pub motor_commands: [f32; 4],
    /// Whether simulation is running
    pub running: bool,
    /// Armed state from flight controller
    pub armed: bool,
    /// The stall guard is holding the motors at zero because the flight
    /// controller's motor commands went stale; PX4's armed state is unchanged.
    /// Cleared by the next motor command and by `reset()`.
    pub board_stall: bool,
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
    /// Terminal voltage the motors saw on the last physics tick (V). Equals the
    /// open-circuit voltage whenever no current is drawn.
    pub battery_terminal_voltage: f64,
    /// Total motor current on the last physics tick (A). Zero while idle or
    /// depleted.
    pub battery_current_a: f64,
}

impl SimulationStateInner {
    /// Create new simulation state at rest on ground
    pub fn new(config: &SimulationConfig) -> Self {
        let battery = BatteryState::fully_charged(&config.battery);
        let battery_terminal_voltage = battery.voltage();
        Self {
            quadrotor: QuadrotorState::default(),
            battery,
            battery_terminal_voltage,
            battery_current_a: 0.0,
            sensors: Sensors::with_config(config.sensors.clone()),
            sim_time_us: 0,
            motor_commands: [0.0; 4],
            running: true,
            armed: false,
            board_stall: false,
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
        self.battery_terminal_voltage = self.battery.voltage();
        self.battery_current_a = 0.0;
        self.sensors = Sensors::with_config(config.sensors.clone());
        // `sim_time_us` is deliberately NOT cleared. It is the session's
        // timebase, shared with the GPS rate limiter, PX4 and the browser, and
        // it must be monotone for the life of the loop — see the field's own
        // documentation. Zeroing it here also let a session that lost wall time
        // hide that by changing the build.
        self.motor_commands = [0.0; 4];
        self.armed = false;
        self.board_stall = false;
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
    /// How many configs `reconfigure` has applied. ConfigureBuild reads it
    /// before handing a build to the simulation loop and waits for it to move,
    /// so the EKF2 restart never runs against the previous build.
    config_generation: Arc<AtomicU64>,
}

impl SimulationState {
    /// Create new thread-safe simulation state
    pub fn new(config: SimulationConfig) -> Self {
        let inner = SimulationStateInner::new(&config);
        Self {
            inner: Arc::new(RwLock::new(inner)),
            config: Arc::new(RwLock::new(config)),
            config_generation: Arc::new(AtomicU64::new(0)),
        }
    }

    /// The number of configs applied so far; 0 until the first `reconfigure`.
    /// A reader that sees a new value also sees the vehicle that `reconfigure`
    /// reset.
    pub fn config_generation(&self) -> u64 {
        self.config_generation.load(Ordering::Acquire)
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

    /// The flight-controller connection was declared lost (main.rs reconnect
    /// task): report disarmed, end any stall and stop the motors, in one write.
    /// The stall guard keeps PX4's armed state, so losing the FC is what
    /// reports the disarm that ends a flight in the browser.
    pub fn mark_fc_lost(&self) {
        let mut inner = self.inner.write();
        inner.armed = false;
        inner.board_stall = false;
        inner.motor_commands = [0.0; 4];
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
        (
            inner.heartbeat_count,
            inner.hitl_enabled,
            inner.is_quadrotor,
        )
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
        // Bumped last, so the reset vehicle is in place before anyone sees it.
        self.config_generation.fetch_add(1, Ordering::Release);
    }

    /// Recharge the battery to full
    pub fn recharge_battery(&self) {
        let mut inner = self.inner.write();
        inner.battery.recharge();
        // Until the next tick there is no draw; a loaded value from the drained
        // pack beside the full pack's open-circuit voltage would read as sag.
        inner.battery_terminal_voltage = inner.battery.voltage();
        inner.battery_current_a = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hitl_physics::BatteryConfig;

    #[test]
    fn mark_fc_lost_disarms_and_clears_the_stall() {
        let sim = SimulationState::new(SimulationConfig::default());
        {
            let mut s = sim.write();
            s.armed = true;
            s.board_stall = true;
            s.motor_commands = [0.5; 4];
        }
        sim.mark_fc_lost();
        let s = sim.read();
        assert!(!s.armed);
        assert!(!s.board_stall);
        assert_eq!(s.motor_commands, [0.0; 4]);
    }

    #[test]
    fn mark_fc_lost_disarms_without_a_stall_in_progress() {
        let sim = SimulationState::new(SimulationConfig::default());
        sim.set_armed(true);
        sim.mark_fc_lost();
        assert!(!sim.read().armed);
    }

    /// A recharge between ticks must not leave the drained pack's loaded
    /// voltage beside the full pack's open-circuit voltage: a client would read
    /// the gap as sag.
    #[test]
    fn recharge_resets_load_fields() {
        let state = SimulationState::new(SimulationConfig::default());
        {
            let mut inner = state.write();
            inner.battery.discharge(10_000.0, 1_000.0);
            inner.battery_terminal_voltage = inner.battery.voltage();
            inner.battery_current_a = 40.0;
        }
        state.recharge_battery();
        let inner = state.read();
        assert_eq!(inner.battery_current_a, 0.0);
        assert_eq!(inner.battery_terminal_voltage, inner.battery.voltage());
    }

    /// The first frame after a rebuild must not carry the previous build's load,
    /// and the terminal voltage must match the NEW pack, not the old one.
    #[test]
    fn reconfigure_resets_load_fields() {
        let state = SimulationState::new(SimulationConfig::default());
        let old_ocv = {
            let mut inner = state.write();
            inner.battery_terminal_voltage = 99.0;
            inner.battery_current_a = 1.0;
            inner.battery.voltage()
        };
        let default_cells = SimulationConfig::default().battery.cell_count;
        let new_config = SimulationConfig {
            battery: BatteryConfig {
                cell_count: default_cells + 2,
                ..BatteryConfig::default()
            },
            ..SimulationConfig::default()
        };
        state.reconfigure(new_config);
        let inner = state.read();
        assert_ne!(inner.battery.voltage(), old_ocv);
        assert_eq!(inner.battery_current_a, 0.0);
        assert_eq!(inner.battery_terminal_voltage, inner.battery.voltage());
    }

    /// A freshly built state reports no load and a terminal voltage equal to
    /// the open-circuit voltage.
    #[test]
    fn new_state_starts_with_no_load() {
        let inner = SimulationStateInner::new(&SimulationConfig::default());
        assert_eq!(inner.battery_current_a, 0.0);
        assert!(inner.battery.voltage() > 0.0);
        assert_eq!(inner.battery_terminal_voltage, inner.battery.voltage());
    }

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

    /// `reconfigure` must not rewind the simulation clock.
    ///
    /// `sim_time_us` is the timebase every downstream consumer shares: PX4
    /// reads it as `HIL_SENSOR.time_usec`, `hitl-sensors`' GPS rate limiter
    /// holds `last_update_time` / `last_output_time` in it, and the browser
    /// stamps `StateUpdate.timestamp_us` from it. A rewind silences the GPS
    /// limiter for the rest of the session — "ekf2 missing data" on the FC —
    /// and it lets a session that lost 4.9 s of wall time hide the fact by
    /// changing the build.
    ///
    /// Today's rewind is benign only by accident: `reset()` rebuilds
    /// `Sensors::with_config`, and `hitl-sensors/src/gps.rs` initialises both
    /// limiter fields to `-1000.0`, so the limiter re-locks against the larger
    /// time. A rewind with *retained* sensor state would not recover.
    #[test]
    fn reconfigure_does_not_rewind_the_simulation_clock() {
        let state = SimulationState::new(SimulationConfig::default());
        // 63 s: the length of the 2026-09-05 session.
        state.write().sim_time_us = 63_000_000;

        state.reconfigure(SimulationConfig::default());

        assert!(
            state.sim_time_us() >= 63_000_000,
            "the simulation clock must be monotone across a reconfigure; \
             rewinding it to {} silences the GPS rate limiter and erases the \
             session's pacing history",
            state.sim_time_us()
        );
    }

    /// The same clock, through the plain `reset()` the WebSocket RTL command
    /// reaches (`src/main.rs`). Monotonicity is a property of the session, not
    /// of one entry point.
    #[test]
    fn reset_does_not_rewind_the_simulation_clock() {
        let state = SimulationState::new(SimulationConfig::default());
        state.write().sim_time_us = 63_000_000;

        state.reset();

        assert!(state.sim_time_us() >= 63_000_000);
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

    #[test]
    fn reconfigure_bumps_the_config_generation() {
        let s = SimulationState::new(SimulationConfig::default());
        assert_eq!(s.config_generation(), 0);
        s.reconfigure(SimulationConfig::default());
        assert_eq!(s.config_generation(), 1);
    }

    #[test]
    fn reset_leaves_the_config_generation_alone() {
        let s = SimulationState::new(SimulationConfig::default());
        s.reset();
        assert_eq!(s.config_generation(), 0);
    }
}
