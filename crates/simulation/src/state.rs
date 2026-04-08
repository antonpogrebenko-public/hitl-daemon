//! Shared simulation state between threads

use hitl_physics::{PhysicsConfig, QuadrotorState};
use hitl_sensors::{Sensors, SensorsConfig};
use parking_lot::RwLock;
use std::sync::Arc;

/// Configuration for the simulation
#[derive(Debug, Clone)]
pub struct SimulationConfig {
    /// Physics configuration
    pub physics: PhysicsConfig,
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
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            physics: PhysicsConfig::default(),
            sensors: SensorsConfig::default(),
            // Default to Boulder, CO
            reference_lat: 40.015,
            reference_lon: -105.2705,
            reference_alt: 1655.0,
            tick_rate_hz: 400,
            gps_rate_hz: 5,
        }
    }
}

/// Core simulation state
pub struct SimulationStateInner {
    /// Quadrotor physics state
    pub quadrotor: QuadrotorState,
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
}

impl SimulationStateInner {
    /// Create new simulation state at rest on ground
    pub fn new(config: &SimulationConfig) -> Self {
        Self {
            quadrotor: QuadrotorState::default(),
            sensors: Sensors::with_config(config.sensors.clone()),
            sim_time_us: 0,
            motor_commands: [0.0; 4],
            running: true,
            armed: false,
            flight_mode: 0,
        }
    }

    /// Reset simulation to initial state
    pub fn reset(&mut self, config: &SimulationConfig) {
        self.quadrotor = QuadrotorState::default();
        self.sensors = Sensors::with_config(config.sensors.clone());
        self.sim_time_us = 0;
        self.motor_commands = [0.0; 4];
        self.armed = false;
        self.flight_mode = 0;
    }
}

/// Thread-safe wrapper for simulation state
#[derive(Clone)]
pub struct SimulationState {
    inner: Arc<RwLock<SimulationStateInner>>,
    config: Arc<SimulationConfig>,
}

impl SimulationState {
    /// Create new thread-safe simulation state
    pub fn new(config: SimulationConfig) -> Self {
        let inner = SimulationStateInner::new(&config);
        Self {
            inner: Arc::new(RwLock::new(inner)),
            config: Arc::new(config),
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

    /// Get simulation configuration
    pub fn config(&self) -> &SimulationConfig {
        &self.config
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

    /// Reset simulation state
    pub fn reset(&self) {
        self.inner.write().reset(&self.config);
    }
}
