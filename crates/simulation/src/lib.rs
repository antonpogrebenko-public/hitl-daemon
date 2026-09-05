//! HITL Simulation Loop
//!
//! Integrates physics engine and sensor models to run a complete
//! hardware-in-the-loop simulation at 400 Hz.

pub mod loop_runner;
pub mod origin;
mod realtime;
pub mod state;
pub mod timebase;

pub use loop_runner::{SimulationLoop, SimulationStats};
pub use origin::{OriginValues, SharedOrigin};
pub use state::{SimulationConfig, SimulationState};
pub use terrain::TerrainCache;
