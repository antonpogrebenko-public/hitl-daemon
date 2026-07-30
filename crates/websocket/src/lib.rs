//! WebSocket server for HITL daemon browser communication
//!
//! This crate provides a WebSocket server that streams telemetry data to browser
//! clients at 30 Hz and receives commands from them.

pub mod build_config;
pub mod handler;
pub mod preflight;
pub mod protocol;
pub mod server;

pub use build_config::BuildConfigHandler;
pub use handler::{ConnectionHandler, ValidatedCommand, ValidatedNshCommand};
pub use preflight::{heartbeat_hitl_signals, PreflightHandler};
pub use protocol::{
    Command, CommandType, ConnectionStatus, HandshakeAck, IncomingMessage, NshCommand, NshResponse,
    OutgoingMessage, PreflightApplyResult, PreflightApplyState, PreflightStatus, StateUpdate,
    TerrainOrigin, VehicleMessage,
};
pub use server::{WebSocketServer, WebSocketServerConfig};
