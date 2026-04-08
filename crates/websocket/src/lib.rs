//! WebSocket server for HITL daemon browser communication
//!
//! This crate provides a WebSocket server that streams telemetry data to browser
//! clients at 30 Hz and receives commands from them.

pub mod handler;
pub mod protocol;
pub mod server;

pub use handler::{ConnectionHandler, ValidatedCommand, ValidatedNshCommand};
pub use protocol::{Command, CommandType, HandshakeAck, IncomingMessage, NshCommand, NshResponse, OutgoingMessage, StateUpdate};
pub use server::{WebSocketServer, WebSocketServerConfig};
