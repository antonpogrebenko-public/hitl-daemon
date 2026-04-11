//! MAVLink I/O library for HITL daemon
//!
//! This crate provides serial port communication with PX4-compatible flight controllers
//! using the MAVLink protocol.

pub mod async_io;
pub mod codec;
pub mod heartbeat;
pub mod messages;
pub mod serial;

pub use async_io::{MavlinkIo, NshRequest, NshResponseData, SerialConnectionState, reconnect_delay};
pub use codec::MavCodec;
pub use heartbeat::{ConnectionState, HeartbeatManager};
pub use messages::{make_hil_gps, make_hil_sensor};
pub use serial::{find_pixhawk_ports, open_serial, SerialConfig};
