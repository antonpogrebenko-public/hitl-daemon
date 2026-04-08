//! MAVLink encode/decode with sequence tracking

use mavlink::{ardupilotmega::MavMessage, peek_reader::PeekReader, MavFrame, MavHeader, MAV_STX_V2};
use std::io::{Read, Write};
use thiserror::Error;
use tracing::{debug, trace};

use crate::messages::{COMPONENT_ID, SYSTEM_ID};

#[derive(Debug, Error)]
pub enum CodecError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("MAVLink parse error: {0}")]
    Parse(#[from] mavlink::error::MessageReadError),

    #[error("MAVLink write error: {0}")]
    Write(#[from] mavlink::error::MessageWriteError),
}

/// MAVLink codec with sequence number tracking
pub struct MavCodec<S: Read> {
    reader: PeekReader<S>,
    sequence: u8,
    system_id: u8,
    component_id: u8,
}

impl<S: Read> MavCodec<S> {
    /// Create a new codec wrapping the given stream
    pub fn new(stream: S) -> Self {
        Self {
            reader: PeekReader::new(stream),
            sequence: 0,
            system_id: SYSTEM_ID,
            component_id: COMPONENT_ID,
        }
    }

    /// Create a new codec with custom system and component IDs
    pub fn with_ids(stream: S, system_id: u8, component_id: u8) -> Self {
        Self {
            reader: PeekReader::new(stream),
            sequence: 0,
            system_id,
            component_id,
        }
    }

    /// Get the current sequence number
    pub fn sequence(&self) -> u8 {
        self.sequence
    }

    /// Increment and return the next sequence number
    fn next_sequence(&mut self) -> u8 {
        let seq = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        seq
    }
}

impl<S: Read> MavCodec<S> {
    /// Receive a MAVLink message from the stream
    pub fn recv(&mut self) -> Result<(MavHeader, MavMessage), CodecError> {
        let (header, message) = mavlink::read_v2_msg::<MavMessage, _>(&mut self.reader)?;

        trace!(
            system_id = header.system_id,
            component_id = header.component_id,
            sequence = header.sequence,
            "Received MAVLink message"
        );

        Ok((header, message))
    }
}

impl<S: Read + Write> MavCodec<S> {
    /// Send a MAVLink message on the stream
    pub fn send(&mut self, message: MavMessage) -> Result<(), CodecError> {
        let header = MavHeader {
            system_id: self.system_id,
            component_id: self.component_id,
            sequence: self.next_sequence(),
        };

        debug!(
            system_id = header.system_id,
            component_id = header.component_id,
            sequence = header.sequence,
            "Sending MAVLink message"
        );

        mavlink::write_v2_msg(self.reader.reader_mut(), header, &message)?;
        Ok(())
    }

    /// Send a MAVLink frame (pre-constructed with header)
    pub fn send_frame(&mut self, frame: &MavFrame<MavMessage>) -> Result<(), CodecError> {
        let mut buffer = Vec::new();
        buffer.push(MAV_STX_V2);
        frame.ser(&mut buffer);
        self.reader.reader_mut().write_all(&buffer)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn test_sequence_wrapping() {
        let stream = Cursor::new(Vec::<u8>::new());
        let mut codec = MavCodec::new(stream);

        assert_eq!(codec.sequence(), 0);
        assert_eq!(codec.next_sequence(), 0);
        assert_eq!(codec.sequence(), 1);

        // Test wrapping
        codec.sequence = 255;
        assert_eq!(codec.next_sequence(), 255);
        assert_eq!(codec.sequence(), 0);
    }

    #[test]
    fn test_custom_ids() {
        let stream = Cursor::new(Vec::<u8>::new());
        let codec = MavCodec::with_ids(stream, 42, 100);

        assert_eq!(codec.system_id, 42);
        assert_eq!(codec.component_id, 100);
    }
}
