//! Tokio-based async reader/writer with channels

use crossbeam_channel::{bounded, Receiver, Sender, TryRecvError};
use mavlink::{ardupilotmega::MavMessage, MavHeader};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::Duration;
use thiserror::Error;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::task::JoinHandle;
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use tracing::{debug, error, info, warn};

use crate::messages::{COMPONENT_ID, SYSTEM_ID};

/// Channel buffer size for send/receive queues
const CHANNEL_BUFFER_SIZE: usize = 256;

/// Reconnection timing constants
const RECONNECT_BASE_DELAY_MS: u64 = 250;
const RECONNECT_MAX_DELAY_MS: u64 = 1000;
/// Maximum reconnection attempts (exported for use in main.rs)
#[allow(dead_code)]
pub const RECONNECT_MAX_ATTEMPTS: u8 = 255;

/// Serial connection state broadcast to WebSocket clients
#[derive(Debug, Clone, PartialEq)]
pub struct SerialConnectionState {
    /// Whether Pixhawk is currently connected via serial
    pub connected: bool,
    /// Whether daemon is actively trying to reconnect
    pub reconnecting: bool,
    /// Number of reconnection attempts so far (0 when connected)
    pub retry_count: u8,
    /// Serial port path (empty if not connected)
    pub port: String,
}

#[derive(Debug, Error)]
pub enum AsyncIoError {
    #[error("Failed to open serial port: {0}")]
    SerialOpen(#[from] tokio_serial::Error),

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Channel send error")]
    ChannelSend,

    #[error("MAVLink parse error: {0}")]
    Parse(#[from] mavlink::error::MessageReadError),

    #[error("MAVLink write error: {0}")]
    Write(#[from] mavlink::error::MessageWriteError),
}

/// Raw bytes for NSH communication
#[derive(Debug, Clone)]
pub struct NshRequest {
    pub request_id: u32,
    pub data: Vec<u8>,
    pub timeout_ms: u16,
}

/// NSH response data
#[derive(Debug, Clone)]
pub struct NshResponseData {
    pub request_id: u32,
    pub data: Vec<u8>,
    pub complete: bool,
}

/// Async MAVLink I/O handler
pub struct MavlinkIo {
    /// Channel for sending messages to the flight controller
    tx: Sender<MavMessage>,
    /// Channel for receiving messages from the flight controller
    rx: Receiver<(MavHeader, MavMessage)>,
    /// Channel for sending NSH commands (raw bytes via SERIAL_CONTROL)
    nsh_tx: Sender<NshRequest>,
    /// Channel for receiving NSH responses
    nsh_rx: Receiver<NshResponseData>,
    /// Flag to signal shutdown
    shutdown: Arc<AtomicBool>,
    /// Count of MAVLink messages successfully parsed from serial
    pub packets_received: Arc<AtomicU32>,
    /// Reader task handle
    reader_handle: Option<JoinHandle<()>>,
    /// Writer task handle
    writer_handle: Option<JoinHandle<()>>,
}

impl MavlinkIo {
    /// Create a new MavlinkIo but don't start the tasks yet
    #[allow(clippy::type_complexity)]
    pub fn new() -> (
        Self,
        Sender<(MavHeader, MavMessage)>,
        Receiver<MavMessage>,
        Sender<NshResponseData>,
        Receiver<NshRequest>,
    ) {
        let (tx_to_fc, rx_from_app) = bounded::<MavMessage>(CHANNEL_BUFFER_SIZE);
        let (tx_to_app, rx_from_fc) = bounded::<(MavHeader, MavMessage)>(CHANNEL_BUFFER_SIZE);
        let (nsh_tx, nsh_rx_from_app) = bounded::<NshRequest>(32);
        let (nsh_tx_to_app, nsh_rx) = bounded::<NshResponseData>(64);

        let io = Self {
            tx: tx_to_fc,
            rx: rx_from_fc,
            nsh_tx,
            nsh_rx,
            shutdown: Arc::new(AtomicBool::new(false)),
            packets_received: Arc::new(AtomicU32::new(0)),
            reader_handle: None,
            writer_handle: None,
        };

        (io, tx_to_app, rx_from_app, nsh_tx_to_app, nsh_rx_from_app)
    }

    /// Send an NSH command (returns channel for response)
    pub fn send_nsh(&self, request: NshRequest) -> Result<(), AsyncIoError> {
        match self.nsh_tx.try_send(request) {
            Ok(()) => Ok(()),
            Err(crossbeam_channel::TrySendError::Full(_)) => {
                warn!("NSH channel is full");
                Err(AsyncIoError::ChannelSend)
            }
            Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                error!("NSH channel is disconnected");
                Err(AsyncIoError::ChannelSend)
            }
        }
    }

    /// Try to receive NSH response data (non-blocking)
    pub fn try_recv_nsh(&self) -> Option<NshResponseData> {
        match self.nsh_rx.try_recv() {
            Ok(data) => Some(data),
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => None,
        }
    }

    /// Spawn the reader and writer tasks for the given serial port
    pub async fn spawn(
        &mut self,
        port: &str,
        baud_rate: u32,
        tx_to_app: Sender<(MavHeader, MavMessage)>,
        rx_from_app: Receiver<MavMessage>,
        nsh_tx_to_app: Sender<NshResponseData>,
        nsh_rx_from_app: Receiver<NshRequest>,
    ) -> Result<(), AsyncIoError> {
        info!(port = %port, baud_rate, "Opening serial port for async I/O");

        let serial = tokio_serial::new(port, baud_rate).open_native_async()?;
        let (reader, writer) = tokio::io::split(serial);

        let shutdown_reader = self.shutdown.clone();
        let shutdown_writer = self.shutdown.clone();
        let packets_counter = self.packets_received.clone();

        // Spawn reader task
        let reader_handle = tokio::spawn(async move {
            Self::reader_task(reader, tx_to_app, nsh_tx_to_app, shutdown_reader, packets_counter).await;
        });

        // Spawn writer task
        let writer_handle = tokio::spawn(async move {
            Self::writer_task(writer, rx_from_app, nsh_rx_from_app, shutdown_writer).await;
        });

        self.reader_handle = Some(reader_handle);
        self.writer_handle = Some(writer_handle);

        Ok(())
    }

    /// Send a message to the flight controller
    pub fn send(&self, message: MavMessage) -> Result<(), AsyncIoError> {
        self.tx.send(message).map_err(|_| AsyncIoError::ChannelSend)
    }

    /// Try to receive a message from the flight controller (non-blocking)
    pub fn try_recv(&self) -> Option<(MavHeader, MavMessage)> {
        match self.rx.try_recv() {
            Ok(msg) => Some(msg),
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => {
                warn!("Receive channel disconnected");
                None
            }
        }
    }

    /// Receive a message from the flight controller (blocking)
    pub fn recv(&self) -> Option<(MavHeader, MavMessage)> {
        self.rx.recv().ok()
    }

    /// Signal shutdown and wait for tasks to complete
    pub async fn shutdown(mut self) {
        info!("Shutting down MAVLink I/O");
        self.shutdown.store(true, Ordering::SeqCst);

        if let Some(handle) = self.reader_handle.take() {
            let _ = handle.await;
        }
        if let Some(handle) = self.writer_handle.take() {
            let _ = handle.await;
        }
    }

    async fn reader_task(
        mut reader: tokio::io::ReadHalf<SerialStream>,
        tx: Sender<(MavHeader, MavMessage)>,
        nsh_tx: Sender<NshResponseData>,
        shutdown: Arc<AtomicBool>,
        packets_received: Arc<AtomicU32>,
    ) {
        info!("Reader task started");
        let mut buffer = [0u8; 1024];
        let mut parse_buffer = Vec::with_capacity(1024);

        loop {
            if shutdown.load(Ordering::SeqCst) {
                debug!("Reader task received shutdown signal");
                break;
            }

            match reader.read(&mut buffer).await {
                Ok(0) => {
                    warn!("Serial port closed");
                    break;
                }
                Ok(n) => {
                    parse_buffer.extend_from_slice(&buffer[..n]);

                    // Try to parse complete messages from the buffer
                    while let Some((header, message, consumed)) =
                        Self::try_parse_message(&parse_buffer)
                    {
                        parse_buffer.drain(..consumed);
                        packets_received.fetch_add(1, Ordering::Relaxed);

                        // Check for SERIAL_CONTROL responses (NSH data)
                        // Forward all SERIAL_CONTROL messages as NSH responses
                        if let MavMessage::SERIAL_CONTROL(ref sc) = message {
                            // Extract data up to count bytes
                            let data_len = sc.count.min(70) as usize;
                            let data = sc.data[..data_len].to_vec();

                            // PX4 sends count=0 to signal end of response.
                            // Intermediate packets may have count < 70 without
                            // meaning "complete", so only treat count==0 as done.
                            let complete = sc.count == 0;

                            debug!(
                                count = sc.count,
                                data_len = data_len,
                                complete = complete,
                                "Received SERIAL_CONTROL response"
                            );

                            // Send response if there's data OR if this is the completion signal
                            if !data.is_empty() || complete {
                                if nsh_tx
                                    .send(NshResponseData {
                                        request_id: 0, // Application correlates via pending request tracking
                                        data,
                                        complete,
                                    })
                                    .is_err()
                                {
                                    warn!("Failed to send NSH response to application");
                                }
                            }
                        }

                        if tx.send((header, message)).is_err() {
                            error!("Failed to send message to application");
                            return;
                        }
                    }
                }
                Err(e) => {
                    error!(error = %e, "Error reading from serial port");
                    break;
                }
            }
        }

        // Signal disconnect so the connection manager knows the FC is gone
        shutdown.store(true, Ordering::SeqCst);
        info!("Reader task finished");
    }

    async fn writer_task(
        mut writer: tokio::io::WriteHalf<SerialStream>,
        rx: Receiver<MavMessage>,
        nsh_rx: Receiver<NshRequest>,
        shutdown: Arc<AtomicBool>,
    ) {
        info!("Writer task started");
        let mut sequence: u8 = 0;
        let mut last_heartbeat = tokio::time::Instant::now();
        let heartbeat_interval = std::time::Duration::from_secs(1);

        // Send initial heartbeat immediately so PX4 knows we're here
        if let Ok(hb) = Self::serialize_heartbeat(&mut sequence) {
            let _ = writer.write_all(&hb).await;
        }

        loop {
            if shutdown.load(Ordering::SeqCst) {
                debug!("Writer task received shutdown signal");
                break;
            }

            // Send periodic GCS heartbeat (1 Hz) — PX4 requires this
            if last_heartbeat.elapsed() >= heartbeat_interval {
                if let Ok(hb) = Self::serialize_heartbeat(&mut sequence) {
                    if let Err(e) = writer.write_all(&hb).await {
                        error!(error = %e, "Failed to write heartbeat");
                        break;
                    }
                }
                last_heartbeat = tokio::time::Instant::now();
            }

            // Check for NSH requests first
            match nsh_rx.try_recv() {
                Ok(nsh_request) => {
                    debug!(
                        request_id = nsh_request.request_id,
                        len = nsh_request.data.len(),
                        "Sending NSH request via SERIAL_CONTROL"
                    );

                    // Send NSH data via SERIAL_CONTROL message
                    // Split into chunks of 70 bytes max (SERIAL_CONTROL data field size)
                    for chunk in nsh_request.data.chunks(70) {
                        let mut data = [0u8; 70];
                        data[..chunk.len()].copy_from_slice(chunk);

                        let sc = mavlink::ardupilotmega::SERIAL_CONTROL_DATA {
                            device: mavlink::ardupilotmega::SerialControlDev::SERIAL_CONTROL_DEV_SHELL,
                            flags: mavlink::ardupilotmega::SerialControlFlag::SERIAL_CONTROL_FLAG_RESPOND
                                | mavlink::ardupilotmega::SerialControlFlag::SERIAL_CONTROL_FLAG_EXCLUSIVE,
                            timeout: nsh_request.timeout_ms,
                            baudrate: 0, // Not used for shell
                            count: chunk.len() as u8,
                            data,
                        };

                        let message = MavMessage::SERIAL_CONTROL(sc);
                        let header = MavHeader {
                            system_id: SYSTEM_ID,
                            component_id: COMPONENT_ID,
                            sequence,
                        };
                        sequence = sequence.wrapping_add(1);

                        let mut buf = Vec::new();
                        if let Err(e) = mavlink::write_v2_msg(&mut buf, header, &message) {
                            error!(error = %e, "Failed to serialize SERIAL_CONTROL message");
                            continue;
                        }

                        if let Err(e) = writer.write_all(&buf).await {
                            error!(error = %e, "Failed to write SERIAL_CONTROL to serial port");
                            break;
                        }
                    }
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => {
                    warn!("NSH channel disconnected");
                }
            }

            // Check for regular MAVLink messages
            match rx.try_recv() {
                Ok(message) => {
                    let header = MavHeader {
                        system_id: SYSTEM_ID,
                        component_id: COMPONENT_ID,
                        sequence,
                    };
                    sequence = sequence.wrapping_add(1);

                    let mut buf = Vec::new();
                    if let Err(e) = mavlink::write_v2_msg(&mut buf, header, &message) {
                        error!(error = %e, "Failed to serialize MAVLink message");
                        continue;
                    }

                    if let Err(e) = writer.write_all(&buf).await {
                        error!(error = %e, "Failed to write to serial port");
                        break;
                    }
                }
                Err(TryRecvError::Empty) => {
                    // No messages to send, sleep briefly to allow other tasks to run
                    // Using sleep instead of yield_now for better tokio scheduling with sync channels
                    tokio::time::sleep(std::time::Duration::from_micros(100)).await;
                }
                Err(TryRecvError::Disconnected) => {
                    warn!("Send channel disconnected");
                    break;
                }
            }
        }

        // Signal disconnect so the connection manager knows the FC is gone
        shutdown.store(true, Ordering::SeqCst);
        info!("Writer task finished");
    }

    /// Serialize a GCS heartbeat message, advancing the sequence counter.
    fn serialize_heartbeat(sequence: &mut u8) -> Result<Vec<u8>, mavlink::error::MessageWriteError> {
        use crate::heartbeat::HeartbeatManager;

        let header = MavHeader {
            system_id: SYSTEM_ID,
            component_id: COMPONENT_ID,
            sequence: *sequence,
        };
        *sequence = sequence.wrapping_add(1);

        let mut buf = Vec::new();
        mavlink::write_v2_msg(&mut buf, header, &HeartbeatManager::make_heartbeat())?;
        Ok(buf)
    }

    fn try_parse_message(buffer: &[u8]) -> Option<(MavHeader, MavMessage, usize)> {
        use mavlink::peek_reader::PeekReader;
        use std::io::Cursor;

        if buffer.len() < 8 {
            return None;
        }

        let cursor = Cursor::new(buffer);
        let mut reader = PeekReader::new(cursor);
        match mavlink::read_v2_msg::<MavMessage, _>(&mut reader) {
            Ok((header, message)) => {
                let consumed = reader.reader_ref().position() as usize;
                Some((header, message, consumed))
            }
            Err(_) => None,
        }
    }
}

impl Default for MavlinkIo {
    fn default() -> Self {
        let (io, _, _, _, _) = Self::new();
        io
    }
}

impl MavlinkIo {
    /// Read the current packet count and reset to zero.
    /// Call once per second to get packets/sec.
    pub fn take_packet_count(&self) -> u32 {
        self.packets_received.swap(0, Ordering::Relaxed)
    }

    /// Check if the I/O tasks have shut down (connection lost)
    pub fn is_disconnected(&self) -> bool {
        self.shutdown.load(Ordering::Relaxed)
    }

    /// Signal disconnection (called when I/O error detected)
    pub fn signal_disconnect(&self) {
        self.shutdown.store(true, Ordering::SeqCst);
    }
}

/// Calculate reconnection delay with exponential backoff
pub fn reconnect_delay(attempt: u8) -> Duration {
    let delay_ms = RECONNECT_BASE_DELAY_MS * 2u64.pow(attempt.min(5) as u32);
    Duration::from_millis(delay_ms.min(RECONNECT_MAX_DELAY_MS))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mavlink_io_new() {
        let (io, _tx_to_app, _rx_from_app, _nsh_tx_to_app, _nsh_rx_from_app) = MavlinkIo::new();
        assert!(io.reader_handle.is_none());
        assert!(io.writer_handle.is_none());
    }
}
