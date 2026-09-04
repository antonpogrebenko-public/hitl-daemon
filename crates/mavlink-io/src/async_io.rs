//! Tokio-based async reader/writer with channels

use crossbeam_channel::{bounded, Receiver, Sender, TryRecvError};
use mavlink::{ardupilotmega::MavMessage, MavHeader};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;
use thiserror::Error;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::mpsc as tokio_mpsc;
use tokio::sync::mpsc::{Receiver as TokioReceiver, Sender as TokioSender};
use tokio::task::JoinHandle;
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use tracing::{debug, error, info, warn};

use crate::messages::{COMPONENT_ID, SYSTEM_ID};

/// Channel buffer size for send/receive queues
const CHANNEL_BUFFER_SIZE: usize = 256;

/// Maximum parse buffer size before forced drain (prevents OOM on corrupt streams)
const MAX_PARSE_BUFFER_SIZE: usize = 8192;

/// MAVLink v2 start byte
const MAVLINK_V2_STX: u8 = 0xFD;

/// Timeout for serial write operations
/// How long the writer waits for outbound MAVLink before re-checking the NSH
/// queue. Only bounds NSH's responsiveness; MAVLink itself is awaited.
const NSH_POLL_INTERVAL: std::time::Duration = std::time::Duration::from_millis(5);

const WRITE_TIMEOUT: Duration = Duration::from_secs(2);

/// How long to wait for a serial `open()` before abandoning the port.
///
/// A healthy board opens in well under a millisecond, so this budget is not
/// for slow hardware — it is for hardware whose `open()` never returns at all.
/// A PX4 board sitting in its bootloader does exactly that: the call does not
/// fail, it simply never completes, and without a deadline the connection
/// manager stops forever with no log line and no recovery short of SIGKILL.
const OPEN_TIMEOUT: Duration = Duration::from_secs(5);

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

/// Why a [`MavlinkIo::try_send`] failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrySendError {
    /// The writer is behind, and the message was dropped.
    Full,
    /// The writer is gone; the link is finished.
    Disconnected,
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

    #[error("Blocking task join error: {0}")]
    TaskJoin(String),

    #[error("Timed out after {seconds}s opening serial port '{port}' — the device accepted the connection but never completed it")]
    OpenTimeout { port: String, seconds: u64 },
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
    /// Outbound queue to the serial writer.
    ///
    /// A tokio channel, not a crossbeam one, so the writer can `await` a
    /// message instead of polling. Every producer is on the runtime, and the
    /// writer's previous shape was `try_recv` plus a 1 ms sleep — which is 1 ms
    /// of added latency on a link whose control period is 2.5 ms, and a
    /// thousand pointless wakeups a second when idle.
    tx: TokioSender<MavMessage>,
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
    /// Count of successfully parsed MAVLink frames (for link quality tracking)
    parse_successes: Arc<AtomicU64>,
    /// Count of corrupted/dropped frames detected via drain_to_next_frame (for link quality tracking)
    parse_failures: Arc<AtomicU64>,
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
        // The outbound queue is a tokio receiver now, so the writer can await
        // it rather than poll.
        TokioReceiver<MavMessage>,
        Sender<NshResponseData>,
        Receiver<NshRequest>,
    ) {
        let (tx_to_fc, rx_from_app) = tokio_mpsc::channel::<MavMessage>(CHANNEL_BUFFER_SIZE);
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
            parse_successes: Arc::new(AtomicU64::new(0)),
            parse_failures: Arc::new(AtomicU64::new(0)),
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
        rx_from_app: TokioReceiver<MavMessage>,
        nsh_tx_to_app: Sender<NshResponseData>,
        nsh_rx_from_app: Receiver<NshRequest>,
    ) -> Result<(), AsyncIoError> {
        info!(port = %port, baud_rate, "Opening serial port for async I/O");

        // `open_native_async()` performs a synchronous, unyielding OS
        // `open()` call under the hood — mio_serial's own debug log says
        // "opening serial port in synchronous blocking mode". On a wedged
        // macOS USB CDC-ACM device this can stall for tens of seconds.
        // Run it on tokio's dedicated blocking-thread pool instead of
        // inline in this async task: a stall then ties up a blocking-pool
        // thread instead of one of the runtime's small number of core
        // worker threads, which otherwise starves unrelated async work
        // (e.g. accepting new WebSocket connections) for the same
        // duration — observed on real hardware as WS handshakes timing out
        // while a reconnect attempt was stuck opening the port.
        let port_owned = port.to_string();
        let open_task = tokio::task::spawn_blocking(move || {
            tokio_serial::new(port_owned, baud_rate).open_native_async()
        });

        // A blocking `open()` cannot be cancelled, so on timeout the thread is
        // abandoned rather than awaited — it stays parked until the device is
        // unplugged. That costs one blocking-pool thread per timeout, which is
        // why callers must also refuse ports already known to hang (see
        // `is_bootloader_product`) instead of leaning on this deadline. It is
        // the backstop for the unknown case, not the primary defence.
        let serial = match tokio::time::timeout(OPEN_TIMEOUT, open_task).await {
            Ok(joined) => joined.map_err(|e| AsyncIoError::TaskJoin(e.to_string()))??,
            Err(_) => {
                warn!(
                    port = %port,
                    seconds = OPEN_TIMEOUT.as_secs(),
                    "Serial open did not complete — abandoning this port"
                );
                return Err(AsyncIoError::OpenTimeout {
                    port: port.to_string(),
                    seconds: OPEN_TIMEOUT.as_secs(),
                });
            }
        };
        let (reader, writer) = tokio::io::split(serial);

        let shutdown_reader = self.shutdown.clone();
        let shutdown_writer = self.shutdown.clone();
        let packets_counter = self.packets_received.clone();
        let parse_successes = self.parse_successes.clone();
        let parse_failures = self.parse_failures.clone();

        // Spawn reader task
        let reader_handle = tokio::spawn(async move {
            Self::reader_task(
                reader,
                tx_to_app,
                nsh_tx_to_app,
                shutdown_reader,
                packets_counter,
                parse_successes,
                parse_failures,
            )
            .await;
        });

        // Spawn writer task
        let writer_handle = tokio::spawn(async move {
            Self::writer_task(writer, rx_from_app, nsh_rx_from_app, shutdown_writer).await;
        });

        self.reader_handle = Some(reader_handle);
        self.writer_handle = Some(writer_handle);

        Ok(())
    }

    /// Send a message to the flight controller, blocking if the queue is full.
    ///
    /// Only safe from a thread that may block. From an async task use
    /// [`try_send`](Self::try_send): this parks the calling worker until the
    /// writer drains, and the writer's own `WRITE_TIMEOUT` is two seconds, so a
    /// stalled serial port takes a runtime thread with it.
    pub fn send(&self, message: MavMessage) -> Result<(), AsyncIoError> {
        self.tx
            .blocking_send(message)
            .map_err(|_| AsyncIoError::ChannelSend)
    }

    /// Send without blocking; report whether the queue had room.
    ///
    /// A full queue means the writer is not keeping up, and the honest
    /// response is to drop the message and say so. Blocking instead moves the
    /// backlog into the runtime, where it stops unrelated tasks — the comment
    /// on the reader task above records starved workers causing WebSocket
    /// handshake timeouts on real hardware, which is the same failure reached
    /// by a different path.
    pub fn try_send(&self, message: MavMessage) -> Result<(), TrySendError> {
        match self.tx.try_send(message) {
            Ok(()) => Ok(()),
            Err(tokio_mpsc::error::TrySendError::Full(_)) => Err(TrySendError::Full),
            Err(tokio_mpsc::error::TrySendError::Closed(_)) => Err(TrySendError::Disconnected),
        }
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
        parse_successes: Arc<AtomicU64>,
        parse_failures: Arc<AtomicU64>,
    ) {
        info!("Reader task started");
        let mut buffer = [0u8; 1024];
        let mut parse_buffer = Vec::with_capacity(1024);

        loop {
            if shutdown.load(Ordering::SeqCst) {
                debug!("Reader task received shutdown signal");
                break;
            }

            // Timeout read so we can check shutdown flag periodically
            let read_result =
                tokio::time::timeout(Duration::from_secs(1), reader.read(&mut buffer)).await;

            let read_result = match read_result {
                Ok(result) => result,
                Err(_) => continue, // Timeout — loop back to check shutdown
            };

            match read_result {
                Ok(0) => {
                    warn!("Serial port closed");
                    break;
                }
                Ok(n) => {
                    parse_buffer.extend_from_slice(&buffer[..n]);

                    // Prevent unbounded growth from corrupt streams
                    if parse_buffer.len() > MAX_PARSE_BUFFER_SIZE {
                        warn!(
                            size = parse_buffer.len(),
                            "Parse buffer exceeded limit — scanning for next frame start"
                        );
                        // Do NOT skip byte 0: if it's already 0xFD, preserve it as a
                        // potential frame start (no parse attempt has been made yet).
                        parse_failures.fetch_add(1, Ordering::Relaxed);
                        Self::drain_to_next_frame(&mut parse_buffer, false);
                    }

                    // Try to parse complete messages from the buffer.
                    //
                    // `parsed` advances through the chunk instead of each message
                    // draining the front of the buffer. `drain(..consumed)` moves
                    // every remaining byte down by the size of the message just
                    // taken, so a chunk carrying k messages memmoved the tail k
                    // times; the single drain after this loop moves it once.
                    let mut parsed = 0usize;
                    loop {
                        match Self::try_parse_message(&parse_buffer[parsed..]) {
                            Some((header, message, consumed)) => {
                                parsed += consumed;
                                packets_received.fetch_add(1, Ordering::Relaxed);
                                parse_successes.fetch_add(1, Ordering::Relaxed);

                                // Check for SERIAL_CONTROL responses (NSH data)
                                if let MavMessage::SERIAL_CONTROL(ref sc) = message {
                                    let data_len = sc.count.min(70) as usize;
                                    let data = sc.data[..data_len].to_vec();
                                    let complete = sc.count == 0;

                                    debug!(
                                        count = sc.count,
                                        data_len = data_len,
                                        complete = complete,
                                        "Received SERIAL_CONTROL response"
                                    );

                                    if !data.is_empty() || complete {
                                        if nsh_tx
                                            .send(NshResponseData {
                                                request_id: 0,
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
                            None => {
                                // Retire what parsed before dealing with the
                                // remainder: `drain_to_next_frame` scans from the
                                // front of the buffer, so it has to see a buffer
                                // whose front is the unparsed bytes and not the
                                // messages already handed to the application.
                                if parsed > 0 {
                                    parse_buffer.drain(..parsed);
                                    parsed = 0;
                                }

                                // No valid frame found — if buffer has data but starts
                                // with a non-STX byte, skip to the next potential frame.
                                // skip_first=true because byte 0 is confirmed non-STX and
                                // should be discarded before searching.
                                if parse_buffer.len() >= 8 && parse_buffer[0] != MAVLINK_V2_STX {
                                    parse_failures.fetch_add(1, Ordering::Relaxed);
                                    Self::drain_to_next_frame(&mut parse_buffer, true);
                                }
                                break;
                            }
                        }
                    }

                    // One memmove for the whole chunk. Reached only when the loop
                    // broke on a partial frame with nothing to resynchronise, since
                    // the `None` arm above retires `parsed` before it touches the
                    // buffer and resets the counter.
                    if parsed > 0 {
                        parse_buffer.drain(..parsed);
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
        mut rx: TokioReceiver<MavMessage>,
        nsh_rx: Receiver<NshRequest>,
        shutdown: Arc<AtomicBool>,
    ) {
        info!("Writer task started");
        let mut sequence: u8 = 0;
        // One buffer for the life of the writer task; see the note at the
        // message-write site below.
        let mut buf: Vec<u8> = Vec::with_capacity(300);
        let mut last_heartbeat = tokio::time::Instant::now();
        let heartbeat_interval = std::time::Duration::from_secs(1);

        // Send initial heartbeat immediately so PX4 knows we're here
        if let Ok(hb) = Self::serialize_heartbeat(&mut sequence) {
            let _ = tokio::time::timeout(WRITE_TIMEOUT, writer.write_all(&hb)).await;
        }

        loop {
            if shutdown.load(Ordering::SeqCst) {
                debug!("Writer task received shutdown signal");
                break;
            }

            // Send periodic GCS heartbeat (1 Hz) — PX4 requires this
            if last_heartbeat.elapsed() >= heartbeat_interval {
                if let Ok(hb) = Self::serialize_heartbeat(&mut sequence) {
                    match tokio::time::timeout(WRITE_TIMEOUT, writer.write_all(&hb)).await {
                        Ok(Ok(())) => {}
                        Ok(Err(e)) => {
                            error!(error = %e, "Failed to write heartbeat");
                            break;
                        }
                        Err(_) => {
                            error!("Heartbeat write timed out — serial port stalled");
                            break;
                        }
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

                    let mut write_failed = false;
                    for chunk in nsh_request.data.chunks(70) {
                        let mut data = [0u8; 70];
                        data[..chunk.len()].copy_from_slice(chunk);

                        let sc = mavlink::ardupilotmega::SERIAL_CONTROL_DATA {
                            device: mavlink::ardupilotmega::SerialControlDev::SERIAL_CONTROL_DEV_SHELL,
                            flags: mavlink::ardupilotmega::SerialControlFlag::SERIAL_CONTROL_FLAG_RESPOND
                                | mavlink::ardupilotmega::SerialControlFlag::SERIAL_CONTROL_FLAG_EXCLUSIVE,
                            timeout: nsh_request.timeout_ms,
                            baudrate: 0,
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

                        buf.clear();
                        if let Err(e) = mavlink::write_v2_msg(&mut buf, header, &message) {
                            error!(error = %e, "Failed to serialize SERIAL_CONTROL message");
                            continue;
                        }

                        match tokio::time::timeout(WRITE_TIMEOUT, writer.write_all(&buf)).await {
                            Ok(Ok(())) => {}
                            Ok(Err(e)) => {
                                error!(error = %e, "Failed to write SERIAL_CONTROL to serial port");
                                write_failed = true;
                                break;
                            }
                            Err(_) => {
                                error!("SERIAL_CONTROL write timed out — serial port stalled");
                                write_failed = true;
                                break;
                            }
                        }
                    }
                    if write_failed {
                        break;
                    }
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => {
                    warn!("NSH channel disconnected");
                }
            }

            // Wait for the next outbound MAVLink message rather than polling
            // for it.
            //
            // This was `try_recv` followed by a 1 ms sleep when empty, which at
            // the arrival rate meant the queue was usually empty on inspection
            // and the message that arrived a moment later waited out the sleep.
            // That is up to 1 ms of added latency on a link whose control
            // period is 2.5 ms, plus a thousand wakeups a second doing nothing.
            //
            // The timeout is only so the NSH branch above still gets serviced;
            // NSH is an interactive shell, so a 5 ms polling interval there is
            // imperceptible, while MAVLink is now delivered the moment it is
            // queued.
            match tokio::time::timeout(NSH_POLL_INTERVAL, rx.recv()).await {
                Ok(Some(message)) => {
                    let header = MavHeader {
                        system_id: SYSTEM_ID,
                        component_id: COMPONENT_ID,
                        sequence,
                    };
                    sequence = sequence.wrapping_add(1);

                    // Reused across messages rather than allocated per frame.
                    // HIL_SENSOR alone is 400 a second, and a fresh zero-capacity
                    // Vec reallocates several times on its way to a ~76-byte
                    // frame — about 1,600 allocations a second on the daemon's
                    // busiest path, for a buffer whose size never changes.
                    buf.clear();
                    if let Err(e) = mavlink::write_v2_msg(&mut buf, header, &message) {
                        error!(error = %e, "Failed to serialize MAVLink message");
                        continue;
                    }

                    match tokio::time::timeout(WRITE_TIMEOUT, writer.write_all(&buf)).await {
                        Ok(Ok(())) => {}
                        Ok(Err(e)) => {
                            error!(error = %e, "Failed to write to serial port");
                            break;
                        }
                        Err(_) => {
                            error!("Serial write timed out — port stalled");
                            break;
                        }
                    }
                }
                Ok(None) => {
                    warn!("Send channel disconnected");
                    break;
                }
                // Nothing outbound within the interval — loop round so the NSH
                // branch is checked again.
                Err(_) => {}
            }
        }

        // Signal disconnect so the connection manager knows the FC is gone
        shutdown.store(true, Ordering::SeqCst);
        info!("Writer task finished");
    }

    /// Serialize a GCS heartbeat message, advancing the sequence counter.
    fn serialize_heartbeat(
        sequence: &mut u8,
    ) -> Result<Vec<u8>, mavlink::error::MessageWriteError> {
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

    /// Drain bytes up to the next MAVLink v2 start byte (0xFD).
    /// If no start byte found, clears the entire buffer.
    ///
    /// `skip_first`: when `true`, always skip byte 0 before searching (caller has
    /// already confirmed byte 0 is not a valid frame start, e.g. it is not 0xFD).
    /// When `false`, byte 0 is inspected first; if it is already 0xFD it is
    /// preserved as the next frame candidate, otherwise the search begins from
    /// byte 1 (byte 0 is discarded as garbage).
    fn drain_to_next_frame(buffer: &mut Vec<u8>, skip_first: bool) {
        let start = if skip_first || buffer.first() != Some(&MAVLINK_V2_STX) {
            1
        } else {
            0
        };
        if let Some(pos) = buffer.iter().skip(start).position(|&b| b == MAVLINK_V2_STX) {
            let drain_count = pos + start;
            debug!(
                drained = drain_count,
                "Skipped corrupt bytes to next frame start"
            );
            buffer.drain(..drain_count);
        } else {
            let drained = buffer.len();
            buffer.clear();
            debug!(drained, "No frame start found — cleared parse buffer");
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

    /// Returns the serial link quality as a percentage (0–100).
    /// 100 means no parse failures observed since last reset (or no data yet).
    pub fn link_quality_percent(&self) -> u8 {
        let successes = self.parse_successes.load(Ordering::Relaxed);
        let failures = self.parse_failures.load(Ordering::Relaxed);
        let total = successes + failures;
        if total == 0 {
            return 100;
        }
        ((successes * 100) / total).min(100) as u8
    }

    /// Atomically swap parse success and failure counters to zero and return
    /// their previous values as `(successes, failures)`.
    /// Call periodically (e.g. every 5 s) to get a rolling window of link quality.
    pub fn take_parse_stats(&self) -> (u64, u64) {
        let s = self.parse_successes.swap(0, Ordering::Relaxed);
        let f = self.parse_failures.swap(0, Ordering::Relaxed);
        (s, f)
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

    /// `n` concatenated v2 frames, as one serial read would deliver them.
    fn frames(n: usize) -> Vec<u8> {
        let mut sequence = 0u8;
        let mut buf = Vec::new();
        for _ in 0..n {
            buf.extend_from_slice(&MavlinkIo::serialize_heartbeat(&mut sequence).unwrap());
        }
        buf
    }

    /// The reader's loop: parse at an advancing offset rather than draining
    /// each message off the front.
    fn parse_all(buffer: &[u8]) -> (usize, usize) {
        let mut parsed = 0usize;
        let mut count = 0usize;
        while let Some((_, _, consumed)) = MavlinkIo::try_parse_message(&buffer[parsed..]) {
            parsed += consumed;
            count += 1;
        }
        (count, parsed)
    }

    #[test]
    fn every_message_in_a_chunk_is_parsed() {
        // The property the cursor rests on: parsing at an advancing offset must
        // find the same messages that draining the front found, and consume the
        // buffer exactly.
        let buffer = frames(5);
        let (count, parsed) = parse_all(&buffer);
        assert_eq!(count, 5, "all five frames parse from one chunk");
        assert_eq!(parsed, buffer.len(), "the chunk is consumed exactly");
    }

    #[test]
    fn a_trailing_partial_frame_is_left_for_the_next_chunk() {
        // The case that made the per-message drain look necessary. What must
        // survive is the tail: the reader drains `parsed` and keeps the rest,
        // so the next read appends to a partial frame rather than a mangled one.
        let whole = frames(3);
        let one = frames(1).len();
        let truncated = &whole[..whole.len() - 4];

        let (count, parsed) = parse_all(truncated);
        assert_eq!(count, 2, "two complete frames, the third is short");
        assert_eq!(parsed, one * 2, "only the complete frames are consumed");

        // What the reader keeps, and what it must be.
        let remainder = &truncated[parsed..];
        assert_eq!(remainder.len(), one - 4);
        assert_eq!(
            remainder[0], MAVLINK_V2_STX,
            "the tail still starts a frame"
        );
    }

    #[test]
    fn draining_once_leaves_what_draining_per_message_left() {
        // Equivalence with the behaviour this replaced, asserted rather than
        // assumed: the buffer after one drain of `parsed` is byte-identical to
        // the buffer after draining each message as it was parsed.
        let whole = frames(4);
        let truncated = whole[..whole.len() - 6].to_vec();

        let mut per_message = truncated.clone();
        loop {
            match MavlinkIo::try_parse_message(&per_message) {
                Some((_, _, consumed)) => {
                    per_message.drain(..consumed);
                }
                None => break,
            }
        }

        let mut cursored = truncated.clone();
        let (_, parsed) = parse_all(&cursored);
        cursored.drain(..parsed);

        assert_eq!(cursored, per_message);
    }

    #[test]
    fn an_empty_or_short_buffer_parses_nothing() {
        assert_eq!(parse_all(&[]).0, 0);
        assert_eq!(parse_all(&[0xFD, 0x00, 0x00]).0, 0);
    }
}
