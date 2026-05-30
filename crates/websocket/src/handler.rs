//! WebSocket connection handler
//!
//! Handles individual client connections, parsing incoming commands and
//! sending state updates.

use crate::build_config::BuildConfigHandler;
use crate::protocol::{
    Command, CommandAck, CommandType, ConfigResult, ConfigState, HandshakeAck, IncomingMessage,
    NshCommand, NshResponse, OutgoingMessage, StateUpdate,
};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, mpsc, RwLock};
use tracing::{debug, info, warn};

/// Callback for battery recharge (avoids websocket → simulation crate dependency)
pub type RechargeCallback = Arc<dyn Fn() + Send + Sync>;

/// Rate limiting configuration
const COMMAND_RATE_LIMIT_WINDOW: Duration = Duration::from_secs(1);
const MAX_COMMANDS_PER_WINDOW: usize = 10;

/// NSH commands that are blocked by default for safety.
/// These can cause data loss or make the FC unresponsive.
const BLOCKED_NSH_COMMANDS: &[&str] = &[
    "shutdown",
    "erase",
    "format",
    "hardfault_log",
    "bl_update",
];

/// Takeoff altitude bounds (meters)
const MIN_TAKEOFF_ALTITUDE: f32 = 1.0;
const MAX_TAKEOFF_ALTITUDE: f32 = 100.0;

/// Validated command ready for execution
#[derive(Debug, Clone)]
pub struct ValidatedCommand {
    pub command: Command,
    pub client_id: u64,
}

/// Validated NSH command ready for execution
#[derive(Debug, Clone)]
pub struct ValidatedNshCommand {
    pub request_id: u32,
    pub command: String,
    pub timeout_ms: u16,
    pub client_id: u64,
}

/// Connection handler state shared across all clients
pub struct ConnectionHandler {
    /// Daemon version
    version_major: u8,
    version_minor: u8,
    version_patch: u8,
    /// Current serial port
    serial_port: String,
    /// Whether Pixhawk is connected
    pixhawk_connected: Arc<RwLock<bool>>,
    /// Channel to send validated commands to simulation
    command_tx: mpsc::Sender<ValidatedCommand>,
    /// Channel to send NSH commands for execution
    nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>,
    /// Handler for build configuration requests
    build_config: Option<Arc<BuildConfigHandler>>,
    /// Callback to recharge battery in simulation
    recharge_fn: Option<RechargeCallback>,
    /// Broadcast channel to receive state updates
    state_rx: broadcast::Receiver<StateUpdate>,
    /// Rate limiting: client_id -> (window_start, command_count)
    rate_limits: Arc<RwLock<HashMap<u64, (Instant, usize)>>>,
    /// Next client ID
    next_client_id: Arc<RwLock<u64>>,
    /// Shutdown signal triggered by browser client
    shutdown_signal: Arc<AtomicBool>,
}

impl ConnectionHandler {
    /// Create a new connection handler
    pub fn new(
        version_major: u8,
        version_minor: u8,
        version_patch: u8,
        serial_port: String,
        command_tx: mpsc::Sender<ValidatedCommand>,
        state_rx: broadcast::Receiver<StateUpdate>,
        shutdown_signal: Arc<AtomicBool>,
    ) -> Self {
        Self {
            version_major,
            version_minor,
            version_patch,
            serial_port,
            pixhawk_connected: Arc::new(RwLock::new(false)),
            command_tx,
            nsh_tx: None,
            build_config: None,
            recharge_fn: None,
            state_rx,
            rate_limits: Arc::new(RwLock::new(HashMap::new())),
            next_client_id: Arc::new(RwLock::new(1)),
            shutdown_signal,
        }
    }

    /// Set the NSH command channel
    pub fn set_nsh_sender(&mut self, nsh_tx: mpsc::Sender<ValidatedNshCommand>) {
        self.nsh_tx = Some(nsh_tx);
    }

    /// Get a clone of the NSH sender (if available)
    pub fn nsh_sender(&self) -> Option<mpsc::Sender<ValidatedNshCommand>> {
        self.nsh_tx.clone()
    }

    /// Set the build config handler
    pub fn set_build_config_handler(&mut self, handler: Arc<BuildConfigHandler>) {
        self.build_config = Some(handler);
    }

    /// Set the battery recharge callback
    pub fn set_recharge_callback(&mut self, callback: RechargeCallback) {
        self.recharge_fn = Some(callback);
    }

    /// Allocate a new client ID
    pub async fn allocate_client_id(&self) -> u64 {
        let mut id = self.next_client_id.write().await;
        let client_id = *id;
        *id += 1;
        client_id
    }

    /// Set FC connection status
    pub async fn set_pixhawk_connected(&self, connected: bool) {
        *self.pixhawk_connected.write().await = connected;
    }

    /// Get a new state update receiver
    pub fn subscribe_state(&self) -> broadcast::Receiver<StateUpdate> {
        self.state_rx.resubscribe()
    }

    /// Handle a handshake request
    pub async fn handle_handshake(&self) -> OutgoingMessage {
        let pixhawk_connected = *self.pixhawk_connected.read().await;
        OutgoingMessage::HandshakeAck(HandshakeAck {
            version_major: self.version_major,
            version_minor: self.version_minor,
            version_patch: self.version_patch,
            pixhawk_connected,
            serial_port: self.serial_port.clone(),
        })
    }

    /// Handle an incoming message from a client.
    ///
    /// `progress_tx` is the per-client outgoing channel. Most handlers ignore
    /// it and return their single response via `Ok(Some(...))`. ConfigureBuild
    /// uses it to push an interim `state: Configuring` ConfigResult before
    /// the final result is returned (two-stage flow — see `BuildConfigHandler::handle`).
    pub async fn handle_message(
        &self,
        client_id: u64,
        data: &[u8],
        progress_tx: &mpsc::Sender<OutgoingMessage>,
    ) -> Result<Option<OutgoingMessage>, String> {
        let msg = IncomingMessage::from_bytes(data).map_err(|e| e.to_string())?;

        match msg {
            IncomingMessage::Handshake => {
                info!(client_id, "Client handshake");
                Ok(Some(self.handle_handshake().await))
            }
            IncomingMessage::Command(cmd) => {
                debug!(client_id, command_id = cmd.command_id, ?cmd.command_type, "Received command");
                Ok(Some(self.handle_command(client_id, cmd).await))
            }
            IncomingMessage::NshCommand(nsh) => {
                debug!(client_id, request_id = nsh.request_id, cmd = %nsh.command, "Received NSH command");
                Ok(Some(self.handle_nsh_command(client_id, nsh).await))
            }
            IncomingMessage::ConfigureBuild(build) => {
                debug!(client_id, motor_slug = %build.motor_slug, "Received build config request");
                let handler = match &self.build_config {
                    Some(h) => h.clone(),
                    None => {
                        let result = ConfigResult {
                            state: ConfigState::Error,
                            success: false,
                            error: Some("Build configuration not available".to_string()),
                            config: None,
                        };
                        return Ok(Some(OutgoingMessage::ConfigResult(result)));
                    }
                };
                let result = handler.handle(build, progress_tx.clone()).await;
                Ok(Some(OutgoingMessage::ConfigResult(result)))
            }
            IncomingMessage::Shutdown => {
                info!(client_id, "Shutdown command received from browser");
                self.shutdown_signal.store(true, Ordering::SeqCst);
                Ok(None)
            }
        }
    }

    /// Handle an NSH command from a client
    async fn handle_nsh_command(&self, client_id: u64, nsh: NshCommand) -> OutgoingMessage {
        // Check if NSH is available
        let Some(ref nsh_tx) = self.nsh_tx else {
            return OutgoingMessage::NshResponse(NshResponse {
                request_id: nsh.request_id,
                success: false,
                complete: true,
                output: "NSH not available (no FC connected)".to_string(),
            });
        };

        // Rate limiting check
        if let Err(e) = self.check_rate_limit(client_id).await {
            warn!(client_id, request_id = nsh.request_id, "Rate limited: {}", e);
            return OutgoingMessage::NshResponse(NshResponse {
                request_id: nsh.request_id,
                success: false,
                complete: true,
                output: e,
            });
        }

        // Validate command (basic sanity checks)
        if nsh.command.is_empty() {
            return OutgoingMessage::NshResponse(NshResponse {
                request_id: nsh.request_id,
                success: false,
                complete: true,
                output: "Empty command".to_string(),
            });
        }

        if nsh.command.len() > 256 {
            return OutgoingMessage::NshResponse(NshResponse {
                request_id: nsh.request_id,
                success: false,
                complete: true,
                output: "Command too long (max 256 chars)".to_string(),
            });
        }

        // Block dangerous commands
        let cmd_lower = nsh.command.trim().to_lowercase();
        for &blocked in BLOCKED_NSH_COMMANDS {
            if cmd_lower == blocked || cmd_lower.starts_with(&format!("{} ", blocked)) {
                warn!(client_id, cmd = %nsh.command, "Blocked dangerous NSH command");
                return OutgoingMessage::NshResponse(NshResponse {
                    request_id: nsh.request_id,
                    success: false,
                    complete: true,
                    output: format!("Command '{}' is blocked for safety", blocked),
                });
            }
        }

        // Forward to NSH handler
        let validated = ValidatedNshCommand {
            request_id: nsh.request_id,
            command: nsh.command.clone(),
            timeout_ms: if nsh.timeout_ms == 0 { 2000 } else { nsh.timeout_ms },
            client_id,
        };

        match nsh_tx.try_send(validated) {
            Ok(_) => {
                info!(
                    client_id,
                    request_id = nsh.request_id,
                    cmd = %nsh.command,
                    "NSH command forwarded"
                );
                OutgoingMessage::NshResponse(NshResponse {
                    request_id: nsh.request_id,
                    success: true,
                    complete: false,
                    output: String::new(),
                })
            }
            Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                warn!(client_id, request_id = nsh.request_id, "NSH queue full");
                OutgoingMessage::NshResponse(NshResponse {
                    request_id: nsh.request_id,
                    success: false,
                    complete: true,
                    output: "NSH busy — previous command still in progress".to_string(),
                })
            }
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => {
                warn!(client_id, request_id = nsh.request_id, "NSH channel closed");
                OutgoingMessage::NshResponse(NshResponse {
                    request_id: nsh.request_id,
                    success: false,
                    complete: true,
                    output: "Internal error: NSH channel closed".to_string(),
                })
            }
        }
    }

    /// Handle a command from a client
    async fn handle_command(&self, client_id: u64, cmd: Command) -> OutgoingMessage {
        // Rate limiting check
        if let Err(e) = self.check_rate_limit(client_id).await {
            warn!(client_id, command_id = cmd.command_id, "Rate limited: {}", e);
            return OutgoingMessage::CommandAck(CommandAck {
                command_id: cmd.command_id,
                success: false,
                error: Some(e),
            });
        }

        // Handle recharge locally (doesn't go to FC)
        if cmd.command_type == CommandType::Recharge {
            if let Some(ref recharge) = self.recharge_fn {
                recharge();
                info!(client_id, "Battery recharged");
                return OutgoingMessage::CommandAck(CommandAck {
                    command_id: cmd.command_id,
                    success: true,
                    error: None,
                });
            }
            return OutgoingMessage::CommandAck(CommandAck {
                command_id: cmd.command_id,
                success: false,
                error: Some("Battery recharge unavailable".to_string()),
            });
        }

        // Validate command parameters
        if let Err(e) = self.validate_command(&cmd) {
            warn!(
                client_id,
                command_id = cmd.command_id,
                "Invalid command: {}", e
            );
            return OutgoingMessage::CommandAck(CommandAck {
                command_id: cmd.command_id,
                success: false,
                error: Some(e),
            });
        }

        // Forward to simulation
        let validated = ValidatedCommand {
            command: cmd.clone(),
            client_id,
        };

        match self.command_tx.send(validated).await {
            Ok(_) => {
                info!(
                    client_id,
                    command_id = cmd.command_id,
                    ?cmd.command_type,
                    "Command forwarded to simulation"
                );
                OutgoingMessage::CommandAck(CommandAck {
                    command_id: cmd.command_id,
                    success: true,
                    error: None,
                })
            }
            Err(e) => {
                warn!(
                    client_id,
                    command_id = cmd.command_id,
                    "Failed to forward command: {}", e
                );
                OutgoingMessage::CommandAck(CommandAck {
                    command_id: cmd.command_id,
                    success: false,
                    error: Some("Internal error: command channel closed".to_string()),
                })
            }
        }
    }

    /// Check rate limiting for a client
    async fn check_rate_limit(&self, client_id: u64) -> Result<(), String> {
        let mut limits = self.rate_limits.write().await;
        let now = Instant::now();

        let (window_start, count) = limits.entry(client_id).or_insert((now, 0));

        // Reset window if expired
        if now.duration_since(*window_start) > COMMAND_RATE_LIMIT_WINDOW {
            *window_start = now;
            *count = 0;
        }

        if *count >= MAX_COMMANDS_PER_WINDOW {
            return Err(format!(
                "Rate limit exceeded: max {} commands per {:?}",
                MAX_COMMANDS_PER_WINDOW, COMMAND_RATE_LIMIT_WINDOW
            ));
        }

        *count += 1;
        Ok(())
    }

    /// Validate command parameters
    fn validate_command(&self, cmd: &Command) -> Result<(), String> {
        match cmd.command_type {
            CommandType::Takeoff => {
                let alt = cmd
                    .takeoff_altitude
                    .ok_or("Takeoff altitude not specified")?;
                if alt < MIN_TAKEOFF_ALTITUDE || alt > MAX_TAKEOFF_ALTITUDE {
                    return Err(format!(
                        "Takeoff altitude must be between {} and {} meters",
                        MIN_TAKEOFF_ALTITUDE, MAX_TAKEOFF_ALTITUDE
                    ));
                }
            }
            CommandType::SetMode => {
                if cmd.set_mode_value.is_none() {
                    return Err("Mode value not specified".to_string());
                }
            }
            // Other commands have no parameters to validate
            _ => {}
        }
        Ok(())
    }

    /// Clean up rate limit entries for disconnected clients
    pub async fn cleanup_client(&self, client_id: u64) {
        self.rate_limits.write().await.remove(&client_id);
    }
}

impl Clone for ConnectionHandler {
    fn clone(&self) -> Self {
        Self {
            version_major: self.version_major,
            version_minor: self.version_minor,
            version_patch: self.version_patch,
            serial_port: self.serial_port.clone(),
            pixhawk_connected: Arc::clone(&self.pixhawk_connected),
            command_tx: self.command_tx.clone(),
            nsh_tx: self.nsh_tx.clone(),
            build_config: self.build_config.clone(),
            state_rx: self.state_rx.resubscribe(),
            rate_limits: Arc::clone(&self.rate_limits),
            next_client_id: Arc::clone(&self.next_client_id),
            shutdown_signal: Arc::clone(&self.shutdown_signal),
            recharge_fn: self.recharge_fn.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::sync::broadcast;

    async fn create_test_handler() -> (ConnectionHandler, mpsc::Receiver<ValidatedCommand>) {
        let (cmd_tx, cmd_rx) = mpsc::channel(32);
        let (state_tx, state_rx) = broadcast::channel(16);
        drop(state_tx); // We don't need to send states in these tests
        let shutdown = Arc::new(AtomicBool::new(false));

        let handler = ConnectionHandler::new(1, 0, 0, "/dev/test".to_string(), cmd_tx, state_rx, shutdown);

        (handler, cmd_rx)
    }

    #[tokio::test]
    async fn test_handshake() {
        let (handler, _) = create_test_handler().await;
        let response = handler.handle_handshake().await;

        match response {
            OutgoingMessage::HandshakeAck(ack) => {
                assert_eq!(ack.version_major, 1);
                assert_eq!(ack.version_minor, 0);
                assert!(!ack.pixhawk_connected);
                assert_eq!(ack.serial_port, "/dev/test");
            }
            _ => panic!("Expected HandshakeAck"),
        }
    }

    #[tokio::test]
    async fn test_client_id_allocation() {
        let (handler, _) = create_test_handler().await;

        let id1 = handler.allocate_client_id().await;
        let id2 = handler.allocate_client_id().await;
        let id3 = handler.allocate_client_id().await;

        assert_eq!(id1, 1);
        assert_eq!(id2, 2);
        assert_eq!(id3, 3);
    }

    #[tokio::test]
    async fn test_validate_takeoff_altitude() {
        let (handler, _) = create_test_handler().await;

        // Valid altitude
        let cmd = Command {
            command_id: 1,
            command_type: CommandType::Takeoff,
            takeoff_altitude: Some(10.0),
            set_mode_value: None,
        };
        assert!(handler.validate_command(&cmd).is_ok());

        // Too low
        let cmd = Command {
            command_id: 2,
            command_type: CommandType::Takeoff,
            takeoff_altitude: Some(0.5),
            set_mode_value: None,
        };
        assert!(handler.validate_command(&cmd).is_err());

        // Too high
        let cmd = Command {
            command_id: 3,
            command_type: CommandType::Takeoff,
            takeoff_altitude: Some(150.0),
            set_mode_value: None,
        };
        assert!(handler.validate_command(&cmd).is_err());

        // Missing altitude
        let cmd = Command {
            command_id: 4,
            command_type: CommandType::Takeoff,
            takeoff_altitude: None,
            set_mode_value: None,
        };
        assert!(handler.validate_command(&cmd).is_err());
    }

    #[tokio::test]
    async fn test_rate_limiting() {
        let (handler, _) = create_test_handler().await;
        let client_id = 1;

        // Should allow MAX_COMMANDS_PER_WINDOW commands
        for _ in 0..MAX_COMMANDS_PER_WINDOW {
            assert!(handler.check_rate_limit(client_id).await.is_ok());
        }

        // Should reject the next one
        assert!(handler.check_rate_limit(client_id).await.is_err());
    }

    #[tokio::test]
    async fn test_command_forwarding() {
        let (handler, mut cmd_rx) = create_test_handler().await;
        let client_id = handler.allocate_client_id().await;

        let cmd = Command {
            command_id: 42,
            command_type: CommandType::Arm,
            takeoff_altitude: None,
            set_mode_value: None,
        };

        let response = handler.handle_command(client_id, cmd).await;

        // Should get success ACK
        match response {
            OutgoingMessage::CommandAck(ack) => {
                assert!(ack.success);
                assert_eq!(ack.command_id, 42);
            }
            _ => panic!("Expected CommandAck"),
        }

        // Command should be forwarded
        let validated = cmd_rx.recv().await.unwrap();
        assert_eq!(validated.command.command_id, 42);
        assert_eq!(validated.command.command_type, CommandType::Arm);
        assert_eq!(validated.client_id, client_id);
    }
}
