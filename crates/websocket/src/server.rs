//! Axum WebSocket server
//!
//! Provides the HTTP/WebSocket server that accepts connections and manages
//! client communication.

use crate::build_config::BuildConfigHandler;
use crate::handler::{ConnectionHandler, ValidatedCommand, ValidatedNshCommand};
use crate::protocol::{ConnectionStatus, OutgoingMessage, StateUpdate, VehicleMessage};
use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
    },
    response::IntoResponse,
    routing::get,
    Router,
};
use futures_util::{SinkExt, StreamExt};
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::sync::{broadcast, mpsc};
use tokio::time::interval;
use tower_http::cors::{Any, CorsLayer};
use tracing::{error, info, warn};

/// WebSocket server configuration
#[derive(Debug, Clone)]
pub struct WebSocketServerConfig {
    /// Port to listen on
    pub port: u16,
    /// Update rate in Hz
    pub update_rate_hz: u32,
    /// CORS allowed origins (empty = allow localhost only)
    pub allowed_origins: Vec<String>,
}

impl Default for WebSocketServerConfig {
    fn default() -> Self {
        Self {
            port: 9876,
            update_rate_hz: 30,
            allowed_origins: vec![],
        }
    }
}

/// Shared state for the WebSocket server
struct AppState {
    handler: ConnectionHandler,
    #[allow(dead_code)]
    update_interval: Duration,
    /// NSH response sender for broadcasting to clients
    nsh_resp_tx: Option<broadcast::Sender<crate::protocol::NshResponse>>,
    /// Connection status sender for broadcasting to clients
    conn_status_tx: Option<broadcast::Sender<ConnectionStatus>>,
    /// Vehicle message sender for broadcasting to clients
    vehicle_msg_tx: Option<broadcast::Sender<VehicleMessage>>,
    /// Terrain origin sender for broadcasting to clients
    terrain_origin_tx: Option<broadcast::Sender<crate::protocol::TerrainOrigin>>,
    /// Cached latest terrain origin (sent to late-joining clients)
    terrain_origin_latest: Arc<tokio::sync::RwLock<Option<crate::protocol::TerrainOrigin>>>,
    /// System-initiated `ConfigResult` messages (e.g. from `repush_if_configured`
    /// on FC reconnect). Forwarded to all connected clients so the browser can
    /// display the spinner / ready state without requiring a manual re-configure.
    system_config_tx: Option<broadcast::Sender<OutgoingMessage>>,
}

/// WebSocket server for browser communication
pub struct WebSocketServer {
    config: WebSocketServerConfig,
    /// Channel to send state updates to clients
    state_tx: broadcast::Sender<StateUpdate>,
    /// Channel to receive validated commands from clients
    command_rx: mpsc::Receiver<ValidatedCommand>,
    /// Channel sender for commands (passed to handler)
    command_tx: mpsc::Sender<ValidatedCommand>,
    /// Channel sender for NSH commands (optional, passed to handler)
    nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>,
    /// Channel to receive NSH responses for broadcasting to clients
    nsh_resp_rx: Option<broadcast::Receiver<crate::protocol::NshResponse>>,
    /// Channel to receive connection status updates for broadcasting to clients
    conn_status_rx: Option<broadcast::Receiver<ConnectionStatus>>,
    /// Channel to receive vehicle messages (STATUSTEXT) for broadcasting to clients
    vehicle_msg_rx: Option<broadcast::Receiver<VehicleMessage>>,
    /// Channel to receive terrain origin for broadcasting to clients
    terrain_origin_rx: Option<broadcast::Receiver<crate::protocol::TerrainOrigin>>,
    /// Shutdown signal that browser can trigger
    shutdown_signal: Arc<AtomicBool>,
    /// Build configuration handler
    build_config_handler: Option<Arc<BuildConfigHandler>>,
    /// Recharge callback (for recharge command)
    recharge_fn: Option<crate::handler::RechargeCallback>,
}

impl WebSocketServer {
    /// Create a new WebSocket server
    pub fn new(config: WebSocketServerConfig) -> Self {
        let (state_tx, _) = broadcast::channel(64);
        let (command_tx, command_rx) = mpsc::channel(64);

        Self {
            config,
            state_tx,
            command_rx,
            command_tx,
            nsh_tx: None,
            nsh_resp_rx: None,
            conn_status_rx: None,
            vehicle_msg_rx: None,
            terrain_origin_rx: None,
            shutdown_signal: Arc::new(AtomicBool::new(false)),
            build_config_handler: None,
            recharge_fn: None,
        }
    }

    /// Get the shutdown signal (set to true when a browser sends 0x07 shutdown command)
    pub fn shutdown_signal(&self) -> Arc<AtomicBool> {
        Arc::clone(&self.shutdown_signal)
    }

    /// Set the NSH command channel (enables NSH support)
    pub fn set_nsh_sender(&mut self, nsh_tx: mpsc::Sender<ValidatedNshCommand>) {
        self.nsh_tx = Some(nsh_tx);
    }

    /// Set the NSH response receiver (for broadcasting responses to clients)
    pub fn set_nsh_response_receiver(&mut self, nsh_resp_rx: broadcast::Receiver<crate::protocol::NshResponse>) {
        self.nsh_resp_rx = Some(nsh_resp_rx);
    }

    /// Set the connection status receiver (for broadcasting status to clients)
    pub fn set_connection_status_receiver(&mut self, conn_status_rx: broadcast::Receiver<ConnectionStatus>) {
        self.conn_status_rx = Some(conn_status_rx);
    }

    /// Set the vehicle message receiver (for broadcasting STATUSTEXT messages to clients)
    pub fn set_vehicle_message_receiver(&mut self, vehicle_msg_rx: broadcast::Receiver<VehicleMessage>) {
        self.vehicle_msg_rx = Some(vehicle_msg_rx);
    }

    /// Set the terrain origin receiver (for broadcasting to clients)
    pub fn set_terrain_origin_receiver(&mut self, terrain_origin_rx: broadcast::Receiver<crate::protocol::TerrainOrigin>) {
        self.terrain_origin_rx = Some(terrain_origin_rx);
    }

    /// Set the build configuration handler
    pub fn set_build_config_handler(&mut self, handler: Arc<BuildConfigHandler>) {
        self.build_config_handler = Some(handler);
    }

    /// Set the battery recharge callback
    pub fn set_recharge_callback(&mut self, callback: crate::handler::RechargeCallback) {
        self.recharge_fn = Some(callback);
    }

    /// Get a sender for broadcasting state updates
    pub fn state_sender(&self) -> broadcast::Sender<StateUpdate> {
        self.state_tx.clone()
    }

    /// Take the command receiver (can only be called once)
    pub fn take_command_receiver(&mut self) -> mpsc::Receiver<ValidatedCommand> {
        let (new_tx, new_rx) = mpsc::channel(64);
        let old_rx = std::mem::replace(&mut self.command_rx, new_rx);
        self.command_tx = new_tx;
        old_rx
    }

    /// Run the WebSocket server
    pub async fn run(
        self,
        version_major: u8,
        version_minor: u8,
        version_patch: u8,
        serial_port: String,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let state_rx = self.state_tx.subscribe();
        let mut handler = ConnectionHandler::new(
            version_major,
            version_minor,
            version_patch,
            serial_port,
            self.command_tx.clone(),
            state_rx,
            self.shutdown_signal.clone(),
        );

        // Enable NSH support (always available; FC availability is tracked separately)
        if let Some(nsh_tx) = self.nsh_tx {
            handler.set_nsh_sender(nsh_tx);
        }
        // Enable build config handler if set; subscribe to its system-config broadcast
        // so reconnect-triggered ConfigResult messages reach all connected clients.
        let system_config_tx: Option<broadcast::Sender<OutgoingMessage>> =
            if let Some(build_config_handler) = self.build_config_handler {
                let rx = build_config_handler.subscribe_system_config();
                // Bridge the broadcast::Receiver into a new broadcast::Sender so
                // handle_socket can subscribe independently per client.
                let (tx, _) = broadcast::channel::<OutgoingMessage>(16);
                let tx_clone = tx.clone();
                tokio::spawn(async move {
                    let mut rx = rx;
                    loop {
                        match rx.recv().await {
                            Ok(msg) => { let _ = tx_clone.send(msg); }
                            Err(broadcast::error::RecvError::Closed) => break,
                            Err(broadcast::error::RecvError::Lagged(_)) => continue,
                        }
                    }
                });
                handler.set_build_config_handler(build_config_handler);
                Some(tx)
            } else {
                None
            };
        // Enable recharge callback
        if let Some(recharge_fn) = self.recharge_fn {
            handler.set_recharge_callback(recharge_fn);
        }
        // Start with FC disconnected — connection manager will update via ConnectionStatus
        handler.set_pixhawk_connected(false).await;

        let update_interval = Duration::from_secs_f64(1.0 / self.config.update_rate_hz as f64);

        // Create NSH response broadcast sender if we have a receiver
        let nsh_resp_tx = self.nsh_resp_rx.map(|rx| {
            // Create a new broadcast channel and forward from the receiver
            let (tx, _) = broadcast::channel(64);
            let tx_clone = tx.clone();
            tokio::spawn(async move {
                let mut rx = rx;
                loop {
                    match rx.recv().await {
                        Ok(resp) => {
                            let _ = tx_clone.send(resp);
                        }
                        Err(broadcast::error::RecvError::Closed) => break,
                        Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    }
                }
            });
            tx
        });

        // Create connection status broadcast sender if we have a receiver
        let handler_for_status = handler.clone();
        let conn_status_tx = self.conn_status_rx.map(|rx| {
            let (tx, _) = broadcast::channel(16);
            let tx_clone = tx.clone();
            let handler_clone = handler_for_status.clone();
            tokio::spawn(async move {
                let mut rx = rx;
                loop {
                    match rx.recv().await {
                        Ok(status) => {
                            info!(
                                connected = status.connected,
                                reconnecting = status.reconnecting,
                                retry_count = status.retry_count,
                                "Broadcasting connection status to clients"
                            );
                            // Update handler's FC connection status for new client handshakes
                            handler_clone.set_pixhawk_connected(status.connected).await;
                            let _ = tx_clone.send(status);
                        }
                        Err(broadcast::error::RecvError::Closed) => break,
                        Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    }
                }
            });
            tx
        });

        // Create vehicle message broadcast sender if we have a receiver
        let vehicle_msg_tx = self.vehicle_msg_rx.map(|rx| {
            let (tx, _) = broadcast::channel(64);
            let tx_clone = tx.clone();
            tokio::spawn(async move {
                let mut rx = rx;
                info!("Vehicle message forwarder task started");
                loop {
                    match rx.recv().await {
                        Ok(msg) => {
                            info!(severity = msg.severity, text = %msg.text, "Forwarding vehicle message to clients");
                            let _ = tx_clone.send(msg);
                        }
                        Err(broadcast::error::RecvError::Closed) => {
                            info!("Vehicle message channel closed");
                            break;
                        }
                        Err(broadcast::error::RecvError::Lagged(n)) => {
                            warn!(lagged = n, "Vehicle message receiver lagged");
                            continue;
                        }
                    }
                }
            });
            tx
        });

        // Create terrain origin broadcast sender if we have a receiver.
        // Also cache the latest value so late-joining clients receive it.
        let terrain_origin_latest: Arc<tokio::sync::RwLock<Option<crate::protocol::TerrainOrigin>>> =
            Arc::new(tokio::sync::RwLock::new(None));
        let terrain_origin_tx = self.terrain_origin_rx.map(|rx| {
            let (tx, _) = broadcast::channel(4);
            let tx_clone = tx.clone();
            let cache = terrain_origin_latest.clone();
            tokio::spawn(async move {
                let mut rx = rx;
                loop {
                    match rx.recv().await {
                        Ok(origin) => {
                            *cache.write().await = Some(origin);
                            let _ = tx_clone.send(origin);
                        }
                        Err(broadcast::error::RecvError::Closed) => break,
                        Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    }
                }
            });
            tx
        });

        let app_state = Arc::new(AppState {
            handler,
            update_interval,
            nsh_resp_tx,
            conn_status_tx,
            vehicle_msg_tx,
            terrain_origin_tx,
            terrain_origin_latest,
            system_config_tx,
        });

        // Build CORS layer — restrict origins in production, allow localhost for dev
        let cors = if self.config.allowed_origins.is_empty() {
            CorsLayer::new()
                .allow_origin([
                    "http://localhost:3000".parse().unwrap(),
                    "http://localhost:5173".parse().unwrap(),
                    "http://127.0.0.1:3000".parse().unwrap(),
                    "http://127.0.0.1:5173".parse().unwrap(),
                ])
                .allow_methods(Any)
                .allow_headers(Any)
        } else {
            let origins: Vec<_> = self.config.allowed_origins.iter()
                .filter_map(|o| o.parse().ok())
                .collect();
            CorsLayer::new()
                .allow_origin(origins)
                .allow_methods(Any)
                .allow_headers(Any)
        };

        let app = Router::new()
            .route("/ws", get(ws_handler))
            .route("/health", get(health_handler))
            .layer(cors)
            .with_state(app_state);

        let addr = SocketAddr::from(([0, 0, 0, 0], self.config.port));
        info!(port = self.config.port, "Starting WebSocket server");

        let listener = tokio::net::TcpListener::bind(addr).await?;
        axum::serve(listener, app).await?;

        Ok(())
    }
}

/// Health check endpoint
async fn health_handler() -> impl IntoResponse {
    "OK"
}

/// Maximum allowed incoming WebSocket message size (1 KB)
const MAX_INCOMING_MESSAGE_SIZE: usize = 1024;

/// How often the server sends a WebSocket Ping frame to each client
const PING_INTERVAL: Duration = Duration::from_secs(5);

/// Maximum time allowed between any received message before the connection
/// is considered a zombie and closed (3 missed pings)
const PONG_TIMEOUT: Duration = Duration::from_secs(15);

/// WebSocket upgrade handler
async fn ws_handler(
    ws: WebSocketUpgrade,
    State(state): State<Arc<AppState>>,
) -> impl IntoResponse {
    ws.max_message_size(MAX_INCOMING_MESSAGE_SIZE)
        .on_upgrade(|socket| handle_socket(socket, state))
}

/// Handle a WebSocket connection
async fn handle_socket(socket: WebSocket, state: Arc<AppState>) {
    let client_id = state.handler.allocate_client_id().await;
    info!(client_id, "Client connected");

    let (mut ws_sender, mut ws_receiver) = socket.split();
    let handler = state.handler.clone();
    let mut state_rx = handler.subscribe_state();

    // Subscribe to NSH responses if available
    let mut nsh_resp_rx = state.nsh_resp_tx.as_ref().map(|tx| tx.subscribe());

    // Subscribe to connection status updates if available
    let mut conn_status_rx = state.conn_status_tx.as_ref().map(|tx| tx.subscribe());

    // Subscribe to vehicle messages if available
    let mut vehicle_msg_rx = state.vehicle_msg_tx.as_ref().map(|tx| tx.subscribe());

    // Subscribe to terrain origin if available
    let mut terrain_origin_rx = state.terrain_origin_tx.as_ref().map(|tx| tx.subscribe());

    // Send cached terrain origin to late-joining client
    if let Some(origin) = *state.terrain_origin_latest.read().await {
        let msg = OutgoingMessage::TerrainOrigin(origin);
        let _ = ws_sender.send(Message::Binary(msg.to_bytes().into())).await;
    }

    // Subscribe to system-initiated config results (reconnect re-push) if available
    let mut system_config_rx = state.system_config_tx.as_ref().map(|tx| tx.subscribe());

    // Channel for sending responses from the receive task to the send task
    let (response_tx, mut response_rx) = mpsc::channel::<OutgoingMessage>(32);

    // Shared last-activity timestamp (seconds since UNIX_EPOCH).
    // Updated by the receive task on every incoming frame; checked by the send task.
    let now_secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let last_pong_ts = Arc::new(AtomicU64::new(now_secs));
    let last_pong_ts_recv = Arc::clone(&last_pong_ts);

    // Task to send state updates and responses to client
    let send_task = tokio::spawn(async move {
        let mut ping_ticker = interval(PING_INTERVAL);
        // The first tick fires immediately; skip it so the first ping goes out after
        // one full interval rather than at connection establishment.
        ping_ticker.tick().await;

        loop {
            tokio::select! {
                // Handle state updates from broadcast
                result = state_rx.recv() => {
                    match result {
                        Ok(state_update) => {
                            let msg = OutgoingMessage::StateUpdate(state_update);
                            let bytes = msg.to_bytes();
                            if ws_sender.send(Message::Binary(bytes.into())).await.is_err() {
                                break;
                            }
                        }
                        Err(broadcast::error::RecvError::Lagged(n)) => {
                            warn!(client_id, lagged = n, "State receiver lagged, skipping frames");
                        }
                        Err(broadcast::error::RecvError::Closed) => {
                            break;
                        }
                    }
                }
                // Handle responses from command processing
                Some(response) = response_rx.recv() => {
                    let bytes = response.to_bytes();
                    if ws_sender.send(Message::Binary(bytes.into())).await.is_err() {
                        break;
                    }
                }
                // Handle NSH responses (if available)
                result = async {
                    match nsh_resp_rx.as_mut() {
                        Some(rx) => rx.recv().await,
                        None => std::future::pending().await,
                    }
                } => {
                    match result {
                        Ok(nsh_resp) => {
                            info!(client_id, request_id = nsh_resp.request_id, "Sending NSH response to client");
                            let msg = OutgoingMessage::NshResponse(nsh_resp);
                            let bytes = msg.to_bytes();
                            if ws_sender.send(Message::Binary(bytes.into())).await.is_err() {
                                break;
                            }
                        }
                        Err(broadcast::error::RecvError::Lagged(n)) => {
                            warn!(client_id, lagged = n, "NSH response receiver lagged");
                        }
                        Err(broadcast::error::RecvError::Closed) => {
                            // NSH channel closed, continue without it
                            nsh_resp_rx = None;
                        }
                    }
                }
                // Handle connection status updates (if available)
                result = async {
                    match conn_status_rx.as_mut() {
                        Some(rx) => rx.recv().await,
                        None => std::future::pending().await,
                    }
                } => {
                    match result {
                        Ok(status) => {
                            info!(
                                client_id,
                                connected = status.connected,
                                reconnecting = status.reconnecting,
                                "Sending connection status to client"
                            );
                            let msg = OutgoingMessage::ConnectionStatus(status);
                            let bytes = msg.to_bytes();
                            if ws_sender.send(Message::Binary(bytes.into())).await.is_err() {
                                break;
                            }
                        }
                        Err(broadcast::error::RecvError::Lagged(n)) => {
                            warn!(client_id, lagged = n, "Connection status receiver lagged");
                        }
                        Err(broadcast::error::RecvError::Closed) => {
                            conn_status_rx = None;
                        }
                    }
                }
                // Handle vehicle messages (if available)
                result = async {
                    match vehicle_msg_rx.as_mut() {
                        Some(rx) => rx.recv().await,
                        None => std::future::pending().await,
                    }
                } => {
                    match result {
                        Ok(msg) => {
                            info!(client_id, severity = msg.severity, text = %msg.text, "Sending vehicle message to client");
                            let outgoing = OutgoingMessage::VehicleMessage(msg);
                            let bytes = outgoing.to_bytes();
                            if ws_sender.send(Message::Binary(bytes.into())).await.is_err() {
                                break;
                            }
                        }
                        Err(broadcast::error::RecvError::Lagged(n)) => {
                            warn!(client_id, lagged = n, "Vehicle message receiver lagged");
                        }
                        Err(broadcast::error::RecvError::Closed) => {
                            vehicle_msg_rx = None;
                        }
                    }
                }
                // Handle terrain origin updates (event-driven, rare)
                result = async {
                    match terrain_origin_rx.as_mut() {
                        Some(rx) => rx.recv().await,
                        None => std::future::pending().await,
                    }
                } => {
                    match result {
                        Ok(origin) => {
                            let msg = OutgoingMessage::TerrainOrigin(origin);
                            let bytes = msg.to_bytes();
                            if ws_sender.send(Message::Binary(bytes.into())).await.is_err() {
                                break;
                            }
                        }
                        Err(broadcast::error::RecvError::Lagged(_)) => {}
                        Err(broadcast::error::RecvError::Closed) => {
                            terrain_origin_rx = None;
                        }
                    }
                }
                // Handle system-initiated config results (e.g. reconnect re-push)
                result = async {
                    match system_config_rx.as_mut() {
                        Some(rx) => rx.recv().await,
                        None => std::future::pending().await,
                    }
                } => {
                    match result {
                        Ok(msg) => {
                            let bytes = msg.to_bytes();
                            if ws_sender.send(Message::Binary(bytes.into())).await.is_err() {
                                break;
                            }
                        }
                        Err(broadcast::error::RecvError::Lagged(n)) => {
                            warn!(client_id, lagged = n, "System config receiver lagged");
                        }
                        Err(broadcast::error::RecvError::Closed) => {
                            system_config_rx = None;
                        }
                    }
                }
                // Periodic ping — keep-alive heartbeat
                _ = ping_ticker.tick() => {
                    // Check for zombie connection before sending the ping
                    let last_ts = last_pong_ts.load(Ordering::Relaxed);
                    let now_ts = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_secs();
                    let elapsed_secs = now_ts.saturating_sub(last_ts);
                    if elapsed_secs >= PONG_TIMEOUT.as_secs() {
                        warn!(
                            client_id,
                            elapsed_secs,
                            "No pong received within timeout — closing zombie connection"
                        );
                        break;
                    }
                    if ws_sender.send(Message::Ping(vec![].into())).await.is_err() {
                        break;
                    }
                }
            }
        }
    });

    // Handle incoming messages
    let recv_handler = handler.clone();
    while let Some(msg) = ws_receiver.next().await {
        // Any frame from the client proves liveness — update the timestamp.
        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();
        last_pong_ts_recv.store(ts, Ordering::Relaxed);

        match msg {
            Ok(Message::Binary(data)) => {
                match recv_handler
                    .handle_message(client_id, &data, &response_tx)
                    .await
                {
                    Ok(Some(response)) => {
                        // Send response via channel to the send task
                        if response_tx.send(response).await.is_err() {
                            break;
                        }
                    }
                    Ok(None) => {}
                    Err(e) => {
                        warn!(client_id, error = %e, "Failed to handle message");
                    }
                }
            }
            Ok(Message::Close(_)) => {
                info!(client_id, "Client requested close");
                break;
            }
            Ok(Message::Ping(_)) => {
                // Pong is handled automatically by axum-ws
            }
            Ok(Message::Pong(_)) => {
                // Liveness already recorded above via last_pong_ts_recv
            }
            Ok(_) => {
                // Ignore text messages, etc.
            }
            Err(e) => {
                error!(client_id, error = %e, "WebSocket error");
                break;
            }
        }
    }

    // Clean up
    send_task.abort();
    handler.cleanup_client(client_id).await;
    info!(client_id, "Client disconnected");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = WebSocketServerConfig::default();
        assert_eq!(config.port, 9876);
        assert_eq!(config.update_rate_hz, 30);
        assert!(config.allowed_origins.is_empty());
    }

    #[test]
    fn test_server_creation() {
        let config = WebSocketServerConfig {
            port: 8080,
            update_rate_hz: 60,
            allowed_origins: vec!["http://localhost:3000".to_string()],
        };

        let server = WebSocketServer::new(config);
        assert_eq!(server.config.port, 8080);
        assert_eq!(server.config.update_rate_hz, 60);
    }
}
