//! Axum WebSocket server
//!
//! Provides the HTTP/WebSocket server that accepts connections and manages
//! client communication.

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
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{broadcast, mpsc};
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
        }
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
        serial_port: String,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let state_rx = self.state_tx.subscribe();
        let mut handler = ConnectionHandler::new(
            version_major,
            version_minor,
            serial_port,
            self.command_tx.clone(),
            state_rx,
        );

        // Enable NSH support (always available; FC availability is tracked separately)
        if let Some(nsh_tx) = self.nsh_tx {
            handler.set_nsh_sender(nsh_tx);
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
                loop {
                    match rx.recv().await {
                        Ok(msg) => {
                            let _ = tx_clone.send(msg);
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

/// WebSocket upgrade handler
async fn ws_handler(
    ws: WebSocketUpgrade,
    State(state): State<Arc<AppState>>,
) -> impl IntoResponse {
    ws.on_upgrade(|socket| handle_socket(socket, state))
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

    // Channel for sending responses from the receive task to the send task
    let (response_tx, mut response_rx) = mpsc::channel::<OutgoingMessage>(32);

    // Task to send state updates and responses to client
    let send_task = tokio::spawn(async move {
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
            }
        }
    });

    // Handle incoming messages
    let recv_handler = handler.clone();
    while let Some(msg) = ws_receiver.next().await {
        match msg {
            Ok(Message::Binary(data)) => {
                match recv_handler.handle_message(client_id, &data).await {
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
            Ok(_) => {
                // Ignore text messages, pongs, etc.
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
