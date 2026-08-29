//! Axum WebSocket server
//!
//! Provides the HTTP/WebSocket server that accepts connections and manages
//! client communication.

use crate::build_config::BuildConfigHandler;
use crate::handler::{ConnectionHandler, ValidatedCommand, ValidatedNshCommand};
use crate::preflight::PreflightHandler;
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
use tracing::{debug, error, info, warn};

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
    /// Terrain the physics collides against, so the server can ask the browser
    /// for what it is missing around the vehicle.
    terrain: Option<Arc<terrain::TerrainCache>>,
    /// System-initiated `ConfigResult` messages (e.g. from `repush_if_configured`
    /// on FC reconnect). Forwarded to all connected clients so the browser can
    /// display the spinner / ready state without requiring a manual re-configure.
    system_config_tx: Option<broadcast::Sender<OutgoingMessage>>,
    provisioning_tx: Option<broadcast::Sender<OutgoingMessage>>,
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
    /// Terrain the physics collides against, so the server can ask browsers for
    /// what is missing around the vehicle.
    terrain: Option<Arc<terrain::TerrainCache>>,
    /// Shutdown signal that browser can trigger
    shutdown_signal: Arc<AtomicBool>,
    /// Build configuration handler
    build_config_handler: Option<Arc<BuildConfigHandler>>,
    /// Preflight HITL/quadrotor gate handler
    preflight_handler: Option<Arc<PreflightHandler>>,
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
            terrain: None,
            shutdown_signal: Arc::new(AtomicBool::new(false)),
            build_config_handler: None,
            preflight_handler: None,
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
    pub fn set_nsh_response_receiver(
        &mut self,
        nsh_resp_rx: broadcast::Receiver<crate::protocol::NshResponse>,
    ) {
        self.nsh_resp_rx = Some(nsh_resp_rx);
    }

    /// Set the connection status receiver (for broadcasting status to clients)
    pub fn set_connection_status_receiver(
        &mut self,
        conn_status_rx: broadcast::Receiver<ConnectionStatus>,
    ) {
        self.conn_status_rx = Some(conn_status_rx);
    }

    /// Set the vehicle message receiver (for broadcasting STATUSTEXT messages to clients)
    pub fn set_vehicle_message_receiver(
        &mut self,
        vehicle_msg_rx: broadcast::Receiver<VehicleMessage>,
    ) {
        self.vehicle_msg_rx = Some(vehicle_msg_rx);
    }

    /// Set the terrain origin receiver (for broadcasting to clients)
    pub fn set_terrain_origin_receiver(
        &mut self,
        terrain_origin_rx: broadcast::Receiver<crate::protocol::TerrainOrigin>,
    ) {
        self.terrain_origin_rx = Some(terrain_origin_rx);
    }

    /// Give the server the terrain cache, so it can ask connected browsers for
    /// the tiles the physics is missing around the vehicle.
    pub fn set_terrain_cache(&mut self, terrain: Arc<terrain::TerrainCache>) {
        self.terrain = Some(terrain);
    }

    /// Set the build configuration handler
    pub fn set_build_config_handler(&mut self, handler: Arc<BuildConfigHandler>) {
        self.build_config_handler = Some(handler);
    }

    /// Set the preflight handler
    pub fn set_preflight_handler(&mut self, handler: Arc<PreflightHandler>) {
        self.preflight_handler = Some(handler);
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
                            Ok(msg) => {
                                let _ = tx_clone.send(msg);
                            }
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
        // Provisioning and restore progress reaches every connected client, so
        // a reloaded page or a second tab converges on the same state instead
        // of being told an operation is "already in progress" with nothing to
        // show for it.
        let provisioning_tx: Option<broadcast::Sender<OutgoingMessage>> =
            if let Some(preflight_handler) = self.preflight_handler {
                let rx = preflight_handler.subscribe_provisioning();
                let (tx, _) = broadcast::channel::<OutgoingMessage>(64);
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
                handler.set_snapshot_ack_sender(preflight_handler.snapshot_ack_sender());
                handler.set_preflight_handler(preflight_handler);
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
        let terrain_origin_latest: Arc<
            tokio::sync::RwLock<Option<crate::protocol::TerrainOrigin>>,
        > = Arc::new(tokio::sync::RwLock::new(None));
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

        if let Some(terrain) = &self.terrain {
            handler.set_terrain_cache(terrain.clone());
        }

        let app_state = Arc::new(AppState {
            handler,
            update_interval,
            nsh_resp_tx,
            conn_status_tx,
            vehicle_msg_tx,
            terrain_origin_tx,
            terrain_origin_latest,
            terrain: self.terrain.clone(),
            system_config_tx,
            provisioning_tx,
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
            let origins: Vec<_> = self
                .config
                .allowed_origins
                .iter()
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

        let listener = match tokio::net::TcpListener::bind(addr).await {
            Ok(listener) => listener,
            Err(e) if e.kind() == std::io::ErrorKind::AddrInUse => {
                // Naming the port and the likely cause matters more than the
                // raw errno: the overwhelmingly common case is a second daemon
                // already running, and "address in use" does not say that.
                return Err(std::io::Error::new(
                    std::io::ErrorKind::AddrInUse,
                    format!(
                        "port {} is already in use — another hitl-daemon is probably \
                         running. Stop it, or start this one with --websocket-port <PORT>.",
                        self.config.port
                    ),
                )
                .into());
            }
            Err(e) => return Err(e.into()),
        };

        // The one line a first-time user needs. Printed only once the socket is
        // actually bound, so it is never a promise the daemon cannot keep.
        info!(
            "Ready. Open {} in your browser to start the simulator.",
            simulator_url()
        );

        axum::serve(listener, app).await?;

        Ok(())
    }
}

/// Where the user should point their browser.
///
/// Overridable so a developer running the web app locally, or pointing at
/// staging, is not told to open production.
fn simulator_url() -> String {
    std::env::var("HITL_SIMULATOR_URL")
        .unwrap_or_else(|_| "https://th3seus.net/simulator".to_string())
}

/// Health check endpoint
async fn health_handler() -> impl IntoResponse {
    "OK"
}

/// Maximum allowed incoming WebSocket message size.
///
/// 1 KB was too small for the largest message this protocol defines. A restore
/// carries the board's whole parameter snapshot — 21 entries of name, value and
/// type serialise to roughly 1.2 KB — so the daemon rejected a message it had
/// asked the browser to send, at the WebSocket layer, before any handler could
/// log it. The connection was closed and the interface waited forever on a
/// restore that never started.
///
/// 64 KB then left room for a snapshot several times larger than any board's
/// current parameter set — but a restore stopped being the largest message
/// this protocol defines the moment the browser became the only party that
/// fetches terrain. One 256x256 f32 tile is 256 KB, and a collision set is
/// several of them, so *every* terrain push was killed at the transport with
/// `Space limit exceeded` and the physics ran the whole session on flat
/// ground. `TerrainTiles::MAX_FRAME_BYTES` was unreachable: axum rejected the
/// frame long before `from_bytes` could apply it.
///
/// Deriving the transport limit from the protocol's own bound is the point.
/// Two independent limits for one message is what allowed the smaller to sit
/// below the larger unnoticed, and the protocol test suite could not catch it
/// because those tests never cross a socket.
const MAX_INCOMING_MESSAGE_SIZE: usize = crate::protocol::TerrainTiles::MAX_FRAME_BYTES;

/// How often the server sends a WebSocket Ping frame to each client
const PING_INTERVAL: Duration = Duration::from_secs(5);

/// How often the daemon re-states the terrain it is missing.
///
/// The vehicle needs at least a tile of margin ahead of it, and a z14 tile is
/// ~1.9 km, so half a second is many times faster than a multirotor can outrun
/// its own coverage.
const TERRAIN_NEED_INTERVAL: Duration = Duration::from_millis(500);
/// Slower cadence once requests are going unanswered, so a browser that cannot
/// supply a tile is not asked at full rate for the rest of the session.
const TERRAIN_NEED_IDLE_INTERVAL: Duration = Duration::from_secs(5);
/// Consecutive identical unanswered requests before backing off.
const TERRAIN_NEED_BACKOFF_AFTER: u32 = 6;
/// Tile ring the physics keeps around the vehicle.
const TERRAIN_NEED_RADIUS: u32 = 1;

/// Maximum time allowed between any received message before the connection
/// is considered a zombie and closed (3 missed pings)
const PONG_TIMEOUT: Duration = Duration::from_secs(15);

/// WebSocket upgrade handler
async fn ws_handler(ws: WebSocketUpgrade, State(state): State<Arc<AppState>>) -> impl IntoResponse {
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

    // The cache this client is asked to fill.
    let terrain_state = state.terrain.clone();

    // Capabilities go out first, before any state update, so a client never
    // has to interpret a frame before it knows what this daemon speaks.
    {
        let capabilities =
            OutgoingMessage::Capabilities(crate::protocol::Capabilities::current(format!(
                "{}.{}.{}",
                handler.version_major(),
                handler.version_minor(),
                handler.version_patch()
            )));
        if ws_sender
            .send(Message::Binary(capabilities.to_bytes().into()))
            .await
            .is_err()
        {
            return;
        }
    }

    // Send cached terrain origin to late-joining client
    if let Some(origin) = *state.terrain_origin_latest.read().await {
        let msg = OutgoingMessage::TerrainOrigin(origin);
        let _ = ws_sender.send(Message::Binary(msg.to_bytes().into())).await;
    }

    // Subscribe to system-initiated config results (reconnect re-push) if available
    let mut system_config_rx = state.system_config_tx.as_ref().map(|tx| tx.subscribe());
    let mut provisioning_rx = state.provisioning_tx.as_ref().map(|tx| tx.subscribe());

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
        // Reconciliation, not RPC: the daemon re-states what it still lacks on
        // a slow tick, so a dropped frame, a tab reload or a daemon restart all
        // recover with no acknowledgement bookkeeping. Backs off to
        // TERRAIN_NEED_IDLE_INTERVAL once a request has gone unanswered, so a
        // browser that cannot supply a tile is not asked at full rate forever.
        let mut terrain_need_ticker = interval(TERRAIN_NEED_INTERVAL);
        terrain_need_ticker.tick().await;
        let mut terrain_need_unanswered: u32 = 0;
        let mut last_terrain_need: Vec<crate::protocol::WireTileCoord> = Vec::new();
        // The first tick fires immediately; skip it so the first ping goes out after
        // one full interval rather than at connection establishment.
        ping_ticker.tick().await;

        loop {
            tokio::select! {
                // Ask the browser for terrain the physics is missing.
                _ = terrain_need_ticker.tick() => {
                    if let Some(cache) = &terrain_state {
                        let coords: Vec<crate::protocol::WireTileCoord> = cache
                            .missing_around_vehicle(TERRAIN_NEED_RADIUS)
                            .into_iter()
                            .map(|c| crate::protocol::WireTileCoord { x: c.x, y: c.y, z: c.z })
                            .collect();

                        if coords.is_empty() {
                            // The steady state. Say nothing and reset the backoff.
                            terrain_need_unanswered = 0;
                            last_terrain_need.clear();
                            terrain_need_ticker = interval(TERRAIN_NEED_INTERVAL);
                            terrain_need_ticker.tick().await;
                        } else {
                            if coords == last_terrain_need {
                                terrain_need_unanswered =
                                    terrain_need_unanswered.saturating_add(1);
                                if terrain_need_unanswered == TERRAIN_NEED_BACKOFF_AFTER {
                                    debug!(
                                        tiles = coords.len(),
                                        "Terrain requests going unanswered; backing off"
                                    );
                                    terrain_need_ticker = interval(TERRAIN_NEED_IDLE_INTERVAL);
                                    terrain_need_ticker.tick().await;
                                }
                            } else {
                                // A different set: progress is being made.
                                terrain_need_unanswered = 0;
                                last_terrain_need = coords.clone();
                            }

                            let msg = OutgoingMessage::TerrainNeed(
                                crate::protocol::TerrainNeed { coords },
                            );
                            if ws_sender.send(Message::Binary(msg.to_bytes().into())).await.is_err() {
                                break;
                            }
                        }
                    }
                }

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
                // Provisioning and restore progress, fanned out to every client.
                result = async {
                    match provisioning_rx.as_mut() {
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
                            warn!(client_id, lagged = n, "Provisioning receiver lagged");
                        }
                        Err(broadcast::error::RecvError::Closed) => {
                            provisioning_rx = None;
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

#[cfg(test)]
mod message_size_tests {
    use super::MAX_INCOMING_MESSAGE_SIZE;

    /// A realistic restore payload: every parameter provisioning writes, with
    /// names and types, as the browser actually sends it.
    fn restore_frame_size(param_count: usize) -> usize {
        let params: Vec<serde_json::Value> = (0..param_count)
            .map(|i| {
                serde_json::json!({
                    // Names dominate the payload. PX4 caps them at 16
                    // characters, so a full-length name is the honest worst case.
                    "name": format!("PARAM_NAME_{i:05}"),
                    "value": 1.2345,
                    "param_type": "real32",
                })
            })
            .collect();
        // Built as the browser builds it, rather than by serialising the
        // incoming type — which is deserialize-only, and whose field order the
        // browser does not have to match anyway.
        let body = serde_json::json!({
            "board_identity": "uid:3034510f33323831",
            "params": params,
        });
        // 1 type byte + JSON body, matching the browser's framing.
        1 + serde_json::to_vec(&body)
            .expect("restore request serialises")
            .len()
    }

    #[test]
    fn a_full_restore_fits_within_the_incoming_message_limit() {
        // The limit was 1 KB and a 21-parameter restore is ~1.2 KB, so the
        // daemon rejected a message it had asked the browser to send — at the
        // WebSocket layer, before any handler could log it. The connection was
        // closed and the interface waited forever on a restore that never
        // started.
        let size = restore_frame_size(21);
        assert!(
            size > 1024,
            "the payload that broke this must still exceed the old 1 KB limit, \
             or this test no longer guards anything (was {size} bytes)"
        );
        assert!(
            size < MAX_INCOMING_MESSAGE_SIZE,
            "a 21-parameter restore ({size} bytes) must fit in \
             MAX_INCOMING_MESSAGE_SIZE ({MAX_INCOMING_MESSAGE_SIZE})"
        );
    }

    #[test]
    fn the_limit_leaves_room_for_a_far_larger_snapshot() {
        // Provisioning writes ~21 parameters today. A future one that writes
        // ten times as many must not silently hit the same wall.
        assert!(restore_frame_size(210) < MAX_INCOMING_MESSAGE_SIZE);
    }

    /// The transport must accept every frame the protocol says it will accept.
    ///
    /// It did not: the cap sat at 64 KB while a single terrain tile is 256 KB,
    /// so the browser's terrain push was rejected by axum before any handler
    /// ran, and the physics silently fell back to flat ground for a whole
    /// session. `TerrainTiles::from_bytes` has its own limit and a test for it,
    /// but that test hands bytes straight to the parser and never crosses a
    /// socket, so it could not see the smaller wall in front of it.
    #[test]
    fn the_transport_accepts_every_frame_the_protocol_defines() {
        use crate::protocol::TerrainTiles;
        assert!(
            MAX_INCOMING_MESSAGE_SIZE >= TerrainTiles::MAX_FRAME_BYTES,
            "transport cap ({MAX_INCOMING_MESSAGE_SIZE}) is below the terrain \
             frame limit the protocol advertises ({}) — frames the parser would \
             accept die at the socket instead",
            TerrainTiles::MAX_FRAME_BYTES
        );
    }

    /// The specific size that was being dropped, stated concretely so the
    /// regression is legible without reconstructing the arithmetic.
    #[test]
    fn a_single_terrain_tile_fits() {
        const TILE_SAMPLES: usize = 256 * 256;
        // 1 tag byte + 4 header-length bytes + a JSON header + the payload.
        let one_tile = 1 + 4 + 256 + TILE_SAMPLES * 4;
        assert!(
            one_tile > 64 * 1024,
            "a tile that fits in the old 64 KB limit would mean this test \
             guards nothing (was {one_tile} bytes)"
        );
        assert!(
            one_tile < MAX_INCOMING_MESSAGE_SIZE,
            "one 256x256 f32 tile ({one_tile} bytes) must fit"
        );
    }
}
