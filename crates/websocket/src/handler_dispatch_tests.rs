//! Work that runs off the receive loop: the byte-8 reset (acknowledged first)
//! and ConfigureBuild.

use super::tests::create_test_handler;
use super::*;
use crate::fc_session::FcSession;
use crate::protocol::{
    ConfigState, NshCommand, Px4HealthReport, ResetResult, ResetStage, ResetState,
    MSG_TYPE_COMMAND, MSG_TYPE_CONFIGURE_BUILD,
};
use crate::test_support::{fake_nsh, spawn_catalogue, spawn_fake_sim_loop, FakeReply, FAST_TIMING};
use simulation::{Px4HealthMonitor, SimulationConfig, SimulationState};
use tokio::time::Instant;

/// A reset command frame: id 7, `ResetVehicle`.
const RESET: [u8; 6] = [MSG_TYPE_COMMAND, 7, 0, 0, 0, 8];

/// Capacity of the progress channel; a reset sends a few results at most.
const PROGRESS_CAPACITY: usize = 16;

const RECV_TIMEOUT: Duration = Duration::from_secs(2);

/// How long the fake catalogue holds each response: long enough that an apply
/// awaited on the receive loop would be seen blocking it.
const CATALOGUE_DELAY: Duration = Duration::from_millis(500);

/// A handler whose reset runs against `session`. A reset never sends a config;
/// the receiver is only here to build the handler.
async fn with_session(session: FcSession) -> (ConnectionHandler, mpsc::Receiver<ValidatedCommand>) {
    let (mut handler, cmd_rx) = create_test_handler().await;
    let (config_tx, _config_rx) = crossbeam_channel::bounded(PROGRESS_CAPACITY);
    let build = BuildConfigHandler::new(config_tx, None, None, None, (0.0, 0.0, 0.0), None)
        .with_fc_session(
            Arc::new(session),
            SimulationState::new(SimulationConfig::default()),
        );
    handler.set_build_config_handler(Arc::new(build));
    (handler, cmd_rx)
}

/// A handler whose reset runs against a sim-only session (no board to restart).
async fn with_reset() -> (ConnectionHandler, mpsc::Receiver<ValidatedCommand>) {
    with_session(FcSession::new(None, Arc::new(Px4HealthMonitor::new()))).await
}

/// A handler whose reset meets a board that never answers NSH.
async fn with_silent_board() -> (ConnectionHandler, mpsc::Receiver<ValidatedCommand>) {
    let board = fake_nsh(Arc::default(), |_| FakeReply::Silent);
    let session =
        FcSession::new(Some(board), Arc::new(Px4HealthMonitor::new())).with_timing(FAST_TIMING);
    with_session(session).await
}

/// A handler whose ConfigureBuild runs sim-only, against a fake simulation
/// loop and a catalogue that takes `CATALOGUE_DELAY` to answer.
async fn with_slow_configure() -> (ConnectionHandler, mpsc::Receiver<ValidatedCommand>) {
    let (mut handler, cmd_rx) = create_test_handler().await;
    let (config_tx, config_rx) = crossbeam_channel::bounded(PROGRESS_CAPACITY);
    let sim = SimulationState::new(SimulationConfig::default());
    spawn_fake_sim_loop(config_rx, sim.clone(), Arc::default());
    let session = Arc::new(FcSession::new(None, Arc::new(Px4HealthMonitor::new())));
    let catalogue = vec![("motor", serde_json::json!({"kvRating": 1950.0}))];
    let build = BuildConfigHandler::new(config_tx, None, None, None, (0.0, 0.0, 0.0), None)
        .with_fc_session(session, sim)
        .with_api_url(spawn_catalogue(catalogue, CATALOGUE_DELAY).await);
    handler.set_build_config_handler(Arc::new(build));
    (handler, cmd_rx)
}

/// A ConfigureBuild frame for the catalogue's only motor.
fn configure_bytes() -> Vec<u8> {
    let mut bytes = vec![MSG_TYPE_CONFIGURE_BUILD];
    bytes.extend_from_slice(
        br#"{"motor_slug": "motor", "prop_diameter_inches": 5.0, "frame_weight_g": 300.0}"#,
    );
    bytes
}

/// A console command. The test connection has no NSH sender, so it is
/// answered at once.
fn nsh_bytes() -> Vec<u8> {
    NshCommand {
        request_id: 5,
        timeout_ms: 0,
        command: "ver all".into(),
    }
    .to_bytes()
}

fn progress() -> (
    mpsc::Sender<OutgoingMessage>,
    mpsc::Receiver<OutgoingMessage>,
) {
    mpsc::channel(PROGRESS_CAPACITY)
}

async fn recv(rx: &mut mpsc::Receiver<OutgoingMessage>) -> OutgoingMessage {
    tokio::time::timeout(RECV_TIMEOUT, rx.recv())
        .await
        .expect("a message within 2 s")
        .expect("channel open")
}

/// Receives until a `ConfigResult` that is ready or an error.
async fn next_terminal_config_result(
    rx: &mut mpsc::Receiver<OutgoingMessage>,
    within: Duration,
) -> ConfigResult {
    let terminal = async {
        loop {
            match rx.recv().await.expect("channel open") {
                OutgoingMessage::ConfigResult(r)
                    if !matches!(r.state, ConfigState::Configuring) =>
                {
                    return r
                }
                _ => {}
            }
        }
    };
    tokio::time::timeout(within, terminal)
        .await
        .unwrap_or_else(|_| panic!("no terminal ConfigResult within {within:?}"))
}

/// Receives until a `ResetResult` that is ready or an error.
async fn next_terminal_reset_result(
    rx: &mut mpsc::Receiver<OutgoingMessage>,
    within: Duration,
) -> ResetResult {
    let terminal = async {
        loop {
            match rx.recv().await.expect("channel open") {
                OutgoingMessage::ResetResult(r) if !matches!(r.state, ResetState::Resetting) => {
                    return r
                }
                _ => {}
            }
        }
    };
    tokio::time::timeout(within, terminal)
        .await
        .unwrap_or_else(|_| panic!("no terminal ResetResult within {within:?}"))
}

#[tokio::test]
async fn a_reset_command_is_acked_before_any_reset_result() {
    let (handler, _cmd_rx) = with_reset().await;
    let (tx, mut rx) = progress();
    assert!(handler
        .handle_message(1, &RESET, &tx)
        .await
        .unwrap()
        .is_none());
    assert!(matches!(
        recv(&mut rx).await,
        OutgoingMessage::CommandAck(CommandAck {
            command_id: 7,
            success: true,
            ..
        })
    ));
    assert!(matches!(
        recv(&mut rx).await,
        OutgoingMessage::ResetResult(r) if r == ResetResult::resetting(ResetStage::ResettingVehicle)
    ));
    assert!(matches!(
        recv(&mut rx).await,
        OutgoingMessage::ResetResult(r) if r == ResetResult::ready(Px4HealthReport::NotApplicable)
    ));
}

#[tokio::test]
async fn a_reset_without_a_build_config_handler_is_acked_and_reported_unavailable() {
    let (handler, _cmd_rx) = create_test_handler().await;
    let (tx, mut rx) = progress();
    assert!(handler
        .handle_message(1, &RESET, &tx)
        .await
        .unwrap()
        .is_none());
    assert!(matches!(
        recv(&mut rx).await,
        OutgoingMessage::CommandAck(CommandAck { success: true, .. })
    ));
    assert!(matches!(
        recv(&mut rx).await,
        OutgoingMessage::ResetResult(r) if r == ResetResult::failed("Reset is unavailable in this session")
    ));
}

#[tokio::test]
async fn a_reset_is_never_forwarded_to_the_command_loop() {
    let (handler, mut cmd_rx) = with_reset().await;
    let (tx, _rx) = progress();
    handler.handle_message(1, &RESET, &tx).await.unwrap();
    assert!(cmd_rx.try_recv().is_err());
}

#[tokio::test]
async fn configure_build_is_dispatched_off_the_receive_loop() {
    let (handler, _cmd_rx) = with_slow_configure().await;
    let (tx, mut rx) = progress();
    let started = Instant::now();
    assert!(handler
        .handle_message(1, &configure_bytes(), &tx)
        .await
        .unwrap()
        .is_none());
    assert!(started.elapsed() < Duration::from_millis(200));
    // The same connection's console command is answered while the apply runs.
    assert!(matches!(
        handler.handle_message(1, &nsh_bytes(), &tx).await.unwrap(),
        Some(OutgoingMessage::NshResponse(_))
    ));
    let terminal = next_terminal_config_result(&mut rx, Duration::from_secs(5)).await;
    assert!(terminal.success, "{:?}", terminal.error);
    assert!(started.elapsed() >= CATALOGUE_DELAY);
}

/// A guard rather than a test of new code: the reset has run off the receive
/// loop since it was added. It was seen failing with the reset awaited inline.
#[tokio::test]
async fn a_running_reset_does_not_block_the_receive_loop() {
    let (handler, _cmd_rx) = with_silent_board().await;
    let (tx, mut rx) = progress();
    let started = Instant::now();
    assert!(handler
        .handle_message(1, &[MSG_TYPE_COMMAND, 9, 0, 0, 0, 8], &tx)
        .await
        .unwrap()
        .is_none());
    assert!(matches!(
        handler.handle_message(1, &nsh_bytes(), &tx).await.unwrap(),
        Some(OutgoingMessage::NshResponse(_))
    ));
    assert!(started.elapsed() < Duration::from_millis(100));
    let terminal = next_terminal_reset_result(&mut rx, Duration::from_secs(5)).await;
    assert_eq!(
        terminal.error.as_deref(),
        Some("EKF2 restart failed: ekf2 stop: no reply within 300 ms")
    );
}
