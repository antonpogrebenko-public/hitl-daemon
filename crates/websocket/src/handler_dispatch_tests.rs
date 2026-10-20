//! The byte-8 reset command: acknowledged first, run off the receive loop.

use super::tests::create_test_handler;
use super::*;
use crate::fc_session::FcSession;
use crate::protocol::{Px4HealthReport, ResetResult, ResetStage, MSG_TYPE_COMMAND};
use simulation::{Px4HealthMonitor, SimulationConfig, SimulationState};

/// A reset command frame: id 7, `ResetVehicle`.
const RESET: [u8; 6] = [MSG_TYPE_COMMAND, 7, 0, 0, 0, 8];

/// Capacity of the progress channel; a reset sends a few results at most.
const PROGRESS_CAPACITY: usize = 16;

const RECV_TIMEOUT: Duration = Duration::from_secs(2);

/// A handler whose reset runs against a sim-only session (no board to restart).
async fn with_reset() -> (ConnectionHandler, mpsc::Receiver<ValidatedCommand>) {
    let (mut handler, cmd_rx) = create_test_handler().await;
    // A reset never sends a config; the receiver is only here to build the handler.
    let (config_tx, _config_rx) = crossbeam_channel::bounded(PROGRESS_CAPACITY);
    let session = Arc::new(FcSession::new(None, Arc::new(Px4HealthMonitor::new())));
    let build = BuildConfigHandler::new(config_tx, None, None, None, (0.0, 0.0, 0.0), None)
        .with_fc_session(session, SimulationState::new(SimulationConfig::default()));
    handler.set_build_config_handler(Arc::new(build));
    (handler, cmd_rx)
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
