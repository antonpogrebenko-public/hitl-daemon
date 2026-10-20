//! The vehicle reset: put the simulated vehicle back at its start, restart EKF2
//! and report each step as a `ResetResult`.
//!
//! It never receives the `SharedOrigin` or the terrain cache, so the flight
//! location and the vertical datum cannot change; `SimulationState::reset`
//! keeps the live config (the applied build) and `sim_time_us`.

use crate::fc_session::FcSession;
use crate::protocol::{OutgoingMessage, Px4HealthReport, ResetResult, ResetStage};
use simulation::SimulationState;
use tokio::sync::mpsc;
use tracing::{debug, info};

/// Reported when the daemon has no reset flow to run.
pub(crate) const RESET_UNAVAILABLE: &str = "Reset is unavailable in this session";

/// Runs one reset and returns its terminal result.
///
/// Every result, the terminal one included, is also sent on `progress_tx`; a
/// closed receiver (the tab left) is not an error, the reset still finishes so
/// the vehicle and PX4 end in a known state and the session is released.
pub(crate) async fn run_reset(
    session: &FcSession,
    sim_state: &SimulationState,
    progress_tx: &mpsc::Sender<OutgoingMessage>,
) -> ResetResult {
    let result = reset_steps(session, sim_state, progress_tx).await;
    send(progress_tx, result.clone()).await;
    result
}

async fn send(progress_tx: &mpsc::Sender<OutgoingMessage>, result: ResetResult) {
    if progress_tx
        .send(OutgoingMessage::ResetResult(result))
        .await
        .is_err()
    {
        debug!("Reset progress receiver closed; continuing the reset");
    }
}

async fn report(progress_tx: &mpsc::Sender<OutgoingMessage>, stage: ResetStage) {
    send(progress_tx, ResetResult::resetting(stage)).await;
}

/// Everything except sending the terminal result, so each exit is one `return`.
async fn reset_steps(
    session: &FcSession,
    sim_state: &SimulationState,
    progress_tx: &mpsc::Sender<OutgoingMessage>,
) -> ResetResult {
    // The guard is held to the end: configure and the re-push cannot start
    // while the vehicle is reset or EKF2 restarts.
    let _guard = match session.begin_unless_armed(sim_state) {
        Ok(guard) => guard,
        Err(refusal) => return ResetResult::failed(refusal),
    };

    report(progress_tx, ResetStage::ResettingVehicle).await;
    sim_state.reset();
    info!("Vehicle reset to the start");

    if session.is_sim_only() {
        return ResetResult::ready(Px4HealthReport::NotApplicable);
    }

    report(progress_tx, ResetStage::RestartingEkf).await;
    let restart = match session.restart_ekf2().await {
        Ok(restart) => restart,
        Err(reason) => return ResetResult::failed(reason),
    };

    report(progress_tx, ResetStage::WaitingForPx4).await;
    match session.wait_for_px4_ready(&restart).await {
        Ok(health) => ResetResult::ready(health),
        Err(reason) => ResetResult::failed(reason),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::ResetState;
    use crate::test_support::{fake_nsh, ok_board, publish_px4_events, FakeReply, Px4Event};
    use hitl_physics::QuadrotorState;
    use nalgebra::{UnitQuaternion, Vector3};
    use simulation::{Px4HealthMonitor, SimulationConfig};
    use std::sync::Arc;
    use std::sync::Mutex;
    use tokio::time::Instant;

    /// Capacity of the progress channel; a reset sends at most four results.
    const PROGRESS_CAPACITY: usize = 16;

    type Log = Arc<Mutex<Vec<String>>>;

    fn board(
        script: impl Fn(&str) -> FakeReply + Send + 'static,
    ) -> (Arc<FcSession>, Arc<Px4HealthMonitor>, Log) {
        let log: Log = Arc::new(Mutex::new(Vec::new()));
        let health = Arc::new(Px4HealthMonitor::new());
        let nsh = fake_nsh(log.clone(), script);
        let session = Arc::new(FcSession::new(Some(nsh), health.clone()));
        (session, health, log)
    }

    fn sim_only() -> Arc<FcSession> {
        Arc::new(FcSession::new(None, Arc::new(Px4HealthMonitor::new())))
    }

    fn flown(sim: &SimulationState) {
        let mut s = sim.write();
        s.quadrotor.position = [40.0, 0.0, 0.0];
        s.quadrotor.velocity = [3.0, 0.0, 0.0];
        s.quadrotor.angular_velocity = [0.1, 0.0, 0.0];
        s.quadrotor.quaternion = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.2);
        s.board_stall = true;
        s.battery.discharge(10_000.0, 1_000.0);
        s.sim_time_us = 63_000_000;
    }

    fn results(rx: &mut mpsc::Receiver<OutgoingMessage>) -> Vec<ResetResult> {
        let mut out = Vec::new();
        while let Ok(msg) = rx.try_recv() {
            if let OutgoingMessage::ResetResult(r) = msg {
                out.push(r);
            }
        }
        out
    }

    fn channel() -> (
        mpsc::Sender<OutgoingMessage>,
        mpsc::Receiver<OutgoingMessage>,
    ) {
        mpsc::channel(PROGRESS_CAPACITY)
    }

    fn healthy_events(health: &Arc<Px4HealthMonitor>) {
        let events = (1..=10)
            .map(|i| (i * 500, Px4Event::SysStatus(true)))
            .collect();
        publish_px4_events(health.clone(), Instant::now(), events);
    }

    #[tokio::test(start_paused = true)]
    async fn reset_returns_the_vehicle_to_the_start_and_keeps_location_build_and_clock() {
        let sim = SimulationState::new(SimulationConfig::default());
        flown(&sim);
        let mass = sim.config().physics.mass_kg;
        let (tx, _rx) = channel();
        run_reset(&sim_only(), &sim, &tx).await;
        let s = sim.read();
        let home = QuadrotorState::default();
        assert_eq!(
            (
                s.quadrotor.position,
                s.quadrotor.velocity,
                s.quadrotor.angular_velocity
            ),
            (home.position, home.velocity, home.angular_velocity)
        );
        assert_eq!(s.quadrotor.quaternion, home.quaternion);
        assert_eq!(
            (s.quadrotor.motor_speeds, s.motor_commands),
            ([0.0; 4], [0.0; 4])
        );
        assert_eq!(s.battery.percent(), 100);
        assert!(!s.board_stall);
        assert!(s.sim_time_us >= 63_000_000);
        assert_eq!(sim.config().physics.mass_kg, mass); // the applied build is kept
    }

    #[tokio::test(start_paused = true)]
    async fn a_full_reset_reports_three_stages_then_ready_healthy() {
        let (session, health, _log) = board(ok_board);
        healthy_events(&health);
        let sim = SimulationState::new(SimulationConfig::default());
        let (tx, mut rx) = channel();
        let terminal = run_reset(&session, &sim, &tx).await;
        assert_eq!(
            results(&mut rx),
            [
                ResetResult::resetting(ResetStage::ResettingVehicle),
                ResetResult::resetting(ResetStage::RestartingEkf),
                ResetResult::resetting(ResetStage::WaitingForPx4),
                ResetResult::ready(Px4HealthReport::Healthy),
            ]
        );
        assert_eq!(terminal, ResetResult::ready(Px4HealthReport::Healthy));
    }

    #[tokio::test(start_paused = true)]
    async fn sim_only_reset_reports_resetting_vehicle_then_ready_not_applicable() {
        let sim = SimulationState::new(SimulationConfig::default());
        let (tx, mut rx) = channel();
        run_reset(&sim_only(), &sim, &tx).await;
        assert_eq!(
            results(&mut rx),
            [
                ResetResult::resetting(ResetStage::ResettingVehicle),
                ResetResult::ready(Px4HealthReport::NotApplicable)
            ]
        );
    }

    #[tokio::test(start_paused = true)]
    async fn sim_only_reset_ignores_the_generators_armed_flag() {
        let sim = SimulationState::new(SimulationConfig::default());
        sim.set_armed(true); // what the sim-only hover generator reports all session
        let (tx, _rx) = channel();
        assert_eq!(
            run_reset(&sim_only(), &sim, &tx).await,
            ResetResult::ready(Px4HealthReport::NotApplicable)
        );
    }

    #[tokio::test(start_paused = true)]
    async fn an_armed_reset_is_refused_and_changes_nothing() {
        let (session, _health, log) = board(ok_board);
        let sim = SimulationState::new(SimulationConfig::default());
        flown(&sim);
        sim.set_armed(true);
        let (tx, mut rx) = channel();
        assert_eq!(
            run_reset(&session, &sim, &tx).await,
            ResetResult::failed("Land and disarm first.")
        );
        assert_eq!(
            results(&mut rx),
            [ResetResult::failed("Land and disarm first.")]
        );
        assert_eq!(sim.read().quadrotor.position, [40.0, 0.0, 0.0]);
        assert!(sim.read().battery.percent() < 100);
        assert!(log.lock().unwrap().is_empty());
    }

    #[tokio::test(start_paused = true)]
    async fn a_reset_during_another_is_refused_as_busy() {
        let (session, _health, _log) = board(ok_board);
        let sim = SimulationState::new(SimulationConfig::default());
        flown(&sim);
        let _running = session.try_begin().unwrap();
        let (tx, _rx) = channel();
        assert_eq!(
            run_reset(&session, &sim, &tx).await,
            ResetResult::failed("Another reset or configuration is in progress.")
        );
        assert_eq!(sim.read().quadrotor.position, [40.0, 0.0, 0.0]);
    }

    #[tokio::test(start_paused = true)]
    async fn a_reset_without_a_board_keeps_the_vehicle_reset_and_reports_ekf2_restart_failed() {
        let (session, _health, _log) = board(|_| FakeReply::Fail("No FC connected"));
        let sim = SimulationState::new(SimulationConfig::default());
        flown(&sim);
        let (tx, _rx) = channel();
        let terminal = run_reset(&session, &sim, &tx).await;
        assert!(terminal
            .error
            .as_deref()
            .unwrap()
            .starts_with("EKF2 restart failed:"));
        assert_eq!(terminal.state, ResetState::Error);
        assert_eq!(sim.read().quadrotor.position, [0.0; 3]);
    }

    #[tokio::test(start_paused = true)]
    async fn a_retry_after_a_failed_start_passes_not_running() {
        let (session, health, log) = board(|cmd| match cmd {
            "ekf2 stop" => FakeReply::Ok("ekf2 stop\r\nekf2 not running\r\nnsh> "),
            other => ok_board(other),
        });
        publish_px4_events(
            health,
            Instant::now(),
            vec![(1_500, Px4Event::SysStatus(true))],
        );
        let sim = SimulationState::new(SimulationConfig::default());
        let (tx, _rx) = channel();
        assert_eq!(
            run_reset(&session, &sim, &tx).await,
            ResetResult::ready(Px4HealthReport::Healthy)
        );
        assert_eq!(*log.lock().unwrap(), ["ekf2 stop", "ekf2 start"]);
    }

    #[tokio::test(start_paused = true)]
    async fn a_reset_whose_browser_left_still_finishes_and_releases_the_session() {
        let (session, health, _log) = board(ok_board);
        healthy_events(&health);
        let sim = SimulationState::new(SimulationConfig::default());
        let (tx, rx) = channel();
        drop(rx); // the tab closed
        assert_eq!(
            run_reset(&session, &sim, &tx).await,
            ResetResult::ready(Px4HealthReport::Healthy)
        );
        assert!(session.try_begin().is_some());
    }
}
