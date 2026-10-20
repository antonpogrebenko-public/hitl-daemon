//! ConfigureBuild refusals and an apply that is never half done: a refused or
//! failed request leaves the flight location, the physics and the vehicle as
//! they were, and the retry pushes every parameter again.

use super::*;
use crate::protocol::{FlightLocation, Px4HealthReport};
use crate::test_support::{
    catalogue_components, catalogue_request, fake_nsh, ok_board, publish_px4_events,
    spawn_catalogue, spawn_fake_px4_with_capture, spawn_fake_sim_loop, CapturedMsg, FakeReply,
    Px4Event::{SysStatus, Text},
    FAST_TIMING,
};
use crossbeam_channel::{bounded, Receiver};
use hitl_physics::px4_pids::REF_PIDS;
use simulation::{Px4HealthMonitor, SharedOrigin, SimulationConfig};
use terrain::TerrainCache;
use tokio::time::Instant;

/// Where every rig starts, and where a request moves the flight.
const A: (f64, f64) = (40.015, -105.2705);
const B: (f64, f64) = (47.3769, 8.5417);

/// The datum the rig starts on, and the elevation `request_at` asks for.
const START_DATUM: f64 = 1655.0;
const REQUESTED_ELEVATION: f64 = 1700.0;

/// Capacity of the progress channel; an apply reports at most four stages.
const PROGRESS_CAPACITY: usize = 16;
/// Capacity of the config channel; a test sends at most two builds.
const CONFIG_CAPACITY: usize = 4;
/// Capacity of the MAVLink and PARAM_VALUE channels; a push sends 21 parameters.
const MAV_CAPACITY: usize = 64;

/// The rig's wait for the simulation to apply a build, short so a test of a
/// loop that never applies it costs little; production waits
/// `CONFIG_APPLY_TIMEOUT`.
const APPLY_TIMEOUT: Duration = Duration::from_millis(100);

type Log = Arc<Mutex<Vec<String>>>;
type Captured = Arc<Mutex<Vec<CapturedMsg>>>;
type ConfigRx = Receiver<(PhysicsConfig, BatteryConfig, hitl_sensors::SensorsConfig)>;
type Px4Link = (
    Option<Sender<MavMessage>>,
    Option<broadcast::Sender<ParamValue>>,
    Option<Captured>,
);

/// What PX4 does with the parameter push.
enum Px4Params {
    /// Acks every PARAM_SET.
    Acks,
    /// Acks every PARAM_SET, but a pilot arms from the RC before the first ack.
    AcksArmingOnFirst,
    /// The MAVLink link is gone: the first PARAM_SET fails at once.
    Disconnected,
    /// PX4 receives every PARAM_SET and acks none: a push stalls on its
    /// per-parameter timeouts.
    Silent,
    /// No parameter link at all (`--sim-only`): the push is skipped.
    Absent,
}

struct Rig {
    handler: Arc<BuildConfigHandler>,
    session: Arc<FcSession>,
    /// What the session reads PX4's pre-arm state from; a test scripts it.
    health: Arc<Px4HealthMonitor>,
    sim: SimulationState,
    origin: Arc<SharedOrigin>,
    cache: Arc<TerrainCache>,
    /// The NSH commands the fake board received and the fake simulation
    /// loop's "reconfigured", in the order they happened.
    log: Log,
    /// What the fake PX4 received; `None` when there is no acker.
    captured: Option<Captured>,
    /// Holds the config channel open when no simulation loop drains it.
    _config_rx: Option<ConfigRx>,
}

/// The parameter link `params` describes: the MAVLink sender, the PARAM_VALUE
/// tap and what the fake PX4 captures.
fn px4_link(params: Px4Params, sim: &SimulationState) -> Px4Link {
    let on_first_param_set: Option<Box<dyn FnOnce() + Send>> = match params {
        Px4Params::Absent => return (None, None, None),
        Px4Params::Disconnected => {
            let (mav_tx, _dropped) = bounded(MAV_CAPACITY);
            return (Some(mav_tx), Some(broadcast::channel(MAV_CAPACITY).0), None);
        }
        Px4Params::Silent => {
            let (mav_tx, mav_rx) = bounded::<MavMessage>(MAV_CAPACITY);
            // Drained so every send succeeds; nothing is acked.
            std::thread::spawn(move || while mav_rx.recv().is_ok() {});
            return (Some(mav_tx), Some(broadcast::channel(MAV_CAPACITY).0), None);
        }
        Px4Params::Acks => None,
        Px4Params::AcksArmingOnFirst => {
            let sim = sim.clone();
            Some(Box::new(move || sim.set_armed(true)))
        }
    };
    let (mav_tx, mav_rx) = bounded(MAV_CAPACITY);
    let (pv_tx, _) = broadcast::channel(MAV_CAPACITY);
    let (_px4, captured) = spawn_fake_px4_with_capture(mav_rx, pv_tx.clone(), on_first_param_set);
    (Some(mav_tx), Some(pv_tx), Some(captured))
}

/// A handler anchored at `A`, with a catalogue, a session (`nsh: None` is
/// sim-only) and, with `sim_loop`, a fake simulation loop applying each build.
async fn rig(params: Px4Params, nsh: Option<fn(&str) -> FakeReply>, sim_loop: bool) -> Rig {
    let origin = Arc::new(SharedOrigin::new(A.0, A.1, START_DATUM));
    let cache = Arc::new(TerrainCache::new());
    cache.set_origin(A.0, A.1, Some(START_DATUM));
    let sim = SimulationState::new(SimulationConfig::default());
    let log: Log = Arc::default();
    let board = nsh.map(|script| fake_nsh(log.clone(), script));
    let health = Arc::new(Px4HealthMonitor::new());
    let session = Arc::new(FcSession::new(board, health.clone()).with_timing(FAST_TIMING));
    let (mav_tx, pv_tx, captured) = px4_link(params, &sim);
    let (config_tx, config_rx) = bounded(CONFIG_CAPACITY);
    let mut handler = BuildConfigHandler::new(
        config_tx,
        mav_tx,
        pv_tx,
        Some(cache.clone()),
        (A.0, A.1, START_DATUM),
        Some(origin.clone()),
    )
    .with_fc_session(session.clone(), sim.clone());
    handler.api_url = spawn_catalogue(catalogue_components(), Duration::ZERO).await;
    handler.config_apply_timeout = APPLY_TIMEOUT;
    let config_rx = if sim_loop {
        spawn_fake_sim_loop(config_rx, sim.clone(), log.clone());
        None
    } else {
        Some(config_rx)
    };
    Rig {
        handler: Arc::new(handler),
        session,
        health,
        sim,
        origin,
        cache,
        log,
        captured,
        _config_rx: config_rx,
    }
}

/// The catalogue build, flown at `loc` on the elevation the browser sampled.
fn request_at(loc: (f64, f64)) -> ConfigureBuild {
    ConfigureBuild {
        flight_location: Some(FlightLocation {
            lat: loc.0,
            lon: loc.1,
        }),
        origin_elevation_msl: Some(REQUESTED_ELEVATION),
        ..catalogue_request(false)
    }
}

/// Runs one ConfigureBuild: its result and everything sent on the progress
/// channel meanwhile.
async fn run(rig: &Rig, request: ConfigureBuild) -> (ConfigResult, Vec<OutgoingMessage>) {
    let (tx, mut rx) = mpsc::channel(PROGRESS_CAPACITY);
    let result = rig.handler.handle(request, tx).await;
    let mut sent = Vec::new();
    while let Ok(msg) = rx.try_recv() {
        sent.push(msg);
    }
    (result, sent)
}

/// The stage of every `Configuring` result, in the order they were sent.
fn stages(sent: &[OutgoingMessage]) -> Vec<ConfigStage> {
    sent.iter()
        .filter_map(|msg| match msg {
            OutgoingMessage::ConfigResult(r) if matches!(r.state, ConfigState::Configuring) => {
                r.stage
            }
            _ => None,
        })
        .collect()
}

/// How many PARAM_SETs PX4 has received.
fn param_sets(rig: &Rig) -> usize {
    rig.captured.as_ref().map_or(0, |captured| {
        captured
            .lock()
            .unwrap()
            .iter()
            .filter(|m| matches!(m, CapturedMsg::ParamSet(..)))
            .count()
    })
}

/// The value of the last PARAM_SET PX4 received for `name`.
fn last_param(rig: &Rig, name: &str) -> Option<f32> {
    rig.captured.as_ref().and_then(|captured| {
        captured.lock().unwrap().iter().rev().find_map(|m| match m {
            CapturedMsg::ParamSet(n, v) if n == name => Some(*v),
            _ => None,
        })
    })
}

/// The flight origin's latitude and longitude.
fn at(rig: &Rig) -> (f64, f64) {
    let origin = rig.origin.get();
    (origin.lat, origin.lon)
}

#[tokio::test]
async fn an_armed_configure_is_refused_and_changes_nothing() {
    let rig = rig(Px4Params::Acks, Some(ok_board), true).await;
    rig.sim.set_armed(true);
    rig.sim.write().quadrotor.position = [3.0, 4.0, -10.0];
    let (result, sent) = run(&rig, request_at(B)).await;
    assert_eq!(
        (result.success, result.error.as_deref()),
        (false, Some("Land and disarm first."))
    );
    assert!(sent.is_empty(), "a refusal reports no stage");
    assert_eq!(
        (param_sets(&rig), at(&rig), rig.sim.config_generation()),
        (0, A, 0)
    );
    assert_eq!(rig.sim.read().quadrotor.position, [3.0, 4.0, -10.0]);
    assert!(
        rig.log.lock().unwrap().is_empty(),
        "nothing reaches the board"
    );
}

#[tokio::test]
async fn a_configure_during_a_reset_is_refused_as_busy() {
    let rig = rig(Px4Params::Acks, Some(ok_board), true).await;
    let _reset = rig.session.try_begin().unwrap();
    let (result, sent) = run(&rig, request_at(B)).await;
    assert_eq!(
        result.error.as_deref(),
        Some("Another reset or configuration is in progress.")
    );
    assert!(sent.is_empty());
    assert_eq!(
        (param_sets(&rig), at(&rig), rig.sim.config_generation()),
        (0, A, 0)
    );
    assert!(
        rig.log.lock().unwrap().is_empty(),
        "nothing reaches the board"
    );
}

#[tokio::test]
async fn sim_only_configure_ignores_the_generators_armed_flag() {
    let rig = rig(Px4Params::Absent, None, true).await;
    rig.sim.set_armed(true);
    assert!(run(&rig, request_at(B)).await.0.success);
}

#[tokio::test]
async fn ready_waits_for_the_simulation_to_apply_the_build() {
    let rig = rig(Px4Params::Absent, None, true).await;
    let (result, _) = run(&rig, request_at(B)).await;
    assert!(result.success, "{:?}", result.error);
    assert_eq!(rig.sim.config_generation(), 1);
    assert_eq!(
        rig.sim.config().physics.mass_kg,
        result.config.unwrap().mass_kg
    );
}

#[tokio::test]
async fn a_build_the_simulation_never_applies_fails() {
    let rig = rig(Px4Params::Absent, None, false).await;
    assert_eq!(
        run(&rig, request_at(B)).await.0.error.as_deref(),
        Some("Simulation did not apply the build within 100 ms")
    );
}

/// The production wait, on the paused clock: the text names the real limit.
#[tokio::test(start_paused = true)]
async fn the_apply_wait_gives_up_after_one_second() {
    let sim = SimulationState::new(SimulationConfig::default());
    let started = Instant::now();
    assert_eq!(
        wait_for_config_applied(&sim, 0, CONFIG_APPLY_TIMEOUT).await,
        Err("Simulation did not apply the build within 1 s".to_string())
    );
    assert_eq!(started.elapsed(), CONFIG_APPLY_TIMEOUT);
}

#[tokio::test]
async fn a_failed_parameter_push_keeps_the_previous_location_physics_and_vehicle() {
    let rig = rig(Px4Params::Disconnected, Some(ok_board), true).await;
    rig.sim.write().quadrotor.position = [5.0, 0.0, 0.0];
    let (result, _) = run(&rig, request_at(B)).await;
    assert!(result.error.unwrap().starts_with("PID verification failed"));
    assert_eq!((at(&rig), rig.origin.get().alt_datum), (A, START_DATUM));
    assert!(rig.cache.is_anchored_to(A.0, A.1));
    assert_eq!(rig.sim.config_generation(), 0);
    assert_eq!(rig.sim.read().quadrotor.position, [5.0, 0.0, 0.0]);
}

#[tokio::test]
async fn a_failed_spec_fetch_keeps_the_previous_location() {
    let rig = rig(Px4Params::Acks, Some(ok_board), true).await;
    let mut req = request_at(B);
    req.motor_slug = "missing".to_string();
    assert!(run(&rig, req)
        .await
        .0
        .error
        .unwrap()
        .starts_with("Failed to fetch motor"));
    assert_eq!(
        (at(&rig), rig.sim.config_generation(), param_sets(&rig)),
        (A, 0, 0)
    );
    assert!(rig.cache.is_anchored_to(A.0, A.1));
}

#[tokio::test]
async fn after_a_failed_apply_the_same_build_pushes_every_parameter_again() {
    // No simulation loop: every apply fails after the push.
    let rig = rig(Px4Params::Acks, Some(ok_board), false).await;
    assert!(!run(&rig, request_at(B)).await.0.success);
    let first = param_sets(&rig);
    assert!(first > 0);
    assert!(!run(&rig, request_at(B)).await.0.success);
    assert_eq!(
        param_sets(&rig),
        2 * first,
        "a failed apply must not leave a fingerprint that skips the push"
    );
}

#[tokio::test]
async fn arming_during_an_apply_stops_it_before_the_vehicle_is_reset() {
    let rig = rig(Px4Params::AcksArmingOnFirst, Some(ok_board), true).await;
    let (result, _) = run(&rig, request_at(B)).await;
    assert_eq!(result.error.as_deref(), Some("Land and disarm first."));
    assert!(param_sets(&rig) > 0);
    assert_eq!((at(&rig), rig.sim.config_generation()), (A, 0));
    assert!(rig.cache.is_anchored_to(A.0, A.1));
}

#[tokio::test]
async fn a_successful_apply_anchors_the_terrain_cache_to_the_new_location() {
    let rig = rig(Px4Params::Acks, Some(ok_board), true).await;
    let (result, _) = run(&rig, request_at(B)).await;
    assert!(result.success, "{:?}", result.error);
    assert_eq!(
        (at(&rig), rig.origin.get().alt_datum),
        (B, REQUESTED_ELEVATION)
    );
    assert!(rig.cache.is_anchored_to(B.0, B.1));
}

#[tokio::test]
async fn configure_reports_waiting_for_px4_then_ready_healthy() {
    let rig = rig(Px4Params::Acks, Some(ok_board), true).await;
    let _px4 = publish_px4_events(
        rig.health.clone(),
        Instant::now(),
        (0..250).map(|i| (i * 20, SysStatus(true))).collect(),
    );
    let (result, sent) = run(&rig, request_at(B)).await;
    use ConfigStage::*;
    assert_eq!(
        stages(&sent),
        [
            FetchingSpecs,
            Computing,
            PushingParams,
            RestartingEkf,
            WaitingForPx4
        ]
    );
    assert!(matches!(result.state, ConfigState::Ready));
    assert_eq!(result.px4_health, Some(Px4HealthReport::Healthy));
    assert_eq!(
        serde_json::to_value(&result).unwrap()["px4_health"],
        "healthy"
    );
}

#[tokio::test]
async fn configure_fails_with_px4s_reason_when_the_prearm_check_never_passes() {
    let rig = rig(Px4Params::Acks, Some(ok_board), true).await;
    let failing = (0..250).map(|i| (i * 20, SysStatus(false)));
    let reasons = (0..50).map(|i| (i * 100, Text("Preflight Fail: ekf2 missing data")));
    let _px4 = publish_px4_events(
        rig.health.clone(),
        Instant::now(),
        failing.chain(reasons).collect(),
    );
    let (result, sent) = run(&rig, request_at(B)).await;
    assert_eq!(
        result.error.as_deref(),
        Some("Preflight Fail: ekf2 missing data")
    );
    assert_eq!(stages(&sent).last(), Some(&ConfigStage::WaitingForPx4));
    assert_eq!(result.px4_health, None);
}

#[tokio::test]
async fn configure_fails_when_ekf2_start_reports_an_error() {
    let rig = rig(
        Px4Params::Acks,
        Some(|cmd| match cmd {
            "ekf2 start" => FakeReply::Ok("ekf2 start\r\nERROR [ekf2] task start failed\r\nnsh> "),
            other => ok_board(other),
        }),
        true,
    )
    .await;
    assert!(run(&rig, request_at(B))
        .await
        .0
        .error
        .unwrap()
        .starts_with("EKF2 restart failed: ekf2 start:"));
}

#[tokio::test]
async fn sim_only_configure_skips_the_restart_and_reports_not_applicable() {
    let rig = rig(Px4Params::Absent, None, true).await;
    let (result, sent) = run(&rig, request_at(B)).await;
    use ConfigStage::*;
    assert_eq!(stages(&sent), [FetchingSpecs, Computing, PushingParams]);
    assert_eq!(result.px4_health, Some(Px4HealthReport::NotApplicable));
    // The simulation applied the build, and nothing else happened.
    assert_eq!(*rig.log.lock().unwrap(), ["reconfigured"]);
}

#[tokio::test]
async fn the_ekf2_restart_starts_after_the_simulation_applied_the_build() {
    let rig = rig(Px4Params::Absent, Some(ok_board), true).await;
    // No SYS_STATUS at all: ready, with px4_health unknown.
    let (result, _) = run(&rig, request_at(B)).await;
    assert!(result.success, "{:?}", result.error);
    assert_eq!(result.px4_health, Some(Px4HealthReport::Unknown));
    assert_eq!(
        *rig.log.lock().unwrap(),
        ["reconfigured", "ekf2 stop", "ekf2 start"]
    );
}

#[tokio::test]
async fn the_repush_waits_for_a_running_apply_before_writing_parameters() {
    let rig = rig(Px4Params::Acks, Some(ok_board), true).await;
    rig.handler.seed_verified_params(REF_PIDS, 0.5, 0.15);
    let mut clients = rig.handler.subscribe_system_config();
    // Stands in for a running configure or reset.
    let apply = rig.session.try_begin().unwrap();
    let h = rig.handler.clone();
    let repush = tokio::spawn(async move { h.repush_if_configured().await });
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert_eq!(
        param_sets(&rig),
        0,
        "no PARAM_SET while the session is held"
    );
    assert!(
        clients.try_recv().is_err(),
        "nothing announced while waiting"
    );
    // The running apply verified a new build meanwhile: that build is the one re-pushed.
    rig.handler.seed_verified_params(REF_PIDS, 0.42, 0.15);
    drop(apply);
    assert!(repush.await.unwrap().is_ok());
    assert_eq!(param_sets(&rig), 21);
    assert_eq!(last_param(&rig, "MPC_THR_HOVER"), Some(0.42));
}

#[tokio::test]
async fn a_configure_or_reset_during_the_repush_is_refused_as_busy() {
    // The re-push stalls on unanswered PARAM_SETs, holding the session.
    let rig = rig(Px4Params::Silent, Some(ok_board), true).await;
    rig.handler.seed_verified_params(REF_PIDS, 0.5, 0.15);
    let h = rig.handler.clone();
    let repush = tokio::spawn(async move { h.repush_if_configured().await });
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert_eq!(
        run(&rig, request_at(B)).await.0.error.as_deref(),
        Some("Another reset or configuration is in progress.")
    );
    let (tx, _rx) = mpsc::channel(PROGRESS_CAPACITY);
    assert_eq!(
        rig.handler.reset_vehicle(tx).await,
        ResetResult::failed("Another reset or configuration is in progress.")
    );
    assert!(
        !repush.is_finished(),
        "the refusals came while it held the session"
    );
    // Its eventual failure takes 2.4 s of ack timeouts; this test is done.
    repush.abort();
}
