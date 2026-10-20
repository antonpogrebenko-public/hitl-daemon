//! ConfigureBuild refusals and an apply that is never half done: a refused or
//! failed request leaves the flight location, the physics and the vehicle as
//! they were, and the retry pushes every parameter again.

use super::*;
use crate::protocol::FlightLocation;
use crate::test_support::{
    catalogue_components, catalogue_request, fake_nsh, ok_board, spawn_catalogue,
    spawn_fake_px4_with_capture, spawn_fake_sim_loop, CapturedMsg, FakeReply, FAST_TIMING,
};
use crossbeam_channel::{bounded, Receiver};
use simulation::{Px4HealthMonitor, SharedOrigin, SimulationConfig};
use terrain::TerrainCache;

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
    /// No parameter link at all (`--sim-only`): the push is skipped.
    Absent,
}

struct Rig {
    handler: Arc<BuildConfigHandler>,
    session: Arc<FcSession>,
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
    let session =
        Arc::new(FcSession::new(board, Arc::new(Px4HealthMonitor::new())).with_timing(FAST_TIMING));
    let (mav_tx, pv_tx, captured) = px4_link(params, &sim);
    let (config_tx, config_rx) = bounded(CONFIG_CAPACITY);
    let mut handler = BuildConfigHandler::new(
        config_tx,
        None,
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
    assert_eq!(
        wait_for_config_applied(&sim, 0, CONFIG_APPLY_TIMEOUT).await,
        Err("Simulation did not apply the build within 1 s".to_string())
    );
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
