//! Shared fakes for the reset and reconfigure tests.

use crate::handler::ValidatedNshCommand;
use crate::nsh_await::NshLink;
use crate::param_io::ParamValue;
use crate::protocol::{ConfigureBuild, NshResponse};
use crate::px4_ready::Px4Timing;
use hitl_physics::{BatteryConfig, PhysicsConfig};
use hitl_sensors::SensorsConfig;
use mavlink::ardupilotmega::MavMessage;
use simulation::{Px4HealthMonitor, SimulationConfig, SimulationState};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{broadcast, mpsc};
use tokio::time::Instant;

/// `Px4Timing` scaled down so the real-time tests (the ones with a thread-based
/// fake, which cannot run on a paused clock) finish in well under a second.
pub(crate) const FAST_TIMING: Px4Timing = Px4Timing {
    nsh_reply: Duration::from_millis(300),
    health_guard: Duration::from_millis(50),
    ready_timeout: Duration::from_millis(400),
    sys_status_absent: Duration::from_millis(200),
};

/// Serve `{"specs": <value>}` for `GET /api/components/<slug>` from a local
/// port, one connection at a time, waiting `delay` before each response.
/// Returns the base URL.
pub(crate) async fn spawn_catalogue(
    components: Vec<(&'static str, serde_json::Value)>,
    delay: Duration,
) -> String {
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let base = format!("http://{}", listener.local_addr().unwrap());
    tokio::spawn(async move {
        loop {
            let Ok((mut sock, _)) = listener.accept().await else {
                return;
            };
            let mut buf = vec![0u8; 4096];
            let n = sock.read(&mut buf).await.unwrap_or(0);
            let head = String::from_utf8_lossy(&buf[..n]).to_string();
            let path = head.split_whitespace().nth(1).unwrap_or("").to_string();
            let slug = path.rsplit('/').next().unwrap_or("");
            let (status, body) = match components.iter().find(|(s, _)| *s == slug) {
                Some((_, specs)) => ("200 OK", serde_json::json!({ "specs": specs })),
                None => ("404 Not Found", serde_json::json!({})),
            };
            let body = body.to_string();
            let resp = format!(
                "HTTP/1.1 {status}\r\nContent-Type: application/json\r\n\
                 Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
                body.len()
            );
            tokio::time::sleep(delay).await;
            let _ = sock.write_all(resp.as_bytes()).await;
            let _ = sock.shutdown().await;
        }
    });
    base
}

/// The components `catalogue_request` names: a 1950 KV motor, a 50/60 A ESC
/// and a 100 C pack.
pub(crate) fn catalogue_components() -> Vec<(&'static str, serde_json::Value)> {
    vec![
        ("motor", serde_json::json!({ "kvRating": 1950.0 })),
        (
            "esc",
            serde_json::json!({ "continuousCurrentA": 50.0, "burstCurrentA": 60.0 }),
        ),
        ("pack", serde_json::json!({ "dischargeRateC": 100.0 })),
    ]
}

/// A build of the `catalogue_components` motor and ESC, and the pack with
/// `with_battery`.
pub(crate) fn catalogue_request(with_battery: bool) -> ConfigureBuild {
    let mut v = serde_json::json!({
        "motor_slug": "motor",
        "prop_diameter_inches": 5.0,
        "frame_weight_g": 300.0,
        "battery_capacity_mah": 1500.0,
        "esc_slug": "esc",
    });
    if with_battery {
        v["battery_slug"] = serde_json::json!("pack");
    }
    serde_json::from_value(v).expect("valid ConfigureBuild")
}

/// Captured outbound MAVLink message: either a PARAM_SET (`name`, `value`)
/// or a COMMAND_LONG identified by command id. Tests use this to verify
/// the daemon pushes the right param set *and* the trailing PARAM_SAVE.
#[derive(Debug, Clone)]
pub(crate) enum CapturedMsg {
    ParamSet(String, f32),
    CommandLong(u32),
}

/// A fake PX4 that records every PARAM_SET and COMMAND_LONG and acks each
/// PARAM_SET. `on_first_param_set` runs once, just before the first ack (a
/// pilot arming from the RC while the parameters are pushed, for example).
pub(crate) fn spawn_fake_px4_with_capture(
    mav_rx: crossbeam_channel::Receiver<MavMessage>,
    param_value_tx: broadcast::Sender<ParamValue>,
    mut on_first_param_set: Option<Box<dyn FnOnce() + Send>>,
) -> (tokio::task::JoinHandle<()>, Arc<Mutex<Vec<CapturedMsg>>>) {
    let captured = Arc::new(Mutex::new(Vec::<CapturedMsg>::new()));
    let captured_clone = captured.clone();
    let handle = tokio::task::spawn_blocking(move || {
        while let Ok(msg) = mav_rx.recv() {
            match msg {
                MavMessage::PARAM_SET(p) => {
                    let name = std::str::from_utf8(&p.param_id)
                        .unwrap_or("")
                        .trim_end_matches('\0')
                        .to_string();
                    captured_clone
                        .lock()
                        .expect("capture mutex poisoned")
                        .push(CapturedMsg::ParamSet(name.clone(), p.param_value));
                    if let Some(hook) = on_first_param_set.take() {
                        hook();
                    }
                    let _ = param_value_tx.send(ParamValue {
                        name,
                        value: p.param_value,
                        param_type: p.param_type,
                        index: 0,
                    });
                }
                MavMessage::COMMAND_LONG(c) => {
                    captured_clone
                        .lock()
                        .expect("capture mutex poisoned")
                        .push(CapturedMsg::CommandLong(c.command as u32));
                }
                _ => {}
            }
        }
    });
    (handle, captured)
}

/// A fake simulation loop on its own thread: applies each config it receives
/// through `SimulationState::reconfigure`, as the real loop does, then logs
/// `"reconfigured"`. The log stays locked across both, so an entry made by
/// anyone who saw the new config generation lands after `"reconfigured"`.
pub(crate) fn spawn_fake_sim_loop(
    config_rx: crossbeam_channel::Receiver<(PhysicsConfig, BatteryConfig, SensorsConfig)>,
    sim_state: SimulationState,
    log: Arc<Mutex<Vec<String>>>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        while let Ok((physics, battery, sensors)) = config_rx.recv() {
            let mut log = log.lock().unwrap();
            sim_state.reconfigure(SimulationConfig {
                physics,
                battery,
                sensors,
                ..sim_state.config()
            });
            log.push("reconfigured".to_string());
        }
    })
}

/// One scripted PX4 report.
pub(crate) enum Px4Event {
    SysStatus(bool),
    Text(&'static str),
}

/// Replays `events` into `monitor`: each `(offset_ms, event)` is recorded at
/// `from + offset_ms`, in offset order, stamped with the time it actually fires.
pub(crate) fn publish_px4_events(
    monitor: Arc<Px4HealthMonitor>,
    from: Instant,
    mut events: Vec<(u64, Px4Event)>,
) -> tokio::task::JoinHandle<()> {
    events.sort_by_key(|(offset, _)| *offset);
    tokio::spawn(async move {
        for (offset, event) in events {
            tokio::time::sleep_until(from + Duration::from_millis(offset)).await;
            let now = Instant::now();
            match event {
                Px4Event::SysStatus(can_arm) => monitor.record_sys_status(can_arm, now),
                Px4Event::Text(text) => monitor.record_statustext(text, now),
            }
        }
    })
}

/// What the fake NSH task does with one command.
pub(crate) enum FakeReply {
    /// A complete `success: true` reply carrying this text.
    Ok(&'static str),
    /// A complete `success: false` reply carrying this text.
    Fail(&'static str),
    /// No reply at all.
    Silent,
}

/// Capacity of the fake NSH command and reply channels; a test never queues more.
const FAKE_NSH_CHANNEL_CAPACITY: usize = 16;

/// A fake NSH task: logs each command string, then replies as `script` says.
pub(crate) fn fake_nsh(
    log: Arc<std::sync::Mutex<Vec<String>>>,
    script: impl Fn(&str) -> FakeReply + Send + 'static,
) -> NshLink {
    let (commands, mut inbox) = mpsc::channel::<ValidatedNshCommand>(FAKE_NSH_CHANNEL_CAPACITY);
    let (replies, _) = broadcast::channel::<NshResponse>(FAKE_NSH_CHANNEL_CAPACITY);
    let reply_tx = replies.clone();
    tokio::spawn(async move {
        while let Some(cmd) = inbox.recv().await {
            log.lock().unwrap().push(cmd.command.clone());
            let (success, output) = match script(&cmd.command) {
                FakeReply::Ok(text) => (true, text),
                FakeReply::Fail(text) => (false, text),
                FakeReply::Silent => continue,
            };
            let _ = reply_tx.send(NshResponse {
                request_id: cmd.request_id,
                success,
                complete: true,
                output: output.to_string(),
            });
        }
    });
    NshLink { commands, replies }
}

/// A healthy board: `ekf2 stop` and `ekf2 start` succeed, anything else gets no reply.
pub(crate) fn ok_board(command: &str) -> FakeReply {
    match command {
        "ekf2 stop" => FakeReply::Ok("ekf2 stop\r\nnsh> "),
        "ekf2 start" => FakeReply::Ok("ekf2 start\r\nnsh> "),
        _ => FakeReply::Silent,
    }
}
