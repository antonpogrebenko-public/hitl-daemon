//! Shared fakes for the reset and reconfigure tests.

use crate::handler::ValidatedNshCommand;
use crate::nsh_await::NshLink;
use crate::protocol::NshResponse;
use simulation::Px4HealthMonitor;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{broadcast, mpsc};
use tokio::time::Instant;

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
