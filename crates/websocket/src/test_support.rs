//! Shared fakes for the reset and reconfigure tests.

use simulation::Px4HealthMonitor;
use std::sync::Arc;
use std::time::Duration;
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
