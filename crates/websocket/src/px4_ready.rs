//! The readiness rule: after `ekf2 start`, "ready" means PX4 says it can arm.

use crate::protocol::Px4HealthReport;
use simulation::Px4HealthMonitor;
use std::time::Duration;
use tokio::time::Instant;
use tracing::{info, warn};

pub const NSH_REPLY_TIMEOUT: Duration = Duration::from_secs(3);
pub const PX4_HEALTH_GUARD: Duration = Duration::from_secs(1);
pub const PX4_READY_TIMEOUT: Duration = Duration::from_secs(30);
pub const SYS_STATUS_ABSENT: Duration = Duration::from_secs(3);

#[derive(Debug, Clone, Copy)]
pub struct Px4Timing {
    pub nsh_reply: Duration,
    pub health_guard: Duration,
    pub ready_timeout: Duration,
    pub sys_status_absent: Duration,
}

impl Default for Px4Timing {
    fn default() -> Self {
        Self {
            nsh_reply: NSH_REPLY_TIMEOUT,
            health_guard: PX4_HEALTH_GUARD,
            ready_timeout: PX4_READY_TIMEOUT,
            sys_status_absent: SYS_STATUS_ABSENT,
        }
    }
}

/// Waits until PX4 reports it can arm, or gives up.
///
/// `began_at` is when the restart began (the "since the restart" for a
/// Preflight Fail text); `confirmed_at` is when `ekf2 start` was confirmed. A
/// passing `SYS_STATUS` counts only if received at least `health_guard` after
/// `confirmed_at`, because one sent just before the restart took effect still
/// describes the old estimator.
pub async fn wait_for_px4_ready(
    monitor: &Px4HealthMonitor,
    began_at: Instant,
    confirmed_at: Instant,
    timing: &Px4Timing,
) -> Result<Px4HealthReport, String> {
    let mut rx = monitor.subscribe();
    let ready_after = confirmed_at + timing.health_guard;
    let absent_by = confirmed_at + timing.sys_status_absent;
    let give_up_at = confirmed_at + timing.ready_timeout;
    // A SYS_STATUS received at or after confirmed_at.
    let mut heard = false;
    loop {
        // Copied out: a watch::Ref must not live across the await below.
        let current = *rx.borrow_and_update();
        if let Some(h) = current {
            heard |= h.received_at >= confirmed_at;
            if h.can_arm && h.received_at >= ready_after {
                let after_ms = (Instant::now() - confirmed_at).as_millis() as u64;
                info!(after_ms, "PX4 reports ready to arm after the EKF2 restart");
                return Ok(Px4HealthReport::Healthy);
            }
        }
        let now = Instant::now();
        if !heard && now >= absent_by {
            warn!("PX4 sent no SYS_STATUS after the EKF2 restart; reporting ready with px4_health unknown");
            return Ok(Px4HealthReport::Unknown);
        }
        if now >= give_up_at {
            let reason = monitor.preflight_fail_since(began_at).unwrap_or_else(|| {
                format!(
                    "PX4 did not report ready to arm within {} s",
                    timing.ready_timeout.as_secs()
                )
            });
            let after_ms = (now - confirmed_at).as_millis() as u64;
            warn!(after_ms, reason = %reason, "PX4 did not report ready to arm");
            return Err(reason);
        }
        let next = if heard {
            give_up_at
        } else {
            absent_by.min(give_up_at)
        };
        if let Ok(Err(_)) = tokio::time::timeout_at(next, rx.changed()).await {
            return Err("PX4 health channel closed".to_string());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::{publish_px4_events, Px4Event, Px4Event::*};
    use std::sync::Arc;

    const C: u64 = 1_100;
    const NOT_READY: &str = "PX4 did not report ready to arm within 30 s";

    /// Timeline from t0: the restart began at 1000 ms, `ekf2 start` was
    /// confirmed at `C` ms.
    async fn wait_with(
        events: Vec<(u64, Px4Event)>,
    ) -> (Result<Px4HealthReport, String>, Duration) {
        let monitor = Arc::new(Px4HealthMonitor::new());
        let t0 = Instant::now();
        let (began_at, confirmed_at) = (
            t0 + Duration::from_millis(1000),
            t0 + Duration::from_millis(C),
        );
        publish_px4_events(monitor.clone(), t0, events);
        tokio::time::sleep_until(confirmed_at).await;
        let out = wait_for_px4_ready(&monitor, began_at, confirmed_at, &Px4Timing::default()).await;
        (out, Instant::now() - confirmed_at)
    }

    fn failing_every_second() -> Vec<(u64, Px4Event)> {
        (1..=29).map(|s| (C + s * 1000, SysStatus(false))).collect()
    }

    #[tokio::test(start_paused = true)]
    async fn ready_when_the_prearm_check_passes_two_seconds_after_start() {
        let (out, took) = wait_with(vec![
            (C + 500, SysStatus(false)),
            (C + 1500, SysStatus(false)),
            (C + 2000, SysStatus(true)),
        ])
        .await;
        assert_eq!(out, Ok(Px4HealthReport::Healthy));
        assert_eq!(took, Duration::from_secs(2));
    }

    #[tokio::test(start_paused = true)]
    async fn stays_waiting_through_twelve_seconds_of_failing_checks() {
        let mut ev: Vec<_> = (1..=12).map(|s| (C + s * 1000, SysStatus(false))).collect();
        ev.push((C + 12_500, SysStatus(true)));
        let (out, took) = wait_with(ev).await;
        assert_eq!(
            (out, took),
            (Ok(Px4HealthReport::Healthy), Duration::from_millis(12_500))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn a_passing_report_inside_the_guard_does_not_end_the_wait() {
        let (out, took) = wait_with(vec![
            (C + 200, SysStatus(true)),
            (C + 1500, SysStatus(true)),
        ])
        .await;
        assert_eq!(
            (out, took),
            (Ok(Px4HealthReport::Healthy), Duration::from_millis(1500))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn a_passing_report_only_inside_the_guard_times_out() {
        let (out, took) = wait_with(vec![(C + 200, SysStatus(true))]).await;
        assert_eq!(
            (out, took),
            (Err(NOT_READY.to_string()), Duration::from_secs(30))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn gives_up_after_30_s_with_px4s_preflight_fail_text() {
        let mut ev = failing_every_second();
        ev.push((C + 2000, Text("Preflight Fail: ekf2 missing data")));
        let (out, took) = wait_with(ev).await;
        assert_eq!(
            (out, took),
            (
                Err("Preflight Fail: ekf2 missing data".to_string()),
                Duration::from_secs(30)
            )
        );
    }

    #[tokio::test(start_paused = true)]
    async fn gives_up_with_the_default_text_when_px4_gave_no_reason() {
        assert_eq!(
            wait_with(failing_every_second()).await.0,
            Err(NOT_READY.to_string())
        );
    }

    #[tokio::test(start_paused = true)]
    async fn a_preflight_fail_from_before_the_restart_is_not_the_reason() {
        let mut ev = failing_every_second();
        ev.push((500, Text("Preflight Fail: stale")));
        assert_eq!(wait_with(ev).await.0, Err(NOT_READY.to_string()));
    }

    #[tokio::test(start_paused = true)]
    async fn a_preflight_fail_sent_during_the_restart_is_the_reason() {
        let mut ev = failing_every_second();
        ev.push((1_050, Text("Preflight Fail: ekf2 missing data")));
        assert_eq!(
            wait_with(ev).await.0,
            Err("Preflight Fail: ekf2 missing data".to_string())
        );
    }

    #[tokio::test(start_paused = true)]
    async fn no_sys_status_within_three_seconds_reports_unknown() {
        assert_eq!(
            wait_with(vec![]).await,
            (Ok(Px4HealthReport::Unknown), Duration::from_secs(3))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn a_sys_status_from_before_the_restart_does_not_count() {
        assert_eq!(
            wait_with(vec![(500, SysStatus(true))]).await,
            (Ok(Px4HealthReport::Unknown), Duration::from_secs(3))
        );
    }
}
