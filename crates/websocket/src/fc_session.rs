//! The session lock shared by configure, reset and the reconnect re-push, and
//! the confirmed EKF2 restart that runs under it.

use crate::nsh_await::{judge_ekf2_start, judge_ekf2_stop, run_nsh, InternalRequestIds, NshLink};
use crate::protocol::Px4HealthReport;
use crate::px4_ready::{wait_for_px4_ready, Px4Timing};
use simulation::Px4HealthMonitor;
use std::sync::Arc;
use tokio::sync::{Mutex, MutexGuard};
use tokio::time::Instant;
use tracing::info;

pub const ARMED_REFUSAL: &str = "Land and disarm first.";
pub const BUSY_REFUSAL: &str = "Another reset or configuration is in progress.";

/// When an EKF2 restart began and when PX4 confirmed `ekf2 start`.
#[derive(Debug, Clone, Copy)]
pub struct Ekf2Restart {
    pub began_at: Instant,
    pub confirmed_at: Instant,
}

/// One flight-controller session: at most one reset, configure or re-push runs
/// at a time, and EKF2 restarts go through here.
pub struct FcSession {
    lock: Mutex<()>,
    nsh: Option<NshLink>,
    health: Arc<Px4HealthMonitor>,
    timing: Px4Timing,
    ids: InternalRequestIds,
}

impl FcSession {
    /// `nsh: None` is sim-only mode: there is no board to restart.
    pub fn new(nsh: Option<NshLink>, health: Arc<Px4HealthMonitor>) -> Self {
        Self {
            lock: Mutex::new(()),
            nsh,
            health,
            timing: Px4Timing::default(),
            ids: InternalRequestIds::new(),
        }
    }

    #[cfg(test)]
    pub(crate) fn with_timing(self, timing: Px4Timing) -> Self {
        Self { timing, ..self }
    }

    pub fn is_sim_only(&self) -> bool {
        self.nsh.is_none()
    }

    /// Takes the session without waiting; `None` means another flow holds it.
    /// Configure and reset use this and answer with `BUSY_REFUSAL` (design D2:
    /// a second request is refused, not queued behind a long flow).
    pub fn try_begin(&self) -> Option<MutexGuard<'_, ()>> {
        self.lock.try_lock().ok()
    }

    /// Takes the session, waiting for the holder. The reconnect re-push uses
    /// this (design D2): it must run once the current flow finishes, not be
    /// dropped.
    pub async fn begin(&self) -> MutexGuard<'_, ()> {
        self.lock.lock().await
    }

    /// Restarts EKF2 and returns once PX4 has confirmed both commands.
    ///
    /// Every error already reads `EKF2 restart failed: <reason>`, so callers
    /// pass it on unchanged.
    pub async fn restart_ekf2(&self) -> Result<Ekf2Restart, String> {
        self.restart_ekf2_steps()
            .await
            .map_err(|reason| format!("EKF2 restart failed: {reason}"))
    }

    async fn restart_ekf2_steps(&self) -> Result<Ekf2Restart, String> {
        let nsh = self
            .nsh
            .as_ref()
            .ok_or_else(|| "no flight controller in sim-only mode".to_string())?;
        let began_at = Instant::now();
        // No settle sleep between the steps: the awaited prompt means stop finished.
        run_nsh(nsh, self.ids.next(), "ekf2 stop", self.timing.nsh_reply)
            .await
            .and_then(|out| judge_ekf2_stop(&out))
            .map_err(|e| format!("ekf2 stop: {e}"))?;
        let stopped_at = Instant::now();
        run_nsh(nsh, self.ids.next(), "ekf2 start", self.timing.nsh_reply)
            .await
            .and_then(|out| judge_ekf2_start(&out))
            .map_err(|e| format!("ekf2 start: {e}"))?;
        let confirmed_at = Instant::now();
        self.health.mark_ekf2_started(confirmed_at);
        info!(
            stop_ms = (stopped_at - began_at).as_millis() as u64,
            start_ms = (confirmed_at - stopped_at).as_millis() as u64,
            "EKF2 restart confirmed by PX4"
        );
        Ok(Ekf2Restart {
            began_at,
            confirmed_at,
        })
    }

    /// Waits for PX4 to report it can arm after `restart` (see `px4_ready`).
    pub async fn wait_for_px4_ready(
        &self,
        restart: &Ekf2Restart,
    ) -> Result<Px4HealthReport, String> {
        wait_for_px4_ready(
            &self.health,
            restart.began_at,
            restart.confirmed_at,
            &self.timing,
        )
        .await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::{fake_nsh, ok_board, FakeReply};
    use std::sync::Mutex;
    use std::time::Duration;

    fn session(
        script: impl Fn(&str) -> FakeReply + Send + 'static,
    ) -> (FcSession, Arc<Mutex<Vec<String>>>) {
        let log = Arc::new(Mutex::new(Vec::new()));
        let nsh = fake_nsh(log.clone(), script);
        (
            FcSession::new(Some(nsh), Arc::new(Px4HealthMonitor::new())),
            log,
        )
    }

    fn sim_only() -> FcSession {
        FcSession::new(None, Arc::new(Px4HealthMonitor::new()))
    }

    #[tokio::test(start_paused = true)]
    async fn restart_sends_stop_then_start_and_confirms() {
        let (s, log) = session(ok_board);
        let r = s.restart_ekf2().await.unwrap();
        assert_eq!(*log.lock().unwrap(), ["ekf2 stop", "ekf2 start"]);
        assert!(r.confirmed_at >= r.began_at);
    }

    #[tokio::test(start_paused = true)]
    async fn restart_continues_when_ekf2_was_not_running() {
        let (s, log) = session(|cmd| match cmd {
            "ekf2 stop" => FakeReply::Ok("ekf2 stop\r\nekf2 not running\r\nnsh> "),
            other => ok_board(other),
        });
        assert!(s.restart_ekf2().await.is_ok());
        assert_eq!(*log.lock().unwrap(), ["ekf2 stop", "ekf2 start"]);
    }

    #[tokio::test(start_paused = true)]
    async fn restart_fails_when_start_reports_an_error() {
        let (s, _log) = session(|cmd| match cmd {
            "ekf2 start" => FakeReply::Ok("ekf2 start\r\nERROR [ekf2] task start failed\r\nnsh> "),
            other => ok_board(other),
        });
        assert_eq!(
            s.restart_ekf2().await.unwrap_err(),
            "EKF2 restart failed: ekf2 start: ERROR [ekf2] task start failed"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn restart_without_a_board_fails_at_stop_and_never_sends_start() {
        let (s, log) = session(|_| FakeReply::Fail("No FC connected"));
        assert_eq!(
            s.restart_ekf2().await.unwrap_err(),
            "EKF2 restart failed: ekf2 stop: No FC connected"
        );
        assert_eq!(*log.lock().unwrap(), ["ekf2 stop"]);
    }

    #[tokio::test]
    async fn a_sim_only_session_has_no_restart() {
        let s = sim_only();
        assert!(s.is_sim_only());
        assert_eq!(
            s.restart_ekf2().await.unwrap_err(),
            "EKF2 restart failed: no flight controller in sim-only mode"
        );
    }

    #[test]
    fn try_begin_is_refused_while_the_session_is_held() {
        let s = sim_only();
        let held = s.try_begin().expect("free");
        assert!(s.try_begin().is_none());
        drop(held);
        assert!(s.try_begin().is_some());
    }

    #[tokio::test(start_paused = true)]
    async fn begin_waits_for_the_holder() {
        let s = Arc::new(sim_only());
        let held = s.try_begin().unwrap();
        let waiter = {
            let s = s.clone();
            tokio::spawn(async move { drop(s.begin().await) })
        };
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(!waiter.is_finished());
        drop(held);
        waiter.await.unwrap();
    }
}
