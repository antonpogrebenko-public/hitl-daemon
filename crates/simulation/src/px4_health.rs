//! PX4's own verdict on whether it can arm, captured from the receiver.
//!
//! `SYS_STATUS` carries the `MAV_SYS_STATUS_PREARM_CHECK` bit in
//! `onboard_control_sensors_health`; PX4 sets it while its pre-arm checks pass
//! in the current flight mode. The latest "Preflight Fail" `STATUSTEXT` is kept
//! so a failed wait can name PX4's own reason.

use mavlink::ardupilotmega::{MavMessage, MavSysStatusSensor, SYS_STATUS_DATA};
use parking_lot::Mutex;
use tokio::sync::watch;
use tokio::time::Instant;
use tracing::info;

/// PX4 prefixes every pre-arm refusal text with this.
pub const PREFLIGHT_FAIL_PREFIX: &str = "Preflight Fail";

/// One `SYS_STATUS` verdict and when it was received.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Px4Health {
    pub can_arm: bool,
    pub received_at: Instant,
}

/// True when PX4 reports its pre-arm checks passing. Reads the health field:
/// `present` only says the check exists.
pub fn prearm_check_passing(sys: &SYS_STATUS_DATA) -> bool {
    sys.onboard_control_sensors_health
        .contains(MavSysStatusSensor::MAV_SYS_STATUS_PREARM_CHECK)
}

/// Shared between the receiver tasks (writers) and the reset flow (reader).
pub struct Px4HealthMonitor {
    health_tx: watch::Sender<Option<Px4Health>>,
    preflight_fail: Mutex<Option<(String, Instant)>>,
    ekf2_started_at: Mutex<Option<Instant>>,
}

impl Default for Px4HealthMonitor {
    fn default() -> Self {
        Self::new()
    }
}

impl Px4HealthMonitor {
    pub fn new() -> Self {
        let (health_tx, _) = watch::channel(None);
        Self {
            health_tx,
            preflight_fail: Mutex::new(None),
            ekf2_started_at: Mutex::new(None),
        }
    }

    /// Feeds one received message; everything but `SYS_STATUS` and
    /// `STATUSTEXT` is ignored.
    pub fn observe(&self, msg: &MavMessage, at: Instant) {
        match msg {
            MavMessage::SYS_STATUS(sys) => self.record_sys_status(prearm_check_passing(sys), at),
            MavMessage::STATUSTEXT(status) => {
                let text = std::str::from_utf8(&status.text)
                    .unwrap_or("")
                    .trim_end_matches('\0');
                self.record_statustext(text, at);
            }
            _ => {}
        }
    }

    /// Publishes a verdict. `send_replace` stores it even with no subscriber.
    pub fn record_sys_status(&self, can_arm: bool, at: Instant) {
        let previous = self.health_tx.send_replace(Some(Px4Health {
            can_arm,
            received_at: at,
        }));
        if previous.map(|h| h.can_arm) != Some(can_arm) {
            let started = *self.ekf2_started_at.lock();
            let since = started.map(|s| at.saturating_duration_since(s).as_millis() as u64);
            info!(can_arm, since_ekf2_start_ms = ?since, "PX4 pre-arm check changed");
        }
    }

    /// Keeps the text only when it is a pre-arm refusal.
    pub fn record_statustext(&self, text: &str, at: Instant) {
        if text.starts_with(PREFLIGHT_FAIL_PREFIX) {
            *self.preflight_fail.lock() = Some((text.to_string(), at));
        }
    }

    pub fn subscribe(&self) -> watch::Receiver<Option<Px4Health>> {
        self.health_tx.subscribe()
    }

    pub fn latest(&self) -> Option<Px4Health> {
        *self.health_tx.borrow()
    }

    /// The latest "Preflight Fail" text, when received at or after `since`.
    pub fn preflight_fail_since(&self, since: Instant) -> Option<String> {
        self.preflight_fail
            .lock()
            .as_ref()
            .filter(|(_, at)| *at >= since)
            .map(|(text, _)| text.clone())
    }

    /// Anchors the `since_ekf2_start_ms` field of the change log.
    pub fn mark_ekf2_started(&self, at: Instant) {
        *self.ekf2_started_at.lock() = Some(at);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::ardupilotmega::STATUSTEXT_DATA;
    use std::time::Duration;

    const PREARM: MavSysStatusSensor = MavSysStatusSensor::MAV_SYS_STATUS_PREARM_CHECK;

    fn sys_status(health: MavSysStatusSensor, present: MavSysStatusSensor) -> MavMessage {
        MavMessage::SYS_STATUS(SYS_STATUS_DATA {
            onboard_control_sensors_health: health,
            onboard_control_sensors_present: present,
            ..Default::default()
        })
    }

    fn statustext(s: &str) -> MavMessage {
        let mut text = [0u8; 50];
        text[..s.len()].copy_from_slice(s.as_bytes());
        MavMessage::STATUSTEXT(STATUSTEXT_DATA {
            text,
            ..Default::default()
        })
    }

    #[test]
    fn the_prearm_bit_is_read_from_the_health_field_not_present() {
        let MavMessage::SYS_STATUS(pass) = sys_status(PREARM, MavSysStatusSensor::empty()) else {
            unreachable!()
        };
        let MavMessage::SYS_STATUS(present_only) = sys_status(MavSysStatusSensor::empty(), PREARM)
        else {
            unreachable!()
        };
        assert!(prearm_check_passing(&pass));
        assert!(!prearm_check_passing(&present_only));
    }

    #[test]
    fn a_sys_status_publishes_can_arm_with_its_time() {
        let m = Px4HealthMonitor::new();
        let rx = m.subscribe();
        let t = Instant::now();
        m.observe(&sys_status(PREARM, MavSysStatusSensor::empty()), t);
        assert_eq!(
            m.latest(),
            Some(Px4Health {
                can_arm: true,
                received_at: t
            })
        );
        assert!(rx.has_changed().unwrap());
    }

    #[test]
    fn only_preflight_fail_texts_are_kept() {
        let (m, t0) = (Px4HealthMonitor::new(), Instant::now());
        m.observe(&statustext("Takeoff detected"), t0);
        assert_eq!(m.preflight_fail_since(t0), None);
        m.observe(&statustext("Preflight Fail: ekf2 missing data"), t0);
        assert_eq!(
            m.preflight_fail_since(t0).as_deref(),
            Some("Preflight Fail: ekf2 missing data")
        );
    }

    #[test]
    fn preflight_fail_since_ignores_older_texts() {
        let (m, t0) = (Px4HealthMonitor::new(), Instant::now());
        m.record_statustext("Preflight Fail: old", t0);
        assert_eq!(m.preflight_fail_since(t0 + Duration::from_secs(1)), None);
    }

    #[test]
    fn nothing_is_reported_before_the_first_sys_status() {
        assert_eq!(Px4HealthMonitor::new().latest(), None);
    }
}
