//! The daemon's session-scoped copy of a parameter snapshot.
//!
//! Deliberately in memory only. The browser is the system of record: it
//! persists the snapshot to `localStorage` and replicates it to the user's
//! account. The daemon keeps a copy so a restore issued during the same session
//! does not need a round trip, and nothing more.
//!
//! Writing it to disk would make the daemon stateful, against the standing
//! convention that these crates read no runtime files, and would create a
//! second source of truth that could disagree with the browser's. A restarted
//! daemon therefore knows nothing until a browser supplies a snapshot, which is
//! the correct behaviour rather than a limitation: the browser's copy is the
//! one that survived.

use crate::protocol::SnapshotParam;
use std::sync::RwLock;

/// A snapshot bound to the board it was taken from.
#[derive(Debug, Clone, PartialEq)]
pub struct StoredSnapshot {
    pub board_identity: String,
    pub params: Vec<SnapshotParam>,
}

/// Session-lifetime snapshot holder. Cleared when the process exits.
#[derive(Debug, Default)]
pub struct SessionSnapshot {
    inner: RwLock<Option<StoredSnapshot>>,
}

impl SessionSnapshot {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a snapshot for this session.
    ///
    /// Replaces any previous one: a newer capture from the same browser is the
    /// better restore point, and a capture from a different board means the
    /// previous board is no longer the one connected.
    pub fn store(&self, snapshot: StoredSnapshot) {
        *write_recovering(&self.inner) = Some(snapshot);
    }

    /// Fetch the snapshot for `board_identity`, if one is held for that board.
    ///
    /// Returns `None` on a mismatch rather than the held snapshot. Restoring
    /// one board's tuning onto another is the failure this whole mechanism
    /// exists to prevent, so the check belongs at every read.
    pub fn get(&self, board_identity: &str) -> Option<StoredSnapshot> {
        let guard = read_recovering(&self.inner);
        let held = guard.as_ref()?;
        if held.board_identity != board_identity {
            return None;
        }
        Some(held.clone())
    }

    /// Whether any snapshot is held, regardless of board.
    pub fn is_empty(&self) -> bool {
        read_recovering(&self.inner).is_none()
    }

    /// Drop the held snapshot.
    pub fn clear(&self) {
        *write_recovering(&self.inner) = None;
    }
}

/// Recover from lock poisoning rather than propagating a panic.
///
/// The guarded value is a plain struct with no cross-field invariant, so a
/// panic elsewhere cannot leave it half-updated. Refusing to hand back the
/// snapshot because an unrelated thread died would strand the user's flight
/// controller in HITL mode — a far worse outcome than reading data that is,
/// in fact, intact.
fn read_recovering<T>(lock: &RwLock<T>) -> std::sync::RwLockReadGuard<'_, T> {
    lock.read().unwrap_or_else(|e| e.into_inner())
}

fn write_recovering<T>(lock: &RwLock<T>) -> std::sync::RwLockWriteGuard<'_, T> {
    lock.write().unwrap_or_else(|e| e.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snapshot(board: &str) -> StoredSnapshot {
        StoredSnapshot {
            board_identity: board.to_string(),
            params: vec![SnapshotParam {
                name: "SYS_HITL".to_string(),
                value: 0.0,
                param_type: "int32".to_string(),
            }],
        }
    }

    #[test]
    fn a_fresh_daemon_holds_nothing() {
        // Models a daemon restart: the process has no memory of prior sessions
        // and must wait for a browser to supply the snapshot it persisted.
        let store = SessionSnapshot::new();
        assert!(store.is_empty());
        assert_eq!(store.get("uid:3034510f33323831"), None);
    }

    #[test]
    fn a_supplied_snapshot_is_readable_for_its_board() {
        let store = SessionSnapshot::new();
        store.store(snapshot("uid:aaaa"));

        let held = store.get("uid:aaaa").expect("stored for this board");
        assert_eq!(held.params.len(), 1);
        assert_eq!(held.params[0].param_type, "int32");
    }

    #[test]
    fn a_snapshot_is_not_returned_for_a_different_board() {
        // The dangerous case: a second board plugged in mid-session must not
        // be handed the first board's tuning.
        let store = SessionSnapshot::new();
        store.store(snapshot("uid:aaaa"));
        assert_eq!(store.get("uid:bbbb"), None);
    }

    #[test]
    fn a_newer_capture_replaces_the_previous_one() {
        let store = SessionSnapshot::new();
        store.store(snapshot("uid:aaaa"));

        let mut newer = snapshot("uid:aaaa");
        newer.params[0].value = 1.0;
        store.store(newer);

        assert_eq!(store.get("uid:aaaa").unwrap().params[0].value, 1.0);
    }

    #[test]
    fn clearing_returns_the_store_to_empty() {
        let store = SessionSnapshot::new();
        store.store(snapshot("uid:aaaa"));
        store.clear();
        assert!(store.is_empty());
        assert_eq!(store.get("uid:aaaa"), None);
    }
}
