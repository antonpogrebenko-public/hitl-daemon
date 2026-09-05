//! How a scheduling-contract verification becomes a line in `SimulationStats`.
//!
//! Separated from the loop so the two rules that matter — say it once, and
//! never un-say it — are unit tests rather than intentions buried in a 3 600
//! line hot loop.
//!
//! None of this is the timebase fix. A `granted:` token here says the kernel
//! accepted a declaration; it says nothing about whether the loop is keeping
//! cadence, which is what `timebase_health` reports.

use rtsched::Verification;

/// Value published in `SimulationStats::realtime_policy` once a granted
/// contract is no longer the contract that was granted.
///
/// A distinct token rather than reverting to the "unavailable" one: a host that
/// granted a contract and then stopped honouring it is a different — and much
/// more interesting — state than one that never granted it.
///
/// TWO causes, and they are not the same fault. Either the kernel DEMOTED the
/// thread to the default policy, which on macOS is what it does when a thread
/// chronically overruns the computation budget it declared; or the contract in
/// force no longer matches the period and constraint `promote` declared — a
/// REPLACEMENT, which is neither a demotion nor an overrun and which
/// `get_default` alone reports as intact. `macos::verify` returns `Revoked`
/// for both, so any prose downstream of this has to name both.
pub(crate) const REVOKED: &str = "revoked";

/// The policy string a stats window should publish, and whether THIS window is
/// the one that first saw the contract taken away.
///
/// The second half of the tuple exists so the caller can log the revocation
/// exactly once. macOS revokes silently and does not re-grant, so a per-window
/// warning would be an unbounded log stream describing a single event — on the
/// 400 Hz thread, into the same lossy 512-slot TUI channel every other rate
/// limit in this loop exists to protect.
///
/// Revocation is STICKY. A later `Intact` does not clear it: the interesting
/// fact about the session is that the guarantee stopped applying at all, and a
/// metric that could quietly return to green is the failure mode this whole
/// change exists to remove.
pub(crate) fn fold_realtime_verdict(
    granted_token: &str,
    verdict: Verification,
    already_revoked: bool,
) -> (String, bool) {
    if already_revoked {
        return (REVOKED.to_string(), false);
    }
    match verdict {
        Verification::Revoked => (REVOKED.to_string(), true),
        // `NotApplicable` covers a thread nothing promoted (`--no-realtime`,
        // an unprivileged Linux host, a target with no mechanism) AND a
        // read that failed. Neither is evidence of a revocation, and treating
        // them as one would put a permanent false warning in front of the
        // largest population of users this daemon has.
        Verification::Intact | Verification::NotApplicable => (granted_token.to_string(), false),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GRANTED: &str = "granted:mach-time-constraint";
    const REFUSED: &str = "unavailable:sched-fifo:1";

    #[test]
    fn an_intact_contract_publishes_what_was_granted() {
        let (policy, newly) = fold_realtime_verdict(GRANTED, Verification::Intact, false);
        assert_eq!(policy, GRANTED);
        assert!(!newly);
    }

    /// The false-alarm guard. Every unprivileged Linux run and every
    /// `--no-realtime` run takes this path on every stats window for the life
    /// of the session.
    #[test]
    fn a_thread_that_was_never_promoted_is_not_reported_as_revoked() {
        let (policy, newly) = fold_realtime_verdict(REFUSED, Verification::NotApplicable, false);
        assert_eq!(
            policy, REFUSED,
            "a host that never granted a contract has not had one revoked"
        );
        assert!(
            !newly,
            "an unpromoted thread must never trigger the revocation warning"
        );
    }

    #[test]
    fn the_first_revoked_window_is_the_only_one_that_reports_it() {
        let (policy, newly) = fold_realtime_verdict(GRANTED, Verification::Revoked, false);
        assert_eq!(policy, REVOKED);
        assert!(
            newly,
            "the first window to see a revocation must be able to log it"
        );

        let (policy, newly) = fold_realtime_verdict(GRANTED, Verification::Revoked, true);
        assert_eq!(policy, REVOKED);
        assert!(
            !newly,
            "a revocation is one event; logging it once per 5 s window would be an \
             unbounded stream into the loop's lossy log channel"
        );
    }

    /// macOS does not re-grant a contract it withdrew, and a metric that could
    /// go quietly green again is the exact failure this change exists to
    /// remove.
    #[test]
    fn a_revocation_is_sticky_even_if_a_later_read_says_intact() {
        let (policy, newly) = fold_realtime_verdict(GRANTED, Verification::Intact, true);
        assert_eq!(policy, REVOKED);
        assert!(!newly);

        let (policy, _) = fold_realtime_verdict(GRANTED, Verification::NotApplicable, true);
        assert_eq!(policy, REVOKED);
    }
}
