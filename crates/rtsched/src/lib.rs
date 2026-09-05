//! Scheduling-class promotion for the HITL simulation thread.
//!
//! # This is exposure reduction, NOT the proven fix
//!
//! Read this before citing the crate as a remedy for anything.
//!
//! The 2026-09-05 session lost 148.3 ms of simulation per second of wall time.
//! The arithmetic says that shortfall was late WAKE-UPS, not slow compute: the
//! loop's own worst-case work was ~95 µs against a 2500 µs budget, and the
//! measured shortfall was ~435 µs per iteration. Even a 3x penalty for running
//! on an efficiency core is an order of magnitude short of explaining it.
//!
//! Worse for the QoS hypothesis specifically: a thread's QoS class is fixed
//! when it is promoted, so it cannot produce a STEP CHANGE at t=36 s that then
//! never recovers — which is the shape the incident actually had. The leading
//! suspect is Selective Forced Idle on a fanless Mac14,2 (MacBook Air M2,
//! confirmed by `sysctl hw.model`), which is thermal, sticky, and QoS-tiered.
//!
//! Note carefully what is NOT claimed. An earlier version of this paragraph
//! said the crate raises the QoS tier and that raising it lowers exposure to
//! SFI. It does not raise the tier: on macOS the mach time-constraint contract
//! SUPERSEDES the QoS class and leaves the thread at `QOS_CLASS_UNSPECIFIED`
//! (measured — see `macos.rs` and
//! `a_time_constraint_contract_supersedes_the_qos_tier`). What ships is a
//! real-time contract, so any SFI argument has to be made about
//! `TH_MODE_REALTIME` and not about the tier, and this crate does not have the
//! evidence to make it. The honest description of the macOS arm is "a
//! periodic real-time declaration, of unknown relevance to SFI".
//!
//! Either way it is not "the timebase fix", which is [`crate`]-independent and
//! lives in `crates/simulation/src/timebase.rs`.
//!
//! So: no line in this crate, in its logs, or in the CHANGELOG may say this
//! prevents the incident. Until `powermetrics` names the mechanism during a
//! reproduction, the honest description is "a mitigation of unknown
//! relevance, shipped because it is cheap and directionally right".
//!
//! # Shape of the API
//!
//! Every mechanism used here is SELF-DIRECTED: it acts on the calling thread
//! and cannot be applied to another thread by the one that spawned it.
//! [`promote_current_thread`] must therefore be the first statement inside a
//! spawned closure, never a call the spawner makes around it.
//!
//! The crate does no logging. It returns what happened and the caller decides
//! how loudly to say it — which keeps the platform code free of a `tracing`
//! dependency and makes the outcome a value a test can assert on.

use std::time::Duration;

#[cfg(target_os = "macos")]
mod macos;

#[cfg(target_os = "linux")]
mod linux;

#[cfg(windows)]
mod windows;

#[cfg(not(any(target_os = "macos", target_os = "linux", windows)))]
mod stub;

// One name for the platform module so the dispatch below is a single line
// rather than a cfg ladder repeated per function.
#[cfg(target_os = "linux")]
use linux as platform;
#[cfg(target_os = "macos")]
use macos as platform;
#[cfg(not(any(target_os = "macos", target_os = "linux", windows)))]
use stub as platform;
#[cfg(windows)]
use windows as platform;

/// What the host did with a promotion request.
///
/// `Unavailable` is a NORMAL outcome, not an error. On Linux, `SCHED_FIFO`
/// requires `CAP_SYS_NICE`, and the shipped Linux artefact is a static musl
/// binary that an ordinary user runs from their downloads directory — so
/// EPERM is the EXPECTED branch there, and every caller must treat it as such.
/// A daemon that refuses to run, or that warns once a second, because the
/// kernel declined an optional privilege is worse than one that never asked.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Promotion {
    /// The host granted an elevated scheduling class.
    Granted {
        /// Which mechanism was used, for the log line and the stats field.
        mechanism: &'static str,
        /// The specific parameters granted, in the platform's own units.
        detail: String,
    },
    /// The host understood the request and refused it. Expected, not fatal.
    Unavailable {
        mechanism: &'static str,
        /// Raw OS error number, so a support conversation can be specific.
        errno: i32,
        /// What the user would have to do to grant it, if anything.
        remedy: &'static str,
    },
    /// No mechanism exists for this target.
    Unsupported,
    /// Nobody asked — `--no-realtime`.
    ///
    /// Distinct from [`Promotion::Unsupported`] on purpose, and the distinction
    /// is a support-cost one: a macOS report reading "unsupported" would send
    /// someone hunting a broken FFI binding when the real answer is that the
    /// operator passed a flag. Produced by callers, never by the platform
    /// layer, which is the one thing in this enum that cannot answer "I was not
    /// asked".
    NotRequested,
}

impl Promotion {
    /// True only when the host actually granted something.
    pub fn is_granted(&self) -> bool {
        matches!(self, Promotion::Granted { .. })
    }

    /// A short, stable, machine-readable token.
    ///
    /// Goes into `SimulationStats::realtime_policy` and the
    /// `--selftest-timebase` JSON, so it is part of an interface other tools
    /// parse: keep the shape `<outcome>[:<mechanism>][:<errno>]` and keep it
    /// lowercase.
    pub fn token(&self) -> String {
        match self {
            Promotion::Granted { mechanism, .. } => format!("granted:{mechanism}"),
            Promotion::Unavailable {
                mechanism, errno, ..
            } => format!("unavailable:{mechanism}:{errno}"),
            Promotion::Unsupported => "unsupported".to_string(),
            Promotion::NotRequested => "disabled".to_string(),
        }
    }

    /// One sentence naming what was granted, what was refused, and why.
    ///
    /// The caller logs this once. It carries the remedy inline because a user
    /// reading a daemon log has no other place to find out that a Linux host
    /// needs `setcap`.
    pub fn describe(&self) -> String {
        match self {
            Promotion::Granted { mechanism, detail } => {
                format!("simulation thread promoted via {mechanism} ({detail})")
            }
            Promotion::Unavailable {
                mechanism,
                errno,
                remedy,
            } => format!(
                "simulation thread runs at normal priority: the host refused {mechanism} \
                 (errno {errno}). This is expected without extra privilege and is not an \
                 error; pacing is reported by --selftest-timebase either way. To grant it: \
                 {remedy}"
            ),
            Promotion::Unsupported => {
                "simulation thread runs at normal priority: no scheduling-promotion mechanism \
                 is implemented for this target"
                    .to_string()
            }
            Promotion::NotRequested => {
                "simulation thread runs at normal priority: the promotion was not requested"
                    .to_string()
            }
        }
    }
}

/// Whether a promotion granted earlier is still in force.
///
/// Exists because macOS is BELIEVED to revoke a time-constraint contract
/// silently when a thread chronically overruns its declared computation
/// budget. A revoked contract with nothing said about it is this incident's
/// exact shape a second time, so the loop re-reads the contract on every stats
/// window.
///
/// "Believed" is deliberate and load-bearing. The premise is not verified
/// against a source: `thread_policy_get` for `THREAD_TIME_CONSTRAINT_POLICY`
/// may consult `thread->saved_mode` as well as `sched_mode`, in which case a
/// demoted thread would keep reporting its saved contract with
/// `get_default == FALSE` and this detector would read `Intact` throughout. So
/// [`Verification::Intact`] is evidence the contract was not *replaced*; it is
/// NOT proof the kernel is still honouring it. Do not read a green
/// `realtime_policy` as a guarantee about pacing — the pacing metrics
/// (`steps_per_sec`, `wake_late_p99_us`) are the measurement, and this is a
/// cross-check on the contract itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verification {
    /// The contract set at promotion is still the one the kernel is using.
    Intact,
    /// The kernel has taken it back. Say so.
    Revoked,
    /// Nothing was promoted, or this target cannot report. Never a fault.
    NotApplicable,
}

/// Ask the host to schedule THIS thread as a periodic real-time task.
///
/// `period` is the loop's iteration interval and `computation` the worst-case
/// work inside one iteration. Both are declarations, not limits: overrunning
/// `computation` does not fail a call, it makes the kernel trust the
/// declaration less (and, on macOS, eventually revoke it — see
/// [`verify_current_thread`]).
///
/// Never panics and never returns an error type: a host that will not promote
/// the thread is a host the daemon still has to run on.
pub fn promote_current_thread(period: Duration, computation: Duration) -> Promotion {
    platform::promote(period, computation)
}

/// Re-read the kernel's view of this thread's scheduling contract.
///
/// Cheap enough to call on a 5 s cadence from inside the loop; it is one
/// syscall on macOS and a no-op elsewhere.
pub fn verify_current_thread() -> Verification {
    platform::verify()
}

/// Release anything the promotion acquired that outlives the call.
///
/// Only Windows needs this (`AvRevertMmThreadCharacteristics`); everywhere
/// else it is a no-op. Safe to call on a thread that was never promoted.
pub fn revert_current_thread() {
    platform::revert();
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 400 Hz default, and the worst-case work the loop measured
    /// (`loop_runner.rs` records ~95 µs) with margin.
    const TEST_PERIOD: Duration = Duration::from_micros(2500);
    const TEST_COMPUTATION: Duration = Duration::from_micros(250);

    /// Runs on a SPAWNED thread, not the caller's.
    ///
    /// Every mechanism here is self-directed, and macOS cannot undo one:
    /// `revert_current_thread` is a documented no-op there, so promoting the
    /// test thread would leave it holding a mach time-constraint contract and
    /// `QOS_CLASS_USER_INTERACTIVE` for the rest of its life. Harmless under
    /// libtest's default thread-per-test, but under `--test-threads=1
    /// --nocapture` libtest can run in-process on the MAIN thread, where the
    /// promotion would outlive the test.
    #[test]
    fn promote_current_thread_reports_its_outcome_and_never_panics() {
        let outcome = std::thread::spawn(|| {
            let outcome = promote_current_thread(TEST_PERIOD, TEST_COMPUTATION);
            // Releases what Windows acquires. On macOS and Linux the
            // promotion is undone only by the thread exiting, which is the
            // other reason this runs on a thread of its own.
            revert_current_thread();
            outcome
        })
        .join()
        .expect("promotion must not panic");

        // Every branch is a legal result. `Unavailable` in particular is a
        // PASS: on a Linux CI container without CAP_SYS_NICE it is the only
        // branch reachable, and a test that failed there would either be
        // deleted or ignored within a week.
        match &outcome {
            Promotion::Granted { mechanism, detail } => {
                assert!(
                    !mechanism.is_empty(),
                    "a granted promotion must name its mechanism"
                );
                assert!(
                    !detail.is_empty(),
                    "a granted promotion must name its parameters"
                );
            }
            Promotion::Unavailable {
                mechanism,
                errno,
                remedy,
            } => {
                assert!(
                    !mechanism.is_empty(),
                    "a refusal must name what was refused"
                );
                // A refusal normally quotes the OS's own reason. The one
                // exception is a call that SUCCEEDED and did not take effect —
                // macOS accepting a time-constraint contract inside a sandbox
                // and then not applying it — where there is no errno to quote
                // and the mechanism name carries the explanation instead.
                assert!(
                    *errno != 0 || mechanism.ends_with("-not-applied"),
                    "a refusal must carry either the errno that explains it or a mechanism \
                     name that does, got {mechanism:?} with errno {errno}"
                );
                assert!(
                    !remedy.is_empty(),
                    "a refusal must carry a remedy the user can act on"
                );
            }
            Promotion::Unsupported | Promotion::NotRequested => {}
        }

        // The token is an interface other tools parse.
        let token = outcome.token();
        assert_eq!(
            token,
            token.to_lowercase(),
            "the stats token must be lowercase"
        );
        assert!(
            !token.contains(' '),
            "the stats token must not contain spaces: {token}"
        );
        assert!(!outcome.describe().is_empty());
    }

    /// The four outcomes must stay distinguishable from the token alone, which
    /// is all a support conversation or a `jq` filter ever sees.
    #[test]
    fn every_outcome_has_its_own_token() {
        let tokens = [
            Promotion::Granted {
                mechanism: "mach-time-constraint",
                detail: "x".to_string(),
            }
            .token(),
            Promotion::Unavailable {
                mechanism: "sched-fifo",
                errno: 1,
                remedy: "y",
            }
            .token(),
            Promotion::Unsupported.token(),
            Promotion::NotRequested.token(),
        ];
        let unique: std::collections::BTreeSet<&String> = tokens.iter().collect();
        assert_eq!(
            unique.len(),
            tokens.len(),
            "two outcomes share a token, so a report cannot tell them apart: {tokens:?}"
        );
        assert_eq!(
            Promotion::NotRequested.token(),
            "disabled",
            "`--no-realtime` must not read as `unsupported`: on macOS that would send a \
             reader hunting a broken FFI binding for a flag the operator passed"
        );
    }

    /// Guards the 5 s re-read in the simulation loop against crying wolf.
    ///
    /// `verify` runs unconditionally on every stats window, including on a
    /// thread nothing ever promoted (`--no-realtime`, or a target with no
    /// mechanism). Reporting `Revoked` there would put a permanent false
    /// warning in front of every user who opted out.
    #[test]
    fn verify_on_an_unpromoted_thread_is_never_a_revocation() {
        // A fresh thread, deliberately not promoted.
        let seen = std::thread::spawn(verify_current_thread)
            .join()
            .expect("verify must not panic");
        assert_ne!(
            seen,
            Verification::Revoked,
            "an unpromoted thread must not report a revoked contract"
        );
    }

    #[test]
    fn revert_is_safe_on_a_thread_that_was_never_promoted() {
        std::thread::spawn(revert_current_thread)
            .join()
            .expect("revert must not panic on an unpromoted thread");
    }
}
