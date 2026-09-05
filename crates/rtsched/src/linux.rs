//! Linux: `SCHED_FIFO`, where refusal is the expected outcome.
//!
//! Read the [`REMEDY`] comment before treating an EPERM here as a bug. The
//! shipped Linux artefact is a static musl binary that people download and run
//! from their own account; `SCHED_FIFO` needs `CAP_SYS_NICE`, which such a
//! process does not have and cannot give itself. EPERM is therefore the
//! ORDINARY path on the ordinary install, and the daemon must run perfectly
//! well on it — logged once at `info!`, never at `warn!`, never repeatedly,
//! and never fatal.
//!
//! There is deliberately no `nice(-20)` fallback: lowering the nice value below
//! zero needs exactly the same `CAP_SYS_NICE`, so a fallback would be a second
//! call failing for the identical reason and a second line of log saying so.

use crate::{Promotion, Verification};
use std::time::Duration;

/// Mechanism name carried in [`Promotion::token`] and the log line.
const MECHANISM: &str = "sched-fifo";

/// Static priority for the simulation thread under `SCHED_FIFO`.
///
/// 10 out of the 1..99 real-time band. Deliberately low in it: the loop must
/// outrank ordinary timeshare work, but it must NOT outrank kernel threads or
/// the system's own audio and network servicing, and a runaway at high FIFO
/// priority on a single-core host is a machine that has to be power-cycled.
/// 10 is the value the pro-audio world settled on for the same trade.
const RT_PRIORITY: libc::c_int = 10;

/// What a user would have to do to grant the privilege.
///
/// Spelled out in full because there is no packaging step where this could
/// have been explained: the artefact is a single binary fetched by a curl
/// pipeline.
const REMEDY: &str = "run `sudo setcap cap_sys_nice+ep <path to hitl-daemon>` once, or start \
                      the daemon as root";

// Set by `promote` on the thread it promoted, read by `verify`. See the macOS
// module for why this is thread-local: `sched_setscheduler(0, ...)` is
// self-directed in exactly the same way.
std::thread_local! {
    static PROMOTED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

pub(crate) fn promote(_period: Duration, _computation: Duration) -> Promotion {
    // `SCHED_FIFO` has no way to express a period or a computation budget —
    // it is "run until you yield or something more important arrives". Both
    // parameters are therefore unused here rather than approximated into a
    // priority number, which would be an invented mapping with no basis.
    //
    // Zeroed rather than field-initialised: musl's `sched_param` carries four
    // sporadic-server fields (`sched_ss_*`) that glibc's does not, so naming
    // fields here compiles on one libc and not the other. Zero is the correct
    // value for every one of them under `SCHED_FIFO`.
    //
    // SAFETY: `sched_param` is a plain C struct of integers, for which
    // all-zeroes is a valid bit pattern.
    let mut param: libc::sched_param = unsafe { std::mem::zeroed() };
    param.sched_priority = RT_PRIORITY;

    // SAFETY: pid 0 means the calling thread; `sched_setscheduler` reads a
    // `sched_param` through the pointer and does not retain it.
    let rc = unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) };

    if rc != 0 {
        // Read the errno FIRST, before touching anything else. A
        // const-initialised non-Drop thread-local compiles to a plain store
        // today and cannot clobber errno, but the whole value of `Unavailable`
        // is that the number is quotable in a support conversation — and a
        // later change to that thread-local (a Drop type, a lazy initialiser, a
        // log line) would silently turn it into garbage with nothing to catch
        // it.
        let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
        PROMOTED.with(|p| p.set(false));
        return Promotion::Unavailable {
            mechanism: MECHANISM,
            errno,
            remedy: REMEDY,
        };
    }

    PROMOTED.with(|p| p.set(true));
    Promotion::Granted {
        mechanism: MECHANISM,
        detail: format!("SCHED_FIFO priority {RT_PRIORITY}"),
    }
}

pub(crate) fn verify() -> Verification {
    if !PROMOTED.with(|p| p.get()) {
        // Nothing was set on this thread, so there is nothing to revoke.
        // Without this every unprivileged run — which is the normal Linux
        // install — would report a revocation on every stats window.
        return Verification::NotApplicable;
    }

    // SAFETY: pid 0 is the calling thread; the call has no out-parameters.
    let policy = unsafe { libc::sched_getscheduler(0) };
    if policy < 0 {
        // A failed read is not evidence of a revocation.
        return Verification::NotApplicable;
    }
    if policy == libc::SCHED_FIFO {
        Verification::Intact
    } else {
        Verification::Revoked
    }
}

pub(crate) fn revert() {
    // Nothing to release: the scheduling policy dies with the thread and holds
    // no kernel resource that outlives it.
}
