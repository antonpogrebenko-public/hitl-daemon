//! Windows: time-critical thread priority plus an MMCSS "Pro Audio"
//! registration.
//!
//! Explicitly NOT `timeBeginPeriod`. `spin_sleep` 1.3.3 — which this daemon
//! already uses for the loop's sleep — calls it itself and additionally uses
//! `CreateWaitableTimerExW` with `CREATE_WAITABLE_TIMER_HIGH_RESOLUTION`.
//! Calling it again here would raise the process's timer-resolution refcount a
//! second time for no change in behaviour, and every such call is a
//! system-wide power cost.
//!
//! `avrt` is declared by hand rather than pulled from a `windows-sys` feature:
//! `libavrt.a` is present in the mingw-w64 14.0.0 sysroot this project
//! cross-compiles against, so the link resolves with no extra build flags and
//! no additional feature surface.

use crate::{Promotion, Verification};
use std::time::Duration;
use windows_sys::Win32::Foundation::HANDLE;
use windows_sys::Win32::System::Threading::{
    GetCurrentThread, GetThreadPriority, SetThreadPriority, THREAD_PRIORITY_TIME_CRITICAL,
};
// A different header from the priority values it is compared against, and a
// `u32` where they are `i32`. Imported rather than written out because the
// value is NOT what a reader guesses: see [`verify`].
use windows_sys::Win32::System::WindowsProgramming::THREAD_PRIORITY_ERROR_RETURN;

/// Mechanism name carried in [`Promotion::token`] and the log line.
const MECHANISM: &str = "mmcss-pro-audio";

/// MMCSS task class. "Pro Audio" is the highest-guarantee class Windows
/// publishes, and a 400 Hz fixed-period loop is the same scheduling shape as
/// an audio callback: a small amount of work that has to happen on a deadline.
///
/// UTF-16 with an explicit NUL, because `AvSetMmThreadCharacteristicsW` takes
/// a wide C string.
const TASK_NAME: &[u16] = &[
    b'P' as u16,
    b'r' as u16,
    b'o' as u16,
    b' ' as u16,
    b'A' as u16,
    b'u' as u16,
    b'd' as u16,
    b'i' as u16,
    b'o' as u16,
    0,
];

/// What a user could do about a refusal.
///
/// MMCSS registration can fail if the Multimedia Class Scheduler service is
/// disabled, which some "gaming optimisation" guides recommend. That is the
/// only realistic cause and it is worth naming.
const REMEDY: &str = "check that the `MMCSS` (Multimedia Class Scheduler) service is running; \
                      some system tuning guides disable it";

/// `INVALID_HANDLE_VALUE`, which `AvSetMmThreadCharacteristicsW` returns on
/// failure. Written as the pointer form rather than `-1isize as HANDLE` so the
/// comparison below reads the same way on both pointer widths.
fn invalid_handle() -> HANDLE {
    -1isize as HANDLE
}

#[link(name = "avrt")]
extern "system" {
    fn AvSetMmThreadCharacteristicsW(TaskName: *const u16, TaskIndex: *mut u32) -> HANDLE;
    fn AvRevertMmThreadCharacteristics(AvrtHandle: HANDLE) -> i32;
}

// The MMCSS registration is the one thing here that DOES outlive the call: it
// is a handle the thread must hand back. Kept thread-local because
// `AvSetMmThreadCharacteristicsW` registers the CALLING thread, so the handle
// and the thread it belongs to can never be separated.
std::thread_local! {
    static AVRT_HANDLE: std::cell::Cell<isize> = const { std::cell::Cell::new(0) };
    static PROMOTED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

pub(crate) fn promote(_period: Duration, _computation: Duration) -> Promotion {
    // Neither Windows mechanism takes a period or a computation budget: MMCSS
    // infers its guarantee from the task class, and thread priority is a
    // single number. The parameters are unused rather than mapped onto a
    // priority through an invented formula.
    let mut task_index: u32 = 0;

    // SAFETY: `GetCurrentThread` returns a pseudo-handle that needs no close;
    // `SetThreadPriority` takes it by value. `AvSetMmThreadCharacteristicsW`
    // reads a NUL-terminated wide string and writes one u32 through
    // `task_index`.
    let (priority_ok, avrt) = unsafe {
        let thread = GetCurrentThread();
        let priority_ok = SetThreadPriority(thread, THREAD_PRIORITY_TIME_CRITICAL) != 0;
        let avrt = AvSetMmThreadCharacteristicsW(TASK_NAME.as_ptr(), &mut task_index);
        (priority_ok, avrt)
    };

    let mmcss_ok = avrt != invalid_handle() && !avrt.is_null();
    if mmcss_ok {
        AVRT_HANDLE.with(|h| h.set(avrt as isize));
    }

    if !priority_ok && !mmcss_ok {
        PROMOTED.with(|p| p.set(false));
        return Promotion::Unavailable {
            mechanism: MECHANISM,
            errno: std::io::Error::last_os_error().raw_os_error().unwrap_or(0),
            remedy: REMEDY,
        };
    }

    // Either half on its own is a real improvement, so a partial success is
    // reported as granted with the missing half named rather than discarded.
    //
    // `PROMOTED` deliberately records only the PRIORITY half, because that is
    // the only half `verify` can re-read: Windows publishes no query for an
    // MMCSS registration (`AvSetMmThreadCharacteristicsW` hands back a handle,
    // not a state anyone can poll). So a run that got MMCSS but not priority
    // reports `Granted` here and `NotApplicable` from `verify` forever, and a
    // later loss of the MMCSS registration is invisible. That asymmetry is
    // accepted rather than papered over: reporting `Revoked` for a half nobody
    // can read would be a false alarm, and this crate's whole job is to keep
    // `realtime_policy` believable.
    PROMOTED.with(|p| p.set(priority_ok));
    Promotion::Granted {
        mechanism: MECHANISM,
        detail: format!(
            "THREAD_PRIORITY_TIME_CRITICAL {}, MMCSS \"Pro Audio\" {} (task index {task_index})",
            if priority_ok { "granted" } else { "REFUSED" },
            if mmcss_ok { "registered" } else { "REFUSED" },
        ),
    }
}

pub(crate) fn verify() -> Verification {
    if !PROMOTED.with(|p| p.get()) {
        return Verification::NotApplicable;
    }

    // SAFETY: pseudo-handle in, plain i32 out, no out-parameters.
    let priority = unsafe { GetThreadPriority(GetCurrentThread()) };
    classify_priority(priority)
}

/// Turn a `GetThreadPriority` result into a verdict.
///
/// Split out of [`verify`] because it carries the HIGH bug this module shipped
/// with and none of it is FFI: as a free function it can be unit-tested for the
/// failed-read case, which is otherwise a branch that has never executed in any
/// process. `verify` cannot be tested that way — it would need the syscall to
/// fail on demand.
fn classify_priority(priority: i32) -> Verification {
    // A failed read is not evidence of a revocation, and reporting one here
    // would be a false alarm on the very metric this change exists to make
    // believable: `fold_realtime_verdict` is STICKY, so one bad read would pin
    // `realtime_policy` to "revoked" for the whole session and fire a warning
    // asserting the loop is overrunning.
    //
    // `THREAD_PRIORITY_ERROR_RETURN` is MAXLONG (2147483647), NOT the `i32::MIN`
    // that reads like a failure sentinel — that literal was here and made this
    // guard dead code. It is a `u32` in a different header from the `i32`
    // priority values, hence the cast rather than a hand-written number.
    if priority as u32 == THREAD_PRIORITY_ERROR_RETURN {
        return Verification::NotApplicable;
    }
    if priority == THREAD_PRIORITY_TIME_CRITICAL {
        Verification::Intact
    } else {
        Verification::Revoked
    }
}

pub(crate) fn revert() {
    AVRT_HANDLE.with(|h| {
        let raw = h.get();
        if raw == 0 {
            return;
        }
        h.set(0);
        // SAFETY: the handle was produced by `AvSetMmThreadCharacteristicsW`
        // on THIS thread and is cleared above before the call, so it can be
        // reverted at most once.
        unsafe {
            AvRevertMmThreadCharacteristics(raw as HANDLE);
        }
    });
    PROMOTED.with(|p| p.set(false));
}

#[cfg(test)]
mod tests {
    use super::*;
    use windows_sys::Win32::System::Threading::THREAD_PRIORITY_NORMAL;

    /// The HIGH bug this module shipped with, pinned.
    ///
    /// `GetThreadPriority` signals failure by returning `THREAD_PRIORITY_ERROR_RETURN`,
    /// which is MAXLONG — not `i32::MIN`, and not any negative number. The
    /// original code compared against `i32::MIN`, so the failed-read guard in
    /// [`verify`] never fired: a failing read fell through to the
    /// `THREAD_PRIORITY_TIME_CRITICAL` comparison, missed, and returned
    /// `Verification::Revoked`. That verdict is sticky, so one failed syscall
    /// permanently pinned `SimulationStats::realtime_policy` to "revoked" and
    /// fired a warning asserting the loop's own work was overrunning.
    ///
    /// Windows cannot be executed on the machine this crate is developed on, so
    /// this test and the cross-compile are the only guards the value has.
    #[test]
    fn the_failed_read_sentinel_is_maxlong_not_a_negative_number() {
        assert_eq!(
            THREAD_PRIORITY_ERROR_RETURN, 2_147_483_647,
            "GetThreadPriority's failure sentinel is MAXLONG; if this ever changes, \
             `verify`'s guard has to change with it"
        );
        assert_ne!(
            THREAD_PRIORITY_ERROR_RETURN,
            i32::MIN as u32,
            "the sentinel is NOT i32::MIN — that literal is the bug this test exists for"
        );
        assert_ne!(
            THREAD_PRIORITY_ERROR_RETURN as i32, THREAD_PRIORITY_TIME_CRITICAL,
            "a failed read must not be mistakable for the priority that was granted"
        );
    }

    /// The classification itself, over all three branches.
    ///
    /// The constant test above pins the windows-sys value; it does not exercise
    /// [`verify`]'s use of it, and the failed-read branch has never been taken
    /// in any process. This is the coverage that branch has.
    #[test]
    fn a_failed_read_is_not_a_revocation() {
        assert_eq!(
            classify_priority(THREAD_PRIORITY_ERROR_RETURN as i32),
            Verification::NotApplicable,
            "GetThreadPriority failed; that is not evidence the contract was \
             withdrawn, and `fold_realtime_verdict` is sticky — one bad read \
             would pin realtime_policy to \"revoked\" for the whole session"
        );
        assert_eq!(
            classify_priority(THREAD_PRIORITY_TIME_CRITICAL),
            Verification::Intact
        );
        assert_eq!(
            classify_priority(THREAD_PRIORITY_NORMAL),
            Verification::Revoked,
            "a thread that has been dropped back to a normal priority HAS lost \
             what it was granted"
        );
        // And the sentinel must not be reachable as an ordinary demotion, or
        // the first branch would be swallowing a real revocation.
        assert_ne!(
            classify_priority(THREAD_PRIORITY_ERROR_RETURN as i32),
            Verification::Revoked
        );
    }
}
