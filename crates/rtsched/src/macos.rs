//! macOS: a mach time-constraint contract, which SUPERSEDES the QoS tier.
//!
//! The two are asked for in that order and only one of them survives. Measured
//! on a Mac14,2 (Apple M2, macOS 27.0 — the incident machine), with a probe
//! making the same calls in the same order on a spawned thread:
//! `pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE)` returns 0 and
//! the class reads back 0x21; the immediately following
//! `thread_policy_set(THREAD_TIME_CONSTRAINT_POLICY)` returns KERN_SUCCESS and
//! the class then reads back 0x00, `QOS_CLASS_UNSPECIFIED`. The contract takes
//! the thread out of QoS management entirely. Reversing the order does not
//! help either: with the contract in force `pthread_set_qos_class_self_np`
//! returns 1 and the class stays 0x00. So what ships on macOS is a
//! time-constraint contract INSTEAD OF a raised QoS tier, and any
//! Selective-Forced-Idle argument has to be made about `TH_MODE_REALTIME`
//! rather than about the tier.
//!
//! The QoS request is still made, and still worth making: it is what the
//! thread is left holding when the contract is REFUSED (a sandbox or a
//! device-management profile), which is the one case where the tier is the
//! only mitigation in force. [`promote`] therefore decides the mechanism it
//! reports from a READ-BACK of the tier, never from the return code of the
//! call that asked for it.
//!
//! The mach entry points are declared here by hand rather than taken from
//! `libc`. `libc` does carry them, but marks every one `#[deprecated]` ("use
//! the `mach2` crate instead") — so using them costs thirteen warnings today
//! and a broken build whenever libc finally drops them, in exchange for
//! nothing: these four symbols live in libSystem, which every macOS binary
//! already links, and their signatures are fixed by a stable kernel ABI.
//!
//! `pthread_set_qos_class_self_np` is taken from `libc`, which declares it in
//! its non-deprecated `apple/libpthread` tree.

use crate::{Promotion, Verification};
use std::time::Duration;

// ── mach types, as declared in <mach/*.h> ────────────────────────────────
#[allow(non_camel_case_types)]
type kern_return_t = libc::c_int;
#[allow(non_camel_case_types)]
type mach_port_t = libc::c_uint;
#[allow(non_camel_case_types)]
type thread_t = mach_port_t;
#[allow(non_camel_case_types)]
type natural_t = libc::c_uint;
#[allow(non_camel_case_types)]
type thread_policy_flavor_t = natural_t;
#[allow(non_camel_case_types)]
type mach_msg_type_number_t = natural_t;
#[allow(non_camel_case_types)]
type integer_t = libc::c_int;
/// `int` on arm64, `unsigned int` on x86_64 macOS. Both are 32 bits and both
/// carry 0/1, so the alias is taken from `libc` rather than picked here.
#[allow(non_camel_case_types)]
type boolean_t = libc::boolean_t;

const KERN_SUCCESS: kern_return_t = 0;

/// `THREAD_TIME_CONSTRAINT_POLICY` from `<mach/thread_policy.h>`.
const THREAD_TIME_CONSTRAINT_POLICY: thread_policy_flavor_t = 2;

/// `struct thread_time_constraint_policy` from `<mach/thread_policy.h>`.
///
/// All three time fields are mach ABSOLUTE-TIME TICKS. See [`to_mach_ticks`].
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct ThreadTimeConstraintPolicy {
    period: u32,
    computation: u32,
    constraint: u32,
    preemptible: boolean_t,
}

/// `THREAD_TIME_CONSTRAINT_POLICY_COUNT`: the struct's size in `integer_t`s,
/// computed rather than written as `4` so it cannot drift from the struct.
const THREAD_TIME_CONSTRAINT_POLICY_COUNT: mach_msg_type_number_t =
    (std::mem::size_of::<ThreadTimeConstraintPolicy>() / std::mem::size_of::<integer_t>())
        as mach_msg_type_number_t;

/// `struct mach_timebase_info` from `<mach/mach_time.h>`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct MachTimebaseInfo {
    numer: u32,
    denom: u32,
}

extern "C" {
    fn mach_timebase_info(info: *mut MachTimebaseInfo) -> kern_return_t;
    fn mach_thread_self() -> thread_t;
    fn mach_port_deallocate(task: mach_port_t, name: mach_port_t) -> kern_return_t;
    fn thread_policy_set(
        thread: thread_t,
        flavor: thread_policy_flavor_t,
        policy_info: *mut integer_t,
        count: mach_msg_type_number_t,
    ) -> kern_return_t;
    fn thread_policy_get(
        thread: thread_t,
        flavor: thread_policy_flavor_t,
        policy_info: *mut integer_t,
        count: *mut mach_msg_type_number_t,
        get_default: *mut boolean_t,
    ) -> kern_return_t;
    /// `mach_task_self()` is a macro over this global in `<mach/mach_init.h>`.
    static mach_task_self_: mach_port_t;
}

/// Slack the kernel is allowed between the start of a period and the moment
/// this thread's computation must be finished.
///
/// Sized as a fraction of the period rather than equal to it. Setting
/// `constraint == period` says "anywhere in the period will do", which is what
/// an ordinary timeshare thread already gets — it asks the scheduler for
/// nothing. 1 ms leaves real room to place the thread while still being 4x the
/// declared computation.
const CONSTRAINT: Duration = Duration::from_micros(1000);

/// Mechanism name carried in [`Promotion::token`] and the log line.
const MECHANISM: &str = "mach-time-constraint";

/// Mechanism name for a contract in force on a thread that is NOT at
/// `QOS_CLASS_USER_INTERACTIVE`.
///
/// A separate name rather than a word buried in `detail`, because
/// [`Promotion::token`] is the single machine-readable field the A/B and every
/// support report are graded on, and a token must never claim a half that is
/// not in force.
///
/// This is the ORDINARY macOS outcome, not an exotic one: the contract clears
/// the tier (see the module header), so a granted contract normally reports
/// this token and not [`MECHANISM`]. It was written for the case where
/// `pthread_set_qos_class_self_np` itself failed, and was selected on that
/// call's return code — which always succeeds, so the variant was unreachable
/// and every macOS session published the full-grant token on a thread whose
/// tier was `QOS_CLASS_UNSPECIFIED`.
const MECHANISM_NO_QOS: &str = "mach-time-constraint-noqos";

/// Mechanism name for a contract the kernel accepted and did not apply.
///
/// `thread_policy_set` returning KERN_SUCCESS is not evidence the policy took:
/// a sandbox or a device-management profile can accept and ignore it, which is
/// the case [`REMEDY`] names. Reporting that as `granted:` would make the A/B
/// compare two identical arms and would delay the first honest signal by a
/// whole stats window — where it would arrive as a REVOCATION, a diagnosis
/// that is flatly wrong for a contract that never applied.
const MECHANISM_NOT_APPLIED: &str = "mach-time-constraint-not-applied";

/// What a user could do about a refusal.
///
/// macOS grants this policy to ordinary processes, so a refusal is not a
/// missing privilege the user can add — saying "run as root" here would send
/// someone down a road that does not lead anywhere.
const REMEDY: &str = "nothing to grant: macOS gives this policy to unprivileged processes, so a \
                      refusal points at a sandbox or a device-management profile, not at a \
                      permission you can add";

// `(period, constraint)` in mach ticks, as `promote` declared them on the
// thread it promoted, read back by `verify`. `None` means "nothing was
// promoted on this thread".
//
// Thread-local because every mach thread-policy call is self-directed: the
// contract and the thread it describes belong to each other by construction,
// so no synchronisation is needed or possible. Without it a thread that never
// asked for a contract would report one as revoked, and `--no-realtime` would
// warn once every stats window forever.
//
// The VALUES, not just a bool: the kernel writes the policy actually in force
// into the struct `verify` hands it, so comparing that against what was
// declared costs nothing and catches a contract that was REPLACED rather than
// defaulted, which the `get_default` flag alone reports as intact.
//
// `computation` is DELIBERATELY not stored and not compared. XNU normalises it
// upward to a 500 µs floor: on this Mac14,2 a declared 250 µs (6000 ticks)
// reads back as 500 µs (12000 ticks), while 600 µs and 1200 µs come back
// verbatim — measured across seven period/computation pairs, all of which
// echoed `period` and `constraint` unchanged. Comparing all three, which is the
// obvious thing to write, therefore reports EVERY 400 Hz session as revoked on
// its first stats window: a permanent false alarm on the exact metric this
// release exists to make believable. `the_kernel_echoes_the_period_and_constraint_it_was_given`
// pins the two fields this comparison does rely on, so a macOS that starts
// rewriting them fails a test here rather than a user's daemon in the field.
std::thread_local! {
    static DECLARED: std::cell::Cell<Option<(u32, u32)>> = const { std::cell::Cell::new(None) };
}

/// Convert a wall duration into mach absolute-time TICKS.
///
/// NOT nanoseconds. The conversion is `ticks = nanoseconds * denom / numer`
/// with numer/denom read from `mach_timebase_info()` AT RUNTIME.
///
/// Hardcoding either side is a real, silent, platform-dependent bug. Intel
/// Macs report 1/1, so ticks and nanoseconds are numerically equal there and
/// code written on Intel looks correct everywhere. This Mac14,2 (Apple M2)
/// reports numer/denom = 125/3 — a 24 MHz tick — so 2.5 ms is 60 000 ticks,
/// and the Intel-shaped literal 2_500_000 declares a 104 ms period instead: a
/// factor of 41.7 wrong, in the direction that tells the kernel this thread is
/// very nearly idle.
pub(crate) fn to_mach_ticks(d: Duration) -> u32 {
    let (numer, denom) = timebase();
    let ticks = d.as_nanos().saturating_mul(u128::from(denom)) / u128::from(numer);
    // Saturating rather than wrapping. A duration that overflows u32 ticks is
    // a programming error either way, but clamping declares an implausibly
    // LONG period, which the kernel merely ignores, while wrapping would
    // declare a short one, which it enforces.
    u32::try_from(ticks).unwrap_or(u32::MAX)
}

/// `mach_timebase_info()`, as `(numer, denom)`, both guaranteed non-zero.
///
/// Read on every call rather than cached: it is a vDSO-backed read of a
/// constant the kernel published at boot, so a `OnceLock` would buy nothing
/// and add a failure mode.
fn timebase() -> (u32, u32) {
    // SAFETY: `mach_timebase_info` writes two u32s into the out-parameter and
    // returns KERN_SUCCESS on every supported OS version. On the impossible
    // failure path the struct keeps the identity ratio initialised below, so
    // the fallback is the Intel behaviour rather than a division by zero.
    let info = unsafe {
        let mut info = MachTimebaseInfo { numer: 1, denom: 1 };
        if mach_timebase_info(&mut info) != KERN_SUCCESS {
            return (1, 1);
        }
        info
    };
    if info.numer == 0 || info.denom == 0 {
        return (1, 1);
    }
    (info.numer, info.denom)
}

/// Run `f` with a send right to the calling thread, releasing it afterwards.
///
/// `mach_thread_self()` mints a FRESH send right on every call. `verify` runs
/// once per stats window for the life of the process, so dropping the
/// deallocate would be a port leak of roughly 17 000 rights per day.
fn with_thread_port<T>(f: impl FnOnce(thread_t) -> T) -> T {
    /// Releases the send right on the way out, INCLUDING on an unwind.
    ///
    /// A plain call after `f` would leak the right if `f` ever panicked —
    /// which is the exact leak this function exists to prevent. Both closures
    /// passed today are single FFI calls that cannot panic, so this is a guard
    /// against a future edit rather than a live bug.
    struct ThreadPort(thread_t);
    impl Drop for ThreadPort {
        fn drop(&mut self) {
            // SAFETY: the right was minted by `mach_thread_self` for this
            // thread and is released exactly once, here. `mach_task_self_` is
            // a process-lifetime global the kernel initialises before `main`.
            let kr = unsafe { mach_port_deallocate(mach_task_self_, self.0) };
            // Not swallowed. A failure here means the right leaks, and at one
            // `verify` per stats window that is ~17 000 rights a day.
            debug_assert_eq!(
                kr, KERN_SUCCESS,
                "mach_port_deallocate refused a send right this thread minted; \
                 the port is leaking once per stats window"
            );
        }
    }

    // SAFETY: `mach_thread_self` always succeeds for the calling thread and
    // mints a FRESH send right on every call.
    let port = ThreadPort(unsafe { mach_thread_self() });
    f(port.0)
}

/// The contract `promote` declares, derived in one place so a test can observe
/// the same values the kernel is handed.
///
/// Extracted from `promote` because a test that re-derives the clamp proves
/// only that `Duration::clamp` works: deleting the clamp from `promote` left
/// both ordering tests green. This is reachable, not hypothetical —
/// `validate_tick_rate` accepts any exact divisor of 1 000 000, so
/// `--tick-rate 2000` gives a 500 µs period against the 1000 µs [`CONSTRAINT`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Contract {
    period: Duration,
    computation: Duration,
    constraint: Duration,
}

impl Contract {
    /// The triple as mach absolute-time ticks, in the order the kernel reads
    /// them out of [`ThreadTimeConstraintPolicy`].
    fn ticks(&self) -> (u32, u32, u32) {
        (
            to_mach_ticks(self.period),
            to_mach_ticks(self.computation),
            to_mach_ticks(self.constraint),
        )
    }
}

/// Build a coherent `computation <= constraint <= period` triple.
///
/// A computation budget at or above the period declares a CPU hog, which this
/// thread is not (measured worst-case work ~95 µs against 2500 µs). Clamping is
/// not a silent correction of user input — both values are constants at the
/// only call site — it is a guard so a future `--tick-rate` change cannot
/// construct an incoherent contract, which the kernel accepts and then quietly
/// ignores.
fn contract(period: Duration, computation: Duration) -> Contract {
    let computation = computation.min(period);
    Contract {
        period,
        computation,
        constraint: CONSTRAINT.clamp(computation, period),
    }
}

pub(crate) fn promote(period: Duration, computation: Duration) -> Promotion {
    // Ask for the QoS class FIRST, because with the contract in force the
    // request is REFUSED — measured: `pthread_set_qos_class_self_np` returns 1
    // and the class stays `QOS_CLASS_UNSPECIFIED`. Ordering does not preserve
    // the tier (the contract clears it either way, see the module header); it
    // decides whether the thread is left at USER_INTERACTIVE or at the default
    // tier on the path where `thread_policy_set` FAILS, and USER_INTERACTIVE
    // is the better answer there since it is then the only mitigation in
    // force.
    //
    // SAFETY: takes its argument by value, acts on the calling thread, has no
    // out-parameters. The class is `libc`'s own `qos_class_t` variant rather
    // than a hand-written 0x21 transmuted into the enum: transmuting an integer
    // that is not a declared discriminant is UB, and the literal would have
    // become exactly that the first time anyone edited it.
    let qos_rc = unsafe {
        libc::pthread_set_qos_class_self_np(libc::qos_class_t::QOS_CLASS_USER_INTERACTIVE, 0)
    };
    // Read BEFORE anything else runs. `last_os_error` is only meaningful
    // immediately after the failing call.
    let qos_errno = if qos_rc == 0 {
        None
    } else {
        std::io::Error::last_os_error().raw_os_error()
    };

    let declared = contract(period, computation);
    let (period_ticks, computation_ticks, constraint_ticks) = declared.ticks();

    let mut policy = ThreadTimeConstraintPolicy {
        period: period_ticks,
        computation: computation_ticks,
        constraint: constraint_ticks,
        // Preemptible. A non-preemptible declaration on a 4-P-core part is a
        // request to stall the rest of the system, and this loop has nothing
        // that cannot tolerate preemption between sub-steps.
        preemptible: 1,
    };

    // SAFETY: `thread_policy_set` reads exactly
    // `THREAD_TIME_CONSTRAINT_POLICY_COUNT` `integer_t`s through the pointer,
    // and that count is derived from the size of the struct being passed.
    let kr = with_thread_port(|thread| unsafe {
        thread_policy_set(
            thread,
            THREAD_TIME_CONSTRAINT_POLICY,
            std::ptr::addr_of_mut!(policy).cast::<integer_t>(),
            THREAD_TIME_CONSTRAINT_POLICY_COUNT,
        )
    });

    if kr != KERN_SUCCESS {
        DECLARED.with(|d| d.set(None));
        return Promotion::Unavailable {
            mechanism: MECHANISM,
            // A `kern_return_t`, not an errno. It shares the field because the
            // only thing anyone does with either is quote it, and the
            // mechanism name beside it says which namespace to look it up in.
            errno: kr,
            remedy: REMEDY,
        };
    }

    DECLARED.with(|d| d.set(Some((period_ticks, constraint_ticks))));

    // Read the contract back IMMEDIATELY, not just on the 5 s window.
    //
    // KERN_SUCCESS says the kernel accepted the request; it does not say the
    // policy took. A sandbox or a device-management profile can accept and
    // ignore it, and without this check the daemon would log
    // `granted:mach-time-constraint`, the A/B would compare two identical
    // arms, and the first honest signal would arrive five seconds later as a
    // REVOCATION — a diagnosis that is wrong for a contract that never
    // applied. This is the read that separates those two.
    if verify() != Verification::Intact {
        DECLARED.with(|d| d.set(None));
        return Promotion::Unavailable {
            mechanism: MECHANISM_NOT_APPLIED,
            // KERN_SUCCESS. The set call did not fail — that is the whole
            // point of this branch, and the mechanism name carries the reason
            // the errno cannot.
            errno: kr,
            remedy: REMEDY,
        };
    }

    // Decide the mechanism from what the thread ACTUALLY HOLDS, not from the
    // return code of the call that asked for it.
    //
    // This is the same standard the contract half is already held to twelve
    // lines above ("KERN_SUCCESS says the kernel accepted the request; it does
    // not say the policy took"). The QoS half was exempted from it, and
    // branching on `qos_rc` made `MECHANISM_NO_QOS` unreachable: the call
    // succeeds, the contract then clears the tier, and every macOS session
    // published `granted:mach-time-constraint` on a thread sitting at
    // `QOS_CLASS_UNSPECIFIED`.
    let qos_class = current_qos_class();
    // `matches!` rather than `==`: libc's `qos_class_t` derives Copy and Debug
    // and nothing else, so it has no `PartialEq`.
    let qos_held = matches!(qos_class, libc::qos_class_t::QOS_CLASS_USER_INTERACTIVE);

    Promotion::Granted {
        // The QoS half is named in the MECHANISM, not only in the prose, so a
        // partial grant is distinguishable from `token()` alone.
        mechanism: if qos_held {
            MECHANISM
        } else {
            MECHANISM_NO_QOS
        },
        detail: format!(
            "period={} ticks ({} us), computation={} ticks ({} us), constraint={} ticks ({} us), \
             preemptible; QOS_CLASS_USER_INTERACTIVE {}",
            policy.period,
            declared.period.as_micros(),
            policy.computation,
            declared.computation.as_micros(),
            policy.constraint,
            declared.constraint.as_micros(),
            match (qos_held, qos_errno) {
                (true, _) => "in force".to_string(),
                // The ordinary outcome on this OS: the request succeeded and
                // the time-constraint contract then superseded it. Reported as
                // what it is rather than as a refusal, so a reader does not go
                // hunting a broken FFI binding.
                (false, None) => format!(
                    "requested and granted, then SUPERSEDED by the contract (class now {:#04x})",
                    qos_class as u32
                ),
                // The call itself failed. Named with its errno rather than
                // swallowed.
                (false, Some(errno)) => format!("REFUSED (errno {errno})"),
            }
        ),
    }
}

/// This thread's QoS class, as the kernel reports it.
///
/// `QOS_CLASS_UNSPECIFIED` on any failure — the only caller is deciding
/// whether the tier is HELD, and "could not read it" is not evidence that it
/// is.
fn current_qos_class() -> libc::qos_class_t {
    let mut class = libc::qos_class_t::QOS_CLASS_UNSPECIFIED;
    let mut relative_priority: libc::c_int = 0;
    // SAFETY: `pthread_get_qos_class_np` writes one `qos_class_t` and one
    // `int` through its out-parameters and returns 0 on success. `qos_class_t`
    // is a closed, versioned enum in `<pthread/qos.h>` and the kernel writes
    // only its declared members, so the typed out-parameter cannot be handed a
    // value outside the enum.
    let rc = unsafe {
        libc::pthread_get_qos_class_np(libc::pthread_self(), &mut class, &mut relative_priority)
    };
    if rc == 0 {
        class
    } else {
        libc::qos_class_t::QOS_CLASS_UNSPECIFIED
    }
}

pub(crate) fn verify() -> Verification {
    let Some(declared) = DECLARED.with(|d| d.get()) else {
        return Verification::NotApplicable;
    };

    let mut policy = ThreadTimeConstraintPolicy {
        period: 0,
        computation: 0,
        constraint: 0,
        preemptible: 0,
    };
    let mut count = THREAD_TIME_CONSTRAINT_POLICY_COUNT;
    // In-out. FALSE going in means "give me the policy actually in force"; the
    // kernel sets it TRUE to say the thread is on the DEFAULT policy — i.e.
    // the contract set earlier is gone. That flip is the only notice macOS
    // gives when it demotes a thread for chronic overrun, and a silently
    // revoked real-time contract is exactly this incident's shape a second
    // time.
    let mut get_default: boolean_t = 0;

    // SAFETY: the read direction of the same contract as `promote`. `count` is
    // in-out and starts at the struct's own size.
    let kr = with_thread_port(|thread| unsafe {
        thread_policy_get(
            thread,
            THREAD_TIME_CONSTRAINT_POLICY,
            std::ptr::addr_of_mut!(policy).cast::<integer_t>(),
            &mut count,
            &mut get_default,
        )
    });

    if kr != KERN_SUCCESS {
        // A failed read is not evidence of a revocation, and reporting one
        // here would be a false alarm on the very metric this change exists to
        // make believable.
        return Verification::NotApplicable;
    }

    if get_default != 0 {
        return Verification::Revoked;
    }

    // The kernel has already written the policy actually in force into
    // `policy`; throwing it away and deciding on `get_default` alone would
    // report a contract that was REPLACED — a different period or computation
    // set by any later call on this thread — as intact.
    // The kernel has already written the policy actually in force into
    // `policy`; deciding on `get_default` alone and discarding it would report
    // a contract that was REPLACED — a different period set by any later call
    // on this thread — as intact.
    //
    // Period and constraint only. See `DECLARED` for why `computation` is
    // excluded: XNU raises it to a 500 µs floor, so comparing it would report
    // every 400 Hz session as revoked.
    if (policy.period, policy.constraint) != declared {
        return Verification::Revoked;
    }

    Verification::Intact
}

pub(crate) fn revert() {
    // Nothing to release. Both the contract and the QoS class die with the
    // thread, and neither holds a kernel resource that outlives it.
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 41.7x bug, pinned.
    #[test]
    fn mach_ticks_conversion_uses_the_runtime_timebase() {
        let (numer, denom) = timebase();
        let nanos: u128 = 2_500_000;

        let expected = u32::try_from(nanos * u128::from(denom) / u128::from(numer))
            .expect("2.5 ms in mach ticks fits in u32 on any real timebase");
        assert_eq!(
            to_mach_ticks(Duration::from_micros(2500)),
            expected,
            "the conversion must go through mach_timebase_info() at runtime \
             (this host reports numer={numer} denom={denom})"
        );

        // The whole point of the runtime read. On a 1:1 host (every Intel Mac)
        // ticks and nanoseconds are numerically equal, so this assertion could
        // not tell a correct conversion from a hardcoded one and is made only
        // where it can discriminate. On this M2 (125/3) it catches the literal
        // 2_500_000, which would declare a 104 ms period.
        if numer != denom {
            assert_ne!(
                to_mach_ticks(Duration::from_micros(2500)),
                2_500_000,
                "2.5 ms is not 2_500_000 mach ticks on a host whose timebase is not 1:1 \
                 (numer={numer} denom={denom}); that literal is the 41.7x Intel-shaped error"
            );
        }
    }

    #[test]
    fn a_zero_duration_converts_to_zero_ticks() {
        assert_eq!(to_mach_ticks(Duration::ZERO), 0);
    }

    #[test]
    fn an_absurd_duration_saturates_rather_than_wrapping() {
        // Wrapping would declare a SHORT period, which the kernel enforces.
        // Saturating declares a long one, which it ignores.
        assert_eq!(to_mach_ticks(Duration::from_secs(u64::MAX)), u32::MAX);
    }

    /// The count must be derived from the struct, never written as a literal.
    #[test]
    fn the_policy_count_matches_the_struct_the_kernel_is_handed() {
        assert_eq!(
            THREAD_TIME_CONSTRAINT_POLICY_COUNT, 4,
            "thread_time_constraint_policy is four 32-bit fields; a mismatch here is a \
             buffer-length lie told to the kernel"
        );
    }

    /// macOS grants `THREAD_TIME_CONSTRAINT_POLICY` to an ordinary process —
    /// no entitlement, no root. If that ever stops being true, the daemon must
    /// find out from a test rather than from a flight.
    #[test]
    fn macos_grants_a_time_constraint_contract_to_an_ordinary_process() {
        let (promotion, verification) = std::thread::spawn(|| {
            let p = promote(Duration::from_micros(2500), Duration::from_micros(250));
            (p, verify())
        })
        .join()
        .expect("promotion must not panic");

        assert!(
            promotion.is_granted(),
            "macOS refused a time-constraint contract to an unprivileged thread: {}",
            promotion.describe()
        );
        assert_eq!(
            verification,
            Verification::Intact,
            "a contract read back immediately after being set must read as intact"
        );
    }

    /// The declared contract must be internally coherent, whatever the
    /// timebase: `computation <= constraint <= period`. An incoherent triple
    /// is accepted by the kernel and then quietly ignored.
    ///
    /// Asserts on [`contract`] — the function `promote` actually calls — not on
    /// a clamp re-derived in the test body. The earlier version of this test
    /// stayed green with the clamp deleted from `promote` entirely, which made
    /// it decoration rather than a guard.
    #[test]
    fn the_declared_contract_is_ordered_computation_constraint_period() {
        let c = contract(Duration::from_micros(2500), Duration::from_micros(250));
        assert!(
            c.computation <= c.constraint && c.constraint <= c.period,
            "incoherent contract {c:?}"
        );
        let (period, computation, constraint) = c.ticks();
        assert!(
            computation <= constraint && constraint <= period,
            "the tick triple the kernel is handed must be ordered too, got \
             computation={computation} constraint={constraint} period={period}"
        );
    }

    /// A tick rate so fast that the default constraint exceeds the period must
    /// still produce an ordered triple. `--tick-rate 2000` reaches this:
    /// `validate_tick_rate` accepts any exact divisor of 1 000 000, and a
    /// 500 µs period against the 1000 µs [`CONSTRAINT`] is the incoherent
    /// triple the clamp exists to prevent.
    #[test]
    fn a_period_shorter_than_the_default_constraint_still_orders_correctly() {
        let c = contract(Duration::from_micros(500), Duration::from_micros(250));
        assert!(
            c.computation <= c.constraint && c.constraint <= c.period,
            "incoherent contract {c:?}"
        );
        assert_eq!(
            c.constraint,
            Duration::from_micros(500),
            "the constraint must be clamped DOWN to the period, not left at the \
             1000 us default the kernel would discard"
        );
        let (period, computation, constraint) = c.ticks();
        assert!(computation <= constraint && constraint <= period);
    }

    /// A contract the kernel accepted and did not apply must not read as
    /// granted, and the immediate read-back in `promote` is what separates the
    /// two. On a host that DOES apply it (every unsandboxed Mac) this pins the
    /// pairing: granted implies the read-back agreed.
    #[test]
    fn a_granted_promotion_has_already_been_read_back() {
        let (promotion, verification) = std::thread::spawn(|| {
            let p = promote(Duration::from_micros(2500), Duration::from_micros(250));
            (p, verify())
        })
        .join()
        .expect("promotion must not panic");

        if promotion.is_granted() {
            assert_eq!(
                verification,
                Verification::Intact,
                "`promote` returned Granted, so it had already read the contract back as \
                 intact; a later read on the same thread cannot disagree"
            );
        } else {
            assert_eq!(
                verification,
                Verification::NotApplicable,
                "a refused promotion must leave nothing behind for `verify` to call revoked"
            );
        }
    }

    /// The premise `verify`'s value comparison rests on, measured rather than
    /// assumed.
    ///
    /// `thread_policy_get` does NOT echo the struct back verbatim. On this
    /// Mac14,2 (Apple M2, timebase 125/3) XNU normalises `computation` upward
    /// to a 500 µs floor — a declared 250 µs comes back as 500 µs — while
    /// `period` and `constraint` are returned exactly as given, across every
    /// period from 500 µs to 10 ms that was tried.
    ///
    /// So `verify` compares period and constraint and ignores computation.
    /// Comparing all three, which is the obvious thing to write, makes EVERY
    /// 400 Hz session report a revoked contract on its first stats window.
    /// This test is what stops a future macOS changing the rule silently: if it
    /// ever starts rewriting period or constraint, this fails on a developer's
    /// machine instead of putting a false "real-time contract WITHDRAWN"
    /// warning in front of a user.
    #[test]
    fn the_kernel_echoes_the_period_and_constraint_it_was_given() {
        let readbacks = std::thread::spawn(|| {
            let mut out = Vec::new();
            for (period_us, computation_us) in
                [(2500u64, 250u64), (2500, 600), (500, 250), (10000, 250)]
            {
                let declared = contract(
                    Duration::from_micros(period_us),
                    Duration::from_micros(computation_us),
                );
                let (period, computation, constraint) = declared.ticks();
                let mut policy = ThreadTimeConstraintPolicy {
                    period,
                    computation,
                    constraint,
                    preemptible: 1,
                };
                // SAFETY: same contract as `promote` sets, same derived count.
                let set_kr = with_thread_port(|thread| unsafe {
                    thread_policy_set(
                        thread,
                        THREAD_TIME_CONSTRAINT_POLICY,
                        std::ptr::addr_of_mut!(policy).cast::<integer_t>(),
                        THREAD_TIME_CONSTRAINT_POLICY_COUNT,
                    )
                });
                let mut back = ThreadTimeConstraintPolicy {
                    period: 0,
                    computation: 0,
                    constraint: 0,
                    preemptible: 0,
                };
                let mut count = THREAD_TIME_CONSTRAINT_POLICY_COUNT;
                let mut get_default: boolean_t = 0;
                // SAFETY: the read direction of the same contract.
                let get_kr = with_thread_port(|thread| unsafe {
                    thread_policy_get(
                        thread,
                        THREAD_TIME_CONSTRAINT_POLICY,
                        std::ptr::addr_of_mut!(back).cast::<integer_t>(),
                        &mut count,
                        &mut get_default,
                    )
                });
                out.push((
                    period_us,
                    computation_us,
                    set_kr,
                    get_kr,
                    get_default,
                    (period, constraint),
                    (back.period, back.computation, back.constraint),
                ));
            }
            out
        })
        .join()
        .expect("the probe thread must not panic");

        for (period_us, computation_us, set_kr, get_kr, get_default, declared, back) in readbacks {
            assert_eq!(
                set_kr, KERN_SUCCESS,
                "set failed for {period_us}/{computation_us} us"
            );
            assert_eq!(
                get_kr, KERN_SUCCESS,
                "get failed for {period_us}/{computation_us} us"
            );
            assert_eq!(
                get_default, 0,
                "a contract set moments ago must not read as the default policy"
            );
            assert_eq!(
                (back.0, back.2),
                declared,
                "XNU rewrote the period or the constraint for {period_us} us / {computation_us} us \
                 (declared {declared:?}, read back {back:?}). `verify` compares those two fields, \
                 so this now produces a permanent false 'contract WITHDRAWN' warning — narrow the \
                 comparison further rather than shipping that"
            );
            assert!(
                back.1
                    >= to_mach_ticks(
                        Duration::from_micros(computation_us).min(Duration::from_micros(period_us))
                    ),
                "the kernel returned a SMALLER computation budget than was declared for \
                 {period_us} us / {computation_us} us: {back:?}"
            );
        }
    }

    /// The measured interaction, pinned: a granted contract CLEARS the QoS
    /// tier, and asking for the tier afterwards is refused.
    ///
    /// This is the premise the mechanism selection in [`promote`] now rests on,
    /// and it was not checked before — the code branched on
    /// `pthread_set_qos_class_self_np`'s return code, which always succeeds, so
    /// [`MECHANISM_NO_QOS`] was unreachable and every macOS session claimed a
    /// QoS tier the thread did not hold. If a future XNU stops superseding the
    /// tier this fails on a developer's machine rather than quietly making the
    /// token pessimistic in the field.
    #[test]
    fn a_time_constraint_contract_supersedes_the_qos_tier() {
        let (after_set, after_contract, second_set_rc, after_second_set) =
            std::thread::spawn(|| {
                // SAFETY: takes its argument by value and acts on the calling
                // thread; no out-parameters.
                let first = unsafe {
                    libc::pthread_set_qos_class_self_np(
                        libc::qos_class_t::QOS_CLASS_USER_INTERACTIVE,
                        0,
                    )
                };
                assert_eq!(
                    first, 0,
                    "macOS must grant USER_INTERACTIVE to a plain thread"
                );
                let after_set = current_qos_class() as u32;

                let granted = promote(Duration::from_micros(2500), Duration::from_micros(250));
                assert!(granted.is_granted(), "{}", granted.describe());
                let after_contract = current_qos_class() as u32;

                // SAFETY: as above.
                let second_set_rc = unsafe {
                    libc::pthread_set_qos_class_self_np(
                        libc::qos_class_t::QOS_CLASS_USER_INTERACTIVE,
                        0,
                    )
                };
                let after_second_set = current_qos_class() as u32;
                (after_set, after_contract, second_set_rc, after_second_set)
            })
            .join()
            .expect("the probe thread must not panic");

        assert_eq!(
            after_set, 0x21,
            "the QoS request on its own does raise the tier to USER_INTERACTIVE"
        );
        assert_ne!(
            after_contract, 0x21,
            "and the time-constraint contract then takes it away — measured 0x00 \
             (QOS_CLASS_UNSPECIFIED) on a Mac14,2 running macOS 27.0. If this ever \
             passes both halves at once, `promote` may report MECHANISM again and \
             lib.rs's Selective-Forced-Idle paragraph can go back to being about \
             the tier"
        );
        assert_ne!(
            second_set_rc, 0,
            "and re-asking with the contract in force is refused, so no ordering \
             of the two calls leaves both in force"
        );
        assert_ne!(after_second_set, 0x21);
    }

    /// A macOS grant reports the QoS half from what the thread HOLDS.
    ///
    /// Given the test above, that means the ordinary outcome on this OS is the
    /// `-noqos` token. The point is not which token: it is that the token is
    /// decided by a read-back rather than by a return code, so it cannot claim
    /// a mitigation that is not in force.
    #[test]
    fn the_published_token_names_the_qos_half_that_is_actually_in_force() {
        let (promotion, qos_class) = std::thread::spawn(|| {
            let p = promote(Duration::from_micros(2500), Duration::from_micros(250));
            (p, current_qos_class() as u32)
        })
        .join()
        .expect("promotion must not panic");

        let Promotion::Granted { mechanism, .. } = &promotion else {
            // A sandboxed host refuses the contract; nothing to check here.
            return;
        };
        let expected = if qos_class == 0x21 {
            MECHANISM
        } else {
            MECHANISM_NO_QOS
        };
        assert_eq!(
            *mechanism, expected,
            "the thread's QoS class reads {qos_class:#04x}, so the mechanism must be \
             {expected:?}. Selecting it from pthread_set_qos_class_self_np's return \
             code instead published the full-grant token on every macOS session"
        );
    }

    /// A partial grant — contract yes, QoS tier no — must be distinguishable
    /// from a full one in [`Promotion::token`], which is the only field the
    /// A/B and support reports read.
    #[test]
    fn a_qos_refusal_has_its_own_token() {
        let full = Promotion::Granted {
            mechanism: MECHANISM,
            detail: String::new(),
        }
        .token();
        let partial = Promotion::Granted {
            mechanism: MECHANISM_NO_QOS,
            detail: String::new(),
        }
        .token();
        let not_applied = Promotion::Unavailable {
            mechanism: MECHANISM_NOT_APPLIED,
            errno: 0,
            remedy: REMEDY,
        }
        .token();

        assert_ne!(
            full, partial,
            "the QoS tier is the half that bears on Selective Forced Idle; a host that \
             did not get it must not report the same token as one that did"
        );
        assert_ne!(full, not_applied);
        for token in [&full, &partial, &not_applied] {
            assert_eq!(*token, token.to_lowercase());
            assert!(!token.contains(' '));
        }
    }
}
