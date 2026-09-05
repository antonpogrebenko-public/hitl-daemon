//! `--selftest-timebase` is the one command to hand a user reporting a
//! simulation problem, so its output has to stay machine-readable and has to
//! keep naming what it measured.
//!
//! SHAPE ONLY, NEVER A THRESHOLD. This runs on whatever CI machine picks it up,
//! under whatever else that machine is doing, and a test that graded the
//! achieved rate there would fail for reasons that have nothing to do with the
//! daemon. A flaky check on the pacing metric is worse than no check: it trains
//! people to ignore exactly the signal this whole change exists to make
//! believable. The exit code is deliberately not asserted for the same reason —
//! a genuinely slow CI box SHOULD exit non-zero.
//!
//! Lives in the root package's `tests/` and not in `crates/simulation/tests/`
//! because `CARGO_BIN_EXE_<name>` is only set for the package that declares the
//! binary, and `--selftest-timebase` is a flag on the daemon binary.

use std::process::Command;

/// Seconds of measurement. Two full 1 s health windows plus margin, which is
/// enough for every key below to be populated from a real measurement rather
/// than from the inconclusive fallback.
const SELFTEST_SECS: &str = "3";

/// Every key the report promises. Anything parsing this output — a support
/// script, a `jq` one-liner in a bug report — breaks silently if one
/// disappears, and the daemon would not notice.
const REQUIRED_KEYS: &[&str] = &[
    "achieved_rate_hz",
    "target_rate_hz",
    "steps_per_sec",
    // NOT `drift_ms_per_s`. That key divided an instantaneous `wall − sim`
    // offset by the run length and called the result a rate, so two identical
    // hosts reported different figures purely as a function of the N passed on
    // the command line.
    "drift_us_at_end",
    "min_realised",
    "wake_late_p99_us",
    "wake_late_max_us",
    "wake_late_avg_us",
    "deadline_overrun_us",
    "discarded_ms",
    "gap_count",
    "health",
    "last_window_health",
    "rt_outcome",
];

#[test]
fn selftest_timebase_emits_wellformed_json_and_names_the_rt_outcome() {
    let output = Command::new(env!("CARGO_BIN_EXE_hitl-daemon"))
        .args(["--selftest-timebase", SELFTEST_SECS])
        .output()
        .expect("the daemon binary must be runnable");

    let stdout = String::from_utf8(output.stdout).expect("the report must be UTF-8");

    // Exactly one non-empty line on stdout. The whole point of routing
    // diagnostics to stderr is that this pipes into `jq`; a stray `println!`
    // anywhere in the startup path breaks that silently, and only a test that
    // counts the lines catches it.
    let lines: Vec<&str> = stdout.lines().filter(|l| !l.trim().is_empty()).collect();
    assert_eq!(
        lines.len(),
        1,
        "--selftest-timebase must put exactly one line on stdout so it pipes into `jq`; \
         got {} lines:\n{stdout}",
        lines.len()
    );

    let report: serde_json::Value = serde_json::from_str(lines[0])
        .unwrap_or_else(|e| panic!("the report must parse as JSON ({e}): {}", lines[0]));

    for key in REQUIRED_KEYS {
        assert!(
            report.get(key).is_some(),
            "the report must carry `{key}`: {report}"
        );
    }

    // The scheduling outcome has to be NAMED, whatever it is. "granted:...",
    // "unavailable:<mechanism>:<errno>", "unsupported" and "disabled" are all
    // legitimate answers — a Linux CI container can only ever produce the
    // second — but an absent or empty one means nobody asked, and the A/B this
    // key exists for cannot be run.
    let rt_outcome = report["rt_outcome"]
        .as_str()
        .expect("`rt_outcome` must be a string");
    assert!(
        !rt_outcome.is_empty(),
        "`rt_outcome` must name what the host did with the promotion request"
    );
    assert_eq!(
        rt_outcome,
        rt_outcome.to_lowercase(),
        "`rt_outcome` is a machine-readable token: {rt_outcome}"
    );

    // The target is the one figure in the report that does not depend on the
    // host, so it is the one figure that can be asserted without inviting a
    // flake.
    assert_eq!(
        report["target_rate_hz"], 400,
        "the default tick rate belongs in the report so a reader knows what the achieved \
         rate is being compared against"
    );
}

/// `--no-realtime` is the other arm of the A/B, and it has to be reachable
/// from the same command or the comparison cannot be made.
#[test]
fn selftest_timebase_honours_no_realtime() {
    let output = Command::new(env!("CARGO_BIN_EXE_hitl-daemon"))
        .args(["--selftest-timebase", SELFTEST_SECS, "--no-realtime"])
        .output()
        .expect("the daemon binary must be runnable");

    let stdout = String::from_utf8(output.stdout).expect("the report must be UTF-8");
    let line = stdout
        .lines()
        .find(|l| !l.trim().is_empty())
        .unwrap_or_else(|| panic!("--no-realtime produced no report:\n{stdout}"));
    let report: serde_json::Value =
        serde_json::from_str(line).expect("the report must parse as JSON");

    assert_eq!(
        report["rt_outcome"], "disabled",
        "--no-realtime must decline to ask, and must say so in the same field the granted \
         case uses — otherwise the two arms of the A/B are not distinguishable from the \
         output alone"
    );
}
