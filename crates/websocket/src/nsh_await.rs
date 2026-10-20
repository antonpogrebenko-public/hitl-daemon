//! Awaited NSH commands for the daemon's own use (EKF2 restart).
//!
//! The NSH task reports `success: true` whenever the shell prompt returns, even
//! if the command printed an error (design D4). So a reply is judged per
//! command from its text, not from the `success` flag alone.

use crate::handler::ValidatedNshCommand;
use crate::protocol::NshResponse;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;
use tokio::sync::{broadcast, mpsc};
use tokio::time::{timeout_at, Instant};

/// First request id the daemon uses for its own NSH commands. Above every
/// fixed internal id (`preflight.rs` uses `0xFFFF_FF04` for the board reboot).
pub(crate) const INTERNAL_REQUEST_ID_FIRST: u32 = 0xFFFF_FF10;

/// The NSH task's own per-command deadline. It sits inside the 3 s reply window
/// so a command queued behind a pilot's still fits; if the window expires anyway
/// the restart error carries the timeout and the pilot can Retry.
pub(crate) const NSH_COMMAND_TIMEOUT_MS: u16 = 2000;

/// The two channels that reach the NSH task: commands in, replies out.
#[derive(Clone)]
pub struct NshLink {
    pub commands: mpsc::Sender<ValidatedNshCommand>,
    pub replies: broadcast::Sender<NshResponse>,
}

/// Hands out request ids for internal NSH commands, wrapping inside the
/// internal range.
pub(crate) struct InternalRequestIds {
    next: AtomicU32,
}

impl InternalRequestIds {
    pub(crate) fn new() -> Self {
        Self {
            next: AtomicU32::new(INTERNAL_REQUEST_ID_FIRST),
        }
    }

    #[cfg(test)]
    pub(crate) fn starting_at(first: u32) -> Self {
        Self {
            next: AtomicU32::new(first),
        }
    }

    /// Returns the current id and advances, wrapping from `u32::MAX` back to
    /// `INTERNAL_REQUEST_ID_FIRST`.
    pub(crate) fn next(&self) -> u32 {
        let advance = |id: u32| {
            Some(if id == u32::MAX {
                INTERNAL_REQUEST_ID_FIRST
            } else {
                id + 1
            })
        };
        match self
            .next
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, advance)
        {
            Ok(id) | Err(id) => id,
        }
    }
}

/// `1 s` for a whole number of seconds, `100 ms` otherwise, so a timeout
/// shorter than a second never reads `0 s`.
pub(crate) fn whole_seconds_or_ms(d: Duration) -> String {
    if d.subsec_nanos() == 0 {
        format!("{} s", d.as_secs())
    } else {
        format!("{} ms", d.as_millis())
    }
}

/// Sends one NSH command and waits for its complete reply.
///
/// Subscribes before sending so the reply cannot be missed. Replies for other
/// request ids, and lagged broadcast gaps, are skipped.
pub(crate) async fn run_nsh(
    link: &NshLink,
    request_id: u32,
    command: &str,
    reply_timeout: Duration,
) -> Result<String, String> {
    let mut replies = link.replies.subscribe();
    let deadline = Instant::now() + reply_timeout;
    let request = ValidatedNshCommand {
        request_id,
        command: command.to_string(),
        timeout_ms: NSH_COMMAND_TIMEOUT_MS,
        client_id: 0,
    };
    link.commands.try_send(request).map_err(|e| match e {
        mpsc::error::TrySendError::Full(_) => "NSH queue full".to_string(),
        mpsc::error::TrySendError::Closed(_) => "NSH task not running".to_string(),
    })?;
    loop {
        match timeout_at(deadline, replies.recv()).await {
            Err(_) => {
                return Err(format!(
                    "no reply within {}",
                    whole_seconds_or_ms(reply_timeout)
                ));
            }
            Ok(Err(broadcast::error::RecvError::Lagged(_))) => continue,
            Ok(Err(broadcast::error::RecvError::Closed)) => {
                return Err("NSH reply channel closed".to_string());
            }
            Ok(Ok(reply)) if reply.request_id == request_id && reply.complete => {
                return if reply.success {
                    Ok(reply.output)
                } else {
                    Err(reply.output.trim().to_string())
                };
            }
            Ok(Ok(_)) => {}
        }
    }
}

/// `ekf2 stop` passes on any reply: "not running" is the state we want anyway.
pub(crate) fn judge_ekf2_stop(_output: &str) -> Result<(), String> {
    Ok(())
}

/// `ekf2 start` fails when a line says `error` or `failed`, in any case. The
/// error is the first such line, trimmed.
pub(crate) fn judge_ekf2_start(output: &str) -> Result<(), String> {
    let failing = output.lines().map(str::trim).find(|line| {
        let lower = line.to_lowercase();
        lower.contains("error") || lower.contains("failed")
    });
    match failing {
        Some(line) => Err(line.to_string()),
        None => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::px4_ready::NSH_REPLY_TIMEOUT;
    use crate::test_support::{fake_nsh, FakeReply};
    use std::sync::{Arc, Mutex};
    use tokio::time::Instant;

    fn new_log() -> Arc<Mutex<Vec<String>>> {
        Arc::new(Mutex::new(Vec::new()))
    }

    #[test]
    fn internal_request_ids_start_above_every_fixed_internal_id() {
        let ids = InternalRequestIds::new();
        assert_eq!((ids.next(), ids.next()), (0xFFFF_FF10, 0xFFFF_FF11));
    }

    #[test]
    fn internal_request_ids_wrap_within_the_internal_range() {
        let ids = InternalRequestIds::starting_at(u32::MAX);
        assert_eq!((ids.next(), ids.next()), (u32::MAX, 0xFFFF_FF10));
    }

    #[test]
    fn ekf2_stop_passes_on_any_reply() {
        for out in [
            "ekf2 stop\r\nnsh> ",
            "ekf2 not running",
            "ERROR [ekf2] not running",
            "",
        ] {
            assert_eq!(judge_ekf2_stop(out), Ok(()));
        }
    }

    #[test]
    fn ekf2_start_fails_on_error_or_failed_in_any_case() {
        assert_eq!(judge_ekf2_start("ekf2 start\r\nnsh> "), Ok(()));
        assert_eq!(judge_ekf2_start("INFO [ekf2] starting"), Ok(()));
        assert_eq!(
            judge_ekf2_start("ekf2 start\r\nERROR [ekf2] task start failed\r\nnsh> "),
            Err("ERROR [ekf2] task start failed".to_string())
        );
        assert_eq!(
            judge_ekf2_start("Ekf2 FAILED to allocate"),
            Err("Ekf2 FAILED to allocate".to_string())
        );
    }

    #[tokio::test]
    async fn run_nsh_returns_the_reply_for_its_own_request_id() {
        // Capacity 16: a test never queues more than a few commands or replies.
        let (commands, mut inbox) = mpsc::channel::<ValidatedNshCommand>(16);
        let (replies, _) = broadcast::channel::<NshResponse>(16);
        let reply_tx = replies.clone();
        tokio::spawn(async move {
            while let Some(cmd) = inbox.recv().await {
                for (id, text) in [(cmd.request_id + 1, "other"), (cmd.request_id, "mine")] {
                    let _ = reply_tx.send(NshResponse {
                        request_id: id,
                        success: true,
                        complete: true,
                        output: text.to_string(),
                    });
                }
            }
        });
        let link = NshLink { commands, replies };
        assert_eq!(
            run_nsh(&link, 0xFFFF_FF10, "ekf2 stop", NSH_REPLY_TIMEOUT).await,
            Ok("mine".to_string())
        );
    }

    #[tokio::test]
    async fn run_nsh_reports_the_nsh_tasks_failure_text() {
        let link = fake_nsh(new_log(), |_| FakeReply::Fail("No FC connected"));
        assert_eq!(
            run_nsh(&link, 0xFFFF_FF10, "ekf2 stop", NSH_REPLY_TIMEOUT).await,
            Err("No FC connected".to_string())
        );
    }

    #[tokio::test(start_paused = true)]
    async fn run_nsh_gives_up_after_three_seconds_without_a_reply() {
        let link = fake_nsh(new_log(), |_| FakeReply::Silent);
        let t = Instant::now();
        assert_eq!(
            run_nsh(&link, 0xFFFF_FF10, "ekf2 stop", NSH_REPLY_TIMEOUT).await,
            Err("no reply within 3 s".to_string())
        );
        assert_eq!(t.elapsed(), Duration::from_secs(3));
    }

    #[tokio::test(start_paused = true)]
    async fn run_nsh_names_a_sub_second_timeout_in_milliseconds() {
        let link = fake_nsh(new_log(), |_| FakeReply::Silent);
        assert_eq!(
            run_nsh(&link, 0xFFFF_FF10, "ekf2 stop", Duration::from_millis(300)).await,
            Err("no reply within 300 ms".to_string())
        );
    }

    #[tokio::test]
    async fn run_nsh_fails_fast_on_a_full_queue() {
        // Capacity 1, already holding one command, and no consumer.
        let (commands, _inbox) = mpsc::channel::<ValidatedNshCommand>(1);
        let (replies, _) = broadcast::channel::<NshResponse>(16);
        let filler = ValidatedNshCommand {
            request_id: 1,
            command: "filler".to_string(),
            timeout_ms: 1000,
            client_id: 0,
        };
        commands.try_send(filler).unwrap();
        let link = NshLink { commands, replies };
        assert_eq!(
            run_nsh(&link, 0xFFFF_FF10, "ekf2 stop", NSH_REPLY_TIMEOUT).await,
            Err("NSH queue full".to_string())
        );
    }
}
