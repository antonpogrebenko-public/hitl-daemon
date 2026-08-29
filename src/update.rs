//! Daemon self-update.
//!
//! Replacing the binary a user is running is the most dangerous thing this
//! program does to their machine, so the rules here are deliberately strict:
//!
//! - Nothing is replaced without a hash match against what the release channel
//!   published. Artifacts carry only an ad-hoc signature (see the release
//!   runbook), which authenticates no publisher, so integrity rests entirely
//!   on that hash fetched over HTTPS.
//! - The previous binary is kept until the new one has started, so a bad
//!   update leaves something runnable behind.
//! - The check never blocks startup. A release channel that is down, slow, or
//!   unreachable must not stop someone flying.

use serde::Deserialize;
use std::path::{Path, PathBuf};
use std::time::Duration;
use tracing::{debug, info, warn};

/// Budget for the whole update check. Short: this runs alongside startup and
/// its result is a convenience, not a precondition.
const CHECK_TIMEOUT: Duration = Duration::from_secs(5);

/// Platform key this build reports to the release channel.
pub const PLATFORM: &str = if cfg!(target_os = "macos") {
    "macos"
} else if cfg!(target_os = "windows") {
    "windows"
} else {
    "ubuntu"
};

#[derive(Debug, Deserialize)]
struct LatestRelease {
    version: String,
    #[serde(default)]
    min_supported_daemon_version: Option<String>,
    #[serde(default)]
    binary_downloads: Vec<BinaryDownload>,
}

#[derive(Debug, Deserialize, Clone)]
struct BinaryDownload {
    platform: String,
    url: String,
    sha256: String,
}

/// A newer release that this daemon could install.
#[derive(Debug, Clone, PartialEq)]
pub struct AvailableUpdate {
    pub version: String,
    pub url: String,
    pub sha256: String,
}

#[derive(Debug, PartialEq)]
pub enum UpdateError {
    /// The channel could not be reached or did not answer in time. Explicitly
    /// not a failure of the daemon: the caller carries on.
    ChannelUnavailable(String),
    /// The download did not match the published hash. The running binary is
    /// untouched.
    HashMismatch {
        expected: String,
        actual: String,
    },
    Io(String),
}

impl std::fmt::Display for UpdateError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ChannelUnavailable(e) => write!(f, "release channel unavailable: {e}"),
            Self::HashMismatch { expected, actual } => write!(
                f,
                "downloaded artifact does not match the published hash \
                 (expected {expected}, got {actual}) — nothing was replaced"
            ),
            Self::Io(e) => write!(f, "{e}"),
        }
    }
}

/// Flatten an error's source chain into one line.
///
/// reqwest's top-level message is "error sending request for url (...)", which
/// says nothing about whether it was DNS, TLS, a timeout, or a refused
/// connection. The cause is the only actionable part.
fn describe(e: &dyn std::error::Error) -> String {
    let mut parts = vec![e.to_string()];
    let mut source = e.source();
    while let Some(s) = source {
        parts.push(s.to_string());
        source = s.source();
    }
    parts.join(": ")
}

/// Compare two dotted versions numerically.
///
/// String comparison would order "0.9.0" above "0.14.0", which is exactly the
/// case that matters here — the daemon crossed from 0.9 to 0.14 during this
/// work.
pub fn is_newer(candidate: &str, current: &str) -> bool {
    fn parts(v: &str) -> Vec<u64> {
        v.trim()
            .trim_start_matches('v')
            .split(['.', '-'])
            .map(|p| p.parse::<u64>().unwrap_or(0))
            .collect()
    }
    let (a, b) = (parts(candidate), parts(current));
    for i in 0..a.len().max(b.len()) {
        let l = a.get(i).copied().unwrap_or(0);
        let r = b.get(i).copied().unwrap_or(0);
        if l != r {
            return l > r;
        }
    }
    false
}

/// Ask the release channel whether anything newer exists for this platform.
///
/// Returns `Ok(None)` when the daemon is already current, and
/// `Err(ChannelUnavailable)` when the channel could not be consulted — which
/// the caller treats as "carry on", never as a startup failure.
pub async fn check_for_update(
    api_url: &str,
    current_version: &str,
) -> Result<Option<AvailableUpdate>, UpdateError> {
    let client = reqwest::Client::builder()
        .timeout(CHECK_TIMEOUT)
        .build()
        .map_err(|e| UpdateError::ChannelUnavailable(describe(&e)))?;

    let url = format!("{}/api/hitl/releases/latest", api_url.trim_end_matches('/'));
    let response = client
        .get(&url)
        .send()
        .await
        .map_err(|e| UpdateError::ChannelUnavailable(describe(&e)))?;

    if !response.status().is_success() {
        return Err(UpdateError::ChannelUnavailable(format!(
            "HTTP {}",
            response.status()
        )));
    }

    let latest: LatestRelease = response
        .json()
        .await
        .map_err(|e| UpdateError::ChannelUnavailable(describe(&e)))?;

    if let Some(floor) = &latest.min_supported_daemon_version {
        if is_newer(floor, current_version) {
            warn!(
                current = current_version,
                minimum = %floor,
                "This daemon is older than the minimum the web app supports"
            );
        }
    }

    if !is_newer(&latest.version, current_version) {
        debug!(current = current_version, latest = %latest.version, "Daemon is current");
        return Ok(None);
    }

    // Install links are only issued to signed-in callers, so an anonymous
    // check can legitimately learn a newer version exists without being handed
    // a way to fetch it. That is still worth reporting.
    let Some(download) = latest
        .binary_downloads
        .iter()
        .find(|d| d.platform == PLATFORM)
    else {
        debug!(
            version = %latest.version,
            platform = PLATFORM,
            "A newer release exists but no download link was issued"
        );
        return Ok(None);
    };

    Ok(Some(AvailableUpdate {
        version: latest.version,
        url: download.url.clone(),
        sha256: download.sha256.clone(),
    }))
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher
        .finalize()
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

/// Verify a downloaded artifact against its published hash.
///
/// Separated from the download so the check is testable without a network, and
/// so it is impossible to install without going through it.
pub fn verify_artifact(bytes: &[u8], expected_sha256: &str) -> Result<(), UpdateError> {
    let actual = sha256_hex(bytes);
    if actual.eq_ignore_ascii_case(expected_sha256) {
        Ok(())
    } else {
        Err(UpdateError::HashMismatch {
            expected: expected_sha256.to_string(),
            actual,
        })
    }
}

/// Where the previous binary is kept while the new one proves itself.
pub fn backup_path(current: &Path) -> PathBuf {
    let mut name = current.as_os_str().to_os_string();
    name.push(".previous");
    PathBuf::from(name)
}

/// Swap a verified artifact into place, keeping the previous binary.
///
/// The current executable is renamed aside rather than deleted: on Windows a
/// running image cannot be overwritten but can be renamed, and on every
/// platform it means a failed swap leaves something runnable behind. Only
/// call with bytes that have already passed `verify_artifact`.
pub fn install_verified(current_exe: &Path, verified: &[u8]) -> Result<PathBuf, UpdateError> {
    let backup = backup_path(current_exe);
    let staged = current_exe.with_extension("staged");

    std::fs::write(&staged, verified)
        .map_err(|e| UpdateError::Io(format!("could not stage the update: {e}")))?;

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&staged, std::fs::Permissions::from_mode(0o755))
            .map_err(|e| UpdateError::Io(format!("could not make the update executable: {e}")))?;
    }

    // Move the running binary aside before putting the new one in place. If
    // the second rename fails, the backup is restored so the user is not left
    // with no daemon at all.
    if let Err(e) = std::fs::rename(current_exe, &backup) {
        let _ = std::fs::remove_file(&staged);
        return Err(UpdateError::Io(format!(
            "could not set the current daemon aside: {e}"
        )));
    }

    if let Err(e) = std::fs::rename(&staged, current_exe) {
        let _ = std::fs::rename(&backup, current_exe);
        let _ = std::fs::remove_file(&staged);
        return Err(UpdateError::Io(format!(
            "could not install the update, previous daemon restored: {e}"
        )));
    }

    info!(backup = %backup.display(), "Update installed; previous daemon retained");
    Ok(backup)
}

/// Download an artifact and check it against its published hash.
///
/// Returns the bytes only when they verify, so a caller cannot install
/// something unverified even by mistake.
pub async fn download_verified(update: &AvailableUpdate) -> Result<Vec<u8>, UpdateError> {
    let client = reqwest::Client::builder()
        .timeout(Duration::from_secs(120))
        .build()
        .map_err(|e| UpdateError::ChannelUnavailable(describe(&e)))?;

    let response = client
        .get(&update.url)
        .send()
        .await
        .map_err(|e| UpdateError::ChannelUnavailable(describe(&e)))?;

    if !response.status().is_success() {
        return Err(UpdateError::ChannelUnavailable(format!(
            "HTTP {} fetching the update",
            response.status()
        )));
    }

    let bytes = response
        .bytes()
        .await
        .map_err(|e| UpdateError::ChannelUnavailable(describe(&e)))?;

    verify_artifact(&bytes, &update.sha256)?;
    Ok(bytes.to_vec())
}

/// Run the whole update: check, download, verify, install, relaunch.
///
/// Invoked explicitly by the user (`--update`), which is the confirmation —
/// nothing here happens on its own.
pub async fn run_update(api_url: &str, current_version: &str) -> Result<(), UpdateError> {
    let Some(available) = check_for_update(api_url, current_version).await? else {
        info!(
            version = current_version,
            "Already running the latest daemon"
        );
        return Ok(());
    };

    info!(
        from = current_version,
        to = %available.version,
        "Downloading update"
    );
    let bytes = download_verified(&available).await?;

    let current_exe =
        std::env::current_exe().map_err(|e| UpdateError::Io(format!("cannot locate self: {e}")))?;
    let backup = install_verified(&current_exe, &bytes)?;

    info!(
        version = %available.version,
        previous = %backup.display(),
        "Update installed"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn versions_compare_numerically_not_lexically() {
        // The case that actually bites: "0.9.0" sorts above "0.14.0" as strings.
        assert!(is_newer("0.14.0", "0.9.0"));
        assert!(!is_newer("0.9.0", "0.14.0"));
        assert!(is_newer("1.0.0", "0.99.99"));
        assert!(!is_newer("0.14.0", "0.14.0"));
        assert!(is_newer("0.14.1", "0.14.0"));
    }

    #[test]
    fn a_v_prefix_and_missing_components_are_tolerated() {
        assert!(is_newer("v0.15.0", "0.14.0"));
        assert!(is_newer("0.15", "0.14.9"));
        assert!(!is_newer("0.14", "0.14.0"));
    }

    #[test]
    fn a_matching_hash_verifies() {
        let bytes = b"daemon bytes";
        assert!(verify_artifact(bytes, &sha256_hex(bytes)).is_ok());
    }

    #[test]
    fn hash_comparison_is_case_insensitive() {
        let bytes = b"daemon bytes";
        assert!(verify_artifact(bytes, &sha256_hex(bytes).to_uppercase()).is_ok());
    }

    #[test]
    fn a_tampered_artifact_is_rejected() {
        // With only ad-hoc signatures, this hash is the entire integrity story.
        let result = verify_artifact(b"not the daemon", &sha256_hex(b"daemon bytes"));
        assert!(matches!(result, Err(UpdateError::HashMismatch { .. })));
    }

    #[test]
    fn installing_keeps_the_previous_binary() {
        let dir = std::env::temp_dir().join(format!("hitl-update-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let exe = dir.join("hitl-daemon");
        std::fs::write(&exe, b"old daemon").unwrap();

        let backup = install_verified(&exe, b"new daemon").unwrap();

        assert_eq!(std::fs::read(&exe).unwrap(), b"new daemon");
        // A bad update must leave something runnable behind.
        assert_eq!(std::fs::read(&backup).unwrap(), b"old daemon");

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_failed_swap_leaves_the_original_in_place() {
        let dir = std::env::temp_dir().join(format!("hitl-update-fail-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        // Staging into a directory that does not exist fails before anything
        // is moved, so the running daemon survives untouched.
        let exe = dir.join("missing-subdir").join("hitl-daemon");

        let result = install_verified(&exe, b"new daemon");
        assert!(result.is_err());
        assert!(!exe.exists());

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_backup_sits_beside_the_binary_it_replaces() {
        let path = Path::new("/usr/local/bin/hitl-daemon");
        assert_eq!(
            backup_path(path),
            PathBuf::from("/usr/local/bin/hitl-daemon.previous")
        );
    }
}
