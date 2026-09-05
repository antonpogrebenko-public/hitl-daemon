//! Repo hygiene: a crate whose files changed must have moved its version, and
//! the CHANGELOG's newest entry must be the version the root manifest ships.
//!
//! This project's rule is that every code change bumps the version of the crate
//! it lands in. A rule enforced only by remembering is enforced for about two
//! weeks, and the first thing it silently costs is release notes: the
//! `release_daemon` tool tags `hitl-daemon-v{version}` from the root manifest,
//! so a forgotten bump publishes a second, different binary under a tag that
//! already exists.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::Command;

/// The repository root — this test file's package directory.
fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// `version = "x.y.z"` from a Cargo manifest's `[package]` section.
///
/// Deliberately a first-match scan rather than a TOML parse: the workspace has
/// no TOML dependency and adding one so a hygiene test can read a line it can
/// already see would be the wrong trade. Every manifest here puts `version`
/// directly under `[package]` at the top, and the assertions below fail loudly
/// rather than silently if that ever stops being true.
fn package_version(manifest: &str) -> Option<String> {
    manifest
        .lines()
        .map(str::trim)
        .take_while(|line| !line.starts_with("[dependencies"))
        .find_map(|line| {
            let rest = line
                .strip_prefix("version")?
                .trim_start()
                .strip_prefix('=')?;
            Some(rest.trim().trim_matches('"').to_string())
        })
}

/// Every `crates/<name>` directory declared in the workspace `members` list.
fn workspace_members(root: &Path) -> Vec<String> {
    let manifest = std::fs::read_to_string(root.join("Cargo.toml"))
        .expect("the root manifest must be readable");
    let members = manifest
        .lines()
        .find(|line| line.trim_start().starts_with("members ="))
        .expect("the workspace must declare its members");
    members
        .split(['[', ']', ',', '"'])
        .map(str::trim)
        .filter(|part| part.starts_with("crates/"))
        .map(str::to_string)
        .collect()
}

#[test]
fn every_workspace_member_exists_and_declares_a_version() {
    let root = repo_root();
    let members = workspace_members(&root);

    assert!(
        members.iter().any(|m| m == "crates/rtsched"),
        "crates/rtsched must be a workspace member, or `cargo test --workspace` never \
         compiles it and the FFI in it rots unnoticed. Members: {members:?}"
    );

    for member in &members {
        let manifest_path = root.join(member).join("Cargo.toml");
        let manifest = std::fs::read_to_string(&manifest_path)
            .unwrap_or_else(|e| panic!("{} is a declared member but {e}", manifest_path.display()));
        let version =
            package_version(&manifest).unwrap_or_else(|| panic!("{member} declares no version"));
        assert!(
            !version.is_empty() && version != "0.0.0",
            "{member} must carry a real version, got {version:?}"
        );
    }
}

#[test]
fn the_changelogs_newest_entry_is_the_version_the_root_manifest_ships() {
    let root = repo_root();
    let manifest = std::fs::read_to_string(root.join("Cargo.toml")).expect("root manifest");
    let shipped = package_version(&manifest).expect("the root manifest must declare a version");

    let changelog = std::fs::read_to_string(root.join("CHANGELOG.md")).expect("CHANGELOG.md");
    let newest = changelog
        .lines()
        .find_map(|line| {
            line.trim()
                .strip_prefix("## [")?
                .split(']')
                .next()
                .map(str::to_string)
        })
        .expect("the CHANGELOG must have at least one `## [version]` heading");

    assert_eq!(
        newest, shipped,
        "the CHANGELOG's newest entry is [{newest}] but the root manifest ships {shipped}. \
         `release_daemon` tags `hitl-daemon-v{{version}}` from the manifest, so these \
         disagreeing means a release goes out with someone else's release notes"
    );
}

/// Paths git reports as changed in this working tree: tracked modifications
/// UNION untracked files.
///
/// `git diff --name-only HEAD` lists tracked modifications only, so a crate
/// whose entire change is new files is invisible to it. That is not
/// hypothetical: `crates/rtsched/` and `tests/` arrived in this repo as
/// untracked directories, so the rule could not see the crate the release that
/// introduced it added. `git ls-files --others --exclude-standard` supplies the
/// other half, listing untracked FILES (never bare directories), which keeps
/// the `crates/<name>/…` mapping below working unchanged.
///
/// `None` means git could not answer and the caller should skip, not fail.
fn changed_paths(root: &Path) -> Option<Vec<String>> {
    let run = |args: &[&str]| -> Option<String> {
        Command::new("git").args(args).current_dir(root).ok_output()
    };
    let tracked = run(&["diff", "--name-only", "HEAD"])?;
    let untracked = run(&["ls-files", "--others", "--exclude-standard"])?;
    Some(
        tracked
            .lines()
            .chain(untracked.lines())
            .filter(|l| !l.is_empty())
            .map(str::to_string)
            .collect(),
    )
}

/// `Command::output()` reduced to "the stdout, if the command ran and
/// succeeded and produced UTF-8". Every failure mode here means "git cannot
/// answer", which this file treats as a skip.
trait OkOutput {
    fn ok_output(&mut self) -> Option<String>;
}

impl OkOutput for Command {
    fn ok_output(&mut self) -> Option<String> {
        let out = self.output().ok()?;
        if !out.status.success() {
            return None;
        }
        String::from_utf8(out.stdout).ok()
    }
}

/// Best-effort, VCS-backed half of the rule.
///
/// Skips — passing — when git is unavailable or the tree is clean, which covers
/// a packaged crate, a CI checkout without history, and the ordinary
/// everything-is-committed state. It fires exactly when there are uncommitted
/// changes, which is the moment the bump is still cheap to make.
///
/// # It only ever runs locally, and it has an escape hatch
///
/// On a clean CI checkout `changed` is empty and this returns immediately, so
/// nothing in CI enforces the bump rule — a developer's own working tree is the
/// only place this fires. That is also the one place it is a nuisance: during
/// ordinary TDD (edit `crates/simulation`, run the tests) it reports a
/// version-hygiene failure instead of the logic result being looked for, and it
/// cannot be satisfied without bumping mid-edit.
///
/// `HITL_SKIP_VERSION_HYGIENE=1` turns it off for that case. Opt-OUT rather
/// than opt-in on purpose: an opt-in check that CI never runs is a check that
/// runs nowhere.
#[test]
fn a_crate_whose_files_changed_has_moved_its_version() {
    if std::env::var_os("HITL_SKIP_VERSION_HYGIENE").is_some() {
        eprintln!("HITL_SKIP_VERSION_HYGIENE is set; version-bump check skipped");
        return;
    }

    let root = repo_root();

    let Some(changed) = changed_paths(&root) else {
        eprintln!("git could not describe this tree; version-bump check skipped");
        return;
    };
    let changed: BTreeSet<&str> = changed.iter().map(String::as_str).collect();
    if changed.is_empty() {
        return;
    }

    // Which member crates have any changed file at all.
    let mut touched: BTreeSet<String> = BTreeSet::new();
    // And whether anything outside `crates/` changed, which is the root
    // `hitl-daemon` package: src/main.rs, src/tui.rs and src/timebase_cli.rs
    // all live there, and no assertion used to notice a missing root bump.
    // CHANGELOG.md and Cargo.lock are excluded because neither is code —
    // the CHANGELOG is checked against the root version by its own test, and
    // Cargo.lock moves whenever any member does.
    let mut root_touched = false;
    for path in &changed {
        let Some(rest) = path.strip_prefix("crates/") else {
            if !matches!(*path, "CHANGELOG.md" | "Cargo.lock") {
                root_touched = true;
            }
            continue;
        };
        let Some(name) = rest.split('/').next() else {
            continue;
        };
        touched.insert(name.to_string());
    }

    if root_touched {
        let current = package_version(
            &std::fs::read_to_string(root.join("Cargo.toml")).expect("root manifest"),
        )
        .expect("the root manifest must declare a version");
        if let Some(previous) = Command::new("git")
            .args(["show", "HEAD:Cargo.toml"])
            .current_dir(&root)
            .ok_output()
            .as_deref()
            .and_then(package_version)
        {
            assert_ne!(
                current, previous,
                "the root hitl-daemon package has uncommitted changes outside crates/ but is \
                 still at {previous}. `release_daemon` tags `hitl-daemon-v{{version}}` from \
                 this manifest, so a forgotten bump publishes a second, different binary \
                 under a tag that already exists"
            );
        }
    }

    for name in touched {
        let manifest_rel = format!("crates/{name}/Cargo.toml");
        let manifest_path = root.join(&manifest_rel);
        let current = std::fs::read_to_string(&manifest_path)
            .ok()
            .and_then(|m| package_version(&m));
        let Some(current) = current else { continue };

        // A crate that is new in this diff has no committed version to compare
        // against, and adding one is a bump by definition.
        let Ok(show) = Command::new("git")
            .arg("show")
            .arg(format!("HEAD:{manifest_rel}"))
            .current_dir(&root)
            .output()
        else {
            continue;
        };
        if !show.status.success() {
            continue;
        }
        let Some(previous) = String::from_utf8(show.stdout)
            .ok()
            .and_then(|m| package_version(&m))
        else {
            continue;
        };

        assert_ne!(
            current, previous,
            "crates/{name} has uncommitted changes but is still at {previous}. This project \
             bumps the version of every crate it changes, so a consumer can tell two builds \
             apart from the manifest alone"
        );
    }
}
