//! Every other target: nothing to ask for, and nothing to apologise for.
//!
//! The daemon ships for macOS arm64, Linux x86_64 (musl) and Windows x86_64.
//! Anything else is somebody building from source, and the correct behaviour
//! there is to run at normal priority and say so once — not to fail to build,
//! and not to pretend a promotion happened.

use crate::{Promotion, Verification};
use std::time::Duration;

pub(crate) fn promote(_period: Duration, _computation: Duration) -> Promotion {
    Promotion::Unsupported
}

pub(crate) fn verify() -> Verification {
    Verification::NotApplicable
}

pub(crate) fn revert() {}
