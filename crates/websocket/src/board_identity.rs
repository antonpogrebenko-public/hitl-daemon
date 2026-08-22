//! Stable identity for a connected flight controller.
//!
//! A parameter snapshot is only safe to restore onto the board it was taken
//! from — replaying one board's tuning onto another is worse than having no
//! snapshot at all. That makes identity a correctness concern, not a
//! convenience.
//!
//! A serial port path is not identity: it changes across replug and differs per
//! machine. `fc_model` is not identity either — every PX4 quad reports
//! "PX4 Quadrotor". What is left is `AUTOPILOT_VERSION`, which carries a
//! hardware UID on boards that have one.

use mavlink::ardupilotmega::AUTOPILOT_VERSION_DATA;

/// An opaque, stable key for one physical board.
///
/// The string form is what gets sent to the browser and stored alongside a
/// snapshot. It is an identifier, never a secret: anyone on the machine can
/// read it off the FC, so it must not be usable as an access token.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct BoardIdentity(String);

impl BoardIdentity {
    pub fn as_str(&self) -> &str {
        &self.0
    }

    /// Construct from a literal. Test-only: production identities must come
    /// from `derive` so the derivation rules cannot be bypassed.
    #[cfg(test)]
    pub fn from_raw_for_test(raw: &str) -> Self {
        Self(raw.to_string())
    }
}

impl std::fmt::Display for BoardIdentity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// Derive a board identity, preferring the strongest signal available.
///
/// Order matters:
/// 1. `uid` — a genuine hardware serial on boards that expose one.
/// 2. A composite of vendor, product, board version and system id. This is
///    weaker: two identical boards of the same model on the same machine would
///    collide. It is still better than refusing to snapshot at all, because the
///    common case is one FC per machine.
///
/// The MAVLink 2 `uid2` extension would sit between the two, and boards that
/// leave `uid` zero are exactly where it would help. It is not used here
/// because `mavlink` 0.13.1 does not generate the field on
/// `AUTOPILOT_VERSION_DATA` at all — there is nothing to read. Revisit if the
/// binding gains it; until then a zero-`uid` board falls straight to the
/// composite.
///
/// Returns `None` when nothing distinguishing is available, which the caller
/// must treat as "cannot snapshot this board" rather than inventing a key.
pub fn derive(av: &AUTOPILOT_VERSION_DATA, system_id: u8) -> Option<BoardIdentity> {
    if av.uid != 0 {
        return Some(BoardIdentity(format!("uid:{:016x}", av.uid)));
    }

    // Composite fallback. Deliberately includes system_id so two boards on one
    // machine at least stand a chance of differing. Firmware version is
    // deliberately excluded: identity must survive a firmware update, or a
    // snapshot would become unrestorable the moment the user flashes.
    if av.vendor_id != 0 || av.product_id != 0 || av.board_version != 0 {
        return Some(BoardIdentity(format!(
            "board:{:04x}-{:04x}-{:08x}-{:02x}",
            av.vendor_id, av.product_id, av.board_version, system_id
        )));
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::ardupilotmega::MavProtocolCapability;

    fn autopilot_version() -> AUTOPILOT_VERSION_DATA {
        AUTOPILOT_VERSION_DATA {
            capabilities: MavProtocolCapability::empty(),
            uid: 0,
            flight_sw_version: 0,
            middleware_sw_version: 0,
            os_sw_version: 0,
            board_version: 0,
            vendor_id: 0,
            product_id: 0,
            flight_custom_version: [0; 8],
            middleware_custom_version: [0; 8],
            os_custom_version: [0; 8],
        }
    }

    #[test]
    fn prefers_the_hardware_uid() {
        let mut av = autopilot_version();
        av.uid = 0x0123_4567_89ab_cdef;
        // Even with a usable composite available, uid wins: it is the only
        // field that is genuinely per-board.
        av.vendor_id = 0x26ac;
        av.product_id = 0x0011;

        let id = derive(&av, 1).expect("uid is present");
        assert_eq!(id.as_str(), "uid:0123456789abcdef");
    }

    #[test]
    fn identity_survives_a_firmware_update() {
        // Snapshots have to outlive a reflash. Including any firmware version
        // field would silently orphan every snapshot on update.
        let mut before = autopilot_version();
        before.vendor_id = 0x26ac;
        before.flight_sw_version = 0x0100_0000;
        let mut after = autopilot_version();
        after.vendor_id = 0x26ac;
        after.flight_sw_version = 0x0200_0000;

        assert_eq!(derive(&before, 1), derive(&after, 1));
    }

    #[test]
    fn falls_back_to_composite_when_no_uid_is_available() {
        let mut av = autopilot_version();
        av.vendor_id = 0x26ac;
        av.product_id = 0x0032;
        av.board_version = 0x0000_0009;

        let id = derive(&av, 1).expect("composite is available");
        assert_eq!(id.as_str(), "board:26ac-0032-00000009-01");
    }

    #[test]
    fn two_distinct_boards_produce_distinct_keys() {
        let mut first = autopilot_version();
        first.uid = 0x1111_1111_1111_1111;
        let mut second = autopilot_version();
        second.uid = 0x2222_2222_2222_2222;

        assert_ne!(derive(&first, 1), derive(&second, 1));
    }

    #[test]
    fn the_same_board_produces_a_stable_key_across_reads() {
        let mut av = autopilot_version();
        av.uid = 0xfeed_face_dead_beef;
        // Identity must not drift between reads, or a restore would be refused
        // on the very board the snapshot came from.
        assert_eq!(derive(&av, 1), derive(&av, 1));
    }

    #[test]
    fn composite_distinguishes_boards_on_different_system_ids() {
        let mut av = autopilot_version();
        av.vendor_id = 0x26ac;
        assert_ne!(derive(&av, 1), derive(&av, 2));
    }

    #[test]
    fn nothing_distinguishing_yields_no_identity() {
        // Better to refuse the snapshot than to hand every anonymous board the
        // same key and let one board's tuning be restored onto another.
        assert_eq!(derive(&autopilot_version(), 1), None);
    }
}
