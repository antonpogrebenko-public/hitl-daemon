use parking_lot::RwLock as SyncRwLock;

/// Where the simulated flight is, and what "zero altitude" means for it.
///
/// One value, read by everything that has an opinion about altitude:
///
///   * ground contact  — terrain sampled relative to `alt_datum`
///   * barometer       — `alt_datum - position.down`
///   * HIL_GPS MSL     — `alt_datum + height above origin`
///
/// These must agree. When they did not, ground collision sat on the DEM while
/// the barometer and GPS sat on the CLI `--alt`, and the EKF had to absorb the
/// difference as a standing altitude error.
///
/// Set once at configuration time rather than revised when tiles arrive: the
/// browser has already sampled the elevation at the origin by then, so there is
/// no window in which the datum has changed for some consumers and not others.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OriginValues {
    pub lat: f64,
    pub lon: f64,
    /// MSL metres at the origin. Terrain elevation where it is known, otherwise
    /// the configured altitude.
    pub alt_datum: f64,
    /// Whether `alt_datum` came from terrain. False means no elevation data
    /// covered the origin and the configured altitude is standing in — worth
    /// reporting, because ground contact is then flat.
    pub from_terrain: bool,
}

/// Thread-safe holder. Written from the WebSocket task at configuration time,
/// read from the 400 Hz simulation loop.
#[derive(Debug)]
pub struct SharedOrigin {
    inner: SyncRwLock<OriginValues>,
}

impl SharedOrigin {
    pub fn new(lat: f64, lon: f64, alt_datum: f64) -> Self {
        Self {
            inner: SyncRwLock::new(OriginValues {
                lat,
                lon,
                alt_datum,
                from_terrain: false,
            }),
        }
    }

    pub fn get(&self) -> OriginValues {
        *self.inner.read()
    }

    /// Adopt a browser-chosen origin.
    ///
    /// `elevation_msl` is `None` when no elevation data covered the point, in
    /// which case `fallback_alt` becomes the datum for all three consumers
    /// together rather than for some of them.
    pub fn set(&self, lat: f64, lon: f64, elevation_msl: Option<f64>, fallback_alt: f64) {
        let mut inner = self.inner.write();
        *inner = OriginValues {
            lat,
            lon,
            alt_datum: elevation_msl.unwrap_or(fallback_alt),
            from_terrain: elevation_msl.is_some(),
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn terrain_elevation_becomes_the_datum() {
        let origin = SharedOrigin::new(0.0, 0.0, 1655.0);
        origin.set(51.5, -0.12, Some(35.0), 1655.0);
        let v = origin.get();
        assert_eq!(v.lat, 51.5);
        assert_eq!(v.lon, -0.12);
        assert_eq!(v.alt_datum, 35.0);
        assert!(v.from_terrain);
    }

    #[test]
    fn unknown_elevation_falls_back_for_every_consumer_together() {
        let origin = SharedOrigin::new(0.0, 0.0, 1655.0);
        origin.set(51.5, -0.12, None, 1655.0);
        let v = origin.get();
        assert_eq!(v.alt_datum, 1655.0);
        assert!(
            !v.from_terrain,
            "callers must be able to tell a real datum from a stand-in"
        );
    }

    #[test]
    fn sea_level_is_a_real_datum_not_a_missing_one() {
        // 0.0 and None are different answers and must not collapse.
        let origin = SharedOrigin::new(0.0, 0.0, 1655.0);
        origin.set(0.0, 0.0, Some(0.0), 1655.0);
        let v = origin.get();
        assert_eq!(v.alt_datum, 0.0);
        assert!(v.from_terrain);
    }
}

#[cfg(test)]
mod datum_parity_tests {
    use super::*;

    /// The three things that derive altitude from the datum, expressed the same
    /// way the simulation loop computes them:
    ///   * barometer   `alt_datum - position.down`   (loop_runner: baro sample)
    ///   * HIL_GPS MSL `alt_datum + height`          (loop_runner: alt_msl)
    ///   * ground      terrain MSL relative to `alt_datum`
    /// A vehicle resting on the ground at the origin must read the same
    /// altitude through all three, or the EKF absorbs the difference.
    struct Readings {
        baro_msl: f64,
        gps_msl: f64,
        ground_msl: f64,
    }

    fn read_all(
        origin: &SharedOrigin,
        ned_down: f64,
        terrain_msl_at_point: Option<f64>,
    ) -> Readings {
        let v = origin.get();
        Readings {
            baro_msl: v.alt_datum - ned_down,
            gps_msl: v.alt_datum + (-ned_down),
            // Ground contact works relative to the datum, so terrain that is
            // unknown leaves the vehicle on the flat datum plane.
            ground_msl: terrain_msl_at_point.unwrap_or(v.alt_datum),
        }
    }

    #[test]
    fn all_three_consumers_agree_when_elevation_is_known() {
        let origin = SharedOrigin::new(0.0, 0.0, 1655.0);
        origin.set(51.5, -0.12, Some(35.0), 1655.0);

        // Resting on the ground at the origin: down = 0, terrain = the datum.
        let r = read_all(&origin, 0.0, Some(35.0));
        assert_eq!(r.baro_msl, 35.0);
        assert_eq!(r.gps_msl, 35.0);
        assert_eq!(r.ground_msl, 35.0);

        // ...and the CLI altitude is nowhere in the answer.
        assert_ne!(r.baro_msl, 1655.0);
    }

    #[test]
    fn all_three_consumers_agree_when_elevation_is_unknown() {
        let origin = SharedOrigin::new(0.0, 0.0, 1655.0);
        origin.set(51.5, -0.12, None, 1655.0);

        let r = read_all(&origin, 0.0, None);
        assert_eq!(r.baro_msl, 1655.0);
        assert_eq!(r.gps_msl, 1655.0);
        assert_eq!(r.ground_msl, 1655.0);
    }

    #[test]
    fn ten_metres_up_moves_all_three_by_ten_metres() {
        let origin = SharedOrigin::new(0.0, 0.0, 1655.0);
        origin.set(51.5, -0.12, Some(35.0), 1655.0);

        let ground = read_all(&origin, 0.0, Some(35.0));
        let aloft = read_all(&origin, -10.0, Some(35.0));
        assert_eq!(aloft.baro_msl - ground.baro_msl, 10.0);
        assert_eq!(aloft.gps_msl - ground.gps_msl, 10.0);
        // The ground beneath does not move with the vehicle.
        assert_eq!(aloft.ground_msl, ground.ground_msl);
    }

    #[test]
    fn the_datum_never_changes_for_only_some_consumers() {
        // The defect this replaces: the CLI path adopted the DEM elevation into
        // reference_alt while the WebSocket path did not, so ground contact sat
        // on the DEM and baro/GPS sat on --alt.
        let origin = SharedOrigin::new(0.0, 0.0, 1655.0);
        for elevation in [Some(35.0), None, Some(0.0), Some(-420.0)] {
            origin.set(51.5, -0.12, elevation, 1655.0);
            let r = read_all(&origin, 0.0, elevation);
            assert_eq!(
                r.baro_msl, r.gps_msl,
                "baro and GPS disagree for elevation {elevation:?}"
            );
            assert_eq!(
                r.baro_msl, r.ground_msl,
                "altitude and ground disagree for elevation {elevation:?}"
            );
        }
    }
}
