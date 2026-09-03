//! Terrain the physics collides against.
//!
//! The daemon does not fetch elevation data. The browser is the sole fetcher:
//! it resolves each tile (authoritative store first, free global fallback
//! second), decodes it once, and pushes the decoded heights in over the
//! WebSocket. That is what makes "the physics collides against what the viewer
//! draws" true by construction rather than by two systems independently
//! resolving the same coordinate and happening to agree.
//!
//! Because the WebSocket is therefore a data ingress, every tile is validated
//! at the boundary — see [`TerrainCache::insert_tiles`].

use std::sync::atomic::{AtomicU64, Ordering};
use parking_lot::RwLock as SyncRwLock;
use std::collections::HashMap;
use tracing::{debug, warn};

/// Samples per tile edge. Part of the tile contract shared with the store, the
/// baker and the viewer: `{z}/{x}/{y}.bin` is `TILE_SIZE * TILE_SIZE` f32 LE,
/// row-major from the tile's north-west corner, in MSL metres.
pub const TILE_SIZE: usize = 256;
const R_EARTH: f64 = 6378137.0;

/// Elevation bounds for real terrain on Earth, with generous margin.
///
/// Everest is 8849 m and the lowest exposed land is about −430 m. Anything well
/// outside this is corrupt rather than unusual, and colliding against it would
/// fling the vehicle or bury it.
const MIN_ELEVATION_M: f32 = -12_000.0;
const MAX_ELEVATION_M: f32 = 12_000.0;

/// Beyond this the coordinate is not a rounding error, it is a different place.
/// A tile that far from the origin can only be a stale frame or a bad sender,
/// and accepting it would put phantom ground under the vehicle.
const MAX_ORIGIN_DISTANCE_M: f64 = 200_000.0;

/// Slippy zoom levels that exist. Also guards `2u32.pow(z)` from overflowing.
const MAX_ZOOM: u32 = 24;

/// Resident tile bound. 64 tiles of 256^2 f32 is 16 MiB, which is far more
/// than a vehicle-centred ring needs and small enough to never matter.
pub const DEFAULT_MAX_RESIDENT_TILES: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TileCoord {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

impl TileCoord {
    pub fn from_lon_lat(lon: f64, lat: f64, z: u32) -> Self {
        let n = 2_u32.pow(z.min(MAX_ZOOM)) as f64;
        let x = ((lon + 180.0) / 360.0 * n).floor() as u32;
        let lat_rad = lat.to_radians();
        let y = ((1.0 - (lat_rad.tan() + 1.0 / lat_rad.cos()).ln() / std::f64::consts::PI) / 2.0
            * n)
            .floor() as u32;
        Self { x, y, z }
    }
}

/// Why a submitted tile was refused.
///
/// Every rejection names the tile and the reason so the sender can be told
/// rather than left to infer it from terrain that silently never appears.
#[derive(Debug, Clone, PartialEq)]
pub enum TileRejection {
    UnsupportedTileSize {
        declared: u32,
        supported: u32,
    },
    WrongSampleCount {
        got: usize,
        expected: usize,
    },
    CoordOutOfRange {
        coord: TileCoord,
    },
    TooFarFromOrigin {
        metres: f64,
        limit: f64,
    },
    ImplausibleElevation {
        index: usize,
        value: f32,
    },
    /// No origin has been set, so "near the vehicle" has no meaning yet.
    NoOrigin,
}

impl std::fmt::Display for TileRejection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnsupportedTileSize {
                declared,
                supported,
            } => write!(f, "tile_size {declared} is not the supported {supported}"),
            Self::WrongSampleCount { got, expected } => {
                write!(f, "{got} samples, expected {expected}")
            }
            Self::CoordOutOfRange { coord } => write!(
                f,
                "coord {}/{}/{} is out of range for its zoom",
                coord.z, coord.x, coord.y
            ),
            Self::TooFarFromOrigin { metres, limit } => {
                write!(f, "{metres:.0} m from the origin, limit {limit:.0} m")
            }
            Self::ImplausibleElevation { index, value } => {
                write!(f, "sample {index} is {value}, not a real elevation")
            }
            Self::NoOrigin => write!(f, "no flight origin has been set"),
        }
    }
}

/// Outcome of one `insert_tiles` call.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct InsertReport {
    pub accepted: usize,
    pub rejected: Vec<(TileCoord, TileRejection)>,
    /// Tiles dropped to stay within the resident bound.
    pub evicted: usize,
}

#[derive(Debug, Clone)]
struct TileEntry {
    heights: Vec<f32>,
    /// True when this tile came from the free global fallback rather than
    /// authoritative coverage. Drives the "approximate terrain" warning, so it
    /// must survive ingestion.
    approximate: bool,
}

fn tile_to_lon_lat(x: u32, y: u32, z: u32) -> (f64, f64) {
    let n = 2_u32.pow(z.min(MAX_ZOOM)) as f64;
    let lon = x as f64 / n * 360.0 - 180.0;
    let lat_rad = (std::f64::consts::PI * (1.0 - 2.0 * y as f64 / n))
        .sinh()
        .atan();
    (lon, lat_rad.to_degrees())
}

/// Sync terrain cache for use in the physics loop.
///
/// Written from the WebSocket task as tiles arrive, read from the 400 Hz loop.
pub struct TerrainCache {
    inner: SyncRwLock<TerrainCacheInner>,
    max_resident: usize,
    /// Vehicle position in NED metres from the origin, for eviction ordering.
    ///
    /// Outside the lock deliberately. The 400 Hz loop writes this on every
    /// tick, and taking the *exclusive* lock to do so put 400 writer
    /// acquisitions a second in front of tile ingress — parking_lot queues
    /// readers behind a waiting writer, so the loop's own sampler could be made
    /// to wait for a write it had just requested itself.
    ///
    /// It orders eviction and derives which tiles to ask for. Neither needs to
    /// be consistent with the tile map at an instant: a position one tick stale
    /// picks the same tiles, because a tile spans hundreds of metres and the
    /// vehicle moves centimetres per tick.
    vehicle_north: AtomicU64,
    vehicle_east: AtomicU64,
}

/// `f64` through `AtomicU64` — the pair is read and written independently, and
/// a torn read between them is a position that never existed but is still
/// within a tick's travel of one that did.
fn store_f64(cell: &AtomicU64, value: f64) {
    cell.store(value.to_bits(), Ordering::Relaxed);
}

fn load_f64(cell: &AtomicU64) -> f64 {
    f64::from_bits(cell.load(Ordering::Relaxed))
}

struct TerrainCacheInner {
    /// Zoom of the resident tiles. The collision set is always at one zoom —
    /// the finest available — so ground contact never depends on which level of
    /// detail happened to load.
    zoom: Option<u32>,
    tiles: HashMap<TileCoord, TileEntry>,
    origin_lat: f64,
    origin_lon: f64,
    origin_elevation: Option<f64>,
    has_origin: bool,
}

impl TerrainCache {
    pub fn new() -> Self {
        Self::with_capacity(DEFAULT_MAX_RESIDENT_TILES)
    }

    pub fn with_capacity(max_resident: usize) -> Self {
        Self {
            inner: SyncRwLock::new(TerrainCacheInner {
                zoom: None,
                tiles: HashMap::new(),
                origin_lat: 0.0,
                origin_lon: 0.0,
                origin_elevation: None,
                has_origin: false,
            }),
            max_resident: max_resident.max(1),
            vehicle_north: AtomicU64::new(0.0f64.to_bits()),
            vehicle_east: AtomicU64::new(0.0f64.to_bits()),
        }
    }

    /// Anchor the cache to a flight origin.
    ///
    /// Re-anchoring to a materially different place discards the resident
    /// tiles: they describe ground relative to the old origin, and keeping them
    /// would compute contact against terrain anchored somewhere else.
    pub fn set_origin(&self, lat: f64, lon: f64, elevation_msl: Option<f64>) {
        let mut inner = self.inner.write();
        let moved =
            inner.has_origin && distance_m(inner.origin_lat, inner.origin_lon, lat, lon) > 1.0;
        if moved {
            debug!(
                from = ?(inner.origin_lat, inner.origin_lon),
                to = ?(lat, lon),
                dropped = inner.tiles.len(),
                "Terrain re-anchored; dropping tiles anchored to the old origin"
            );
            inner.tiles.clear();
            inner.zoom = None;
        }
        inner.origin_lat = lat;
        inner.origin_lon = lon;
        inner.origin_elevation = elevation_msl;
        inner.has_origin = true;
        store_f64(&self.vehicle_north, 0.0);
        store_f64(&self.vehicle_east, 0.0);
    }

    /// Report where the vehicle is, in NED metres from the origin.
    ///
    /// Used only to order eviction, so the tile under the vehicle is the last
    /// thing dropped.
    pub fn set_vehicle_ned(&self, north: f64, east: f64) {
        store_f64(&self.vehicle_north, north);
        store_f64(&self.vehicle_east, east);
    }

    /// Accept decoded elevation tiles.
    ///
    /// This is the one way terrain enters the physics. Every tile is validated
    /// before it is stored, and a rejected tile leaves previously accepted
    /// terrain untouched — a bad frame degrades to "no new terrain", never to
    /// corrupted ground.
    pub fn insert_tiles(
        &self,
        tile_size: u32,
        tiles: Vec<(TileCoord, Vec<f32>, bool)>,
    ) -> InsertReport {
        let mut report = InsertReport::default();

        if tile_size as usize != TILE_SIZE {
            // One rejection per tile so the sender learns which were dropped.
            for (coord, _, _) in tiles {
                report.rejected.push((
                    coord,
                    TileRejection::UnsupportedTileSize {
                        declared: tile_size,
                        supported: TILE_SIZE as u32,
                    },
                ));
            }
            return report;
        }

        // Validate outside the lock.
        //
        // The 400 Hz loop takes a read lock every 2.5 ms. Validation walks
        // 65 536 samples per tile, so doing it while holding the write lock
        // would stall the simulation for the length of a whole batch — the
        // ingress must not be able to slow the thing it is feeding.
        let (origin_lat, origin_lon, has_origin) = {
            let inner = self.inner.read();
            (inner.origin_lat, inner.origin_lon, inner.has_origin)
        };

        if !has_origin {
            for (coord, _, _) in tiles {
                report.rejected.push((coord, TileRejection::NoOrigin));
            }
            return report;
        }

        let mut valid: Vec<(TileCoord, Vec<f32>, bool)> = Vec::with_capacity(tiles.len());
        for (coord, heights, approximate) in tiles {
            match validate_tile(coord, &heights, origin_lat, origin_lon) {
                Ok(()) => valid.push((coord, heights, approximate)),
                Err(reason) => report.rejected.push((coord, reason)),
            }
        }

        let mut inner = self.inner.write();
        for (coord, heights, approximate) in valid {
            // Tiles all share one zoom. A frame at a different zoom replaces the
            // set rather than mixing levels of detail under the vehicle.
            if inner.zoom != Some(coord.z) {
                inner.tiles.clear();
                inner.zoom = Some(coord.z);
            }
            inner.tiles.insert(
                coord,
                TileEntry {
                    heights,
                    approximate,
                },
            );
            report.accepted += 1;
        }

        report.evicted = evict_to_bound(
            &mut inner,
            self.max_resident,
            load_f64(&self.vehicle_north),
            load_f64(&self.vehicle_east),
        );

        if report.accepted > 0 {
            debug!(
                accepted = report.accepted,
                rejected = report.rejected.len(),
                evicted = report.evicted,
                resident = inner.tiles.len(),
                "Terrain tiles ingested"
            );
        }
        for (coord, reason) in &report.rejected {
            warn!(
                coord = format!("{}/{}/{}", coord.z, coord.x, coord.y),
                reason = %reason,
                "Rejected terrain tile"
            );
        }
        report
    }

    /// Coordinates the physics wants around the vehicle but does not hold.
    ///
    /// The daemon is the only party that knows what it is missing, so it is the
    /// one that asks. An empty result is the steady state.
    pub fn missing_around_vehicle(&self, radius: u32) -> Vec<TileCoord> {
        let inner = self.inner.read();
        let Some(z) = inner.zoom else {
            // Nothing has ever arrived, so there is no zoom to ask at. The
            // browser sends the first batch unprompted after configuration.
            return Vec::new();
        };
        if !inner.has_origin {
            return Vec::new();
        }
        let (north, east) = (load_f64(&self.vehicle_north), load_f64(&self.vehicle_east));
        let (lat, lon) = ned_to_lat_lon(north, east, inner.origin_lat, inner.origin_lon);
        let centre = TileCoord::from_lon_lat(lon, lat, z);

        let r = radius as i64;
        let mut missing = Vec::new();
        for dy in -r..=r {
            for dx in -r..=r {
                let coord = TileCoord {
                    x: (centre.x as i64 + dx).max(0) as u32,
                    y: (centre.y as i64 + dy).max(0) as u32,
                    z,
                };
                if !inner.tiles.contains_key(&coord) {
                    missing.push(coord);
                }
            }
        }
        missing
    }

    /// Whether any resident tile came from the free global fallback.
    pub fn any_approximate(&self) -> bool {
        self.inner.read().tiles.values().any(|t| t.approximate)
    }

    pub fn resident_tiles(&self) -> usize {
        self.inner.read().tiles.len()
    }

    /// Sample ground elevation in NED coordinates relative to origin terrain.
    ///
    /// Returns the NED "down" coordinate of the ground at (north, east):
    /// positive is below origin ground level, negative above. `None` means the
    /// height is *unknown* — outside resident coverage — which is deliberately
    /// distinct from flat: collapsing it to 0.0 teleports a vehicle flying
    /// below the origin datum straight up to it.
    pub fn sample_ground_ned(&self, north: f64, east: f64) -> Option<f32> {
        let inner = self.inner.read();
        let origin_elev = inner.origin_elevation?;
        let msl = inner.sample_msl_ned(north, east)?;
        Some((origin_elev - msl) as f32)
    }

    /// Unit surface normal at (north, east), in NED.
    ///
    /// NED is down-positive, so a level surface returns `[0, 0, -1]` — the
    /// normal points *up*, against the down axis. Returns `None` whenever the
    /// centre falls outside resident coverage, keeping this function's coverage
    /// identical to [`TerrainCache::sample_ground_ned`]'s: a shell where the
    /// ground has a height but no normal would let a caller default it to
    /// "level" and flatten genuinely sloped terrain.
    ///
    /// The probe spacing is deliberately close to the DEM's own sample spacing
    /// (~7 m at zoom 14): sampling much tighter only re-reads the bilinear patch
    /// between the same two posts and reports its slope, not the terrain's.
    pub fn sample_ground_normal_ned(&self, north: f64, east: f64) -> Option<[f32; 3]> {
        const PROBE_M: f64 = 5.0;

        // Height above the datum is the negation of the NED down coordinate.
        let h = |n: f64, e: f64| self.sample_ground_ned(n, e).map(|z| -(z as f64));
        let h_centre = h(north, east)?;

        // Central differences where both probes land inside coverage, one-sided
        // against the centre where only one does.
        let slope = |pos: Option<f64>, neg: Option<f64>| match (pos, neg) {
            (Some(a), Some(b)) => Some((a - b) / (2.0 * PROBE_M)),
            (Some(a), None) => Some((a - h_centre) / PROBE_M),
            (None, Some(b)) => Some((h_centre - b) / PROBE_M),
            (None, None) => None,
        };

        let dh_dn = slope(h(north + PROBE_M, east), h(north - PROBE_M, east))?;
        let dh_de = slope(h(north, east + PROBE_M), h(north, east - PROBE_M))?;

        // Surface height h(n, e) has upward normal (-dh/dn, -dh/de, 1) in
        // (north, east, up). Flipping the third axis into NED down gives -1.
        let len = (dh_dn * dh_dn + dh_de * dh_de + 1.0).sqrt();
        Some([
            (-dh_dn / len) as f32,
            (-dh_de / len) as f32,
            (-1.0 / len) as f32,
        ])
    }

    /// Whether any terrain is resident.
    pub fn is_loaded(&self) -> bool {
        let inner = self.inner.read();
        inner.zoom.is_some() && !inner.tiles.is_empty()
    }

    /// MSL elevation at the origin, as set by [`TerrainCache::set_origin`].
    pub fn origin_elevation_msl(&self) -> Option<f64> {
        self.inner.read().origin_elevation
    }

    /// Whether a vertical datum has been set. Ground contact needs one: without
    /// it `sample_ground_ned` returns `None` no matter how many tiles are
    /// resident, which is a wholly different fault from being off the edge of
    /// coverage and must not be reported as one.
    pub fn has_datum(&self) -> bool {
        self.inner.read().origin_elevation.is_some()
    }

    /// Diagnostic: the coordinate a NED offset resolves to, and what is resident.
    pub fn describe_lookup(&self, north: f64, east: f64) -> String {
        let inner = self.inner.read();
        let Some(z) = inner.zoom else {
            return "no zoom (nothing resident)".to_string();
        };
        let (lat, lon) = ned_to_lat_lon(north, east, inner.origin_lat, inner.origin_lon);
        let want = TileCoord::from_lon_lat(lon, lat, z);
        let mut have: Vec<String> = inner
            .tiles
            .keys()
            .map(|c| format!("{}/{}/{}", c.z, c.x, c.y))
            .collect();
        have.sort();
        format!(
            "origin=({:.7},{:.7}) probe=({:.7},{:.7}) want={}/{}/{} have=[{}]",
            inner.origin_lat,
            inner.origin_lon,
            lat,
            lon,
            want.z,
            want.x,
            want.y,
            have.join(" ")
        )
    }
}

impl TerrainCacheInner {
    /// Bilinear MSL elevation at a NED offset from the origin.
    fn sample_msl_ned(&self, north: f64, east: f64) -> Option<f64> {
        let z = self.zoom?;
        let (lat, lon) = ned_to_lat_lon(north, east, self.origin_lat, self.origin_lon);
        let coord = TileCoord::from_lon_lat(lon, lat, z);
        let entry = self.tiles.get(&coord)?;

        let (nw_lon, nw_lat) = tile_to_lon_lat(coord.x, coord.y, coord.z);
        let (se_lon, se_lat) = tile_to_lon_lat(coord.x + 1, coord.y + 1, coord.z);

        let fx = ((lon - nw_lon) / (se_lon - nw_lon) * (TILE_SIZE - 1) as f64)
            .clamp(0.0, (TILE_SIZE - 1) as f64);
        let fy = ((nw_lat - lat) / (nw_lat - se_lat) * (TILE_SIZE - 1) as f64)
            .clamp(0.0, (TILE_SIZE - 1) as f64);

        let x0 = fx.floor() as usize;
        let y0 = fy.floor() as usize;
        let x1 = (x0 + 1).min(TILE_SIZE - 1);
        let y1 = (y0 + 1).min(TILE_SIZE - 1);
        let dx = fx - x0 as f64;
        let dy = fy - y0 as f64;

        let h = &entry.heights;
        let h00 = h[y0 * TILE_SIZE + x0] as f64;
        let h10 = h[y0 * TILE_SIZE + x1] as f64;
        let h01 = h[y1 * TILE_SIZE + x0] as f64;
        let h11 = h[y1 * TILE_SIZE + x1] as f64;

        Some(
            h00 * (1.0 - dx) * (1.0 - dy)
                + h10 * dx * (1.0 - dy)
                + h01 * (1.0 - dx) * dy
                + h11 * dx * dy,
        )
    }
}

fn validate_tile(
    coord: TileCoord,
    heights: &[f32],
    origin_lat: f64,
    origin_lon: f64,
) -> Result<(), TileRejection> {
    if heights.len() != TILE_SIZE * TILE_SIZE {
        return Err(TileRejection::WrongSampleCount {
            got: heights.len(),
            expected: TILE_SIZE * TILE_SIZE,
        });
    }
    // Checked before `2u32.pow(z)` is evaluated anywhere.
    if coord.z > MAX_ZOOM {
        return Err(TileRejection::CoordOutOfRange { coord });
    }
    let n = 2u32.pow(coord.z);
    if coord.x >= n || coord.y >= n {
        return Err(TileRejection::CoordOutOfRange { coord });
    }

    let (nw_lon, nw_lat) = tile_to_lon_lat(coord.x, coord.y, coord.z);
    let metres = distance_m(origin_lat, origin_lon, nw_lat, nw_lon);
    if metres > MAX_ORIGIN_DISTANCE_M {
        return Err(TileRejection::TooFarFromOrigin {
            metres,
            limit: MAX_ORIGIN_DISTANCE_M,
        });
    }

    for (index, value) in heights.iter().enumerate() {
        if !value.is_finite() || *value < MIN_ELEVATION_M || *value > MAX_ELEVATION_M {
            return Err(TileRejection::ImplausibleElevation {
                index,
                value: *value,
            });
        }
    }
    Ok(())
}

/// Drop the tiles furthest from the vehicle until the resident bound holds.
/// The tile under the vehicle is by construction the nearest, so it is never
/// the one dropped.
fn evict_to_bound(
    inner: &mut TerrainCacheInner,
    max_resident: usize,
    // Passed in rather than read from `inner`: the vehicle position lives
    // outside the lock now, so that the 400 Hz loop does not have to take the
    // exclusive lock to update it.
    north: f64,
    east: f64,
) -> usize {
    if inner.tiles.len() <= max_resident {
        return 0;
    }
    let Some(z) = inner.zoom else { return 0 };
    let (lat, lon) = ned_to_lat_lon(north, east, inner.origin_lat, inner.origin_lon);
    let centre = TileCoord::from_lon_lat(lon, lat, z);

    let mut by_distance: Vec<(i64, TileCoord)> = inner
        .tiles
        .keys()
        .map(|c| {
            let dx = c.x as i64 - centre.x as i64;
            let dy = c.y as i64 - centre.y as i64;
            (dx * dx + dy * dy, *c)
        })
        .collect();
    // Furthest first.
    by_distance.sort_by(|a, b| b.0.cmp(&a.0));

    let excess = inner.tiles.len() - max_resident;
    for (_, coord) in by_distance.into_iter().take(excess) {
        inner.tiles.remove(&coord);
    }
    excess
}

/// Great-circle-ish distance, flat-earth at these scales.
fn distance_m(lat_a: f64, lon_a: f64, lat_b: f64, lon_b: f64) -> f64 {
    let dn = (lat_b - lat_a).to_radians() * R_EARTH;
    let de = (lon_b - lon_a).to_radians() * R_EARTH * lat_a.to_radians().cos();
    (dn * dn + de * de).sqrt()
}

impl Default for TerrainCache {
    fn default() -> Self {
        Self::new()
    }
}

fn ned_to_lat_lon(north: f64, east: f64, origin_lat: f64, origin_lon: f64) -> (f64, f64) {
    let lat = origin_lat + (north / R_EARTH) * (180.0 / std::f64::consts::PI);
    let cos_lat = origin_lat.to_radians().cos();
    let cos_lat_safe = if cos_lat.abs() < 1e-10 {
        1e-10
    } else {
        cos_lat
    };
    let lon = origin_lon + (east / (R_EARTH * cos_lat_safe)) * (180.0 / std::f64::consts::PI);
    (lat, lon)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tile_coord_from_lon_lat() {
        let coord = TileCoord::from_lon_lat(-105.27, 40.01, 14);
        assert_eq!(coord.z, 14);
        assert!(coord.x >= 3397 && coord.x <= 3406);
        assert!(coord.y >= 6196 && coord.y <= 6208);
    }

    #[test]
    fn test_tile_to_lon_lat_roundtrip() {
        let (lon, lat) = tile_to_lon_lat(3400, 6200, 14);
        let coord = TileCoord::from_lon_lat(lon + 0.001, lat - 0.001, 14);
        assert_eq!(coord.x, 3400);
        assert_eq!(coord.y, 6200);
    }

    const TEST_LAT: f64 = 40.015;
    const TEST_LON: f64 = -105.2705;
    const TEST_ZOOM: u32 = 14;

    /// Build a cache holding the 3x3 tile block around (TEST_LAT, TEST_LON) —
    /// the same footprint the real HTTP loader fetches. `height_at(lat, lon)`
    /// supplies MSL metres, so tests can express real-world gradients without
    /// caring which tile a sample lands in.
    fn cache_with_tiles(height_at: impl Fn(f64, f64) -> f32) -> TerrainCache {
        let center = TileCoord::from_lon_lat(TEST_LON, TEST_LAT, TEST_ZOOM);
        let mut tiles = Vec::new();

        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                let coord = TileCoord {
                    x: (center.x as i32 + dx) as u32,
                    y: (center.y as i32 + dy) as u32,
                    z: TEST_ZOOM,
                };
                // Invert the sampler's fx/fy mapping so sample (col, row)
                // resolves back to the lat/lon the sampler will ask for.
                let (nw_lon, nw_lat) = tile_to_lon_lat(coord.x, coord.y, coord.z);
                let (se_lon, se_lat) = tile_to_lon_lat(coord.x + 1, coord.y + 1, coord.z);
                let last = (TILE_SIZE - 1) as f64;

                let mut heights = vec![0.0f32; TILE_SIZE * TILE_SIZE];
                for row in 0..TILE_SIZE {
                    let lat = nw_lat - (row as f64 / last) * (nw_lat - se_lat);
                    for col in 0..TILE_SIZE {
                        let lon = nw_lon + (col as f64 / last) * (se_lon - nw_lon);
                        heights[row * TILE_SIZE + col] = height_at(lat, lon);
                    }
                }
                tiles.push((coord, heights, false));
            }
        }

        let cache = TerrainCache::new();
        // The origin elevation is what the browser sampled at the origin; here
        // it is derived from the same generator so tests stay self-consistent.
        cache.set_origin(
            TEST_LAT,
            TEST_LON,
            Some(height_at(TEST_LAT, TEST_LON) as f64),
        );
        let report = cache.insert_tiles(TILE_SIZE as u32, tiles);
        assert_eq!(report.accepted, 9, "rejected: {:?}", report.rejected);
        cache
    }

    /// The NED ground datum is defined so that ground == 0 exactly at the
    /// origin, whatever the terrain's absolute MSL elevation. The frontend
    /// mesh puts local Y=0 at the same point, so any drift here shows up as
    /// the viewer's ground disagreeing with the physics ground.
    #[test]
    fn sample_ground_ned_is_zero_at_origin() {
        for elevation in [0.0f32, 1655.0, -50.0] {
            let cache = cache_with_tiles(|_, _| elevation);
            let ground = cache
                .sample_ground_ned(0.0, 0.0)
                .expect("origin is inside the loaded tile");
            assert!(
                ground.abs() < 1e-3,
                "ground at origin must be 0 for elevation {elevation}, got {ground}"
            );
        }
    }

    /// NED is down-positive: terrain higher than the origin must report a
    /// negative ground coordinate, terrain lower must report positive.
    #[test]
    fn sample_ground_ned_sign_follows_ned_down_convention() {
        // Ground rises 1 m per 0.001 deg of latitude (~9 m per 100 m north).
        let cache = cache_with_tiles(|lat, _| (1655.0 + (lat - TEST_LAT) * 1000.0) as f32);

        let north = cache
            .sample_ground_ned(500.0, 0.0)
            .expect("500 m north stays inside the 3x3 block");
        let south = cache
            .sample_ground_ned(-500.0, 0.0)
            .expect("500 m south stays inside the 3x3 block");

        assert!(
            north < 0.0,
            "higher terrain to the north must be negative in NED, got {north}"
        );
        assert!(
            south > 0.0,
            "lower terrain to the south must be positive in NED, got {south}"
        );
    }

    /// Outside the cached tiles the sampler must report "unknown" rather than
    /// silently implying flat ground at the origin datum. The physics loop
    /// depends on this distinction to avoid clamping the drone to the wrong
    /// height.
    #[test]
    fn sample_ground_ned_returns_none_outside_coverage() {
        let cache = cache_with_tiles(|_, _| 1655.0);
        // 50 km north is many tiles away at zoom 14.
        assert!(cache.sample_ground_ned(50_000.0, 0.0).is_none());
        assert!(cache.sample_ground_ned(0.0, 50_000.0).is_none());
    }

    #[test]
    fn sample_ground_ned_returns_none_when_not_loaded() {
        let cache = TerrainCache::new();
        assert!(!cache.is_loaded());
        assert!(cache.sample_ground_ned(0.0, 0.0).is_none());
    }

    /// Level ground must produce a normal that points straight up in NED
    /// (down-negative), so the ground contact model can recognise flat terrain
    /// and keep the exact flat-ground accelerometer path.
    #[test]
    fn ground_normal_is_straight_up_on_level_terrain() {
        let cache = cache_with_tiles(|_, _| 1655.0);
        let n = cache
            .sample_ground_normal_ned(0.0, 0.0)
            .expect("inside tiles");
        assert!(
            n[0].abs() < 1e-4,
            "north component should vanish, got {}",
            n[0]
        );
        assert!(
            n[1].abs() < 1e-4,
            "east component should vanish, got {}",
            n[1]
        );
        assert!(
            (n[2] + 1.0).abs() < 1e-4,
            "down component should be -1 (pointing up), got {}",
            n[2]
        );
    }

    /// On a slope rising to the north, the normal must tilt *away* from the
    /// uphill direction — i.e. lean south — which is what makes a drone
    /// resting on it pitch back and slide downhill.
    #[test]
    fn ground_normal_tilts_away_from_rising_terrain() {
        // 1 m of rise per 1 m north: a 45-degree slope.
        let cache = cache_with_tiles(|lat, _| (1655.0 + (lat - TEST_LAT) * 111_320.0) as f32);
        let n = cache
            .sample_ground_normal_ned(0.0, 0.0)
            .expect("inside tiles");

        assert!(
            n[0] < 0.0,
            "normal should lean south (negative north) on north-rising ground, got {}",
            n[0]
        );
        assert!(n[1].abs() < 1e-3, "no east tilt expected, got {}", n[1]);
        // 45 degrees: north and down components have equal magnitude.
        assert!(
            (n[0].abs() - n[2].abs()).abs() < 1e-2,
            "45-degree slope should split the normal evenly, got {n:?}"
        );
    }

    #[test]
    fn ground_normal_is_unit_length() {
        let cache = cache_with_tiles(|lat, lon| {
            (1655.0 + (lat - TEST_LAT) * 40_000.0 + (lon - TEST_LON) * 25_000.0) as f32
        });
        let n = cache
            .sample_ground_normal_ned(0.0, 0.0)
            .expect("inside tiles");
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        assert!(
            (len - 1.0).abs() < 1e-5,
            "expected unit normal, got length {len}"
        );
    }

    /// A normal built from a partly-missing neighbourhood would be wrong in an
    /// unbounded way, so coverage gaps must propagate as None.
    #[test]
    fn ground_normal_returns_none_outside_coverage() {
        let cache = cache_with_tiles(|_, _| 1655.0);
        assert!(cache.sample_ground_normal_ned(50_000.0, 0.0).is_none());
        assert!(TerrainCache::new()
            .sample_ground_normal_ned(0.0, 0.0)
            .is_none());
    }

    /// The normal must be available at exactly the same points the height is.
    /// Central differences alone leave a probe-width shell along the tile-block
    /// edge where the height resolves but the normal does not — and a caller
    /// defaulting that to "level" would flatten genuinely sloped ground, the
    /// same unknown-vs-flat conflation the height path exists to avoid. One-
    /// sided differences at the boundary close the shell.
    #[test]
    fn ground_normal_coverage_matches_ground_height_coverage() {
        // Sloped, so a boundary sample that silently fell back to level would
        // show up as a suspiciously vertical normal.
        let cache = cache_with_tiles(|lat, _| (1655.0 + (lat - TEST_LAT) * 20_000.0) as f32);

        let mut inside = 0;
        let mut outside = 0;
        for i in 0..600 {
            let north = i as f64 * 10.0;
            if cache.sample_ground_ned(north, 0.0).is_some() {
                assert!(
                    cache.sample_ground_normal_ned(north, 0.0).is_some(),
                    "height resolved but normal did not at {north} m north"
                );
                inside += 1;
            } else {
                assert!(
                    cache.sample_ground_normal_ned(north, 0.0).is_none(),
                    "normal resolved but height did not at {north} m north"
                );
                outside += 1;
            }
        }
        assert!(
            inside > 0 && outside > 0,
            "test must span the coverage edge"
        );
    }

    /// At the very edge the one-sided difference must still see the slope
    /// rather than degrading to level ground.
    #[test]
    fn ground_normal_at_the_coverage_edge_still_reports_slope() {
        let cache = cache_with_tiles(|lat, _| (1655.0 + (lat - TEST_LAT) * 20_000.0) as f32);

        let mut last_inside = 0.0;
        for i in 0..600 {
            let north = i as f64 * 10.0;
            if cache.sample_ground_ned(north, 0.0).is_some() {
                last_inside = north;
            }
        }

        let edge = cache
            .sample_ground_normal_ned(last_inside, 0.0)
            .expect("edge point has a height, so it must have a normal");
        let interior = cache.sample_ground_normal_ned(0.0, 0.0).expect("interior");

        assert!(
            edge[0] < 0.0,
            "edge normal should still lean off the rising slope, got {}",
            edge[0]
        );
        assert!(
            (edge[0] - interior[0]).abs() < 0.05,
            "edge normal {edge:?} should closely match interior {interior:?}"
        );
    }

    /// `origin_elevation_msl` is the bridge between the terrain datum and the
    /// baro/GPS reference altitude; it must report the sampled DEM value.
    #[test]
    fn origin_elevation_msl_reports_sampled_dem_value() {
        let cache = cache_with_tiles(|_, _| 1655.0);
        let elev = cache.origin_elevation_msl().expect("origin inside tile");
        assert!(
            (elev - 1655.0).abs() < 1e-3,
            "expected 1655 m MSL at origin, got {elev}"
        );
    }
}

#[cfg(test)]
mod ingress_validation_tests {
    use super::*;

    const LAT: f64 = 40.015;
    const LON: f64 = -105.2705;
    const Z: u32 = 14;

    fn cache() -> TerrainCache {
        let c = TerrainCache::new();
        c.set_origin(LAT, LON, Some(1655.0));
        c
    }

    fn origin_coord() -> TileCoord {
        TileCoord::from_lon_lat(LON, LAT, Z)
    }

    fn flat(v: f32) -> Vec<f32> {
        vec![v; TILE_SIZE * TILE_SIZE]
    }

    #[test]
    fn a_valid_tile_is_accepted() {
        let c = cache();
        let report = c.insert_tiles(
            TILE_SIZE as u32,
            vec![(origin_coord(), flat(1655.0), false)],
        );
        assert_eq!(report.accepted, 1, "rejected: {:?}", report.rejected);
        assert!(report.rejected.is_empty());
        assert!(c.is_loaded());
    }

    #[test]
    fn a_tile_with_the_wrong_sample_count_is_rejected() {
        let c = cache();
        let report = c.insert_tiles(
            TILE_SIZE as u32,
            vec![(origin_coord(), vec![1655.0; 10], false)],
        );
        assert_eq!(report.accepted, 0);
        assert!(matches!(
            report.rejected[0].1,
            TileRejection::WrongSampleCount { .. }
        ));
    }

    #[test]
    fn a_declared_tile_size_that_does_not_match_the_cache_is_rejected() {
        let c = cache();
        // 128 would silently halve every index into the height grid.
        let report = c.insert_tiles(128, vec![(origin_coord(), vec![1655.0; 128 * 128], false)]);
        assert_eq!(report.accepted, 0);
        assert!(matches!(
            report.rejected[0].1,
            TileRejection::UnsupportedTileSize { .. }
        ));
    }

    #[test]
    fn a_coordinate_outside_the_range_for_its_zoom_is_rejected() {
        let c = cache();
        let n = 2u32.pow(Z);
        for bad in [
            TileCoord { x: n, y: 0, z: Z },
            TileCoord { x: 0, y: n, z: Z },
        ] {
            let report = c.insert_tiles(TILE_SIZE as u32, vec![(bad, flat(1655.0), false)]);
            assert_eq!(report.accepted, 0, "{bad:?} must be rejected");
            assert!(matches!(
                report.rejected[0].1,
                TileRejection::CoordOutOfRange { .. }
            ));
        }
    }

    #[test]
    fn an_absurd_zoom_is_rejected_before_it_is_used_as_an_exponent() {
        let c = cache();
        let report = c.insert_tiles(
            TILE_SIZE as u32,
            vec![(TileCoord { x: 0, y: 0, z: 40 }, flat(1655.0), false)],
        );
        assert_eq!(report.accepted, 0);
    }

    #[test]
    fn a_tile_far_from_the_origin_is_rejected() {
        let c = cache();
        // Same zoom, but on the other side of the planet.
        let far = TileCoord::from_lon_lat(100.0, -30.0, Z);
        let report = c.insert_tiles(TILE_SIZE as u32, vec![(far, flat(0.0), false)]);
        assert_eq!(report.accepted, 0);
        assert!(matches!(
            report.rejected[0].1,
            TileRejection::TooFarFromOrigin { .. }
        ));
    }

    #[test]
    fn non_finite_heights_are_rejected() {
        let c = cache();
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut heights = flat(1655.0);
            heights[42] = bad;
            let report = c.insert_tiles(TILE_SIZE as u32, vec![(origin_coord(), heights, false)]);
            assert_eq!(report.accepted, 0, "{bad} must be rejected");
            assert!(matches!(
                report.rejected[0].1,
                TileRejection::ImplausibleElevation { .. }
            ));
        }
    }

    #[test]
    fn elevations_outside_earths_real_range_are_rejected() {
        let c = cache();
        // Everest is 8849 m; the Dead Sea shore is about -430 m. Anything well
        // outside that is corrupt data, and colliding against it would fling
        // the vehicle.
        for bad in [-20_000.0f32, 100_000.0] {
            let mut heights = flat(1655.0);
            heights[7] = bad;
            let report = c.insert_tiles(TILE_SIZE as u32, vec![(origin_coord(), heights, false)]);
            assert_eq!(report.accepted, 0, "{bad} m must be rejected");
        }
    }

    #[test]
    fn a_rejected_tile_leaves_previously_accepted_terrain_untouched() {
        let c = cache();
        c.insert_tiles(
            TILE_SIZE as u32,
            vec![(origin_coord(), flat(1655.0), false)],
        );
        let before = c.sample_ground_ned(0.0, 0.0);

        let mut poison = flat(1655.0);
        poison[0] = f32::NAN;
        let report = c.insert_tiles(TILE_SIZE as u32, vec![(origin_coord(), poison, false)]);

        assert_eq!(report.accepted, 0);
        assert_eq!(
            c.sample_ground_ned(0.0, 0.0),
            before,
            "a bad tile must not overwrite good terrain"
        );
    }

    #[test]
    fn a_mixed_batch_accepts_the_good_and_names_the_bad() {
        let c = cache();
        let good = origin_coord();
        let bad = TileCoord {
            x: good.x + 1,
            y: good.y,
            z: Z,
        };
        let report = c.insert_tiles(
            TILE_SIZE as u32,
            vec![(good, flat(1655.0), false), (bad, vec![0.0; 4], false)],
        );
        assert_eq!(report.accepted, 1);
        assert_eq!(report.rejected.len(), 1);
        assert_eq!(report.rejected[0].0, bad);
    }

    #[test]
    fn provenance_survives_ingestion() {
        // The "approximate terrain" warning is driven by this flag; losing it
        // would tell the user authoritative ground was under them when it was
        // not.
        let c = cache();
        c.insert_tiles(TILE_SIZE as u32, vec![(origin_coord(), flat(1655.0), true)]);
        assert!(c.any_approximate());
    }
}

#[cfg(test)]
mod streaming_lifecycle_tests {
    use super::*;

    const LAT: f64 = 40.015;
    const LON: f64 = -105.2705;
    const Z: u32 = 14;

    fn flat(v: f32) -> Vec<f32> {
        vec![v; TILE_SIZE * TILE_SIZE]
    }

    fn ring(centre: TileCoord, r: i64) -> Vec<(TileCoord, Vec<f32>, bool)> {
        let mut out = Vec::new();
        for dy in -r..=r {
            for dx in -r..=r {
                out.push((
                    TileCoord {
                        x: (centre.x as i64 + dx) as u32,
                        y: (centre.y as i64 + dy) as u32,
                        z: centre.z,
                    },
                    flat(1655.0),
                    false,
                ));
            }
        }
        out
    }

    fn cache(max_resident: usize) -> TerrainCache {
        let c = TerrainCache::with_capacity(max_resident);
        c.set_origin(LAT, LON, Some(1655.0));
        c
    }

    #[test]
    fn the_resident_bound_holds_across_a_long_traverse() {
        let c = cache(9);
        let start = TileCoord::from_lon_lat(LON, LAT, Z);

        // Fly north across many tiles, feeding a ring at each step — more
        // distinct tiles than the bound by a wide margin.
        for step in 0..12 {
            let north = step as f64 * 1_900.0; // ~one z14 tile at this latitude
            c.set_vehicle_ned(north, 0.0);
            let centre = TileCoord {
                x: start.x,
                y: start.y - step,
                z: Z,
            };
            c.insert_tiles(TILE_SIZE as u32, ring(centre, 1));
            assert!(
                c.resident_tiles() <= 9,
                "resident {} exceeded the bound at step {step}",
                c.resident_tiles()
            );
        }
    }

    #[test]
    fn the_tile_under_the_vehicle_is_never_the_one_evicted() {
        let c = cache(4);
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        c.set_vehicle_ned(0.0, 0.0);

        // Nine tiles into a cache that holds four: five must go, and the one
        // the vehicle is standing on must not be among them.
        let report = c.insert_tiles(TILE_SIZE as u32, ring(centre, 1));
        assert_eq!(report.accepted, 9);
        assert_eq!(report.evicted, 5);
        assert_eq!(c.resident_tiles(), 4);
        assert!(
            c.sample_ground_ned(0.0, 0.0).is_some(),
            "the ground under the vehicle was evicted"
        );
    }

    #[test]
    fn eviction_drops_the_furthest_tiles_first() {
        let c = cache(1);
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        c.set_vehicle_ned(0.0, 0.0);
        c.insert_tiles(TILE_SIZE as u32, ring(centre, 1));

        assert_eq!(c.resident_tiles(), 1);
        // The single survivor is the centre tile, so a sample at the vehicle
        // still resolves and a sample two tiles away does not.
        assert!(c.sample_ground_ned(0.0, 0.0).is_some());
        assert!(c.sample_ground_ned(4_000.0, 0.0).is_none());
    }

    #[test]
    fn a_flood_cannot_grow_memory_without_bound() {
        let c = cache(16);
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        // Twenty batches of 25 tiles: 500 submissions against a 16-tile bound.
        for _ in 0..20 {
            c.insert_tiles(TILE_SIZE as u32, ring(centre, 2));
            assert!(c.resident_tiles() <= 16);
        }
        assert!(c.resident_tiles() <= 16);
    }

    #[test]
    fn re_anchoring_discards_terrain_tied_to_the_old_origin() {
        let c = cache(64);
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        c.insert_tiles(TILE_SIZE as u32, ring(centre, 1));
        assert!(c.is_loaded());

        // A materially different place. Keeping the old tiles would compute
        // ground contact against terrain anchored somewhere else entirely.
        c.set_origin(51.5, -0.12, Some(35.0));
        assert!(!c.is_loaded());
        assert_eq!(c.resident_tiles(), 0);
        assert!(c.sample_ground_ned(0.0, 0.0).is_none());
    }

    #[test]
    fn re_anchoring_to_effectively_the_same_place_keeps_its_tiles() {
        // Origin reports jitter by centimetres. Throwing the whole set away on
        // every one of them would keep the physics permanently without ground.
        let c = cache(64);
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        c.insert_tiles(TILE_SIZE as u32, ring(centre, 1));
        let before = c.resident_tiles();

        c.set_origin(LAT + 0.000_001, LON, Some(1655.0));
        assert_eq!(c.resident_tiles(), before);
    }

    #[test]
    fn missing_coords_are_reported_around_the_vehicle() {
        let c = cache(64);
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        // Seed one tile so the cache knows which zoom to ask at.
        c.insert_tiles(TILE_SIZE as u32, vec![(centre, flat(1655.0), false)]);

        let missing = c.missing_around_vehicle(1);
        assert_eq!(
            missing.len(),
            8,
            "the centre is held, its 8 neighbours are not"
        );
        assert!(!missing.contains(&centre));
    }

    #[test]
    fn nothing_is_missing_once_the_ring_is_complete() {
        // The steady state, which is what keeps the exchange quiet.
        let c = cache(64);
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        c.insert_tiles(TILE_SIZE as u32, ring(centre, 1));
        assert!(c.missing_around_vehicle(1).is_empty());
    }

    #[test]
    fn moving_the_vehicle_makes_new_tiles_missing() {
        let c = cache(64);
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        c.insert_tiles(TILE_SIZE as u32, ring(centre, 1));
        assert!(c.missing_around_vehicle(1).is_empty());

        c.set_vehicle_ned(4_000.0, 0.0);
        assert!(
            !c.missing_around_vehicle(1).is_empty(),
            "the vehicle moved off the loaded ring and nothing was requested"
        );
    }

    #[test]
    fn nothing_is_requested_before_any_terrain_has_arrived() {
        // There is no zoom to ask at yet. The browser sends the first batch
        // unprompted after configuration.
        let c = cache(64);
        assert!(c.missing_around_vehicle(1).is_empty());
    }
}

#[cfg(test)]
mod ingress_does_not_stall_the_loop_tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    const LAT: f64 = 40.015;
    const LON: f64 = -105.2705;
    const Z: u32 = 14;

    /// The simulation loop reads ground at 400 Hz, i.e. every 2.5 ms. A flood of
    /// tiles arriving on the WebSocket must not stall it. The bar here is
    /// deliberately loose (25 ms, ten loop periods) so this measures "the
    /// ingress does not hold the lock across a whole batch" rather than
    /// machine speed — before validation was moved out of the write lock, a
    /// batch held it for the length of 1.6M float checks.
    const MAX_ACCEPTABLE_STALL: Duration = Duration::from_millis(25);

    #[test]
    fn a_reader_is_never_starved_by_a_flood_of_tiles() {
        let cache = Arc::new(TerrainCache::with_capacity(16));
        cache.set_origin(LAT, LON, Some(1655.0));
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);

        let stop = Arc::new(AtomicBool::new(false));
        let reader_cache = cache.clone();
        let reader_stop = stop.clone();

        let reader = std::thread::spawn(move || {
            let mut worst = Duration::ZERO;
            let mut reads: u64 = 0;
            while !reader_stop.load(Ordering::Relaxed) {
                let t0 = Instant::now();
                let _ = reader_cache.sample_ground_ned(0.0, 0.0);
                worst = worst.max(t0.elapsed());
                reads += 1;
                std::thread::sleep(Duration::from_micros(250));
            }
            (worst, reads)
        });

        for _ in 0..20 {
            let mut batch = Vec::new();
            for dy in -2i64..=2 {
                for dx in -2i64..=2 {
                    batch.push((
                        TileCoord {
                            x: (centre.x as i64 + dx) as u32,
                            y: (centre.y as i64 + dy) as u32,
                            z: Z,
                        },
                        vec![1655.0f32; TILE_SIZE * TILE_SIZE],
                        false,
                    ));
                }
            }
            cache.insert_tiles(TILE_SIZE as u32, batch);
        }

        stop.store(true, Ordering::Relaxed);
        let (worst, reads) = reader.join().expect("reader thread");

        assert!(reads > 0, "the reader never ran");
        assert!(
            worst < MAX_ACCEPTABLE_STALL,
            "a ground sample blocked for {worst:?} during ingestion (limit {MAX_ACCEPTABLE_STALL:?}); \
             the ingress is holding the write lock across validation"
        );
        assert!(
            cache.resident_tiles() <= 16,
            "the bound was breached under load"
        );
    }
}

#[cfg(test)]
mod parity_tests {
    use super::*;
    use std::collections::HashMap as Map;

    const LAT: f64 = 40.015;
    const LON: f64 = -105.2705;
    const Z: u32 = 14;

    fn sloped_tile() -> Vec<f32> {
        // A real gradient, so a disagreement between the two paths shows up as
        // a number rather than cancelling out on a flat surface.
        let mut h = vec![0.0f32; TILE_SIZE * TILE_SIZE];
        for row in 0..TILE_SIZE {
            for col in 0..TILE_SIZE {
                h[row * TILE_SIZE + col] = 1600.0 + row as f32 * 0.35 + col as f32 * 0.15;
            }
        }
        h
    }

    /// The viewer's `sampleMslAt`, written out independently.
    ///
    /// The point of a parity test is that two separate expressions of the same
    /// rule agree, so this mirrors the browser rather than calling the
    /// production path: pick the tile containing the point, then bilinearly
    /// blend the four surrounding posts *of that tile*. Clamping fx/fy to a
    /// single tile instead of selecting the neighbour is exactly the mistake
    /// this shape is written to avoid.
    fn viewer_msl(tiles: &Map<TileCoord, Vec<f32>>, lat: f64, lon: f64) -> Option<f64> {
        let coord = TileCoord::from_lon_lat(lon, lat, Z);
        let heights = tiles.get(&coord)?;
        let (nw_lon, nw_lat) = tile_to_lon_lat(coord.x, coord.y, Z);
        let (se_lon, se_lat) = tile_to_lon_lat(coord.x + 1, coord.y + 1, Z);
        let last = (TILE_SIZE - 1) as f64;

        let fx = ((lon - nw_lon) / (se_lon - nw_lon) * last).clamp(0.0, last);
        let fy = ((nw_lat - lat) / (nw_lat - se_lat) * last).clamp(0.0, last);
        let x0 = fx.floor() as usize;
        let y0 = fy.floor() as usize;
        let x1 = (x0 + 1).min(TILE_SIZE - 1);
        let y1 = (y0 + 1).min(TILE_SIZE - 1);
        let dx = fx - x0 as f64;
        let dy = fy - y0 as f64;
        let h = |r: usize, c: usize| heights[r * TILE_SIZE + c] as f64;
        Some(
            h(y0, x0) * (1.0 - dx) * (1.0 - dy)
                + h(y0, x1) * dx * (1.0 - dy)
                + h(y1, x0) * (1.0 - dx) * dy
                + h(y1, x1) * dx * dy,
        )
    }

    /// A 3x3 ring, and the same tiles in a plain map for the viewer reference.
    fn scene(approximate: bool) -> (TerrainCache, Map<TileCoord, Vec<f32>>, f64) {
        let heights = sloped_tile();
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        let mut map = Map::new();
        let mut batch = Vec::new();
        for dy in -1i64..=1 {
            for dx in -1i64..=1 {
                let coord = TileCoord {
                    x: (centre.x as i64 + dx) as u32,
                    y: (centre.y as i64 + dy) as u32,
                    z: Z,
                };
                map.insert(coord, heights.clone());
                batch.push((coord, heights.clone(), approximate));
            }
        }

        // The datum is what the browser sampled at the origin, exactly as it is
        // in production: an input to configuration, not something re-derived.
        let origin_elev = viewer_msl(&map, LAT, LON).expect("origin is covered");

        let cache = TerrainCache::new();
        cache.set_origin(LAT, LON, Some(origin_elev));
        let report = cache.insert_tiles(TILE_SIZE as u32, batch);
        assert_eq!(report.accepted, 9, "rejected: {:?}", report.rejected);
        (cache, map, origin_elev)
    }

    #[test]
    fn a_vehicle_at_rest_at_the_origin_is_on_the_ground_in_both() {
        // Ground contact and the drawn surface must put the origin at the same
        // height, or a vehicle sitting still reads as hovering in one of them.
        let (cache, _, _) = scene(false);
        let physics_ground = cache.sample_ground_ned(0.0, 0.0).expect("covered") as f64;
        // The viewer places local Y = 0 at the origin elevation by construction.
        assert!(
            physics_ground.abs() < 1e-3,
            "physics puts the origin {physics_ground} m off the viewer's ground plane"
        );
    }

    #[test]
    fn the_two_paths_agree_away_from_the_origin_on_a_slope() {
        let (cache, map, origin_elev) = scene(false);

        for (north, east) in [
            (0.0, 0.0),
            (120.0, 0.0),
            (0.0, 200.0),
            (-90.0, -150.0),
            (900.0, 900.0),
            (-1500.0, 1200.0),
        ] {
            let physics = cache.sample_ground_ned(north, east).expect("covered") as f64;
            let (lat, lon) = ned_to_lat_lon(north, east, LAT, LON);
            let viewer_local_y = viewer_msl(&map, lat, lon).expect("covered") - origin_elev;

            // ground_z (NED, down-positive) == -local_y (viewer, up-positive)
            assert!(
                (physics - (-viewer_local_y)).abs() < 1e-3,
                "at ({north}, {east}): physics {physics} vs viewer {viewer_local_y}"
            );
        }
    }

    #[test]
    fn a_fallback_sourced_tile_is_collided_against_not_ignored() {
        // The defect this replaces: the browser drew hills from the fallback
        // while the physics had nothing and used flat ground.
        let (cache, _, _) = scene(true);
        let ground = cache.sample_ground_ned(300.0, 300.0);
        assert!(
            ground.is_some(),
            "fallback terrain must be collided against"
        );
        assert!(
            ground.unwrap().abs() > 0.01,
            "a sloped fallback tile must not read as flat ground"
        );
        assert!(
            cache.any_approximate(),
            "provenance must survive to the label"
        );
    }

    #[test]
    fn provenance_changes_the_label_not_the_surface() {
        let (approx, _, _) = scene(true);
        let (auth, _, _) = scene(false);
        for (north, east) in [(0.0, 0.0), (150.0, -75.0), (-400.0, 600.0)] {
            assert_eq!(
                approx.sample_ground_ned(north, east),
                auth.sample_ground_ned(north, east),
                "provenance must not change the surface"
            );
        }
        assert!(approx.any_approximate());
        assert!(!auth.any_approximate());
    }

    #[test]
    fn with_no_terrain_at_all_neither_side_invents_a_surface() {
        let cache = TerrainCache::new();
        cache.set_origin(LAT, LON, None);
        // Unknown, not flat: the caller decides, and both sides get the same
        // "no data" answer rather than one of them making something up.
        assert!(cache.sample_ground_ned(0.0, 0.0).is_none());
        assert!(!cache.is_loaded());
    }
}
