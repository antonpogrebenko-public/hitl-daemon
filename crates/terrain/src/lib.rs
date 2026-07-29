use parking_lot::RwLock as SyncRwLock;
use serde::Deserialize;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{debug, info, warn};

/// Samples per tile edge. Part of the public tile contract: callers of
/// [`TerrainCache::load_from_tiles`] must supply `TILE_SIZE * TILE_SIZE` heights.
pub const TILE_SIZE: usize = 256;
const R_EARTH: f64 = 6378137.0;

#[derive(Debug, Clone, Deserialize)]
pub struct TileMeta {
    #[serde(rename = "schemaVersion")]
    pub schema_version: u32,
    pub provider: String,
    pub zoom: u32,
    #[serde(rename = "tileSize")]
    pub tile_size: u32,
    pub bbox: BBox,
    pub elevation: ElevationMeta,
}

#[derive(Debug, Clone, Deserialize)]
pub struct BBox {
    pub west: f64,
    pub south: f64,
    pub east: f64,
    pub north: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ElevationMeta {
    pub units: String,
    pub datum: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TileCoord {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

impl TileCoord {
    pub fn from_lon_lat(lon: f64, lat: f64, z: u32) -> Self {
        let n = 2_u32.pow(z) as f64;
        let x = ((lon + 180.0) / 360.0 * n).floor() as u32;
        let lat_rad = lat.to_radians();
        let y = ((1.0 - (lat_rad.tan() + 1.0 / lat_rad.cos()).ln() / std::f64::consts::PI) / 2.0
            * n)
            .floor() as u32;
        Self { x, y, z }
    }
}

struct LoadedTile {
    heights: Vec<f32>,
}

pub struct TerrainProvider {
    base_url: String,
    meta: RwLock<Option<TileMeta>>,
    cache: RwLock<HashMap<TileCoord, Arc<LoadedTile>>>,
    client: reqwest::Client,
    max_cache_size: usize,
}

impl TerrainProvider {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url: base_url.trim_end_matches('/').to_string(),
            meta: RwLock::new(None),
            cache: RwLock::new(HashMap::new()),
            client: reqwest::Client::new(),
            max_cache_size: 16,
        }
    }

    pub async fn ensure_meta(&self) -> Option<TileMeta> {
        {
            let meta = self.meta.read().await;
            if meta.is_some() {
                return meta.clone();
            }
        }

        let url = format!("{}/meta.json", self.base_url);
        match self.client.get(&url).send().await {
            Ok(resp) if resp.status().is_success() => match resp.json::<TileMeta>().await {
                Ok(m) => {
                    debug!("Loaded terrain meta: zoom={}, bbox={:?}", m.zoom, m.bbox);
                    let mut meta = self.meta.write().await;
                    *meta = Some(m.clone());
                    Some(m)
                }
                Err(e) => {
                    warn!("Failed to parse terrain meta: {}", e);
                    None
                }
            },
            Ok(resp) => {
                warn!("Failed to fetch terrain meta: HTTP {}", resp.status());
                None
            }
            Err(e) => {
                warn!("Failed to fetch terrain meta: {}", e);
                None
            }
        }
    }

    pub async fn ensure_tiles_loaded(&self, lat: f64, lon: f64) {
        let meta = match self.ensure_meta().await {
            Some(m) => m,
            None => return,
        };

        let center = TileCoord::from_lon_lat(lon, lat, meta.zoom);

        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                let coord = TileCoord {
                    x: (center.x as i32 + dx) as u32,
                    y: (center.y as i32 + dy) as u32,
                    z: meta.zoom,
                };
                self.load_tile(coord).await;
            }
        }
    }

    async fn load_tile(&self, coord: TileCoord) -> Option<Arc<LoadedTile>> {
        {
            let cache = self.cache.read().await;
            if let Some(tile) = cache.get(&coord) {
                return Some(Arc::clone(tile));
            }
        }

        let url = format!("{}/{}/{}/{}.bin", self.base_url, coord.z, coord.x, coord.y);
        let resp = match self.client.get(&url).send().await {
            Ok(r) if r.status().is_success() => r,
            Ok(r) => {
                debug!("Tile not found: {} (HTTP {})", url, r.status());
                return None;
            }
            Err(e) => {
                warn!("Failed to fetch tile {}: {}", url, e);
                return None;
            }
        };

        let bytes = match resp.bytes().await {
            Ok(b) => b,
            Err(e) => {
                warn!("Failed to read tile bytes: {}", e);
                return None;
            }
        };

        if bytes.len() != TILE_SIZE * TILE_SIZE * 4 {
            warn!(
                "Invalid tile size: expected {}, got {}",
                TILE_SIZE * TILE_SIZE * 4,
                bytes.len()
            );
            return None;
        }

        let heights: Vec<f32> = bytes
            .chunks_exact(4)
            .map(|chunk| f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
            .collect();

        let tile = Arc::new(LoadedTile { heights });

        {
            let mut cache = self.cache.write().await;
            cache.insert(coord, Arc::clone(&tile));
            self.evict_old_tiles(&mut cache);
        }

        debug!("Loaded terrain tile {}/{}/{}", coord.z, coord.x, coord.y);
        Some(tile)
    }

    fn evict_old_tiles(&self, cache: &mut HashMap<TileCoord, Arc<LoadedTile>>) {
        while cache.len() > self.max_cache_size {
            if let Some(key) = cache.keys().next().cloned() {
                cache.remove(&key);
            }
        }
    }

    pub async fn sample_elevation(&self, lat: f64, lon: f64) -> Option<f32> {
        let meta = {
            let m = self.meta.read().await;
            m.clone()?
        };

        let coord = TileCoord::from_lon_lat(lon, lat, meta.zoom);

        let tile = {
            let cache = self.cache.read().await;
            cache.get(&coord).cloned()
        }?;

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

        let h00 = tile.heights[y0 * TILE_SIZE + x0] as f64;
        let h10 = tile.heights[y0 * TILE_SIZE + x1] as f64;
        let h01 = tile.heights[y1 * TILE_SIZE + x0] as f64;
        let h11 = tile.heights[y1 * TILE_SIZE + x1] as f64;

        let elevation = h00 * (1.0 - dx) * (1.0 - dy)
            + h10 * dx * (1.0 - dy)
            + h01 * (1.0 - dx) * dy
            + h11 * dx * dy;

        Some(elevation as f32)
    }

    pub async fn sample_elevation_ned(
        &self,
        lat: f64,
        lon: f64,
        reference_alt: f64,
    ) -> Option<f32> {
        let msl_elevation = self.sample_elevation(lat, lon).await?;
        let ned_down = reference_alt - msl_elevation as f64;
        Some(ned_down as f32)
    }
}

fn tile_to_lon_lat(x: u32, y: u32, z: u32) -> (f64, f64) {
    let n = 2_u32.pow(z) as f64;
    let lon = x as f64 / n * 360.0 - 180.0;
    let lat_rad = (std::f64::consts::PI * (1.0 - 2.0 * y as f64 / n))
        .sinh()
        .atan();
    let lat = lat_rad.to_degrees();
    (lon, lat)
}

/// Sync terrain cache for use in physics loop.
/// Populated async at startup, queried sync during simulation.
pub struct TerrainCache {
    inner: SyncRwLock<TerrainCacheInner>,
}

struct TerrainCacheInner {
    meta: Option<TileMeta>,
    tiles: HashMap<TileCoord, Vec<f32>>,
    origin_lat: f64,
    origin_lon: f64,
    reference_alt: f64,
    origin_elevation: Option<f64>,
}

impl TerrainCache {
    pub fn new() -> Self {
        Self {
            inner: SyncRwLock::new(TerrainCacheInner {
                meta: None,
                tiles: HashMap::new(),
                origin_lat: 0.0,
                origin_lon: 0.0,
                reference_alt: 0.0,
                origin_elevation: None,
            }),
        }
    }

    /// Load terrain tiles around origin. Call from async context at startup.
    pub async fn load(&self, base_url: &str, lat: f64, lon: f64, reference_alt: f64) -> bool {
        let client = reqwest::Client::new();
        let base_url = base_url.trim_end_matches('/');

        let meta_url = format!("{}/meta.json", base_url);
        let meta: TileMeta = match client.get(&meta_url).send().await {
            Ok(resp) if resp.status().is_success() => match resp.json().await {
                Ok(m) => m,
                Err(e) => {
                    warn!("Failed to parse terrain meta: {}", e);
                    return false;
                }
            },
            Ok(resp) => {
                warn!("Failed to fetch terrain meta: HTTP {}", resp.status());
                return false;
            }
            Err(e) => {
                warn!("Failed to fetch terrain meta: {}", e);
                return false;
            }
        };

        let center = TileCoord::from_lon_lat(lon, lat, meta.zoom);
        let mut tiles = HashMap::new();

        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                let coord = TileCoord {
                    x: (center.x as i32 + dx) as u32,
                    y: (center.y as i32 + dy) as u32,
                    z: meta.zoom,
                };

                let url = format!("{}/{}/{}/{}.bin", base_url, coord.z, coord.x, coord.y);
                match client.get(&url).send().await {
                    Ok(resp) if resp.status().is_success() => {
                        if let Ok(bytes) = resp.bytes().await {
                            if bytes.len() == TILE_SIZE * TILE_SIZE * 4 {
                                let heights: Vec<f32> = bytes
                                    .chunks_exact(4)
                                    .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                                    .collect();
                                tiles.insert(coord, heights);
                                debug!("Loaded terrain tile {}/{}/{}", coord.z, coord.x, coord.y);
                            }
                        }
                    }
                    _ => {
                        debug!("Tile not available: {}/{}/{}", coord.z, coord.x, coord.y);
                    }
                }
            }
        }

        self.load_from_tiles(meta, tiles, lat, lon, reference_alt)
    }

    /// Populate the cache from already-decoded height tiles, bypassing HTTP.
    ///
    /// `tiles` maps tile coordinates to `TILE_SIZE * TILE_SIZE` f32 samples in
    /// MSL metres, row-major from the tile's north-west corner. Shared by
    /// [`TerrainCache::load`] and by callers that source tiles from disk, a
    /// bundle, or a test fixture. Returns whether any tile was stored.
    pub fn load_from_tiles(
        &self,
        meta: TileMeta,
        tiles: HashMap<TileCoord, Vec<f32>>,
        lat: f64,
        lon: f64,
        reference_alt: f64,
    ) -> bool {
        let tile_count = tiles.len();

        let origin_elevation = Self::sample_elevation_raw(&tiles, &meta, lat, lon);

        {
            let mut inner = self.inner.write();
            inner.meta = Some(meta);
            inner.tiles = tiles;
            inner.origin_lat = lat;
            inner.origin_lon = lon;
            inner.reference_alt = reference_alt;
            inner.origin_elevation = origin_elevation;
        }

        if let Some(elev) = origin_elevation {
            info!(
                "Terrain cache loaded: {} tiles around ({}, {}), origin elevation: {:.1}m MSL",
                tile_count, lat, lon, elev
            );
        } else {
            warn!(
                "Terrain cache loaded: {} tiles around ({}, {}) but origin outside tile coverage",
                tile_count, lat, lon
            );
        }

        tile_count > 0
    }

    fn sample_elevation_raw(
        tiles: &HashMap<TileCoord, Vec<f32>>,
        meta: &TileMeta,
        lat: f64,
        lon: f64,
    ) -> Option<f64> {
        let coord = TileCoord::from_lon_lat(lon, lat, meta.zoom);
        let heights = tiles.get(&coord)?;

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

        let h00 = heights[y0 * TILE_SIZE + x0] as f64;
        let h10 = heights[y0 * TILE_SIZE + x1] as f64;
        let h01 = heights[y1 * TILE_SIZE + x0] as f64;
        let h11 = heights[y1 * TILE_SIZE + x1] as f64;

        let elevation = h00 * (1.0 - dx) * (1.0 - dy)
            + h10 * dx * (1.0 - dy)
            + h01 * (1.0 - dx) * dy
            + h11 * dx * dy;

        Some(elevation)
    }

    /// Sample ground elevation in NED coordinates relative to origin terrain.
    /// Returns the NED "down" coordinate of the ground at (north, east).
    /// Positive = below origin ground level, negative = above.
    /// Uses origin_elevation (terrain at lat/lon origin) as vertical reference
    /// so ground_z=0 at origin, matching the frontend 3D mesh (Y=0 at origin).
    /// Returns None if terrain not loaded or position outside cached tiles.
    /// Call from sync physics loop.
    pub fn sample_ground_ned(&self, north: f64, east: f64) -> Option<f32> {
        let inner = self.inner.read();
        let meta = inner.meta.as_ref()?;
        let origin_elev = inner.origin_elevation?;

        let (lat, lon) = ned_to_lat_lon(north, east, inner.origin_lat, inner.origin_lon);

        let coord = TileCoord::from_lon_lat(lon, lat, meta.zoom);
        let heights = inner.tiles.get(&coord)?;

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

        let h00 = heights[y0 * TILE_SIZE + x0] as f64;
        let h10 = heights[y0 * TILE_SIZE + x1] as f64;
        let h01 = heights[y1 * TILE_SIZE + x0] as f64;
        let h11 = heights[y1 * TILE_SIZE + x1] as f64;

        let msl_elevation = h00 * (1.0 - dx) * (1.0 - dy)
            + h10 * dx * (1.0 - dy)
            + h01 * (1.0 - dx) * dy
            + h11 * dx * dy;

        // Ground in NED: how far below origin ground level is this point.
        // origin_elev - msl_elevation: positive if point is lower than origin,
        // negative if point is higher (e.g., a hill).
        // At origin (north=0, east=0), this returns 0.0 (ground = NED zero).
        let ned_ground = origin_elev - msl_elevation;
        Some(ned_ground as f32)
    }

    /// Check if terrain is loaded
    pub fn is_loaded(&self) -> bool {
        let inner = self.inner.read();
        inner.meta.is_some() && !inner.tiles.is_empty()
    }

    /// Get the MSL elevation at the origin point.
    /// Returns None if terrain not loaded or origin outside tile coverage.
    pub fn origin_elevation_msl(&self) -> Option<f64> {
        self.inner.read().origin_elevation
    }
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

    fn test_meta() -> TileMeta {
        TileMeta {
            schema_version: 1,
            provider: "test".to_string(),
            zoom: TEST_ZOOM,
            tile_size: TILE_SIZE as u32,
            bbox: BBox {
                west: -180.0,
                south: -85.0,
                east: 180.0,
                north: 85.0,
            },
            elevation: ElevationMeta {
                units: "meters".to_string(),
                datum: "test".to_string(),
            },
        }
    }

    /// Build a cache holding the 3x3 tile block around (TEST_LAT, TEST_LON) —
    /// the same footprint the real HTTP loader fetches. `height_at(lat, lon)`
    /// supplies MSL metres, so tests can express real-world gradients without
    /// caring which tile a sample lands in.
    fn cache_with_tiles(height_at: impl Fn(f64, f64) -> f32) -> TerrainCache {
        let center = TileCoord::from_lon_lat(TEST_LON, TEST_LAT, TEST_ZOOM);
        let mut tiles = HashMap::new();

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
                tiles.insert(coord, heights);
            }
        }

        let cache = TerrainCache::new();
        assert!(cache.load_from_tiles(test_meta(), tiles, TEST_LAT, TEST_LON, 0.0));
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
