//! Loading terrain from a local tile pack, for runs with no browser attached.
//!
//! Headless and CI runs still need ground to collide against, but reintroducing
//! a network fetcher here would recreate the very thing this design removes: a
//! second party resolving tiles independently of the viewer, free to disagree
//! with it. Reading local files does not, because there is no viewer to
//! disagree with — and the tiles go through the same validated ingress the
//! browser's do, so there is one way terrain enters the physics.

use std::path::{Path, PathBuf};
use terrain::{TerrainCache, TileCoord, TILE_SIZE};
use tracing::{debug, info, warn};

/// Directory layout `{z}/{x}/{y}.bin`, matching the tile store and the baker.
///
/// Returns the number of tiles accepted. Zero is not an error at this level —
/// the caller decides whether a run without terrain should proceed — but it is
/// always reported, because silently flying on flat ground is exactly the
/// failure this work exists to remove.
pub fn load_pack(cache: &TerrainCache, dir: &Path) -> std::io::Result<usize> {
    let mut tiles: Vec<(TileCoord, Vec<f32>, bool)> = Vec::new();

    for z_entry in read_numeric_dir(dir)? {
        let (z, z_path) = z_entry;
        for x_entry in read_numeric_dir(&z_path)? {
            let (x, x_path) = x_entry;
            for file in std::fs::read_dir(&x_path)? {
                let path = file?.path();
                let Some(y) = tile_index_from_bin(&path) else {
                    continue;
                };
                match read_tile(&path) {
                    Ok(heights) => tiles.push((TileCoord { x, y, z }, heights, false)),
                    Err(e) => warn!(path = %path.display(), error = %e, "Skipping unreadable tile"),
                }
            }
        }
    }

    if tiles.is_empty() {
        warn!(dir = %dir.display(), "Terrain pack contains no tiles");
        return Ok(0);
    }

    let found = tiles.len();
    let report = cache.insert_tiles(TILE_SIZE as u32, tiles);
    if !report.rejected.is_empty() {
        warn!(
            rejected = report.rejected.len(),
            "Some tiles in the pack were rejected; see the per-tile reasons above"
        );
    }
    info!(
        dir = %dir.display(),
        found,
        accepted = report.accepted,
        "Terrain pack loaded"
    );
    Ok(report.accepted)
}

fn read_numeric_dir(dir: &Path) -> std::io::Result<Vec<(u32, PathBuf)>> {
    let mut out = Vec::new();
    for entry in std::fs::read_dir(dir)? {
        let path = entry?.path();
        if !path.is_dir() {
            continue;
        }
        match path
            .file_name()
            .and_then(|n| n.to_str())
            .and_then(|n| n.parse::<u32>().ok())
        {
            Some(n) => out.push((n, path)),
            None => debug!(path = %path.display(), "Ignoring non-numeric directory in tile pack"),
        }
    }
    Ok(out)
}

fn tile_index_from_bin(path: &Path) -> Option<u32> {
    if path.extension()?.to_str()? != "bin" {
        return None;
    }
    path.file_stem()?.to_str()?.parse().ok()
}

/// Read one `{y}.bin`: `TILE_SIZE^2` f32 little-endian, row-major from the
/// tile's north-west corner, in MSL metres.
fn read_tile(path: &Path) -> std::io::Result<Vec<f32>> {
    let bytes = std::fs::read(path)?;
    // A short or ragged file is reported here rather than being padded into
    // plausible-looking ground. The ingress would reject it anyway; failing at
    // the read keeps the reason specific.
    if bytes.len() % 4 != 0 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("{} bytes is not a whole number of f32 samples", bytes.len()),
        ));
    }
    Ok(bytes
        .chunks_exact(4)
        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    const LAT: f64 = 40.015;
    const LON: f64 = -105.2705;
    const Z: u32 = 14;

    fn write_tile(root: &Path, coord: TileCoord, value: f32) {
        let dir = root.join(coord.z.to_string()).join(coord.x.to_string());
        std::fs::create_dir_all(&dir).unwrap();
        let mut bytes = Vec::new();
        for _ in 0..TILE_SIZE * TILE_SIZE {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        std::fs::write(dir.join(format!("{}.bin", coord.y)), bytes).unwrap();
    }

    fn temp_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("hitl-terrain-pack-{name}"));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn a_pack_loads_through_the_same_ingress_as_the_browser() {
        let dir = temp_dir("basic");
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        write_tile(&dir, centre, 1655.0);

        let cache = TerrainCache::new();
        cache.set_origin(LAT, LON, Some(1655.0));
        assert_eq!(load_pack(&cache, &dir).unwrap(), 1);
        assert!(cache.is_loaded());
        // Ground at the origin sits on the datum, same as the browser-fed case.
        // Compared with a tolerance rather than exactly: the sample runs through
        // an f32 grid and an f64 bilinear blend, which leaves sub-picometre
        // residue that means nothing physically.
        let ground = cache.sample_ground_ned(0.0, 0.0).expect("covered");
        assert!(
            ground.abs() < 1e-6,
            "ground at origin was {ground} m off the datum"
        );
    }

    #[test]
    fn a_corrupt_tile_is_skipped_without_failing_the_pack() {
        let dir = temp_dir("corrupt");
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        write_tile(&dir, centre, 1655.0);
        // A ragged neighbour must not take the good tile down with it.
        let bad_dir = dir.join(Z.to_string()).join((centre.x + 1).to_string());
        std::fs::create_dir_all(&bad_dir).unwrap();
        std::fs::write(bad_dir.join(format!("{}.bin", centre.y)), [1u8, 2, 3]).unwrap();

        let cache = TerrainCache::new();
        cache.set_origin(LAT, LON, Some(1655.0));
        assert_eq!(load_pack(&cache, &dir).unwrap(), 1);
    }

    #[test]
    fn an_empty_pack_reports_zero_rather_than_pretending_to_have_terrain() {
        let dir = temp_dir("empty");
        let cache = TerrainCache::new();
        cache.set_origin(LAT, LON, Some(1655.0));
        assert_eq!(load_pack(&cache, &dir).unwrap(), 0);
        assert!(!cache.is_loaded());
    }

    #[test]
    fn non_numeric_directories_are_ignored() {
        let dir = temp_dir("junk");
        let centre = TileCoord::from_lon_lat(LON, LAT, Z);
        write_tile(&dir, centre, 1655.0);
        std::fs::create_dir_all(dir.join(".git")).unwrap();

        let cache = TerrainCache::new();
        cache.set_origin(LAT, LON, Some(1655.0));
        assert_eq!(load_pack(&cache, &dir).unwrap(), 1);
    }
}
