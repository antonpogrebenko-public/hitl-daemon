//! Terrain sampling benchmarks.
//!
//! The 400 Hz loop asks this cache for a ground height and a surface normal
//! on every tick. The daemon's per-phase tick breakdown reports the terrain
//! phase at 72-80 ns with nothing resident — that figure is the `is_loaded`
//! check and the vehicle-position write, and it says nothing about what the
//! sampler costs once tiles are actually loaded, which is the state the
//! vehicle is in for a real flight.
//!
//! This measures that state. `ground_and_normal` is the one that matters: it
//! is what a tick does, and it resolves the vehicle's coordinate to a tile
//! six separate times behind eight lock acquisitions.
//!
//! Run: cargo bench -p terrain

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use terrain::{TerrainCache, TileCoord, TILE_SIZE};

const LAT: f64 = 40.015;
const LON: f64 = -105.2705;
const ZOOM: u32 = 14;

/// A cache holding a square block of tiles centred on the origin.
///
/// Heights are a deterministic slope rather than real elevation data — the
/// sampler's cost is in the coordinate resolution and the locking, not in the
/// values, and a slope keeps the normal computation off the degenerate
/// all-equal path.
fn cache_with(radius: i32) -> TerrainCache {
    let center = TileCoord::from_lon_lat(LON, LAT, ZOOM);
    let mut tiles = Vec::new();

    for dy in -radius..=radius {
        for dx in -radius..=radius {
            let coord = TileCoord {
                x: (center.x as i32 + dx) as u32,
                y: (center.y as i32 + dy) as u32,
                z: ZOOM,
            };
            let mut heights = vec![0.0f32; TILE_SIZE * TILE_SIZE];
            for row in 0..TILE_SIZE {
                for col in 0..TILE_SIZE {
                    heights[row * TILE_SIZE + col] = 1655.0 + (row as f32) * 0.5 + (col as f32) * 0.25;
                }
            }
            tiles.push((coord, heights, false));
        }
    }

    let cache = TerrainCache::new();
    cache.set_origin(LAT, LON, Some(1655.0));
    let report = cache.insert_tiles(TILE_SIZE as u32, tiles);
    assert!(
        report.rejected.is_empty(),
        "bench fixture rejected: {:?}",
        report.rejected
    );
    assert!(cache.is_loaded());
    cache
}

fn bench_sample_ground(c: &mut Criterion) {
    let cache = cache_with(1);
    c.bench_function("sample_ground_ned/9_tiles", |b| {
        b.iter(|| black_box(&cache).sample_ground_ned(black_box(12.0), black_box(-8.0)))
    });
}

fn bench_sample_normal(c: &mut Criterion) {
    let cache = cache_with(1);
    c.bench_function("sample_ground_normal_ned/9_tiles", |b| {
        b.iter(|| black_box(&cache).sample_ground_normal_ned(black_box(12.0), black_box(-8.0)))
    });
}

/// What one tick actually does.
///
/// `set_vehicle_ned` takes the write lock; the two samplers take shared locks
/// and resolve the coordinate independently. Compare this against the
/// daemon's reported `terrain_ns` to see the difference terrain residency
/// makes to the tick.
fn bench_tick(c: &mut Criterion) {
    let cache = cache_with(1);
    c.bench_function("ground_and_normal/9_tiles", |b| {
        b.iter(|| {
            cache.set_vehicle_ned(black_box(12.0), black_box(-8.0));
            let z = cache.sample_ground_ned(black_box(12.0), black_box(-8.0));
            let n = cache.sample_ground_normal_ned(black_box(12.0), black_box(-8.0));
            black_box((z, n))
        })
    });
}

/// At the residency cap, to show whether tile lookup scales with the number
/// of resident tiles or is flat in it.
fn bench_tick_full_cache(c: &mut Criterion) {
    let cache = cache_with(3); // 49 tiles, under DEFAULT_MAX_RESIDENT_TILES
    c.bench_function("ground_and_normal/49_tiles", |b| {
        b.iter(|| {
            cache.set_vehicle_ned(black_box(12.0), black_box(-8.0));
            let z = cache.sample_ground_ned(black_box(12.0), black_box(-8.0));
            let n = cache.sample_ground_normal_ned(black_box(12.0), black_box(-8.0));
            black_box((z, n))
        })
    });
}

criterion_group!(
    benches,
    bench_sample_ground,
    bench_sample_normal,
    bench_tick,
    bench_tick_full_cache
);
criterion_main!(benches);
