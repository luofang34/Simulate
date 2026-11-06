//! Resident DEM surface elevation queries on [`HeadlessMap`] for conformal SVS symbology
//! and DO-315B onboard runway-profile fusion.

use maplibre::{
    coords::WorldTileCoords,
    headless::{TerrainPatchSnapshot, map::HeadlessMap},
};
use sim_geo::Geodetic;
use sim_scenery::project_to_tile;

use crate::tile_id_to_world_coords;

/// Finest Web Mercator zoom queried against resident SVS DEM tiles (`belief_real_svs_style`
/// streams Terrarium up to `z = 14`, and `HeadlessMap::resident_terrain_patch` automatically
/// falls back to any loaded ancestor down to `z = 0`).
const SVS_DEM_QUERY_ZOOM: u8 = 14;

/// Per-frame cached sampler over [`HeadlessMap::resident_terrain_patch`] so 50+ runway
/// stations across 1–3 tiles decode stitched `129 × 129` patches at most once per tile.
pub(crate) struct ResidentDemSampler<'a> {
    map: &'a HeadlessMap,
    cache: Vec<(WorldTileCoords, Option<TerrainPatchSnapshot>)>,
}

impl<'a> ResidentDemSampler<'a> {
    pub(crate) fn new(map: &'a HeadlessMap) -> Self {
        Self {
            map,
            cache: Vec::with_capacity(4),
        }
    }

    /// Samples `(bilinear_msl_m, cell_max_msl_m)` in unexaggerated orthometric metres MSL
    /// at `(lat_deg, lon_deg)`. Returns `None` outside Web Mercator (`|lat| > 85.0511°`)
    /// or when no DEM tile covering the point is resident yet.
    pub(crate) fn sample_msl_and_cell_max(
        &mut self,
        lat_deg: f64,
        lon_deg: f64,
    ) -> Option<(f64, f64)> {
        if !lat_deg.is_finite() || !lon_deg.is_finite() {
            return None;
        }
        let coord = project_to_tile(
            &Geodetic::from_degrees(lat_deg, lon_deg, 0.0),
            SVS_DEM_QUERY_ZOOM,
        )?;
        let world_coords = tile_id_to_world_coords(coord.tile)?;
        let snapshot = self.patch_for(world_coords)?;
        sample_patch_msl_and_max(snapshot, coord.u, coord.v)
    }

    fn patch_for(&mut self, coords: WorldTileCoords) -> Option<&TerrainPatchSnapshot> {
        if let Some(idx) = self.cache.iter().position(|(c, _)| *c == coords) {
            return self.cache.get(idx).and_then(|(_, p)| p.as_ref());
        }
        let fetched = self.map.resident_terrain_patch(coords);
        self.cache.push((coords, fetched));
        self.cache.last().and_then(|(_, p)| p.as_ref())
    }
}

/// One-shot bilinear MSL sample at `(lat_deg, lon_deg)` from `map`'s resident DEM.
pub(crate) fn sample_point_msl(map: &HeadlessMap, lat_deg: f64, lon_deg: f64) -> Option<f64> {
    if !lat_deg.is_finite() || !lon_deg.is_finite() {
        return None;
    }
    let coord = project_to_tile(
        &Geodetic::from_degrees(lat_deg, lon_deg, 0.0),
        SVS_DEM_QUERY_ZOOM,
    )?;
    let world_coords = tile_id_to_world_coords(coord.tile)?;
    let snapshot = map.resident_terrain_patch(world_coords)?;
    sample_patch_msl_and_max(&snapshot, coord.u, coord.v).map(|(bil, _)| bil)
}

fn sample_patch_msl_and_max(snapshot: &TerrainPatchSnapshot, u: f64, v: f64) -> Option<(f64, f64)> {
    if snapshot.grid_side < 2 {
        return None;
    }
    let max_idx = snapshot.grid_side.saturating_sub(1);
    let gx = (u.clamp(0.0, 1.0) * f64::from(max_idx)) as f32;
    let gy = (v.clamp(0.0, 1.0) * f64::from(max_idx)) as f32;
    let x0 = (gx.floor().max(0.0) as u32).min(max_idx);
    let y0 = (gy.floor().max(0.0) as u32).min(max_idx);
    let x1 = x0.saturating_add(1).min(max_idx);
    let y1 = y0.saturating_add(1).min(max_idx);
    let tx = (gx - (x0 as f32)).clamp(0.0, 1.0);
    let ty = (gy - (y0 as f32)).clamp(0.0, 1.0);

    let h00 = snapshot.height_at_grid(x0, y0)?;
    let h10 = snapshot.height_at_grid(x1, y0)?;
    let h01 = snapshot.height_at_grid(x0, y1)?;
    let h11 = snapshot.height_at_grid(x1, y1)?;
    let exag = f64::from(snapshot.exaggeration.abs().max(1e-6));
    let top = h00 + (h10 - h00) * tx;
    let bot = h01 + (h11 - h01) * tx;
    let bil_msl = f64::from(top + (bot - top) * ty) / exag;
    let cell_max_msl = f64::from(h00.max(h10).max(h01).max(h11)) / exag;
    Some((bil_msl, cell_max_msl))
}
