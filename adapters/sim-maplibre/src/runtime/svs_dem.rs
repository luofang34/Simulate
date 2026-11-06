//! Zero-allocation resident DEM surface elevation queries on [`HeadlessMap`] for conformal
//! SVS symbology and DO-315B onboard runway-profile fusion.

use maplibre::headless::map::HeadlessMap;
use sim_geo::Geodetic;
use sim_scenery::project_to_tile;

use crate::tile_id_to_world_coords;

/// Finest Web Mercator zoom queried against resident SVS DEM tiles (`belief_real_svs_style`
/// streams Terrarium up to `z = 14`, and [`HeadlessMap::sample_resident_dem_grid_cell`]
/// automatically falls back to any loaded ancestor down to `z = 0`).
const SVS_DEM_QUERY_ZOOM: u8 = 14;

/// Zero-allocation sampler over [`HeadlessMap::sample_resident_dem_grid_cell`], evaluating
/// only the `<= 4` stitched grid vertices surrounding each queried station.
pub(crate) struct ResidentDemSampler<'a> {
    map: &'a HeadlessMap,
}

impl<'a> ResidentDemSampler<'a> {
    pub(crate) const fn new(map: &'a HeadlessMap) -> Self {
        Self { map }
    }

    /// Samples `(bilinear_msl_m, cell_max_msl_m)` in unexaggerated orthometric metres MSL
    /// at `(lat_deg, lon_deg)`. Returns `None` outside Web Mercator (`|lat| > 85.0511°`)
    /// or when no DEM tile covering the point is resident yet.
    pub(crate) fn sample_msl_and_cell_max(
        &mut self,
        lat_deg: f64,
        lon_deg: f64,
    ) -> Option<(f64, f64)> {
        sample_cell_msl_and_max(self.map, lat_deg, lon_deg)
    }
}

/// One-shot bilinear MSL sample at `(lat_deg, lon_deg)` from `map`'s resident DEM.
pub(crate) fn sample_point_msl(map: &HeadlessMap, lat_deg: f64, lon_deg: f64) -> Option<f64> {
    sample_cell_msl_and_max(map, lat_deg, lon_deg).map(|(bil, _)| bil)
}

fn sample_cell_msl_and_max(map: &HeadlessMap, lat_deg: f64, lon_deg: f64) -> Option<(f64, f64)> {
    if !lat_deg.is_finite() || !lon_deg.is_finite() {
        return None;
    }
    let coord = project_to_tile(
        &Geodetic::from_degrees(lat_deg, lon_deg, 0.0),
        SVS_DEM_QUERY_ZOOM,
    )?;
    let world_coords = tile_id_to_world_coords(coord.tile)?;
    map.sample_resident_dem_grid_cell(world_coords, coord.u, coord.v)
}
