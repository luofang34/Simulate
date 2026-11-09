//! Zero-allocation resident DEM surface elevation queries and DO-315B eye-clearance
//! envelope computation on [`HeadlessMap`] for conformal SVS rendering.

use maplibre::headless::map::HeadlessMap;
use sim_geo::Geodetic;
use sim_scenery::project_to_tile;

use crate::{ConformalFrameStamp, geodesy::wrap_lon_deg, tile_id_to_world_coords};

/// Finest Web Mercator zoom queried against resident SVS DEM tiles (`belief_real_svs_style`
/// streams Terrarium up to `z = 14`, and [`HeadlessMap::sample_resident_dem_grid_cell`]
/// automatically falls back to any loaded ancestor down to `z = 0`).
const SVS_DEM_QUERY_ZOOM: u8 = 14;

/// Minimum vertical clearance (metres) kept between a conformal SVS camera eye and the
/// local rendered DEM surface, preventing subterranean sky/skirt inversion when lateral
/// GNSS error or unflattened DSM canopy rises above sensed wheel/eyepoint height.
const MIN_SVS_EYE_CLEARANCE_M: f64 = 1.0;

/// Minimum horizontal sampling radius (metres) around the SVS anchor used to bound local
/// triangle chords and near-plane corners.
const MIN_FOOTPRINT_RADIUS_M: f64 = 2.0;

/// Zero-allocation sampler over [`HeadlessMap::sample_resident_dem_grid_cell`], evaluating
/// only the `<= 4` stitched grid vertices surrounding each queried station.
pub(crate) struct ResidentDemSampler<'a> {
    map: &'a HeadlessMap,
}

impl<'a> ResidentDemSampler<'a> {
    pub(crate) const fn new(map: &'a HeadlessMap) -> Self {
        Self { map }
    }

    /// Samples `(surface_msl_m, cell_max_msl_m)` in unexaggerated datum metres at
    /// `(lat_deg, lon_deg)`. Returns `None` when no DEM tile covering the point is resident.
    pub(crate) fn sample_msl_and_cell_max(
        &mut self,
        lat_deg: f64,
        lon_deg: f64,
    ) -> Option<(f64, f64)> {
        sample_cell_msl_and_max(self.map, lat_deg, lon_deg)
    }
}

/// One-shot rendered-surface MSL sample at `(lat_deg, lon_deg)` from `map`'s resident DEM.
pub(crate) fn sample_point_msl(map: &HeadlessMap, lat_deg: f64, lon_deg: f64) -> Option<f64> {
    sample_cell_msl_and_max(map, lat_deg, lon_deg).map(|(surface, _)| surface)
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

/// Computes the DO-315B minimum SVS eye altitude (datum metres) required to keep `stamp`'s
/// camera eye and entire near-clip rectangle strictly above `map`'s rendered terrain mesh.
///
/// Returns `None` when no DEM covering `stamp.anchor` is resident yet, leaving unloaded or
/// DEM-free basins (`< 0 m`) untouched.
pub(crate) fn svs_eye_floor_msl(map: &HeadlessMap, stamp: &ConformalFrameStamp) -> Option<f64> {
    let body_r = stamp.render_body.radius_meters;
    if !body_r.is_finite() || body_r <= 0.0 {
        return None;
    }
    let f = &stamp.external_view.frustum;
    let max_tan_x = f.left.abs().max(f.right.abs());
    let max_tan_y = f.top.abs().max(f.bottom.abs());
    let near_corner_m = f.near * (1.0 + max_tan_x * max_tan_x + max_tan_y * max_tan_y).sqrt();
    let clearance_m = (near_corner_m * 2.5).max(MIN_SVS_EYE_CLEARANCE_M);
    let radius_m = near_corner_m.max(MIN_FOOTPRINT_RADIUS_M);

    let lat0 = stamp.anchor.lat_deg.clamp(-90.0, 90.0);
    let lon0 = wrap_lon_deg(stamp.anchor.lon_deg);
    let d_lat_deg = (radius_m / body_r).to_degrees();
    let cos_lat = lat0.to_radians().cos().abs().max(1e-3);
    let d_lon_deg = (radius_m / (body_r * cos_lat)).to_degrees().min(45.0);

    let probes = [
        (lat0, lon0),
        ((lat0 + d_lat_deg).clamp(-90.0, 90.0), lon0),
        ((lat0 - d_lat_deg).clamp(-90.0, 90.0), lon0),
        (lat0, wrap_lon_deg(lon0 + d_lon_deg)),
        (lat0, wrap_lon_deg(lon0 - d_lon_deg)),
    ];
    let mut max_surface_msl: Option<f64> = None;
    for (lat, lon) in probes {
        if let Some((tri_msl, cell_max_msl)) = sample_cell_msl_and_max(map, lat, lon) {
            let local_peak = tri_msl.max(cell_max_msl);
            max_surface_msl = Some(max_surface_msl.map_or(local_peak, |prev| prev.max(local_peak)));
        }
    }
    max_surface_msl
        .map(|peak| (peak + clearance_m).max(stamp.render_body.lowest_ground_meters + clearance_m))
}
