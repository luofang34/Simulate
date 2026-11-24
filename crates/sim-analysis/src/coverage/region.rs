//! H3 region resolution, bounds validation, and deterministic cell enumeration.

use core::f64::consts::{FRAC_PI_2, PI, TAU};
use std::collections::BTreeSet;

use h3o::{CellIndex, LatLng, Resolution};
use sim_spacetime::{asin_clamped, atan2, cos, sin};

use crate::coverage::{CoverageGridError, CoverageRegionSpec, MAX_GLOBAL_H3_RESOLUTION};

/// Resolved H3 cell with geodetic coordinates (`height_m = 0`).
#[derive(Clone, Copy, Debug)]
pub(super) struct ResolvedH3Cell {
    pub(super) h3_index: u64,
    pub(super) lat_rad: f64,
    pub(super) lon_rad: f64,
}

#[derive(Clone, Copy)]
struct LatLonBounds {
    min_lat_rad: f64,
    max_lat_rad: f64,
    min_lon_rad: f64,
    max_lon_rad: f64,
    lon_center_rad: f64,
    lon_half_width_rad: f64,
}

impl LatLonBounds {
    fn from_spec(
        min_lat_rad: f64,
        max_lat_rad: f64,
        min_lon_rad: f64,
        max_lon_rad: f64,
    ) -> Result<Self, CoverageGridError> {
        if !min_lat_rad.is_finite()
            || !max_lat_rad.is_finite()
            || !min_lon_rad.is_finite()
            || !max_lon_rad.is_finite()
        {
            return Err(CoverageGridError::InvalidRegionBounds {
                reason: "latitude/longitude bounds must be finite".to_owned(),
            });
        }
        if min_lat_rad < -FRAC_PI_2 || max_lat_rad > FRAC_PI_2 || min_lat_rad > max_lat_rad {
            return Err(CoverageGridError::InvalidRegionBounds {
                reason: format!(
                    "invalid latitude bounds [{min_lat_rad}, {max_lat_rad}] rad (expected within [-pi/2, pi/2] with min <= max)"
                ),
            });
        }
        if min_lon_rad < -PI || min_lon_rad > PI || max_lon_rad < -PI || max_lon_rad > PI {
            return Err(CoverageGridError::InvalidRegionBounds {
                reason: format!(
                    "invalid longitude bounds [{min_lon_rad}, {max_lon_rad}] rad (expected within [-pi, pi])"
                ),
            });
        }
        let span = if min_lon_rad <= max_lon_rad {
            max_lon_rad - min_lon_rad
        } else {
            (TAU - min_lon_rad) + max_lon_rad
        };
        let lon_half_width_rad = 0.5 * span;
        let raw_center = min_lon_rad + lon_half_width_rad;
        let lon_center_rad = wrap_pi(raw_center);
        Ok(Self {
            min_lat_rad,
            max_lat_rad,
            min_lon_rad,
            max_lon_rad,
            lon_center_rad,
            lon_half_width_rad,
        })
    }

    fn contains(&self, lat_rad: f64, lon_rad: f64) -> bool {
        if lat_rad < self.min_lat_rad || lat_rad > self.max_lat_rad {
            return false;
        }
        if self.min_lon_rad <= self.max_lon_rad {
            lon_rad >= self.min_lon_rad && lon_rad <= self.max_lon_rad
        } else {
            lon_rad >= self.min_lon_rad || lon_rad <= self.max_lon_rad
        }
    }

    fn overlaps_cap(&self, center_lat_rad: f64, center_lon_rad: f64, radius_rad: f64) -> bool {
        if center_lat_rad + radius_rad < self.min_lat_rad
            || center_lat_rad - radius_rad > self.max_lat_rad
        {
            return false;
        }
        if radius_rad >= FRAC_PI_2 || center_lat_rad.abs() + radius_rad >= FRAC_PI_2 - 1.0e-9 {
            return true;
        }
        let ratio = sin(radius_rad) / cos(center_lat_rad);
        if ratio >= 1.0 {
            return true;
        }
        let dlon = asin_clamped(ratio);
        let center_diff = wrap_pi(center_lon_rad - self.lon_center_rad).abs();
        center_diff <= dlon + self.lon_half_width_rad + 1.0e-12
    }

    fn cell_matches(&self, cell: CellIndex) -> bool {
        let centroid = LatLng::from(cell);
        let c_lat = centroid.lat_radians();
        let c_lon = centroid.lng_radians();
        if self.contains(c_lat, c_lon) {
            return true;
        }
        let boundary = cell.boundary();
        let mut cell_min_lat = c_lat;
        let mut cell_max_lat = c_lat;
        let mut max_dlon = 0.0_f64;
        for v in boundary.iter() {
            let v_lat = v.lat_radians();
            let v_lon = v.lng_radians();
            if self.contains(v_lat, v_lon) {
                return true;
            }
            cell_min_lat = cell_min_lat.min(v_lat);
            cell_max_lat = cell_max_lat.max(v_lat);
            max_dlon = max_dlon.max(wrap_pi(v_lon - c_lon).abs());
        }
        if cell_max_lat < self.min_lat_rad || cell_min_lat > self.max_lat_rad {
            return false;
        }
        let lon_diff = wrap_pi(c_lon - self.lon_center_rad).abs();
        lon_diff <= max_dlon + self.lon_half_width_rad + 1.0e-12
    }
}

fn wrap_pi(angle_rad: f64) -> f64 {
    let (s, c) = sim_spacetime::sin_cos(angle_rad);
    atan2(s, c)
}

fn parse_resolution(raw: u8) -> Result<Resolution, CoverageGridError> {
    Resolution::try_from(raw).map_err(|_| CoverageGridError::InvalidResolution { resolution: raw })
}

fn check_cell_limit(count: usize, max_cells: usize) -> Result<(), CoverageGridError> {
    if count > max_cells {
        Err(CoverageGridError::CellCountExceeded {
            cell_count: count,
            max_cells,
        })
    } else {
        Ok(())
    }
}

fn to_resolved_list(cells: &BTreeSet<CellIndex>) -> Vec<ResolvedH3Cell> {
    cells
        .iter()
        .map(|&cell| {
            let ll = LatLng::from(cell);
            ResolvedH3Cell {
                h3_index: u64::from(cell),
                lat_rad: ll.lat_radians(),
                lon_rad: ll.lng_radians(),
            }
        })
        .collect()
}

/// Resolves a [`CoverageRegionSpec`] into a deduplicated list of cells sorted ascending by `u64`.
pub(super) fn resolve_region_cells(
    region: &CoverageRegionSpec,
    max_cells: usize,
) -> Result<Vec<ResolvedH3Cell>, CoverageGridError> {
    match region {
        CoverageRegionSpec::Global { resolution } => resolve_global_cells(*resolution, max_cells),
        CoverageRegionSpec::LatLonBox {
            resolution,
            min_lat_rad,
            max_lat_rad,
            min_lon_rad,
            max_lon_rad,
        } => resolve_lat_lon_box_cells(
            *resolution,
            *min_lat_rad,
            *max_lat_rad,
            *min_lon_rad,
            *max_lon_rad,
            max_cells,
        ),
        CoverageRegionSpec::ExplicitCells { h3_cells } => {
            resolve_explicit_cells(h3_cells, max_cells)
        }
    }
}

fn resolve_global_cells(
    resolution: u8,
    max_cells: usize,
) -> Result<Vec<ResolvedH3Cell>, CoverageGridError> {
    let res = parse_resolution(resolution)?;
    if resolution > MAX_GLOBAL_H3_RESOLUTION {
        return Err(CoverageGridError::GlobalResolutionTooHigh {
            resolution,
            max_resolution: MAX_GLOBAL_H3_RESOLUTION,
        });
    }
    let total = usize::try_from(res.cell_count()).unwrap_or(usize::MAX);
    check_cell_limit(total, max_cells)?;
    let mut set = BTreeSet::new();
    for base in CellIndex::base_cells() {
        for cell in base.children(res) {
            set.insert(cell);
        }
    }
    Ok(to_resolved_list(&set))
}

fn resolve_explicit_cells(
    h3_cells: &[u64],
    max_cells: usize,
) -> Result<Vec<ResolvedH3Cell>, CoverageGridError> {
    if h3_cells.is_empty() {
        return Err(CoverageGridError::EmptyRegion);
    }
    let mut set = BTreeSet::new();
    for &raw in h3_cells {
        let cell = CellIndex::try_from(raw).map_err(|err| CoverageGridError::InvalidCellIndex {
            raw_index: raw,
            reason: err.to_string(),
        })?;
        set.insert(cell);
        check_cell_limit(set.len(), max_cells)?;
    }
    Ok(to_resolved_list(&set))
}

fn resolve_lat_lon_box_cells(
    resolution: u8,
    min_lat_rad: f64,
    max_lat_rad: f64,
    min_lon_rad: f64,
    max_lon_rad: f64,
    max_cells: usize,
) -> Result<Vec<ResolvedH3Cell>, CoverageGridError> {
    let res = parse_resolution(resolution)?;
    let bounds = LatLonBounds::from_spec(min_lat_rad, max_lat_rad, min_lon_rad, max_lon_rad)?;
    if max_cells == 0 {
        return Err(CoverageGridError::CellCountExceeded {
            cell_count: 1,
            max_cells: 0,
        });
    }
    let mut out = BTreeSet::new();
    seed_box_corners_and_center(&bounds, res, max_cells, &mut out)?;
    for base in CellIndex::base_cells() {
        collect_box_subtree(base, res, &bounds, max_cells, &mut out)?;
    }
    if out.is_empty() {
        return Err(CoverageGridError::EmptyRegion);
    }
    Ok(to_resolved_list(&out))
}

fn seed_box_corners_and_center(
    bounds: &LatLonBounds,
    res: Resolution,
    max_cells: usize,
    out: &mut BTreeSet<CellIndex>,
) -> Result<(), CoverageGridError> {
    let mid_lat = 0.5 * (bounds.min_lat_rad + bounds.max_lat_rad);
    let mid_lon = bounds.lon_center_rad;
    let pts = [
        (mid_lat, mid_lon),
        (bounds.min_lat_rad, bounds.min_lon_rad),
        (bounds.min_lat_rad, bounds.max_lon_rad),
        (bounds.max_lat_rad, bounds.min_lon_rad),
        (bounds.max_lat_rad, bounds.max_lon_rad),
    ];
    for (lat, lon) in pts {
        if let Ok(ll) = LatLng::from_radians(lat, lon) {
            out.insert(ll.to_cell(res));
            check_cell_limit(out.len(), max_cells)?;
        }
    }
    Ok(())
}

fn collect_box_subtree(
    cell: CellIndex,
    target_res: Resolution,
    bounds: &LatLonBounds,
    max_cells: usize,
    out: &mut BTreeSet<CellIndex>,
) -> Result<(), CoverageGridError> {
    let cur_res = cell.resolution();
    if cur_res == target_res {
        if bounds.cell_matches(cell) {
            out.insert(cell);
            check_cell_limit(out.len(), max_cells)?;
        }
        return Ok(());
    }
    let ll = LatLng::from(cell);
    let cap_radius = (2.8 * cur_res.edge_length_rads()).min(PI);
    if !bounds.overlaps_cap(ll.lat_radians(), ll.lng_radians(), cap_radius) {
        return Ok(());
    }
    let Some(next_res) = cur_res.succ() else {
        return Ok(());
    };
    for child in cell.children(next_res) {
        collect_box_subtree(child, target_res, bounds, max_cells, out)?;
    }
    Ok(())
}
