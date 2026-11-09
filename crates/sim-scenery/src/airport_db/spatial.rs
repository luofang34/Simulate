//! A 2° × 2° equirectangular grid over the globe for visible-marker queries
//! and nearest-airport picking.

use super::geometry::{MEAN_EARTH_RADIUS_M, central_angle_rad};
use super::model::{AerodromeTier, AirportIdx, AirportRecord};
use super::{AirportDatabase, rank_records};

/// Cell size, degrees.
pub(crate) const CELL_DEG: f64 = 2.0;
/// Cells around a parallel and along a meridian.
pub(crate) const LON_CELLS: usize = 180;
pub(crate) const LAT_CELLS: usize = 90;
/// Visible cap radius at and below [`FULL_GLOBE_ZOOM`], degrees.
const FULL_GLOBE_RADIUS_DEG: f64 = 82.0;
const FULL_GLOBE_ZOOM: f64 = 2.5;
/// Smallest visible cap radius, degrees.
const MIN_RADIUS_DEG: f64 = 0.01;

/// The angular radius of the cap a globe view at `zoom` shows: about a
/// hemisphere at zoom 2.5 and below, halving with each zoom level above.
#[must_use]
pub fn globe_visible_radius_deg(zoom: f64) -> f64 {
    let above = if zoom.is_finite() {
        (zoom - FULL_GLOBE_ZOOM).max(0.0)
    } else {
        0.0
    };
    (FULL_GLOBE_RADIUS_DEG * (-above).exp2()).clamp(MIN_RADIUS_DEG, FULL_GLOBE_RADIUS_DEG)
}

fn lat_band(lat_deg: f64) -> usize {
    (((lat_deg + 90.0) / CELL_DEG).floor().max(0.0) as usize).min(LAT_CELLS - 1)
}

fn lon_band(lon_deg: f64) -> usize {
    (((lon_deg + 180.0) / CELL_DEG)
        .floor()
        .rem_euclid(LON_CELLS as f64) as usize)
        .min(LON_CELLS - 1)
}

/// Airports bucketed by cell, each cell most prominent first, laid out as
/// one entry array with per-cell offsets.
#[derive(Debug, Clone, Default)]
pub(crate) struct GlobeGrid {
    /// `starts[c]..starts[c + 1]` are cell `c`'s entries.
    starts: Vec<u32>,
    entries: Vec<AirportIdx>,
}

impl GlobeGrid {
    pub(crate) fn build(records: &[AirportRecord]) -> Self {
        let cell_of = |record: &AirportRecord| {
            lat_band(record.lat_deg) * LON_CELLS + lon_band(record.lon_deg)
        };
        let mut order: Vec<(usize, u32)> = records
            .iter()
            .enumerate()
            .filter_map(|(i, record)| Some((cell_of(record), u32::try_from(i).ok()?)))
            .collect();
        order.sort_by(|a, b| {
            let rank = match (records.get(a.1 as usize), records.get(b.1 as usize)) {
                (Some(x), Some(y)) => rank_records(x, y),
                _ => a.1.cmp(&b.1),
            };
            a.0.cmp(&b.0).then(rank)
        });
        let mut starts = Vec::with_capacity(LAT_CELLS * LON_CELLS + 1);
        let mut next = 0;
        for cell in 0..=LAT_CELLS * LON_CELLS {
            next += order
                .get(next..)
                .unwrap_or(&[])
                .iter()
                .take_while(|entry| entry.0 < cell)
                .count();
            starts.push(u32::try_from(next).unwrap_or(u32::MAX));
        }
        let entries = order.into_iter().map(|(_, i)| AirportIdx(i)).collect();
        Self { starts, entries }
    }

    pub(crate) fn cell(&self, lat: usize, lon: usize) -> &[AirportIdx] {
        let cell = lat * LON_CELLS + lon;
        let start = self.starts.get(cell).map_or(0, |&s| s as usize);
        let end = self.starts.get(cell + 1).map_or(start, |&e| e as usize);
        self.entries.get(start..end).unwrap_or(&[])
    }

    /// Entries of every cell a spherical cap of `radius_deg` around a
    /// centre may reach.
    fn cap(&self, centre: (f64, f64), radius_deg: f64) -> impl Iterator<Item = AirportIdx> + '_ {
        let (lat, lon) = centre;
        let south = lat_band((lat - radius_deg).max(-90.0));
        let north = lat_band((lat + radius_deg).min(90.0));
        let reaches_pole = lat + radius_deg >= 90.0 || lat - radius_deg <= -90.0;
        let ratio = radius_deg.to_radians().sin() / lat.to_radians().cos().max(1e-12);
        let half_width_deg = if reaches_pole || ratio >= 1.0 {
            180.0
        } else {
            ratio.asin().to_degrees()
        };
        let lon_cells: Vec<usize> = if half_width_deg >= 180.0 - CELL_DEG {
            (0..LON_CELLS).collect()
        } else {
            let west = lon_band(lon - half_width_deg);
            let span = ((2.0 * half_width_deg / CELL_DEG).ceil() as usize + 2).min(LON_CELLS);
            (0..span).map(|k| (west + k) % LON_CELLS).collect()
        };
        (south..=north).flat_map(move |lat| {
            let lons = lon_cells.clone();
            lons.into_iter()
                .flat_map(move |lon| self.cell(lat, lon).iter().copied())
        })
    }
}

impl AirportDatabase {
    /// Markers for a globe view centred on a point at `zoom`: airports of
    /// the tiers the zoom shows inside its visible cap
    /// ([`globe_visible_radius_deg`], [`AerodromeTier::max_visible_at_zoom`]),
    /// most prominent first, at most `max_markers`. The built-in aerodromes
    /// (`47N`, `KEWR`, `KDOV`) are always kept while on the visible
    /// hemisphere.
    #[must_use]
    pub fn query_globe_visible(
        &self,
        center_lat_deg: f64,
        center_lon_deg: f64,
        zoom: f64,
        max_markers: usize,
    ) -> Vec<AirportIdx> {
        self.query_globe_visible_pinned(center_lat_deg, center_lon_deg, zoom, max_markers, None)
    }

    /// [`Self::query_globe_visible`], also keeping `pinned` (say the selected
    /// airport) ahead of everything while it is on the visible hemisphere.
    #[must_use]
    pub fn query_globe_visible_pinned(
        &self,
        center_lat_deg: f64,
        center_lon_deg: f64,
        zoom: f64,
        max_markers: usize,
        pinned: Option<AirportIdx>,
    ) -> Vec<AirportIdx> {
        let centre = (center_lat_deg, center_lon_deg);
        let radius_deg = globe_visible_radius_deg(zoom);
        let radius = radius_deg.to_radians();
        let max_tier = AerodromeTier::max_visible_at_zoom(zoom);
        let angle = |idx: AirportIdx| {
            self.get(idx).map_or(f64::INFINITY, |r| {
                central_angle_rad(centre, (r.lat_deg, r.lon_deg))
            })
        };
        let mut kept: Vec<AirportIdx> = Vec::new();
        for idx in pinned.into_iter().chain(self.featured.iter().copied()) {
            if angle(idx) < std::f64::consts::FRAC_PI_2 && !kept.contains(&idx) {
                kept.push(idx);
            }
        }
        kept.truncate(max_markers);
        let mut rest: Vec<AirportIdx> = self
            .grid
            .cap(centre, radius_deg)
            .filter(|idx| self.get(*idx).is_some_and(|r| r.tier <= max_tier))
            .filter(|&idx| angle(idx) <= radius && !kept.contains(&idx))
            .collect();
        rest.sort_by(|&a, &b| self.rank(a, b));
        rest.truncate(max_markers.saturating_sub(kept.len()));
        kept.extend(rest);
        kept
    }

    /// The airport of tier `max_tier` or more prominent nearest a point,
    /// within `max_radius_km` along the great circle.
    #[must_use]
    pub fn nearest(
        &self,
        lat_deg: f64,
        lon_deg: f64,
        max_radius_km: f64,
        max_tier: AerodromeTier,
    ) -> Option<AirportIdx> {
        let radius = (max_radius_km * 1_000.0 / MEAN_EARTH_RADIUS_M).min(std::f64::consts::PI);
        self.grid
            .cap((lat_deg, lon_deg), radius.to_degrees())
            .filter_map(|idx| {
                let record = self.get(idx)?;
                let angle = central_angle_rad((lat_deg, lon_deg), (record.lat_deg, record.lon_deg));
                (record.tier <= max_tier && angle <= radius).then_some((angle, idx))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)))
            .map(|(_, idx)| idx)
    }
}
