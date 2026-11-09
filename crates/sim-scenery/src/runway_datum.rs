//! The runway datum: every charted runway's graded strip at its published threshold and
//! end elevations, the one runway height the drawn terrain, the physics ground and the
//! synthetic-vision runway all read.
//!
//! Strips are built lazily, one 2° airport-grid cell at a time, and filed under every
//! fine cell their skirt reaches, so a height query looks at a handful of strips.

use std::collections::BTreeMap;
use std::fmt;
use std::sync::{Arc, OnceLock};

use sim_geo::Geodetic;

use crate::airport_db::{CELL_DEG, LAT_CELLS, LON_CELLS};
use crate::world_query::strongest;
use crate::{AirportDatabase, CatalogVerticalDatum, RunwayStrip};

mod stamp;
#[cfg(test)]
mod tests;

pub use stamp::MIN_STAMP_ZOOM;

/// Fine cells per degree.
const FINE_PER_DEG: f64 = 32.0;
/// Fine cells around a parallel.
const FINE_LON_CELLS: i64 = 360 * FINE_PER_DEG as i64;
/// Beyond this latitude strips live in one polar list: meridians converge too fast for
/// longitude cells to bound a runway.
const POLAR_LAT_DEG: f64 = 80.0;

/// Strips filed by fine cell `(lat index, lon index)`.
type FineCells = BTreeMap<(i64, i64), Vec<RunwayStrip>>;

struct Index {
    database: Arc<AirportDatabase>,
    cells: Vec<OnceLock<FineCells>>,
    polar: OnceLock<Vec<RunwayStrip>>,
}

/// Every runway of an [`AirportDatabase`] as the strip an
/// [`crate::AuthoritativeWorldSurface`] holds: pavement, shoulders and overruns at the
/// charted profile between the published end elevations, feathered into the terrain.
///
/// Heights are charted MSL numbers, read in whatever datum the terrain they blend with
/// uses, as [`AirportRecord::runway_strips`](crate::AirportRecord::runway_strips) does.
/// Clones share one lazily built index.
#[derive(Clone)]
pub struct RunwayDatum {
    index: Arc<Index>,
}

impl fmt::Debug for RunwayDatum {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RunwayDatum")
            .field("airports", &self.index.database.len())
            .finish_non_exhaustive()
    }
}

/// Two datums are equal when they index the same database.
impl PartialEq for RunwayDatum {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.index.database, &other.index.database)
    }
}

impl RunwayDatum {
    /// The runways of `database`.
    #[must_use]
    pub fn new(database: Arc<AirportDatabase>) -> Self {
        Self {
            index: Arc::new(Index {
                database,
                cells: (0..LAT_CELLS * LON_CELLS)
                    .map(|_| OnceLock::new())
                    .collect(),
                polar: OnceLock::new(),
            }),
        }
    }

    /// The runways of [`AirportDatabase::builtin`].
    #[must_use]
    pub fn builtin() -> Self {
        Self::new(Arc::clone(AirportDatabase::builtin()))
    }

    /// The charted runway height at `position` and how strongly it applies: 1 on the
    /// pavement, shoulders and overruns, easing to 0 across the skirt; `None` off every
    /// strip.
    #[must_use]
    pub fn height_and_weight(&self, position: &Geodetic) -> Option<(f64, f64)> {
        let lat_deg = position.lat_deg();
        if lat_deg.abs() > POLAR_LAT_DEG {
            return strongest(self.polar_strips(), position);
        }
        let key = fine_key(lat_deg, position.lon_deg());
        strongest(self.fine_cell(key), position)
    }

    /// `terrain_m` with the runway datum laid over it: the charted height on a strip,
    /// blended toward it across the skirt, `terrain_m` elsewhere.
    #[must_use]
    pub fn blend(&self, position: &Geodetic, terrain_m: f64) -> f64 {
        self.height_and_weight(position)
            .map_or(terrain_m, |(height, weight)| {
                terrain_m + (height - terrain_m) * weight
            })
    }

    /// Whether any strip reaches into the fine cells covering a latitude/longitude box.
    fn any_strip_in(&self, (south, north): (f64, f64), (west, east): (f64, f64)) -> bool {
        if south.abs().max(north.abs()) > POLAR_LAT_DEG {
            return !self.polar_strips().is_empty();
        }
        let (lat_lo, lon_lo) = fine_key(south, west);
        let (lat_hi, lon_hi) = fine_key(north, east);
        let lon_span = (lon_hi - lon_lo).rem_euclid(FINE_LON_CELLS);
        (lat_lo..=lat_hi).any(|lat| {
            (0..=lon_span).any(|step| {
                !self
                    .fine_cell((lat, (lon_lo + step).rem_euclid(FINE_LON_CELLS)))
                    .is_empty()
            })
        })
    }

    fn fine_cell(&self, key: (i64, i64)) -> &[RunwayStrip] {
        let (lat_band, lon_band) = parent_of(key);
        self.index
            .cells
            .get(lat_band * LON_CELLS + lon_band)
            .map(|cell| cell.get_or_init(|| self.build_cell(lat_band, lon_band)))
            .and_then(|cells| cells.get(&key))
            .map_or(&[], Vec::as_slice)
    }

    /// Files the strips of airports in the cell and its neighbours under the fine cells of
    /// `(lat_band, lon_band)` their skirts reach.
    fn build_cell(&self, lat_band: usize, lon_band: usize) -> FineCells {
        let mut cells = FineCells::new();
        for strip in self.strips_around(lat_band, lon_band) {
            let corners = strip.skirt_corners();
            let lats = corners.map(|c| c.lat_deg());
            let lons = unwrapped_lons(corners.map(|c| c.lon_deg()));
            let (lat_lo, lon_lo) = fine_key(min(lats), min(lons));
            let (lat_hi, lon_hi) = fine_key(max(lats), max(lons));
            let lon_span = (lon_hi - lon_lo).rem_euclid(FINE_LON_CELLS);
            for lat in lat_lo..=lat_hi {
                for step in 0..=lon_span {
                    let key = (lat, (lon_lo + step).rem_euclid(FINE_LON_CELLS));
                    if parent_of(key) == (lat_band, lon_band) {
                        cells.entry(key).or_default().push(strip);
                    }
                }
            }
        }
        cells
    }

    fn strips_around(&self, lat_band: usize, lon_band: usize) -> Vec<RunwayStrip> {
        let lat_bands = lat_band.saturating_sub(1)..=(lat_band + 1).min(LAT_CELLS - 1);
        lat_bands
            .flat_map(|lat| {
                [LON_CELLS - 1, 0, 1].map(move |offset| (lat, (lon_band + offset) % LON_CELLS))
            })
            .flat_map(|(lat, lon)| self.index.database.airports_in_cell(lat, lon))
            .flat_map(|record| record.runway_strips(CatalogVerticalDatum::OrthometricMslEgm96))
            .collect()
    }

    fn polar_strips(&self) -> &[RunwayStrip] {
        self.index.polar.get_or_init(|| {
            // Bands reaching within a degree of the polar latitude, so a strip whose
            // airport sits just outside it still covers the polar side.
            let reach_deg = POLAR_LAT_DEG - 1.0;
            (0..LAT_CELLS)
                .filter(|&lat| {
                    let south_edge = -90.0 + lat as f64 * CELL_DEG;
                    south_edge < -reach_deg || south_edge + CELL_DEG > reach_deg
                })
                .flat_map(|lat| (0..LON_CELLS).map(move |lon| (lat, lon)))
                .flat_map(|(lat, lon)| self.index.database.airports_in_cell(lat, lon))
                .flat_map(|record| record.runway_strips(CatalogVerticalDatum::OrthometricMslEgm96))
                .collect()
        })
    }
}

fn fine_key(lat_deg: f64, lon_deg: f64) -> (i64, i64) {
    let lat = ((lat_deg.clamp(-90.0, 90.0) + 90.0) * FINE_PER_DEG).floor() as i64;
    let lon = (((lon_deg + 180.0) * FINE_PER_DEG).floor() as i64).rem_euclid(FINE_LON_CELLS);
    (lat, lon)
}

/// The 2° airport-grid cell holding fine cell `key`.
fn parent_of((lat, lon): (i64, i64)) -> (usize, usize) {
    let per_cell = (CELL_DEG * FINE_PER_DEG) as i64;
    let lat_band = (lat / per_cell).clamp(0, LAT_CELLS as i64 - 1) as usize;
    let lon_band = (lon / per_cell).clamp(0, LON_CELLS as i64 - 1) as usize;
    (lat_band, lon_band)
}

/// Longitudes made continuous across the antimeridian, relative to the first.
fn unwrapped_lons(lons: [f64; 4]) -> [f64; 4] {
    let [first, ..] = lons;
    lons.map(|lon| first + (lon - first + 180.0).rem_euclid(360.0) - 180.0)
}

fn min(values: [f64; 4]) -> f64 {
    values.into_iter().fold(f64::INFINITY, f64::min)
}

fn max(values: [f64; 4]) -> f64 {
    values.into_iter().fold(f64::NEG_INFINITY, f64::max)
}
