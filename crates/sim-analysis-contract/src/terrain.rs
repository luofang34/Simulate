//! Vertical datum specifications (`Wgs84Ellipsoidal`, `DemSurfaceAgl`, `OrthometricMsl`),
//! serializable DEM elevation grids (`DemElevationGrid`), and terrain line-of-sight configuration (`TerrainLosConfig`).

use serde::{Deserialize, Serialize};
use sim_geo::BodyEllipsoid;

use crate::{ContentHash, DomainHasher, NaifId, cos, taxonomy::ModelNotApplicableReason};

/// Vertical datum category used to interpret endpoint or terrain heights.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum VerticalDatumKind {
    /// Height (`m`) measured directly along the normal above the reference ellipsoid (`WGS84`).
    Wgs84Ellipsoidal,
    /// Height (`m`) above local digital elevation model (`DEM`) terrain surface (`AGL`).
    DemSurfaceAgl,
    /// Orthometric height (`m`) above mean sea level (`MSL` / geoid undulation model).
    OrthometricMsl,
}

/// Explicit vertical datum specification for surface stations, aircraft waypoints, or terrain queries.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum VerticalDatumSpec {
    /// Height (`m`) directly above the reference ellipsoid (`WGS84`).
    Wgs84Ellipsoidal,
    /// Height (`m`) above local DEM terrain surface (`AGL`), requiring coverage from `dataset_id`.
    DemSurfaceAgl {
        /// Identifier of the required DEM dataset (e.g. `"regional-ridge-dem-v1"`).
        dataset_id: String,
    },
    /// Orthometric height (`m`) above the geoid (`MSL`), requiring an explicit geoid undulation
    /// $N_{\text{geoid}}$ (`m`) above the reference ellipsoid (`h_ellipsoid = H_msl + N_geoid`).
    OrthometricMsl {
        /// Identifier of the geoid model (e.g. `"EGM2008"`).
        geoid_model_id: String,
        /// Geoid undulation $N_{\text{geoid}}$ (`m`) at the location; `None` fails closed.
        undulation_m: Option<f64>,
    },
}

impl VerticalDatumSpec {
    /// Returns the [`VerticalDatumKind`] corresponding to this specification.
    #[must_use]
    pub const fn kind(&self) -> VerticalDatumKind {
        match self {
            Self::Wgs84Ellipsoidal => VerticalDatumKind::Wgs84Ellipsoidal,
            Self::DemSurfaceAgl { .. } => VerticalDatumKind::DemSurfaceAgl,
            Self::OrthometricMsl { .. } => VerticalDatumKind::OrthometricMsl,
        }
    }
}

/// Altitude value paired with its explicit [`VerticalDatumSpec`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AltitudeSpec {
    /// Vertical reference datum.
    pub datum: VerticalDatumSpec,
    /// Height value (`m`) relative to `datum`.
    pub value_m: f64,
}

/// Serializable bilinear latitude/longitude DEM grid with explicit `Option<f64>` coverage cells.
///
/// Cells with `None` represent missing/void DEM postings and always fail closed with
/// [`DataCoverageError::TerrainDemOutOfCoverage`](crate::DataCoverageError::TerrainDemOutOfCoverage)
/// instead of substituting `0.0 m`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DemElevationGrid {
    /// Dataset identifier.
    pub dataset_id: String,
    /// Host celestial body (`NaifId::EARTH`, etc.).
    pub body: NaifId,
    /// Southern latitude bound (`rad`).
    pub lat_min_rad: f64,
    /// Northern latitude bound (`rad`, `> lat_min_rad`).
    pub lat_max_rad: f64,
    /// Western longitude bound (`rad`).
    pub lon_min_rad: f64,
    /// Eastern longitude bound (`rad`, `> lon_min_rad`).
    pub lon_max_rad: f64,
    /// Number of latitude rows (`>= 2`).
    pub rows: usize,
    /// Number of longitude columns (`>= 2`).
    pub cols: usize,
    /// Row-major (`row * cols + col`) terrain heights (`m` above reference ellipsoid); `None` = no coverage.
    pub cells_m: Vec<Option<f64>>,
}

impl DemElevationGrid {
    /// Validates grid dimensions and bounds, returning [`ModelNotApplicableReason::InvalidTerrainParameters`] if malformed.
    pub fn validate(&self) -> Result<(), ModelNotApplicableReason> {
        let expected_len = self.rows.checked_mul(self.cols).unwrap_or(0);
        if self.rows < 2
            || self.cols < 2
            || expected_len != self.cells_m.len()
            || !self.lat_min_rad.is_finite()
            || !self.lat_max_rad.is_finite()
            || self.lat_max_rad <= self.lat_min_rad
            || !self.lon_min_rad.is_finite()
            || !self.lon_max_rad.is_finite()
            || self.lon_max_rad <= self.lon_min_rad
        {
            return Err(ModelNotApplicableReason::InvalidTerrainParameters {
                reason: "invalid DEM grid bounds or row/col cell count".to_owned(),
            });
        }
        Ok(())
    }

    /// Computes the canonical SHA-256 [`ContentHash`] of this DEM elevation grid.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-analysis-contract:dem-elevation-grid:v1");
        h.write_bytes(self.dataset_id.as_bytes());
        h.write_i32(self.body.0);
        h.write_f64_canonical(self.lat_min_rad);
        h.write_f64_canonical(self.lat_max_rad);
        h.write_f64_canonical(self.lon_min_rad);
        h.write_f64_canonical(self.lon_max_rad);
        h.write_u64(self.rows as u64);
        h.write_u64(self.cols as u64);
        h.write_u64(self.cells_m.len() as u64);
        for &cell in &self.cells_m {
            h.write_opt_f64_canonical(cell);
        }
        h.finish()
    }

    /// Returns the minimum horizontal cell spacing (`m`) on `ellipsoid` across latitude and longitude axes.
    #[must_use]
    pub fn min_horizontal_cell_spacing_m(&self, ellipsoid: BodyEllipsoid) -> f64 {
        let r_m = ellipsoid.semi_major_m.max(1.0);
        let dlat_rad =
            (self.lat_max_rad - self.lat_min_rad) / (self.rows.saturating_sub(1).max(1) as f64);
        let dlon_rad =
            (self.lon_max_rad - self.lon_min_rad) / (self.cols.saturating_sub(1).max(1) as f64);
        let mid_lat = 0.5 * (self.lat_min_rad + self.lat_max_rad);
        let dlat_m = dlat_rad.abs() * r_m;
        let dlon_m = dlon_rad.abs() * r_m * cos(mid_lat).abs().max(0.01);
        dlat_m.min(dlon_m).max(0.1)
    }

    /// Returns the terrain height (`m`) at `(row, col)`, or `None` if out of bounds or a void cell.
    #[must_use]
    pub fn cell_at(&self, row: usize, col: usize) -> Option<f64> {
        let idx = row.checked_mul(self.cols)?.checked_add(col)?;
        self.cells_m.get(idx).copied().flatten()
    }
}

/// Default maximum number of DEM ray samples allowed along a single chord inside the terrain interaction shell.
pub const DEFAULT_MAX_TERRAIN_RAY_SAMPLES: u32 = 50_000;

const fn default_max_terrain_ray_samples() -> u32 {
    DEFAULT_MAX_TERRAIN_RAY_SAMPLES
}

/// Configuration for local DEM terrain line-of-sight occultation along a body-fixed chord.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TerrainLosConfig {
    /// Maximum spacing (`m`, `> 0`) between ray samples within the terrain interaction shell.
    pub sample_step_m: f64,
    /// Minimum required vertical clearance (`m`) of the line-of-sight ray above DEM terrain.
    pub min_clearance_m: f64,
    /// Ellipsoidal altitude ceiling (`m`) below which ray samples are tested against the local DEM.
    /// Any ray sample with `height_m <= max_terrain_interaction_altitude_m` MUST fall within DEM coverage.
    pub max_terrain_interaction_altitude_m: f64,
    /// Maximum number of ray samples permitted within the terrain interaction shell before failing closed
    /// with [`ModelNotApplicableReason::TerrainRaySampleBudgetExceeded`].
    #[serde(default = "default_max_terrain_ray_samples")]
    pub max_ray_samples: u32,
}

impl Default for TerrainLosConfig {
    fn default() -> Self {
        Self {
            sample_step_m: 250.0,
            min_clearance_m: 0.0,
            max_terrain_interaction_altitude_m: 12_000.0,
            max_ray_samples: DEFAULT_MAX_TERRAIN_RAY_SAMPLES,
        }
    }
}
