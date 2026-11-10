//! Vertical datum specifications (`Wgs84Ellipsoidal`, `DemSurfaceAgl`, `OrthometricMsl`),
//! fail-closed digital elevation model (`DEM`) coverage queries ([`TerrainElevationSource`],
//! [`DemElevationGrid`]), and body-fixed line-of-sight terrain occultation testing.
//!
//! Missing DEM cells, out-of-bounds coordinates, or unavailable geoid undulations always fail
//! closed with [`DataCoverageError`] rather than silently substituting `0.0 m` sea level.
//! Ray marching analytically intersects the chord with the terrain interaction shell
//! (`height_m <= max_terrain_interaction_altitude_m`) and fails closed with
//! [`ModelNotApplicableReason::TerrainRaySampleBudgetExceeded`] if the required sample count
//! exceeds `TerrainLosConfig::max_ray_samples`.

use glam::DVec3;
use sim_geo::BodyEllipsoid;
use sim_spacetime::{NaifId, TdbEpoch, sqrt};

pub use sim_analysis_contract::{
    AltitudeSpec, DEFAULT_MAX_TERRAIN_RAY_SAMPLES, DemElevationGrid,
    MARS_AREOID_ABOVE_IAU_ELLIPSOID_M, MOLA_JEZERO_DATASET_ID, MOLA_JEZERO_SHA256, MolaLoadError,
    TerrainLosConfig, VerticalDatumKind, VerticalDatumSpec,
};

use crate::access::{
    AccessAnalysisError, AccessRestrictionReason, DataCoverageError, ModelNotApplicableReason,
};

/// Fail-closed terrain elevation provider returning local terrain height (`m`) above a body's reference ellipsoid.
pub trait TerrainElevationSource: Send + Sync {
    /// Dataset identifier for provenance and vertical-datum binding.
    fn dataset_id(&self) -> &str;

    /// Celestial body (`NaifId::EARTH`, `MOON`, `MARS`) covered by this DEM.
    fn body(&self) -> NaifId;

    /// Returns terrain elevation (`m`) above the reference ellipsoid at `(lat_rad, lon_rad)`,
    /// or [`DataCoverageError::TerrainDemOutOfCoverage`] if outside coverage or in a void cell.
    fn elevation_above_ellipsoid_m(
        &self,
        body: NaifId,
        lat_rad: f64,
        lon_rad: f64,
        epoch: TdbEpoch,
    ) -> Result<f64, DataCoverageError>;

    /// Optional minimum horizontal posting spacing (`m`) of this DEM on `ellipsoid`.
    fn min_horizontal_cell_spacing_m(&self, _ellipsoid: BodyEllipsoid) -> Option<f64> {
        None
    }
}

/// Resolves an [`AltitudeSpec`] into height above the reference ellipsoid (`m`), failing closed
/// with [`DataCoverageError`] if the required DEM or geoid undulation is missing.
pub fn resolve_altitude_ellipsoidal_height_m(
    spec: &AltitudeSpec,
    body: NaifId,
    lat_rad: f64,
    lon_rad: f64,
    epoch: TdbEpoch,
    dem: Option<&dyn TerrainElevationSource>,
) -> Result<f64, DataCoverageError> {
    match &spec.datum {
        VerticalDatumSpec::Wgs84Ellipsoidal => Ok(spec.value_m),
        VerticalDatumSpec::DemSurfaceAgl { dataset_id } => {
            let Some(dem_src) = dem else {
                return Err(DataCoverageError::VerticalDatumUnavailable {
                    datum_kind: VerticalDatumKind::DemSurfaceAgl,
                    lat_rad,
                    lon_rad,
                });
            };
            if dem_src.dataset_id() != dataset_id {
                return Err(DataCoverageError::VerticalDatumUnavailable {
                    datum_kind: VerticalDatumKind::DemSurfaceAgl,
                    lat_rad,
                    lon_rad,
                });
            }
            let surface_h_m = dem_src.elevation_above_ellipsoid_m(body, lat_rad, lon_rad, epoch)?;
            Ok(surface_h_m + spec.value_m)
        }
        VerticalDatumSpec::OrthometricMsl { undulation_m, .. } => {
            let Some(n_m) = *undulation_m else {
                return Err(DataCoverageError::VerticalDatumUnavailable {
                    datum_kind: VerticalDatumKind::OrthometricMsl,
                    lat_rad,
                    lon_rad,
                });
            };
            if !n_m.is_finite() {
                return Err(DataCoverageError::VerticalDatumUnavailable {
                    datum_kind: VerticalDatumKind::OrthometricMsl,
                    lat_rad,
                    lon_rad,
                });
            }
            Ok(spec.value_m + n_m)
        }
    }
}

impl TerrainElevationSource for DemElevationGrid {
    fn dataset_id(&self) -> &str {
        &self.dataset_id
    }

    fn body(&self) -> NaifId {
        self.body
    }

    fn elevation_above_ellipsoid_m(
        &self,
        body: NaifId,
        lat_rad: f64,
        lon_rad: f64,
        epoch: TdbEpoch,
    ) -> Result<f64, DataCoverageError> {
        let out_of_cov = DataCoverageError::TerrainDemOutOfCoverage {
            body,
            lat_rad,
            lon_rad,
            requested_epoch: epoch,
        };
        if body != self.body
            || self.rows < 2
            || self.cols < 2
            || !lat_rad.is_finite()
            || !lon_rad.is_finite()
            || lat_rad < self.lat_min_rad
            || lat_rad > self.lat_max_rad
            || lon_rad < self.lon_min_rad
            || lon_rad > self.lon_max_rad
        {
            return Err(out_of_cov);
        }

        let u_lat = (lat_rad - self.lat_min_rad) / (self.lat_max_rad - self.lat_min_rad);
        let u_lon = (lon_rad - self.lon_min_rad) / (self.lon_max_rad - self.lon_min_rad);
        let max_r = self.rows.saturating_sub(1);
        let max_c = self.cols.saturating_sub(1);
        let rf = (u_lat.clamp(0.0, 1.0)) * (max_r as f64);
        let cf = (u_lon.clamp(0.0, 1.0)) * (max_c as f64);
        let r0 = (rf.floor() as usize).min(max_r.saturating_sub(1));
        let c0 = (cf.floor() as usize).min(max_c.saturating_sub(1));
        let r1 = r0.wrapping_add(1).min(max_r);
        let c1 = c0.wrapping_add(1).min(max_c);
        let fr = (rf - (r0 as f64)).clamp(0.0, 1.0);
        let fc = (cf - (c0 as f64)).clamp(0.0, 1.0);

        let (Some(h00), Some(h01), Some(h10), Some(h11)) = (
            self.cell_at(r0, c0),
            self.cell_at(r0, c1),
            self.cell_at(r1, c0),
            self.cell_at(r1, c1),
        ) else {
            return Err(out_of_cov);
        };
        if !h00.is_finite() || !h01.is_finite() || !h10.is_finite() || !h11.is_finite() {
            return Err(out_of_cov);
        }
        let h0 = h00 * (1.0 - fc) + h01 * fc;
        let h1 = h10 * (1.0 - fc) + h11 * fc;
        Ok(h0 * (1.0 - fr) + h1 * fr)
    }

    fn min_horizontal_cell_spacing_m(&self, ellipsoid: BodyEllipsoid) -> Option<f64> {
        Some(Self::min_horizontal_cell_spacing_m(self, ellipsoid))
    }
}

/// Analytically computes the parameter interval `[s_enter, s_exit] subset [0, 1]` where the chord
/// `p(s) = p0_fixed_m + s * (p1_fixed_m - p0_fixed_m)` intersects the outer bounding ellipsoid of
/// the terrain interaction shell (`height_m <= max_terrain_interaction_altitude_m`).
fn intersect_chord_with_terrain_shell(
    p0_fixed_m: DVec3,
    p1_fixed_m: DVec3,
    ellipsoid: BodyEllipsoid,
    max_terrain_interaction_altitude_m: f64,
) -> Option<(f64, f64)> {
    let margin_m = max_terrain_interaction_altitude_m + 100.0;
    let a = (ellipsoid.semi_major_m + margin_m).max(1.0);
    let b = (ellipsoid.semi_minor_m() + margin_m).max(1.0);
    let d = p1_fixed_m - p0_fixed_m;
    let u0 = DVec3::new(p0_fixed_m.x / a, p0_fixed_m.y / a, p0_fixed_m.z / b);
    let v = DVec3::new(d.x / a, d.y / a, d.z / b);
    let c2 = v.dot(v);
    let c1 = u0.dot(v);
    let c0 = u0.dot(u0) - 1.0;

    if c2 <= 1.0e-30 {
        return (c0 <= 0.0).then_some((0.0, 1.0));
    }
    let disc = c1 * c1 - c2 * c0;
    if disc < 0.0 {
        return None;
    }
    let root = sqrt(disc);
    let s_min = (-c1 - root) / c2;
    let s_max = (-c1 + root) / c2;
    let s_enter = s_min.max(0.0);
    let s_exit = s_max.min(1.0);
    (s_enter <= s_exit).then_some((s_enter, s_exit))
}

#[allow(clippy::too_many_arguments)]
fn sample_chord_in_terrain_shell(
    epoch: TdbEpoch,
    p0_fixed_m: DVec3,
    chord: DVec3,
    s_enter: f64,
    s_exit: f64,
    n_steps: usize,
    ellipsoid: BodyEllipsoid,
    dem: &dyn TerrainElevationSource,
    config: &TerrainLosConfig,
) -> Result<Option<AccessRestrictionReason>, AccessAnalysisError> {
    let mut min_clearance = f64::INFINITY;
    let mut worst_lat = 0.0;
    let mut worst_lon = 0.0;

    let mut idx = 0_usize;
    while idx <= n_steps {
        let u = (idx as f64) / (n_steps as f64);
        let s = s_enter + u * (s_exit - s_enter);
        let pt_fixed = p0_fixed_m + s * chord;
        let geo = ellipsoid.fixed_to_geodetic(pt_fixed);
        if geo.height_m <= config.max_terrain_interaction_altitude_m {
            let terrain_h_m =
                dem.elevation_above_ellipsoid_m(dem.body(), geo.lat_rad, geo.lon_rad, epoch)?;
            let clearance_m = geo.height_m - terrain_h_m;
            if clearance_m < min_clearance {
                min_clearance = clearance_m;
                worst_lat = geo.lat_rad;
                worst_lon = geo.lon_rad;
            }
        }
        idx = idx.wrapping_add(1);
    }

    if min_clearance < config.min_clearance_m {
        Ok(Some(AccessRestrictionReason::OccultedByTerrain {
            body: dem.body(),
            min_clearance_m: min_clearance,
            required_clearance_m: config.min_clearance_m,
            obstruction_lat_rad: worst_lat,
            obstruction_lon_rad: worst_lon,
        }))
    } else {
        Ok(None)
    }
}

/// Evaluates whether local DEM terrain occults the body-fixed line-of-sight chord from
/// `p0_fixed_m` to `p1_fixed_m` at `epoch`.
///
/// Analytically restricts sampling to the sub-segment `[s_enter, s_exit]` inside the terrain
/// interaction shell (`height_m <= config.max_terrain_interaction_altitude_m`).
/// - Fails closed with [`ModelNotApplicableReason::TerrainRaySampleBudgetExceeded`] if the required
///   sample count on `[s_enter, s_exit]` exceeds `config.max_ray_samples` (never silently coarsens the step).
/// - Fails closed with [`DataCoverageError::TerrainDemOutOfCoverage`] if any sample inside the shell
///   falls outside DEM coverage or hits a void cell.
pub fn check_terrain_line_of_sight_fixed(
    epoch: TdbEpoch,
    p0_fixed_m: DVec3,
    p1_fixed_m: DVec3,
    ellipsoid: BodyEllipsoid,
    dem: &dyn TerrainElevationSource,
    config: &TerrainLosConfig,
) -> Result<Option<AccessRestrictionReason>, AccessAnalysisError> {
    if !config.sample_step_m.is_finite()
        || config.sample_step_m <= 0.0
        || !config.min_clearance_m.is_finite()
        || !config.max_terrain_interaction_altitude_m.is_finite()
        || config.max_ray_samples == 0
    {
        return Err(ModelNotApplicableReason::InvalidTerrainParameters {
            reason: "non-finite, non-positive, or zero-budget TerrainLosConfig parameters"
                .to_owned(),
        }
        .into());
    }

    let chord = p1_fixed_m - p0_fixed_m;
    let chord_len_m = sqrt(chord.length_squared());
    if chord_len_m <= 1.0e-3 {
        return Ok(None);
    }

    let Some((s_enter, s_exit)) = intersect_chord_with_terrain_shell(
        p0_fixed_m,
        p1_fixed_m,
        ellipsoid,
        config.max_terrain_interaction_altitude_m,
    ) else {
        return Ok(None);
    };

    let shell_chord_len_m = (s_exit - s_enter).max(0.0) * chord_len_m;
    let step_m = if let Some(dem_spacing_m) = dem.min_horizontal_cell_spacing_m(ellipsoid) {
        config.sample_step_m.min((0.5 * dem_spacing_m).max(1.0))
    } else {
        config.sample_step_m
    };

    let raw_steps = (shell_chord_len_m / step_m).ceil();
    if !raw_steps.is_finite() || raw_steps < 0.0 {
        return Err(ModelNotApplicableReason::InvalidTerrainParameters {
            reason: "non-finite terrain shell sample count".to_owned(),
        }
        .into());
    }
    let required_samples = (raw_steps as u64).max(2);
    if required_samples > u64::from(config.max_ray_samples) {
        return Err(ModelNotApplicableReason::TerrainRaySampleBudgetExceeded {
            chord_length_in_shell_m: shell_chord_len_m,
            sample_step_m: step_m,
            required_samples,
            max_ray_samples: config.max_ray_samples,
        }
        .into());
    }

    sample_chord_in_terrain_shell(
        epoch,
        p0_fixed_m,
        chord,
        s_enter,
        s_exit,
        required_samples as usize,
        ellipsoid,
        dem,
        config,
    )
}
