//! Geodesic H3 planetary coverage grid evaluation (`sim_analysis::evaluate_coverage_grid`).

use std::sync::{Arc, OnceLock};

use glam::DVec3;
use pyo3::{
    prelude::*,
    types::{PyDict, PyList},
};
use sim_analysis::{
    CoverageAssetSpec, CoverageGridReport, CoverageQuerySpec, CoverageRegionSpec,
    OccultationEnvironmentModel, VehicleSensorModel, evaluate_coverage_grid,
};
use sim_geo::BodyEllipsoid;
use sim_orbital::{
    BuiltinSolarError, BuiltinSolarSystem2026_2028, Sgp4CatalogTrajectory, Sgp4Mode,
    propagate_kepler_universal,
};
use sim_spacetime::{
    DataProvenance, DomainHasher, DynamicalModelIdentity, FrameAxesSpec, FramedState, NaifId,
    StateFrameSpec, TdbEpoch, TrajectoryQueryError, TranslationalTrajectorySource, UnitSemantics,
    ValidEpochInterval,
};

use crate::{
    error::{SimPyError, require_finite, require_finite_vec3},
    orbital::{Sgp4Satellite, build_sgp4_from_omm},
    time::parse_utc_iso_to_tdb,
};

static BUILTIN_SOLAR: OnceLock<Result<Arc<BuiltinSolarSystem2026_2028>, BuiltinSolarError>> =
    OnceLock::new();

fn shared_solar_system() -> Result<&'static Arc<BuiltinSolarSystem2026_2028>, SimPyError> {
    let loaded = BUILTIN_SOLAR.get_or_init(|| {
        tracing::debug!("loading verified embedded 2026-2028 solar-system kernels");
        BuiltinSolarSystem2026_2028::load_verified().map(Arc::new)
    });
    match loaded {
        Ok(solar) => Ok(solar),
        Err(err) => Err(SimPyError::BuiltinSolar(err.clone())),
    }
}

struct KeplerTrajectorySource {
    trajectory_id: u64,
    anchor_epoch: TdbEpoch,
    r0_m: DVec3,
    v0_m_s: DVec3,
    mu_m3_s2: f64,
    provenance: DataProvenance,
}

impl KeplerTrajectorySource {
    fn new(
        trajectory_id: u64,
        anchor_epoch: TdbEpoch,
        r0_m: DVec3,
        v0_m_s: DVec3,
        valid_interval: ValidEpochInterval,
    ) -> Self {
        let mu_m3_s2 = BodyEllipsoid::WGS84.mu_m3_s2;
        let mut h = DomainHasher::new(b"sim-py:kepler-coverage-source:v1");
        h.write_u64(trajectory_id);
        h.write_i64(anchor_epoch.j2000_seconds());
        h.write_u64(anchor_epoch.subsec_ticks());
        h.write_vec3(r0_m);
        h.write_vec3(v0_m_s);
        h.write_f64_canonical(mu_m3_s2);
        let provenance = DataProvenance::new("sim-py/kepler", "v1", valid_interval, h.finish());
        Self {
            trajectory_id,
            anchor_epoch,
            r0_m,
            v0_m_s,
            mu_m3_s2,
            provenance,
        }
    }
}

impl TranslationalTrajectorySource for KeplerTrajectorySource {
    fn trajectory_id(&self) -> u64 {
        self.trajectory_id
    }

    fn model_identity(&self) -> DynamicalModelIdentity {
        DynamicalModelIdentity::PlanetaryKinematicOrFlight {
            body: NaifId::EARTH,
        }
    }

    fn provenance(&self) -> &DataProvenance {
        &self.provenance
    }

    fn sample_translational_icrf(
        &self,
        epoch: TdbEpoch,
    ) -> Result<FramedState, TrajectoryQueryError> {
        if !self.provenance.valid_interval.contains(epoch) {
            return Err(TrajectoryQueryError::EpochOutOfCoverage {
                requested_epoch: epoch,
                valid_start: self.provenance.valid_interval.start_epoch,
                valid_end: self.provenance.valid_interval.end_epoch,
            });
        }
        let dt_s = epoch.signed_offset_seconds_from(self.anchor_epoch);
        let (pos, vel) = propagate_kepler_universal(self.r0_m, self.v0_m_s, dt_s, self.mu_m3_s2);
        Ok(FramedState {
            spec: StateFrameSpec {
                epoch,
                origin: NaifId::EARTH,
                axes: FrameAxesSpec::IcrfInertial,
                units: UnitSemantics::SiMetric,
            },
            pos,
            vel,
        })
    }
}

enum OwnedTrajectory {
    Sgp4(Box<Sgp4CatalogTrajectory>),
    Kepler(KeplerTrajectorySource),
}

impl OwnedTrajectory {
    fn as_trait(&self) -> &dyn TranslationalTrajectorySource {
        match self {
            Self::Sgp4(t) => t.as_ref(),
            Self::Kepler(t) => t,
        }
    }
}

/// Evaluates N-fold geodesic H3 Earth coverage and revisit statistics over `[start_utc, start_utc + duration_s]`.
#[pyfunction]
#[pyo3(signature = (
    satellites,
    start_utc,
    duration_s,
    sample_step_s,
    resolution,
    min_elevation_deg = 10.0,
    min_fold = 1,
    lat_min_deg = None,
    lat_max_deg = None,
    lon_min_deg = None,
    lon_max_deg = None,
    max_cells = 10_000,
))]
#[allow(clippy::too_many_arguments)]
pub fn evaluate_h3_coverage(
    py: Python<'_>,
    satellites: Vec<Bound<'_, PyAny>>,
    start_utc: &str,
    duration_s: f64,
    sample_step_s: f64,
    resolution: u8,
    min_elevation_deg: f64,
    min_fold: usize,
    lat_min_deg: Option<f64>,
    lat_max_deg: Option<f64>,
    lon_min_deg: Option<f64>,
    lon_max_deg: Option<f64>,
    max_cells: usize,
) -> PyResult<Py<PyDict>> {
    let start_tdb = parse_utc_iso_to_tdb(start_utc)?;
    let dur = require_finite(duration_s, "duration_s")?;
    if dur < 0.0 {
        return Err(SimPyError::InvalidInput("`duration_s` must be >= 0".to_owned()).into());
    }
    let step = require_finite(sample_step_s, "sample_step_s")?;
    let min_el = require_finite(min_elevation_deg, "min_elevation_deg")?.to_radians();
    let end_tdb = start_tdb.advanced_seconds(dur);
    let interval = ValidEpochInterval::new(start_tdb, end_tdb);
    let region = build_region_spec(
        resolution,
        lat_min_deg,
        lat_max_deg,
        lon_min_deg,
        lon_max_deg,
    )?;
    let owned_trajs = extract_satellites(&satellites, start_tdb, interval)?;
    let sensor = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let query = CoverageQuerySpec {
        body: NaifId::EARTH,
        ellipsoid: BodyEllipsoid::WGS84,
        region,
        min_elevation_rad: min_el,
        min_fold,
        query_interval: interval,
        sample_step_s: step,
        occultation_env: OccultationEnvironmentModel::earth_wgs84(),
        max_cells,
    };
    let solar = shared_solar_system()?;
    let report = py.detach(|| -> Result<CoverageGridReport, SimPyError> {
        let assets: Vec<CoverageAssetSpec<'_>> = owned_trajs
            .iter()
            .map(|traj| CoverageAssetSpec {
                trajectory: traj.as_trait(),
                attitude: None,
                sensor: &sensor,
            })
            .collect();
        Ok(evaluate_coverage_grid(
            &assets,
            &query,
            &solar.ephemeris,
            &solar.orientation,
        )?)
    })?;
    report_to_dict(py, &report)
}

fn build_region_spec(
    resolution: u8,
    lat_min_deg: Option<f64>,
    lat_max_deg: Option<f64>,
    lon_min_deg: Option<f64>,
    lon_max_deg: Option<f64>,
) -> Result<CoverageRegionSpec, SimPyError> {
    match (lat_min_deg, lat_max_deg, lon_min_deg, lon_max_deg) {
        (None, None, None, None) => Ok(CoverageRegionSpec::Global { resolution }),
        (Some(lat_min), Some(lat_max), Some(lon_min), Some(lon_max)) => {
            Ok(CoverageRegionSpec::LatLonBox {
                resolution,
                min_lat_rad: require_finite(lat_min, "lat_min_deg")?.to_radians(),
                max_lat_rad: require_finite(lat_max, "lat_max_deg")?.to_radians(),
                min_lon_rad: require_finite(lon_min, "lon_min_deg")?.to_radians(),
                max_lon_rad: require_finite(lon_max, "lon_max_deg")?.to_radians(),
            })
        }
        _ => Err(SimPyError::InvalidInput(
            "provide either all four lat/lon bounds or none for global H3 coverage".to_owned(),
        )),
    }
}

fn extract_satellites(
    satellites: &[Bound<'_, PyAny>],
    anchor_epoch: TdbEpoch,
    interval: ValidEpochInterval,
) -> Result<Vec<OwnedTrajectory>, SimPyError> {
    let mut out = Vec::with_capacity(satellites.len());
    for (idx, item) in satellites.iter().enumerate() {
        let traj_id = (idx as u64).wrapping_add(1);
        if let Ok(sat) = item.extract::<PyRef<'_, Sgp4Satellite>>() {
            out.push(OwnedTrajectory::Sgp4(Box::new(sat.trajectory().clone())));
        } else if let Ok(omm_text) = item.extract::<String>() {
            let (traj, _) = build_sgp4_from_omm(&omm_text, Sgp4Mode::ModernWgs84Iau)?;
            out.push(OwnedTrajectory::Sgp4(Box::new(traj)));
        } else if let Ok((r_m, v_m_s)) = item.extract::<([f64; 3], [f64; 3])>() {
            let r0 = require_finite_vec3(r_m, "satellite.r_m")?;
            let v0 = require_finite_vec3(v_m_s, "satellite.v_m_s")?;
            out.push(OwnedTrajectory::Kepler(KeplerTrajectorySource::new(
                traj_id,
                anchor_epoch,
                r0,
                v0,
                interval,
            )));
        } else {
            return Err(SimPyError::InvalidInput(format!(
                "satellite[{idx}] must be an OMM string, Sgp4Satellite, or (r_m, v_m_s) tuple"
            )));
        }
    }
    Ok(out)
}

fn report_to_dict(py: Python<'_>, report: &CoverageGridReport) -> PyResult<Py<PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("total_cells", report.total_cells)?;
    dict.set_item("mean_coverage_fraction", report.mean_coverage_fraction)?;
    dict.set_item("min_coverage_fraction", report.min_coverage_fraction)?;
    dict.set_item("global_max_revisit_gap_s", report.global_max_revisit_gap_s)?;
    dict.set_item("n_fold_fraction", report.n_fold_fraction)?;

    let cells = PyList::empty(py);
    for cell in &report.cells {
        let cd = PyDict::new(py);
        cd.set_item("h3_index", cell.h3_index)?;
        cd.set_item("h3_hex", format!("{:016x}", cell.h3_index))?;
        cd.set_item("lat_deg", cell.lat_rad.to_degrees())?;
        cd.set_item("lon_deg", cell.lon_rad.to_degrees())?;
        cd.set_item("covered_samples", cell.covered_samples)?;
        cd.set_item("total_samples", cell.total_samples)?;
        cd.set_item("coverage_fraction", cell.coverage_fraction)?;
        cd.set_item("min_fold_observed", cell.min_fold_observed)?;
        cd.set_item("max_fold_observed", cell.max_fold_observed)?;
        cd.set_item("max_revisit_gap_s", cell.max_revisit_gap_s)?;
        cd.set_item("mean_revisit_gap_s", cell.mean_revisit_gap_s)?;
        cells.append(cd)?;
    }
    dict.set_item("cells", cells)?;
    Ok(dict.unbind())
}

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(evaluate_h3_coverage, m)?)?;
    Ok(())
}
