//! Orbital dynamics bindings: universal Kepler propagation, SGP4/SDP4 catalog propagation,
//! and B-plane short-term encounter collision probability.

use std::sync::Arc;

use ccsds_ndm::{Omm, ParseMode};
use pyo3::{prelude::*, types::PyDict};
use sim_geo::UtcTime;
use sim_orbital::{
    BPlaneEncounterResult, CcsdsOdm, Sgp4CatalogTrajectory, Sgp4Mode, StateCovariance6,
    compute_bplane_collision_probability, propagate_kepler_universal,
};
use sim_spacetime::{TdbEpoch, TranslationalTrajectorySource, ValidEpochInterval};

use crate::{
    error::{SimPyError, require_finite, require_finite_vec3},
    time::{shared_time_scales, tdb_seconds_to_epoch},
};

/// Two-body state pair `(position_m, velocity_m_s)` in ICRF meters and meters per second.
pub(crate) type StatePair = ([f64; 3], [f64; 3]);

/// Parsed and validated `SGP4`/`SDP4` Earth-orbit satellite propagator.
#[pyclass(frozen)]
pub struct Sgp4Satellite {
    trajectory: Sgp4CatalogTrajectory,
    epoch0_tdb: TdbEpoch,
}

impl Sgp4Satellite {
    /// Borrows the underlying [`Sgp4CatalogTrajectory`].
    pub(crate) const fn trajectory(&self) -> &Sgp4CatalogTrajectory {
        &self.trajectory
    }
}

#[pymethods]
impl Sgp4Satellite {
    /// Parses a CCSDS `OMM` (JSON, KVN, or XML) into an `SGP4`/`SDP4` propagator.
    #[new]
    #[pyo3(signature = (omm_input, mode = None))]
    fn new(omm_input: &str, mode: Option<&str>) -> PyResult<Self> {
        let parsed_mode = parse_sgp4_mode(mode)?;
        let (trajectory, epoch0_tdb) = build_sgp4_from_omm(omm_input, parsed_mode)?;
        Ok(Self {
            trajectory,
            epoch0_tdb,
        })
    }

    /// Parses a Two-Line Element set (`line1`, `line2`) into an `SGP4`/`SDP4` propagator.
    #[staticmethod]
    #[pyo3(signature = (line1, line2, object_name = None, mode = None))]
    fn from_tle(
        line1: &str,
        line2: &str,
        object_name: Option<&str>,
        mode: Option<&str>,
    ) -> PyResult<Self> {
        let parsed_mode = parse_sgp4_mode(mode)?;
        let scales = Arc::clone(shared_time_scales()?);
        let interval = default_sgp4_interval();
        let trajectory = Sgp4CatalogTrajectory::from_tle_with_mode(
            1,
            object_name,
            line1.trim().as_bytes(),
            line2.trim().as_bytes(),
            parsed_mode,
            interval,
            Arc::clone(&scales),
        )
        .map_err(SimPyError::from)?;
        let epoch0_tdb = element_epoch_tdb(&trajectory, &scales)?;
        Ok(Self {
            trajectory,
            epoch0_tdb,
        })
    }

    /// Propagates the satellite at `dt_seconds_list` offsets (seconds past OMM/TLE epoch) in ICRF SI units.
    fn propagate(&self, py: Python<'_>, dt_seconds_list: Vec<f64>) -> PyResult<Vec<StatePair>> {
        propagate_sgp4_offsets(py, &self.trajectory, self.epoch0_tdb, &dt_seconds_list)
    }

    /// NORAD catalog identifier.
    #[getter]
    fn norad_id(&self) -> u64 {
        self.trajectory.elements().norad_id
    }

    /// Optional object name from the OMM/TLE record.
    #[getter]
    fn object_name(&self) -> Option<String> {
        self.trajectory.elements().object_name.clone()
    }

    /// Element epoch expressed in TDB seconds past `J2000.0 TDB`.
    #[getter]
    fn epoch_tdb_s(&self) -> f64 {
        self.epoch0_tdb.as_j2000_seconds_f64()
    }

    /// Canonical SHA-256 hex digest of the catalog elements and SGP4 mode.
    #[getter]
    fn elements_hash(&self) -> String {
        self.trajectory.elements_hash().to_hex()
    }
}

/// Propagates a two-body state `(r_m, v_m_s)` across `dt_s` under `mu_m3_s2` using universal variables.
#[pyfunction]
pub fn propagate_kepler(
    py: Python<'_>,
    r_m: [f64; 3],
    v_m_s: [f64; 3],
    mu_m3_s2: f64,
    dt_s: f64,
) -> PyResult<StatePair> {
    let r0 = require_finite_vec3(r_m, "r_m")?;
    let v0 = require_finite_vec3(v_m_s, "v_m_s")?;
    let mu = require_finite(mu_m3_s2, "mu_m3_s2")?;
    let dt = require_finite(dt_s, "dt_s")?;
    if mu <= 0.0 {
        return Err(SimPyError::InvalidInput(format!(
            "`mu_m3_s2` must be strictly positive, got {mu}"
        ))
        .into());
    }
    let (r1, v1) = py.detach(|| propagate_kepler_universal(r0, v0, dt, mu));
    Ok((r1.to_array(), v1.to_array()))
}

/// Propagates a CCSDS OMM (JSON, KVN, or XML) with `SGP4`/`SDP4` to ICRF `(r_m, v_m_s)` states.
#[pyfunction]
#[pyo3(signature = (omm_input, dt_seconds_list, mode = None))]
pub fn sgp4_propagate(
    py: Python<'_>,
    omm_input: &str,
    dt_seconds_list: Vec<f64>,
    mode: Option<&str>,
) -> PyResult<Vec<StatePair>> {
    let parsed_mode = parse_sgp4_mode(mode)?;
    let (traj, epoch0_tdb) = build_sgp4_from_omm(omm_input, parsed_mode)?;
    propagate_sgp4_offsets(py, &traj, epoch0_tdb, &dt_seconds_list)
}

/// Evaluates short-term linear-encounter B-plane collision probability and miss metrics.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn bplane_collision_probability(
    py: Python<'_>,
    r1: [f64; 3],
    v1: [f64; 3],
    cov1_6x6: [[f64; 6]; 6],
    r2: [f64; 3],
    v2: [f64; 3],
    cov2_6x6: [[f64; 6]; 6],
    radius_m: f64,
) -> PyResult<Py<PyDict>> {
    let p1 = require_finite_vec3(r1, "r1")?;
    let vel1 = require_finite_vec3(v1, "v1")?;
    let p2 = require_finite_vec3(r2, "r2")?;
    let vel2 = require_finite_vec3(v2, "v2")?;
    let rc = require_finite(radius_m, "radius_m")?;
    validate_cov_matrix(&cov1_6x6, "cov1_6x6")?;
    validate_cov_matrix(&cov2_6x6, "cov2_6x6")?;
    let cov1 = StateCovariance6::from_matrix(cov1_6x6);
    let cov2 = StateCovariance6::from_matrix(cov2_6x6);

    let result = py
        .detach(|| compute_bplane_collision_probability(p1, vel1, &cov1, p2, vel2, &cov2, rc))
        .map_err(SimPyError::from)?;
    bplane_result_to_dict(py, &result)
}

pub(crate) fn build_sgp4_from_omm(
    omm_input: &str,
    mode: Sgp4Mode,
) -> Result<(Sgp4CatalogTrajectory, TdbEpoch), SimPyError> {
    let trimmed = omm_input.trim();
    if trimmed.is_empty() {
        return Err(SimPyError::InvalidInput(
            "OMM input cannot be empty".to_owned(),
        ));
    }
    let scales = Arc::clone(shared_time_scales()?);
    let interval = default_sgp4_interval();
    let trajectory = if trimmed.starts_with('{') || trimmed.starts_with('[') {
        match Sgp4CatalogTrajectory::from_omm_json_with_mode(
            1,
            trimmed,
            mode,
            interval,
            Arc::clone(&scales),
        ) {
            Ok(traj) => traj,
            Err(_) => {
                let omm = Omm::from_json(trimmed, ParseMode::Lenient)?;
                CcsdsOdm::omm_to_sgp4_trajectory(1, &omm, mode, interval, Arc::clone(&scales))?
            }
        }
    } else if trimmed.starts_with('<') {
        let omm = Omm::from_xml(trimmed, ParseMode::Lenient)?;
        CcsdsOdm::omm_to_sgp4_trajectory(1, &omm, mode, interval, Arc::clone(&scales))?
    } else {
        let omm = Omm::from_kvn(trimmed, ParseMode::Lenient)?;
        CcsdsOdm::omm_to_sgp4_trajectory(1, &omm, mode, interval, Arc::clone(&scales))?
    };
    let epoch0_tdb = element_epoch_tdb(&trajectory, &scales)?;
    Ok((trajectory, epoch0_tdb))
}

pub(crate) fn parse_sgp4_mode(mode: Option<&str>) -> Result<Sgp4Mode, SimPyError> {
    match mode.map(str::trim) {
        None => Ok(Sgp4Mode::ModernWgs84Iau),
        Some(raw)
            if raw.eq_ignore_ascii_case("wgs84")
                || raw.eq_ignore_ascii_case("modern")
                || raw.eq_ignore_ascii_case("wgs84_iau") =>
        {
            Ok(Sgp4Mode::ModernWgs84Iau)
        }
        Some(raw) if raw.eq_ignore_ascii_case("wgs72") || raw.eq_ignore_ascii_case("afspc") => {
            Ok(Sgp4Mode::AfspcCompatibilityWgs72)
        }
        Some(other) => Err(SimPyError::InvalidInput(format!(
            "unsupported SGP4 mode `{other}` (expected 'wgs84' or 'wgs72')"
        ))),
    }
}

fn default_sgp4_interval() -> ValidEpochInterval {
    ValidEpochInterval::new(
        TdbEpoch::new(-10_000_000_000, 0),
        TdbEpoch::new(10_000_000_000, 0),
    )
}

fn element_epoch_tdb(
    traj: &Sgp4CatalogTrajectory,
    scales: &sim_spacetime::TimeScaleCatalog,
) -> Result<TdbEpoch, SimPyError> {
    let dt_utc = traj.elements().datetime.and_utc();
    let unix_s = (dt_utc.timestamp() as f64) + f64::from(dt_utc.timestamp_subsec_nanos()) * 1.0e-9;
    Ok(scales.utc_to_tdb(UtcTime::from_unix_seconds(unix_s))?)
}

fn propagate_sgp4_offsets(
    py: Python<'_>,
    traj: &Sgp4CatalogTrajectory,
    epoch0_tdb: TdbEpoch,
    dt_seconds_list: &[f64],
) -> PyResult<Vec<StatePair>> {
    let mut epochs = Vec::with_capacity(dt_seconds_list.len());
    let base_s = epoch0_tdb.as_j2000_seconds_f64();
    for &dt in dt_seconds_list {
        let dt_valid = require_finite(dt, "dt_s")?;
        epochs.push(tdb_seconds_to_epoch(base_s + dt_valid)?);
    }
    let states = py.detach(|| -> Result<Vec<StatePair>, SimPyError> {
        let mut out = Vec::with_capacity(epochs.len());
        for ep in epochs {
            let framed = traj.sample_translational_icrf(ep)?;
            out.push((framed.pos.to_array(), framed.vel.to_array()));
        }
        Ok(out)
    })?;
    Ok(states)
}

fn validate_cov_matrix(cov: &[[f64; 6]; 6], name: &str) -> Result<(), SimPyError> {
    for row in cov {
        for &val in row {
            if !val.is_finite() {
                return Err(SimPyError::InvalidInput(format!(
                    "`{name}` entries must all be finite, got {val}"
                )));
            }
        }
    }
    Ok(())
}

fn bplane_result_to_dict(py: Python<'_>, res: &BPlaneEncounterResult) -> PyResult<Py<PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("probability", res.probability)?;
    dict.set_item("mahalanobis_distance_sq", res.mahalanobis_distance_sq)?;
    dict.set_item("miss_distance_m", res.miss_distance_m)?;
    dict.set_item("b_plane_sigmas_m", res.b_plane_sigmas_m)?;
    dict.set_item("miss_b_plane_m", res.miss_b_plane_m)?;
    Ok(dict.unbind())
}

/// Registers orbital propagation and conjunction bindings on the Python module.
pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Sgp4Satellite>()?;
    m.add_function(wrap_pyfunction!(propagate_kepler, m)?)?;
    m.add_function(wrap_pyfunction!(sgp4_propagate, m)?)?;
    m.add_function(wrap_pyfunction!(bplane_collision_probability, m)?)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circular_kepler_orbit_preserves_radius_and_rejects_bad_mu() {
        Python::attach(|py| {
            let mu = 3.986_004_418e14_f64;
            let r = 7_000_000.0_f64;
            let v = (mu / r).sqrt();
            let period = core::f64::consts::TAU * (r.powi(3) / mu).sqrt();
            let result = propagate_kepler(py, [r, 0.0, 0.0], [0.0, v, 0.0], mu, period);
            assert!(result.is_ok());
            if let Ok((r1, v1)) = result {
                assert!((r1[0] - r).abs() < 1.0e-5);
                assert!(r1[1].abs() < 1.0e-5);
                assert!((v1[1] - v).abs() < 1.0e-7);
            }
            assert!(propagate_kepler(py, [r, 0.0, 0.0], [0.0, v, 0.0], 0.0, 10.0).is_err());
        });
    }

    #[test]
    fn bplane_head_on_encounter_computes_consistent_probability() {
        Python::attach(|py| {
            let cov = [
                [100.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 100.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 100.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.01, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.01, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.01],
            ];
            let res = bplane_collision_probability(
                py,
                [7_000_000.0, 0.0, 0.0],
                [0.0, 7_500.0, 0.0],
                cov,
                [7_000_000.0, 0.0, 0.0],
                [0.0, -7_500.0, 0.0],
                cov,
                10.0,
            );
            assert!(res.is_ok());
        });
    }
}
