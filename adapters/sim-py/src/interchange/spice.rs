//! NAIF SPICE binary kernel (`DAF/SPK`, `DAF/CK`) inspection, evaluation, and creation.

use pyo3::{
    prelude::*,
    types::{PyBytes, PyDict, PyList},
};
use spice_kernels::{
    CkFile, CkPointingInstance, CkSegment, CkType3, SampledStates, SpkFile, SpkSegment,
};

use crate::error::{SimPyError, require_finite};

/// Inspects a binary `DAF/SPK` kernel (`bytes`) and returns its header, comments, and segment descriptors.
#[pyfunction]
pub fn inspect_spice_spk(py: Python<'_>, data: &[u8]) -> PyResult<Py<PyDict>> {
    let spk = SpkFile::parse(data).map_err(SimPyError::from)?;
    let dict = PyDict::new(py);
    dict.set_item("internal_name", spk.internal_name())?;
    dict.set_item("comment_lines", spk.comment_lines())?;
    dict.set_item("segment_count", spk.segments().len())?;

    let segs = PyList::empty(py);
    for seg in spk.segments() {
        let [start_tdb_s, end_tdb_s] = seg.coverage();
        let sd = PyDict::new(py);
        sd.set_item("name", seg.name())?;
        sd.set_item("target", seg.target())?;
        sd.set_item("center", seg.center())?;
        sd.set_item("frame", seg.frame())?;
        sd.set_item("data_type", seg.data().data_type())?;
        sd.set_item("start_tdb_s", start_tdb_s)?;
        sd.set_item("end_tdb_s", end_tdb_s)?;
        segs.append(sd)?;
    }
    dict.set_item("segments", segs)?;
    Ok(dict.unbind())
}

/// Evaluates an `SPK` state `(pos_km, vel_km_s)` for `(target, center)` at `tdb_s` seconds past J2000.
#[pyfunction]
pub fn evaluate_spice_spk(
    data: &[u8],
    target: i32,
    center: i32,
    tdb_s: f64,
) -> PyResult<([f64; 3], [f64; 3])> {
    let et = require_finite(tdb_s, "tdb_s")?;
    let spk = SpkFile::parse(data).map_err(SimPyError::from)?;
    let [x, y, z, vx, vy, vz] = spk.state(target, center, et).map_err(SimPyError::from)?;
    Ok(([x, y, z], [vx, vy, vz]))
}

/// Builds a deterministic single-segment Type-13 (Hermite) `DAF/SPK` kernel as `bytes`.
#[pyfunction]
#[pyo3(signature = (internal_name, segment_name, target, center, frame, epochs_tdb_s, states_km_kms, window = 2, comments = None))]
#[allow(clippy::too_many_arguments)]
pub fn build_hermite_spk(
    py: Python<'_>,
    internal_name: &str,
    segment_name: &str,
    target: i32,
    center: i32,
    frame: i32,
    epochs_tdb_s: Vec<f64>,
    states_km_kms: Vec<[f64; 6]>,
    window: usize,
    comments: Option<Vec<String>>,
) -> PyResult<Py<PyBytes>> {
    let samples =
        SampledStates::new(epochs_tdb_s, states_km_kms, window).map_err(SimPyError::from)?;
    let seg = SpkSegment::hermite(segment_name, target, center, frame, samples)
        .map_err(SimPyError::from)?;
    let mut spk = SpkFile::new(internal_name).map_err(SimPyError::from)?;
    if let Some(lines) = comments {
        spk.set_comment_lines(lines).map_err(SimPyError::from)?;
    }
    spk.push(seg);
    let bytes = spk.to_bytes().map_err(SimPyError::from)?;
    Ok(PyBytes::new(py, &bytes).unbind())
}

/// Inspects a binary `DAF/CK` kernel (`bytes`) and returns its header, comments, and segment descriptors.
#[pyfunction]
pub fn inspect_spice_ck(py: Python<'_>, data: &[u8]) -> PyResult<Py<PyDict>> {
    let ck = CkFile::parse(data).map_err(SimPyError::from)?;
    let dict = PyDict::new(py);
    dict.set_item("internal_name", ck.internal_name())?;
    dict.set_item("comment_lines", ck.comment_lines())?;
    dict.set_item("segment_count", ck.segments().len())?;

    let segs = PyList::empty(py);
    for seg in ck.segments() {
        let [start_sclk, end_sclk] = seg.coverage();
        let sd = PyDict::new(py);
        sd.set_item("name", seg.name())?;
        sd.set_item("instrument", seg.instrument())?;
        sd.set_item("frame", seg.frame())?;
        sd.set_item("data_type", seg.data().data_type())?;
        sd.set_item("has_angular_velocity", seg.has_angular_velocity())?;
        sd.set_item("start_sclk", start_sclk)?;
        sd.set_item("end_sclk", end_sclk)?;
        segs.append(sd)?;
    }
    dict.set_item("segments", segs)?;
    Ok(dict.unbind())
}

/// Evaluates `CK` attitude quaternion `[q0, q1, q2, q3]`, C-matrix, and optional angular velocity at `sclk`.
#[pyfunction]
#[pyo3(signature = (data, instrument, sclk, tolerance = 0.0))]
pub fn evaluate_spice_ck(
    py: Python<'_>,
    data: &[u8],
    instrument: i32,
    sclk: f64,
    tolerance: f64,
) -> PyResult<Py<PyDict>> {
    let sclk_valid = require_finite(sclk, "sclk")?;
    let tol_valid = require_finite(tolerance, "tolerance")?;
    let ck = CkFile::parse(data).map_err(SimPyError::from)?;
    let pt = ck
        .pointing_and_angular_velocity(instrument, sclk_valid, tol_valid)
        .map_err(SimPyError::from)?;
    let dict = PyDict::new(py);
    dict.set_item("sclk", pt.sclk)?;
    dict.set_item("frame", pt.frame)?;
    dict.set_item("quaternion", pt.quaternion)?;
    dict.set_item("angular_velocity", pt.angular_velocity)?;
    dict.set_item("c_matrix", pt.c_matrix())?;
    Ok(dict.unbind())
}

/// Builds a deterministic single-interval Type-3 `DAF/CK` kernel as `bytes`.
#[pyfunction]
#[pyo3(signature = (internal_name, segment_name, instrument, frame, sclks, quaternions, angular_velocities = None, comments = None))]
#[allow(clippy::too_many_arguments)]
pub fn build_type3_ck(
    py: Python<'_>,
    internal_name: &str,
    segment_name: &str,
    instrument: i32,
    frame: i32,
    sclks: Vec<f64>,
    quaternions: Vec<[f64; 4]>,
    angular_velocities: Option<Vec<[f64; 3]>>,
    comments: Option<Vec<String>>,
) -> PyResult<Py<PyBytes>> {
    let instances = assemble_ck_instances(&sclks, &quaternions, angular_velocities.as_deref())?;
    let has_av = angular_velocities.is_some();
    let t3 = CkType3::single_interval(instances, has_av).map_err(SimPyError::from)?;
    let seg = CkSegment::type3(segment_name, instrument, frame, t3).map_err(SimPyError::from)?;
    let mut ck = CkFile::new(internal_name).map_err(SimPyError::from)?;
    if let Some(lines) = comments {
        ck.set_comment_lines(lines).map_err(SimPyError::from)?;
    }
    ck.push(seg);
    let bytes = ck.to_bytes().map_err(SimPyError::from)?;
    Ok(PyBytes::new(py, &bytes).unbind())
}

fn assemble_ck_instances(
    sclks: &[f64],
    quaternions: &[[f64; 4]],
    angular_velocities: Option<&[[f64; 3]]>,
) -> Result<Vec<CkPointingInstance>, SimPyError> {
    if sclks.len() != quaternions.len() {
        return Err(SimPyError::InvalidInput(format!(
            "sclks length ({}) must equal quaternions length ({})",
            sclks.len(),
            quaternions.len()
        )));
    }
    if let Some(avs) = angular_velocities
        && avs.len() != sclks.len()
    {
        return Err(SimPyError::InvalidInput(format!(
            "angular_velocities length ({}) must equal sclks length ({})",
            avs.len(),
            sclks.len()
        )));
    }
    let mut out = Vec::with_capacity(sclks.len());
    for (idx, (&sclk, &quaternion)) in sclks.iter().zip(quaternions.iter()).enumerate() {
        let angular_velocity = match angular_velocities {
            Some(avs) => avs.get(idx).copied(),
            None => None,
        };
        out.push(CkPointingInstance {
            sclk,
            quaternion,
            angular_velocity,
        });
    }
    Ok(out)
}

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(inspect_spice_spk, m)?)?;
    m.add_function(wrap_pyfunction!(evaluate_spice_spk, m)?)?;
    m.add_function(wrap_pyfunction!(build_hermite_spk, m)?)?;
    m.add_function(wrap_pyfunction!(inspect_spice_ck, m)?)?;
    m.add_function(wrap_pyfunction!(evaluate_spice_ck, m)?)?;
    m.add_function(wrap_pyfunction!(build_type3_ck, m)?)?;
    Ok(())
}
