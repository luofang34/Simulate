//! CCSDS Attitude (`APM`, `AEM`), Conjunction (`CDM`), and Tracking (`TDM`) parsers.

use ccsds_ndm::{Aem, AemAttitudeState, Apm, Cdm, CdmSegment, Tdm, WriteOptions};
use pyo3::{
    prelude::*,
    types::{PyDict, PyList},
};

use super::ccsds_odm::{detect_format_kind, parse_mode};
use crate::error::SimPyError;

/// Parses a CCSDS `APM` (auto-detecting KVN or XML) and returns attitude state, inertia, and round-trip strings.
#[pyfunction]
#[pyo3(signature = (text, strict = true))]
pub fn parse_ccsds_apm(py: Python<'_>, text: &str, strict: bool) -> PyResult<Py<PyDict>> {
    let mode = parse_mode(strict);
    let source_format = detect_format_kind(text)?;
    let apm = match source_format {
        "xml" => Apm::from_xml(text, mode).map_err(SimPyError::from)?,
        "kvn" => Apm::from_kvn(text, mode).map_err(SimPyError::from)?,
        other => {
            return Err(SimPyError::InvalidInput(format!(
                "unsupported APM format `{other}` (expected KVN or XML)"
            ))
            .into());
        }
    };
    let opts = WriteOptions::default();
    let kvn = apm.to_kvn(opts).map_err(SimPyError::from)?;
    let xml = apm.to_xml(opts).map_err(SimPyError::from)?;

    let dict = PyDict::new(py);
    dict.set_item("message_type", "APM")?;
    dict.set_item("source_format", source_format)?;
    dict.set_item("version", apm.version.as_str())?;
    dict.set_item("originator", apm.header.originator.as_deref())?;
    dict.set_item("object_name", &apm.metadata.object_name)?;
    dict.set_item("object_id", &apm.metadata.object_id)?;
    dict.set_item(
        "center_name",
        apm.metadata.center_name.as_ref().map(|c| c.as_str()),
    )?;
    dict.set_item("time_system", apm.metadata.time_system.as_str())?;
    populate_apm_data(py, &dict, &apm)?;
    dict.set_item("kvn", kvn)?;
    dict.set_item("xml", xml)?;
    Ok(dict.unbind())
}

fn populate_apm_data(py: Python<'_>, dict: &Bound<'_, PyDict>, apm: &Apm) -> PyResult<()> {
    let epoch = apm
        .data
        .epoch
        .as_ref()
        .or_else(|| apm.data.quaternion.as_ref().and_then(|q| q.epoch.as_ref()))
        .or_else(|| apm.data.euler.as_ref().and_then(|e| e.epoch.as_ref()))
        .or_else(|| apm.data.ang_vel.as_ref().and_then(|a| a.epoch.as_ref()));
    dict.set_item("epoch", epoch.map(ToString::to_string))?;
    if let Some(q) = &apm.data.quaternion {
        let qd = PyDict::new(py);
        qd.set_item("ref_frame_a", q.ref_frame_a.as_str())?;
        qd.set_item("ref_frame_b", q.ref_frame_b.as_str())?;
        qd.set_item("q1", q.quaternion.q1)?;
        qd.set_item("q2", q.quaternion.q2)?;
        qd.set_item("q3", q.quaternion.q3)?;
        qd.set_item("qc", q.quaternion.qc)?;
        dict.set_item("quaternion", qd)?;
    } else {
        dict.set_item("quaternion", py.None())?;
    }
    dict.set_item(
        "ang_vel_deg_s",
        apm.data.ang_vel.as_ref().map(|av| {
            let [wx, wy, wz] = av.vector;
            [wx.value(), wy.value(), wz.value()]
        }),
    )?;
    if let Some(inertia) = &apm.data.inertia {
        let id = PyDict::new(py);
        id.set_item("ixx", inertia.ixx.value())?;
        id.set_item("iyy", inertia.iyy.value())?;
        id.set_item("izz", inertia.izz.value())?;
        id.set_item("ixy", inertia.ixy.value())?;
        id.set_item("ixz", inertia.ixz.value())?;
        id.set_item("iyz", inertia.iyz.value())?;
        dict.set_item("inertia_kg_m2", id)?;
    } else {
        dict.set_item("inertia_kg_m2", py.None())?;
    }
    dict.set_item("maneuver_count", apm.data.maneuvers.len())?;
    Ok(())
}

/// Parses a CCSDS `AEM` (auto-detecting KVN or XML) and returns attitude segments and round-trip strings.
#[pyfunction]
#[pyo3(signature = (text, strict = true))]
pub fn parse_ccsds_aem(py: Python<'_>, text: &str, strict: bool) -> PyResult<Py<PyDict>> {
    let mode = parse_mode(strict);
    let source_format = detect_format_kind(text)?;
    let aem = match source_format {
        "xml" => Aem::from_xml(text, mode).map_err(SimPyError::from)?,
        "kvn" => Aem::from_kvn(text, mode).map_err(SimPyError::from)?,
        other => {
            return Err(SimPyError::InvalidInput(format!(
                "unsupported AEM format `{other}` (expected KVN or XML)"
            ))
            .into());
        }
    };
    let opts = WriteOptions::default();
    let kvn = aem.to_kvn(opts).map_err(SimPyError::from)?;
    let xml = aem.to_xml(opts).map_err(SimPyError::from)?;

    let dict = PyDict::new(py);
    dict.set_item("message_type", "AEM")?;
    dict.set_item("source_format", source_format)?;
    dict.set_item("version", aem.version.as_str())?;
    dict.set_item("originator", aem.header.originator.as_deref())?;
    dict.set_item("segment_count", aem.segments.len())?;
    dict.set_item("segments", build_aem_segments(py, &aem)?)?;
    dict.set_item("kvn", kvn)?;
    dict.set_item("xml", xml)?;
    Ok(dict.unbind())
}

fn build_aem_segments<'py>(py: Python<'py>, aem: &Aem) -> PyResult<Bound<'py, PyList>> {
    let seg_list = PyList::empty(py);
    for seg in &aem.segments {
        let sd = PyDict::new(py);
        sd.set_item("object_name", &seg.metadata.object_name)?;
        sd.set_item("object_id", &seg.metadata.object_id)?;
        sd.set_item("ref_frame_a", seg.metadata.ref_frame_a.as_str())?;
        sd.set_item("ref_frame_b", seg.metadata.ref_frame_b.as_str())?;
        sd.set_item("time_system", seg.metadata.time_system.as_str())?;
        sd.set_item("start_time", seg.metadata.start_time.to_string())?;
        sd.set_item("stop_time", seg.metadata.stop_time.to_string())?;
        sd.set_item("attitude_type", seg.metadata.attitude_type.as_str())?;
        sd.set_item("state_count", seg.data.states.len())?;
        let states = PyList::empty(py);
        for st in &seg.data.states {
            let entry = PyDict::new(py);
            entry.set_item("epoch", st.epoch.to_string())?;
            populate_aem_state(&entry, &st.state)?;
            states.append(entry)?;
        }
        sd.set_item("states", states)?;
        seg_list.append(sd)?;
    }
    Ok(seg_list)
}

fn populate_aem_state(entry: &Bound<'_, PyDict>, state: &AemAttitudeState) -> PyResult<()> {
    match *state {
        AemAttitudeState::Quaternion { quaternion } => {
            entry.set_item("quaternion_xyzw", quaternion.to_scalar_last())?;
        }
        AemAttitudeState::QuaternionAngVel {
            quaternion,
            ang_vel_deg_s,
        } => {
            entry.set_item("quaternion_xyzw", quaternion.to_scalar_last())?;
            entry.set_item("ang_vel_deg_s", ang_vel_deg_s)?;
        }
        AemAttitudeState::QuaternionDerivative { quaternion, q_dot } => {
            entry.set_item("quaternion_xyzw", quaternion.to_scalar_last())?;
            entry.set_item("q_dot", q_dot)?;
        }
        AemAttitudeState::QuaternionRate {
            quaternion,
            rates_deg_s,
        } => {
            entry.set_item("quaternion_xyzw", quaternion.to_scalar_last())?;
            entry.set_item("rates_deg_s", rates_deg_s)?;
        }
        AemAttitudeState::EulerAngle { angles_deg }
        | AemAttitudeState::EulerAngleDerivative { angles_deg, .. }
        | AemAttitudeState::EulerAngleAngVel { angles_deg, .. }
        | AemAttitudeState::EulerAngleRate { angles_deg, .. } => {
            entry.set_item("angles_deg", angles_deg)?;
        }
        _ => {}
    }
    Ok(())
}

/// Parses a CCSDS `CDM` (auto-detecting KVN or XML) and returns encounter data and round-trip strings.
#[pyfunction]
#[pyo3(signature = (text, strict = true))]
pub fn parse_ccsds_cdm(py: Python<'_>, text: &str, strict: bool) -> PyResult<Py<PyDict>> {
    let mode = parse_mode(strict);
    let source_format = detect_format_kind(text)?;
    let cdm = match source_format {
        "xml" => Cdm::from_xml(text, mode).map_err(SimPyError::from)?,
        "kvn" => Cdm::from_kvn(text, mode).map_err(SimPyError::from)?,
        other => {
            return Err(SimPyError::InvalidInput(format!(
                "unsupported CDM format `{other}` (expected KVN or XML)"
            ))
            .into());
        }
    };
    let opts = WriteOptions::default();
    let kvn = cdm.to_kvn(opts).map_err(SimPyError::from)?;
    let xml = cdm.to_xml(opts).map_err(SimPyError::from)?;

    let dict = PyDict::new(py);
    dict.set_item("message_type", "CDM")?;
    dict.set_item("source_format", source_format)?;
    dict.set_item("version", cdm.version.as_str())?;
    dict.set_item("originator", &cdm.header.originator)?;
    dict.set_item("message_id", &cdm.header.message_id)?;
    dict.set_item("message_for", cdm.header.message_for.as_deref())?;
    dict.set_item("tca", cdm.relative_metadata.tca.to_string())?;
    dict.set_item(
        "miss_distance_m",
        cdm.relative_metadata.miss_distance.value(),
    )?;
    dict.set_item(
        "relative_speed_m_s",
        cdm.relative_metadata.relative_speed.map(|s| s.value()),
    )?;
    dict.set_item(
        "collision_probability",
        cdm.relative_metadata.collision_probability,
    )?;
    dict.set_item(
        "collision_probability_method",
        cdm.relative_metadata
            .collision_probability_method
            .as_deref(),
    )?;
    dict.set_item("object1", build_cdm_segment(py, &cdm.object1)?)?;
    dict.set_item("object2", build_cdm_segment(py, &cdm.object2)?)?;
    dict.set_item("kvn", kvn)?;
    dict.set_item("xml", xml)?;
    Ok(dict.unbind())
}

fn build_cdm_segment<'py>(py: Python<'py>, seg: &CdmSegment) -> PyResult<Bound<'py, PyDict>> {
    let sd = PyDict::new(py);
    sd.set_item("object_name", &seg.metadata.object_name)?;
    sd.set_item("object_designator", &seg.metadata.object_designator)?;
    sd.set_item("catalog_name", &seg.metadata.catalog_name)?;
    sd.set_item("ref_frame", seg.metadata.ref_frame.as_str())?;
    let [px, py_km, pz] = seg.data.state_vector.position;
    let [vx, vy, vz] = seg.data.state_vector.velocity;
    sd.set_item("position_km", [px.value(), py_km.value(), pz.value()])?;
    sd.set_item("velocity_km_s", [vx.value(), vy.value(), vz.value()])?;
    sd.set_item("covariance_6x6_rtn", seg.data.covariance.matrix.to_matrix())?;
    Ok(sd)
}

/// Parses a CCSDS `TDM` (auto-detecting KVN or XML) and returns tracking segments, observations, and round-trip strings.
#[pyfunction]
#[pyo3(signature = (text, strict = true))]
pub fn parse_ccsds_tdm(py: Python<'_>, text: &str, strict: bool) -> PyResult<Py<PyDict>> {
    let mode = parse_mode(strict);
    let source_format = detect_format_kind(text)?;
    let tdm = match source_format {
        "xml" => Tdm::from_xml(text, mode).map_err(SimPyError::from)?,
        "kvn" => Tdm::from_kvn(text, mode).map_err(SimPyError::from)?,
        other => {
            return Err(SimPyError::InvalidInput(format!(
                "unsupported TDM format `{other}` (expected KVN or XML)"
            ))
            .into());
        }
    };
    let opts = WriteOptions::default();
    let kvn = tdm.to_kvn(opts).map_err(SimPyError::from)?;
    let xml = tdm.to_xml(opts).map_err(SimPyError::from)?;

    let dict = PyDict::new(py);
    dict.set_item("message_type", "TDM")?;
    dict.set_item("source_format", source_format)?;
    dict.set_item("version", tdm.version.as_str())?;
    dict.set_item("originator", tdm.header.originator.as_deref())?;
    dict.set_item("segment_count", tdm.segments.len())?;
    dict.set_item("segments", build_tdm_segments(py, &tdm)?)?;
    dict.set_item("kvn", kvn)?;
    dict.set_item("xml", xml)?;
    Ok(dict.unbind())
}

fn build_tdm_segments<'py>(py: Python<'py>, tdm: &Tdm) -> PyResult<Bound<'py, PyList>> {
    let seg_list = PyList::empty(py);
    for seg in &tdm.segments {
        let sd = PyDict::new(py);
        sd.set_item("track_id", seg.metadata.track_id.as_deref())?;
        sd.set_item("time_system", seg.metadata.time_system.as_str())?;
        sd.set_item(
            "start_time",
            seg.metadata.start_time.as_ref().map(ToString::to_string),
        )?;
        sd.set_item(
            "stop_time",
            seg.metadata.stop_time.as_ref().map(ToString::to_string),
        )?;
        let participants: Vec<&str> = seg
            .metadata
            .participants
            .iter()
            .filter_map(|p| p.as_deref())
            .collect();
        sd.set_item("participants", participants)?;
        sd.set_item("mode", seg.metadata.mode.as_ref().map(|m| m.as_str()))?;
        sd.set_item(
            "range_units",
            seg.metadata.range_units.as_ref().map(|r| r.as_str()),
        )?;
        sd.set_item(
            "angle_type",
            seg.metadata.angle_type.as_ref().map(|a| a.as_str()),
        )?;
        let obs_list = PyList::empty(py);
        for obs in &seg.data.observations {
            let od = PyDict::new(py);
            od.set_item("epoch", obs.epoch.to_string())?;
            od.set_item("observable", obs.kind.as_str())?;
            od.set_item("value", obs.value)?;
            obs_list.append(od)?;
        }
        sd.set_item("observations", obs_list)?;
        seg_list.append(sd)?;
    }
    Ok(seg_list)
}

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(parse_ccsds_apm, m)?)?;
    m.add_function(wrap_pyfunction!(parse_ccsds_aem, m)?)?;
    m.add_function(wrap_pyfunction!(parse_ccsds_cdm, m)?)?;
    m.add_function(wrap_pyfunction!(parse_ccsds_tdm, m)?)?;
    Ok(())
}
