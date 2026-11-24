//! CCSDS Orbit Data Message (`OPM`, `OMM`, `OEM`) parsers and SI-state interchange.

use ccsds_ndm::{
    Anomaly, Oem, Omm, Opm, OrbitSize, ParseMode, TleDrag, TleSecondTerm, WriteOptions,
};
use pyo3::{
    prelude::*,
    types::{PyDict, PyList},
};
use sim_orbital::CcsdsOdm;

use crate::{error::SimPyError, time::shared_time_scales};

const M_PER_KM: f64 = 1_000.0;

pub(super) fn parse_mode(strict: bool) -> ParseMode {
    if strict {
        ParseMode::Strict
    } else {
        ParseMode::Lenient
    }
}

pub(super) fn detect_format_kind(text: &str) -> Result<&'static str, SimPyError> {
    let trimmed = text.trim_start();
    if trimmed.is_empty() {
        return Err(SimPyError::InvalidInput(
            "CCSDS message input cannot be empty".to_owned(),
        ));
    }
    if trimmed.starts_with('<') {
        Ok("xml")
    } else if trimmed.starts_with('{') || trimmed.starts_with('[') {
        Ok("json")
    } else {
        Ok("kvn")
    }
}

fn vec3_values<U: ccsds_ndm::Unit>(vec: &[ccsds_ndm::Quantity<U>; 3]) -> [f64; 3] {
    let [x, y, z] = *vec;
    [x.value(), y.value(), z.value()]
}

fn parse_opm_message(text: &str, strict: bool) -> Result<(Opm, &'static str), SimPyError> {
    let mode = parse_mode(strict);
    match detect_format_kind(text)? {
        "xml" => Ok((Opm::from_xml(text, mode)?, "xml")),
        "kvn" => Ok((Opm::from_kvn(text, mode)?, "kvn")),
        other => Err(SimPyError::InvalidInput(format!(
            "unsupported OPM format `{other}` (expected KVN or XML)"
        ))),
    }
}

/// Parses a CCSDS `OPM` (auto-detecting KVN or XML) and returns extracted state, metadata, and round-trip strings.
#[pyfunction]
#[pyo3(signature = (text, strict = true))]
pub fn parse_ccsds_opm(py: Python<'_>, text: &str, strict: bool) -> PyResult<Py<PyDict>> {
    let (opm, source_format) = parse_opm_message(text, strict)?;
    let opts = WriteOptions::default();
    let kvn = opm.to_kvn(opts).map_err(SimPyError::from)?;
    let xml = opm.to_xml(opts).map_err(SimPyError::from)?;

    let dict = PyDict::new(py);
    dict.set_item("message_type", "OPM")?;
    dict.set_item("source_format", source_format)?;
    dict.set_item("version", opm.version.as_str())?;
    dict.set_item("originator", opm.header.originator.as_deref())?;
    dict.set_item(
        "creation_date",
        opm.header.creation_date.as_ref().map(ToString::to_string),
    )?;
    dict.set_item("object_name", &opm.metadata.object_name)?;
    dict.set_item("object_id", &opm.metadata.object_id)?;
    dict.set_item("center_name", opm.metadata.center_name.as_str())?;
    dict.set_item("ref_frame", opm.metadata.ref_frame.as_str())?;
    dict.set_item("time_system", opm.metadata.time_system.as_str())?;
    populate_opm_state(py, &dict, &opm)?;
    dict.set_item("kvn", kvn)?;
    dict.set_item("xml", xml)?;
    Ok(dict.unbind())
}

fn populate_opm_state(py: Python<'_>, dict: &Bound<'_, PyDict>, opm: &Opm) -> PyResult<()> {
    let pos_km = vec3_values(&opm.data.state.position);
    let vel_km_s = vec3_values(&opm.data.state.velocity);
    dict.set_item("epoch", opm.data.state.epoch.to_string())?;
    dict.set_item("position_km", pos_km)?;
    dict.set_item("velocity_km_s", vel_km_s)?;
    dict.set_item("position_m", pos_km.map(|v| v * M_PER_KM))?;
    dict.set_item("velocity_m_s", vel_km_s.map(|v| v * M_PER_KM))?;
    dict.set_item(
        "spacecraft_mass_kg",
        opm.data
            .spacecraft
            .as_ref()
            .and_then(|s| s.mass.map(|m| m.value())),
    )?;
    if let Some(kep) = &opm.data.keplerian {
        let kd = PyDict::new(py);
        kd.set_item("semi_major_axis_km", kep.semi_major_axis.value())?;
        kd.set_item("eccentricity", kep.eccentricity)?;
        kd.set_item("inclination_deg", kep.inclination.value())?;
        kd.set_item("ra_of_asc_node_deg", kep.ra_of_asc_node.value())?;
        kd.set_item("arg_of_pericenter_deg", kep.arg_of_pericenter.value())?;
        let (kind, val) = match kep.anomaly {
            Anomaly::True(a) => ("TRUE_ANOMALY", a.value()),
            Anomaly::Mean(a) => ("MEAN_ANOMALY", a.value()),
        };
        kd.set_item("anomaly_type", kind)?;
        kd.set_item("anomaly_deg", val)?;
        kd.set_item("gm_km3_s2", kep.gm.value())?;
        dict.set_item("keplerian", kd)?;
    } else {
        dict.set_item("keplerian", py.None())?;
    }
    dict.set_item(
        "covariance_6x6_km",
        opm.data.covariance.as_ref().map(|c| c.matrix.to_matrix()),
    )?;
    dict.set_item(
        "covariance_ref_frame",
        opm.data
            .covariance
            .as_ref()
            .and_then(|c| c.ref_frame.as_ref().map(|f| f.as_str())),
    )?;
    dict.set_item("maneuver_count", opm.data.maneuvers.len())?;
    Ok(())
}

/// Imports a CCSDS `OPM` into `sim-orbital` SI coordinates (`m`, `m/s`), `TDB` epoch, and optional ICRF 6x6 covariance.
#[pyfunction]
#[pyo3(signature = (text, strict = true))]
pub fn ccsds_opm_to_state(py: Python<'_>, text: &str, strict: bool) -> PyResult<Py<PyDict>> {
    let (opm, _) = parse_opm_message(text, strict)?;
    let scales = shared_time_scales()?;
    let imported = CcsdsOdm::opm_to_state(&opm, scales).map_err(SimPyError::from)?;
    let dict = PyDict::new(py);
    dict.set_item("tdb_s", imported.epoch.as_j2000_seconds_f64())?;
    dict.set_item("pos_m", imported.state.pos_m.to_array())?;
    dict.set_item("vel_m_s", imported.state.vel_m_s.to_array())?;
    dict.set_item("ref_frame", imported.ref_frame.as_str())?;
    dict.set_item(
        "cov_icrf_6x6",
        imported.covariance_icrf.map(|c| *c.as_matrix()),
    )?;
    dict.set_item("mass_kg", imported.mass_kg)?;
    dict.set_item("maneuver_count", imported.maneuvers.len())?;
    Ok(dict.unbind())
}

/// Parses a CCSDS `OMM` (auto-detecting KVN, XML, or CelesTrak/Space-Track JSON) and returns elements and round-trip strings.
#[pyfunction]
#[pyo3(signature = (text, strict = true))]
pub fn parse_ccsds_omm(py: Python<'_>, text: &str, strict: bool) -> PyResult<Py<PyDict>> {
    let mode = parse_mode(strict);
    let source_format = detect_format_kind(text)?;
    let omm = match source_format {
        "xml" => Omm::from_xml(text, mode).map_err(SimPyError::from)?,
        "json" => Omm::from_json(text, mode).map_err(SimPyError::from)?,
        _ => Omm::from_kvn(text, mode).map_err(SimPyError::from)?,
    };
    let opts = WriteOptions::default();
    let json = omm.to_json(opts).map_err(SimPyError::from)?;
    let (kvn, xml) = if omm.version.is_some() {
        (
            Some(omm.to_kvn(opts).map_err(SimPyError::from)?),
            Some(omm.to_xml(opts).map_err(SimPyError::from)?),
        )
    } else {
        (None, None)
    };
    build_omm_dict(py, &omm, source_format, json, kvn, xml)
}

fn build_omm_dict(
    py: Python<'_>,
    omm: &Omm,
    source_format: &str,
    json: String,
    kvn: Option<String>,
    xml: Option<String>,
) -> PyResult<Py<PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("message_type", "OMM")?;
    dict.set_item("source_format", source_format)?;
    dict.set_item("version", omm.version.map(|v| v.as_str()))?;
    dict.set_item("originator", omm.header.originator.as_deref())?;
    dict.set_item("object_name", &omm.metadata.object_name)?;
    dict.set_item("object_id", &omm.metadata.object_id)?;
    dict.set_item(
        "center_name",
        omm.metadata.center_name.as_ref().map(|c| c.as_str()),
    )?;
    dict.set_item(
        "ref_frame",
        omm.metadata.ref_frame.as_ref().map(|f| f.as_str()),
    )?;
    dict.set_item("time_system", omm.metadata.time_system.map(|t| t.as_str()))?;
    dict.set_item(
        "mean_element_theory",
        omm.metadata
            .mean_element_theory
            .as_ref()
            .map(|m| m.as_str()),
    )?;
    populate_omm_elements(&dict, omm)?;
    dict.set_item("json", json)?;
    dict.set_item("kvn", kvn)?;
    dict.set_item("xml", xml)?;
    Ok(dict.unbind())
}

fn populate_omm_elements(dict: &Bound<'_, PyDict>, omm: &Omm) -> PyResult<()> {
    let me = &omm.data.mean_elements;
    dict.set_item("epoch", me.epoch.to_string())?;
    match me.size {
        OrbitSize::MeanMotion(mm) => {
            dict.set_item("mean_motion_rev_day", Some(mm.value()))?;
            dict.set_item("semi_major_axis_km", None::<f64>)?;
        }
        OrbitSize::SemiMajorAxis(sma) => {
            dict.set_item("mean_motion_rev_day", None::<f64>)?;
            dict.set_item("semi_major_axis_km", Some(sma.value()))?;
        }
    }
    dict.set_item("eccentricity", me.eccentricity)?;
    dict.set_item("inclination_deg", me.inclination.value())?;
    dict.set_item("ra_of_asc_node_deg", me.ra_of_asc_node.value())?;
    dict.set_item("arg_of_pericenter_deg", me.arg_of_pericenter.value())?;
    dict.set_item("mean_anomaly_deg", me.mean_anomaly.value())?;
    dict.set_item(
        "norad_cat_id",
        omm.data.tle.as_ref().and_then(|t| t.norad_cat_id),
    )?;
    dict.set_item(
        "bstar",
        omm.data.tle.as_ref().and_then(|t| match t.drag {
            Some(TleDrag::Bstar(q)) => Some(q.value()),
            _ => None,
        }),
    )?;
    dict.set_item(
        "mean_motion_dot",
        omm.data
            .tle
            .as_ref()
            .and_then(|t| t.mean_motion_dot.map(|q| q.value())),
    )?;
    dict.set_item(
        "mean_motion_ddot",
        omm.data.tle.as_ref().and_then(|t| match t.second_term {
            Some(TleSecondTerm::MeanMotionDdot(q)) => Some(q.value()),
            _ => None,
        }),
    )?;
    Ok(())
}

/// Parses a CCSDS `OEM` (auto-detecting KVN or XML) and returns segments, state vectors, and round-trip strings.
#[pyfunction]
#[pyo3(signature = (text, strict = true))]
pub fn parse_ccsds_oem(py: Python<'_>, text: &str, strict: bool) -> PyResult<Py<PyDict>> {
    let mode = parse_mode(strict);
    let source_format = detect_format_kind(text)?;
    let oem = match source_format {
        "xml" => Oem::from_xml(text, mode).map_err(SimPyError::from)?,
        "kvn" => Oem::from_kvn(text, mode).map_err(SimPyError::from)?,
        other => {
            return Err(SimPyError::InvalidInput(format!(
                "unsupported OEM format `{other}` (expected KVN or XML)"
            ))
            .into());
        }
    };
    let opts = WriteOptions::default();
    let kvn = oem.to_kvn(opts).map_err(SimPyError::from)?;
    let xml = oem.to_xml(opts).map_err(SimPyError::from)?;

    let dict = PyDict::new(py);
    dict.set_item("message_type", "OEM")?;
    dict.set_item("source_format", source_format)?;
    dict.set_item("version", oem.version.as_str())?;
    dict.set_item("originator", oem.header.originator.as_deref())?;
    dict.set_item("segment_count", oem.segments.len())?;
    dict.set_item("segments", build_oem_segments(py, &oem)?)?;
    dict.set_item("kvn", kvn)?;
    dict.set_item("xml", xml)?;
    Ok(dict.unbind())
}

fn build_oem_segments<'py>(py: Python<'py>, oem: &Oem) -> PyResult<Bound<'py, PyList>> {
    let seg_list = PyList::empty(py);
    for seg in &oem.segments {
        let sd = PyDict::new(py);
        sd.set_item("object_name", &seg.metadata.object_name)?;
        sd.set_item("object_id", &seg.metadata.object_id)?;
        sd.set_item("center_name", seg.metadata.center_name.as_str())?;
        sd.set_item("ref_frame", seg.metadata.ref_frame.as_str())?;
        sd.set_item("time_system", seg.metadata.time_system.as_str())?;
        sd.set_item("start_time", seg.metadata.start_time.to_string())?;
        sd.set_item("stop_time", seg.metadata.stop_time.to_string())?;
        sd.set_item(
            "interpolation",
            seg.metadata.interpolation.as_ref().map(|i| i.as_str()),
        )?;
        sd.set_item("interpolation_degree", seg.metadata.interpolation_degree)?;
        sd.set_item("covariance_count", seg.data.covariances.len())?;

        let states = PyList::empty(py);
        for st in &seg.data.states {
            let entry = PyDict::new(py);
            entry.set_item("epoch", st.epoch.to_string())?;
            entry.set_item("position_km", vec3_values(&st.position))?;
            entry.set_item("velocity_km_s", vec3_values(&st.velocity))?;
            entry.set_item(
                "acceleration_km_s2",
                st.acceleration.as_ref().map(vec3_values),
            )?;
            states.append(entry)?;
        }
        sd.set_item("states", states)?;
        seg_list.append(sd)?;
    }
    Ok(seg_list)
}

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(parse_ccsds_opm, m)?)?;
    m.add_function(wrap_pyfunction!(ccsds_opm_to_state, m)?)?;
    m.add_function(wrap_pyfunction!(parse_ccsds_omm, m)?)?;
    m.add_function(wrap_pyfunction!(parse_ccsds_oem, m)?)?;
    Ok(())
}
