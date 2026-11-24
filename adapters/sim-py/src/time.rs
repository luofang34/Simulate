//! Astronomical time-scale conversions backed by `sim_spacetime::TimeScaleCatalog`.

use std::sync::{Arc, OnceLock};

use ccsds_ndm::{Epoch, TimeSystem};
use pyo3::prelude::*;
use sim_orbital::CcsdsEpoch;
use sim_spacetime::{TdbEpoch, TimeScaleCatalog};

use crate::error::{SimPyError, require_finite};

const SECONDS_PER_DAY: f64 = 86_400.0;
const J2000_JULIAN_DATE: f64 = 2_451_545.0;

static BUILTIN_TIME_SCALES: OnceLock<Result<Arc<TimeScaleCatalog>, sim_spacetime::TimeError>> =
    OnceLock::new();

/// Returns the cached built-in [`TimeScaleCatalog`] (`naif0012.tls`).
pub(crate) fn shared_time_scales() -> Result<&'static Arc<TimeScaleCatalog>, SimPyError> {
    match BUILTIN_TIME_SCALES.get_or_init(|| TimeScaleCatalog::builtin().map(Arc::new)) {
        Ok(scales) => Ok(scales),
        Err(err) => Err(SimPyError::Time(err.clone())),
    }
}

/// Parses an ISO-8601 UTC string (`YYYY-MM-DDTHH:MM:SS[.fff][Z|+00:00]`) into [`TdbEpoch`].
pub(crate) fn parse_utc_iso_to_tdb(utc_iso: &str) -> Result<TdbEpoch, SimPyError> {
    let normalized = normalize_iso_utc(utc_iso)?;
    let epoch = Epoch::parse(&normalized)?;
    let scales = shared_time_scales()?;
    Ok(CcsdsEpoch::to_tdb(&epoch, TimeSystem::Utc, scales)?)
}

/// Formats a [`TdbEpoch`] as an ISO-8601 UTC string (`YYYY-MM-DDTHH:MM:SS[.fff]Z`).
pub(crate) fn format_tdb_as_utc_iso(tdb: TdbEpoch) -> Result<String, SimPyError> {
    let scales = shared_time_scales()?;
    let epoch = CcsdsEpoch::from_tdb(tdb, TimeSystem::Utc, scales)?;
    Ok(format!("{epoch}Z"))
}

/// Parses floating-point `TDB` seconds past J2000 into [`TdbEpoch`].
pub(crate) fn tdb_seconds_to_epoch(tdb_s: f64) -> Result<TdbEpoch, SimPyError> {
    let s = require_finite(tdb_s, "tdb_s")?;
    TdbEpoch::from_j2000_seconds_f64(s).ok_or_else(|| {
        SimPyError::InvalidInput(format!("tdb_s={tdb_s} lies outside representable TdbEpoch"))
    })
}

fn normalize_iso_utc(raw: &str) -> Result<String, SimPyError> {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return Err(SimPyError::InvalidInput(
            "UTC ISO-8601 string cannot be empty".to_owned(),
        ));
    }
    let without_offset = if let Some(prefix) = trimmed.strip_suffix("+00:00") {
        format!("{prefix}Z")
    } else if let Some(prefix) = trimmed.strip_suffix("-00:00") {
        format!("{prefix}Z")
    } else if let Some(prefix) = trimmed.strip_suffix('z') {
        format!("{prefix}Z")
    } else {
        trimmed.to_owned()
    };
    if let Some((date, time)) = without_offset.split_once(['t', ' ']) {
        Ok(format!("{date}T{time}"))
    } else {
        Ok(without_offset)
    }
}

/// Converts a UTC ISO-8601 timestamp string to TDB seconds past `J2000.0 TDB`.
#[pyfunction]
pub fn utc_to_tdb_s(utc_iso: &str) -> PyResult<f64> {
    let tdb = parse_utc_iso_to_tdb(utc_iso)?;
    Ok(tdb.as_j2000_seconds_f64())
}

/// Converts TDB seconds past `J2000.0 TDB` to an ISO-8601 UTC string (`...Z`).
#[pyfunction]
pub fn tdb_to_utc(tdb_s: f64) -> PyResult<String> {
    let tdb = tdb_seconds_to_epoch(tdb_s)?;
    Ok(format_tdb_as_utc_iso(tdb)?)
}

/// Converts TDB seconds past `J2000.0 TDB` to TDB Julian Date (`JD`).
#[pyfunction]
pub fn tdb_s_to_jd(tdb_s: f64) -> PyResult<f64> {
    let s = require_finite(tdb_s, "tdb_s")?;
    Ok(J2000_JULIAN_DATE + s / SECONDS_PER_DAY)
}

/// Converts a TDB Julian Date (`JD`) to TDB seconds past `J2000.0 TDB`.
#[pyfunction]
pub fn jd_to_tdb_s(jd_tdb: f64) -> PyResult<f64> {
    let jd = require_finite(jd_tdb, "jd_tdb")?;
    Ok((jd - J2000_JULIAN_DATE) * SECONDS_PER_DAY)
}

/// Returns `TDB - TAI` in seconds (`32.184 s` plus periodic relativistic term) at `tdb_s`.
#[pyfunction]
pub fn tdb_minus_tai_s(tdb_s: f64) -> PyResult<f64> {
    let tdb = tdb_seconds_to_epoch(tdb_s)?;
    let scales = shared_time_scales()?;
    let tai_s = scales.tdb_to_tai_j2000_seconds(tdb);
    Ok(tdb.as_j2000_seconds_f64() - tai_s)
}

/// Registers time-scale functions on the Python module.
pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(utc_to_tdb_s, m)?)?;
    m.add_function(wrap_pyfunction!(tdb_to_utc, m)?)?;
    m.add_function(wrap_pyfunction!(tdb_s_to_jd, m)?)?;
    m.add_function(wrap_pyfunction!(jd_to_tdb_s, m)?)?;
    m.add_function(wrap_pyfunction!(tdb_minus_tai_s, m)?)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn utc_tdb_round_trip_and_leap_offset_are_accurate() -> Result<(), SimPyError> {
        let tdb = utc_to_tdb_s("2026-06-01T12:00:00.250Z")
            .map_err(|e| SimPyError::InvalidInput(format!("unexpected PyErr in test: {e}")))?;
        let back = tdb_to_utc(tdb)
            .map_err(|e| SimPyError::InvalidInput(format!("unexpected PyErr in test: {e}")))?;
        assert_eq!(back, "2026-06-01T12:00:00.25Z");
        let tdb_tai = tdb_minus_tai_s(0.0)
            .map_err(|e| SimPyError::InvalidInput(format!("unexpected PyErr in test: {e}")))?;
        assert!((tdb_tai - 32.184).abs() < 0.002);
        Ok(())
    }

    #[test]
    fn invalid_time_inputs_fail_closed() {
        assert!(parse_utc_iso_to_tdb("").is_err());
        assert!(parse_utc_iso_to_tdb("not-a-date").is_err());
        assert!(tdb_seconds_to_epoch(f64::NAN).is_err());
        assert!(require_finite(f64::INFINITY, "jd_tdb").is_err());
    }
}
