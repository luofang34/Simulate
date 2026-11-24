//! CCSDS Navigation Data Message (`OPM`, `OMM`, `OEM`, `APM`, `AEM`, `CDM`, `TDM`)
//! and NAIF SPICE binary kernel (`DAF/SPK`, `DAF/CK`) Python interchange bindings.

mod ccsds_adm;
mod ccsds_odm;
mod spice;
#[cfg(test)]
mod tests;

use pyo3::prelude::*;

/// Registers CCSDS NDM and SPICE kernel functions on the Python module.
pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    ccsds_odm::register(m)?;
    ccsds_adm::register(m)?;
    spice::register(m)?;
    Ok(())
}
