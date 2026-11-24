//! Python PyO3 bindings (`sim_py`) for `Simulate` astrodynamics, spacetime scales,
//! CCSDS Navigation Data Messages, NAIF SPICE kernels, and H3 coverage / Monte Carlo analysis.

#![forbid(unsafe_code)]

mod analysis;
mod error;
mod interchange;
mod orbital;
mod time;

use pyo3::prelude::*;

/// Python extension module exposing `Simulate`'s orbital, spacetime, interchange, and analysis tools.
#[pymodule]
fn sim_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    time::register(m)?;
    orbital::register(m)?;
    interchange::register(m)?;
    analysis::register(m)?;
    Ok(())
}
