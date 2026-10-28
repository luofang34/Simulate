//! Atmosphere and cloud fields: what the flight model and the renderer sample
//! at a position and UTC instant.
//!
//! [`AtmosphereField`] is the seam real weather (GRIB-driven 4D fields) plugs
//! into; [`StandardAtmosphere`] is the ICAO standard atmosphere with no wind.
//! [`CloudField`] is the weather-map input a volumetric cloud raymarcher reads.

mod cloud;
mod field;
mod isa;

pub use cloud::{ClearSky, CloudField, CloudKind, CloudSample};
pub use field::{AtmosphereError, AtmosphereField, AtmosphereSample, StandardAtmosphere};
pub use isa::{IsaState, geometric_to_geopotential_m, isa_at_geopotential};
