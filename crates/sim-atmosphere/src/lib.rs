//! Atmosphere and cloud fields: what the flight model and the renderer sample
//! at a position and UTC instant.
//!
//! [`AtmosphereField`] is the seam real weather (GRIB-driven 4D fields) plugs
//! into; [`StandardAtmosphere`] is the ICAO standard atmosphere with no wind,
//! [`GeoidStandardAtmosphere`] enters it at EGM96 mean sea level,
//! [`ReentryAtmosphere`] extends it continuously from sea level to space, and
//! [`VacuumAtmosphere`] serves airless bodies and deep space.
//! [`FullColumnAtmosphere`] carries the whole column over the EGM96 geoid with
//! wind, and [`FlowRegime`] classifies rarefied flow by Knudsen number.
//! [`CloudField`] is the weather-map input a volumetric cloud raymarcher reads.

mod cloud;
mod field;
mod full_column;
mod isa;
mod mars;
mod rarefied;
mod reentry;
mod vacuum;
mod world_bridge;

pub use cloud::{ClearSky, CloudField, CloudKind, CloudSample};
pub use field::{
    AtmosphereError, AtmosphereField, AtmosphereSample, GeoidStandardAtmosphere, StandardAtmosphere,
};
pub use full_column::FullColumnAtmosphere;
pub use isa::{IsaState, geometric_to_geopotential_m, isa_at_geopotential};
pub use mars::{
    MARS_GAMMA, MARS_GAS_CONSTANT_J_KG_K, MarsAtmosphere, mars_geometric_to_geopotential_m,
};
pub use rarefied::FlowRegime;
pub use reentry::ReentryAtmosphere;
pub use vacuum::{COSMIC_BACKGROUND_K, VacuumAtmosphere};
pub use world_bridge::{WorldAtmosphere, utc_to_decimal_year};
