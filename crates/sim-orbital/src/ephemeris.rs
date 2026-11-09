//! Multi-segment geometric SPK ephemeris contracts (`ICRF / J2000` axes).

mod spk_catalog;

pub use sim_spacetime::{
    DerivativeOrder, EphemerisCatalog, EphemerisError, KinematicStateIcrf, MarsCenterApproximation,
};
pub use spk_catalog::SpkEphemerisCatalog;
