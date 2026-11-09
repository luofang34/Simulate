//! High-precision `TDB` coordinate time (`TdbEpoch`), coverage intervals (`TdbSpanNs`, `CoverageSet`),
//! time-scale conversions (`TimeScaleCatalog`), and master-clock adapter (`SimClockAdapter`)
//! re-exported from `sim-spacetime`.

pub use sim_spacetime::{
    CoverageSet, EarthEopEntry, EarthEopTable, FutureUtcPolicy, LeapSecondEntry, LeapSecondTable,
    OutOfEopPolicy, SimClockAdapter, TdbEpoch, TdbSpanNs, TimeError, TimeScaleCatalog, Ut1Sample,
};
