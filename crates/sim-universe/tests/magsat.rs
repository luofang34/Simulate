//! The magsat: a small rigid satellite that detumbles on the Earth's
//! magnetic field through the real path, catalog IGRF → magnetometer →
//! B-dot controller → held dipole → `m × B` at every integrator stage.

#[path = "magsat/determinism.rs"]
mod determinism;
#[path = "magsat/detumble.rs"]
mod detumble;
#[path = "magsat/orbit.rs"]
mod orbit;
#[path = "magsat/satellite.rs"]
mod satellite;
