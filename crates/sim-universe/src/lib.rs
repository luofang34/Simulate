//! Vessels on one master clock across fidelity tiers.
//!
//! A vessel in atmosphere, on a surface or near another is flown by a
//! `sim-core` block in its central body's rotating frame ([`LocalPlant`]); a
//! coasting one is propagated by `sim-orbital` in ICRF axes. [`Universe`]
//! advances every vessel together and hands each between tiers through the
//! exact frame change in [`to_inertial`] / [`to_body_fixed`], reading body
//! orientation from the same catalog the orbital propagator uses.

mod error;
mod handoff;
mod local;
mod universe;

pub use error::UniverseError;
pub use handoff::{InertialExtras, body_ellipsoid, naif_body, to_body_fixed, to_inertial};
pub use local::{CoastingRigidBody, LocalFactory, LocalPlant};
pub use universe::{OrbitalContext, Tier, Universe};
