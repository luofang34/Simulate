//! The bodies a vehicle can be near, as data every tier reads the same way.
//!
//! [`HarmonicGravity`] and [`HarmonicMagnetism`] are a body's truth
//! gravity and magnetic fields, synthesised by `world-potential`'s one
//! spherical-harmonic core and served through [`sim_geo::GravityField`]
//! and [`sim_geo::MagneticField`]. Flight dynamics, orbital propagation and
//! sensors hold the same instance, so they cannot disagree about the world.
//! [`AnomalyOverlay`] adds a surveyed local anomaly to either.
//!
//! A [`BodyDescriptor`] gathers one body's identity, shape, rotation source,
//! fields, atmosphere, terrain, sphere of influence, tier limits and solar
//! occulter, and a [`BodyCatalog`] holds the bodies of a run. It sits above
//! `sim-atmosphere` and `sim-spacetime`, whose providers and identifiers a
//! descriptor names, and below every crate that flies or senses, so each of
//! them looks a body up instead of matching on its name.

mod anomaly;
mod catalog;
mod descriptor;
mod error;
mod gravity;
mod magnetic;
mod scratch;

pub use anomaly::{AnomalyOverlay, GravityWithAnomaly, MagnetismWithAnomaly};
pub use catalog::BodyCatalog;
pub use descriptor::{
    AtmosphereProvider, BodyDescriptor, RegimeLimits, RotationSource, SphereOfInfluence,
};
pub use error::FieldBuildError;
pub use gravity::{GravitySample, HarmonicGravity, HarmonicTerm};
pub use magnetic::{HarmonicMagnetism, NoMagneticField};
pub use world_potential::{GridGeometry, OutsideCoverage, TideSystem, Truncation};
