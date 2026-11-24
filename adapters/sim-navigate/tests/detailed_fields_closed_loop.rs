//! Closed-loop verification of detailed `sim-bodies` gravity and geomagnetic
//! fields through `RigidBody6DofBlock`, `SimulatedImu`, `SimulatedMagnetometer`,
//! `MagneticAnomalyMap`, and Navigate's 22-state `InertialNavigationFilter`.

#[cfg(test)]
#[path = "detailed_fields/gravity.rs"]
mod gravity;

#[cfg(test)]
#[path = "detailed_fields/magnetic_world.rs"]
mod magnetic_world;

#[cfg(test)]
#[path = "detailed_fields/magnetic_harness.rs"]
mod magnetic_harness;

#[cfg(test)]
#[path = "detailed_fields/magnetic.rs"]
mod magnetic;
