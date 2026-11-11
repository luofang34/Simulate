//! Vessels on one master clock across fidelity tiers.
//!
//! A vessel in atmosphere, on a surface or near another is flown by a
//! `sim-core` block in its central body's rotating frame ([`LocalPlant`]); a
//! coasting one is propagated by `sim-orbital` in ICRF axes. [`Universe`]
//! advances every vessel together and hands each between tiers through the
//! exact frame change in [`to_inertial`] / [`to_body_fixed`], reading body
//! orientation from the same catalog the orbital propagator uses.

mod bus;
mod error;
mod handoff;
mod local;
mod planning;
mod station_keeping;
mod tiering;
mod universe;

pub use bus::{VesselBusConfig, VesselBusTelemetry};
pub use error::UniverseError;
pub use handoff::{InertialExtras, body_ellipsoid, naif_body, to_body_fixed, to_inertial};
pub use local::{CoastingRigidBody, LocalFactory, LocalPlant, OrbitalBody};
#[cfg(not(target_arch = "wasm32"))]
pub use planning::ThreadPlanExecutor;
pub use planning::{
    PlanExecutor, PlanHold, PlanId, PlanJob, PlanOutput, PlanRecord, PlanReport, PlanRequest,
    PlanSchedule, PlannedCommand, SlicedPlanExecutor, SynchronousPlanExecutor,
};
pub use station_keeping::{DEFAULT_PLAN_DELAY, L2KeepingBurn, L2StationKeeper};
pub use tiering::{AltitudeBand, TierChange, TierPolicy, planetary_dynamic_pressure_pa};
pub use universe::{OrbitalContext, Tier, Universe, UniverseCheckpoint};
