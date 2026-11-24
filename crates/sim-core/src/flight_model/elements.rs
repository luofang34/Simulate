//! The force elements of a multibody vehicle: what loads its model at every
//! integrator stage, and the layout of their states.

use mbd_aero::AeroElement;
use mbd_core::WrenchAtSite;
use mbd_multibody::{
    ContactPointSpec, ForceElement, HydrodynamicsModel, JointDriveInput, LoopConstraint, Wheel,
    WheelSpec, forward_force_element,
};

use crate::flight_model::coupled_mechanism::CoupledMechanism;
use crate::flight_model::environment::{CgGravity, FlightEnvironment};

type Env<'e> = FlightEnvironment<'e>;

/// Numbers of continuous states and of modes `element` keeps.
pub(crate) fn aero_lens(element: &AeroElement) -> (usize, usize) {
    (
        ForceElement::<Env<'_>>::state_len(element),
        ForceElement::<Env<'_>>::mode_len(element),
    )
}

/// Writes the state and modes `element` starts with.
pub(crate) fn aero_initial_state(element: &AeroElement, state: &mut [f64], modes: &mut [f64]) {
    ForceElement::<Env<'_>>::initial_state(element, state, modes);
}

/// A wheel's spin inertia from its radius alone, for specifications that
/// give none: an 8 kg disc, never under 0.05 kg·m².
pub(crate) fn wheel_spin_inertia_estimate(spec: &WheelSpec) -> f64 {
    (0.5 * 8.0 * spec.wheel_radius_m * spec.wheel_radius_m).max(0.05)
}

/// Every element loading a vehicle's model, in the order their states are
/// laid out. Everything up to the hull is the vehicle's own and keeps its
/// state from step to step; the held loads carry none; the coupled
/// mechanisms come last, so a step with any only appends to the layout.
#[derive(Debug, Clone, Default)]
pub(crate) struct VehicleElements {
    pub(crate) gravity: Option<CgGravity>,
    pub(crate) aero: Vec<AeroElement>,
    pub(crate) contacts: Vec<ContactPointSpec>,
    pub(crate) wheels: Vec<Wheel>,
    pub(crate) hull: Option<HydrodynamicsModel>,
    pub(crate) loops: Vec<LoopConstraint>,
    /// Subsystem and external wrenches held over the step.
    pub(crate) wrenches: Vec<WrenchAtSite>,
    /// Actuator commands held over the step.
    pub(crate) drives: Vec<JointDriveInput>,
    pub(crate) mechanisms: Vec<CoupledMechanism>,
}

type OwnSet<'a> = (
    &'a Option<CgGravity>,
    &'a Vec<AeroElement>,
    &'a Vec<ContactPointSpec>,
    &'a Vec<Wheel>,
    &'a Option<HydrodynamicsModel>,
    &'a Vec<LoopConstraint>,
);
type HeldSet<'a> = (
    &'a Vec<WrenchAtSite>,
    &'a Vec<JointDriveInput>,
    &'a Vec<CoupledMechanism>,
);

impl VehicleElements {
    fn set(&self) -> (OwnSet<'_>, HeldSet<'_>) {
        (
            (
                &self.gravity,
                &self.aero,
                &self.contacts,
                &self.wheels,
                &self.hull,
                &self.loops,
            ),
            (&self.wrenches, &self.drives, &self.mechanisms),
        )
    }

    /// Lengths of the state and mode blocks owned by the vehicle's own
    /// elements, which precede those of the coupled mechanisms.
    pub(crate) fn own_lens(&self) -> (usize, usize) {
        let (own, _) = self.set();
        (
            ForceElement::<Env<'_>>::state_len(&own),
            ForceElement::<Env<'_>>::mode_len(&own),
        )
    }
}

impl<'e> ForceElement<Env<'e>> for VehicleElements {
    forward_force_element!(Env<'e>, self => self.set());
}
