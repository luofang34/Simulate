//! The force elements of a multibody vehicle: what loads its model at every
//! integrator stage, and the layout of their states.

use mbd_aero::AeroElement;
use mbd_core::WrenchAtSite;
use mbd_multibody::{
    ContactPointSpec, ContactPointState, ElementContext, ForceElement, HydrodynamicsModel,
    JointDriveInput, Loads, LoopConstraint, ModelScale, MultibodyError, Resolution, SurfaceSample,
    Wheel, WheelSpec, WheelState,
};

use crate::flight_model::coupled_mechanism::CoupledMechanism;
use crate::flight_model::environment::{CgGravity, FlightEnvironment, StageWorld};

type Env<'e> = FlightEnvironment<'e>;
type Ctx<'c, 'e> = ElementContext<'c, Env<'e>>;

pub(crate) fn fill(target: &mut [f64], values: impl IntoIterator<Item = f64>) {
    for (slot, value) in target.iter_mut().zip(values) {
        *slot = value;
    }
}

/// An element that asks its inputs by position alone (air, water),
/// answered from the environment as it is at the instant of each evaluation.
struct AtStage<'a, T: ?Sized>(&'a T);

impl<'e, T> ForceElement<Env<'e>> for AtStage<'_, T>
where
    T: for<'s> ForceElement<StageWorld<'s, 'e>> + ?Sized,
{
    fn state_len(&self) -> usize {
        ForceElement::<StageWorld<'_, 'e>>::state_len(self.0)
    }

    fn mode_len(&self) -> usize {
        ForceElement::<StageWorld<'_, 'e>>::mode_len(self.0)
    }

    fn has_transitions(&self) -> bool {
        ForceElement::<StageWorld<'_, 'e>>::has_transitions(self.0)
    }

    fn initial_state(&self, state: &mut [f64], modes: &mut [f64]) {
        ForceElement::<StageWorld<'_, 'e>>::initial_state(self.0, state, modes);
    }

    fn state_scale(&self, model: &ModelScale, scale: &mut [f64]) {
        ForceElement::<StageWorld<'_, 'e>>::state_scale(self.0, model, scale);
    }

    fn evaluate(
        &self,
        ctx: &Ctx<'_, 'e>,
        state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let world = ctx.inputs.at(ctx.time_s);
        let at_stage = ElementContext {
            time_s: ctx.time_s,
            model: ctx.model,
            state: ctx.state,
            kinematics: ctx.kinematics,
            inputs: &world,
        };
        self.0.evaluate(&at_stage, state, modes, loads, state_dot)
    }

    fn transition(
        &self,
        ctx: &Ctx<'_, 'e>,
        state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        let world = ctx.inputs.at(ctx.time_s);
        let at_stage = ElementContext {
            time_s: ctx.time_s,
            model: ctx.model,
            state: ctx.state,
            kinematics: ctx.kinematics,
            inputs: &world,
        };
        self.0.transition(&at_stage, state, modes)
    }
}

/// Numbers of continuous states and of modes `element` keeps.
pub(crate) fn aero_lens(element: &AeroElement) -> (usize, usize) {
    (
        ForceElement::<StageWorld<'_, '_>>::state_len(element),
        ForceElement::<StageWorld<'_, '_>>::mode_len(element),
    )
}

/// Writes the state and modes `element` starts with.
pub(crate) fn aero_initial_state(element: &AeroElement, state: &mut [f64], modes: &mut [f64]) {
    ForceElement::<StageWorld<'_, '_>>::initial_state(element, state, modes);
}

/// A footpad or skid against the ground of a rotating body. The contact law
/// is evaluated in the body's fixed axes, where the ground stands still, and
/// its bristle deflection is kept in those axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct GroundContact(pub(crate) ContactPointSpec);

impl ForceElement<Env<'_>> for GroundContact {
    fn state_len(&self) -> usize {
        3
    }

    fn mode_len(&self) -> usize {
        1
    }

    fn has_transitions(&self) -> bool {
        true
    }

    fn initial_state(&self, state: &mut [f64], modes: &mut [f64]) {
        ContactPointState::default().write_blocks(state, modes);
    }

    fn state_scale(&self, model: &ModelScale, scale: &mut [f64]) {
        ForceElement::<SurfaceSample>::state_scale(&self.0, model, scale);
    }

    fn evaluate(
        &self,
        ctx: &Ctx<'_, '_>,
        state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        state_dot.fill(0.0);
        if !ctx.kinematics.is_attached(self.0.component) {
            return Ok(());
        }
        let point = ctx
            .kinematics
            .point(self.0.component, self.0.offset_local_m)?;
        let Some((on_ground, surface)) = ctx.inputs.ground_under(&point, ctx.time_s) else {
            return Ok(());
        };
        let typed = ContactPointState::from_blocks(state, modes);
        let out = self.0.evaluate(&typed, &on_ground, &surface);
        fill(state_dot, out.deflection_rate_world_m_s.to_array());
        loads.add_wrench(&out.wrench(&self.0, &on_ground))
    }

    fn transition(
        &self,
        ctx: &Ctx<'_, '_>,
        state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        if !ctx.kinematics.is_attached(self.0.component) {
            return Ok(false);
        }
        let point = ctx
            .kinematics
            .point(self.0.component, self.0.offset_local_m)?;
        let mut typed = ContactPointState::from_blocks(state, modes);
        let before = typed;
        match ctx.inputs.ground_under(&point, ctx.time_s) {
            Some((on_ground, surface)) => {
                self.0.transition(&mut typed, &on_ground, &surface);
            }
            None => typed = ContactPointState::default(),
        }
        typed.write_blocks(state, modes);
        Ok(typed != before)
    }
}

/// A wheel against the ground of a rotating body, evaluated like
/// [`GroundContact`] in the body's fixed axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct GroundWheel(pub(crate) Wheel);

impl GroundWheel {
    /// A wheel's spin inertia from its radius alone, for specifications
    /// that give none: an 8 kg disc, never under 0.05 kg·m².
    pub(crate) fn spin_inertia_estimate(spec: &WheelSpec) -> f64 {
        (0.5 * 8.0 * spec.wheel_radius_m * spec.wheel_radius_m).max(0.05)
    }
}

impl ForceElement<Env<'_>> for GroundWheel {
    fn state_len(&self) -> usize {
        1
    }

    fn state_scale(&self, model: &ModelScale, scale: &mut [f64]) {
        ForceElement::<SurfaceSample>::state_scale(&self.0, model, scale);
    }

    fn evaluate(
        &self,
        ctx: &Ctx<'_, '_>,
        state: &[f64],
        _modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        state_dot.fill(0.0);
        let spec = &self.0.spec;
        if !ctx.kinematics.is_attached(spec.component) {
            return Ok(());
        }
        let hub = ctx
            .kinematics
            .point(spec.component, spec.hub_offset_local_m)?;
        let Some((on_ground, surface)) = ctx.inputs.ground_under(&hub, ctx.time_s) else {
            return Ok(());
        };
        let typed = WheelState {
            spin_rate_rad_s: state.first().copied().unwrap_or(0.0),
        };
        let out = spec.evaluate(
            &typed,
            &self.0.command,
            &on_ground,
            &surface,
            self.0.spin_inertia_kg_m2,
        )?;
        fill(state_dot, [out.spin_accel_rad_s2]);
        loads.add_wrench(&out.wrench(spec, &on_ground))
    }
}

/// Every element loading a vehicle's model, in the order their states are
/// laid out. Everything up to the hull is the vehicle's own and keeps its
/// state from step to step; the held loads carry none; the coupled
/// mechanisms come last, so a step with any only appends to the layout.
#[derive(Debug, Clone, Default)]
pub(crate) struct VehicleElements {
    pub(crate) gravity: Option<CgGravity>,
    pub(crate) aero: Vec<AeroElement>,
    pub(crate) contacts: Vec<GroundContact>,
    pub(crate) wheels: Vec<GroundWheel>,
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
    AtStage<'a, Vec<AeroElement>>,
    &'a Vec<GroundContact>,
    &'a Vec<GroundWheel>,
    AtStage<'a, Option<HydrodynamicsModel>>,
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
                AtStage(&self.aero),
                &self.contacts,
                &self.wheels,
                AtStage(&self.hull),
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
    fn state_len(&self) -> usize {
        ForceElement::<Env<'e>>::state_len(&self.set())
    }

    fn mode_len(&self) -> usize {
        ForceElement::<Env<'e>>::mode_len(&self.set())
    }

    fn has_transitions(&self) -> bool {
        ForceElement::<Env<'e>>::has_transitions(&self.set())
    }

    fn initial_state(&self, state: &mut [f64], modes: &mut [f64]) {
        ForceElement::<Env<'e>>::initial_state(&self.set(), state, modes);
    }

    fn state_scale(&self, model: &ModelScale, scale: &mut [f64]) {
        ForceElement::<Env<'e>>::state_scale(&self.set(), model, scale);
    }

    fn evaluate(
        &self,
        ctx: &Ctx<'_, 'e>,
        state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        self.set().evaluate(ctx, state, modes, loads, state_dot)
    }

    fn has_resolution(&self) -> bool {
        ForceElement::<Env<'e>>::has_resolution(&self.set())
    }

    fn resolve(
        &self,
        ctx: &Ctx<'_, 'e>,
        state: &[f64],
        modes: &[f64],
        resolution: &mut Resolution<'_>,
        loads: &mut Loads<'_>,
    ) -> Result<(), MultibodyError> {
        self.set().resolve(ctx, state, modes, resolution, loads)
    }

    fn transition(
        &self,
        ctx: &Ctx<'_, 'e>,
        state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        self.set().transition(ctx, state, modes)
    }
}
