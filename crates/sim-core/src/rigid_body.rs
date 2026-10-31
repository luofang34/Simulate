use glam::DVec3;
use mbd_core::{
    AeroModel, AeroWrench, ComponentId, ComponentState, ComponentTree, LocalSiteKinematics,
    MassProperties, MbdError, compute_site_kinematics,
};
use sim_atmosphere::AtmosphereField;
use sim_geo::{BodyEllipsoid, UtcTime};

use crate::{Block, BlockError, SimInstant, TickSpan, VehicleState};

use dynamics::{BLOCK, Dynamics, Evaluation, RigidState};

mod dynamics;
mod integrator;

#[cfg(test)]
mod tests;

/// One thruster's load this step, in body axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrusterWrench {
    /// Where the force acts, in the component tree's root axes, metres.
    pub application_point_body_m: DVec3,
    /// Force, N.
    pub force_body_n: DVec3,
    /// Pure moment on top of the force's own, N·m.
    pub moment_body_nm: DVec3,
}

/// A tank's fluid mass rate this step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropellantBurn {
    /// The tank.
    pub tank: ComponentId,
    /// Rate of change of the tank's fluid mass, kg/s: negative drains it. A
    /// tank that runs dry stops at zero.
    pub dm_dt_kg_s: f64,
}

/// What [`RigidBody6DofBlock`] reads each step.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RigidBodyInput {
    /// Propulsion and reaction-control loads.
    pub thrusters: Vec<ThrusterWrench>,
    /// Tank mass flows.
    pub burns: Vec<PropellantBurn>,
    /// Control-surface deflections passed to the aerodynamics model, rad.
    pub control_surfaces_rad: [f64; 8],
}

impl RigidBodyInput {
    fn validate(&self) -> Result<(), BlockError> {
        let invalid = |input, detail: String| {
            Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input,
                detail,
            })
        };
        for thruster in &self.thrusters {
            if !(thruster.application_point_body_m.is_finite()
                && thruster.force_body_n.is_finite()
                && thruster.moment_body_nm.is_finite())
            {
                return invalid("thrusters", format!("{thruster:?} is not finite"));
            }
        }
        for burn in &self.burns {
            if !burn.dm_dt_kg_s.is_finite() {
                return invalid("burns", format!("{burn:?} is not finite"));
            }
        }
        if !self.control_surfaces_rad.iter().all(|v| v.is_finite()) {
            let surfaces = self.control_surfaces_rad;
            return invalid("control_surfaces_rad", format!("{surfaces:?}"));
        }
        Ok(())
    }
}

/// Fixed setup of a [`RigidBody6DofBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBodyConfig {
    /// The central body.
    pub body: BodyEllipsoid,
    /// Integration step.
    pub period: TickSpan,
    /// UTC at [`SimInstant::START`], which drives the atmosphere.
    pub epoch: UtcTime,
    /// State at [`SimInstant::START`]; `angular_rate_body` is relative to the
    /// central body, as [`VehicleState`] defines it.
    pub initial: VehicleState,
}

/// The state of the block's output instant that is derived from its forces,
/// so a restored block answers [`RigidBody6DofBlock::site_kinematics`] exactly
/// as the original did.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Loads {
    specific_force_cg_body: DVec3,
    alpha_body: DVec3,
    aero: AeroWrench,
}

impl From<Evaluation> for Loads {
    fn from(evaluation: Evaluation) -> Self {
        Self {
            specific_force_cg_body: evaluation.specific_force_cg_body,
            alpha_body: evaluation.alpha_body,
            aero: evaluation.aero,
        }
    }
}

/// Complete dynamic state of a [`RigidBody6DofBlock`], component states
/// included, for exact replay and rollback.
#[derive(Debug, Clone, PartialEq)]
pub struct RigidBodySnapshot {
    state: RigidState,
    loads: Loads,
    mass: MassProperties,
    now: SimInstant,
    component_states: Vec<ComponentState>,
}

impl RigidBodySnapshot {
    /// The component tree's runtime states at capture, in id order.
    pub fn component_states(&self) -> &[ComponentState] {
        &self.component_states
    }
}

/// Six-degree-of-freedom rigid-body flight over any central body and
/// atmosphere, with mass properties read live from a [`ComponentTree`].
///
/// Integrates by RK4 in the rotating body-fixed frame, so gravity is joined by
/// the Coriolis and centrifugal terms; Euler's equation runs on the inertial
/// angular rate, and [`VehicleState::angular_rate_body`] reports the rate
/// relative to the central body. Mass properties and thruster loads hold
/// constant across a step: the rate of change of inertia and of the centre of
/// gravity within a step, and jet damping, are not modelled.
///
/// The translational state follows the centre of gravity. When a burn moves it
/// within the airframe, the state transfers to the material point now at the
/// centre of gravity, which leaves the airframe's own path undisturbed.
pub struct RigidBody6DofBlock {
    config: RigidBodyConfig,
    tree: ComponentTree,
    aero: Box<dyn AeroModel>,
    atmosphere: Box<dyn AtmosphereField>,
    state: RigidState,
    loads: Loads,
    mass: MassProperties,
    now: SimInstant,
}

impl RigidBody6DofBlock {
    /// A block at `config.initial`, with loads evaluated for no thrust. Fails
    /// if the period is zero or the tree or environment cannot be evaluated.
    pub fn new(
        config: RigidBodyConfig,
        tree: ComponentTree,
        aero: Box<dyn AeroModel>,
        atmosphere: Box<dyn AtmosphereField>,
    ) -> Result<Self, BlockError> {
        if config.period == TickSpan::ZERO {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "period",
                detail: "a zero period never advances".to_owned(),
            });
        }
        let mass = tree.mass_properties().map_err(block_error)?;
        let state = RigidState::from_vehicle(&config.initial, &config.body);
        let dynamics = Dynamics::of(
            config.body,
            atmosphere.as_ref(),
            aero.as_ref(),
            &mass,
            &RigidBodyInput::default(),
        )?;
        let loads = dynamics.evaluate(&state, config.epoch)?.into();
        Ok(Self {
            config,
            tree,
            aero,
            atmosphere,
            state,
            loads,
            mass,
            now: SimInstant::START,
        })
    }

    /// The live component tree.
    pub fn tree(&self) -> &ComponentTree {
        &self.tree
    }

    /// Mass, centre of gravity and inertia at the output instant.
    pub fn mass_properties(&self) -> &MassProperties {
        &self.mass
    }

    /// The instant the output describes.
    pub fn now(&self) -> SimInstant {
        self.now
    }

    /// Inertial angular rate, body axes, rad/s.
    pub fn omega_inertial_body(&self) -> DVec3 {
        self.state.omega_inertial_body
    }

    /// Inertial angular acceleration, body axes, rad/s².
    pub fn alpha_body(&self) -> DVec3 {
        self.loads.alpha_body
    }

    /// Non-gravitational acceleration at the centre of gravity, body axes,
    /// m/s².
    pub fn specific_force_cg_body(&self) -> DVec3 {
        self.loads.specific_force_cg_body
    }

    /// The aerodynamic load at the output instant.
    pub fn aero_wrench(&self) -> &AeroWrench {
        &self.loads.aero
    }

    /// What an ideal inertial sensor at `site` reads at the output instant,
    /// relative to the centre of gravity in body axes. A site on a detached
    /// stage does not move with this vehicle and is an error.
    pub fn site_kinematics(&self, site: ComponentId) -> Result<LocalSiteKinematics, MbdError> {
        if !self.tree.is_attached(site)? {
            return Err(MbdError::DetachedComponent { id: site });
        }
        Ok(compute_site_kinematics(
            self.loads.specific_force_cg_body,
            self.state.omega_inertial_body,
            self.loads.alpha_body,
            self.tree.world_transform_in_root(site)?,
            self.mass.cg_body_m,
        ))
    }

    /// Separates `stage` and its subtree at the output instant, as a staging
    /// event does.
    ///
    /// The state transfers to the material point at the remaining vehicle's
    /// new centre of gravity, and the inertial angular rate carries over: with
    /// no separation impulse each part keeps its rigid motion. Loads are
    /// re-evaluated with no thrust until the next step supplies some. A stage
    /// already separated, or one that would leave nothing attached, is an
    /// error and leaves the block unchanged.
    pub fn separate_stage(&mut self, stage: ComponentId) -> Result<(), BlockError> {
        if !self.tree.is_attached(stage).map_err(block_error)? {
            return Err(block_error(MbdError::DetachedComponent { id: stage }));
        }
        let saved_states = self.tree.states().to_vec();
        let result = self.detach(stage);
        if result.is_err() {
            self.tree
                .restore_states(&saved_states)
                .map_err(block_error)?;
        }
        result
    }

    /// Restores `snapshot`, or reports that it came from a differently built
    /// tree and leaves the block unchanged.
    pub fn try_restore(&mut self, snapshot: &RigidBodySnapshot) -> Result<(), MbdError> {
        self.tree.restore_states(&snapshot.component_states)?;
        self.state = snapshot.state;
        self.loads = snapshot.loads;
        self.mass = snapshot.mass;
        self.now = snapshot.now;
        Ok(())
    }

    fn utc_at(&self, instant: SimInstant) -> UtcTime {
        let elapsed = instant.duration_since(SimInstant::START).as_seconds();
        self.config.epoch.advanced_by(elapsed)
    }

    fn dynamics<'a>(
        &'a self,
        mass: &'a MassProperties,
        input: &RigidBodyInput,
    ) -> Result<Dynamics<'a>, BlockError> {
        Dynamics::of(
            self.config.body,
            self.atmosphere.as_ref(),
            self.aero.as_ref(),
            mass,
            input,
        )
    }

    fn apply_burns(&mut self, burns: &[PropellantBurn], dt_s: f64) -> Result<(), MbdError> {
        for burn in burns {
            let mut tank = *self.tree.state(burn.tank)?;
            tank.fluid_mass_kg = (tank.fluid_mass_kg + burn.dm_dt_kg_s * dt_s).max(0.0);
            self.tree.set_state(burn.tank, tank)?;
        }
        Ok(())
    }

    /// Everything that can fail in a separation, with the same contract as
    /// [`Self::advance`].
    fn detach(&mut self, stage: ComponentId) -> Result<(), BlockError> {
        let mut detached = *self.tree.state(stage).map_err(block_error)?;
        detached.attached = false;
        self.tree.set_state(stage, detached).map_err(block_error)?;
        let mass = self.tree.mass_properties().map_err(block_error)?;
        let body = self.config.body;
        let state = self
            .state
            .transported(&body, mass.cg_body_m - self.mass.cg_body_m);
        let loads = self
            .dynamics(&mass, &RigidBodyInput::default())?
            .evaluate(&state, self.utc_at(self.now))?
            .into();
        self.state = state;
        self.loads = loads;
        self.mass = mass;
        Ok(())
    }

    /// Everything that can fail in a step. The state fields are assigned only
    /// on success; the tree is put back by the caller on failure.
    fn advance(&mut self, input: &RigidBodyInput) -> Result<(), BlockError> {
        let period = self.config.period;
        let dt_s = period.as_seconds();
        let body = self.config.body;
        let start_utc = self.utc_at(self.now);
        let mass_at_start = self.mass;
        let dynamics = self.dynamics(&mass_at_start, input)?;
        let integrated = integrator::rk4(&self.state, start_utc, dt_s, |state, utc| {
            dynamics.evaluate(state, utc).map(|e| e.derivative)
        })?;

        self.apply_burns(&input.burns, dt_s).map_err(block_error)?;
        let mass = self.tree.mass_properties().map_err(block_error)?;
        let state = integrated.transported(&body, mass.cg_body_m - mass_at_start.cg_body_m);
        let now = self.now.advanced(period);
        let loads = self
            .dynamics(&mass, input)?
            .evaluate(&state, self.utc_at(now))?
            .into();
        if !state.is_finite() {
            return Err(BlockError::NonFiniteState {
                block: BLOCK,
                detail: format!("{state:?}"),
            });
        }
        self.state = state;
        self.loads = loads;
        self.mass = mass;
        self.now = now;
        Ok(())
    }
}

fn block_error(source: MbdError) -> BlockError {
    BlockError::Mbd {
        block: BLOCK,
        source,
    }
}

impl Block for RigidBody6DofBlock {
    type Input = RigidBodyInput;
    type Output = VehicleState;
    type Snapshot = RigidBodySnapshot;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    /// Advances one period, or leaves the block unchanged and reports why it
    /// could not.
    fn step(&mut self, input: &RigidBodyInput) -> Result<(), BlockError> {
        input.validate()?;
        let saved_states = self.tree.states().to_vec();
        let result = self.advance(input);
        if result.is_err() {
            self.tree
                .restore_states(&saved_states)
                .map_err(block_error)?;
        }
        result
    }

    fn output(&self) -> VehicleState {
        self.state.vehicle(&self.config.body)
    }

    fn snapshot(&self) -> RigidBodySnapshot {
        RigidBodySnapshot {
            state: self.state,
            loads: self.loads,
            mass: self.mass,
            now: self.now,
            component_states: self.tree.states().to_vec(),
        }
    }

    /// Restores `snapshot`. One taken from a differently built tree is logged
    /// and ignored; [`RigidBody6DofBlock::try_restore`] reports it instead.
    fn restore(&mut self, snapshot: &RigidBodySnapshot) {
        if let Err(error) = self.try_restore(snapshot) {
            tracing::error!(%error, "rigid-body snapshot does not fit this vehicle");
        }
    }
}
