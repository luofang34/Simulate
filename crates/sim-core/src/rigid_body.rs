use std::sync::Arc;

use glam::DVec3;
use mbd_aero::AeroElement;
use mbd_core::{
    ComponentId, ComponentState, ComponentTree, LocalSiteKinematics, MassProperties, MbdError,
    compute_site_kinematics,
};
use mbd_multibody::{ModelState, MultibodyModel, Workspace};
use sim_atmosphere::AtmosphereField;
use sim_geo::{BodyEllipsoid, GravityField, UtcTime};

use crate::flight_model::elements::VehicleElements;
use crate::flight_model::integration::{Stepper, VehicleIntegrator};
use crate::flight_model::loads::AeroLoad;
use crate::flight_model::planet_frame::{RootMount, spin};
use crate::{Block, BlockError, SimInstant, TickSpan, VehicleState};

use motion::{BLOCK, Loads, block_error};

mod motion;

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
    /// Control channels the aerodynamic elements read, in the order they
    /// name them: surface deflections, rad.
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

/// Complete dynamic state of a [`RigidBody6DofBlock`], component states
/// included, for exact replay and rollback.
#[derive(Debug, Clone, PartialEq)]
pub struct RigidBodySnapshot {
    state: ModelState,
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
/// The vehicle is the multibody model of its tree with every joint held:
/// one rigid body loaded by gravity at its centre of gravity, by the
/// aerodynamic elements mounted on its components, and by the thruster
/// wrenches of the step. It is integrated by one fourth-order Runge-Kutta
/// step per period in inertial axes, and
/// [`VehicleState::angular_rate_body`] reports the rate relative to the
/// central body. Mass properties and thruster loads hold constant across a
/// step: the rate of change of inertia and of the centre of gravity within a
/// step, and jet damping, are not modelled.
///
/// The integrated state is the airframe's. The reported state follows the
/// centre of gravity, so when a burn moves it within the airframe the
/// report moves to the material point now there and the airframe's own path
/// is undisturbed.
pub struct RigidBody6DofBlock {
    config: RigidBodyConfig,
    tree: ComponentTree,
    atmosphere: Box<dyn AtmosphereField>,
    gravity: Arc<dyn GravityField>,
    model: MultibodyModel,
    mount: RootMount,
    elements: VehicleElements,
    stepper: Stepper,
    work: Workspace,
    rates: Vec<f64>,
    /// Root link state, its base in the central body's fixed axes.
    state: ModelState,
    loads: Loads,
    mass: MassProperties,
    now: SimInstant,
}

impl RigidBody6DofBlock {
    /// A block at `config.initial` carrying the aerodynamic elements `aero`
    /// on the components they name, with loads evaluated for no thrust.
    /// Fails if the period is zero or the tree or environment cannot be
    /// evaluated.
    pub fn new(
        config: RigidBodyConfig,
        tree: ComponentTree,
        aero: Vec<AeroElement>,
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
        motion::check_inertia(&mass)?;
        let mount = RootMount::of(&tree).map_err(block_error)?;
        let model = motion::model_of(&tree, &config)?;
        let elements = VehicleElements {
            aero,
            ..VehicleElements::default()
        };
        let gravity: Arc<dyn GravityField> = Arc::new(config.body);
        let state = motion::initial_state(&motion::Start {
            config: &config,
            tree: &tree,
            model: &model,
            mount: &mount,
            elements: &elements,
            atmosphere: atmosphere.as_ref(),
            gravity: gravity.as_ref(),
            cg_body_m: mass.cg_body_m,
        })?;
        let mut block = Self {
            config,
            tree,
            atmosphere,
            gravity,
            model,
            mount,
            elements,
            stepper: Stepper::new(VehicleIntegrator::REAL_TIME),
            work: Workspace::new(),
            rates: Vec::new(),
            state,
            loads: Loads::NONE,
            mass,
            now: SimInstant::START,
        };
        block.loads = block.loads_at_output(&RigidBodyInput::default(), &mass, block.now)?;
        Ok(block)
    }

    /// The block in a world whose truth gravity is `gravity` instead of the
    /// reference field of `config.body`. Hand the same instance to every
    /// sensor that senses gravity, so they read the field the vehicle flies
    /// through.
    pub fn with_gravity(mut self, gravity: Arc<dyn GravityField>) -> Self {
        self.gravity = gravity;
        self
    }

    /// The truth gravity the block integrates.
    pub fn gravity(&self) -> &Arc<dyn GravityField> {
        &self.gravity
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
        let state = self.output();
        state.angular_rate_body + state.attitude.inverse() * spin(&self.config.body)
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
    pub fn aero_wrench(&self) -> &AeroLoad {
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
            self.omega_inertial_body(),
            self.loads.alpha_body,
            self.tree.world_transform_in_root(site)?,
            self.mass.cg_body_m,
        ))
    }

    /// Separates `stage` and its subtree at the output instant, as a staging
    /// event does.
    ///
    /// The airframe keeps its motion and the report moves to the remaining
    /// vehicle's new centre of gravity: with no separation impulse each part
    /// keeps its rigid motion. Loads are re-evaluated with no thrust until
    /// the next step supplies some. A stage already separated, or one that
    /// would leave nothing attached, is an error and leaves the block
    /// unchanged.
    pub fn separate_stage(&mut self, stage: ComponentId) -> Result<(), BlockError> {
        if !self.tree.is_attached(stage).map_err(block_error)? {
            return Err(block_error(MbdError::DetachedComponent { id: stage }));
        }
        let saved_states = self.tree.states().to_vec();
        let result = self.detach(stage);
        if result.is_err() {
            self.put_tree_back(&saved_states)?;
        }
        result
    }

    /// Restores `snapshot`, or reports that it came from a differently built
    /// tree and leaves the block unchanged.
    pub fn try_restore(&mut self, snapshot: &RigidBodySnapshot) -> Result<(), MbdError> {
        let saved_states = self.tree.states().to_vec();
        self.tree.restore_states(&snapshot.component_states)?;
        match motion::model_of(&self.tree, &self.config) {
            Ok(model) if motion::fits(&model, &self.elements, &snapshot.state) => {
                self.model = model;
            }
            _ => {
                self.tree.restore_states(&saved_states)?;
                return Err(MbdError::StateCountMismatch {
                    expected: self.state.as_slice().len(),
                    actual: snapshot.state.as_slice().len(),
                });
            }
        }
        self.state = snapshot.state.clone();
        self.loads = snapshot.loads;
        self.mass = snapshot.mass;
        self.now = snapshot.now;
        Ok(())
    }

    fn utc_at(&self, instant: SimInstant) -> UtcTime {
        let elapsed = instant.duration_since(SimInstant::START).as_seconds();
        self.config.epoch.advanced_by(elapsed)
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
            self.put_tree_back(&saved_states)?;
        }
        result
    }

    fn output(&self) -> VehicleState {
        self.mount
            .vehicle_of(&self.state.base(), self.mass.cg_body_m)
    }

    fn snapshot(&self) -> RigidBodySnapshot {
        RigidBodySnapshot {
            state: self.state.clone(),
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
