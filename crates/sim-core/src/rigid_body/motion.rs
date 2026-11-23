//! Stepping the rigid vehicle: its model, the loads held over a step, and
//! what is read back at the output instant.

use glam::DVec3;
use mbd_core::{
    ComponentId, ComponentState, ComponentTree, MassProperties, MbdError, WrenchAtSite,
};
use mbd_multibody::{ForceElement, ModelState, MultibodyError, MultibodyModel};
use sim_atmosphere::AtmosphereField;
use sim_geo::GravityField;

use super::{PropellantBurn, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput};
use crate::flight_model::elements::VehicleElements;
use crate::flight_model::environment::{CgGravity, FlightEnvironment};
use crate::flight_model::integration::{MotionTimeScale, StepSetup, scaled_model};
use crate::flight_model::loads::AeroLoad;
use crate::flight_model::planet_frame::RootMount;
use crate::{Block, BlockError, SimInstant};

/// The name [`BlockError`]s from this block carry.
pub(super) const BLOCK: &str = "rigid_body_6dof";

/// Smallest ratio of the inertia determinant to the cube of its mean moment.
/// Below this the tensor is a point or a line, whose Euler equation has no
/// solution.
const MIN_INERTIA_CONDITION: f64 = 1e-12;

pub(super) fn block_error(source: MbdError) -> BlockError {
    BlockError::Mbd {
        block: BLOCK,
        source,
    }
}

fn model_error(source: MultibodyError) -> BlockError {
    match source {
        MultibodyError::Mbd(source) => block_error(source),
        source => BlockError::Multibody {
            block: BLOCK,
            source,
        },
    }
}

/// The state of the block's output instant that is derived from its forces,
/// so a restored block answers [`RigidBody6DofBlock::site_kinematics`] exactly
/// as the original did.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Loads {
    pub(super) specific_force_cg_body: DVec3,
    pub(super) alpha_body: DVec3,
    pub(super) aero: AeroLoad,
}

impl Loads {
    pub(super) const NONE: Self = Self {
        specific_force_cg_body: DVec3::ZERO,
        alpha_body: DVec3::ZERO,
        aero: AeroLoad::ZERO,
    };
}

/// Rejects an inertia about the centre of gravity too degenerate to turn.
pub(super) fn check_inertia(mass: &MassProperties) -> Result<(), BlockError> {
    let inertia = mass.inertia_cg_body;
    let mean_moment = (inertia.x_axis.x + inertia.y_axis.y + inertia.z_axis.z) / 3.0;
    if inertia.determinant() <= MIN_INERTIA_CONDITION * mean_moment.powi(3) || mean_moment <= 0.0 {
        return Err(BlockError::InputOutOfRange {
            block: BLOCK,
            input: "inertia_cg_body",
            detail: format!("{inertia} cannot be inverted"),
        });
    }
    Ok(())
}

/// The model of `tree` as it is attached now. A single fixed step per
/// period weighs no error, so the time of the model's scale is the period
/// itself: the span over which the block resolves motion.
pub(super) fn model_of(
    tree: &ComponentTree,
    config: &RigidBodyConfig,
) -> Result<MultibodyModel, BlockError> {
    let time = MotionTimeScale::Seconds(config.period.as_seconds());
    scaled_model(tree, &[], time, &config.body).map_err(model_error)
}

/// Whether `state` has the layout of `model` loaded by `elements`.
pub(super) fn fits(model: &MultibodyModel, elements: &VehicleElements, state: &ModelState) -> bool {
    let states = ForceElement::<FlightEnvironment<'_>>::state_len(elements);
    let modes = ForceElement::<FlightEnvironment<'_>>::mode_len(elements);
    state.as_slice().len() == model.layout().len() + states && state.modes().len() == modes
}

/// What the first state of a block is built from.
pub(super) struct Start<'a> {
    pub(super) config: &'a RigidBodyConfig,
    pub(super) tree: &'a ComponentTree,
    pub(super) model: &'a MultibodyModel,
    pub(super) mount: &'a RootMount,
    pub(super) elements: &'a VehicleElements,
    pub(super) atmosphere: &'a dyn AtmosphereField,
    pub(super) gravity: &'a dyn GravityField,
    pub(super) cg_body_m: DVec3,
}

/// The model state of a vehicle at `config.initial`, every element at its
/// own initial state.
pub(super) fn initial_state(start: &Start<'_>) -> Result<ModelState, BlockError> {
    let base = start.mount.base_of(&start.config.initial, start.cg_body_m);
    let environment = FlightEnvironment::new(
        start.config.body,
        start.gravity,
        start.atmosphere,
        start.config.epoch,
        &[],
        &base,
    );
    start
        .model
        .state(start.tree, &base, start.elements, &environment)
        .map_err(model_error)
}

impl RigidBody6DofBlock {
    /// Puts the thruster loads of `input` and the centre of gravity of
    /// `mass` into the elements held over a step.
    fn hold(&mut self, input: &RigidBodyInput, mass: &MassProperties) {
        let mount = self.mount;
        self.elements.gravity = Some(CgGravity {
            cg_root_link_m: mount.point_in_link(mass.cg_body_m),
        });
        self.elements.wrenches.clear();
        self.elements
            .wrenches
            .extend(input.thrusters.iter().map(|thruster| WrenchAtSite {
                site: mount.component,
                point_local_m: mount.point_in_link(thruster.application_point_body_m),
                force_local_n: mount.vector_in_link(thruster.force_body_n),
                torque_local_nm: mount.vector_in_link(thruster.moment_body_nm),
            }));
    }

    /// The loads at the block's current state taken as the state of
    /// `instant`, under `input` and with mass `mass`.
    pub(super) fn loads_at_output(
        &mut self,
        input: &RigidBodyInput,
        mass: &MassProperties,
        instant: SimInstant,
    ) -> Result<Loads, BlockError> {
        self.hold(input, mass);
        let body = self.config.body;
        let environment = FlightEnvironment::new(
            body,
            self.gravity.as_ref(),
            self.atmosphere.as_ref(),
            self.utc_at(instant),
            &input.control_surfaces_rad,
            &self.state.base(),
        );
        let setup = StepSetup {
            model: &self.model,
            elements: &self.elements,
            environment: &environment,
            dt_s: 0.0,
        };
        let readout = setup
            .read_loads(&self.mount, &self.state, &mut self.work, &mut self.rates)
            .map_err(|fault| fault.into_block_error(BLOCK))?;
        Ok(Loads {
            specific_force_cg_body: readout.external_force_body_n / mass.total_mass_kg,
            alpha_body: readout.alpha_body,
            aero: readout.aero,
        })
    }

    fn apply_burns(&mut self, burns: &[PropellantBurn], dt_s: f64) -> Result<(), MbdError> {
        for burn in burns {
            let mut tank = *self.tree.state(burn.tank)?;
            tank.fluid_mass_kg = (tank.fluid_mass_kg + burn.dm_dt_kg_s * dt_s).max(0.0);
            self.tree.set_state(burn.tank, tank)?;
        }
        Ok(())
    }

    /// Restores the tree to `states` and everything derived from it, after
    /// an operation that changed the tree and then failed.
    pub(super) fn put_tree_back(&mut self, states: &[ComponentState]) -> Result<(), BlockError> {
        self.tree.restore_states(states).map_err(block_error)?;
        self.model = model_of(&self.tree, &self.config)?;
        self.mass = self.tree.mass_properties().map_err(block_error)?;
        Ok(())
    }

    /// Everything that can fail in a separation. The caller puts the tree
    /// and what follows from it back on failure.
    pub(super) fn detach(&mut self, stage: ComponentId) -> Result<(), BlockError> {
        let mut detached = *self.tree.state(stage).map_err(block_error)?;
        detached.attached = false;
        self.tree.set_state(stage, detached).map_err(block_error)?;
        let mass = self.tree.mass_properties().map_err(block_error)?;
        check_inertia(&mass)?;
        self.model = model_of(&self.tree, &self.config)?;
        let loads = self.loads_at_output(&RigidBodyInput::default(), &mass, self.now)?;
        self.mass = mass;
        self.loads = loads;
        Ok(())
    }

    /// Everything that can fail in a step. The integrated state is put back
    /// here on failure; the caller puts back the tree and what follows from
    /// it.
    pub(super) fn advance(&mut self, input: &RigidBodyInput) -> Result<(), BlockError> {
        let start = self.state.clone();
        let result = self.integrate(input);
        if result.is_err() {
            self.state = start;
        }
        result
    }

    fn integrate(&mut self, input: &RigidBodyInput) -> Result<(), BlockError> {
        let period = self.config.period;
        let dt_s = period.as_seconds();
        let mass_at_start = self.mass;
        check_inertia(&mass_at_start)?;
        self.hold(input, &mass_at_start);
        let environment = FlightEnvironment::new(
            self.config.body,
            self.gravity.as_ref(),
            self.atmosphere.as_ref(),
            self.utc_at(self.now),
            &input.control_surfaces_rad,
            &self.state.base(),
        )
        .pushed_by(self.output().attitude * self.loads.specific_force_cg_body);
        let setup = StepSetup {
            model: &self.model,
            elements: &self.elements,
            environment: &environment,
            dt_s,
        };
        setup
            .advance(&mut self.stepper, &mut self.state, &mut self.work)
            .map_err(|fault| fault.into_block_error(BLOCK))?;

        self.apply_burns(&input.burns, dt_s).map_err(block_error)?;
        let mass = self.tree.mass_properties().map_err(block_error)?;
        self.model
            .refresh_mass(&self.tree, &[])
            .map_err(model_error)?;
        let now = self.now.advanced(period);
        let loads = self.loads_at_output(input, &mass, now)?;
        self.mass = mass;
        let state = self.output();
        if !(state.position_ecef.is_finite()
            && state.velocity_ecef.is_finite()
            && state.attitude.is_finite()
            && state.angular_rate_body.is_finite())
        {
            return Err(BlockError::NonFiniteState {
                block: BLOCK,
                detail: format!("{state:?}"),
            });
        }
        self.loads = loads;
        self.now = now;
        Ok(())
    }
}
