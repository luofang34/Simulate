use glam::DVec3;
use mbd_core::{DynamicMassContribution, MbdError};
use mbd_multibody::{FloatingBaseState, ModelState, PinnedSurfaceRest};
use mbd_systems::step_solid_motor;
use sim_atmosphere::AtmosphereField;
use sim_geo::{BodyEllipsoid, LocalFrame, UtcTime};

use super::rest::{PinnedHold, hold_pinned_rest, update_pinned_rest};
use super::step_subsystems::{SubsystemStepOut, run_subsystems_step};
use super::subsystems::{MultibodySubsystems, MultibodyVehicleInput, MultibodyVehicleTelemetry};
use super::telemetry::{build_telemetry, merge_dynamic_masses};
use super::{MultibodyVehicleBlock, element_state};
use crate::BlockError;
use crate::flight_model::environment::CgGravity;
use crate::flight_model::integration::{StepSetup, Stepper, step_error};
use crate::flight_model::loads::LoadReadout;

pub(super) const BLOCK_NAME: &str = "multibody_vehicle";

pub(super) fn block_mbd(source: MbdError) -> BlockError {
    BlockError::Mbd {
        block: BLOCK_NAME,
        source,
    }
}

pub(super) fn block_multi(source: mbd_multibody::MultibodyError) -> BlockError {
    BlockError::Multibody {
        block: BLOCK_NAME,
        source,
    }
}

pub(super) fn block_sys(source: mbd_systems::SystemsError) -> BlockError {
    BlockError::Systems {
        block: BLOCK_NAME,
        source,
    }
}

pub(super) fn invalid_input(input: &'static str, detail: String) -> BlockError {
    BlockError::InputOutOfRange {
        block: BLOCK_NAME,
        input,
        detail,
    }
}

pub(super) fn collect_dynamic_masses(
    subsystems: &MultibodySubsystems,
) -> Result<Vec<DynamicMassContribution>, BlockError> {
    let mut out = subsystems.fluids.mass_contributions().map_err(block_sys)?;
    let dummy_ambient = mbd_core::AmbientGas::VACUUM;
    for (spec, state) in &subsystems.propulsion.solid_motors {
        let mut s = *state;
        let grain =
            step_solid_motor(spec, &mut s, 0.0, &dummy_ambient, false).map_err(block_sys)?;
        out.push(grain.grain_mass_contribution);
    }
    Ok(out)
}

/// The air the vehicle's subsystems see at its centre of gravity for the
/// span of a step.
pub(super) struct EnvSample {
    pub(super) ambient: mbd_core::AmbientGas,
    pub(super) air_vel_root_m_s: DVec3,
    pub(super) airspeed_m_s: f64,
    pub(super) dynamic_pressure_pa: f64,
}

/// Samples the air at `cg_fixed_m` for a vehicle whose body axes move as
/// `body_axes`.
pub(super) fn sample_env(
    body: &BodyEllipsoid,
    atmosphere: &dyn AtmosphereField,
    body_axes: &FloatingBaseState,
    cg_fixed_m: DVec3,
    utc: UtcTime,
) -> Result<EnvSample, BlockError> {
    let geo = body.fixed_to_geodetic(cg_fixed_m);
    let sample = atmosphere
        .sample(&geo, utc)
        .map_err(|source| BlockError::Environment {
            block: BLOCK_NAME,
            source,
        })?;
    let wind_fixed = LocalFrame::at(&geo).ned_to_ecef() * sample.wind_ned_m_s;
    let air_vel_root_m_s = body_axes.rotation_world_from_root.conjugate()
        * (body_axes.linear_vel_world_m_s - wind_fixed);
    let ambient = sample.ambient_gas();
    let v2 = air_vel_root_m_s.length_squared();
    Ok(EnvSample {
        ambient,
        air_vel_root_m_s,
        airspeed_m_s: v2.sqrt(),
        dynamic_pressure_pa: 0.5 * ambient.density_kg_m3 * v2,
    })
}

impl MultibodyVehicleBlock {
    /// Body axes of the vehicle as a moving frame, in fixed axes.
    pub(crate) fn body_axes(&self) -> FloatingBaseState {
        self.mount.body_axes(&self.state.base())
    }

    /// Centre of gravity in fixed axes, m.
    pub(super) fn cg_fixed_m(&self) -> DVec3 {
        let axes = self.body_axes();
        axes.position_world_m + axes.rotation_world_from_root * self.mass.cg_body_m
    }

    /// Samples the air and steps every subsystem by `dt_s` with the vehicle
    /// where it is.
    pub(super) fn step_subsystems(
        &mut self,
        dt_s: f64,
        input: &MultibodyVehicleInput,
    ) -> Result<(EnvSample, SubsystemStepOut), BlockError> {
        let body = self.config.body;
        let body_axes = self.body_axes();
        self.poses = self.tree.evaluate_pose_table().map_err(block_mbd)?;
        let env = sample_env(
            &body,
            self.world.atmosphere.as_ref(),
            &body_axes,
            self.cg_fixed_m(),
            self.current_utc(),
        )?;
        let sub = run_subsystems_step(
            dt_s,
            &mut self.tree,
            &self.poses,
            &mut self.subsystems,
            &env,
            body_axes,
            &self.mass,
            self.telemetry.specific_force_cg_body,
            self.last_alpha_root_rad_s2,
            &body,
            input,
        )?;
        Ok((env, sub))
    }

    /// Takes the masses the subsystems now carry into the model and the
    /// vehicle's mass properties.
    pub(super) fn take_up_masses(&mut self, sub: &SubsystemStepOut) -> Result<(), BlockError> {
        let dyn_masses =
            merge_dynamic_masses(&sub.fluid_out.mass_contributions, &sub.grain_contribs);
        self.model
            .refresh_mass(&self.tree, &dyn_masses)
            .map_err(block_multi)?;
        self.mass = self
            .tree
            .mass_properties_with_contributions(&dyn_masses)
            .map_err(block_mbd)?;
        Ok(())
    }

    /// Puts the loads held over a step into the elements: gravity at the
    /// centre of gravity, the wrenches of the subsystems and of `input`,
    /// its joint drives, wheel commands and mechanisms.
    pub(super) fn hold_loads(&mut self, sub: &SubsystemStepOut, input: &MultibodyVehicleInput) {
        let elements = &mut self.elements;
        elements.gravity = Some(CgGravity {
            cg_root_link_m: self.mount.point_in_link(self.mass.cg_body_m),
        });
        elements.wrenches.clear();
        elements.wrenches.extend_from_slice(&sub.prop_wrenches);
        elements
            .wrenches
            .extend_from_slice(&sub.fluid_out.slosh_wrenches);
        elements
            .wrenches
            .extend_from_slice(&input.external_wrenches);
        elements.drives.clear();
        elements.drives.extend_from_slice(&input.joint_drives);
        for (idx, wheel) in elements.wheels.iter_mut().enumerate() {
            wheel.command = input.wheel_commands.get(idx).copied().unwrap_or_default();
        }
        elements.mechanisms.clear();
        elements.mechanisms.extend_from_slice(&input.mechanisms);
    }

    /// Reads the loads at `state`, whose base is in fixed axes, taken as the
    /// state at the start of the current step.
    pub(super) fn read_loads(
        &mut self,
        state: &ModelState,
        input: &MultibodyVehicleInput,
    ) -> Result<LoadReadout, BlockError> {
        let environment = self
            .world
            .environment(self.config.body, self.current_utc(), input);
        let setup = StepSetup {
            model: &self.model,
            elements: &self.elements,
            environment: &environment,
            dt_s: 0.0,
        };
        setup
            .read_loads(&self.mount, state, &mut self.work, &mut self.rates)
            .map_err(|source| step_error(BLOCK_NAME, source))
    }

    /// The state the model is integrated from this step: the vehicle's own,
    /// with the states of this step's mechanisms appended.
    fn step_state(&self, input: &MultibodyVehicleInput) -> Result<ModelState, BlockError> {
        if self.elements.mechanisms.is_empty() {
            return Ok(self.state.clone());
        }
        let base = self.state.base();
        let environment = self
            .world
            .environment(self.config.body, self.current_utc(), input);
        let mut extended = self
            .model
            .state(&self.tree, &base, &self.elements, &environment)
            .map_err(block_multi)?;
        element_state::copy_prefix(&self.state, &mut extended);
        Ok(extended)
    }

    /// Integrates the model across the period under the held loads and
    /// reads the loads at the state it reaches.
    fn integrate(
        &mut self,
        state: &mut ModelState,
        input: &MultibodyVehicleInput,
        override_stepper: Option<&mut Stepper>,
    ) -> Result<LoadReadout, BlockError> {
        // A far end is carried on with the acceleration this vehicle has
        // as the step starts: the field at its root and the specific force
        // last read at its centre of gravity. Both are the same vector in
        // fixed and in world axes at that instant.
        let far_end_acceleration = if self.elements.mechanisms.is_empty() {
            DVec3::ZERO
        } else {
            self.world
                .gravity
                .attraction_fixed(state.base().position_world_m)
                + self.body_axes().rotation_world_from_root * self.telemetry.specific_force_cg_body
        };
        let environment = self
            .world
            .environment(self.config.body, self.current_utc(), input)
            .carrying_far_ends_at(far_end_acceleration);
        let setup = StepSetup {
            model: &self.model,
            elements: &self.elements,
            environment: &environment,
            dt_s: self.config.period.as_seconds(),
        };
        let stepper = override_stepper.unwrap_or(&mut self.stepper);
        setup
            .advance_and_read(&self.mount, stepper, state, &mut self.work, &mut self.rates)
            .map_err(|source| step_error(BLOCK_NAME, source))
    }

    /// Integrates the model across the period under the loads already held
    /// and takes what it reached into the vehicle: joints into the tree, the
    /// mass they now give, the states of the elements and of this step's
    /// mechanisms. Returns the loads read at the end.
    pub(super) fn integrate_held(
        &mut self,
        sub: &SubsystemStepOut,
        input: &MultibodyVehicleInput,
    ) -> Result<LoadReadout, BlockError> {
        self.integrate_held_with(sub, input, None)
    }

    pub(super) fn integrate_held_with(
        &mut self,
        sub: &SubsystemStepOut,
        input: &MultibodyVehicleInput,
        override_stepper: Option<&mut Stepper>,
    ) -> Result<LoadReadout, BlockError> {
        let mut state = self.step_state(input)?;
        let readout = self.integrate(&mut state, input, override_stepper)?;
        self.model
            .write_joints(&state, &mut self.tree)
            .map_err(block_multi)?;
        let dyn_masses =
            merge_dynamic_masses(&sub.fluid_out.mass_contributions, &sub.grain_contribs);
        self.mass = self
            .tree
            .mass_properties_with_contributions(&dyn_masses)
            .map_err(block_mbd)?;
        self.mechanism_states = element_state::mechanism_states(&self.elements, &state);
        element_state::copy_prefix(&state, &mut self.state);
        self.elements.mechanisms.clear();
        Ok(readout)
    }

    pub(super) fn advance_step(
        &mut self,
        input: &MultibodyVehicleInput,
    ) -> Result<MultibodyVehicleTelemetry, BlockError> {
        self.sync_elements()?;
        if let PinnedSurfaceRest::Pinned {
            world_pos_m,
            world_rot,
        } = self.pinned_rest
        {
            let hold = PinnedHold {
                world_pos_m,
                world_rot,
            };
            if let Some(held) = hold_pinned_rest(self, &hold, input)? {
                return Ok(held);
            }
            self.pinned_rest = PinnedSurfaceRest::Free;
        }
        self.integrate_free_step(input)
    }

    fn integrate_free_step(
        &mut self,
        input: &MultibodyVehicleInput,
    ) -> Result<MultibodyVehicleTelemetry, BlockError> {
        let dt_s = self.config.period.as_seconds();
        let (env, sub) = self.step_subsystems(dt_s, input)?;
        self.take_up_masses(&sub)?;
        self.hold_loads(&sub, input);

        let readout = self.integrate_held(&sub, input)?;
        self.last_alpha_root_rad_s2 = readout.alpha_body;
        update_pinned_rest(self, &readout);

        let specific_force = readout.external_force_body_n / self.mass.total_mass_kg;
        Ok(build_telemetry(
            &self.subsystems,
            &env,
            &sub,
            &readout,
            specific_force,
            matches!(self.pinned_rest, PinnedSurfaceRest::Pinned { .. }),
        ))
    }
}
