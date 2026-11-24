//! Building a vehicle's model, elements and state from its tree and
//! subsystems, and rebuilding them when the tree changes shape.

use mbd_core::{ComponentState, ComponentTree};
use mbd_multibody::{
    FloatingBaseState, ForceElement, ModelState, MultibodyModel, PinnedSurfaceRest, Wheel,
    Workspace,
};

use super::element_state::{read_own, write_own};
use super::step::{block_mbd, block_multi, collect_dynamic_masses, invalid_input, sample_env};
use super::subsystems::MultibodySubsystems;
use super::{
    MultibodyVehicleBlock, MultibodyVehicleConfig, MultibodyVehicleInput, MultibodyVehicleSnapshot,
    MultibodyVehicleTelemetry, VehicleAero, VehicleWorld,
};
use crate::BlockError;
use crate::flight_model::elements::{VehicleElements, wheel_spin_inertia_estimate};
use crate::flight_model::environment::{CgGravity, FlightEnvironment};
use crate::flight_model::integration::{Stepper, scaled_model};
use crate::flight_model::planet_frame::RootMount;

/// Copies the specifications of the elements that live in `subsystems` into
/// `elements`.
fn take_up_specs(elements: &mut VehicleElements, subsystems: &MultibodySubsystems) {
    elements.contacts.clear();
    elements
        .contacts
        .extend(subsystems.contact_points.iter().map(|(spec, _)| *spec));
    elements.wheels.clear();
    elements
        .wheels
        .extend(subsystems.wheels.iter().map(|(spec, _)| Wheel {
            spec: *spec,
            spin_inertia_kg_m2: wheel_spin_inertia_estimate(spec),
            command: mbd_multibody::WheelCommand::default(),
        }));
    if elements.hull != subsystems.hydrodynamics {
        elements.hull.clone_from(&subsystems.hydrodynamics);
    }
    if elements.loops != subsystems.loop_constraints {
        elements.loops.clone_from(&subsystems.loop_constraints);
    }
    elements.mechanisms.clear();
}

/// Whether `state` has the layout of `model` loaded by the vehicle's own
/// `elements`.
fn fits(model: &MultibodyModel, elements: &VehicleElements, state: &ModelState) -> bool {
    let states = ForceElement::<FlightEnvironment<'_>>::state_len(elements);
    let modes = ForceElement::<FlightEnvironment<'_>>::mode_len(elements);
    state.as_slice().len() == model.layout().len() + states && state.modes().len() == modes
}

impl MultibodyVehicleBlock {
    pub(crate) fn build_with_parts(
        config: MultibodyVehicleConfig,
        tree: ComponentTree,
        aero: VehicleAero,
        subsystems: MultibodySubsystems,
        world: VehicleWorld,
        pinned_rest: PinnedSurfaceRest,
    ) -> Result<Self, BlockError> {
        if config.period == crate::TickSpan::ZERO {
            return Err(invalid_input(
                "period",
                "a zero period never advances".to_owned(),
            ));
        }
        let dyn_masses = collect_dynamic_masses(&subsystems)?;
        let mass = tree
            .mass_properties_with_contributions(&dyn_masses)
            .map_err(block_mbd)?;
        let poses = tree.evaluate_pose_table().map_err(block_mbd)?;
        let mount = RootMount::of(&tree).map_err(block_mbd)?;
        let model = scaled_model(&tree, &dyn_masses, config.time_scale, &config.body)
            .map_err(block_multi)?;
        let (aero, canopies): (Vec<_>, Vec<_>) = aero.into_iter().unzip();
        let mut elements = VehicleElements {
            gravity: Some(CgGravity {
                cg_root_link_m: mount.point_in_link(mass.cg_body_m),
            }),
            aero,
            ..VehicleElements::default()
        };
        take_up_specs(&mut elements, &subsystems);
        let base = mount.base_of(&config.initial, mass.cg_body_m);
        let input = MultibodyVehicleInput::default();
        let state = model
            .state(
                &tree,
                &base,
                &elements,
                &world.environment(config.body, config.epoch, &input),
            )
            .map_err(block_multi)?;
        let mut block = Self {
            config,
            tree,
            poses,
            subsystems,
            world,
            scale: model.scale(),
            model,
            mount,
            elements,
            canopies,
            stepper: Stepper::new(config.integrator),
            work: Workspace::new(),
            rates: Vec::new(),
            state,
            mass,
            pinned_rest,
            last_alpha_root_rad_s2: glam::DVec3::ZERO,
            telemetry: MultibodyVehicleTelemetry::default(),
            mechanism_states: Vec::new(),
            now: crate::SimInstant::START,
        };
        block.write_own_states();
        block.telemetry = block.initial_telemetry(&input)?;
        Ok(block)
    }

    /// Telemetry of a vehicle that has not stepped: the air it sits in and
    /// the load that air puts on it.
    fn initial_telemetry(
        &mut self,
        input: &MultibodyVehicleInput,
    ) -> Result<MultibodyVehicleTelemetry, BlockError> {
        let env = sample_env(
            &self.config.body,
            self.world.atmosphere.as_ref(),
            &self.body_axes(),
            self.cg_fixed_m(),
            self.current_utc(),
        )?;
        let state = self.state.clone();
        let loads = self.read_loads(&state, input)?;
        self.last_alpha_root_rad_s2 = loads.alpha_body;
        Ok(MultibodyVehicleTelemetry {
            aero_wrench: loads.aero,
            dynamic_pressure_pa: env.dynamic_pressure_pa,
            specific_force_cg_body: loads.aero.force_body_n / self.mass.total_mass_kg,
            total_propellant_kg: self.subsystems.fluids.total_fluid_mass_kg()
                + self.subsystems.propulsion.total_solid_propellant_kg(),
            bus_voltage_v: self.subsystems.electrical.nominal_bus_voltage_v(),
            ao_remaining_fraction: 1.0,
            used_rigid_fast_path: loads.rigid_fast_path,
            is_pinned_rest: matches!(self.pinned_rest, PinnedSurfaceRest::Pinned { .. }),
            ..MultibodyVehicleTelemetry::default()
        })
    }

    fn write_own_states(&mut self) {
        write_own(
            &self.elements,
            &self.canopies,
            &self.subsystems,
            self.model.layout().len(),
            &mut self.state,
        );
    }

    /// A model state for the vehicle as it is now attached, its root link at
    /// `base`, joints as the tree has them and every element at its typed
    /// state.
    fn fresh_state(&mut self, base: &FloatingBaseState) -> Result<(), BlockError> {
        let input = MultibodyVehicleInput::default();
        let environment = self
            .world
            .environment(self.config.body, self.config.epoch, &input);
        self.state = self
            .model
            .state(&self.tree, base, &self.elements, &environment)
            .map_err(block_multi)?;
        self.write_own_states();
        Ok(())
    }

    /// Prepares the model of the tree as it is now attached, at the scale
    /// the vehicle was built with.
    fn rebuild_model(&mut self) -> Result<(), BlockError> {
        let dyn_masses = collect_dynamic_masses(&self.subsystems)?;
        self.model =
            MultibodyModel::new(&self.tree, &dyn_masses, self.scale).map_err(block_multi)?;
        self.mount = RootMount::of(&self.tree).map_err(block_mbd)?;
        self.mass = self
            .tree
            .mass_properties_with_contributions(&dyn_masses)
            .map_err(block_mbd)?;
        self.poses = self.tree.evaluate_pose_table().map_err(block_mbd)?;
        Ok(())
    }

    /// Brings the elements in line with the subsystems, which the host may
    /// have edited since the last step, and takes their typed states into
    /// the model state.
    pub(super) fn sync_elements(&mut self) -> Result<(), BlockError> {
        take_up_specs(&mut self.elements, &self.subsystems);
        if fits(&self.model, &self.elements, &self.state) {
            self.write_own_states();
            return Ok(());
        }
        let base = self.state.base();
        self.fresh_state(&base)
    }

    /// Updates joint state kinematics directly (e.g. from position-controlled servo loops)
    /// and refreshes the model, pose table and mass properties. Attaching or
    /// detaching a component here reshapes the model.
    pub fn set_component_states(&mut self, states: &[ComponentState]) -> Result<(), BlockError> {
        self.tree.restore_states(states).map_err(block_mbd)?;
        self.rebuild_model()?;
        take_up_specs(&mut self.elements, &self.subsystems);
        let base = self.state.base();
        self.fresh_state(&base)
    }

    /// Restores `snapshot`, returning an error without altering state if tree topology mismatches.
    pub fn try_restore(&mut self, snapshot: &MultibodyVehicleSnapshot) -> Result<(), BlockError> {
        self.tree
            .restore_states(&snapshot.component_states)
            .map_err(block_mbd)?;
        self.subsystems.fluids = snapshot.fluids.clone();
        self.subsystems.propulsion = snapshot.propulsion.clone();
        self.subsystems.electrical = snapshot.electrical.clone();
        self.subsystems.thermal = snapshot.thermal.clone();
        for ((_, dst), src) in self
            .subsystems
            .tps_shields
            .iter_mut()
            .zip(&snapshot.tps_states)
        {
            *dst = *src;
        }
        for ((_, dst), src) in self
            .subsystems
            .ao_coatings
            .iter_mut()
            .zip(&snapshot.ao_states)
        {
            *dst = *src;
        }
        self.rebuild_model()?;
        take_up_specs(&mut self.elements, &self.subsystems);
        if !fits(&self.model, &self.elements, &snapshot.state) {
            return Err(invalid_input(
                "snapshot",
                format!(
                    "model state of {} values does not fit this vehicle",
                    snapshot.state.as_slice().len()
                ),
            ));
        }
        self.state = snapshot.state.clone();
        read_own(
            &self.elements,
            &self.state,
            &mut self.canopies,
            &mut self.subsystems,
        );
        self.last_alpha_root_rad_s2 = snapshot.last_alpha_root_rad_s2;
        self.pinned_rest = snapshot.pinned_rest;
        self.telemetry = snapshot.telemetry;
        self.mechanism_states.clear();
        self.now = snapshot.now;
        Ok(())
    }

    pub(crate) fn replace_topology_state(
        &mut self,
        tree: ComponentTree,
        base_fixed: FloatingBaseState,
        subsystems: MultibodySubsystems,
        aero: VehicleAero,
    ) -> Result<(), BlockError> {
        let dyn_masses = collect_dynamic_masses(&subsystems)?;
        let model = scaled_model(
            &tree,
            &dyn_masses,
            self.config.time_scale,
            &self.config.body,
        )
        .map_err(block_multi)?;
        self.tree = tree;
        self.subsystems = subsystems;
        self.scale = model.scale();
        (self.elements.aero, self.canopies) = aero.into_iter().unzip();
        self.rebuild_model()?;
        take_up_specs(&mut self.elements, &self.subsystems);
        self.fresh_state(&base_fixed)?;
        self.telemetry.total_propellant_kg = self.subsystems.fluids.total_fluid_mass_kg()
            + self.subsystems.propulsion.total_solid_propellant_kg();
        Ok(())
    }

    pub(crate) fn spawn_detached(
        &self,
        tree: ComponentTree,
        base_fixed: FloatingBaseState,
        subsystems: MultibodySubsystems,
        aero: VehicleAero,
    ) -> Result<Self, BlockError> {
        let dyn_masses = collect_dynamic_masses(&subsystems)?;
        let mass = tree
            .mass_properties_with_contributions(&dyn_masses)
            .map_err(block_mbd)?;
        let mount = RootMount::of(&tree).map_err(block_mbd)?;
        let config = MultibodyVehicleConfig {
            initial: mount.vehicle_of(&base_fixed, mass.cg_body_m),
            ..self.config
        };
        let world = VehicleWorld {
            atmosphere: std::sync::Arc::clone(&self.world.atmosphere),
            gravity: std::sync::Arc::clone(&self.world.gravity),
            surface: std::sync::Arc::clone(&self.world.surface),
        };
        let mut child = Self::build_with_parts(
            config,
            tree,
            aero,
            subsystems,
            world,
            PinnedSurfaceRest::Free,
        )?;
        child.state.set_base(&base_fixed);
        child.now = self.now;
        Ok(child)
    }
}
