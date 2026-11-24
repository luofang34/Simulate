use std::sync::Arc;

use glam::DVec3;
use mbd_aero::AeroElement;
use mbd_core::{
    ComponentId, ComponentTree, DynamicMassContribution, LocalSiteKinematics, MassProperties,
    MbdError, PoseTable, compute_site_kinematics,
};
use mbd_multibody::{
    FloatingBaseState, ModelScale, ModelState, MultibodyModel, PinnedSurfaceRest, Workspace,
};
use sim_atmosphere::AtmosphereField;
use sim_geo::{BodyEllipsoid, GravityField, UtcTime};

pub use carried::{CarriedDecoupledStep, CarriedReaction};
use element_state::CanopyState;
pub use mbd_multibody::JointDriveInput;
pub(crate) use step::EnvSample;
use step::{BLOCK_NAME, collect_dynamic_masses};
pub(crate) use step_subsystems::SubsystemStepOut;
use subsystems::validate_input;
pub use subsystems::{
    CgTrimConfig, EllipsoidSurfaceProvider, MultibodyStructuralSnapshot, MultibodySubsystems,
    MultibodyVehicleConfig, MultibodyVehicleInput, MultibodyVehicleSnapshot,
    MultibodyVehicleTelemetry, PropulsionAssembly, PropulsionCommands, SurfaceContactProvider,
};
pub(crate) use topology::{
    VehicleAero, merge_aero_remapped, merge_subsystems_remapped, remap_aero_subset,
    remap_subsystems_subset,
};

use crate::flight_model::coupled_mechanism::CoupledMechanismState;
use crate::flight_model::elements::VehicleElements;
use crate::flight_model::environment::FlightEnvironment;
use crate::flight_model::integration::Stepper;
use crate::flight_model::loads::AeroLoad;
use crate::flight_model::planet_frame::{RootMount, spin};

mod assembly;
mod carried;
mod element_state;
mod rest;
mod step;
mod step_subsystems;
mod step_thermal;
mod subsystems;
mod telemetry;
mod topology;

#[cfg(test)]
mod tests;

/// The fields a vehicle flies through and the surface under it.
pub(crate) struct VehicleWorld {
    pub(crate) atmosphere: Arc<dyn AtmosphereField>,
    pub(crate) gravity: Arc<dyn GravityField>,
    pub(crate) surface: Arc<dyn SurfaceContactProvider>,
}

impl VehicleWorld {
    /// What the force elements sample over a step starting at `utc`, under
    /// the control channels of `input`.
    pub(crate) fn environment<'a>(
        &'a self,
        body: BodyEllipsoid,
        utc: UtcTime,
        input: &'a MultibodyVehicleInput,
    ) -> FlightEnvironment<'a> {
        FlightEnvironment::new(
            body,
            self.gravity.as_ref(),
            self.atmosphere.as_ref(),
            utc,
            &input.control_surfaces_rad,
        )
        .over(self.surface.as_ref())
    }
}

/// Floating-base articulated multibody vehicle with its subsystems.
///
/// One [`MultibodyModel`] of the component tree is loaded by force elements
/// that are evaluated at every integrator stage: gravity at the centre of
/// gravity, the aerodynamic elements on their own links with the air sampled
/// where each one is, footpads and wheels against the surface, the hull in
/// water, and loop constraints. The fluid, propulsion, electrical and thermal
/// subsystems of [`mbd_systems`] step once per period and hold their wrenches
/// and masses over it. The same block flies atmospheric flight, orbital
/// coasting, powered ascent, entry, ground contact, splashdown and articulated
/// manipulation; [`MultibodyVehicleConfig::integrator`] picks the scheme.
pub struct MultibodyVehicleBlock {
    config: MultibodyVehicleConfig,
    tree: ComponentTree,
    poses: PoseTable,
    subsystems: MultibodySubsystems,
    world: VehicleWorld,
    model: MultibodyModel,
    /// Scale of the model, kept across rebuilds so a restored vehicle
    /// integrates with the tolerances it had.
    scale: ModelScale,
    mount: RootMount,
    elements: VehicleElements,
    canopies: Vec<CanopyState>,
    stepper: Stepper,
    work: Workspace,
    rates: Vec<f64>,
    /// The model's state with its base in the central body's fixed axes.
    state: ModelState,
    mass: MassProperties,
    pinned_rest: PinnedSurfaceRest,
    last_alpha_root_rad_s2: DVec3,
    telemetry: MultibodyVehicleTelemetry,
    mechanism_states: Vec<CoupledMechanismState>,
    now: crate::SimInstant,
}

impl MultibodyVehicleBlock {
    /// Constructs a multibody vehicle block carrying the aerodynamic elements
    /// `aero` on the components they name, over the solid ellipsoid of
    /// `config.body` and in its truth gravity field `gravity`: the one
    /// instance the body's catalog entry holds, so the vehicle's first loads
    /// and every sensor aboard meet the same field. A vehicle that flies in
    /// vacuum has no aerodynamic elements.
    pub fn new(
        config: MultibodyVehicleConfig,
        tree: ComponentTree,
        aero: Vec<AeroElement>,
        subsystems: MultibodySubsystems,
        atmosphere: Box<dyn AtmosphereField>,
        gravity: Arc<dyn GravityField>,
    ) -> Result<Self, crate::BlockError> {
        let canopies = element_state::initial_canopies(&aero);
        Self::build_with_parts(
            config,
            tree,
            aero.into_iter().zip(canopies).collect(),
            subsystems,
            VehicleWorld {
                atmosphere: Arc::from(atmosphere),
                gravity,
                surface: Arc::new(EllipsoidSurfaceProvider::solid(config.body)),
            },
            PinnedSurfaceRest::Free,
        )
    }

    /// Replaces the terrain / ocean contact surface provider.
    #[must_use]
    pub fn with_surface_provider(mut self, surface: Arc<dyn SurfaceContactProvider>) -> Self {
        self.world.surface = surface;
        self
    }

    /// Overrides the initial [`PinnedSurfaceRest`] latch state.
    #[must_use]
    pub fn with_pinned_rest(mut self, pinned_rest: PinnedSurfaceRest) -> Self {
        self.pinned_rest = pinned_rest;
        self.telemetry.is_pinned_rest = matches!(pinned_rest, PinnedSurfaceRest::Pinned { .. });
        self
    }

    /// Central body ellipsoid.
    #[must_use]
    pub const fn body(&self) -> BodyEllipsoid {
        self.config.body
    }

    /// Fixed vehicle configuration.
    #[must_use]
    pub const fn config(&self) -> &MultibodyVehicleConfig {
        &self.config
    }

    /// Shared truth gravity field.
    #[must_use]
    pub fn gravity(&self) -> &Arc<dyn GravityField> {
        &self.world.gravity
    }

    /// Shared atmosphere field.
    #[must_use]
    pub fn atmosphere(&self) -> &Arc<dyn AtmosphereField> {
        &self.world.atmosphere
    }

    /// Shared surface contact/ocean provider.
    #[must_use]
    pub fn surface_provider(&self) -> &Arc<dyn SurfaceContactProvider> {
        &self.world.surface
    }

    /// Live component tree.
    #[must_use]
    pub const fn tree(&self) -> &ComponentTree {
        &self.tree
    }

    /// Live evaluated forward-kinematics [`PoseTable`] at [`Self::now`].
    #[must_use]
    pub const fn pose_table(&self) -> &PoseTable {
        &self.poses
    }

    /// Live total mass, CG, and inertia properties.
    #[must_use]
    pub const fn mass_properties(&self) -> &MassProperties {
        &self.mass
    }

    /// Floating-base root pose/velocity expressed in central-body fixed coordinates.
    #[must_use]
    pub fn floating_base_fixed(&self) -> FloatingBaseState {
        self.state.base()
    }

    /// The model's state: base in fixed axes, joints, and the continuous
    /// states and modes of every element.
    #[must_use]
    pub const fn model_state(&self) -> &ModelState {
        &self.state
    }

    /// Where the mechanisms of the latest step's input ended, in its order.
    #[must_use]
    pub fn mechanism_states(&self) -> &[CoupledMechanismState] {
        &self.mechanism_states
    }

    /// Live subsystem bundle (fluids, propulsion, electrical, thermal, manipulators, wheels).
    #[must_use]
    pub const fn subsystems(&self) -> &MultibodySubsystems {
        &self.subsystems
    }

    /// Mutable access to subsystems (for fault injection, tank refill, or latch control).
    pub fn subsystems_mut(&mut self) -> &mut MultibodySubsystems {
        &mut self.subsystems
    }

    /// Latest diagnostic and subsystem telemetry.
    #[must_use]
    pub const fn telemetry(&self) -> &MultibodyVehicleTelemetry {
        &self.telemetry
    }

    /// Current aerodynamic load in body axes.
    #[must_use]
    pub const fn aero_wrench(&self) -> &AeroLoad {
        &self.telemetry.aero_wrench
    }

    /// Current pinned-surface latch state.
    #[must_use]
    pub const fn pinned_rest(&self) -> &PinnedSurfaceRest {
        &self.pinned_rest
    }

    /// Current simulation clock instant.
    #[must_use]
    pub const fn now(&self) -> crate::SimInstant {
        self.now
    }

    /// Collects combined fluid and solid-grain [`DynamicMassContribution`]s.
    pub fn dynamic_mass_contributions(
        &self,
    ) -> Result<Vec<DynamicMassContribution>, crate::BlockError> {
        collect_dynamic_masses(&self.subsystems)
    }

    /// Returns IMU / accelerometer / gyro kinematics at attached component `site`.
    pub fn site_kinematics(&self, site: ComponentId) -> Result<LocalSiteKinematics, MbdError> {
        if !self.tree.is_attached(site)? {
            return Err(MbdError::DetachedComponent { id: site });
        }
        let pose = self.poses.get(site)?;
        let axes = self.mount.body_axes(&self.state.base());
        let omega_i = axes.angular_vel_root_rad_s
            + axes.rotation_world_from_root.conjugate() * spin(&self.config.body);
        Ok(compute_site_kinematics(
            self.telemetry.specific_force_cg_body,
            omega_i,
            self.last_alpha_root_rad_s2,
            (pose.position_root_m, pose.rotation_root_from_link),
            self.mass.cg_body_m,
        ))
    }

    /// The aerodynamic elements with the canopy state each one holds.
    pub(crate) fn aero_with_states(&self) -> VehicleAero {
        self.elements
            .aero
            .iter()
            .cloned()
            .zip(self.canopies.iter().copied())
            .collect()
    }

    pub(crate) fn snapshot_for_coord(&self) -> MultibodyVehicleSnapshot {
        crate::Block::snapshot(self)
    }

    pub(crate) fn world_environment<'a>(
        &'a self,
        input: &'a MultibodyVehicleInput,
    ) -> FlightEnvironment<'a> {
        self.world
            .environment(self.config.body, self.current_utc(), input)
    }

    pub(crate) const fn model(&self) -> &MultibodyModel {
        &self.model
    }

    pub(crate) const fn mount(&self) -> &RootMount {
        &self.mount
    }

    pub(crate) const fn elements(&self) -> &VehicleElements {
        &self.elements
    }

    pub(crate) fn elements_mut(&mut self) -> &mut VehicleElements {
        &mut self.elements
    }

    pub(crate) fn current_utc(&self) -> UtcTime {
        let elapsed = self
            .now
            .duration_since(crate::SimInstant::START)
            .as_seconds();
        self.config.epoch.advanced_by(elapsed)
    }

    fn vehicle_state(&self) -> crate::VehicleState {
        self.mount
            .vehicle_of(&self.state.base(), self.mass.cg_body_m)
    }

    pub(crate) fn finalize_step_telemetry(
        &mut self,
        tel: MultibodyVehicleTelemetry,
    ) -> Result<(), crate::BlockError> {
        let state = self.vehicle_state();
        if !(state.position_ecef.is_finite()
            && state.velocity_ecef.is_finite()
            && state.attitude.is_finite()
            && state.angular_rate_body.is_finite())
        {
            return Err(crate::BlockError::NonFiniteState {
                block: BLOCK_NAME,
                detail: format!("{state:?}"),
            });
        }
        self.poses = self.tree.evaluate_pose_table().map_err(step::block_mbd)?;
        element_state::read_own(
            &self.elements,
            &self.state,
            &mut self.canopies,
            &mut self.subsystems,
        );
        self.telemetry = tel;
        self.now = self.now.advanced(self.config.period);
        Ok(())
    }

    fn run_step(&mut self, input: &MultibodyVehicleInput) -> Result<(), crate::BlockError> {
        validate_input(input)?;
        let tel = self.advance_step(input)?;
        self.finalize_step_telemetry(tel)
    }
}

impl crate::Block for MultibodyVehicleBlock {
    type Input = MultibodyVehicleInput;
    type Output = crate::VehicleState;
    type Snapshot = MultibodyVehicleSnapshot;

    fn period(&self) -> crate::TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &Self::Input) -> Result<(), crate::BlockError> {
        let saved = self.snapshot_for_coord();
        let res = self.run_step(input);
        if res.is_err()
            && let Err(error) = self.try_restore(&saved)
        {
            tracing::error!(%error, "multibody rollback failed");
        }
        res
    }

    fn output(&self) -> Self::Output {
        self.vehicle_state()
    }

    fn snapshot(&self) -> Self::Snapshot {
        MultibodyVehicleSnapshot {
            now: self.now,
            state: self.state.clone(),
            last_alpha_root_rad_s2: self.last_alpha_root_rad_s2,
            component_states: self.tree.states().to_vec(),
            fluids: self.subsystems.fluids.clone(),
            propulsion: self.subsystems.propulsion.clone(),
            electrical: self.subsystems.electrical.clone(),
            thermal: self.subsystems.thermal.clone(),
            tps_states: self
                .subsystems
                .tps_shields
                .iter()
                .map(|(_, s)| *s)
                .collect(),
            ao_states: self
                .subsystems
                .ao_coatings
                .iter()
                .map(|(_, s)| *s)
                .collect(),
            pinned_rest: self.pinned_rest,
            telemetry: self.telemetry,
        }
    }

    fn restore(&mut self, snapshot: &Self::Snapshot) {
        if let Err(error) = self.try_restore(snapshot) {
            tracing::error!(%error, "multibody snapshot does not fit this vehicle");
        }
    }
}
