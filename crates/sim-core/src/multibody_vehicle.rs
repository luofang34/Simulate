use std::sync::Arc;

use glam::DVec3;
use mbd_aero::CompositeAeroModel;
use mbd_core::{
    AeroModel, AeroWrench, ComponentId, ComponentState, ComponentTree, DynamicMassContribution,
    LocalSiteKinematics, MassProperties, MbdError, PoseTable, compute_site_kinematics,
};
use mbd_multibody::{FloatingBaseState, MultiBodyDynamics, PinnedSurfaceRest};
use sim_atmosphere::AtmosphereField;
use sim_geo::{BodyEllipsoid, GravityField, UtcTime};

use step::{
    BLOCK_NAME, StepContext, advance_step, block_mbd, block_multi, collect_dynamic_masses,
    invalid_input, sample_env_and_aero,
};
use subsystems::{AeroStorage, fixed_base_from_vehicle, validate_input, vehicle_from_fixed_base};
pub use subsystems::{
    CgTrimConfig, EllipsoidSurfaceProvider, MultibodySubsystems, MultibodyVehicleConfig,
    MultibodyVehicleInput, MultibodyVehicleSnapshot, MultibodyVehicleTelemetry, PropulsionAssembly,
    PropulsionCommands, SurfaceContactProvider,
};
pub(crate) use subsystems::{fixed_to_instantaneous_inertial, inertial_to_fixed_after_step};
pub(crate) use topology::{
    merge_aero_remapped, merge_subsystems_remapped, remap_aero_subset, remap_subsystems_subset,
};

mod step;
mod step_contacts;
mod step_subsystems;
mod subsystems;
mod topology;

#[cfg(test)]
mod tests;

/// Unified floating-base Featherstone multibody and subsystem vehicle block.
///
/// Shares one physical representation (`ComponentTree`, [`mbd_aero`], [`mbd_multibody`],
/// and [`mbd_systems`]) across atmospheric flight, orbital coasting, powered ascent,
/// hypersonic ablation, ground wheel/footpad contact, splashdown hydrodynamics, and
/// articulated robotic manipulation.
pub struct MultibodyVehicleBlock {
    config: MultibodyVehicleConfig,
    tree: ComponentTree,
    poses: PoseTable,
    subsystems: MultibodySubsystems,
    aero: AeroStorage,
    atmosphere: Arc<dyn AtmosphereField>,
    gravity: Arc<dyn GravityField>,
    surface: Arc<dyn SurfaceContactProvider>,
    base_fixed: FloatingBaseState,
    mass: MassProperties,
    pinned_rest: PinnedSurfaceRest,
    last_alpha_root_rad_s2: DVec3,
    telemetry: MultibodyVehicleTelemetry,
    now: crate::SimInstant,
}

impl MultibodyVehicleBlock {
    /// Constructs a multibody vehicle block with composite multi-element aerodynamics
    /// and complete [`MultibodySubsystems`].
    pub fn new(
        config: MultibodyVehicleConfig,
        tree: ComponentTree,
        aero: CompositeAeroModel,
        subsystems: MultibodySubsystems,
        atmosphere: Box<dyn AtmosphereField>,
    ) -> Result<Self, crate::BlockError> {
        Self::build_with_storage(
            config,
            tree,
            AeroStorage::Composite(aero),
            subsystems,
            Arc::from(atmosphere),
            Arc::new(config.body),
            Arc::new(EllipsoidSurfaceProvider::solid(config.body)),
            PinnedSurfaceRest::Free,
        )
    }

    /// Constructs a multibody vehicle block with any boxed [`AeroModel`].
    pub fn with_aero_model(
        config: MultibodyVehicleConfig,
        tree: ComponentTree,
        aero: Box<dyn AeroModel>,
        subsystems: MultibodySubsystems,
        atmosphere: Box<dyn AtmosphereField>,
    ) -> Result<Self, crate::BlockError> {
        Self::build_with_storage(
            config,
            tree,
            AeroStorage::Dynamic(aero),
            subsystems,
            Arc::from(atmosphere),
            Arc::new(config.body),
            Arc::new(EllipsoidSurfaceProvider::solid(config.body)),
            PinnedSurfaceRest::Free,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn build_with_storage(
        config: MultibodyVehicleConfig,
        tree: ComponentTree,
        aero: AeroStorage,
        subsystems: MultibodySubsystems,
        atmosphere: Arc<dyn AtmosphereField>,
        gravity: Arc<dyn GravityField>,
        surface: Arc<dyn SurfaceContactProvider>,
        pinned_rest: PinnedSurfaceRest,
    ) -> Result<Self, crate::BlockError> {
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
        let base_fixed = fixed_base_from_vehicle(&config.initial, &mass);
        let env = sample_env_and_aero(
            &config.body,
            atmosphere.as_ref(),
            &aero,
            &poses,
            &base_fixed,
            &mass,
            config.epoch,
            [0.0; 8],
        )?;
        let cg_fixed =
            base_fixed.position_world_m + base_fixed.rotation_world_from_root * mass.cg_body_m;
        let g_fixed = gravity.attraction_fixed(cg_fixed);
        let base_i = fixed_to_instantaneous_inertial(&base_fixed, &config.body);
        let accel = MultiBodyDynamics::forward_dynamics(
            &tree,
            &dyn_masses,
            &base_i,
            g_fixed,
            &[],
            &[env.aero_site_wrench],
        )
        .map_err(block_multi)?;
        let telemetry = MultibodyVehicleTelemetry {
            aero_wrench: env.aero_root,
            dynamic_pressure_pa: env.dynamic_pressure_pa,
            specific_force_cg_body: env.aero_root.force_body_n / mass.total_mass_kg,
            total_propellant_kg: subsystems.fluids.total_fluid_mass_kg()
                + subsystems.propulsion.total_solid_propellant_kg(),
            bus_voltage_v: subsystems.electrical.nominal_bus_voltage_v(),
            ao_remaining_fraction: 1.0,
            used_rigid_fast_path: accel.used_rigid_fast_path,
            is_pinned_rest: matches!(pinned_rest, PinnedSurfaceRest::Pinned { .. }),
            ..MultibodyVehicleTelemetry::default()
        };
        Ok(Self {
            config,
            tree,
            poses,
            subsystems,
            aero,
            atmosphere,
            gravity,
            surface,
            base_fixed,
            mass,
            pinned_rest,
            last_alpha_root_rad_s2: accel.base_spatial_accel_root.ang,
            telemetry,
            now: crate::SimInstant::START,
        })
    }

    /// Replaces the truth gravity field read during integration.
    #[must_use]
    pub fn with_gravity(mut self, gravity: Arc<dyn GravityField>) -> Self {
        self.gravity = gravity;
        self
    }

    /// Replaces the terrain / ocean contact surface provider.
    #[must_use]
    pub fn with_surface_provider(mut self, surface: Arc<dyn SurfaceContactProvider>) -> Self {
        self.surface = surface;
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
        &self.gravity
    }

    /// Shared atmosphere field.
    #[must_use]
    pub fn atmosphere(&self) -> &Arc<dyn AtmosphereField> {
        &self.atmosphere
    }

    /// Shared surface contact/ocean provider.
    #[must_use]
    pub fn surface_provider(&self) -> &Arc<dyn SurfaceContactProvider> {
        &self.surface
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
    pub const fn floating_base_fixed(&self) -> &FloatingBaseState {
        &self.base_fixed
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

    /// Current aerodynamic load in root axes.
    #[must_use]
    pub const fn aero_wrench(&self) -> &AeroWrench {
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
        let omega_i = self.base_fixed.angular_vel_root_rad_s
            + self.base_fixed.rotation_world_from_root.conjugate()
                * (DVec3::Z * self.config.body.omega_rad_s);
        Ok(compute_site_kinematics(
            self.telemetry.specific_force_cg_body,
            omega_i,
            self.last_alpha_root_rad_s2,
            (pose.position_root_m, pose.rotation_root_from_link),
            self.mass.cg_body_m,
        ))
    }

    /// Updates joint state kinematics directly (e.g. from position-controlled servo loops)
    /// and refreshes the pose table and mass properties.
    pub fn set_component_states(
        &mut self,
        states: &[ComponentState],
    ) -> Result<(), crate::BlockError> {
        self.tree.restore_states(states).map_err(block_mbd)?;
        self.poses = self.tree.evaluate_pose_table().map_err(block_mbd)?;
        let dyn_masses = collect_dynamic_masses(&self.subsystems)?;
        self.mass = self
            .tree
            .mass_properties_with_contributions(&dyn_masses)
            .map_err(block_mbd)?;
        Ok(())
    }

    pub(crate) fn aero_storage(&self) -> &AeroStorage {
        &self.aero
    }

    pub(crate) fn snapshot_for_coord(&self) -> MultibodyVehicleSnapshot {
        crate::Block::snapshot(self)
    }

    fn current_utc(&self) -> UtcTime {
        let elapsed = self
            .now
            .duration_since(crate::SimInstant::START)
            .as_seconds();
        self.config.epoch.advanced_by(elapsed)
    }

    fn run_step(&mut self, input: &MultibodyVehicleInput) -> Result<(), crate::BlockError> {
        validate_input(input)?;
        let ctx = StepContext {
            body: self.config.body,
            gravity: self.gravity.as_ref(),
            atmosphere: self.atmosphere.as_ref(),
            surface: self.surface.as_ref(),
            aero: &self.aero,
            utc: self.current_utc(),
            dt_s: self.config.period.as_seconds(),
        };
        let tel = advance_step(
            &ctx,
            &mut self.tree,
            &mut self.subsystems,
            &mut self.base_fixed,
            &mut self.mass,
            &mut self.pinned_rest,
            self.telemetry.specific_force_cg_body,
            &mut self.last_alpha_root_rad_s2,
            input,
        )?;
        let next_now = self.now.advanced(self.config.period);
        let state = vehicle_from_fixed_base(&self.base_fixed, &self.mass);
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
        self.poses = self.tree.evaluate_pose_table().map_err(block_mbd)?;
        self.telemetry = tel;
        self.now = next_now;
        Ok(())
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
        vehicle_from_fixed_base(&self.base_fixed, &self.mass)
    }

    fn snapshot(&self) -> Self::Snapshot {
        MultibodyVehicleSnapshot {
            now: self.now,
            base_fixed: self.base_fixed,
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
            contact_states: self
                .subsystems
                .contact_points
                .iter()
                .map(|(_, s)| *s)
                .collect(),
            wheel_states: self.subsystems.wheels.iter().map(|(_, s)| *s).collect(),
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
