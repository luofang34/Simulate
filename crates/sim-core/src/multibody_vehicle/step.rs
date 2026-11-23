use glam::{DQuat, DVec3};
use mbd_core::{
    AeroFlowState, AeroWrench, ComponentId, ComponentTree, DynamicMassContribution, MassProperties,
    MbdError, PoseTable, WrenchAtSite,
};
use mbd_multibody::{
    FloatingBaseState, JointDriveInput, MultiBodyDynamics, PinnedSurfaceRest, step_rk4_with_forces,
};
use mbd_systems::step_solid_motor;
use sim_atmosphere::AtmosphereField;
use sim_geo::{BodyEllipsoid, GravityField, LocalFrame, UtcTime};

use super::step_contacts::{
    build_telemetry, collect_free_forces, effective_gravity_fixed, merge_dynamic_masses,
    surface_up_fixed, update_pinned_rest_step,
};
use super::step_subsystems::run_subsystems_step;
use super::subsystems::{
    AeroStorage, MultibodySubsystems, MultibodyVehicleInput, MultibodyVehicleTelemetry,
    SurfaceContactProvider, fixed_to_instantaneous_inertial, inertial_to_fixed_after_step,
};
use crate::BlockError;

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

pub(super) struct EnvSample {
    pub(super) ambient: mbd_core::AmbientGas,
    pub(super) air_vel_root_m_s: DVec3,
    pub(super) airspeed_m_s: f64,
    pub(super) dynamic_pressure_pa: f64,
    pub(super) aero_root: AeroWrench,
    pub(super) aero_site_wrench: WrenchAtSite,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn sample_env_and_aero(
    body: &BodyEllipsoid,
    atmosphere: &dyn AtmosphereField,
    aero: &AeroStorage,
    poses: &PoseTable,
    base_fixed: &FloatingBaseState,
    mass: &MassProperties,
    utc: UtcTime,
    control_surfaces_rad: [f64; 8],
) -> Result<EnvSample, BlockError> {
    let cg_fixed =
        base_fixed.position_world_m + base_fixed.rotation_world_from_root * mass.cg_body_m;
    let geo = body.fixed_to_geodetic(cg_fixed);
    let sample = atmosphere
        .sample(&geo, utc)
        .map_err(|source| BlockError::Environment {
            block: BLOCK_NAME,
            source,
        })?;
    let wind_fixed = LocalFrame::at(&geo).ned_to_ecef() * sample.wind_ned_m_s;
    let v_root_fixed = base_fixed.rotation_world_from_root * base_fixed.linear_vel_root_m_s;
    let air_vel_root_m_s =
        base_fixed.rotation_world_from_root.conjugate() * (v_root_fixed - wind_fixed);
    let ambient = sample.ambient_gas();
    let flow = AeroFlowState {
        velocity_air_body_m_s: air_vel_root_m_s,
        angular_rate_body_rad_s: base_fixed.angular_vel_root_rad_s,
        ambient,
        control_surfaces_rad,
    };
    let aero_root = aero.evaluate_root(&flow, poses).map_err(block_mbd)?;
    let aero_site_wrench = WrenchAtSite {
        site: ComponentId(0),
        point_local_m: DVec3::ZERO,
        force_local_n: aero_root.force_body_n,
        torque_local_nm: aero_root.moment_about_cg(DVec3::ZERO),
    };
    let v2 = air_vel_root_m_s.length_squared();
    Ok(EnvSample {
        ambient,
        air_vel_root_m_s,
        airspeed_m_s: v2.sqrt(),
        dynamic_pressure_pa: 0.5 * ambient.density_kg_m3 * v2,
        aero_root,
        aero_site_wrench,
    })
}

pub(super) struct StepContext<'a> {
    pub(super) body: BodyEllipsoid,
    pub(super) gravity: &'a dyn GravityField,
    pub(super) atmosphere: &'a dyn AtmosphereField,
    pub(super) surface: &'a dyn SurfaceContactProvider,
    pub(super) aero: &'a AeroStorage,
    pub(super) utc: UtcTime,
    pub(super) dt_s: f64,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn advance_step(
    ctx: &StepContext<'_>,
    tree: &mut ComponentTree,
    subsystems: &mut MultibodySubsystems,
    base_fixed: &mut FloatingBaseState,
    mass: &mut MassProperties,
    pinned_rest: &mut PinnedSurfaceRest,
    last_specific_force_cg: DVec3,
    last_alpha_root: &mut DVec3,
    input: &MultibodyVehicleInput,
) -> Result<MultibodyVehicleTelemetry, BlockError> {
    if let PinnedSurfaceRest::Pinned {
        world_pos_m,
        world_rot,
    } = *pinned_rest
    {
        if let Some(held) = try_hold_pinned_rest(
            ctx,
            tree,
            subsystems,
            *base_fixed,
            mass,
            world_pos_m,
            world_rot,
            last_specific_force_cg,
            last_alpha_root,
            input,
        )? {
            return Ok(held);
        }
        *pinned_rest = PinnedSurfaceRest::Free;
    }

    integrate_free_step(
        ctx,
        tree,
        subsystems,
        base_fixed,
        mass,
        pinned_rest,
        last_specific_force_cg,
        last_alpha_root,
        input,
    )
}

#[allow(clippy::too_many_arguments)]
fn try_hold_pinned_rest(
    ctx: &StepContext<'_>,
    tree: &mut ComponentTree,
    subsystems: &mut MultibodySubsystems,
    base_fixed: FloatingBaseState,
    mass: &mut MassProperties,
    world_pos_m: DVec3,
    world_rot: DQuat,
    last_sf: DVec3,
    last_alpha: &mut DVec3,
    input: &MultibodyVehicleInput,
) -> Result<Option<MultibodyVehicleTelemetry>, BlockError> {
    let mut temp_subs = subsystems.clone();
    let mut temp_tree = tree.clone();
    let poses = temp_tree.evaluate_pose_table().map_err(block_mbd)?;
    let env = sample_env_and_aero(
        &ctx.body,
        ctx.atmosphere,
        ctx.aero,
        &poses,
        &base_fixed,
        mass,
        ctx.utc,
        input.control_surfaces_rad,
    )?;
    let probe = run_subsystems_step(
        0.0,
        &mut temp_tree,
        &poses,
        &mut temp_subs,
        &env,
        base_fixed,
        mass,
        last_sf,
        *last_alpha,
        &ctx.body,
        input,
    )?;
    let cg_fixed = world_pos_m + world_rot * mass.cg_body_m;
    let g_eff = effective_gravity_fixed(ctx, cg_fixed);
    let up = surface_up_fixed(ctx.surface, cg_fixed);
    let f_ng =
        world_rot * (env.aero_root.force_body_n + probe.thrust_body_n + probe.ext_force_root_n);
    let thresh = subsystems
        .pinned_breakout_threshold_n
        .unwrap_or_else(|| 0.98 * mass.total_mass_kg * g_eff.length());
    if f_ng.dot(up) >= thresh {
        return Ok(None);
    }
    let sub = run_subsystems_step(
        ctx.dt_s,
        tree,
        &poses,
        subsystems,
        &env,
        base_fixed,
        mass,
        last_sf,
        *last_alpha,
        &ctx.body,
        input,
    )?;
    let dyn_masses = merge_dynamic_masses(&sub.fluid_out.mass_contributions, &sub.grain_contribs);
    *mass = tree
        .mass_properties_with_contributions(&dyn_masses)
        .map_err(block_mbd)?;
    let sf_cg = world_rot.conjugate() * (-g_eff);
    *last_alpha = DVec3::ZERO;
    Ok(Some(build_telemetry(
        subsystems, &env, &sub, sf_cg, true, true, false, 0.0, true,
    )))
}

#[allow(clippy::too_many_arguments)]
fn integrate_free_step(
    ctx: &StepContext<'_>,
    tree: &mut ComponentTree,
    subsystems: &mut MultibodySubsystems,
    base_fixed: &mut FloatingBaseState,
    mass: &mut MassProperties,
    pinned_rest: &mut PinnedSurfaceRest,
    last_sf: DVec3,
    last_alpha: &mut DVec3,
    input: &MultibodyVehicleInput,
) -> Result<MultibodyVehicleTelemetry, BlockError> {
    let poses = tree.evaluate_pose_table().map_err(block_mbd)?;
    let env = sample_env_and_aero(
        &ctx.body,
        ctx.atmosphere,
        ctx.aero,
        &poses,
        base_fixed,
        mass,
        ctx.utc,
        input.control_surfaces_rad,
    )?;
    let sub = run_subsystems_step(
        ctx.dt_s,
        tree,
        &poses,
        subsystems,
        &env,
        *base_fixed,
        mass,
        last_sf,
        *last_alpha,
        &ctx.body,
        input,
    )?;
    let free = collect_free_forces(ctx, tree, &poses, subsystems, &env, &sub, base_fixed, input)?;
    let used_fast = step_dynamics_rk4(
        ctx,
        tree,
        &free.dyn_masses,
        base_fixed,
        mass,
        input.control_surfaces_rad,
        &free.all_drives,
        &free.all_wrenches,
        last_alpha,
    )?;
    update_pinned_rest_step(
        ctx,
        subsystems,
        base_fixed,
        mass,
        free.net_ext_root,
        free.any_contact,
        pinned_rest,
    );

    Ok(build_telemetry(
        subsystems,
        &env,
        &sub,
        free.spec_force_cg,
        free.any_contact,
        matches!(pinned_rest, PinnedSurfaceRest::Pinned { .. }),
        free.buoyancy_n > 0.0,
        free.buoyancy_n,
        used_fast,
    ))
}

enum Rk4StageError {
    Multi(mbd_multibody::MultibodyError),
    Block(BlockError),
}

impl From<mbd_multibody::MultibodyError> for Rk4StageError {
    fn from(value: mbd_multibody::MultibodyError) -> Self {
        Self::Multi(value)
    }
}

impl Rk4StageError {
    fn into_block_error(self) -> BlockError {
        match self {
            Self::Multi(err) => block_multi(err),
            Self::Block(err) => err,
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn eval_rk4_stage_forces(
    ctx: &StepContext<'_>,
    stage_tree: &ComponentTree,
    dyn_masses: &[DynamicMassContribution],
    base_i_stage: &FloatingBaseState,
    stage_dt_s: f64,
    control_surfaces_rad: [f64; 8],
    drives: &[JointDriveInput],
    non_aero_wrenches: &[WrenchAtSite],
) -> Result<(DVec3, Vec<JointDriveInput>, Vec<WrenchAtSite>), Rk4StageError> {
    let base_fixed_stage = inertial_to_fixed_after_step(base_i_stage, &ctx.body, stage_dt_s);
    let stage_poses = stage_tree
        .evaluate_pose_table()
        .map_err(|e| Rk4StageError::Block(block_mbd(e)))?;
    let stage_mass = stage_tree
        .mass_properties_with_contributions(dyn_masses)
        .map_err(|e| Rk4StageError::Block(block_mbd(e)))?;
    let cg_fixed_stage = base_fixed_stage.position_world_m
        + base_fixed_stage.rotation_world_from_root * stage_mass.cg_body_m;
    let g_fixed_stage = ctx.gravity.attraction_fixed(cg_fixed_stage);
    let g_i_stage =
        DQuat::from_axis_angle(DVec3::Z, ctx.body.omega_rad_s * stage_dt_s) * g_fixed_stage;
    let stage_env = sample_env_and_aero(
        &ctx.body,
        ctx.atmosphere,
        ctx.aero,
        &stage_poses,
        &base_fixed_stage,
        &stage_mass,
        ctx.utc.advanced_by(stage_dt_s),
        control_surfaces_rad,
    )
    .map_err(Rk4StageError::Block)?;
    let mut stage_wrenches = Vec::with_capacity(1 + non_aero_wrenches.len());
    stage_wrenches.push(stage_env.aero_site_wrench);
    stage_wrenches.extend_from_slice(non_aero_wrenches);
    Ok((g_i_stage, drives.to_vec(), stage_wrenches))
}

#[allow(clippy::too_many_arguments)]
fn step_dynamics_rk4(
    ctx: &StepContext<'_>,
    tree: &mut ComponentTree,
    dyn_masses: &[DynamicMassContribution],
    base_fixed: &mut FloatingBaseState,
    mass: &mut MassProperties,
    control_surfaces_rad: [f64; 8],
    drives: &[JointDriveInput],
    wrenches: &[WrenchAtSite],
    last_alpha: &mut DVec3,
) -> Result<bool, BlockError> {
    let base_i = fixed_to_instantaneous_inertial(base_fixed, &ctx.body);
    let cg_fixed =
        base_fixed.position_world_m + base_fixed.rotation_world_from_root * mass.cg_body_m;
    let gravity_fixed = ctx.gravity.attraction_fixed(cg_fixed);
    let accel_eval = MultiBodyDynamics::forward_dynamics(
        tree,
        dyn_masses,
        &base_i,
        gravity_fixed,
        drives,
        wrenches,
    )
    .map_err(block_multi)?;
    *last_alpha = accel_eval.base_spatial_accel_root.ang;

    let non_aero_wrenches = wrenches.get(1..).unwrap_or(&[]);
    let base_i_next = step_rk4_with_forces::<Rk4StageError, _>(
        tree,
        dyn_masses,
        &base_i,
        ctx.dt_s,
        |stage_tree, base_i_stage, stage_dt_s| {
            eval_rk4_stage_forces(
                ctx,
                stage_tree,
                dyn_masses,
                base_i_stage,
                stage_dt_s,
                control_surfaces_rad,
                drives,
                non_aero_wrenches,
            )
        },
    )
    .map_err(Rk4StageError::into_block_error)?;
    *base_fixed = inertial_to_fixed_after_step(&base_i_next, &ctx.body, ctx.dt_s);
    *mass = tree
        .mass_properties_with_contributions(dyn_masses)
        .map_err(block_mbd)?;
    Ok(accel_eval.used_rigid_fast_path)
}
