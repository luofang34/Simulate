use glam::DVec3;
use mbd_core::{
    ComponentId, ComponentTree, DynamicMassContribution, MassProperties, PoseTable, PowerRailState,
    ThermalLoad, WrenchAtSite,
};
use mbd_multibody::{
    FloatingBaseState, JointDriveInput, PinnedSurfaceRest, evaluate_contact_point,
    evaluate_loop_constraints, evaluate_wheel, step_pinned_surface_rest,
};
use mbd_systems::{
    CgTrimController, ElectricalStepOutput, ThermalStepEnv, apply_tps_ablation_to_tree,
    step_atomic_oxygen_erosion, step_tps_ablation,
};

use super::step::{EnvSample, StepContext, block_mbd, block_multi, block_sys};
use super::step_subsystems::SubsystemStepOut;
use super::subsystems::{
    MultibodySubsystems, MultibodyVehicleInput, MultibodyVehicleTelemetry, SurfaceContactProvider,
    wheel_inertia_estimate,
};
use crate::BlockError;

fn link_point_kinematics(
    base_fixed: &FloatingBaseState,
    pose: &mbd_core::LinkPose,
    offset_local_m: DVec3,
) -> (DVec3, glam::DQuat, DVec3) {
    let q_root = base_fixed.rotation_world_from_root;
    let rot_link = (q_root * pose.rotation_root_from_link).normalize();
    let r_link_world = rot_link * offset_local_m;
    let pt_fixed = base_fixed.position_world_m + q_root * pose.position_root_m + r_link_world;
    let vel_rel_root = base_fixed.linear_vel_root_m_s
        + base_fixed
            .angular_vel_root_rad_s
            .cross(pose.position_root_m)
        + pose.linear_vel_root_m_s;
    let w_link_root = base_fixed.angular_vel_root_rad_s + pose.angular_vel_root_rad_s;
    let vel_fixed = q_root * vel_rel_root + (q_root * w_link_root).cross(r_link_world);
    (pt_fixed, rot_link, vel_fixed)
}

pub(super) fn evaluate_contacts_and_wheels(
    surface: &dyn SurfaceContactProvider,
    poses: &PoseTable,
    subsystems: &mut MultibodySubsystems,
    base_fixed: &FloatingBaseState,
    input: &MultibodyVehicleInput,
    dt_s: f64,
) -> Result<(Vec<WrenchAtSite>, bool), BlockError> {
    let mut out = Vec::new();
    let mut any_contact = false;

    for (spec, state) in &mut subsystems.contact_points {
        let pose = poses.get(spec.component).map_err(block_mbd)?;
        if !pose.attached {
            continue;
        }
        let (pt_fixed, rot_link, vel_fixed) =
            link_point_kinematics(base_fixed, pose, spec.offset_local_m);
        if let Some(sample) = surface.sample_surface(pt_fixed) {
            let res =
                evaluate_contact_point(spec, state, pt_fixed, vel_fixed, &sample, rot_link, dt_s)
                    .map_err(block_multi)?;
            if res.normal_force_n > 0.0 {
                any_contact = true;
                out.push(res.wrench);
            }
        }
    }

    for (idx, (spec, state)) in subsystems.wheels.iter_mut().enumerate() {
        let pose = poses.get(spec.component).map_err(block_mbd)?;
        if !pose.attached {
            continue;
        }
        let (pt_fixed, rot_link, vel_fixed) =
            link_point_kinematics(base_fixed, pose, spec.hub_offset_local_m);
        if let Some(sample) = surface.sample_surface(pt_fixed) {
            let cmd = input.wheel_commands.get(idx).copied().unwrap_or_default();
            let res = evaluate_wheel(
                spec,
                state,
                &cmd,
                pt_fixed,
                vel_fixed,
                rot_link,
                &sample,
                wheel_inertia_estimate(spec),
                dt_s,
            )
            .map_err(block_multi)?;
            if res.contact_load_n > 0.0 {
                any_contact = true;
            }
            if res.wrench.force_local_n.length_squared() > 0.0 {
                out.push(res.wrench);
            }
        }
    }
    Ok((out, any_contact))
}

pub(super) fn evaluate_hydrodynamics(
    surface: &dyn SurfaceContactProvider,
    hydrodynamics: &Option<mbd_multibody::HydrodynamicsModel>,
    base_fixed: &FloatingBaseState,
) -> Result<(Vec<WrenchAtSite>, f64), BlockError> {
    let Some(hydro) = hydrodynamics else {
        return Ok((Vec::new(), 0.0));
    };
    let Some(water) = surface.sample_water(base_fixed.position_world_m) else {
        return Ok((Vec::new(), 0.0));
    };
    let out = hydro.evaluate(base_fixed, &water).map_err(block_multi)?;
    Ok((out.wrenches, out.total_buoyancy_n))
}

pub(super) struct FreeForceEval {
    pub(super) all_wrenches: Vec<WrenchAtSite>,
    pub(super) all_drives: Vec<JointDriveInput>,
    pub(super) dyn_masses: Vec<DynamicMassContribution>,
    pub(super) net_ext_root: DVec3,
    pub(super) spec_force_cg: DVec3,
    pub(super) any_contact: bool,
    pub(super) buoyancy_n: f64,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn collect_free_forces(
    ctx: &StepContext<'_>,
    tree: &ComponentTree,
    poses: &PoseTable,
    subsystems: &mut MultibodySubsystems,
    env: &EnvSample,
    sub: &SubsystemStepOut,
    base_fixed: &FloatingBaseState,
    input: &MultibodyVehicleInput,
) -> Result<FreeForceEval, BlockError> {
    let (cw_wrenches, any_contact) =
        evaluate_contacts_and_wheels(ctx.surface, poses, subsystems, base_fixed, input, ctx.dt_s)?;
    let (hydro_wrenches, buoyancy_n) =
        evaluate_hydrodynamics(ctx.surface, &subsystems.hydrodynamics, base_fixed)?;
    let loop_out = evaluate_loop_constraints(tree, poses, &subsystems.loop_constraints)
        .map_err(block_multi)?;
    let (all_wrenches, n_ext) = assemble_wrenches(
        env,
        sub,
        input,
        &cw_wrenches,
        &hydro_wrenches,
        &loop_out.wrenches,
    );
    let net_ext_root = sum_wrenches_in_root(poses, all_wrenches.get(..n_ext).unwrap_or(&[]));
    let dyn_masses = merge_dynamic_masses(&sub.fluid_out.mass_contributions, &sub.grain_contribs);
    let step_mass_kg = tree
        .mass_properties_with_contributions(&dyn_masses)
        .map_err(block_mbd)?
        .total_mass_kg;
    let mut all_drives = Vec::with_capacity(input.joint_drives.len() + loop_out.joint_drives.len());
    all_drives.extend_from_slice(&input.joint_drives);
    all_drives.extend_from_slice(&loop_out.joint_drives);
    Ok(FreeForceEval {
        all_wrenches,
        all_drives,
        dyn_masses,
        net_ext_root,
        spec_force_cg: net_ext_root / step_mass_kg,
        any_contact,
        buoyancy_n,
    })
}

pub(super) fn apply_cg_trim(
    subsystems: &mut MultibodySubsystems,
    mass: &MassProperties,
    input: &MultibodyVehicleInput,
) -> Result<(), BlockError> {
    if let Some(trim) = subsystems.cg_trim {
        let ctrl = CgTrimController {
            target_cg_coord_m: input
                .cg_trim_target_x_m
                .unwrap_or(trim.controller.target_cg_coord_m),
            ..trim.controller
        };
        let cmd = ctrl.compute_command(mass.cg_body_m.x).map_err(block_sys)?;
        subsystems
            .fluids
            .set_line_command(trim.line_id, cmd)
            .map_err(block_sys)?;
    }
    Ok(())
}

pub(super) fn step_thermal_bus(
    dt_s: f64,
    tree: &mut ComponentTree,
    subsystems: &mut MultibodySubsystems,
    env: &EnvSample,
    elec_out: &ElectricalStepOutput,
    thermal_loads: &[ThermalLoad],
    input: &MultibodyVehicleInput,
) -> Result<(), BlockError> {
    let default_v = subsystems.electrical.nominal_bus_voltage_v();
    let therm_env = ThermalStepEnv {
        dt_s,
        ambient: env.ambient,
        airspeed_m_s: env.airspeed_m_s,
        radiative_sink_temp_k: input.sink_temperature_k.max(2.7),
        solar_irradiance_w_m2: input.sun_irradiance_w_m2,
        sunlit: input.sun_visibility > 0.0,
    };
    subsystems
        .thermal
        .step(tree, &therm_env, thermal_loads, |comp| {
            find_rail(elec_out, comp, default_v)
        })
        .map_err(block_sys)?;
    subsystems.thermal.sync_to_tree(tree).map_err(block_sys)
}

pub(super) fn step_tps_and_ao(
    dt_s: f64,
    tree: &mut ComponentTree,
    subsystems: &mut MultibodySubsystems,
    env: &EnvSample,
    thermal_loads: &mut Vec<ThermalLoad>,
) -> Result<(bool, f64, f64, f64), BlockError> {
    let mut blackout = false;
    let mut peak_flux = 0.0_f64;
    for (spec, state) in &mut subsystems.tps_shields {
        let out = step_tps_ablation(spec, state, dt_s, &env.ambient, env.airspeed_m_s)
            .map_err(block_sys)?;
        apply_tps_ablation_to_tree(spec, state, tree).map_err(block_sys)?;
        blackout |= out.comm_blackout;
        peak_flux = peak_flux.max(state.peak_heat_flux_w_m2);
        if out.backface_thermal_load.heat_rate_w > 0.0 {
            thermal_loads.push(out.backface_thermal_load);
        }
    }
    let ram_cos = if env.airspeed_m_s > 1e-6 {
        (env.air_vel_root_m_s.x / env.airspeed_m_s).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let mut max_loss_m = 0.0_f64;
    let mut min_rem_frac = 1.0_f64;
    for (spec, state) in &mut subsystems.ao_coatings {
        let out =
            step_atomic_oxygen_erosion(spec, state, dt_s, &env.ambient, env.airspeed_m_s, ram_cos)
                .map_err(block_sys)?;
        max_loss_m = max_loss_m.max(state.thickness_lost_m);
        min_rem_frac = min_rem_frac.min(out.remaining_fraction);
    }
    Ok((blackout, peak_flux, max_loss_m, min_rem_frac))
}

pub(super) fn assemble_wrenches(
    env: &EnvSample,
    sub: &SubsystemStepOut,
    input: &MultibodyVehicleInput,
    cw_wrenches: &[WrenchAtSite],
    hydro_wrenches: &[WrenchAtSite],
    loop_wrenches: &[WrenchAtSite],
) -> (Vec<WrenchAtSite>, usize) {
    let mut out = Vec::with_capacity(
        1 + sub.prop_wrenches.len()
            + sub.fluid_out.slosh_wrenches.len()
            + input.external_wrenches.len()
            + cw_wrenches.len()
            + hydro_wrenches.len()
            + loop_wrenches.len(),
    );
    out.push(env.aero_site_wrench);
    out.extend_from_slice(&sub.prop_wrenches);
    out.extend_from_slice(&sub.fluid_out.slosh_wrenches);
    out.extend_from_slice(&input.external_wrenches);
    out.extend_from_slice(cw_wrenches);
    out.extend_from_slice(hydro_wrenches);
    let n_external = out.len();
    out.extend_from_slice(loop_wrenches);
    (out, n_external)
}

pub(super) fn effective_gravity_fixed(ctx: &StepContext<'_>, cg_fixed: DVec3) -> DVec3 {
    let omega_p = DVec3::Z * ctx.body.omega_rad_s;
    ctx.gravity.attraction_fixed(cg_fixed) - omega_p.cross(omega_p.cross(cg_fixed))
}

pub(super) fn surface_up_fixed(surface: &dyn SurfaceContactProvider, point_fixed: DVec3) -> DVec3 {
    surface.sample_surface(point_fixed).map_or_else(
        || point_fixed.normalize_or(DVec3::Z),
        |s| s.outward_normal_world,
    )
}

pub(super) fn update_pinned_rest_step(
    ctx: &StepContext<'_>,
    subsystems: &MultibodySubsystems,
    base_fixed: &FloatingBaseState,
    mass: &MassProperties,
    net_ext_root: DVec3,
    any_contact: bool,
    pinned_rest: &mut PinnedSurfaceRest,
) {
    let cg_next =
        base_fixed.position_world_m + base_fixed.rotation_world_from_root * mass.cg_body_m;
    let g_eff_next = effective_gravity_fixed(ctx, cg_next);
    let up_next = surface_up_fixed(ctx.surface, cg_next);
    let net_upward_n = (base_fixed.rotation_world_from_root * net_ext_root).dot(up_next);
    let breakout = subsystems
        .pinned_breakout_threshold_n
        .unwrap_or_else(|| 0.98 * mass.total_mass_kg * g_eff_next.length());
    step_pinned_surface_rest(
        pinned_rest,
        any_contact,
        base_fixed.linear_vel_root_m_s.length(),
        base_fixed.angular_vel_root_rad_s.length(),
        base_fixed.position_world_m,
        base_fixed.rotation_world_from_root,
        net_upward_n,
        breakout,
        subsystems.pinned_dwell_s,
        ctx.dt_s,
    );
}

pub(super) fn merge_dynamic_masses(
    fluid_contribs: &[DynamicMassContribution],
    grain_contribs: &[DynamicMassContribution],
) -> Vec<DynamicMassContribution> {
    let mut out = Vec::with_capacity(fluid_contribs.len() + grain_contribs.len());
    out.extend_from_slice(fluid_contribs);
    out.extend_from_slice(grain_contribs);
    out
}

pub(super) fn sum_wrenches_in_root(poses: &PoseTable, wrenches: &[WrenchAtSite]) -> DVec3 {
    wrenches.iter().fold(DVec3::ZERO, |acc, w| {
        let rot = poses
            .get(w.site)
            .map_or(glam::DQuat::IDENTITY, |p| p.rotation_root_from_link);
        acc + rot * w.force_local_n
    })
}

pub(super) fn find_rail(
    out: &ElectricalStepOutput,
    site: ComponentId,
    default_v: f64,
) -> PowerRailState {
    out.component_rails
        .iter()
        .find(|r| r.component == site)
        .map_or_else(|| PowerRailState::nominal(default_v), |r| r.rail)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn build_telemetry(
    subsystems: &MultibodySubsystems,
    env: &EnvSample,
    sub: &SubsystemStepOut,
    spec_force_cg_body: DVec3,
    in_ground_contact: bool,
    is_pinned_rest: bool,
    is_splashed_down: bool,
    total_buoyancy_n: f64,
    used_rigid_fast_path: bool,
) -> MultibodyVehicleTelemetry {
    let total_prop =
        subsystems.fluids.total_fluid_mass_kg() + subsystems.propulsion.total_solid_propellant_kg();
    let root_rail = find_rail(
        &sub.elec_out,
        ComponentId(0),
        subsystems.electrical.nominal_bus_voltage_v(),
    );
    MultibodyVehicleTelemetry {
        aero_wrench: env.aero_root,
        dynamic_pressure_pa: env.dynamic_pressure_pa,
        specific_force_cg_body: spec_force_cg_body,
        total_thrust_body_n: sub.thrust_body_n,
        total_propellant_kg: total_prop,
        bus_voltage_v: if root_rail.enabled {
            root_rail.voltage_v
        } else {
            0.0
        },
        total_electrical_gen_w: sub.elec_out.total_generated_w,
        electrical_shed_w: sub.elec_out.total_shed_w,
        rf_blackout: sub.rf_blackout,
        peak_heat_flux_w_m2: sub.peak_heat_flux_w_m2,
        ao_erosion_depth_m: sub.ao_erosion_depth_m,
        ao_remaining_fraction: sub.ao_remaining_fraction,
        in_ground_contact,
        is_pinned_rest,
        is_splashed_down,
        total_buoyancy_n,
        used_rigid_fast_path,
    }
}
