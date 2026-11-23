use glam::DVec3;
use mbd_core::{
    ComponentId, ComponentTree, DynamicMassContribution, ElectricalLoad, MassProperties, PoseTable,
    PowerRailState, ThermalLoad, WrenchAtSite, compute_site_kinematics,
};
use mbd_multibody::FloatingBaseState;
use mbd_systems::{
    AbepStepInput, AirbreathingStepInput, ElectricalEnvInput, ElectricalStepOutput,
    FluidStepOutput, LiquidRocketStepInput, step_abep_thruster, step_airbreathing_engine,
    step_electric_thruster, step_liquid_rocket, step_solid_motor,
};
use sim_geo::BodyEllipsoid;

use super::step::{EnvSample, block_sys};
use super::step_contacts::{
    apply_cg_trim, find_rail, step_thermal_bus, step_tps_and_ao, sum_wrenches_in_root,
};
use super::subsystems::{MultibodySubsystems, MultibodyVehicleInput};
use super::topology::apply_fault_override;
use crate::BlockError;

pub(super) struct SubsystemStepOut {
    pub(super) fluid_out: FluidStepOutput,
    pub(super) elec_out: ElectricalStepOutput,
    pub(super) prop_wrenches: Vec<WrenchAtSite>,
    pub(super) grain_contribs: Vec<DynamicMassContribution>,
    pub(super) thrust_body_n: DVec3,
    pub(super) thrust_torque_cg_body_nm: DVec3,
    pub(super) ext_force_root_n: DVec3,
    pub(super) rf_blackout: bool,
    pub(super) peak_heat_flux_w_m2: f64,
    pub(super) ao_erosion_depth_m: f64,
    pub(super) ao_remaining_fraction: f64,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn run_subsystems_step(
    dt_s: f64,
    tree: &mut ComponentTree,
    poses: &PoseTable,
    subsystems: &mut MultibodySubsystems,
    env: &EnvSample,
    base_fixed: FloatingBaseState,
    mass: &MassProperties,
    spec_force_cg_body: DVec3,
    alpha_root_rad_s2: DVec3,
    body: &BodyEllipsoid,
    input: &MultibodyVehicleInput,
) -> Result<SubsystemStepOut, BlockError> {
    apply_cg_trim(subsystems, mass, input)?;
    let (fluid_out, elec_out, ep_range, abep_range) = step_fluid_and_electrical(
        dt_s,
        tree,
        poses,
        subsystems,
        base_fixed,
        mass,
        spec_force_cg_body,
        alpha_root_rad_s2,
        body,
        input,
    )?;

    let mut thermal_loads =
        Vec::with_capacity(elec_out.thermal_loads.len() + input.extra_thermal_loads.len() + 8);
    thermal_loads.extend_from_slice(&elec_out.thermal_loads);
    thermal_loads.extend_from_slice(&input.extra_thermal_loads);

    let prop_ctx = PropulsionCtx {
        dt_s,
        poses,
        env,
        elec_out: &elec_out,
        ep_range,
        abep_range,
        default_v: subsystems.electrical.nominal_bus_voltage_v(),
        cg_root_m: mass.cg_body_m,
        input,
    };
    let mut prop_acc = PropulsionAcc::new(&mut thermal_loads);
    step_propulsion(&prop_ctx, subsystems, &mut prop_acc)?;
    let PropulsionAcc {
        wrenches: prop_wrenches,
        grains: grain_contribs,
        thrust_root_n: thrust_body_n,
        torque_cg_root_nm: thrust_torque_cg_body_nm,
        ..
    } = prop_acc;
    let (rf_blackout, peak_heat_flux_w_m2, ao_erosion_depth_m, ao_remaining_fraction) =
        step_tps_and_ao(dt_s, tree, subsystems, env, &mut thermal_loads)?;
    step_thermal_bus(
        dt_s,
        tree,
        subsystems,
        env,
        &elec_out,
        &thermal_loads,
        input,
    )?;

    Ok(SubsystemStepOut {
        fluid_out,
        elec_out,
        prop_wrenches,
        grain_contribs,
        thrust_body_n,
        thrust_torque_cg_body_nm,
        ext_force_root_n: sum_wrenches_in_root(poses, &input.external_wrenches),
        rf_blackout,
        peak_heat_flux_w_m2,
        ao_erosion_depth_m,
        ao_remaining_fraction,
    })
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn step_fluid_and_electrical(
    dt_s: f64,
    tree: &ComponentTree,
    poses: &PoseTable,
    subsystems: &mut MultibodySubsystems,
    base_fixed: FloatingBaseState,
    mass: &MassProperties,
    spec_force_cg_body: DVec3,
    alpha_root_rad_s2: DVec3,
    body: &BodyEllipsoid,
    input: &MultibodyVehicleInput,
) -> Result<
    (
        FluidStepOutput,
        ElectricalStepOutput,
        core::ops::Range<usize>,
        core::ops::Range<usize>,
    ),
    BlockError,
> {
    let omega_inertial_root = base_fixed.angular_vel_root_rad_s
        + base_fixed.rotation_world_from_root.conjugate() * (DVec3::Z * body.omega_rad_s);
    let site_sf = |comp: ComponentId| -> DVec3 {
        let Ok(p) = poses.get(comp) else {
            return spec_force_cg_body;
        };
        compute_site_kinematics(
            spec_force_cg_body,
            omega_inertial_root,
            alpha_root_rad_s2,
            (p.position_root_m, p.rotation_root_from_link),
            mass.cg_body_m,
        )
        .specific_force_site_m_s2
    };
    let fluid_out = subsystems
        .fluids
        .step(
            dt_s,
            tree,
            site_sf,
            |_| None,
            |_| PowerRailState::nominal(28.0),
        )
        .map_err(block_sys)?;

    let (loads, ep_range, abep_range) = build_electrical_loads(subsystems, &fluid_out, input);
    let sun_root = base_fixed.rotation_world_from_root.conjugate() * input.sun_dir_world;
    let elec_env = ElectricalEnvInput {
        dt_s,
        default_sun_dir_local: sun_root,
        solar_irradiance_w_m2: input.sun_irradiance_w_m2,
        sunlit: input.sun_visibility > 0.0,
        default_shaft_power_w: 0.0,
    };
    let elec_out = subsystems
        .electrical
        .step(
            tree,
            &loads,
            &elec_env,
            Some(&mut subsystems.fluids),
            |comp| {
                poses.get(comp).map_or(sun_root, |p| {
                    p.rotation_root_from_link.conjugate() * sun_root
                })
            },
        )
        .map_err(block_sys)?;
    Ok((fluid_out, elec_out, ep_range, abep_range))
}

fn build_electrical_loads(
    subsystems: &MultibodySubsystems,
    fluid_out: &FluidStepOutput,
    input: &MultibodyVehicleInput,
) -> (
    Vec<ElectricalLoad>,
    core::ops::Range<usize>,
    core::ops::Range<usize>,
) {
    let mut loads = Vec::new();
    loads.extend_from_slice(&fluid_out.electrical_loads);
    loads.extend_from_slice(&input.extra_electrical_loads);
    let ep_start = loads.len();
    for (idx, (spec, _)) in subsystems.propulsion.electric_thrusters.iter().enumerate() {
        let th = input
            .propulsion
            .electric_throttles
            .get(idx)
            .copied()
            .unwrap_or(0.0)
            .clamp(0.0, 1.0);
        loads.push(ElectricalLoad {
            component: spec.site,
            power_w: spec.max_power_w * th,
            priority: spec.priority,
        });
    }
    let abep_start = loads.len();
    for (idx, (spec, _)) in subsystems.propulsion.abep_thrusters.iter().enumerate() {
        let th = input
            .propulsion
            .abep_throttles
            .get(idx)
            .copied()
            .unwrap_or(0.0)
            .clamp(0.0, 1.0);
        loads.push(ElectricalLoad {
            component: spec.site,
            power_w: spec.max_power_w * th,
            priority: spec.priority,
        });
    }
    let abep_end = loads.len();
    (loads, ep_start..abep_start, abep_start..abep_end)
}

struct PropulsionCtx<'a> {
    dt_s: f64,
    poses: &'a PoseTable,
    env: &'a EnvSample,
    elec_out: &'a ElectricalStepOutput,
    ep_range: core::ops::Range<usize>,
    abep_range: core::ops::Range<usize>,
    default_v: f64,
    cg_root_m: DVec3,
    input: &'a MultibodyVehicleInput,
}

struct PropulsionAcc<'a> {
    wrenches: Vec<WrenchAtSite>,
    grains: Vec<DynamicMassContribution>,
    thrust_root_n: DVec3,
    torque_cg_root_nm: DVec3,
    thermal_loads: &'a mut Vec<ThermalLoad>,
}

impl<'a> PropulsionAcc<'a> {
    fn new(thermal_loads: &'a mut Vec<ThermalLoad>) -> Self {
        Self {
            wrenches: Vec::new(),
            grains: Vec::new(),
            thrust_root_n: DVec3::ZERO,
            torque_cg_root_nm: DVec3::ZERO,
            thermal_loads,
        }
    }
}

fn step_propulsion(
    ctx: &PropulsionCtx<'_>,
    subsystems: &mut MultibodySubsystems,
    acc: &mut PropulsionAcc<'_>,
) -> Result<(), BlockError> {
    step_chemical_and_solid(ctx, subsystems, acc)?;
    step_electric_propulsion(ctx, subsystems, acc)?;
    step_abep_propulsion(ctx, subsystems, acc)?;
    step_airbreathing_propulsion(ctx, subsystems, acc)
}

fn step_chemical_and_solid(
    ctx: &PropulsionCtx<'_>,
    subsystems: &mut MultibodySubsystems,
    acc: &mut PropulsionAcc<'_>,
) -> Result<(), BlockError> {
    let p = &ctx.input.propulsion;
    for (idx, (spec, state)) in subsystems.propulsion.liquid_rockets.iter_mut().enumerate() {
        if let Some(&th) = p.liquid_throttles.get(idx) {
            state.throttle_cmd = th.clamp(0.0, 1.0);
        }
        if let Some(&gim) = p.liquid_gimbals_rad.get(idx) {
            state.gimbal_cmd_rad = gim;
        }
        if let Some(&ov) = p.liquid_faults.get(idx) {
            apply_fault_override(&mut state.fault, ov);
        }
        let isolated = p.liquid_isolated.get(idx).copied().unwrap_or(false);
        let out = step_liquid_rocket(
            spec,
            state,
            &mut subsystems.fluids,
            LiquidRocketStepInput {
                dt_s: ctx.dt_s,
                ambient: ctx.env.ambient,
                manifold_isolated: isolated,
            },
        )
        .map_err(block_sys)?;
        record_wrench(ctx, out.wrench_local, acc);
        if out.thermal_load.heat_rate_w > 0.0 {
            acc.thermal_loads.push(out.thermal_load);
        }
    }
    for (idx, (spec, state)) in subsystems.propulsion.solid_motors.iter_mut().enumerate() {
        let ignite = p.solid_ignites.get(idx).copied().unwrap_or(false);
        let out =
            step_solid_motor(spec, state, ctx.dt_s, &ctx.env.ambient, ignite).map_err(block_sys)?;
        acc.grains.push(out.grain_mass_contribution);
        record_wrench(ctx, out.wrench_local, acc);
    }
    Ok(())
}

fn step_electric_propulsion(
    ctx: &PropulsionCtx<'_>,
    subsystems: &mut MultibodySubsystems,
    acc: &mut PropulsionAcc<'_>,
) -> Result<(), BlockError> {
    let p = &ctx.input.propulsion;
    for (idx, (spec, state)) in subsystems
        .propulsion
        .electric_thrusters
        .iter_mut()
        .enumerate()
    {
        if let Some(&th) = p.electric_throttles.get(idx) {
            state.power_throttle_cmd = th.clamp(0.0, 1.0);
        }
        let alloc_w = ctx
            .elec_out
            .granted_powers_w
            .get(ctx.ep_range.start.saturating_add(idx))
            .copied()
            .unwrap_or(0.0);
        let out = step_electric_thruster(
            spec,
            state,
            &mut subsystems.fluids,
            mbd_systems::ElectricThrusterStepInput {
                dt_s: ctx.dt_s,
                power_rail: find_rail(ctx.elec_out, spec.site, ctx.default_v),
                allocated_power_w: alloc_w,
            },
        )
        .map_err(block_sys)?;
        record_wrench(ctx, out.wrench_local, acc);
        if out.thermal_load.heat_rate_w > 0.0 {
            acc.thermal_loads.push(out.thermal_load);
        }
    }
    Ok(())
}

fn step_abep_propulsion(
    ctx: &PropulsionCtx<'_>,
    subsystems: &mut MultibodySubsystems,
    acc: &mut PropulsionAcc<'_>,
) -> Result<(), BlockError> {
    let p = &ctx.input.propulsion;
    let cos_ram = p.abep_ram_alignment.unwrap_or_else(|| {
        if ctx.env.airspeed_m_s > 1e-6 {
            (ctx.env.air_vel_root_m_s.x / ctx.env.airspeed_m_s).clamp(0.0, 1.0)
        } else {
            0.0
        }
    });
    for (idx, (spec, state)) in subsystems.propulsion.abep_thrusters.iter_mut().enumerate() {
        if let Some(&th) = p.abep_throttles.get(idx) {
            state.power_throttle_cmd = th.clamp(0.0, 1.0);
        }
        let alloc_w = ctx
            .elec_out
            .granted_powers_w
            .get(ctx.abep_range.start.saturating_add(idx))
            .copied()
            .unwrap_or(0.0);
        let out = step_abep_thruster(
            spec,
            state,
            &mut subsystems.fluids,
            AbepStepInput {
                dt_s: ctx.dt_s,
                ambient: ctx.env.ambient,
                airspeed_m_s: ctx.env.airspeed_m_s,
                intake_ram_alignment: cos_ram,
                power_rail: find_rail(ctx.elec_out, spec.site, ctx.default_v),
                allocated_power_w: alloc_w,
            },
        )
        .map_err(block_sys)?;
        record_wrench(ctx, out.net_wrench_local, acc);
    }
    Ok(())
}

fn step_airbreathing_propulsion(
    ctx: &PropulsionCtx<'_>,
    subsystems: &mut MultibodySubsystems,
    acc: &mut PropulsionAcc<'_>,
) -> Result<(), BlockError> {
    let p = &ctx.input.propulsion;
    for (idx, (spec, state)) in subsystems
        .propulsion
        .airbreathing_engines
        .iter_mut()
        .enumerate()
    {
        if let Some(&th) = p.airbreathing_throttles.get(idx) {
            state.throttle_cmd = th.clamp(0.0, 1.0);
        }
        if let Some(&ab) = p.airbreathing_afterburners.get(idx) {
            state.afterburner_cmd = ab.clamp(0.0, 1.0);
        }
        let cutoff = p
            .airbreathing_fuel_cutoffs
            .get(idx)
            .copied()
            .unwrap_or(false);
        let out = step_airbreathing_engine(
            spec,
            state,
            &mut subsystems.fluids,
            AirbreathingStepInput {
                dt_s: ctx.dt_s,
                ambient: ctx.env.ambient,
                true_airspeed_m_s: ctx.env.airspeed_m_s,
                fuel_cutoff: cutoff,
            },
        )
        .map_err(block_sys)?;
        record_wrench(ctx, out.wrench_local, acc);
        if out.thermal_load.heat_rate_w > 0.0 {
            acc.thermal_loads.push(out.thermal_load);
        }
    }
    Ok(())
}

fn record_wrench(ctx: &PropulsionCtx<'_>, w: WrenchAtSite, acc: &mut PropulsionAcc<'_>) {
    let (rot, origin) = ctx
        .poses
        .get(w.site)
        .map_or((glam::DQuat::IDENTITY, DVec3::ZERO), |p| {
            (p.rotation_root_from_link, p.position_root_m)
        });
    let force_root = rot * w.force_local_n;
    let arm = origin + rot * w.point_local_m - ctx.cg_root_m;
    acc.thrust_root_n += force_root;
    acc.torque_cg_root_nm += rot * w.torque_local_nm + arm.cross(force_root);
    acc.wrenches.push(w);
}
