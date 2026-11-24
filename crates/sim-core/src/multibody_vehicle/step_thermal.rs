//! Centre-of-gravity trim, the thermal network and surface erosion: the
//! subsystem work of a step that follows the fluid and electrical buses.

use mbd_core::{ComponentTree, MassProperties, ThermalLoad};
use mbd_systems::{
    CgTrimController, ElectricalStepOutput, ThermalStepEnv, apply_tps_ablation_to_tree,
    step_atomic_oxygen_erosion, step_tps_ablation,
};

use super::step::{EnvSample, block_sys};
use super::subsystems::{MultibodySubsystems, MultibodyVehicleInput};
use super::telemetry::find_rail;
use crate::BlockError;

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
