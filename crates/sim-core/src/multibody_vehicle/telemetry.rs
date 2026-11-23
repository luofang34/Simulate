//! What a step publishes about the vehicle, and the sums it is made from.

use glam::DVec3;
use mbd_core::{ComponentId, DynamicMassContribution, PowerRailState};
use mbd_systems::ElectricalStepOutput;

use super::step::EnvSample;
use super::step_subsystems::SubsystemStepOut;
use super::subsystems::{MultibodySubsystems, MultibodyVehicleTelemetry};
use crate::flight_model::loads::LoadReadout;

pub(super) fn merge_dynamic_masses(
    fluid_contribs: &[DynamicMassContribution],
    grain_contribs: &[DynamicMassContribution],
) -> Vec<DynamicMassContribution> {
    let mut out = Vec::with_capacity(fluid_contribs.len() + grain_contribs.len());
    out.extend_from_slice(fluid_contribs);
    out.extend_from_slice(grain_contribs);
    out
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

/// The telemetry of a step from its subsystem outputs and the loads read
/// at its end.
pub(super) fn build_telemetry(
    subsystems: &MultibodySubsystems,
    env: &EnvSample,
    sub: &SubsystemStepOut,
    loads: &LoadReadout,
    spec_force_cg_body: DVec3,
    is_pinned_rest: bool,
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
        aero_wrench: loads.aero,
        dynamic_pressure_pa: env.dynamic_pressure_pa,
        specific_force_cg_body: spec_force_cg_body,
        total_thrust_body_n: sub.thrust_body_n,
        thrust_torque_cg_body_nm: sub.thrust_torque_cg_body_nm,
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
        in_ground_contact: loads.any_contact,
        is_pinned_rest,
        is_splashed_down: loads.buoyancy_n > 0.0,
        total_buoyancy_n: loads.buoyancy_n,
        used_rigid_fast_path,
    }
}
