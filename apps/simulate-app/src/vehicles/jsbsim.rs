//! JSBSim-backed free-flight plants: trimmed in flight, parked on their gear, and stepped
//! over the shared world surface's ground.

use glam::{DMat3, DVec3};
use sim_core::{
    Block, ControlInputs, EngineStartMode, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock, JsbsimFdmConfig,
    KinematicInput, SystemControls,
};
use sim_geo::{BodyEllipsoid, Geodetic, ecef_to_geodetic};
use sim_pack::LoadedSyspack;

use super::FreeFlightPlant;
use crate::error::AppSimError;
use crate::world_surface::WorldSurfaceHandle;

pub(super) fn build_jsbsim_fdm(
    pkg: &LoadedSyspack,
    initial: Geodetic,
    heading_rad: f64,
    speed_m_s: f64,
    field_elev_m: f64,
) -> Result<(JsbsimFdmBlock, DVec3), AppSimError> {
    let resolver = pkg
        .build_jsbsim_resolver()
        .map_err(sim_scenario::ScenarioError::from)?;
    let (aircraft_path, model) = pkg
        .jsbsim_model()
        .map_err(sim_scenario::ScenarioError::from)?;
    // The ground goes in before the initial conditions run: a field below the ellipsoid
    // would otherwise start an airborne spawn with ground reactions from the default ground.
    let mut fdm = JsbsimFdmBlock::new_on_terrain(
        &JsbsimFdmConfig {
            resolver: &resolver,
            aircraft_path,
            model,
            period: JSBSIM_DEFAULT_PERIOD,
            initial,
            heading_rad,
            speed_m_s,
            alpha_rad: 0.04,
            engine_start: EngineStartMode::Running,
        },
        field_elev_m,
    )
    .map_err(sim_scenario::ScenarioError::from)?;
    if speed_m_s >= 75.0 {
        fdm.set_system_controls(SystemControls {
            gear_down: false,
            landing_light: false,
            ..SystemControls::running_defaults()
        });
    }
    drop(fdm.trim_level_flight());
    let state = fdm.output();
    let g_ecef = BodyEllipsoid::WGS84.gravity_fixed(state.position_ecef);
    let specific_force = -(DMat3::from_quat(state.attitude.inverse()) * g_ecef);
    Ok((fdm, specific_force))
}

/// A JSBSim aircraft at rest on its gear at `ground` (the terrain height under it), its
/// reference point `gear_height_m` up: the package's static height on its gear, so the
/// initial conditions see the gear neither crushed nor hanging.
pub(super) fn build_jsbsim_parked(
    pkg: &LoadedSyspack,
    ground: Geodetic,
    heading_rad: f64,
    engine_start: EngineStartMode,
    gear_height_m: f64,
) -> Result<FreeFlightPlant, AppSimError> {
    let resolver = pkg
        .build_jsbsim_resolver()
        .map_err(sim_scenario::ScenarioError::from)?;
    let (aircraft_path, model) = pkg
        .jsbsim_model()
        .map_err(sim_scenario::ScenarioError::from)?;
    let fdm = JsbsimFdmBlock::new_on_terrain(
        &JsbsimFdmConfig {
            resolver: &resolver,
            aircraft_path,
            model,
            period: JSBSIM_DEFAULT_PERIOD,
            initial: Geodetic {
                height_m: ground.height_m + gear_height_m,
                ..ground
            },
            heading_rad,
            speed_m_s: 0.0,
            alpha_rad: 0.0,
            engine_start,
        },
        ground.height_m,
    )
    .map_err(sim_scenario::ScenarioError::from)?;
    let state = fdm.output();
    let g_ecef = BodyEllipsoid::WGS84.gravity_fixed(state.position_ecef);
    Ok(FreeFlightPlant::Jsbsim {
        fdm: Box::new(fdm),
        specific_force_frd_m_s2: -(DMat3::from_quat(state.attitude.inverse()) * g_ecef),
    })
}

pub(super) fn build_jsbsim_plant(
    pkg: &LoadedSyspack,
    initial: Geodetic,
    heading_rad: f64,
    speed_m_s: f64,
    field_elev_m: f64,
) -> Result<FreeFlightPlant, AppSimError> {
    let (fdm, specific_force_frd_m_s2) =
        build_jsbsim_fdm(pkg, initial, heading_rad, speed_m_s, field_elev_m)?;
    Ok(FreeFlightPlant::Jsbsim {
        fdm: Box::new(fdm),
        specific_force_frd_m_s2,
    })
}

pub(super) fn step_jsbsim_with_force(
    fdm: &mut JsbsimFdmBlock,
    specific_force_frd_m_s2: &mut DVec3,
    controls: ControlInputs,
    surface: &WorldSurfaceHandle,
) -> Result<(), AppSimError> {
    let probe_geo = ecef_to_geodetic(fdm.output().position_ecef);
    if let Some((ground_m, version)) = surface.ellipsoidal_ground_and_version(&probe_geo) {
        fdm.sync_ground_versioned_m(ground_m, version)
            .map_err(sim_scenario::ScenarioError::from)?;
    }
    let prev = fdm.output();
    fdm.step(&KinematicInput { controls })
        .map_err(sim_scenario::ScenarioError::from)?;
    let next = fdm.output();
    let dt_s = fdm.period().as_seconds();
    let accel_ecef = (next.velocity_ecef - prev.velocity_ecef) / dt_s;
    let g_ecef = BodyEllipsoid::WGS84.gravity_fixed(next.position_ecef);
    *specific_force_frd_m_s2 = DMat3::from_quat(next.attitude.inverse()) * (accel_ecef - g_ecef);
    Ok(())
}
