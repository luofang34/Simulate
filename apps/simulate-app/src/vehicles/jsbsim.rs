//! JSBSim-backed free-flight plants: trimmed in flight, parked on their gear, and stepped
//! over the shared world surface's ground.

use glam::{DMat3, DVec3};
use sim_core::{
    Block, ControlInputs, EngineStartMode, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock, JsbsimFdmConfig,
    KinematicInput, SystemControls, TickSpan,
};
use sim_geo::{BodyEllipsoid, Geodetic, ecef_to_geodetic};
use sim_pack::LoadedSyspack;

use super::FreeFlightPlant;
use crate::error::AppSimError;
use crate::world_surface::WorldSurfaceHandle;

/// A JSBSim aircraft in flight at `initial` over a field at `field_elev_m`, stepping
/// every `period`, trimmed for level flight where its trim converges, and the specific
/// force of that unaccelerated flight.
pub(super) fn build_jsbsim_fdm(
    pkg: &LoadedSyspack,
    initial: Geodetic,
    heading_rad: f64,
    speed_m_s: f64,
    field_elev_m: f64,
    period: TickSpan,
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
            period,
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
/// reference point `gear_height_m` up and pitched `pitch_rad` nose-up: the package's
/// static height and attitude on its gear, so the initial conditions see the gear neither
/// crushed nor hanging.
pub(super) fn build_jsbsim_parked(
    pkg: &LoadedSyspack,
    ground: Geodetic,
    heading_rad: f64,
    engine_start: EngineStartMode,
    gear_height_m: f64,
    pitch_rad: f64,
) -> Result<FreeFlightPlant, AppSimError> {
    let stance = ParkedStance {
        ground,
        heading_rad,
        engine_start,
        gear_height_m,
        pitch_rad,
    };
    let fdm = build_parked_fdm(pkg, &stance, JSBSIM_DEFAULT_PERIOD)?;
    let state = fdm.output();
    let g_ecef = BodyEllipsoid::WGS84.gravity_fixed(state.position_ecef);
    Ok(FreeFlightPlant::Jsbsim {
        fdm: Box::new(fdm),
        specific_force_frd_m_s2: -(DMat3::from_quat(state.attitude.inverse()) * g_ecef),
    })
}

/// How an aircraft stands on its gear.
#[derive(Debug, Clone, Copy)]
pub(super) struct ParkedStance {
    /// Where it stands; the height is the terrain's under it.
    pub ground: Geodetic,
    /// Heading from true north, radians.
    pub heading_rad: f64,
    /// Engine running or cold.
    pub engine_start: EngineStartMode,
    /// Static height of the reference point over the ground, metres.
    pub gear_height_m: f64,
    /// Static nose-up pitch, radians.
    pub pitch_rad: f64,
}

/// The flight model of an aircraft standing as `stance` says, stepping every `period`.
pub(super) fn build_parked_fdm(
    pkg: &LoadedSyspack,
    stance: &ParkedStance,
    period: TickSpan,
) -> Result<JsbsimFdmBlock, AppSimError> {
    let ParkedStance {
        ground,
        heading_rad,
        engine_start,
        gear_height_m,
        pitch_rad,
    } = *stance;
    let resolver = pkg
        .build_jsbsim_resolver()
        .map_err(sim_scenario::ScenarioError::from)?;
    let (aircraft_path, model) = pkg
        .jsbsim_model()
        .map_err(sim_scenario::ScenarioError::from)?;
    JsbsimFdmBlock::new_on_terrain(
        &JsbsimFdmConfig {
            resolver: &resolver,
            aircraft_path,
            model,
            period,
            initial: Geodetic {
                height_m: ground.height_m + gear_height_m,
                ..ground
            },
            heading_rad,
            speed_m_s: 0.0,
            alpha_rad: pitch_rad,
            engine_start,
        },
        ground.height_m,
    )
    .map_err(|e| sim_scenario::ScenarioError::from(e).into())
}

pub(super) fn build_jsbsim_plant(
    pkg: &LoadedSyspack,
    initial: Geodetic,
    heading_rad: f64,
    speed_m_s: f64,
    field_elev_m: f64,
) -> Result<FreeFlightPlant, AppSimError> {
    let (fdm, specific_force_frd_m_s2) = build_jsbsim_fdm(
        pkg,
        initial,
        heading_rad,
        speed_m_s,
        field_elev_m,
        JSBSIM_DEFAULT_PERIOD,
    )?;
    Ok(FreeFlightPlant::Jsbsim {
        fdm: Box::new(fdm),
        specific_force_frd_m_s2,
    })
}

/// The ground `surface` has under `position_ecef`, metres above the ellipsoid, and the
/// version of the surface that states it; `None` where the surface has no ground yet.
pub(super) fn ground_under(
    surface: &WorldSurfaceHandle,
    position_ecef: DVec3,
) -> Option<(f64, u64)> {
    surface.ellipsoidal_ground_and_version(&ecef_to_geodetic(position_ecef))
}

pub(super) fn step_jsbsim_with_force(
    fdm: &mut JsbsimFdmBlock,
    specific_force_frd_m_s2: &mut DVec3,
    controls: ControlInputs,
    surface: &WorldSurfaceHandle,
) -> Result<(), AppSimError> {
    if let Some((ground_m, version)) = ground_under(surface, fdm.output().position_ecef) {
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

/// Static height of the Lockheed P-38G Lightning's JSBSim reference point (its CG) above
/// level ground on its tricycle gear, metres.
pub fn p38g_gear_height_m() -> f64 {
    use super::{P38G_LOADED_CG_ABOVE_FRL_IN, P38G_LOADED_CG_FS_IN, P38G_STATIC_PITCH_RAD};
    use sim_p38g::drawing as d;
    let below_main = f64::from(d::MAIN_AXLE_BELOW_FRL_STATIC_IN + d::MAIN_ROLLING_RADIUS_IN);
    let ahead_of_main = f64::from(d::MAIN_AXLE_FS_IN) - P38G_LOADED_CG_FS_IN;
    let above_ground_line =
        P38G_LOADED_CG_ABOVE_FRL_IN + below_main + ahead_of_main * P38G_STATIC_PITCH_RAD.tan();
    above_ground_line * P38G_STATIC_PITCH_RAD.cos() * 0.0254
}

impl FreeFlightPlant {
    /// Instantiates a runtime-imported external `syspack` fixed-wing plant in flight.
    pub fn new_external_at(
        pkg: &LoadedSyspack,
        cruise_mps: f64,
        spawn: Geodetic,
        heading_rad: f64,
        field_elev_m: f64,
    ) -> Result<Self, AppSimError> {
        build_jsbsim_plant(pkg, spawn, heading_rad, cruise_mps, field_elev_m)
    }

    /// Instantiates a runtime-imported external `syspack` plant parked on its landing gear.
    pub fn new_external_on_ground(
        pkg: &LoadedSyspack,
        gear_height_m: f64,
        ground: Geodetic,
        heading_rad: f64,
        cold: bool,
    ) -> Result<Self, AppSimError> {
        let engine = if cold {
            EngineStartMode::ColdAndDark
        } else {
            EngineStartMode::Running
        };
        build_jsbsim_parked(pkg, ground, heading_rad, engine, gear_height_m, 0.0)
    }
}
