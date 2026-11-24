//! Where and how each free-flight plant starts: in flight over a field, or at rest on
//! the ground.

use sim_core::{EngineStartMode, KinematicModel, VehicleState};
use sim_geo::Geodetic;
use sim_pack::{builtin_c172p_package, builtin_c172s_package, builtin_p38g_package};

use super::autopilot_c172::AutopilotC172;
use super::aviate_rigs::{build_aviate_fixed_wing, build_aviate_x500};
use super::jsbsim::{build_jsbsim_parked, build_jsbsim_plant};
use super::{
    FBW_GROUND_FALLBACK_AGL_M, FreeFlightPlant, FreeFlightVehicle, KINEMATIC_PARKED_HEIGHT_M,
    P38G_STATIC_PITCH_RAD, PageFeed, X500_GROUND_HOVER_M, chain_rig, p38g_gear_height_m,
};
use crate::error::AppSimError;
use crate::supervisor_rig::{DELAWARE_RUNWAY_BEARING_RAD, default_delaware_spawn};

impl FreeFlightPlant {
    /// Instantiates `vehicle` over the Delaware corridor runway alignment.
    pub fn new(vehicle: FreeFlightVehicle) -> Result<Self, AppSimError> {
        Self::new_at(
            vehicle,
            default_delaware_spawn(),
            DELAWARE_RUNWAY_BEARING_RAD,
            8.5,
        )
    }

    /// Instantiates `vehicle` at `spawn` (`heading_rad`, `field_elev_m`).
    pub fn new_at(
        vehicle: FreeFlightVehicle,
        spawn: Geodetic,
        heading_rad: f64,
        field_elev_m: f64,
    ) -> Result<Self, AppSimError> {
        match vehicle {
            FreeFlightVehicle::JsbsimC172 | FreeFlightVehicle::JsbsimC172p => {
                let (pkg, speed_m_s) = if vehicle == FreeFlightVehicle::JsbsimC172p {
                    (builtin_c172p_package(), 50.0)
                } else {
                    (builtin_c172s_package(), 52.0)
                };
                let pkg = pkg.map_err(sim_scenario::ScenarioError::from)?;
                Ok(Self::C172Autopilot(Box::new(AutopilotC172::in_flight(
                    &pkg,
                    spawn,
                    heading_rad,
                    speed_m_s,
                    field_elev_m,
                )?)))
            }
            FreeFlightVehicle::JsbsimP38g => {
                let pkg = builtin_p38g_package().map_err(sim_scenario::ScenarioError::from)?;
                build_jsbsim_plant(&pkg, spawn, heading_rad, 95.0, field_elev_m)
            }
            FreeFlightVehicle::AviateX500Direct => {
                let hover_spawn = Geodetic {
                    height_m: field_elev_m + 110.0,
                    ..spawn
                };
                Ok(Self::AviateX500(Box::new(build_aviate_x500(
                    hover_spawn,
                    heading_rad,
                )?)))
            }
            FreeFlightVehicle::AviateFixedWingDirect => Ok(Self::AviateFixedWing(Box::new(
                build_aviate_fixed_wing(spawn, heading_rad, field_elev_m)?,
            ))),
            FreeFlightVehicle::C172Mission => Self::on_first_link(),
            FreeFlightVehicle::Kinematic => Ok(Self::Kinematic(KinematicModel::new(
                VehicleState::level_flight(&spawn, heading_rad, 55.0),
            ))),
            FreeFlightVehicle::External(_) => Err(sim_scenario::ScenarioError::Unsupported {
                detail: "external plugin vehicle requires LiverySession context",
            }
            .into()),
        }
    }

    /// The C172 a link's page flies, drawn from what the page publishes.
    #[must_use]
    pub fn page_mission(feed: PageFeed) -> Self {
        Self::C172Mission(Box::new(chain_rig::page_chain_rig(feed)))
    }

    /// The C172 at the first link's start, wherever the spawn was: the link states where
    /// the flight begins.
    fn on_first_link() -> Result<Self, AppSimError> {
        Ok(Self::C172Mission(Box::new(chain_rig::build_chain_rig()?)))
    }

    /// Instantiates `vehicle` at rest on the ground at `spawn` (whose height is the ground
    /// under it), engine running or `cold`. The X500 hovers [`X500_GROUND_HOVER_M`] up, and
    /// the Aviate fixed-wing controller, which arms only in flight, starts
    /// [`FBW_GROUND_FALLBACK_AGL_M`] over the spawn.
    pub fn new_on_ground(
        vehicle: FreeFlightVehicle,
        spawn: Geodetic,
        heading_rad: f64,
        cold: bool,
    ) -> Result<Self, AppSimError> {
        let engine = if cold {
            EngineStartMode::ColdAndDark
        } else {
            EngineStartMode::Running
        };
        let ground_m = spawn.height_m;
        match vehicle {
            FreeFlightVehicle::JsbsimC172 | FreeFlightVehicle::JsbsimC172p => {
                let pkg = if vehicle == FreeFlightVehicle::JsbsimC172p {
                    builtin_c172p_package()
                } else {
                    builtin_c172s_package()
                }
                .map_err(sim_scenario::ScenarioError::from)?;
                Ok(Self::C172Autopilot(Box::new(AutopilotC172::parked(
                    &pkg,
                    spawn,
                    heading_rad,
                    engine,
                )?)))
            }
            FreeFlightVehicle::JsbsimP38g => {
                let pkg = builtin_p38g_package().map_err(sim_scenario::ScenarioError::from)?;
                build_jsbsim_parked(
                    &pkg,
                    spawn,
                    heading_rad,
                    engine,
                    p38g_gear_height_m(),
                    P38G_STATIC_PITCH_RAD,
                )
            }
            FreeFlightVehicle::AviateX500Direct => {
                Ok(Self::AviateX500(Box::new(build_aviate_x500(
                    Geodetic {
                        height_m: ground_m + X500_GROUND_HOVER_M,
                        ..spawn
                    },
                    heading_rad,
                )?)))
            }
            FreeFlightVehicle::C172Mission => Self::on_first_link(),
            FreeFlightVehicle::AviateFixedWingDirect => Self::new_at(
                vehicle,
                Geodetic {
                    height_m: ground_m + FBW_GROUND_FALLBACK_AGL_M,
                    ..spawn
                },
                heading_rad,
                ground_m,
            ),
            FreeFlightVehicle::Kinematic => Ok(Self::Kinematic(KinematicModel::new(
                VehicleState::level_flight(
                    &Geodetic {
                        height_m: ground_m + KINEMATIC_PARKED_HEIGHT_M,
                        ..spawn
                    },
                    heading_rad,
                    0.0,
                ),
            ))),
            FreeFlightVehicle::External(_) => Err(sim_scenario::ScenarioError::Unsupported {
                detail: "external plugin vehicle requires LiverySession context",
            }
            .into()),
        }
    }
}
