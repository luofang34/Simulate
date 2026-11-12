use glam::DVec3;
use sim_core::{
    AirframeArticulation, Block, ControlInputs, EngineStartMode, EngineStatus, JsbsimFdmBlock,
    JsbsimFdmSnapshot, KinematicInput, KinematicModel, SystemControls, TickSpan, VehicleState,
};
use sim_geo::Geodetic;
use sim_pack::{builtin_c172_package, builtin_p38g_package};

use crate::error::AppSimError;
use crate::supervisor_rig::{DELAWARE_RUNWAY_BEARING_RAD, default_delaware_spawn};
use crate::world_surface::WorldSurfaceHandle;

mod aviate_rigs;
mod chain_rig;
mod jsbsim;
mod selection;
mod structural;
#[cfg(test)]
mod tests;

pub use chain_rig::SetStanding;
pub use selection::FreeFlightVehicle;
pub use structural::ModelOrigins;

/// Height the X500 hovers over a ground spawn, metres.
pub const X500_GROUND_HOVER_M: f64 = 3.0;
/// Height over a ground spawn the Aviate fixed-wing starts at instead, metres.
pub const FBW_GROUND_FALLBACK_AGL_M: f64 = 150.0;
/// Static height of the C172's JSBSim reference point (its CG) above level ground on
/// its gear, metres: the wheels reach 1.30 m below the reference line, the high wing
/// tanks lift the CG 0.08 m above it, and the gear compresses about 0.05 m.
pub const C172_GEAR_HEIGHT_M: f64 = 1.324;
/// Height of the P-38G's loaded centre of gravity above its fuselage reference line in
/// the flight model (`vehicles/p38g/fdm/p38g.xml`, 15,000 lb with full internal fuel),
/// inches.
pub const P38G_LOADED_CG_ABOVE_FRL_IN: f64 = 5.345;
/// Station of the P-38G's loaded centre of gravity in the flight model: `24 %` of the
/// mean aerodynamic chord (`FS 130.51 + 0.24 x 84.25`), inches.
pub const P38G_LOADED_CG_FS_IN: f64 = 150.73;
/// Pitch the Lockheed P-38G Lightning stands at on its gear: AN 01-75F-2's static ground
/// line, `5 deg 33' 46"` under the fuselage reference line, radians.
pub const P38G_STATIC_PITCH_RAD: f64 =
    sim_p38g::drawing::STATIC_GROUND_ANGLE_DEG as f64 * std::f64::consts::PI / 180.0;
/// Static height of the Lockheed P-38G Lightning's JSBSim reference point (its CG) above
/// level ground on its tricycle gear, metres. AN 01-75F-2 puts the main-tyre contact
/// `54.575 + 15.300 in` below the fuselage reference line with the oleos static and the
/// ground rising aft at [`P38G_STATIC_PITCH_RAD`]; the CG sits
/// [`P38G_LOADED_CG_ABOVE_FRL_IN`] above that line at [`P38G_LOADED_CG_FS_IN`], ahead of
/// the main axle.
pub fn p38g_gear_height_m() -> f64 {
    use sim_p38g::drawing as d;
    let below_main = f64::from(d::MAIN_AXLE_BELOW_FRL_STATIC_IN + d::MAIN_ROLLING_RADIUS_IN);
    let ahead_of_main = f64::from(d::MAIN_AXLE_FS_IN) - P38G_LOADED_CG_FS_IN;
    let above_ground_line =
        P38G_LOADED_CG_ABOVE_FRL_IN + below_main + ahead_of_main * P38G_STATIC_PITCH_RAD.tan();
    above_ground_line * P38G_STATIC_PITCH_RAD.cos() * 0.0254
}
/// Height a parked kinematic vehicle sits above the ground, metres: the C172's, so the
/// pilot eye is at a cockpit's height over the runway.
pub const KINEMATIC_PARKED_HEIGHT_M: f64 = C172_GEAR_HEIGHT_M;

pub use aviate_rigs::{AviateFixedWingRig, AviateX500Rig, FixedWingSnapshot, X500Snapshot};
use aviate_rigs::{
    build_aviate_fixed_wing, build_aviate_x500, step_aviate_fixed_wing, step_aviate_x500,
};
use jsbsim::{build_jsbsim_parked, build_jsbsim_plant, step_jsbsim_with_force};

/// Everything a [`FreeFlightPlant`] needs to fly the same bits again from one instant.
#[derive(Debug, Clone)]
pub enum PlantSnapshot {
    /// JSBSim aircraft state, latched systems, and the derived specific force.
    Jsbsim {
        /// Complete JSBSim executive state.
        fdm: Box<JsbsimFdmSnapshot>,
        /// Body FRD specific force (`m/s²`) at the snapshot.
        specific_force_frd_m_s2: DVec3,
        /// Latched operator system controls at the snapshot.
        system: Box<SystemControls>,
    },
    /// Aviate X500 rigid body and position-hold target.
    AviateX500(Box<X500Snapshot>),
    /// Aviate fixed-wing JSBSim aircraft and attitude target.
    AviateFixedWing(Box<FixedWingSnapshot>),
    /// A flight on the real control chain, which keeps no rewindable state: the marker
    /// only says which plant it came from.
    C172Mission,
    /// Kinematic model state.
    Kinematic(Box<KinematicModel>),
}

/// Active free-flight plant stepped by the interactive clock.
pub enum FreeFlightPlant {
    /// Direct pilot-flown JSBSim fixed-wing aircraft (`C172` or `P-38G`).
    Jsbsim {
        /// Loaded JSBSim flight dynamics block.
        fdm: Box<JsbsimFdmBlock>,
        /// Latest body FRD specific force (`m/s²`).
        specific_force_frd_m_s2: DVec3,
    },
    /// Tier-0 Aviate quadrotor flight controller closed around a `400 Hz` 6-DoF rigid body.
    AviateX500(Box<AviateX500Rig>),
    /// Tier-0 Aviate fixed-wing controller closed around a `120 Hz` JSBSim plant.
    AviateFixedWing(Box<AviateFixedWingRig>),
    /// The C172 on the real chain: Pilotage's host core flying the first link through
    /// the Aviate flight controller, Navigate's sets navigating beside it.
    C172Mission(Box<chain_rig::ChainRig>),
    /// Lightweight kinematic model.
    Kinematic(KinematicModel),
}

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
            FreeFlightVehicle::JsbsimC172 => {
                let pkg = builtin_c172_package().map_err(sim_scenario::ScenarioError::from)?;
                build_jsbsim_plant(&pkg, spawn, heading_rad, 52.0, field_elev_m)
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
        }
    }

    /// The C172 at the first link's start, wherever the spawn was: the link states where
    /// the flight begins.
    fn on_first_link() -> Result<Self, AppSimError> {
        Ok(Self::C172Mission(Box::new(chain_rig::build_chain_rig()?)))
    }

    /// The civil time the plant's flight states for its start, Unix seconds, when it
    /// states one: a link's flight happens at the link's epoch, not the host's clock.
    #[must_use]
    pub fn stated_epoch_utc_s(&self) -> Option<f64> {
        match self {
            Self::C172Mission(rig) => Some(rig.epoch_utc_s()),
            _ => None,
        }
    }

    /// The flight controller's account of its last cycle and the estimate the displays
    /// hold, when flying the real chain.
    #[must_use]
    pub fn chain_avionics(&self) -> Option<(sim_flight_chain::FcAvionics, VehicleState)> {
        match self {
            Self::C172Mission(rig) => Some((rig.avionics(), rig.held_estimate())),
            _ => None,
        }
    }

    /// Pilotage's navigation display, when flying the real chain.
    #[must_use]
    pub fn chain_navigation(&self) -> Option<sim_flight_chain::NavigationDisplay> {
        match self {
            Self::C172Mission(rig) => rig.navigation(),
            _ => None,
        }
    }

    /// Pilotage's mission phase and each navigation set's standing, when flying the real
    /// chain.
    #[must_use]
    pub fn chain_readout(
        &self,
    ) -> Option<(
        Option<&str>,
        &std::collections::BTreeMap<String, SetStanding>,
    )> {
        match self {
            Self::C172Mission(rig) => Some((rig.phase(), rig.standings())),
            _ => None,
        }
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
            FreeFlightVehicle::JsbsimC172 => {
                let pkg = builtin_c172_package().map_err(sim_scenario::ScenarioError::from)?;
                build_jsbsim_parked(&pkg, spawn, heading_rad, engine, C172_GEAR_HEIGHT_M, 0.0)
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
        }
    }

    /// Fixed integration period of this plant.
    #[must_use]
    pub fn period(&self) -> TickSpan {
        match self {
            Self::Jsbsim { fdm, .. } => fdm.period(),
            Self::AviateX500(rig) => rig.body.period(),
            Self::AviateFixedWing(rig) => rig.fdm.period(),
            Self::C172Mission(rig) => rig.period(),
            Self::Kinematic(model) => model.period(),
        }
    }

    /// Current `f64` ECEF vehicle state.
    #[must_use]
    pub fn state(&self) -> VehicleState {
        match self {
            Self::Jsbsim { fdm, .. } => fdm.output(),
            Self::AviateX500(rig) => rig.body.output(),
            Self::AviateFixedWing(rig) => rig.fdm.output(),
            Self::C172Mission(rig) => rig.state(),
            Self::Kinematic(model) => model.output(),
        }
    }

    /// Body FRD specific force (`m/s²`) for turn/slip and G-load instrumentation.
    #[must_use]
    pub fn specific_force_frd_m_s2(&self) -> Option<DVec3> {
        match self {
            Self::Jsbsim {
                specific_force_frd_m_s2,
                ..
            } => Some(*specific_force_frd_m_s2),
            Self::AviateX500(rig) => Some(rig.body.specific_force_cg_body()),
            Self::AviateFixedWing(rig) => Some(rig.specific_force_frd_m_s2),
            Self::C172Mission(rig) => Some(rig.specific_force_frd_m_s2()),
            Self::Kinematic(_) => None,
        }
    }

    /// Latched aircraft [`SystemControls`] when flying a JSBSim aircraft.
    #[must_use]
    pub fn system_controls(&self) -> Option<SystemControls> {
        match self {
            Self::Jsbsim { fdm, .. } => Some(fdm.system_controls()),
            Self::AviateFixedWing(rig) => Some(rig.fdm.system_controls()),
            _ => None,
        }
    }

    /// Updates [`SystemControls`] on JSBSim-backed aircraft.
    pub fn set_system_controls(&mut self, controls: SystemControls) {
        match self {
            Self::Jsbsim { fdm, .. } => fdm.set_system_controls(controls),
            Self::AviateFixedWing(rig) => rig.fdm.set_system_controls(controls),
            _ => {}
        }
    }

    /// The centre of gravity in the flight model's structural frame (inches, `x` aft, `y`
    /// right, `z` up) on JSBSim-backed aircraft.
    #[must_use]
    pub fn cg_structural_in(&self) -> Option<DVec3> {
        match self {
            Self::Jsbsim { fdm, .. } => Some(fdm.cg_structural_in()),
            Self::AviateFixedWing(rig) => Some(rig.fdm.cg_structural_in()),
            _ => None,
        }
    }

    /// The ellipsoidal ground height the JSBSim plant was last given.
    #[cfg(test)]
    pub fn fdm_ground_m(&self) -> Option<f64> {
        match self {
            Self::Jsbsim { fdm, .. } => fdm.last_ground_sync_m(),
            Self::AviateFixedWing(rig) => rig.fdm.last_ground_sync_m(),
            _ => None,
        }
    }

    /// Current engine running state and propeller RPM on JSBSim-backed aircraft.
    #[must_use]
    pub fn engine_status(&self) -> Option<EngineStatus> {
        match self {
            Self::Jsbsim { fdm, .. } => Some(fdm.engine_status()),
            Self::AviateFixedWing(rig) => Some(rig.fdm.engine_status()),
            Self::C172Mission(rig) => Some(rig.engine_status()),
            _ => None,
        }
    }

    /// Current engine, propeller, control-surface, and landing-gear articulation.
    #[must_use]
    pub fn articulation(&self, elapsed_s: f64) -> AirframeArticulation {
        match self {
            Self::Jsbsim { fdm, .. } => fdm.articulation(),
            Self::AviateFixedWing(rig) => rig.fdm.articulation(),
            Self::C172Mission(rig) => rig.articulation(),
            Self::AviateX500(rig) => AirframeArticulation::from_kinematic(
                &rig.body.output(),
                ControlInputs {
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: 0.0,
                    throttle: 0.55,
                },
                elapsed_s,
            ),
            Self::Kinematic(model) => AirframeArticulation::from_kinematic(
                &model.output(),
                ControlInputs {
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: 0.0,
                    throttle: 0.70,
                },
                elapsed_s,
            ),
        }
    }

    /// Whether an in-process Aviate flight controller is armed (`None` on non-Aviate plants).
    #[must_use]
    pub fn aviate_armed(&self) -> Option<bool> {
        match self {
            Self::AviateX500(rig) => Some(rig.runner.is_armed()),
            Self::AviateFixedWing(rig) => Some(rig.runner.is_armed()),
            Self::C172Mission(rig) => Some(rig.armed()),
            Self::Jsbsim { .. } | Self::Kinematic(_) => None,
        }
    }

    /// Captures the plant for a later [`Self::restore`].
    #[must_use]
    pub fn snapshot(&self) -> PlantSnapshot {
        match self {
            Self::Jsbsim {
                fdm,
                specific_force_frd_m_s2,
            } => PlantSnapshot::Jsbsim {
                fdm: Box::new(fdm.snapshot()),
                specific_force_frd_m_s2: *specific_force_frd_m_s2,
                system: Box::new(fdm.system_controls()),
            },
            Self::AviateX500(rig) => PlantSnapshot::AviateX500(Box::new(rig.snapshot())),
            Self::AviateFixedWing(rig) => PlantSnapshot::AviateFixedWing(Box::new(rig.snapshot())),
            Self::C172Mission(_) => PlantSnapshot::C172Mission,
            Self::Kinematic(model) => PlantSnapshot::Kinematic(Box::new(model.snapshot())),
        }
    }

    /// Rewinds to `snap`; Aviate controllers are rebuilt, fast-warmed, and re-armed there.
    /// Fails, leaving the plant as it was, on a snapshot of another kind of plant.
    pub fn restore(&mut self, snap: &PlantSnapshot) -> Result<(), AppSimError> {
        match (self, snap) {
            (
                Self::Jsbsim {
                    fdm,
                    specific_force_frd_m_s2,
                },
                PlantSnapshot::Jsbsim {
                    fdm: saved,
                    specific_force_frd_m_s2: force,
                    system,
                },
            ) => {
                fdm.try_restore(saved)
                    .map_err(sim_scenario::ScenarioError::from)?;
                fdm.set_system_controls(**system);
                *specific_force_frd_m_s2 = *force;
                Ok(())
            }
            (Self::AviateX500(rig), PlantSnapshot::AviateX500(saved)) => rig.restore(saved),
            (Self::AviateFixedWing(rig), PlantSnapshot::AviateFixedWing(saved)) => {
                rig.restore(saved)
            }
            (Self::Kinematic(model), PlantSnapshot::Kinematic(saved)) => {
                model.restore(saved);
                Ok(())
            }
            (Self::C172Mission(_), PlantSnapshot::C172Mission) => {
                Err(sim_scenario::ScenarioError::Unsupported {
                    detail: "a flight on the real control chain cannot be rewound; restart it",
                }
                .into())
            }
            _ => Err(sim_scenario::ScenarioError::SnapshotMismatch {
                detail: "the snapshot was taken from another free-flight plant",
            }
            .into()),
        }
    }

    /// Advances the plant by one integration period under `controls`, with JSBSim gear
    /// contact on `surface`'s ground.
    pub fn step(
        &mut self,
        controls: ControlInputs,
        surface: &WorldSurfaceHandle,
    ) -> Result<(), AppSimError> {
        match self {
            Self::Jsbsim {
                fdm,
                specific_force_frd_m_s2,
            } => step_jsbsim_with_force(fdm, specific_force_frd_m_s2, controls, surface),
            Self::AviateX500(rig) => step_aviate_x500(rig, controls),
            Self::AviateFixedWing(rig) => step_aviate_fixed_wing(rig, controls, surface),
            Self::C172Mission(rig) => rig.step(),
            Self::Kinematic(model) => {
                model
                    .step(&KinematicInput { controls })
                    .map_err(sim_scenario::ScenarioError::from)?;
                Ok(())
            }
        }
    }
}
