use glam::DVec3;
use sim_core::{
    AirframeArticulation, Block, ControlInputs, EngineStatus, JsbsimFdmBlock, KinematicInput,
    KinematicModel, SystemControls, TickSpan, VehicleState,
};

use crate::error::AppSimError;
use crate::world_surface::WorldSurfaceHandle;

mod autopilot_c172;
mod aviate_rigs;
mod chain_displays;
mod chain_rig;
mod jsbsim;
mod plant_snapshot;
mod selection;
mod spawn;
mod structural;
#[cfg(test)]
mod tests;

pub use autopilot_c172::AutopilotC172;
#[cfg(any(target_arch = "wasm32", test))]
pub use chain_rig::PageSnapshot;
pub use chain_rig::{PageFeed, SetStanding, SteeredFix};
pub use plant_snapshot::PlantSnapshot;
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
pub use jsbsim::p38g_gear_height_m;
/// Height a parked kinematic vehicle sits above the ground, metres: the C172's, so the
/// pilot eye is at a cockpit's height over the runway.
pub const KINEMATIC_PARKED_HEIGHT_M: f64 = C172_GEAR_HEIGHT_M;

pub use aviate_rigs::{AviateFixedWingRig, AviateX500Rig, FixedWingSnapshot, X500Snapshot};
use aviate_rigs::{step_aviate_fixed_wing, step_aviate_x500};
use jsbsim::step_jsbsim_with_force;

/// Active free-flight plant stepped by the interactive clock.
pub enum FreeFlightPlant {
    /// Direct pilot-flown JSBSim fixed-wing aircraft (the `P-38G` or an imported one).
    Jsbsim {
        /// Loaded JSBSim flight dynamics block.
        fdm: Box<JsbsimFdmBlock>,
        /// Latest body FRD specific force (`m/s²`).
        specific_force_frd_m_s2: DVec3,
    },
    /// The hand-flown C172 with its autopilot installed: the pilot's controls fly it
    /// while the servos are released, and its flight controller and Pilotage's host core
    /// run beside them.
    C172Autopilot(Box<AutopilotC172>),
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
    /// The civil time the plant's flight states for its start, Unix seconds, when it
    /// states one: a link's flight happens at the link's epoch, not the host's clock.
    #[must_use]
    pub fn stated_epoch_utc_s(&self) -> Option<f64> {
        match self {
            Self::C172Mission(rig) => Some(rig.epoch_utc_s()),
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

    /// The JSBSim flight model of a plant that flies one in the open.
    fn jsbsim_fdm(&self) -> Option<&JsbsimFdmBlock> {
        match self {
            Self::Jsbsim { fdm, .. } => Some(fdm),
            Self::C172Autopilot(c172) => Some(c172.fdm()),
            Self::AviateFixedWing(rig) => Some(&rig.fdm),
            _ => None,
        }
    }

    /// Fixed integration period of this plant.
    #[must_use]
    pub fn period(&self) -> TickSpan {
        match self {
            Self::Jsbsim { fdm, .. } => fdm.period(),
            Self::C172Autopilot(c172) => c172.period(),
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
            Self::C172Autopilot(c172) => c172.state(),
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
            Self::C172Autopilot(c172) => Some(c172.specific_force_frd_m_s2()),
            Self::AviateX500(rig) => Some(rig.body.specific_force_cg_body()),
            Self::AviateFixedWing(rig) => Some(rig.specific_force_frd_m_s2),
            Self::C172Mission(rig) => Some(rig.specific_force_frd_m_s2()),
            Self::Kinematic(_) => None,
        }
    }

    /// Latched aircraft [`SystemControls`] when flying a JSBSim aircraft.
    #[must_use]
    pub fn system_controls(&self) -> Option<SystemControls> {
        self.jsbsim_fdm().map(JsbsimFdmBlock::system_controls)
    }

    /// Updates [`SystemControls`] on JSBSim-backed aircraft.
    pub fn set_system_controls(&mut self, controls: SystemControls) {
        match self {
            Self::Jsbsim { fdm, .. } => fdm.set_system_controls(controls),
            Self::C172Autopilot(c172) => c172.set_system_controls(controls),
            Self::AviateFixedWing(rig) => rig.fdm.set_system_controls(controls),
            _ => {}
        }
    }

    /// The centre of gravity in the flight model's structural frame (inches, `x` aft, `y`
    /// right, `z` up) on JSBSim-backed aircraft.
    #[must_use]
    pub fn cg_structural_in(&self) -> Option<DVec3> {
        self.jsbsim_fdm().map(JsbsimFdmBlock::cg_structural_in)
    }

    /// The ellipsoidal ground height the JSBSim plant was last given.
    #[cfg(test)]
    pub fn fdm_ground_m(&self) -> Option<f64> {
        self.jsbsim_fdm()
            .and_then(JsbsimFdmBlock::last_ground_sync_m)
    }

    /// Current engine running state and propeller RPM on JSBSim-backed aircraft.
    #[must_use]
    pub fn engine_status(&self) -> Option<EngineStatus> {
        match self {
            Self::C172Mission(rig) => Some(rig.engine_status()),
            _ => self.jsbsim_fdm().map(JsbsimFdmBlock::engine_status),
        }
    }

    /// Current engine, propeller, control-surface, and landing-gear articulation.
    #[must_use]
    pub fn articulation(&self, elapsed_s: f64) -> AirframeArticulation {
        match self {
            Self::Jsbsim { fdm, .. } => fdm.articulation(),
            Self::C172Autopilot(c172) => c172.fdm().articulation(),
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

    /// Whether an in-process Aviate flight controller that flies the vehicle is armed:
    /// `None` on plants without one, and on the hand-flown C172, whose released servos
    /// leave its flight controller nothing to fly.
    #[must_use]
    pub fn aviate_armed(&self) -> Option<bool> {
        match self {
            Self::AviateX500(rig) => Some(rig.runner.is_armed()),
            Self::AviateFixedWing(rig) => Some(rig.runner.is_armed()),
            Self::C172Mission(rig) => Some(rig.armed()),
            Self::Jsbsim { .. } | Self::C172Autopilot(_) | Self::Kinematic(_) => None,
        }
    }

    /// Skips intermediate [`PageFeed::at`] lookups prior to `cycle` during high-warp page
    /// jumps while sampling the final two controller cycles for display interpolation.
    pub fn set_page_target_cycle(&mut self, cycle: u64) {
        if let Self::C172Mission(rig) = self {
            rig.set_page_target_cycle(cycle);
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
            Self::C172Autopilot(c172) => c172.step(controls, surface),
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
