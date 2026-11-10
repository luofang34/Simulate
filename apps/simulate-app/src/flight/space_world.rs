//! Spaceflight on `Raketa`: one universe whose ownship starts on a launch
//! pad, flies under the pilot's hand or ascent guidance, coasts in orbital
//! propagation, and comes back to flight dynamics whenever it is flown again.
//!
//! The app draws and flies every vehicle in aircraft body axes (forward,
//! right, down). `Raketa` stands on its tail, so its state is reported as an
//! aircraft's whose "up" is the rocket's nose: the chase camera then looks at
//! its side, and the stick tilts the nose away from or across the view.

use glam::{DMat3, DQuat, DVec3};
use sim_apollo::{CockpitInstrumentPanel, CockpitInstrumentSnapshot, EarthVehicleSources};
use sim_core::{
    AirframeArticulation, CameraPoseEcef, ControlInputs, SimInstant, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, ecef_to_geodetic};
use sim_orbital::{
    AscentGuidanceConfig, ContinuousPlantState, FOUR_G_PROPER_ACCEL_M_S2, NaifId, VesselId,
    evaluate_gravity_turn_ascent,
};
use sim_raketa::{RAKETA_CG_Y_M, RAKETA_PLANT_PERIOD, RaketaCommand, RaketaPlant};
use sim_spacetime::SimEpoch;
use sim_universe::{Tier, TierPolicy, Universe, UniverseError};

use super::free_world::SolarSystem;
use crate::error::AppSimError;
use crate::world_surface::WorldSurfaceHandle;

/// The vessel the pilot flies.
const OWNSHIP: VesselId = VesselId(1);

/// How fast a full stick deflection swings the commanded nose direction,
/// rad/s.
const NOSE_SLEW_RAD_S: f64 = 0.35;

/// Launch Complex 39A, where `Raketa` starts.
const PAD_LAT_DEG: f64 = 28.608_4;
const PAD_LON_DEG: f64 = -80.604_3;

/// `Raketa`'s body axes (`+Y` along the nose) seen from the aircraft axes
/// the app flies and draws: forward is the rocket's `+Z`, right its `-X`,
/// down its `-Y`.
const RAKETA_FROM_AIRCRAFT: DMat3 = DMat3::from_cols(
    DVec3::new(0.0, 0.0, 1.0),
    DVec3::new(-1.0, 0.0, 0.0),
    DVec3::new(0.0, -1.0, 0.0),
);

/// A spaceflight's universe and what the cockpit shows of it.
pub(super) struct SpaceWorld {
    universe: Universe,
    ground_height_m: f64,
    truth: VehicleState,
    nose_icrf: Option<DVec3>,
    ascent_guidance: bool,
    panel: CockpitInstrumentPanel,
}

impl SpaceWorld {
    /// `Raketa` on its pad, on ground at `ground_height_m` above the
    /// ellipsoid, at the first tick of `epoch`.
    pub(super) fn on_pad(
        epoch: SimEpoch,
        solar: &SolarSystem,
        ground_height_m: f64,
    ) -> Result<Self, AppSimError> {
        let site = Geodetic::from_degrees(PAD_LAT_DEG, PAD_LON_DEG, ground_height_m);
        let environment = solar.raketa_environment();
        let plant = RaketaPlant::on_pad(
            environment.clone(),
            epoch.clone(),
            SimInstant::START,
            BodyEllipsoid::WGS84,
            &site,
        )
        .map_err(|e| universe_error(UniverseError::LocalPlant(Box::new(e))))?;
        let mut universe =
            Universe::new(epoch, solar.context()).with_tier_policy(TierPolicy::standard());
        universe
            .insert_local(OWNSHIP, Box::new(plant), RaketaPlant::factory(environment))
            .map_err(universe_error)?;
        let truth = aircraft_axes(universe.vehicle_state(OWNSHIP).map_err(universe_error)?);
        let mut world = Self {
            universe,
            ground_height_m,
            truth,
            nose_icrf: None,
            ascent_guidance: true,
            panel: CockpitInstrumentPanel::new(SimInstant::START),
        };
        world.observe();
        Ok(world)
    }

    /// Ellipsoidal height of the ground at the pad, where `surface` knows it.
    pub(super) fn pad_ground_height_m(surface: &WorldSurfaceHandle) -> f64 {
        surface
            .ellipsoidal_ground_m(&Geodetic::from_degrees(PAD_LAT_DEG, PAD_LON_DEG, 0.0))
            .unwrap_or(0.0)
    }

    /// What each tick of this flight means in TDB, UTC and UT1.
    pub(super) const fn epoch(&self) -> &SimEpoch {
        self.universe.epoch()
    }

    /// The step the flight advances by.
    pub(super) const fn period() -> TickSpan {
        RAKETA_PLANT_PERIOD
    }

    /// The instant the truth state describes.
    pub(super) const fn now(&self) -> SimInstant {
        self.universe.now()
    }

    /// The ownship's true state, in aircraft body axes.
    pub(super) const fn vehicle(&self) -> VehicleState {
        self.truth
    }

    /// The cockpit instruments at the latest step.
    pub(super) fn cockpit(&self) -> CockpitInstrumentSnapshot {
        self.panel.snapshot()
    }

    /// Which tier flies the ownship now.
    pub(super) fn tier(&self) -> Tier {
        self.universe.tier(OWNSHIP).unwrap_or(Tier::Local)
    }

    /// Whether ascent guidance is flying the vessel to its parking orbit.
    pub(super) const fn ascent_guidance(&self) -> bool {
        self.ascent_guidance
    }

    /// Hands the flight to ascent guidance, or to the pilot. A spaceflight
    /// starts under guidance, which lets go by itself in its parking orbit.
    pub(super) const fn set_ascent_guidance(&mut self, engaged: bool) {
        self.ascent_guidance = engaged;
    }

    /// The airframe as the scene draws it: engine alight while thrust is
    /// commanded, everything else neutral.
    pub(super) fn articulation(&self) -> AirframeArticulation {
        let thrusting = self
            .plant()
            .is_some_and(|plant| plant.command().main_accel_m_s2 > 0.0);
        AirframeArticulation {
            engine_running: thrusting,
            ..AirframeArticulation::default()
        }
    }

    /// Aboard eye: at the centre of gravity, looking along the aircraft-axes
    /// forward direction.
    pub(super) fn eye_pose(&self) -> CameraPoseEcef {
        CameraPoseEcef {
            position_ecef: self.truth.position_ecef,
            orientation: self.truth.attitude,
        }
    }

    /// Body aircraft-axes metres from the centre of gravity to the drawn
    /// model's origin, at the base of the hull.
    pub(super) const fn model_origin_from_cg_frd_m() -> DVec3 {
        DVec3::new(0.0, 0.0, RAKETA_CG_Y_M)
    }

    /// Advances one step under `controls`: throttle is main-engine thrust up
    /// to `4 g`, and the stick swings the nose.
    pub(super) fn step(&mut self, controls: ControlInputs) -> Result<(), AppSimError> {
        let controls = controls.clamped();
        let flown = self.ascent_guidance
            || controls.throttle > 0.0
            || controls.pitch != 0.0
            || controls.yaw != 0.0;
        if flown && self.tier() == Tier::Orbital {
            // An orbit about an origin with no body-fixed frame cannot be flown by
            // hand; the vessel stays in orbital propagation there.
            self.universe.demote_to_local(OWNSHIP).ok();
            let ground_height_m = self.ground_height_m;
            if let Some(plant) = self.universe.local_mut::<RaketaPlant>(OWNSHIP) {
                plant.set_ground_height_m(ground_height_m);
            }
        }
        if let Some(state) = self.plant().map(|plant| *plant.inertial_state()) {
            let command = self.command_for(&state, controls);
            if let Some(plant) = self.universe.local_mut::<RaketaPlant>(OWNSHIP) {
                plant.set_command(command);
            }
        }
        let next = self.universe.now().advanced(RAKETA_PLANT_PERIOD);
        self.universe.advance_to(next).map_err(universe_error)?;
        self.universe.take_tier_changes();
        if self
            .universe
            .central_body(OWNSHIP)
            .map_err(universe_error)?
            == NaifId::EARTH
        {
            self.truth = aircraft_axes(
                self.universe
                    .vehicle_state(OWNSHIP)
                    .map_err(universe_error)?,
            );
        }
        self.observe();
        Ok(())
    }

    fn plant(&self) -> Option<&RaketaPlant> {
        self.universe.local::<RaketaPlant>(OWNSHIP)
    }

    fn command_for(
        &mut self,
        state: &ContinuousPlantState,
        controls: ControlInputs,
    ) -> RaketaCommand {
        if self.ascent_guidance {
            let east = DVec3::Z.cross(state.pos_rel_icrf_m).normalize_or_zero();
            let steer = evaluate_gravity_turn_ascent(
                state.pos_rel_icrf_m,
                state.vel_rel_icrf_m_s,
                east,
                &AscentGuidanceConfig::earth_leo_185km(),
            );
            if steer.orbit_achieved {
                self.ascent_guidance = false;
                self.nose_icrf = Some(steer.thrust_dir_icrf);
                return RaketaCommand::default();
            }
            self.nose_icrf = Some(steer.thrust_dir_icrf);
            return RaketaCommand {
                main_accel_m_s2: steer.commanded_accel_m_s2,
                nose_icrf: steer.thrust_dir_icrf,
                ..RaketaCommand::default()
            };
        }
        let forward = state.q_body_to_icrf * DVec3::Z;
        let right = state.q_body_to_icrf * DVec3::NEG_X;
        let nose = self
            .nose_icrf
            .unwrap_or_else(|| state.q_body_to_icrf * DVec3::Y);
        let swing = (right * controls.yaw - forward * controls.pitch)
            * (NOSE_SLEW_RAD_S * RAKETA_PLANT_PERIOD.as_seconds());
        let nose = (nose + swing).normalize_or(nose);
        self.nose_icrf = Some(nose);
        RaketaCommand {
            main_accel_m_s2: controls.throttle * FOUR_G_PROPER_ACCEL_M_S2,
            nose_icrf: nose,
            ..RaketaCommand::default()
        }
    }

    fn observe(&mut self) {
        self.panel.observe_earth(&EarthVehicleSources {
            timestamp: self.universe.now(),
            vehicle: self.truth,
            height_m: ecef_to_geodetic(self.truth.position_ecef).height_m,
        });
    }
}

/// `state`, reported in `Raketa`'s own body axes, as the app's aircraft
/// axes see it.
fn aircraft_axes(state: VehicleState) -> VehicleState {
    VehicleState {
        attitude: (state.attitude * DQuat::from_mat3(&RAKETA_FROM_AIRCRAFT)).normalize(),
        angular_rate_body: RAKETA_FROM_AIRCRAFT.transpose() * state.angular_rate_body,
        ..state
    }
}

fn universe_error(error: UniverseError) -> AppSimError {
    AppSimError::Universe(Box::new(error))
}
