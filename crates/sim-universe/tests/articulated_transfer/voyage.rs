//! The voyage the articulated spacecraft flies: the universe it is in, what
//! the test reads of it, and the Sun-centred Lambert targeting and finite
//! burns that take it from a low Earth orbit to Mars.

use std::cell::Cell;
use std::rc::Rc;
use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_core::{Block, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_geo::UtcTime;
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContinuousPlantState, DerivativeOrder,
    Dop853Tolerances, EphemerisCatalog, ForceModel, LatchedBurnLaw, NaifId, PlayerCommandPayload,
    TdbEpoch, VesselId, propagate_kepler_universal, solve_lambert_universal,
};
use sim_spacetime::SimEpoch;
use sim_universe::{MultibodyLocalPlant, OrbitalContext, Universe, UniverseError, to_body_fixed};

use super::spacecraft::{ANTENNA, BUS, TANK, plant};

pub const PROBE: VesselId = VesselId(11);
/// `2026-10-18T00:00:00Z`, inside the 2026 Earth-to-Mars window.
const START_UTC_UNIX_S: f64 = 1_792_281_600.0;
const PARKING_RADIUS_M: f64 = 6_778_137.0;
/// Acceleration of the transfer stage's burns, m/s².
const BURN_ACCEL_M_S2: f64 = 20.0;
pub const DAY_S: u64 = 86_400;

pub const fn at_s(seconds: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(seconds * TICKS_PER_SECOND))
}

/// The universe, the catalogs it reads, and how often its factory ran.
pub struct Voyage {
    pub universe: Universe,
    ephemeris: Arc<dyn EphemerisCatalog>,
    pub mu_sun: f64,
    pub mu_mars: f64,
    pub rebuilds: Rc<Cell<u32>>,
}

/// Where and how a voyage starts: the spacecraft's position and velocity
/// from the Earth's centre and the inertial rate of its body axes, all in
/// ICRF axes, which its body axes start aligned with.
pub struct Placed {
    pub position_from_earth_m: DVec3,
    pub velocity_from_earth_m_s: DVec3,
    pub rate_body_rad_s: DVec3,
}

/// What the test follows of the plant's internals.
#[derive(Debug, Clone, PartialEq)]
pub struct Internals {
    pub gimbal_rad: f64,
    pub gimbal_rate_rad_s: f64,
    pub xenon_kg: f64,
    pub battery_wh: f64,
    pub temperature_k: f64,
    /// Every bit of the block, printed.
    pub block: String,
    /// Where the plant lives; it never moves while it is the same plant.
    pub address: usize,
}

impl Voyage {
    /// The spacecraft in a 400 km parking orbit at 28.5°, in the local tier.
    #[cfg(test)]
    pub fn begin() -> Self {
        // A voyage of months at a centimetre a step: the trims take out
        // more than this leaves.
        let tolerances = Dop853Tolerances {
            rtol: 1.0e-9,
            atol_pos_m: 1.0e-2,
            atol_vel_m_s: 1.0e-6,
            atol_att_rad: 1.0e-6,
            atol_omega_rad_s: 1.0e-8,
            ..Dop853Tolerances::default()
        };
        Self::placed(tolerances, |_, _, mu_earth, _| {
            let inclination = 28.5_f64.to_radians();
            Placed {
                position_from_earth_m: DVec3::X * PARKING_RADIUS_M,
                velocity_from_earth_m_s: DVec3::new(0.0, inclination.cos(), inclination.sin())
                    * (mu_earth / PARKING_RADIUS_M).sqrt(),
                rate_body_rad_s: DVec3::ZERO,
            }
        })
    }

    /// The spacecraft in the local tier of the Earth at the state `place`
    /// gives it from the ephemeris, the starting epoch, the Earth's
    /// gravitational parameter and Mars's, with the orbital tier
    /// integrating to `tolerances`.
    #[cfg(test)]
    pub fn placed(
        tolerances: Dop853Tolerances,
        place: impl FnOnce(&dyn EphemerisCatalog, TdbEpoch, f64, f64) -> Placed,
    ) -> Self {
        let solar = BuiltinSolarSystem2026_2028::load_verified().unwrap();
        let force_model = ForceModel::standard_solar_system(solar.constants.clone()).unwrap();
        let mu = |body| force_model.constants().require_mu(body).unwrap();
        let (mu_earth, mu_sun, mu_mars) = (mu(NaifId::EARTH), mu(NaifId::SUN), mu(NaifId::MARS));
        let ephemeris: Arc<dyn EphemerisCatalog> = Arc::new(solar.ephemeris);
        let orientation: Arc<dyn BodyOrientationCatalog> = Arc::new(solar.orientation);
        let epoch =
            SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(START_UTC_UNIX_S)).unwrap();
        let context = OrbitalContext {
            ephemeris: Arc::clone(&ephemeris),
            orientation: Arc::clone(&orientation),
            force_model,
            quantum: TickSpan::from_ticks(600 * TICKS_PER_SECOND),
            tolerances,
        };
        let mut universe = Universe::new(epoch.clone(), context);
        let start = epoch.tdb_at(SimInstant::START);
        let placed = place(ephemeris.as_ref(), start, mu_earth, mu_mars);
        let inertial = ContinuousPlantState {
            primary_body: NaifId::EARTH,
            pos_rel_icrf_m: placed.position_from_earth_m,
            vel_rel_icrf_m_s: placed.velocity_from_earth_m_s,
            q_body_to_icrf: DQuat::IDENTITY,
            omega_body_rad_s: placed.rate_body_rad_s,
            propellant_mass_kg: 0.0,
            h_rw_body_nms: DVec3::ZERO,
        };
        let rotation = orientation.body_rotation(NaifId::EARTH, start).unwrap();
        let state: VehicleState = to_body_fixed(&inertial, NaifId::EARTH, &rotation)
            .unwrap()
            .0;
        let earth = universe.bodies().get(NaifId::EARTH).unwrap().clone();
        let ship = plant(&earth, &state, SimInstant::START, &epoch).unwrap();
        let rebuilds = Rc::new(Cell::new(0_u32));
        let counted = Rc::clone(&rebuilds);
        universe
            .insert_local(
                PROBE,
                Box::new(ship),
                Box::new(move |_, _, _, _| {
                    counted.set(counted.get().wrapping_add(1));
                    Err(UniverseError::NotCarryable)
                }),
            )
            .unwrap();
        Self {
            universe,
            ephemeris,
            mu_sun,
            mu_mars,
            rebuilds,
        }
    }

    /// The plant, whichever tier holds it.
    #[cfg(test)]
    pub fn ship(&self) -> &MultibodyLocalPlant {
        self.universe
            .carried::<MultibodyLocalPlant>(PROBE)
            .or_else(|| self.universe.local::<MultibodyLocalPlant>(PROBE))
            .unwrap()
    }

    /// The plant, to command it.
    #[cfg(test)]
    pub fn ship_mut(&mut self) -> &mut MultibodyLocalPlant {
        if self
            .universe
            .carried::<MultibodyLocalPlant>(PROBE)
            .is_some()
        {
            return self.universe.carried_mut(PROBE).unwrap();
        }
        self.universe.local_mut(PROBE).unwrap()
    }

    #[cfg(test)]
    pub fn internals(&self) -> Internals {
        let ship = self.ship();
        let block = ship.block();
        let gimbal = block.tree().state(ANTENNA).unwrap().joint;
        let subsystems = block.subsystems();
        Internals {
            gimbal_rad: gimbal.q_scalar,
            gimbal_rate_rad_s: gimbal.q_vel_scalar,
            xenon_kg: subsystems.fluids.tank_state(TANK).unwrap().mass_kg,
            battery_wh: subsystems.electrical.batteries()[0].1.energy_wh,
            temperature_k: subsystems.thermal.temperature_k(BUS).unwrap(),
            block: format!("{:?}", block.snapshot()),
            address: std::ptr::from_ref(ship) as usize,
        }
    }

    #[cfg(test)]
    pub fn tdb(&self) -> TdbEpoch {
        self.universe.epoch().tdb_at(self.universe.now())
    }

    /// `body` about the Sun at `at`.
    #[cfg(test)]
    pub fn about_sun(&self, body: NaifId, at: TdbEpoch) -> (DVec3, DVec3) {
        let state = self
            .ephemeris
            .geometric_state_icrf(body, NaifId::SUN, at, DerivativeOrder::Velocity)
            .unwrap();
        (state.pos_m, state.vel_m_s)
    }

    /// The vessel about `origin` now, whatever the scheduler integrates it
    /// about.
    #[cfg(test)]
    pub fn vessel_about(&self, origin: NaifId) -> (DVec3, DVec3) {
        let vessel = self
            .universe
            .orbital()
            .sample_vessel_barycentric(PROBE, self.universe.now())
            .unwrap();
        let origin = self
            .ephemeris
            .geometric_state_icrf(origin, NaifId::SSB, self.tdb(), DerivativeOrder::Velocity)
            .unwrap();
        (vessel.pos_m - origin.pos_m, vessel.vel_m_s - origin.vel_m_s)
    }

    /// The velocity change, now, that puts the vessel on the Sun-centred
    /// conic reaching Mars's centre at `arrival`: the Lambert branch nearer
    /// the velocity it has.
    #[cfg(test)]
    pub fn correction_to_mars(&self, arrival: TdbEpoch) -> Option<DVec3> {
        let (position, velocity) = self.vessel_about(NaifId::SUN);
        let aim = self.about_sun(NaifId::MARS, arrival).0;
        let flight_s = arrival.signed_offset_seconds_from(self.tdb());
        [false, true]
            .into_iter()
            .filter_map(|long_way| {
                solve_lambert_universal(position, aim, flight_s, self.mu_sun, long_way).ok()
            })
            .map(|solution| solution.v1_m_s - velocity)
            .min_by(|a, b| a.length().total_cmp(&b.length()))
    }

    /// Burns `delta_v` in inertial axes from now at the stage's acceleration
    /// and returns the instant the burn ends.
    #[cfg(test)]
    pub fn burn(&mut self, delta_v: DVec3) -> SimInstant {
        let now = self.universe.now();
        let mass_kg = self.ship().block().mass_properties().total_mass_kg;
        let burn_ms = (delta_v.length() / BURN_ACCEL_M_S2 * 1_000.0).round() as u64;
        let end = now.advanced(TickSpan::from_ticks(burn_ms * (TICKS_PER_SECOND / 1_000)));
        let thrust = LatchedBurnLaw::InertialThrust {
            dir_icrf: delta_v.normalize().to_array(),
            thrust_n: mass_kg * BURN_ACCEL_M_S2,
            exhaust_vel_m_s: 3_000.0,
            rw_torque_cmd_body_nm: [0.0; 3],
        };
        let set = PlayerCommandPayload::SetBurnLaw;
        self.universe.command(PROBE, now, set(thrust)).unwrap();
        self.universe
            .command(PROBE, end, set(LatchedBurnLaw::Coast))
            .unwrap();
        end
    }

    /// Where the Sun-centred conic through (`position`, `velocity`) is
    /// `after_s` later.
    #[cfg(test)]
    pub fn conic(&self, position: DVec3, velocity: DVec3, after_s: f64) -> DVec3 {
        propagate_kepler_universal(position, velocity, after_s, self.mu_sun).0
    }
}
