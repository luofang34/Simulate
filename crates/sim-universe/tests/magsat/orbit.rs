//! The orbit the magsat tests fly and the universe they fly it in: 500 km
//! circular at 60° inclination from 2027-03-01, on the built-in solar system
//! and its body catalog.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_devices::MagnetometerSpec;
use sim_geo::{BodyEllipsoid, UtcTime};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContinuousPlantState, Dop853Tolerances,
    ForceModel, NaifId, VesselId,
};
use sim_spacetime::SimEpoch;
use sim_universe::{LocalFactory, OrbitalContext, Universe, to_body_fixed};

use super::satellite::{Magsat, MagsatSpec};

/// `2027-03-01T00:00:00Z`.
const START_UTC_UNIX_S: f64 = 1_803_859_200.0;
pub const ORBIT_RADIUS_M: f64 = 6_878_137.0;
const INCLINATION_DEG: f64 = 60.0;

/// One revolution, s.
pub fn orbit_period_s() -> f64 {
    std::f64::consts::TAU * (ORBIT_RADIUS_M.powi(3) / BodyEllipsoid::WGS84.mu_m3_s2).sqrt()
}

/// `ms` milliseconds.
pub const fn millis(ms: u64) -> TickSpan {
    TickSpan::from_ticks(ms * (TICKS_PER_SECOND / 1_000))
}

/// The instant `ms` milliseconds after the start.
pub const fn at_millis(ms: u64) -> SimInstant {
    SimInstant::START.advanced(millis(ms))
}

#[cfg(test)]
pub fn epoch() -> SimEpoch {
    SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(START_UTC_UNIX_S)).unwrap()
}

/// A universe on the built-in solar system, and the orientation catalog it
/// hands vessels over with.
pub struct World {
    pub universe: Universe,
    orientation: Arc<dyn BodyOrientationCatalog>,
}

/// An empty world whose orbital tier steps on `quantum` with `workers`
/// speculation threads.
#[cfg(test)]
pub fn world(quantum: TickSpan, workers: usize) -> World {
    let solar = BuiltinSolarSystem2026_2028::load_verified().unwrap();
    let force_model = ForceModel::standard_solar_system(solar.constants.clone()).unwrap();
    let orientation: Arc<dyn BodyOrientationCatalog> = Arc::new(solar.orientation);
    let context = OrbitalContext {
        ephemeris: Arc::new(solar.ephemeris),
        orientation: Arc::clone(&orientation),
        force_model,
        quantum,
        tolerances: Dop853Tolerances {
            rtol: 1e-12,
            atol_pos_m: 1e-6,
            atol_vel_m_s: 1e-9,
            ..Dop853Tolerances::default()
        },
    };
    World {
        universe: Universe::new(epoch(), context).with_orbital_workers(workers),
        orientation,
    }
}

/// A magnetometer with the errors of a small-satellite unit: 20 nT of white
/// noise, half a microtesla of hard iron, half a percent of gain error.
pub const FLIGHT_MAGNETOMETER: MagnetometerSpec = MagnetometerSpec {
    white_sigma_ut: 0.02,
    hard_iron_sigma_ut: 0.5,
    scale_factor_sigma: 0.005,
};

/// The orbit's state at its ascending node, `phase_rad` along, tumbling at
/// `rate_body_rad_s` in inertial space.
#[cfg(test)]
pub fn start_state(world: &World, phase_rad: f64, rate_body_rad_s: DVec3) -> VehicleState {
    let universe = &world.universe;
    let inclination = INCLINATION_DEG.to_radians();
    let node = DVec3::X;
    let across = DVec3::new(0.0, inclination.cos(), inclination.sin());
    let radial = node * phase_rad.cos() + across * phase_rad.sin();
    let along = across * phase_rad.cos() - node * phase_rad.sin();
    let speed = (BodyEllipsoid::WGS84.mu_m3_s2 / ORBIT_RADIUS_M).sqrt();
    let inertial = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: radial * ORBIT_RADIUS_M,
        vel_rel_icrf_m_s: along * speed,
        q_body_to_icrf: DQuat::from_euler(glam::EulerRot::ZYX, 0.3, -0.2, 0.5),
        omega_body_rad_s: rate_body_rad_s,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let rotation = world
        .orientation
        .body_rotation(NaifId::EARTH, universe.epoch().tdb_at(universe.now()))
        .unwrap();
    to_body_fixed(&inertial, NaifId::EARTH, &rotation)
        .unwrap()
        .0
}

/// A factory that builds the satellite of `spec` on the body it comes down
/// to.
#[cfg(test)]
pub fn factory(spec: MagsatSpec) -> LocalFactory {
    Box::new(move |body, state, start, epoch| {
        Ok(Box::new(Magsat::new(body, spec, state, start, epoch)?))
    })
}

/// Adds the satellite of `spec` as `id`, tumbling at `rate_body_rad_s`
/// from `phase_rad` along the orbit, and hands it to the orbital tier with
/// its plant aboard.
#[cfg(test)]
pub fn launch(
    world: &mut World,
    id: VesselId,
    spec: MagsatSpec,
    phase_rad: f64,
    rate_body_rad_s: DVec3,
) {
    let state = start_state(world, phase_rad, rate_body_rad_s);
    let universe = &mut world.universe;
    let earth = universe.bodies().get(NaifId::EARTH).unwrap().clone();
    let plant = Magsat::new(&earth, spec, &state, universe.now(), universe.epoch()).unwrap();
    universe
        .insert_local(id, Box::new(plant), factory(spec))
        .unwrap();
    universe.promote_to_orbital(id).unwrap();
}

/// The carried satellite `id`.
#[cfg(test)]
pub fn magsat(universe: &Universe, id: VesselId) -> &Magsat {
    universe.carried::<Magsat>(id).unwrap()
}
