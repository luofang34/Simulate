//! A body registered in the catalog exists for both tiers and everything
//! between them.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_bodies::{BodyCatalog, BodyDescriptor};
use sim_core::{SimInstant, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_orbital::{BuiltinSolarSystem2026_2028, Dop853Tolerances, ForceModel, NaifId, VesselId};

use super::{QUANTUM, coaster, epoch, factory, seconds};
use crate::universe::{OrbitalContext, Tier, Universe};

/// Venus as a point mass with no air: nothing but this descriptor says it
/// exists.
fn venus() -> BodyDescriptor {
    let ellipsoid = BodyEllipsoid {
        semi_major_m: 6_051_800.0,
        flattening: 0.0,
        mu_m3_s2: 3.248_585_92e14,
        omega_rad_s: -2.992_4e-7,
        j2: 0.0,
    };
    BodyDescriptor::new(NaifId::VENUS, "Venus", ellipsoid)
        .unwrap()
        .with_sphere_of_influence(616_000_000.0, NaifId::SSB)
        .with_regime(250_000.0, 200_000.0)
}

/// A circular orbit `height_m` above `body`, in its rotating frame.
fn circular_orbit(body: &BodyEllipsoid, height_m: f64) -> VehicleState {
    let r = body.semi_major_m + height_m;
    let speed = (body.mu_m3_s2 / r).sqrt();
    let position = DVec3::new(r, 0.0, 0.0);
    let spin = DVec3::Z * body.omega_rad_s;
    VehicleState {
        position_ecef: position,
        velocity_ecef: DVec3::new(0.0, 0.6, 0.8) * speed - spin.cross(position),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    }
}

#[test]
fn a_fourth_body_needs_only_its_descriptor() {
    let bodies = Arc::new(BodyCatalog::standard_solar_system().unwrap().with(venus()));
    let body = bodies.get(NaifId::VENUS).unwrap().ellipsoid;
    let solar = BuiltinSolarSystem2026_2028::load_verified_with_bodies(bodies).unwrap();
    let force_model = ForceModel::standard_solar_system(solar.constants.clone()).unwrap();
    // It attracts in its own right, in place of its system's barycentre.
    assert!(force_model.attracting_bodies().contains(&NaifId::VENUS));
    let barycentre = NaifId::VENUS_BARYCENTER;
    assert!(!force_model.attracting_bodies().contains(&barycentre));
    let context = OrbitalContext {
        ephemeris: Arc::new(solar.ephemeris),
        orientation: Arc::new(solar.orientation),
        force_model,
        quantum: QUANTUM,
        tolerances: Dop853Tolerances::default(),
    };
    let epoch = epoch();
    let mut universe = Universe::new(epoch.clone(), context).with_regime_tier_policy();
    let start = circular_orbit(&body, 400_000.0);
    let plant = coaster(body, &start, SimInstant::START, &epoch).unwrap();
    universe
        .insert_local(VesselId(1), plant, factory())
        .unwrap();

    // Its regime limits hand the vessel to orbital propagation about it.
    universe.advance_to(seconds(5)).unwrap();
    assert_eq!(universe.tier(VesselId(1)).unwrap(), Tier::Orbital);
    assert_eq!(universe.central_body(VesselId(1)).unwrap(), NaifId::VENUS);

    // Half a revolution on, its frame and gravity still describe the orbit.
    universe.advance_to(seconds(2_860)).unwrap();
    let state = universe.vehicle_state(VesselId(1)).unwrap();
    let height_m = body.fixed_to_geodetic(state.position_ecef).height_m;
    assert!((height_m - 400_000.0).abs() < 2_000.0, "{height_m}");
    assert!(state.position_ecef.x < -6.0e6, "{}", state.position_ecef);

    // And flight dynamics take it back about the same body.
    universe.demote_to_local(VesselId(1)).unwrap();
    assert_eq!(universe.tier(VesselId(1)).unwrap(), Tier::Local);
    assert_eq!(universe.central_body(VesselId(1)).unwrap(), NaifId::VENUS);
    let hides_the_sun = |naif| {
        let sunlight = universe.bodies().solar_flux_model();
        sunlight.occulters.iter().any(|sphere| sphere.body == naif)
    };
    assert!(hides_the_sun(NaifId::VENUS));
}
