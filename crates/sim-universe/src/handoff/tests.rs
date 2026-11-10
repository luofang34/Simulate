use glam::{DMat3, DQuat, DVec3};
use sim_core::VehicleState;
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_orbital::{BodyOrientationCatalog, BuiltinSolarSystem2026_2028, NaifId, TdbEpoch};
use sim_spacetime::BodyRotationState;

use super::{InertialExtras, body_ellipsoid, naif_body, to_body_fixed, to_inertial};
use crate::UniverseError;

/// `2027-03-01T00:00:00 TDB`, inside the built-in kernels' coverage.
fn epoch() -> TdbEpoch {
    TdbEpoch::new(820_497_600 + 424 * 86_400, 0)
}

fn earth_rotation() -> BodyRotationState {
    let solar = BuiltinSolarSystem2026_2028::load_verified().unwrap();
    solar
        .orientation
        .body_rotation(NaifId::EARTH, epoch())
        .unwrap()
}

fn tumbling_aircraft() -> VehicleState {
    let mut state =
        VehicleState::level_flight(&Geodetic::from_degrees(39.16, -75.52, 3_000.0), 0.7, 120.0);
    state.angular_rate_body = DVec3::new(0.05, -0.02, 0.11);
    state
}

#[test]
fn body_fixed_to_inertial_and_back_is_exact() {
    let rotation = earth_rotation();
    let start = tumbling_aircraft();
    let extras = InertialExtras {
        propellant_mass_kg: 12.5,
        h_rw_body_nms: DVec3::new(1.0, 2.0, 3.0),
    };
    let inertial = to_inertial(&start, NaifId::EARTH, &rotation, extras);
    let (back, back_extras) = to_body_fixed(&inertial, NaifId::EARTH, &rotation).unwrap();
    assert!((back.position_ecef - start.position_ecef).length() < 1e-8);
    assert!((back.velocity_ecef - start.velocity_ecef).length() < 1e-10);
    assert!(back.attitude.dot(start.attitude).abs() > 1.0 - 1e-14);
    assert!((back.angular_rate_body - start.angular_rate_body).length() < 1e-14);
    assert_eq!(back_extras, extras);
}

#[test]
fn a_parked_point_on_the_equator_moves_with_the_earth() {
    let rotation = earth_rotation();
    let parked = VehicleState {
        position_ecef: DVec3::new(6_378_137.0, 0.0, 0.0),
        velocity_ecef: DVec3::ZERO,
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    };
    let inertial = to_inertial(&parked, NaifId::EARTH, &rotation, InertialExtras::default());
    let speed = inertial.vel_rel_icrf_m_s.length();
    assert!(
        (speed - 465.1).abs() < 0.1,
        "equatorial surface speed {speed}"
    );
    let spin = inertial.omega_body_rad_s.length();
    assert!(
        (spin - 7.292_115e-5).abs() < 1e-9,
        "a parked body turns with the Earth: {spin}"
    );
    let rotation_det = DMat3::from_quat(inertial.q_body_to_icrf).determinant();
    assert!((rotation_det - 1.0).abs() < 1e-12);
}

#[test]
fn a_state_about_another_body_is_refused() {
    let rotation = earth_rotation();
    let inertial = to_inertial(
        &tumbling_aircraft(),
        NaifId::MOON,
        &rotation,
        InertialExtras::default(),
    );
    assert!(matches!(
        to_body_fixed(&inertial, NaifId::EARTH, &rotation),
        Err(UniverseError::WrongPrimary { .. })
    ));
}

#[test]
fn modelled_bodies_map_to_their_naif_ids() {
    assert_eq!(naif_body(BodyEllipsoid::WGS84), Some(NaifId::EARTH));
    assert_eq!(naif_body(BodyEllipsoid::MOON_IAU), Some(NaifId::MOON));
    assert_eq!(naif_body(BodyEllipsoid::MARS_IAU), Some(NaifId::MARS));
    for body in [NaifId::EARTH, NaifId::MOON, NaifId::MARS] {
        assert_eq!(body_ellipsoid(body).and_then(naif_body), Some(body));
    }
    assert_eq!(body_ellipsoid(NaifId::SUN), None);
    assert_eq!(body_ellipsoid(NaifId::SSB), None);
}
