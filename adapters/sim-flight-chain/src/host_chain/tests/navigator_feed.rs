//! The mission flying on an onboard navigator's solution in place of the
//! flight controller's estimate.

use glam::{DMat3, DQuat, DVec3};
use navigate_contract::{
    AttitudeQuaternion, Ellipsoid, FaultDetection, GeodeticPosition, IntegrityAssessment,
    NedVelocity, Redundancy, SensorClass, SolutionQuality, SolutionStamp, SourceComposition,
    SourceEpoch, SymmetricCov3, WrappingSequence,
};
use sim_geo::{LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use super::*;

/// A stand-in navigator for the test: the flight controller's estimate
/// displaced by `offset_ned_m`, published as a navigation solution.
fn displaced_solution(
    chain: &HostChain<C172Airframe, FixedWingController, FixedWingMixer>,
    offset_ned_m: DVec3,
    sequence: u32,
) -> Option<NavigationSolution> {
    let estimate = chain.board().estimated_state()?;
    let frame = LocalFrame::at(&ecef_to_geodetic(estimate.position_ecef));
    let at = ecef_to_geodetic(estimate.position_ecef + frame.ned_to_ecef() * offset_ned_m);
    let velocity = frame.ecef_to_ned_vector(estimate.velocity_ecef);
    let body_to_ned =
        DQuat::from_mat3(&(frame.ned_to_ecef().transpose() * DMat3::from_quat(estimate.attitude)));
    let composition = SourceComposition::of(SensorClass::Gnss);
    let mut solution = NavigationSolution::new(
        SolutionStamp::new(
            SourceEpoch::new(0),
            WrappingSequence::new(sequence),
            chain.host_now(),
            MISSION_CLOCK,
        ),
        GeodeticPosition::new(at.lat_rad, at.lon_rad, at.height_m),
        NedVelocity::new(velocity.x, velocity.y, velocity.z),
        SymmetricCov3::from_diagonal(1.0, 1.0, 1.0),
        SymmetricCov3::from_diagonal(0.1, 0.1, 0.1),
        IntegrityAssessment::new(
            SolutionQuality::Good,
            composition,
            Redundancy::None,
            1.0,
            1.0,
            FaultDetection::Unavailable,
        ),
        composition,
        Ellipsoid::WGS84,
    );
    solution.attitude = Some(AttitudeQuaternion::new(
        body_to_ned.w,
        body_to_ned.x,
        body_to_ned.y,
        body_to_ned.z,
    ));
    Some(solution)
}

fn on_navigator() -> HostChain<C172Airframe, FixedWingController, FixedWingMixer> {
    let mut route = route(PLAN_LOADED_AT_S);
    route.ownship = OwnshipFeed::Navigator;
    HostChain::new(c172(1), &route).unwrap()
}

/// Flies `seconds` on the stand-in navigator, a solution every 40 ms, and
/// returns the true position from the start, north-east-down.
fn fly_on_navigator(offset_ned_m: DVec3, seconds: f64) -> DVec3 {
    let mut chain = on_navigator();
    let mut sequence = 0_u32;
    for cycle in 0..(seconds * C172_RATE_HZ as f64) as u64 {
        if cycle.is_multiple_of(C172_RATE_HZ / 25)
            && let Some(solution) = displaced_solution(&chain, offset_ned_m, sequence)
        {
            sequence = sequence.wrapping_add(1);
            chain.offer_navigation(&solution);
        }
        chain.advance_tick().unwrap();
    }
    assert_eq!(chain.status().mission_state, Some(MissionState::Enroute));
    LocalFrame::at(&start())
        .ecef_to_ned_vector(chain.true_state().position_ecef - geodetic_to_ecef(&start()))
}

#[test]
fn the_true_track_moves_against_an_offset_in_the_navigators_solution() {
    // Well along the first long leg, which runs east.
    let seconds = 150.0;
    let honest = fly_on_navigator(DVec3::ZERO, seconds);
    // The navigator reports the aeroplane 30 m north of and 10 m above where
    // it is, so it flies 30 m south of the route. Pilotage asks a fixed wing
    // for a heading and a height, and the flight controller holds the height
    // on its own air data, so the navigator's height error moves nothing.
    let offset = DVec3::new(30.0, 0.0, -10.0);
    let displaced = fly_on_navigator(offset, seconds);
    let moved = displaced - honest;
    tracing::info!(?honest, ?displaced, ?moved, "track against the offset");
    assert!((moved.x + offset.x).abs() < 5.0, "moved {moved:?}");
    assert!(moved.z.abs() < 3.0, "moved {moved:?}");
}

#[test]
fn a_mission_on_the_navigator_ignores_the_flight_controllers_estimate() {
    let mut chain = on_navigator();
    for _ in 0..10 * C172_RATE_HZ {
        chain.advance_tick().unwrap();
    }
    assert_eq!(
        chain.status().mission_state,
        Some(MissionState::AwaitSolution)
    );
    assert!(!chain.status().armed);
    assert!(chain.board().estimated_state().is_some());
}
