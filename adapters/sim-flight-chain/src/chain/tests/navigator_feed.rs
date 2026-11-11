//! The mission flying on a navigator's solution in place of the flight
//! controller's estimate.

use glam::{DMat3, DQuat};
use navigate_contract::{
    AttitudeQuaternion, ClockDomainId, Ellipsoid, FaultDetection, GeodeticPosition,
    IntegrityAssessment, NavigationSolution, NedVelocity, Redundancy, SensorClass, SolutionQuality,
    SolutionStamp, SourceComposition, SourceEpoch, SymmetricCov3, WrappingSequence,
};

use super::*;

/// A stand-in navigator: the flight controller's estimate displaced by
/// `offset_ned_m`, published as a navigation solution.
fn displaced_solution(
    chain: &FlightChain,
    offset_ned_m: DVec3,
    sequence: u32,
) -> NavigationSolution {
    let estimate = chain.vehicle().estimated_state().unwrap();
    let frame = LocalFrame::at(&pad());
    let at = ecef_to_geodetic(estimate.position_ecef + frame.ned_to_ecef() * offset_ned_m);
    let velocity = frame.ecef_to_ned_vector(estimate.velocity_ecef);
    let body_to_ned =
        DQuat::from_mat3(&(frame.ned_to_ecef().transpose() * DMat3::from_quat(estimate.attitude)));
    let elapsed_s = chain.now().duration_since(SimInstant::START).as_seconds();
    let composition = SourceComposition::of(SensorClass::Gnss);
    let mut solution = NavigationSolution::new(
        SolutionStamp::new(
            SourceEpoch::new(0),
            WrappingSequence::new(sequence),
            MonotonicNanos::from_nanos((elapsed_s * NANOS_PER_SECOND).round() as u64),
            ClockDomainId::new(1),
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
    solution
}

/// Flies `seconds` on the stand-in navigator and returns the true position
/// from the pad, north-east-down.
fn fly_on_navigator(offset_ned_m: DVec3, seconds: u32) -> DVec3 {
    let mut setup = setup();
    setup.mission.ownship = OwnshipFeed::Navigator;
    let mut chain = FlightChain::new(&setup).unwrap();
    for frame in 0..seconds * MISSION_FRAME_HZ as u32 {
        if chain.vehicle().estimated_state().is_some() {
            let solution = displaced_solution(&chain, offset_ned_m, frame);
            chain.offer_navigation(&solution);
        }
        chain.advance_frame().unwrap();
    }
    assert_eq!(chain.mission_state(), MissionState::Enroute);
    ned(&chain.true_state())
}

#[test]
fn the_true_track_moves_against_an_offset_in_the_navigators_solution() {
    // Well along the first leg, which runs north from the pad.
    let seconds = 80;
    let honest = fly_on_navigator(DVec3::ZERO, seconds);
    assert!(honest.x > 250.0, "on the first leg: {honest:?}");
    assert!(honest.y.abs() < 2.0, "on the leg line: {honest:?}");
    assert!(
        (honest.z + CRUISE_HEIGHT_M).abs() < 2.0,
        "at height: {honest:?}"
    );

    // The navigator reports the vehicle 30 m east of and 10 m above where it
    // is, so the vehicle flies 30 m west of and 10 m below the route.
    let offset = DVec3::new(0.0, 30.0, -10.0);
    let displaced = fly_on_navigator(offset, seconds);
    let moved = displaced - honest;
    assert!((moved.y + offset.y).abs() < 2.0, "moved {moved:?}");
    assert!((moved.z + offset.z).abs() < 2.0, "moved {moved:?}");
}

#[test]
fn a_mission_on_the_navigator_ignores_the_flight_controllers_estimate() {
    let mut setup = setup();
    setup.mission.ownship = OwnshipFeed::Navigator;
    let mut chain = FlightChain::new(&setup).unwrap();
    for _ in 0..10 * MISSION_FRAME_HZ {
        let report = chain.advance_frame().unwrap();
        assert_eq!(report.state, MissionState::AwaitSolution);
        assert_eq!(report.intent, None);
    }
    assert!(chain.vehicle().is_on_pad());
    assert!(chain.vehicle().estimated_state().is_some());
}
