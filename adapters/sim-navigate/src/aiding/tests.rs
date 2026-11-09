use navigate_contract::ClockDomainId;
use sim_core::VehicleState;
use sim_geo::{Geodetic, geodetic_to_ecef};

use super::*;

#[test]
fn a_northbound_climb_reads_north_and_negative_down() {
    let position = Geodetic::from_degrees(39.0, -75.0, 1_000.0);
    let frame = LocalFrame::at(&position);
    let velocity = frame.north() * 50.0 + frame.up() * 3.0;
    let mut aiding = KinematicAiding::new(
        MasterClockBridge::new(ClockDomainId::new(1)),
        SourceId::new(2),
        SourceEpoch::new(0),
        0.5,
    );
    let observation = aiding
        .observe(
            SimInstant::START,
            &Estimate::new(geodetic_to_ecef(&position)),
            velocity,
        )
        .unwrap();
    let ObservationValue::VelocityFix {
        velocity,
        covariance,
    } = observation.value
    else {
        panic!("{observation:?}");
    };
    assert!((velocity.north_mps - 50.0).abs() < 1e-9);
    assert!(velocity.east_mps.abs() < 1e-9);
    assert!((velocity.down_mps + 3.0).abs() < 1e-9);
    assert_eq!(covariance.diagonal(), (0.25, 0.25, 0.25));
    assert!(observation.composition.contains(SensorClass::Inertial));
    assert!(!observation.composition.is_estimator_derived());
}

#[test]
fn each_observation_takes_the_next_sequence() {
    let state = VehicleState::level_flight(&Geodetic::from_degrees(39.0, -75.0, 900.0), 1.0, 55.0);
    let mut aiding = KinematicAiding::new(
        MasterClockBridge::default(),
        SourceId::new(2),
        SourceEpoch::new(0),
        0.5,
    );
    let first = aiding
        .observe(
            SimInstant::START,
            &Estimate::new(state.position_ecef),
            state.velocity_ecef,
        )
        .unwrap();
    let second = aiding
        .observe(
            SimInstant::START,
            &Estimate::new(state.position_ecef),
            state.velocity_ecef,
        )
        .unwrap();
    assert!(first.stamp.sequence.admits(second.stamp.sequence));
}
