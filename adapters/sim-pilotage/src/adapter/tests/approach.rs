use pilotage_protocol::LandedState;

use super::*;

/// The start's ellipsoidal height, m.
const START_HEIGHT_M: f64 = 900.0;

#[test]
fn without_a_surface_there_is_no_ranging_or_landed_report() {
    let mut adapter = kinematic();
    adapter.step(StepBudget { ticks: 10 });
    let fc = sample(&mut adapter).fc_state.unwrap();
    assert!(fc.landing_approach.is_none());
    assert!(fc.landed_state.is_none());
}

#[test]
fn over_a_surface_ranging_reports_height_and_descent() {
    let mut adapter = kinematic();
    adapter.set_surface_elevation_m(Some(START_HEIGHT_M - 50.0));
    adapter.apply_control(&command(body_rate(0.0, -0.1, 0.0, 0.5)));
    adapter.step(StepBudget { ticks: 120 });
    let telemetry = sample(&mut adapter);
    let fc = telemetry.fc_state.unwrap();
    let approach = fc.landing_approach.unwrap();
    let truth = telemetry.sim_truth.unwrap();

    let height = ecef_to_geodetic(adapter.vehicle_state().position_ecef).height_m;
    let expected = (height - (START_HEIGHT_M - 50.0)) as f32;
    assert!((approach.surface_distance_m - expected).abs() < 1e-3);
    assert!(approach.surface_distance_m < 50.0);
    assert!(approach.vertical_speed_down_mps > 1.0, "{approach:?}");
    assert!((approach.vertical_speed_down_mps - truth.vel_ned_mps[2]).abs() < 1e-3);
    assert!(approach.is_flying);
    assert_eq!(fc.landed_state.unwrap().state, LandedState::InAir);
    assert_eq!(approach.stamp.sequence, truth.stamp.sequence);

    adapter.set_surface_elevation_m(None);
    adapter.step(StepBudget { ticks: 1 });
    assert!(
        sample(&mut adapter)
            .fc_state
            .unwrap()
            .landing_approach
            .is_none()
    );
}

#[test]
fn a_surface_above_the_vehicle_ranges_zero_and_speed_still_says_flying() {
    let mut adapter = kinematic();
    adapter.set_surface_elevation_m(Some(START_HEIGHT_M + 10.0));
    adapter.step(StepBudget { ticks: 1 });
    let approach = sample(&mut adapter)
        .fc_state
        .unwrap()
        .landing_approach
        .unwrap();
    assert_eq!(approach.surface_distance_m, 0.0);
    assert!(approach.is_flying);
}

#[test]
fn at_rest_on_the_surface_the_vehicle_is_on_the_ground() {
    let state = VehicleState::level_flight(&start(), PI / 2.0, 0.0);
    let mut adapter = SimulateVehicleAdapter::new(CONFIG, KinematicModel::new(state));
    adapter.set_surface_elevation_m(Some(START_HEIGHT_M));
    let at = adapter.now();
    let checkpoint = adapter.checkpoint();
    adapter.restore_checkpoint(&checkpoint, at);
    let fc = sample(&mut adapter).fc_state.unwrap();
    let approach = fc.landing_approach.unwrap();
    assert!(approach.surface_distance_m < 0.35, "{approach:?}");
    assert!(!approach.is_flying);
    assert_eq!(fc.landed_state.unwrap().state, LandedState::OnGround);
}
