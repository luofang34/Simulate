use sim_core::SourceValidity;

use super::*;

fn fly(profile: ScenarioProfile, dynamics: FlightDynamics) -> CorridorRun {
    let mut run = CorridorRun::new(profile, dynamics).unwrap();
    while !run.finished() {
        run.step().unwrap();
    }
    run
}

const EACH_DYNAMICS: [FlightDynamics; 2] = [FlightDynamics::Kinematic, FlightDynamics::Jsbsim];

/// Exact bits, because a replay that merely lands close has diverged.
fn bits(state: &VehicleState) -> [u64; 7] {
    let [px, py, pz] = state.position_ecef.to_array();
    let [qx, qy, qz, qw] = state.attitude.to_array();
    [px, py, pz, qx, qy, qz, qw].map(f64::to_bits)
}

#[test]
fn the_nominal_cruise_earns_independent_anchors_across_many_cells() {
    for dynamics in EACH_DYNAMICS {
        let run = fly(ScenarioProfile::CorridorNominalCruise, dynamics);
        let report = run.report();

        assert!(report.frames >= 3_590, "{dynamics:?}: {}", report.frames);
        assert!(
            report.nav_tiles_crossed.len() >= 8,
            "{dynamics:?}: {report:?}"
        );
        assert!(report.anchor_cells.len() >= 8, "{dynamics:?}: {report:?}");
        assert_eq!(
            report.independent_anchor_fixes,
            report.anchor_cells.len() as u64,
            "{dynamics:?}: one independent fix per cell"
        );
        assert!(report.shared_map_cell_fixes > report.independent_anchor_fixes);
        assert!(
            report.warm_fixes >= 3_590,
            "{dynamics:?}: {}",
            report.warm_fixes
        );
        assert!(
            report.height_error_m.abs() < 25.0,
            "{dynamics:?}: {report:?}"
        );
        assert!(
            report.peak_bank_rad > 0.1,
            "{dynamics:?}: it banks: {report:?}"
        );
        assert!(
            report.peak_bank_rad < 0.7,
            "{dynamics:?}: gently: {report:?}"
        );
    }
}

#[test]
fn the_off_seed_start_converges_onto_the_corridor() {
    for dynamics in EACH_DYNAMICS {
        let mut run =
            CorridorRun::new(ScenarioProfile::CorridorOffSeedAcquisition, dynamics).unwrap();
        run.step().unwrap();
        let first = run.report().clone();
        assert!(
            (first.cross_track_m - OFF_SEED_LATERAL_M).abs() < 5.0,
            "{first:?}"
        );
        assert!(
            (first.height_error_m - OFF_SEED_HEIGHT_M).abs() < 5.0,
            "{first:?}"
        );

        while !run.finished() {
            run.step().unwrap();
        }
        let last = run.report();
        assert!(last.cross_track_m.abs() < 40.0, "{dynamics:?}: {last:?}");
        assert!(last.height_error_m.abs() < 15.0, "{dynamics:?}: {last:?}");
    }
}

#[test]
fn the_aggressive_profile_loses_the_horizon_and_keeps_the_camera_on_truth() {
    for dynamics in EACH_DYNAMICS {
        let mut run = CorridorRun::new(ScenarioProfile::CorridorAggressiveBank, dynamics).unwrap();
        let mut worst_rate_error: f64 = 0.0;
        let mut rate_error_sum = 0.0;
        let mut steps = 0.0;
        while !run.finished() {
            let before = run.vehicle();
            run.step().unwrap();
            let after = run.vehicle();
            // `q` and `-q` are the same attitude, so take the short way round.
            let turned = before.attitude.inverse() * after.attitude;
            let turned = if turned.w < 0.0 { -turned } else { turned };
            let measured = turned.to_scaled_axis() / PERIOD.as_seconds();
            // The kinematic model turns at its new rate for the whole step;
            // JSBSim's rate moves through the step, so it takes the mean.
            let rate = match dynamics {
                FlightDynamics::Kinematic => after.angular_rate_body,
                FlightDynamics::Jsbsim => {
                    (before.angular_rate_body + after.angular_rate_body) / 2.0
                }
            };
            let error = (measured - rate).length();
            worst_rate_error = worst_rate_error.max(error);
            rate_error_sum += error;
            steps += 1.0;

            // The camera holds to the airframe through every manoeuvre.
            let capture = run.latest_capture().unwrap();
            let expected =
                CameraMount::forward_nose(CAMERA_OFFSET_BODY_M).pose(&capture.vehicle, DVec3::ZERO);
            assert_eq!(capture.pose, expected);
        }
        let report = run.report();
        assert!(
            report.peak_bank_rad > 65_f64.to_radians(),
            "{dynamics:?}: {report:?}"
        );
        assert!(
            report.peak_body_rate_rad_s > 0.4,
            "{dynamics:?}: {report:?}"
        );
        assert!(report.horizon_loss_frames > 15, "{dynamics:?}: {report:?}");
        // A stick step moves the JSBSim aircraft's rate within one step, which
        // bounds the worst sample; the mean shows the rate is the true one.
        assert!(worst_rate_error < 0.15, "{dynamics:?}: {worst_rate_error}");
        let mean_rate_error = rate_error_sum / steps;
        assert!(mean_rate_error < 0.01, "{dynamics:?}: {mean_rate_error}");
    }
}

#[test]
fn a_loiter_earns_one_independent_anchor_and_marks_the_rest_shared() {
    for dynamics in EACH_DYNAMICS {
        let run = fly(ScenarioProfile::CorridorSameCellLoiter, dynamics);
        let report = run.report();

        assert_eq!(report.anchor_cells.len(), 1, "{dynamics:?}: {report:?}");
        assert_eq!(
            report.independent_anchor_fixes, 1,
            "{dynamics:?}: {report:?}"
        );
        assert!(
            report.shared_map_cell_fixes >= 230,
            "{dynamics:?}: {report:?}"
        );
        assert!(
            report.peak_bank_rad > 0.4,
            "{dynamics:?}: it orbits: {report:?}"
        );

        // Shared anchors stay usable but never reach the filter as independent
        // fixes, so they cannot collapse its covariance.
        assert!(!FusionEligibility::SharedMapCell.independent_ekf_fix());
        let shared = run.nees(FusionEligibility::SharedMapCell);
        assert_eq!(shared.count, report.shared_map_cell_fixes);
        let independent = run.nees(FusionEligibility::IndependentAnchor);
        assert_eq!(independent.count, 1);
    }
}

#[test]
fn the_synthetic_estimates_are_honest_about_their_covariance() {
    let run = fly(
        ScenarioProfile::CorridorNominalCruise,
        FlightDynamics::Jsbsim,
    );
    for eligibility in [
        FusionEligibility::CorrelatedTrack,
        FusionEligibility::SharedMapCell,
    ] {
        let mean = run.nees(eligibility).mean().unwrap();
        assert!((2.5..3.5).contains(&mean), "{eligibility:?}: {mean}");
    }
}

#[test]
fn a_replay_flies_the_same_bits() {
    for dynamics in EACH_DYNAMICS {
        let [mut one, mut two] = [(); 2]
            .map(|()| CorridorRun::new(ScenarioProfile::CorridorSameCellLoiter, dynamics).unwrap());
        for _ in 0..1_200 {
            one.step().unwrap();
            two.step().unwrap();
        }
        assert_eq!(bits(&one.vehicle()), bits(&two.vehicle()), "{dynamics:?}");
        assert_eq!(one.report(), two.report(), "{dynamics:?}");
    }
}

#[test]
fn the_panel_shows_a_healthy_earth_vehicle_with_no_guidance_computer() {
    let mut run = CorridorRun::new(
        ScenarioProfile::CorridorNominalCruise,
        FlightDynamics::Jsbsim,
    )
    .unwrap();
    for _ in 0..240 {
        run.step().unwrap();
    }
    let cockpit = run.cockpit();
    assert_eq!(cockpit.timestamp, run.now());
    assert_eq!(cockpit.guidance_source, None);
    assert_eq!(cockpit.attitude.validity, SourceValidity::Valid);
    assert_eq!(cockpit.attitude.value, run.vehicle().attitude);
    assert!((cockpit.height_m.value - CRUISE_HEIGHT_M).abs() < 25.0);
}

#[test]
fn past_the_end_of_the_script_nothing_moves() {
    let mut run = fly(
        ScenarioProfile::CorridorAggressiveBank,
        FlightDynamics::Kinematic,
    );
    let (at, state) = (run.now(), run.vehicle());
    run.step().unwrap();
    assert_eq!((run.now(), run.vehicle()), (at, state));
}

#[test]
fn autopilot_throttle_regulates_pitot_airspeed_rather_than_ground_speed() {
    use sim_atmosphere::AtmosphereSample;
    use sim_devices::{
        AirDataSystem, AirFlow, PitotStatic, PressureErrorSpec, sensed_true_airspeed_m_s,
    };

    let anchor = anchor_position();
    let origin_ecef = geodetic_to_ecef(&anchor);
    let frame = LocalFrame::at(&anchor);
    // Vehicle moving over ground at 40 m/s north into a 15 m/s headwind (from north, wn = -15):
    // true airspeed along nose is 55 m/s, matching CRUISE_SPEED_M_S!
    let state = VehicleState::level_flight(&anchor, 0.0, 40.0);
    let headwind = AtmosphereSample {
        pressure_pa: 101_325.0,
        temperature_k: 288.15,
        density_kg_m3: 1.225,
        speed_of_sound_m_s: 340.294,
        wind_ned_m_s: DVec3::new(-15.0, 0.0, 0.0),
    };
    let mut pitot = PitotStatic::with_pitot(
        PressureErrorSpec::IDEAL,
        PressureErrorSpec::IDEAL,
        DVec3::X,
        7,
    )
    .unwrap();
    let sample = pitot.sense(&AirFlow::of(&headwind, &state), 0.04);
    assert!(sample.differential_pressure_pa.unwrap() > 1_500.0);
    let sensed_tas = sensed_true_airspeed_m_s(&sample).unwrap();
    assert!((sensed_tas - 55.0).abs() < 0.05, "sensed_tas={sensed_tas}");

    let guidance = Guidance {
        steering: Steering::Track(0.0),
        height_m: anchor.height_m,
        speed_m_s: 55.0,
    };
    let without_pitot = FlightReading::of(&state, origin_ecef, &frame);
    let with_pitot = without_pitot.with_airspeed(sensed_tas);

    let mut ap_ground = Autopilot::default();
    let mut ap_pitot = Autopilot::default();
    let cmd_ground = ap_ground.update(&guidance, &without_pitot, 1.0 / 120.0);
    let cmd_pitot = ap_pitot.update(&guidance, &with_pitot, 1.0 / 120.0);

    // Without pitot, ground speed 40 < 55 erroneously commands huge extra throttle;
    // with pitot airspeed 55 == 55, throttle holds trim (~0.35).
    assert!(
        cmd_ground.throttle > cmd_pitot.throttle + 0.5,
        "ground={:.3}, pitot={:.3}",
        cmd_ground.throttle,
        cmd_pitot.throttle
    );
    assert!((cmd_pitot.throttle - 0.35).abs() < 0.01);
}
