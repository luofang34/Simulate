use navigate_contract::SensorClass;
use sim_devices::WhiteNoise;
use sim_navigate::trial::{QualityFloor, WaypointSpec};

use super::magnetic_harness::{corridor_waypoints, fly_magnav_route};
use super::magnetic_world::MagneticAidingMode;

#[test]
fn crustal_magnetic_anomaly_aiding_improves_gnss_denied_closed_loop_accuracy() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    let route = corridor_waypoints();
    let unaided = fly_magnav_route(
        &route,
        MagneticAidingMode::None,
        QualityFloor::Unusable,
        0,
        500.0,
        None,
    );
    let main_only = fly_magnav_route(
        &route,
        MagneticAidingMode::MainFieldOnly,
        QualityFloor::Unusable,
        0,
        500.0,
        None,
    );
    let aided = fly_magnav_route(
        &route,
        MagneticAidingMode::CrustalOverlay {
            bias_offset_t: 2.5e-9,
        },
        QualityFloor::Degraded,
        0,
        500.0,
        None,
    );

    tracing::info!(
        unaided_rmse_m = unaided.horizontal_rmse_m,
        unaided_final_m = unaided.final_horizontal_error_m,
        unaided_wp_miss_m = unaided.final_waypoint_miss_m,
        main_only_uninformative = main_only.mag_uninformative,
        aided_rmse_m = aided.horizontal_rmse_m,
        aided_final_m = aided.final_horizontal_error_m,
        aided_wp_miss_m = aided.final_waypoint_miss_m,
        aided_nees = aided.mean_position_nees,
        aided_admitted = aided.mag_admitted,
        "crustal MagNav vs unaided vs main-field-only"
    );

    assert!(unaided.completed && main_only.completed && aided.completed);
    assert!(!unaided.composition.contains(SensorClass::MagneticAnomaly));
    assert!(!main_only.composition.contains(SensorClass::MagneticAnomaly));
    assert_eq!(main_only.mag_admitted, 0);
    assert!(main_only.mag_uninformative > 200);
    assert!((main_only.final_horizontal_error_m - unaided.final_horizontal_error_m).abs() < 1e-6);

    assert!(aided.mag_admitted > 200);
    assert!(aided.composition.contains(SensorClass::MagneticAnomaly));
    assert!(aided.composition.contains(SensorClass::Inertial));
    assert!(aided.composition.contains(SensorClass::Barometric));
    assert!(!aided.composition.contains(SensorClass::Gnss));
    assert!(aided.horizontal_rmse_m < 0.25 * unaided.horizontal_rmse_m);
    assert!(aided.final_horizontal_error_m < 0.20 * unaided.final_horizontal_error_m);
    assert!(
        aided.final_waypoint_miss_m <= 120.0
            && aided.final_waypoint_miss_m < 0.5 * unaided.final_waypoint_miss_m,
        "aided miss {} vs unaided {}",
        aided.final_waypoint_miss_m,
        unaided.final_waypoint_miss_m
    );
    assert!((0.3..=8.0).contains(&aided.mean_position_nees));
}

#[test]
fn crustal_magnetic_anomaly_ensemble_keeps_consistent_nees_with_survey_biases() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    let route = corridor_waypoints();
    let mut bias_rng = WhiteNoise::from_seed(0xB1A5);
    let mut nees_total = 0.0_f64;
    let seeds = 5_u64;
    for seed in 0..seeds {
        let bias_offset_t = bias_rng.next_gaussian() * 3.0e-9;
        let unaided = fly_magnav_route(
            &route,
            MagneticAidingMode::None,
            QualityFloor::Unusable,
            seed,
            500.0,
            None,
        );
        let aided = fly_magnav_route(
            &route,
            MagneticAidingMode::CrustalOverlay { bias_offset_t },
            QualityFloor::Unusable,
            seed,
            500.0,
            None,
        );
        tracing::info!(
            seed,
            bias_nt = bias_offset_t * 1e9,
            unaided_rmse_m = unaided.horizontal_rmse_m,
            aided_rmse_m = aided.horizontal_rmse_m,
            unaided_final_m = unaided.final_horizontal_error_m,
            aided_final_m = aided.final_horizontal_error_m,
            nees = aided.mean_position_nees,
            "ensemble seed"
        );
        assert!(aided.completed);
        assert!(aided.composition.contains(SensorClass::MagneticAnomaly));
        assert!(aided.horizontal_rmse_m < 0.30 * unaided.horizontal_rmse_m);
        assert!(aided.final_horizontal_error_m < 0.25 * unaided.final_horizontal_error_m);
        assert!((0.2..=10.0).contains(&aided.mean_position_nees));
        nees_total += aided.mean_position_nees;
    }
    let ensemble_nees = nees_total / seeds as f64;
    tracing::info!(ensemble_nees, "ensemble mean position NEES");
    assert!((0.5..=6.0).contains(&ensemble_nees), "{ensemble_nees}");
}

#[test]
fn map_revision_switch_and_out_of_coverage_grow_covariance_honestly() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    let wp = |ident: &str, lat_deg: f64, lon_deg: f64| WaypointSpec {
        ident: ident.to_owned(),
        lat_deg,
        lon_deg,
        altitude_m: 1_000.0,
        speed_m_s: None,
        fly_over: false,
    };
    let corridor = vec![wp("C0", 39.00, -75.60), wp("C1", 39.00, -75.25)];
    let summary = fly_magnav_route(
        &corridor,
        MagneticAidingMode::CrustalOverlay {
            bias_offset_t: 3.0e-9,
        },
        QualityFloor::Unusable,
        2,
        550.0,
        Some((120.0, 2, -4.0e-9)),
    );
    assert!(summary.completed);
    assert!(summary.composition.contains(SensorClass::MagneticAnomaly));
    assert!(summary.mag_admitted > 180);
    assert!((0.3..=8.0).contains(&summary.mean_position_nees));
    let inside_sigma = summary
        .sigma_series_m
        .iter()
        .find(|(t, _)| *t >= 200.0)
        .unwrap()
        .1;
    let exit_sigma = summary
        .sigma_series_m
        .iter()
        .find(|(t, _)| *t >= 320.0)
        .unwrap()
        .1;
    let end_sigma = summary.sigma_series_m.last().unwrap().1;
    tracing::info!(inside_sigma, exit_sigma, end_sigma, "coverage exit sigma");
    assert!(inside_sigma < 40.0, "{inside_sigma}");
    assert!(end_sigma > 2.0 * exit_sigma && end_sigma > 2.0 * inside_sigma);
}
