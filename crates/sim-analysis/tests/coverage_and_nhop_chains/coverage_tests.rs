//! Integration tests for geodesic H3 coverage grids and N-fold revisit statistics.

use core::f64::consts::{FRAC_PI_2, PI};
use glam::DVec3;
use sim_analysis::{
    AccessAnalysisError, AccessQuerySpec, CoverageAssetSpec, CoverageGridError, CoverageQuerySpec,
    CoverageRegionSpec, DataCoverageError, OccultationEnvironmentModel, SignalTimeModel,
    VehicleSensorModel, evaluate_coverage_grid, evaluate_station_access,
};
use sim_geo::BodyEllipsoid;
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{NaifId, ValidEpochInterval};

use crate::fixtures::{
    circular_polar_earth_orbit, earth_fixed_stationary_trajectory, mars_opposition_epoch,
};

fn central_angle_rad(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let v1 = DVec3::new(lat1.cos() * lon1.cos(), lat1.cos() * lon1.sin(), lat1.sin());
    let v2 = DVec3::new(lat2.cos() * lon2.cos(), lat2.cos() * lon2.sin(), lat2.sin());
    v1.dot(v2).clamp(-1.0, 1.0).acos()
}

#[test]
fn global_h3_visibility_cone_matches_analytic_half_angle_and_station_access() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = mars_opposition_epoch();
    let alt_m = 2_000_000.0;
    let r_sat = BodyEllipsoid::WGS84.semi_major_m + alt_m;
    let traj = earth_fixed_stationary_trajectory(1, 0.0, 0.0, r_sat, t0, 60.0, &solar);
    let sensor = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let min_el_rad = 10.0_f64.to_radians();
    let spherical_earth = BodyEllipsoid {
        flattening: 0.0,
        ..BodyEllipsoid::WGS84
    };
    let occ_env = OccultationEnvironmentModel { bodies: vec![] };
    let query = CoverageQuerySpec {
        body: NaifId::EARTH,
        ellipsoid: spherical_earth,
        region: CoverageRegionSpec::Global { resolution: 2 },
        min_elevation_rad: min_el_rad,
        min_fold: 1,
        query_interval: ValidEpochInterval::new(t0, t0),
        sample_step_s: 60.0,
        occultation_env: occ_env.clone(),
        max_cells: 10_000,
    };
    let assets = [CoverageAssetSpec {
        trajectory: &traj,
        attitude: None,
        sensor: &sensor,
    }];
    let report = evaluate_coverage_grid(&assets, &query, &solar.ephemeris, &solar.orientation)
        .expect("global h3 coverage");

    assert_eq!(report.total_cells, 5_882);
    assert!(
        report
            .cells
            .windows(2)
            .all(|w| w[0].h3_index < w[1].h3_index)
    );
    assert_station_parity(&solar, &traj, &sensor, &query, r_sat, &report);
}

fn assert_station_parity(
    solar: &BuiltinSolarSystem2026_2028,
    traj: &sim_spacetime::SampledKinematicTrajectory,
    sensor: &VehicleSensorModel,
    query: &CoverageQuerySpec,
    r_sat: f64,
    report: &sim_analysis::CoverageGridReport,
) {
    let psi_analytic = ((query.ellipsoid.semi_major_m / r_sat) * query.min_elevation_rad.cos())
        .clamp(-1.0, 1.0)
        .acos()
        - query.min_elevation_rad;
    let expected_cap_fraction = 0.5 * (1.0 - psi_analytic.cos());
    assert!(
        (report.n_fold_fraction - expected_cap_fraction).abs() < 0.006,
        "covered {} vs analytic cap {}",
        report.n_fold_fraction,
        expected_cap_fraction
    );
    for cell in &report.cells {
        let theta = central_angle_rad(0.0, 0.0, cell.lat_rad, cell.lon_rad);
        if theta < psi_analytic - 1.0e-3 {
            assert_eq!(cell.covered_samples, 1);
        } else if theta > psi_analytic + 1.0e-3 {
            assert_eq!(cell.covered_samples, 0);
        }
    }
    let sample_cell = report
        .cells
        .iter()
        .find(|c| c.covered_samples == 1)
        .expect("lit cell");
    let station =
        sample_cell.as_ground_station(query.body, query.ellipsoid, query.min_elevation_rad);
    let access = evaluate_station_access(
        traj,
        None,
        &station,
        sensor,
        &query.occultation_env,
        &AccessQuerySpec {
            query_interval: query.query_interval,
            sample_step_s: 60.0,
            boundary_time_tolerance_s: 60.0,
            min_range_m: None,
            max_range_m: None,
            signal_time: SignalTimeModel::Geometric,
        },
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("station access parity");
    assert!(access.samples[0].accessible);
}

#[test]
fn dual_satellite_n_fold_overlap_and_determinism() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = mars_opposition_epoch();
    let r_sat = BodyEllipsoid::WGS84.semi_major_m + 2_000_000.0;
    let sat_a = earth_fixed_stationary_trajectory(10, 0.0, 0.0, r_sat, t0, 60.0, &solar);
    let sat_b = earth_fixed_stationary_trajectory(11, 0.0, 30.0, r_sat, t0, 60.0, &solar);
    let sensor = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let min_el = 10.0_f64.to_radians();
    let psi = ((BodyEllipsoid::WGS84.semi_major_m / r_sat) * min_el.cos()).acos() - min_el;

    let make_query = |min_fold| CoverageQuerySpec {
        body: NaifId::EARTH,
        ellipsoid: BodyEllipsoid::WGS84,
        region: CoverageRegionSpec::Global { resolution: 1 },
        min_elevation_rad: min_el,
        min_fold,
        query_interval: ValidEpochInterval::new(t0, t0),
        sample_step_s: 60.0,
        occultation_env: OccultationEnvironmentModel::earth_wgs84(),
        max_cells: 5_000,
    };
    let assets = [
        CoverageAssetSpec {
            trajectory: &sat_a,
            attitude: None,
            sensor: &sensor,
        },
        CoverageAssetSpec {
            trajectory: &sat_b,
            attitude: None,
            sensor: &sensor,
        },
    ];
    let r1 = evaluate_coverage_grid(
        &assets,
        &make_query(1),
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("1-fold");
    let r2 = evaluate_coverage_grid(
        &assets,
        &make_query(2),
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("2-fold");
    let r2_again = evaluate_coverage_grid(
        &assets,
        &make_query(2),
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("2-fold repeat");

    assert_eq!(
        serde_json::to_vec(&r2).expect("json"),
        serde_json::to_vec(&r2_again).expect("json")
    );
    assert_eq!(r1.total_cells, 842);
    assert!(r2.n_fold_fraction > 0.0 && r2.n_fold_fraction < r1.n_fold_fraction);
    for c in &r2.cells {
        let th_a = central_angle_rad(0.0, 0.0, c.lat_rad, c.lon_rad);
        let th_b = central_angle_rad(0.0, 30.0_f64.to_radians(), c.lat_rad, c.lon_rad);
        if th_a < psi - 0.03 && th_b < psi - 0.03 {
            assert_eq!(c.max_fold_observed, 2);
            assert_eq!(c.covered_samples, 1);
        } else if th_a > psi + 0.03 && th_b > psi + 0.03 {
            assert_eq!(c.max_fold_observed, 0);
        }
    }
}

#[test]
fn polar_walker_revisit_gaps_shrink_with_second_antipodal_satellite() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = mars_opposition_epoch();
    let duration_s = 14_400.0;
    let sat1 = circular_polar_earth_orbit(21, 1_500_000.0, 0.0, 0.0, t0, duration_s);
    let sat2 = circular_polar_earth_orbit(22, 1_500_000.0, 0.0, PI, t0, duration_s);
    let sensor = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let query = CoverageQuerySpec {
        body: NaifId::EARTH,
        ellipsoid: BodyEllipsoid::WGS84,
        region: CoverageRegionSpec::LatLonBox {
            resolution: 2,
            min_lat_rad: (-10.0_f64).to_radians(),
            max_lat_rad: 10.0_f64.to_radians(),
            min_lon_rad: (-15.0_f64).to_radians(),
            max_lon_rad: 15.0_f64.to_radians(),
        },
        min_elevation_rad: 5.0_f64.to_radians(),
        min_fold: 1,
        query_interval: ValidEpochInterval::new(t0, t0.advanced_seconds(duration_s)),
        sample_step_s: 120.0,
        occultation_env: OccultationEnvironmentModel::earth_wgs84(),
        max_cells: 2_000,
    };

    let one_sat = evaluate_coverage_grid(
        &[CoverageAssetSpec {
            trajectory: &sat1,
            attitude: None,
            sensor: &sensor,
        }],
        &query,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("single polar sat");
    let two_sats = evaluate_coverage_grid(
        &[
            CoverageAssetSpec {
                trajectory: &sat1,
                attitude: None,
                sensor: &sensor,
            },
            CoverageAssetSpec {
                trajectory: &sat2,
                attitude: None,
                sensor: &sensor,
            },
        ],
        &query,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("dual polar walker");

    assert!(one_sat.total_cells > 0);
    assert!(two_sats.mean_coverage_fraction > one_sat.mean_coverage_fraction);
    assert!(two_sats.global_max_revisit_gap_s < one_sat.global_max_revisit_gap_s);
    for c in &one_sat.cells {
        assert!(c.gaps.windows(2).all(|w| w[0].end_epoch < w[1].start_epoch));
        if c.covered_samples > 0 && c.covered_samples < c.total_samples {
            assert!(!c.gaps.is_empty());
            assert!(c.mean_revisit_gap_s > 0.0);
            assert!(c.max_revisit_gap_s >= c.mean_revisit_gap_s);
        }
    }
}

#[test]
fn antimeridian_box_explicit_cells_and_validation_errors() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = mars_opposition_epoch();
    let sat = earth_fixed_stationary_trajectory(30, 0.0, 180.0, 8_378_137.0, t0, 60.0, &solar);
    let sensor = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let assets = [CoverageAssetSpec {
        trajectory: &sat,
        attitude: None,
        sensor: &sensor,
    }];

    let antimeridian_q = CoverageQuerySpec {
        body: NaifId::EARTH,
        ellipsoid: BodyEllipsoid::WGS84,
        region: CoverageRegionSpec::LatLonBox {
            resolution: 2,
            min_lat_rad: (-10.0_f64).to_radians(),
            max_lat_rad: 10.0_f64.to_radians(),
            min_lon_rad: 170.0_f64.to_radians(),
            max_lon_rad: (-170.0_f64).to_radians(),
        },
        min_elevation_rad: 5.0_f64.to_radians(),
        min_fold: 1,
        query_interval: ValidEpochInterval::new(t0, t0),
        sample_step_s: 60.0,
        occultation_env: OccultationEnvironmentModel::earth_wgs84(),
        max_cells: 1_000,
    };
    let anti_rep = evaluate_coverage_grid(
        &assets,
        &antimeridian_q,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("antimeridian box");
    assert!(
        anti_rep
            .cells
            .iter()
            .any(|c| c.lon_rad > 165.0_f64.to_radians())
    );
    assert!(
        anti_rep
            .cells
            .iter()
            .any(|c| c.lon_rad < (-165.0_f64).to_radians())
    );
    assert!(
        anti_rep
            .cells
            .iter()
            .all(|c| c.lon_rad.abs() > 140.0_f64.to_radians())
    );

    let mut raw_ids: Vec<u64> = anti_rep.cells.iter().take(4).map(|c| c.h3_index).collect();
    raw_ids.reverse();
    raw_ids.push(anti_rep.cells[0].h3_index);
    let explicit_q = CoverageQuerySpec {
        region: CoverageRegionSpec::ExplicitCells { h3_cells: raw_ids },
        ..antimeridian_q.clone()
    };
    let explicit_rep =
        evaluate_coverage_grid(&assets, &explicit_q, &solar.ephemeris, &solar.orientation)
            .expect("explicit dedup & sort");
    assert_eq!(explicit_rep.total_cells, 4);
    assert!(
        explicit_rep
            .cells
            .windows(2)
            .all(|w| w[0].h3_index < w[1].h3_index)
    );
}

#[test]
fn coverage_grid_validation_errors_fail_closed() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = mars_opposition_epoch();
    let sat = earth_fixed_stationary_trajectory(30, 0.0, 180.0, 8_378_137.0, t0, 60.0, &solar);
    let sensor = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let assets = [CoverageAssetSpec {
        trajectory: &sat,
        attitude: None,
        sensor: &sensor,
    }];
    let base_q = CoverageQuerySpec {
        body: NaifId::EARTH,
        ellipsoid: BodyEllipsoid::WGS84,
        region: CoverageRegionSpec::Global { resolution: 1 },
        min_elevation_rad: 5.0_f64.to_radians(),
        min_fold: 1,
        query_interval: ValidEpochInterval::new(t0, t0),
        sample_step_s: 60.0,
        occultation_env: OccultationEnvironmentModel::earth_wgs84(),
        max_cells: 1_000,
    };

    let bad_global = CoverageQuerySpec {
        region: CoverageRegionSpec::Global { resolution: 4 },
        ..base_q.clone()
    };
    assert!(matches!(
        evaluate_coverage_grid(&assets, &bad_global, &solar.ephemeris, &solar.orientation),
        Err(CoverageGridError::GlobalResolutionTooHigh { resolution: 4, .. })
    ));
    assert!(matches!(
        evaluate_coverage_grid(&[], &base_q, &solar.ephemeris, &solar.orientation),
        Err(CoverageGridError::EmptyConstellation)
    ));
    let bad_fold = CoverageQuerySpec {
        min_fold: 2,
        ..base_q.clone()
    };
    assert!(matches!(
        evaluate_coverage_grid(&assets, &bad_fold, &solar.ephemeris, &solar.orientation),
        Err(CoverageGridError::InvalidRequiredFold {
            required_fold: 2,
            asset_count: 1
        })
    ));
    let out_of_cov = CoverageQuerySpec {
        query_interval: ValidEpochInterval::new(t0, t0.advanced_seconds(120.0)),
        ..base_q.clone()
    };
    assert!(matches!(
        evaluate_coverage_grid(&assets, &out_of_cov, &solar.ephemeris, &solar.orientation),
        Err(CoverageGridError::Access(
            AccessAnalysisError::DataCoverage(DataCoverageError::TrajectoryOutOfCoverage { .. })
        ))
    ));
    let bad_bounds = CoverageQuerySpec {
        region: CoverageRegionSpec::LatLonBox {
            resolution: 2,
            min_lat_rad: FRAC_PI_2,
            max_lat_rad: 0.0,
            min_lon_rad: 0.0,
            max_lon_rad: 1.0,
        },
        ..base_q
    };
    assert!(matches!(
        evaluate_coverage_grid(&assets, &bad_bounds, &solar.ephemeris, &solar.orientation),
        Err(CoverageGridError::InvalidRegionBounds { .. })
    ));
}
