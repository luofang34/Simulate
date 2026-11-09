//! Formal regression tests for P1 #2 (coordinate frame, unit, accuracy, and seal verification)
//! and P1 #3 (DEM terrain interaction shell intersection and ray sample budget enforcement).

use glam::DVec3;
use sim_analysis::{
    AccessAnalysisError, AnalysisJobStatus, AnalysisOutcomePayload, AnalysisRequestArtifact,
    ArtifactBindingError, DataCoverageError, DemElevationGrid, ModelNotApplicableReason,
    TerrainLosConfig, check_terrain_line_of_sight_fixed, evaluate_analysis_request_artifact,
};
use sim_geo::BodyEllipsoid;
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{NaifId, TdbEpoch};

const VALID_REQ_JSON: &str = r#"{
  "schema_version": 1,
  "request_id": "req-valley-relay-rev2",
  "design_revision": {
    "design_id": "design-valley-relay-001",
    "revision_number": 2,
    "design_digest_sha256": "9203be83a9ebc04b9d4a855c7d56cb7e6532e3185df41c6bf0733fbe0be3ac0e"
  },
  "time_scale": {
    "time_scale": "TdbJ2000",
    "subsecond_ticks_per_second": 46080000000,
    "utc_leap_second_table_id": "IERS-Bulletin-C-2026"
  },
  "coordinate_frames": {
    "inertial_frame": "SSB_ICRF_J2000",
    "body_fixed_frame": "EARTH_IAU_FIXED_WGS84",
    "length_unit": "m",
    "angle_unit": "rad"
  },
  "numerical_accuracy": {
    "trajectory_interpolation_method": "CubicHermiteIcrf",
    "requested_position_interpolation_error_m": null,
    "requested_boundary_time_tolerance_s": 0.5,
    "window_boundary_time_tolerance_s": 0.5,
    "terrain_chord_sample_step_m": 200.0,
    "max_terrain_ray_samples_per_chord": 50000
  },
  "query": {
    "query_interval": {
      "start_epoch": { "j2000_seconds": 840000000, "subsec_ticks": 0 },
      "end_epoch": { "j2000_seconds": 840000060, "subsec_ticks": 0 }
    },
    "sample_step_s": 10.0,
    "boundary_time_tolerance_s": 0.5,
    "min_range_m": null,
    "max_range_m": 2000000.0
  },
  "occultation_env": {
    "bodies": [
      {
        "body": 399,
        "shape": { "Ellipsoid": [6378137.0, 0.0033528106647474805, 398600441800000.0, 0.00007292115, 0.00108262668] }
      }
    ]
  },
  "terrain_dem": {
    "dataset_id": "regional-ridge-dem-v1",
    "body": 399,
    "lat_min_rad": -0.005,
    "lat_max_rad": 0.005,
    "lon_min_rad": -0.003,
    "lon_max_rad": 0.001,
    "rows": 3,
    "cols": 5,
    "cells_m": [
      400.0, 400.0, 2400.0, 450.0, 450.0,
      400.0, 400.0, 2400.0, 450.0, 450.0,
      400.0, 400.0, 2400.0, 450.0, 450.0
    ]
  },
  "terrain_los_config": {
    "sample_step_m": 200.0,
    "min_clearance_m": 10.0,
    "max_terrain_interaction_altitude_m": 12000.0,
    "max_ray_samples": 50000
  },
  "workload": {
    "AircraftSatelliteGroundRelay": {
      "aircraft": {
        "endpoint_id": 101,
        "label": "SOKIL-RECON-1",
        "host_body": 399,
        "nodes_frame": { "BodyFixedEcef": { "body": 399 } },
        "nodes": [
          {
            "epoch": { "j2000_seconds": 840000000, "subsec_ticks": 0 },
            "pos_m": [6382924.234130255, -12765.865489419035, 0.0],
            "vel_m_s": [0.07659520213492214, 42.55284411394671, 0.0],
            "q_body_to_axes": [0.0, 0.0, 0.0, 1.0],
            "omega_body_rad_s": [0.0, 0.0, 0.0]
          },
          {
            "epoch": { "j2000_seconds": 840000060, "subsec_ticks": 0 },
            "pos_m": [6382928.829842383, -10212.694842582232, 0.0],
            "vel_m_s": [0.07659520213492214, 42.55284411394671, 0.0],
            "q_body_to_axes": [0.0, 0.0, 0.0, 1.0],
            "omega_body_rad_s": [0.0, 0.0, 0.0]
          }
        ],
        "min_local_elevation_rad": null
      },
      "aircraft_to_sat_sensor": {
        "sensor_id": 11,
        "label": "Dorsal Ku-Band Satcom",
        "mount": { "offset_body_m": [0.0, 0.0, 0.0], "body_to_mount": [0.0, -0.7071067811865476, 0.0, 0.7071067811865476] },
        "cg_body_m": [0.0, 0.0, 0.0],
        "fov": { "ConicalHalfAngle": { "max_half_angle_rad": 0.95 } },
        "require_attitude": true
      },
      "aircraft_to_ground_sensor": {
        "sensor_id": 12,
        "label": "Belly C-Band Direct Link",
        "mount": { "offset_body_m": [0.0, 0.0, 0.0], "body_to_mount": [0.0, 0.0, 0.0, 1.0] },
        "cg_body_m": [0.0, 0.0, 0.0],
        "fov": { "ConicalHalfAngle": { "max_half_angle_rad": 3.141592653589793 } },
        "require_attitude": false
      },
      "satellite": {
        "endpoint_id": 201,
        "label": "RELAY-LEO-1",
        "host_body": 399,
        "nodes_frame": { "BodyFixedEcef": { "body": 399 } },
        "nodes": [
          {
            "epoch": { "j2000_seconds": 840000000, "subsec_ticks": 0 },
            "pos_m": [6978133.510931791, -6978.135836977225, 0.0],
            "vel_m_s": [0.020934408142541844, 23.26044720741494, 0.0],
            "q_body_to_axes": [0.0, 0.0, 0.0, 1.0],
            "omega_body_rad_s": [0.0, 0.0, 0.0]
          },
          {
            "epoch": { "j2000_seconds": 840000060, "subsec_ticks": 0 },
            "pos_m": [6978134.766996279, -5582.509004532329, 0.0],
            "vel_m_s": [0.020934408142541844, 23.26044720741494, 0.0],
            "q_body_to_axes": [0.0, 0.0, 0.0, 1.0],
            "omega_body_rad_s": [0.0, 0.0, 0.0]
          }
        ],
        "min_local_elevation_rad": null
      },
      "sat_to_aircraft_sensor": {
        "sensor_id": 21,
        "label": "Satellite Crosslink Spot Beam",
        "mount": { "offset_body_m": [0.0, 0.0, 0.0], "body_to_mount": [0.0, 0.7071067811865476, 0.0, 0.7071067811865476] },
        "cg_body_m": [0.0, 0.0, 0.0],
        "fov": { "ConicalHalfAngle": { "max_half_angle_rad": 0.6 } },
        "require_attitude": true
      },
      "sat_to_ground_sensor": {
        "sensor_id": 22,
        "label": "Satellite Ka-Band Feeder Link",
        "mount": { "offset_body_m": [0.0, 0.0, 0.0], "body_to_mount": [0.0, 0.7071067811865476, 0.0, 0.7071067811865476] },
        "cg_body_m": [0.0, 0.0, 0.0],
        "fov": { "ConicalHalfAngle": { "max_half_angle_rad": 0.6 } },
        "require_attitude": true
      },
      "ground_station": {
        "station": {
          "station_id": 301,
          "name": "Valley Forward Ground Station",
          "body": 399,
          "geodetic": [0.0, 0.0, 500.0],
          "ellipsoid": [6378137.0, 0.0033528106647474805, 398600441800000.0, 0.00007292115, 0.00108262668],
          "mount": { "offset_body_m": [0.0, 0.0, 0.0], "body_to_mount": [0.0, 0.0, 0.0, 1.0] },
          "fov": "OmnidirectionalHemisphere",
          "min_elevation_rad": 0.02
        },
        "altitude_spec": {
          "datum": { "DemSurfaceAgl": { "dataset_id": "regional-ridge-dem-v1" } },
          "value_m": 50.0
        }
      }
    }
  }
}"#;

#[test]
fn p1_regression_invalid_units_and_unprovable_interpolation_fail_closed() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("load verified solar system");
    let base_req: AnalysisRequestArtifact =
        serde_json::from_str(VALID_REQ_JSON).expect("deserialize req");

    let mut req_km = base_req.clone();
    req_km.coordinate_frames.length_unit = "km".to_owned();
    let res_km = evaluate_analysis_request_artifact(
        &req_km,
        "res-km-rejected",
        &solar.ephemeris,
        &solar.orientation,
    );
    assert!(matches!(
        res_km.outcome,
        AnalysisOutcomePayload::EvaluationFailed(AccessAnalysisError::ModelNotApplicable(
            ModelNotApplicableReason::UnsupportedCoordinateFrameOrUnitContract { .. }
        ))
    ));

    let mut req_unprovable = base_req;
    req_unprovable
        .numerical_accuracy
        .requested_position_interpolation_error_m = Some(0.05);
    let res_unprovable = evaluate_analysis_request_artifact(
        &req_unprovable,
        "res-unprovable-interp",
        &solar.ephemeris,
        &solar.orientation,
    );
    assert!(matches!(
        res_unprovable.outcome,
        AnalysisOutcomePayload::EvaluationFailed(AccessAnalysisError::ModelNotApplicable(
            ModelNotApplicableReason::UnprovableInterpolationAccuracyBound {
                requested_error_m
            }
        )) if (requested_error_m - 0.05).abs() < 1e-12
    ));
}

#[test]
fn p1_regression_tampered_result_fields_invalidate_digest() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("load verified solar system");
    let base_req: AnalysisRequestArtifact =
        serde_json::from_str(VALID_REQ_JSON).expect("deserialize req");
    let valid_res = evaluate_analysis_request_artifact(
        &base_req,
        "res-valid",
        &solar.ephemeris,
        &solar.orientation,
    );
    assert_eq!(valid_res.verify_against_request(&base_req), Ok(()));

    let mut tampered_unit = valid_res.clone();
    tampered_unit.coordinate_frames.length_unit = "miles".to_owned();
    assert!(!tampered_unit.verify_revision_binding(&base_req.design_revision));
    assert_eq!(
        tampered_unit.verify_against_request(&base_req),
        Err(ArtifactBindingError::ResultDigestMismatch)
    );

    let mut tampered_acc = valid_res.clone();
    tampered_acc
        .numerical_accuracy
        .proven_position_interpolation_error_m = Some(0.001);
    assert!(!tampered_acc.verify_revision_binding(&base_req.design_revision));
    assert_eq!(
        tampered_acc.verify_against_request(&base_req),
        Err(ArtifactBindingError::ResultDigestMismatch)
    );

    let mut tampered_status = valid_res.clone();
    tampered_status.job_status = AnalysisJobStatus::SupersededByNewerRevision {
        newer_revision_number: 99,
    };
    assert!(!tampered_status.verify_revision_binding(&base_req.design_revision));
    assert_eq!(
        tampered_status.verify_against_request(&base_req),
        Err(ArtifactBindingError::ResultDigestMismatch)
    );

    let mut tampered_repro = valid_res;
    tampered_repro.reproducibility.evaluator_version = "9.9.9".to_owned();
    assert!(!tampered_repro.verify_revision_binding(&base_req.design_revision));
    assert_eq!(
        tampered_repro.verify_against_request(&base_req),
        Err(ArtifactBindingError::ResultDigestMismatch)
    );
}

#[test]
fn p1_regression_geo_35786km_ray_does_not_skip_low_altitude_dem_and_enforces_sample_budget() {
    let out_of_coverage_dem = DemElevationGrid {
        dataset_id: "distant-dem".to_owned(),
        body: NaifId::EARTH,
        lat_min_rad: 0.5,
        lat_max_rad: 0.6,
        lon_min_rad: 0.5,
        lon_max_rad: 0.6,
        rows: 2,
        cols: 2,
        cells_m: vec![Some(100.0); 4],
    };

    let p0_low_alt = DVec3::new(6_378_237.0, 0.0, 0.0);
    let p1_geo = DVec3::new(42_164_137.0, 0.0, 0.0);
    let cfg_10m = TerrainLosConfig {
        sample_step_m: 10.0,
        min_clearance_m: 5.0,
        max_terrain_interaction_altitude_m: 1_000.0,
        max_ray_samples: 50_000,
    };
    let res_geo = check_terrain_line_of_sight_fixed(
        TdbEpoch::JAN_1_2026_00Z_TDB,
        p0_low_alt,
        p1_geo,
        BodyEllipsoid::WGS84,
        &out_of_coverage_dem,
        &cfg_10m,
    );
    assert!(matches!(
        res_geo,
        Err(AccessAnalysisError::DataCoverage(
            DataCoverageError::TerrainDemOutOfCoverage { .. }
        ))
    ));

    let cfg_tight_budget = TerrainLosConfig {
        sample_step_m: 10.0,
        min_clearance_m: 5.0,
        max_terrain_interaction_altitude_m: 12_000.0,
        max_ray_samples: 50,
    };
    let res_budget = check_terrain_line_of_sight_fixed(
        TdbEpoch::JAN_1_2026_00Z_TDB,
        p0_low_alt,
        p1_geo,
        BodyEllipsoid::WGS84,
        &out_of_coverage_dem,
        &cfg_tight_budget,
    );
    assert!(matches!(
        res_budget,
        Err(AccessAnalysisError::ModelNotApplicable(
            ModelNotApplicableReason::TerrainRaySampleBudgetExceeded {
                max_ray_samples: 50,
                ..
            }
        ))
    ));

    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("load verified solar system");
    let mut req_no_dem: AnalysisRequestArtifact =
        serde_json::from_str(VALID_REQ_JSON).expect("deserialize req");
    req_no_dem.terrain_dem = None;
    let res_no_dem = evaluate_analysis_request_artifact(
        &req_no_dem,
        "res-no-dem",
        &solar.ephemeris,
        &solar.orientation,
    );
    assert!(matches!(
        res_no_dem.outcome,
        AnalysisOutcomePayload::EvaluationFailed(AccessAnalysisError::DataCoverage(
            DataCoverageError::TerrainDemOutOfCoverage { .. }
        ))
    ));
}
