//! End-to-end verification of two-endpoint LOS, DEM terrain occultation,
//! aircraft-satellite-ground geometric relay windows, and the shared
//! `Sokoly` <-> `Simulate` versioned `AnalysisRequestArtifact` / `AnalysisResultArtifact` contract.

use sim_analysis::{
    ANALYSIS_CONTRACT_SCHEMA_VERSION, AccessAnalysisError, AccessRestrictionReason, AltitudeSpec,
    AnalysisJobStatus, AnalysisOutcomePayload, AnalysisRequestArtifact, DataCoverageError,
    RelayHopKind, VerticalDatumKind, VerticalDatumSpec, evaluate_analysis_request_artifact,
};
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{BodyOrientationCatalog, EphemerisCatalog};

const SOKOLY_SAMPLE_REQ_REV1_JSON: &str = r#"{
  "schema_version": 2,
  "request_id": "req-valley-relay-rev1",
  "design_revision": {
    "design_id": "design-valley-relay-001",
    "revision_number": 1,
    "design_digest_sha256": "f4c3a81c4dceced0c0911962212a3503e843a179353a2fc7456cde8a774514f1"
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
    "max_range_m": 2000000.0,
    "signal_time": "Geometric"
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
            "pos_m": [6379324.241330253, -12758.665494219034, 0.0],
            "vel_m_s": [0.07655200214746098, 42.52884415298671, 0.0],
            "q_body_to_axes": [0.0, 0.0, 0.0, 1.0],
            "omega_body_rad_s": [0.0, 0.0, 0.0]
          },
          {
            "epoch": { "j2000_seconds": 840000060, "subsec_ticks": 0 },
            "pos_m": [6379328.834450382, -10206.934845039832, 0.0],
            "vel_m_s": [0.07655200214746098, 42.52884415298671, 0.0],
            "q_body_to_axes": [0.0, 0.0, 0.0, 1.0],
            "omega_body_rad_s": [0.0, 0.0, 0.0]
          }
        ],
        "min_local_elevation_rad": null
      },
      "aircraft_to_sat_sensor": {
        "sensor_id": 11,
        "label": "Dorsal Ku-Band Satcom",
        "mount": { "offset_body_m": [0.0, 0.0, 0.0], "body_to_mount": [0.0, 0.0, 0.0, 1.0] },
        "cg_body_m": [0.0, 0.0, 0.0],
        "fov": { "ConicalHalfAngle": { "max_half_angle_rad": 0.25 } },
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

const SOKOLY_SAMPLE_REQ_REV2_JSON: &str = r#"{
  "schema_version": 2,
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
    "max_range_m": 2000000.0,
    "signal_time": "Geometric"
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
fn sokoly_shared_sample_revision_1_roundtrip_in_simulate() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("load verified solar system");
    let req1: AnalysisRequestArtifact =
        serde_json::from_str(SOKOLY_SAMPLE_REQ_REV1_JSON).expect("deserialize req1");
    assert_eq!(req1.schema_version, ANALYSIS_CONTRACT_SCHEMA_VERSION);

    let res1 = evaluate_analysis_request_artifact(
        &req1,
        "res-valley-relay-rev1",
        &solar.ephemeris,
        &solar.orientation,
    );
    assert!(res1.verify_revision_binding(&req1.design_revision));
    assert_eq!(res1.verify_against_request(&req1), Ok(()));
    assert_eq!(res1.job_status, AnalysisJobStatus::Completed);
    assert!(res1.verification_scope.geometric_los_and_fov_verified);
    assert!(!res1.verification_scope.rf_link_budget_verified);
    assert!(!res1.verification_scope.store_and_forward_modeled);
    assert_eq!(
        res1.numerical_accuracy
            .proven_position_interpolation_error_m,
        None
    );
    assert_eq!(
        res1.numerical_accuracy
            .proven_window_boundary_time_tolerance_s,
        Some(0.5)
    );
    assert_eq!(res1.reproducibility.evaluator_id, "sim-analysis");
    assert_eq!(
        res1.reproducibility.ephemeris_catalog_hash,
        solar.ephemeris.content_hash()
    );
    assert_eq!(
        res1.reproducibility.orientation_catalog_hash,
        solar.orientation.content_hash()
    );
    assert!(res1.reproducibility.terrain_dataset_hash.is_some());

    let AnalysisOutcomePayload::GeometricRelayReport(rep1) = &res1.outcome else {
        panic!("expected GeometricRelayReport for rev1");
    };
    assert!(rep1.relay_windows.is_empty());
    assert!(rep1.hop1_windows.is_empty());
    assert_eq!(rep1.hop2_windows.len(), 1);
    assert!(rep1.direct_windows.is_empty());
    for sample in &rep1.samples {
        assert!(sample.relay_restrictions.iter().any(|entry| {
            entry.hop == RelayHopKind::AircraftToSatellite
                && matches!(
                    entry.reason,
                    AccessRestrictionReason::OutsideVehicleSensorFov { .. }
                )
        }));
        assert!(sample.relay_restrictions.iter().any(|entry| {
            entry.hop == RelayHopKind::DirectAircraftToGround
                && matches!(
                    entry.reason,
                    AccessRestrictionReason::OccultedByTerrain { .. }
                )
        }));
    }
}

#[test]
fn sokoly_shared_sample_revision_2_roundtrip_and_missing_dem_fail_closed() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("load verified solar system");
    let req1: AnalysisRequestArtifact =
        serde_json::from_str(SOKOLY_SAMPLE_REQ_REV1_JSON).expect("deserialize req1");
    let req2: AnalysisRequestArtifact =
        serde_json::from_str(SOKOLY_SAMPLE_REQ_REV2_JSON).expect("deserialize req2");
    let res2 = evaluate_analysis_request_artifact(
        &req2,
        "res-valley-relay-rev2",
        &solar.ephemeris,
        &solar.orientation,
    );
    assert!(res2.verify_revision_binding(&req2.design_revision));
    assert!(!res2.verify_revision_binding(&req1.design_revision));
    assert_eq!(res2.verify_against_request(&req2), Ok(()));
    assert_eq!(res2.job_status, AnalysisJobStatus::Completed);

    let AnalysisOutcomePayload::GeometricRelayReport(rep2) = &res2.outcome else {
        panic!("expected GeometricRelayReport for rev2");
    };
    assert_eq!(rep2.relay_windows.len(), 1);
    assert!((rep2.relay_windows[0].duration_s - 60.0).abs() < 1e-6);
    assert_eq!(rep2.direct_windows.len(), 1);
    assert!((rep2.direct_windows[0].duration_s - 60.0).abs() < 1e-6);

    let mut req_missing_dem = req2.clone();
    if let Some(dem) = &mut req_missing_dem.terrain_dem {
        dem.cells_m[7] = None;
    }
    let res_dem = evaluate_analysis_request_artifact(
        &req_missing_dem,
        "res-missing-dem-cell",
        &solar.ephemeris,
        &solar.orientation,
    );
    assert!(matches!(
        res_dem.outcome,
        AnalysisOutcomePayload::EvaluationFailed(AccessAnalysisError::DataCoverage(
            DataCoverageError::TerrainDemOutOfCoverage { .. }
        ))
    ));

    let mut req_msl = req2;
    if let sim_analysis::AnalysisWorkloadSpec::AircraftSatelliteGroundRelay {
        ground_station, ..
    } = &mut req_msl.workload
    {
        ground_station.altitude_spec = Some(AltitudeSpec {
            datum: VerticalDatumSpec::OrthometricMsl {
                geoid_model_id: "EGM2008".to_owned(),
                undulation_m: None,
            },
            value_m: 500.0,
        });
    }
    let res_msl = evaluate_analysis_request_artifact(
        &req_msl,
        "res-missing-geoid",
        &solar.ephemeris,
        &solar.orientation,
    );
    assert!(matches!(
        res_msl.outcome,
        AnalysisOutcomePayload::EvaluationFailed(AccessAnalysisError::DataCoverage(
            DataCoverageError::VerticalDatumUnavailable {
                datum_kind: VerticalDatumKind::OrthometricMsl,
                ..
            }
        ))
    ));
}
