//! WP 3 regressions:
//! 1. Brief terrain occlusion strictly between two coarse grid samples splits bracket-refined
//!    windows and prevents false continuous-coverage certification.
//! 2. Analytical demo ephemeris/orientation catalogs are explicitly tagged (`is_certified = false`)
//!    in `DataProvenance`, never masquerading as precision kernels.
//! 3. A single materialized trajectory (`SampledKinematicTrajectory`) is reused across ground-station
//!    visibility, directional sensor `FOV` link access, and multi-hop satellite relay analysis.

use glam::{DMat3, DQuat, DVec3};
use sim_analysis::{
    AccessQuerySpec, AnalysisEndpoint, DemElevationGrid, GroundStationModel,
    OccultationEnvironmentModel, SensorFovModel, TerrainLosConfig, VehicleSensorModel,
    evaluate_endpoint_link_access, evaluate_shared_trajectory_access_suite,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{
    BodyOrientationCatalog, BodyRotationState, ContentHash, CoverageSet, DerivativeOrder,
    DynamicalModelIdentity, EphemerisCatalog, EphemerisError, FrameCode, KinematicStateIcrf,
    KinematicTrajectoryNode, NaifId, OrientationError, SampledKinematicTrajectory, SensorMount,
    SurfaceNavGrade, TdbEpoch, TdbSpanNs, TickSpan, TranslationalTrajectorySource,
    ValidEpochInterval,
};

struct AnalyticalDemoCatalog;

impl EphemerisCatalog for AnalyticalDemoCatalog {
    fn content_hash(&self) -> ContentHash {
        ContentHash::of_bytes(b"analytical-demo-ephemeris-v1")
    }

    fn catalog_model_id(&self) -> &str {
        "analytical-demo"
    }

    fn is_analytical_approximation(&self) -> bool {
        true
    }

    fn coverage(
        &self,
        _target: NaifId,
        _center: NaifId,
        _order: DerivativeOrder,
    ) -> Vec<TdbSpanNs> {
        CoverageSet::unbounded().spans().to_vec()
    }

    fn geometric_state_icrf(
        &self,
        _target: NaifId,
        _center: NaifId,
        _epoch: TdbEpoch,
        _order: DerivativeOrder,
    ) -> Result<KinematicStateIcrf, EphemerisError> {
        Ok(KinematicStateIcrf::ZERO)
    }

    fn segment_knots_in(
        &self,
        _target: NaifId,
        _center: NaifId,
        _start: TdbEpoch,
        _end: TdbEpoch,
    ) -> Vec<TdbEpoch> {
        Vec::new()
    }
}

impl BodyOrientationCatalog for AnalyticalDemoCatalog {
    fn content_hash(&self) -> ContentHash {
        ContentHash::of_bytes(b"analytical-demo-orientation-v1")
    }

    fn catalog_model_id(&self) -> &str {
        "analytical-demo-iau"
    }

    fn coverage(&self, body: NaifId, min_grade: SurfaceNavGrade) -> Vec<TdbSpanNs> {
        if body == NaifId::EARTH && min_grade == SurfaceNavGrade::ApproximateIau {
            CoverageSet::unbounded().spans().to_vec()
        } else {
            Vec::new()
        }
    }

    fn body_rotation(
        &self,
        body: NaifId,
        _epoch: TdbEpoch,
    ) -> Result<BodyRotationState, OrientationError> {
        if body != NaifId::EARTH {
            return Err(OrientationError::MissingBodyModel(body));
        }
        Ok(BodyRotationState {
            r_icrf_to_fixed: DMat3::IDENTITY,
            omega_body_rad_s: DVec3::new(0.0, 0.0, 7.292_115e-5),
            alpha_body_rad_s2: DVec3::ZERO,
            frame_code: FrameCode(10_013),
            grade: SurfaceNavGrade::ApproximateIau,
        })
    }
}

fn build_spire_test_aircraft_and_dem(
    solar: &BuiltinSolarSystem2026_2028,
    t0: TdbEpoch,
) -> Result<(SampledKinematicTrajectory, DemElevationGrid), sim_spacetime::TrajectoryQueryError> {
    let mut nodes_fixed = Vec::new();
    for (step_s, lat_rad, vel_z) in [(0_u64, -0.003, 1_275.0), (30, 0.003, 0.0), (60, 0.003, 0.0)] {
        let geo = Geodetic {
            lat_rad,
            lon_rad: -0.002,
            height_m: 600.0,
        };
        let pos_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&geo);
        nodes_fixed.push(KinematicTrajectoryNode {
            epoch: t0.add_span(TickSpan::from_seconds_and_subsec_ticks(step_s, 0)),
            pos_m: pos_ecef,
            vel_m_s: DVec3::new(0.0, 0.0, vel_z),
            q_body_to_axes: DQuat::IDENTITY,
            omega_body_rad_s: DVec3::ZERO,
        });
    }
    let aircraft_traj = SampledKinematicTrajectory::from_body_fixed_samples(
        101,
        NaifId::EARTH,
        &nodes_fixed,
        &solar.orientation,
    )?;

    let mut cells_m = vec![Some(50.0); 21 * 9];
    if let Some(cell) = cells_m.get_mut(10 * 9 + 4) {
        *cell = Some(2_500.0);
    }
    let dem = DemElevationGrid {
        dataset_id: "narrow-spire-dem".to_owned(),
        body: NaifId::EARTH,
        lat_min_rad: -0.004,
        lat_max_rad: 0.004,
        lon_min_rad: -0.0022,
        lon_max_rad: 0.0002,
        rows: 21,
        cols: 9,
        cells_m,
    };
    Ok((aircraft_traj, dem))
}

#[test]
fn brief_interior_occlusion_between_coarse_samples_splits_window_and_blocks_continuous_coverage() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = TdbEpoch::new(840_000_000, 0);
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(60, 0));
    let station = GroundStationModel::earth_wgs84(
        301,
        "Valley Station",
        Geodetic {
            lat_rad: 0.0,
            lon_rad: 0.0,
            height_m: 100.0,
        },
        0.01,
    );
    let (aircraft_traj, dem) =
        build_spire_test_aircraft_and_dem(&solar, t0).expect("aircraft and dem");
    let los_cfg = TerrainLosConfig {
        sample_step_m: 100.0,
        min_clearance_m: 10.0,
        max_terrain_interaction_altitude_m: 10_000.0,
        max_ray_samples: 10_000,
    };
    let omni_sensor = VehicleSensorModel::omnidirectional_at_cg(11, "Omni");
    let occ_env = OccultationEnvironmentModel::earth_wgs84_and_moon();
    let src_ep = AnalysisEndpoint::MovingPlatform {
        endpoint_id: 101,
        label: "Aircraft",
        host_body: NaifId::EARTH,
        ellipsoid: BodyEllipsoid::WGS84,
        trajectory: &aircraft_traj,
        attitude: Some(&aircraft_traj),
        sensor: &omni_sensor,
        min_local_elevation_rad: None,
    };
    let dst_ep = AnalysisEndpoint::SurfaceStation {
        station: &station,
        altitude_spec: None,
    };

    let make_query = |tol_s: f64| AccessQuerySpec {
        query_interval: ValidEpochInterval::new(t0, t1),
        sample_step_s: 30.0,
        boundary_time_tolerance_s: tol_s,
        min_range_m: None,
        max_range_m: None,
    };
    let unrefined_rep = evaluate_endpoint_link_access(
        src_ep,
        dst_ep,
        &occ_env,
        Some((&dem, &los_cfg)),
        &make_query(30.0),
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("unrefined eval");
    assert!(unrefined_rep.samples.iter().all(|s| s.accessible));
    assert!(
        !unrefined_rep
            .completeness
            .certifies_continuous_coverage(30.0)
    );

    let refined_rep = evaluate_endpoint_link_access(
        src_ep,
        dst_ep,
        &occ_env,
        Some((&dem, &los_cfg)),
        &make_query(0.05),
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("refined eval");
    assert!(refined_rep.samples.iter().all(|s| s.accessible));
    assert_eq!(refined_rep.windows.len(), 2);
    let total_access_s: f64 = refined_rep.windows.iter().map(|w| w.duration_s).sum();
    assert!(total_access_s < 55.0);
    assert!(refined_rep.completeness.certifies_continuous_coverage(2.0));
    assert!(!refined_rep.completeness.certifies_continuous_coverage(0.5));
}

#[test]
fn analytical_demo_catalogs_are_explicitly_marked_uncertified_in_provenance() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let demo = AnalyticalDemoCatalog;
    assert!(!solar.ephemeris.is_analytical_approximation());
    assert!(demo.is_analytical_approximation());
    assert_eq!(
        solar.orientation.max_supported_grade(NaifId::EARTH),
        Some(SurfaceNavGrade::PrecisionGeodetic)
    );
    assert_eq!(
        demo.max_supported_grade(NaifId::EARTH),
        Some(SurfaceNavGrade::ApproximateIau)
    );

    let t0 = TdbEpoch::new(840_000_000, 0);
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(20, 0));
    let station = GroundStationModel::earth_wgs84(
        1,
        "Test Station",
        Geodetic {
            lat_rad: 0.0,
            lon_rad: 0.0,
            height_m: 10.0,
        },
        0.05,
    );
    let traj = SampledKinematicTrajectory::from_icrf_nodes(
        10,
        NaifId::EARTH,
        DynamicalModelIdentity::PlanetaryKinematicOrFlight {
            body: NaifId::EARTH,
        },
        vec![
            KinematicTrajectoryNode {
                epoch: t0,
                pos_m: DVec3::new(7_000_000.0, 0.0, 0.0),
                vel_m_s: DVec3::ZERO,
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            },
            KinematicTrajectoryNode {
                epoch: t1,
                pos_m: DVec3::new(7_000_000.0, 0.0, 0.0),
                vel_m_s: DVec3::ZERO,
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            },
        ],
    )
    .expect("traj");
    let omni = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let occ_env = OccultationEnvironmentModel::earth_and_moon_spherical();
    let query = AccessQuerySpec {
        query_interval: ValidEpochInterval::new(t0, t1),
        sample_step_s: 10.0,
        boundary_time_tolerance_s: 1.0,
        min_range_m: None,
        max_range_m: None,
    };

    let demo_rep = sim_analysis::evaluate_station_access(
        &traj,
        Some(&traj),
        &station,
        &omni,
        &occ_env,
        &query,
        &demo,
        &demo,
    )
    .expect("demo report");
    assert!(demo_rep.provenance.iter().any(|p| {
        p.dataset_name == "ephemeris:analytical-demo"
            && p.dataset_edition == "analytical-approximation"
    }));
    assert!(demo_rep.provenance.iter().any(|p| {
        p.dataset_name == "orientation:analytical-demo-iau"
            && p.dataset_edition == "approximate-iau"
    }));
}

fn build_stationary_ecef_platform(
    solar: &BuiltinSolarSystem2026_2028,
    endpoint_id: u64,
    t0: TdbEpoch,
    t1: TdbEpoch,
    height_m: f64,
) -> Result<SampledKinematicTrajectory, sim_spacetime::TrajectoryQueryError> {
    let pos_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&Geodetic {
        lat_rad: 0.0,
        lon_rad: 0.0,
        height_m,
    });
    SampledKinematicTrajectory::from_body_fixed_samples(
        endpoint_id,
        NaifId::EARTH,
        &[
            KinematicTrajectoryNode {
                epoch: t0,
                pos_m: pos_ecef,
                vel_m_s: DVec3::ZERO,
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            },
            KinematicTrajectoryNode {
                epoch: t1,
                pos_m: pos_ecef,
                vel_m_s: DVec3::ZERO,
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            },
        ],
        &solar.orientation,
    )
}

fn make_cone_sensor(
    id: u32,
    label: &str,
    boresight: DVec3,
    half_angle_rad: f64,
) -> VehicleSensorModel {
    VehicleSensorModel::directional(
        id,
        label,
        SensorMount::with_boresight_z(DVec3::ZERO, boresight),
        DVec3::ZERO,
        SensorFovModel::ConicalHalfAngle {
            max_half_angle_rad: half_angle_rad,
        },
    )
}

#[test]
fn shared_materialized_trajectory_reused_across_visibility_fov_and_relay() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = TdbEpoch::new(840_000_000, 0);
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(40, 0));
    let station = GroundStationModel::earth_wgs84(
        301,
        "Ground Hub",
        Geodetic {
            lat_rad: 0.0,
            lon_rad: 0.0,
            height_m: 100.0,
        },
        0.01,
    );
    let aircraft_traj =
        build_stationary_ecef_platform(&solar, 101, t0, t1, 5_000.0).expect("aircraft");
    let sat_traj = build_stationary_ecef_platform(&solar, 201, t0, t1, 600_000.0).expect("sat");
    let belly_sensor = make_cone_sensor(12, "Belly Camera", DVec3::NEG_X, 0.35);
    let dorsal_sensor = make_cone_sensor(11, "Dorsal Satcom", DVec3::X, 0.35);
    let sat_nadir_sensor = make_cone_sensor(21, "Sat Nadir Beam", DVec3::NEG_X, 0.5);
    let sat_ep = AnalysisEndpoint::MovingPlatform {
        endpoint_id: 201,
        label: "Relay Sat",
        host_body: NaifId::EARTH,
        ellipsoid: BodyEllipsoid::WGS84,
        trajectory: &sat_traj,
        attitude: Some(&sat_traj),
        sensor: &sat_nadir_sensor,
        min_local_elevation_rad: None,
    };
    let query = AccessQuerySpec {
        query_interval: ValidEpochInterval::new(t0, t1),
        sample_step_s: 10.0,
        boundary_time_tolerance_s: 0.5,
        min_range_m: None,
        max_range_m: None,
    };
    let occ_env = OccultationEnvironmentModel::earth_wgs84_and_moon();
    let suite = evaluate_shared_trajectory_access_suite(
        101,
        "Aircraft-101",
        NaifId::EARTH,
        BodyEllipsoid::WGS84,
        &aircraft_traj,
        Some(&aircraft_traj),
        &belly_sensor,
        &station,
        None,
        Some((sat_ep, sat_ep, &dorsal_sensor)),
        &occ_env,
        None,
        &query,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("shared trajectory suite");

    let expected_prov = aircraft_traj.provenance();
    let vis_prov = suite.station_visibility.provenance.first();
    let fov_prov = suite.sensor_fov_link.provenance.first();
    assert_eq!(vis_prov, Some(expected_prov));
    assert_eq!(fov_prov, Some(expected_prov));
    let relay_rep = suite.relay_access.expect("relay report");
    assert_eq!(relay_rep.provenance.first(), Some(expected_prov));
    let scope = suite.sensor_fov_link.verification_scope;
    assert!(!scope.rf_link_budget_verified && !scope.store_and_forward_modeled);
    assert_eq!(suite.station_visibility.windows.len(), 1);
    assert_eq!(suite.sensor_fov_link.windows.len(), 1);
    assert_eq!(relay_rep.relay_windows.len(), 1);
}
