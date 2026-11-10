//! Verifies `sim-analysis` across both orbital (`sim-orbital::OrbitalTrajectoryProduct`)
//! and non-orbital (`sim-core::KinematicModel` -> `SampledKinematicTrajectory`) motion sources,
//! plus the 4-way outcome taxonomy (`AccessRestrictionReason`, `DataCoverageError`,
//! `ModelNotApplicableReason`, `WindowSearchCompleteness`).

use glam::{DMat3, DQuat, DVec3};
use sim_analysis::{
    AccessAnalysisError, AccessQuerySpec, AccessRestrictionReason, DataCoverageError,
    GroundStationModel, ModelNotApplicableReason, OccultationEnvironmentModel, SensorFovModel,
    VehicleSensorModel, WindowSearchCompleteness, evaluate_station_access,
};
use sim_core::{Block, KinematicInput, KinematicModel, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, CoastControlLaw, ContinuousPlantState, Dop853Tolerances,
    DynamicalMode, ForceModel, MassPropertiesModel, NaifId, OrbitalTrajectoryProduct,
    PlanningInputFingerprint, PropellantMode, ReactionWheelMode, TdbEpoch, VesselConfig,
    propagate_plant_interval,
};
use sim_spacetime::{
    BodyOrientationCatalog, DynamicalModelIdentity, KinematicTrajectoryNode,
    SampledKinematicTrajectory, SensorMount, TickSpan, ValidEpochInterval,
};

#[cfg(test)]
fn build_leo_product(
    solar: &BuiltinSolarSystem2026_2028,
    t0: TdbEpoch,
    t1: TdbEpoch,
) -> (OrbitalTrajectoryProduct, DVec3) {
    let r0 = 6_778_137.0;
    let v0 = (3.986_004_418e14_f64 / r0).sqrt();
    let plant0 = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(r0, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, v0, 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::new(0.0, 0.0, 0.01),
        propellant_mass_kg: 100.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let force_model = ForceModel::new(
        solar.constants.clone(),
        vec![NaifId::SUN, NaifId::EARTH, NaifId::MOON],
        Vec::new(),
        DynamicalMode::PrescribedEphemeris,
        0,
    )
    .expect("force model");
    let vessel_cfg = VesselConfig {
        mass_model: MassPropertiesModel::diagonal(1_000.0, [500.0; 3], [1.0; 3]),
        propellant_mode: PropellantMode::FiniteVariableMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 100.0,
            tau_max_nm: 10.0,
        },
        tolerances: Dop853Tolerances::default(),
        bounding_radius_m: 5.0,
        auto_soi_switch: false,
    };
    let prop_out = propagate_plant_interval(
        &plant0,
        t0,
        t1,
        Some(10.0),
        &vessel_cfg.tolerances,
        &vessel_cfg.mass_model,
        vessel_cfg.propellant_mode,
        vessel_cfg.rw_mode,
        &force_model,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )
    .expect("leo propagation");
    let model_identity = DynamicalModelIdentity::SolarSystemEphemerisNBody {
        force_model_hash: force_model.content_hash(),
    };
    let fp = PlanningInputFingerprint::compute(
        model_identity,
        &force_model,
        &solar.ephemeris,
        &solar.orientation,
        &vessel_cfg,
    );
    let orb_product = OrbitalTrajectoryProduct::from_segments(
        1,
        fp,
        vessel_cfg.mass_model.dry_mass_kg,
        plant0,
        prop_out.final_state,
        t0,
        prop_out.segments,
    );
    (orb_product, plant0.pos_rel_icrf_m)
}

#[cfg(test)]
fn build_tracking_station(
    solar: &BuiltinSolarSystem2026_2028,
    t0: TdbEpoch,
    pos_rel_icrf_m: DVec3,
) -> GroundStationModel {
    let rot0 = solar
        .orientation
        .body_rotation(NaifId::EARTH, t0)
        .expect("earth rot");
    let sub_sat_fixed = rot0.r_icrf_to_fixed * pos_rel_icrf_m;
    let sub_sat_geo = BodyEllipsoid::WGS84.fixed_to_geodetic(sub_sat_fixed);
    GroundStationModel::earth_wgs84(
        1,
        "Equatorial Tracking Station",
        Geodetic {
            lat_rad: sub_sat_geo.lat_rad,
            lon_rad: sub_sat_geo.lon_rad + 0.15,
            height_m: 50.0,
        },
        10.0_f64.to_radians(),
    )
}

#[cfg(test)]
fn build_aircraft_trajectory_past_station(
    solar: &BuiltinSolarSystem2026_2028,
    t0: TdbEpoch,
    sta_geo: Geodetic,
) -> SampledKinematicTrajectory {
    let start_geo = Geodetic {
        lat_rad: sta_geo.lat_rad,
        lon_rad: sta_geo.lon_rad - 0.001,
        height_m: 3_000.0,
    };
    let start_lf = LocalFrame::at(&start_geo);
    let init_vs = VehicleState {
        position_ecef: geodetic_to_ecef(&start_geo),
        velocity_ecef: start_lf.east() * 100.0,
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    };
    let mut kin = KinematicModel::new(init_vs);
    let mut nodes_fixed = Vec::new();
    for step_idx in 0..=30_u64 {
        let ep = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(step_idx, 0));
        let vs = kin.output();
        let geo = ecef_to_geodetic(vs.position_ecef);
        let lf = LocalFrame::at(&geo);
        let r_ned_to_enu = DMat3::from_cols(DVec3::Y, DVec3::X, DVec3::NEG_Z);
        let r_ned_to_fixed = lf.enu_to_ecef() * r_ned_to_enu;
        let q_ned_to_fixed = DQuat::from_mat3(&r_ned_to_fixed);
        let q_body_to_fixed = (q_ned_to_fixed * vs.attitude).normalize();

        nodes_fixed.push(KinematicTrajectoryNode {
            epoch: ep,
            pos_m: vs.position_ecef,
            vel_m_s: vs.velocity_ecef,
            q_body_to_axes: q_body_to_fixed,
            omega_body_rad_s: vs.angular_rate_body,
        });
        for _ in 0..120 {
            kin.step(&KinematicInput::default()).expect("kin step");
        }
    }
    SampledKinematicTrajectory::from_body_fixed_samples(
        2,
        NaifId::EARTH,
        &nodes_fixed,
        &solar.orientation,
    )
    .expect("aircraft trajectory from body-fixed ECEF samples")
}

#[test]
fn orbital_and_aircraft_trajectories_reuse_identical_station_access_interface() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = TdbEpoch::new(840_000_000, 0);
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(3_600, 0));
    let (orb_product, pos0) = build_leo_product(&solar, t0, t1);
    let station = build_tracking_station(&solar, t0, pos0);

    let omni_sensor = VehicleSensorModel::omnidirectional_at_cg(10, "Omni S-Band");
    let occ_env = OccultationEnvironmentModel::earth_wgs84_and_moon();
    let orb_query = AccessQuerySpec {
        query_interval: ValidEpochInterval::new(t0, t1),
        sample_step_s: 30.0,
        boundary_time_tolerance_s: 1.0e-3,
        min_range_m: None,
        max_range_m: None,
        signal_time: sim_analysis::SignalTimeModel::Geometric,
    };

    let orb_report = evaluate_station_access(
        &orb_product,
        Some(&orb_product),
        &station,
        &omni_sensor,
        &occ_env,
        &orb_query,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("orbital station access");
    assert!(!orb_report.windows.is_empty());
    assert!(matches!(
        orb_report.completeness,
        WindowSearchCompleteness::BracketRefinedOnGrid { .. }
    ));

    let aircraft_traj = build_aircraft_trajectory_past_station(&solar, t0, station.geodetic);
    let belly_mount = SensorMount::with_boresight_z(DVec3::new(0.0, 0.0, 1.5), DVec3::Z);
    let belly_sensor = VehicleSensorModel::directional(
        20,
        "Belly Downlink Antenna",
        belly_mount,
        DVec3::ZERO,
        SensorFovModel::OmnidirectionalHemisphere,
    );
    let aircraft_query = AccessQuerySpec {
        query_interval: ValidEpochInterval::new(
            t0,
            t0.add_span(TickSpan::from_seconds_and_subsec_ticks(30, 0)),
        ),
        sample_step_s: 5.0,
        boundary_time_tolerance_s: 0.1,
        min_range_m: None,
        max_range_m: Some(50_000.0),
        signal_time: sim_analysis::SignalTimeModel::Geometric,
    };

    let aircraft_report = evaluate_station_access(
        &aircraft_traj,
        Some(&aircraft_traj),
        &station,
        &belly_sensor,
        &occ_env,
        &aircraft_query,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("aircraft station access");

    assert!(!aircraft_report.windows.is_empty());
    let first_sample = aircraft_report.samples.first().expect("sample");
    let r_mat_cols = first_sample
        .geometry
        .r_vehicle_mount_to_station_mount
        .expect("mount alignment matrix");
    let r_mat = DMat3::from_cols_array_2d(&r_mat_cols);
    assert!((r_mat.determinant() - 1.0).abs() < 1.0e-12);
}

#[cfg(test)]
fn sample_goldstone_station() -> GroundStationModel {
    GroundStationModel::earth_wgs84(
        1,
        "Goldstone",
        Geodetic {
            lat_rad: 35.4267_f64.to_radians(),
            lon_rad: (-116.89_f64).to_radians(),
            height_m: 1_000.0,
        },
        15.0_f64.to_radians(),
    )
}

#[test]
fn outcome_taxonomy_rejects_inapplicable_models_and_missing_coverage() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = TdbEpoch::new(840_000_000, 0);
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(600, 0));
    let interval = ValidEpochInterval::new(t0, t1);
    let station = sample_goldstone_station();
    let omni_sensor = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let occ_env = OccultationEnvironmentModel::earth_and_moon_spherical();

    let cr3bp_traj = SampledKinematicTrajectory::from_icrf_nodes(
        99,
        NaifId::MOON,
        DynamicalModelIdentity::AutonomousCr3bp {
            primary: NaifId::EARTH,
            secondary: NaifId::MOON,
            mu_star: 0.01215,
        },
        vec![
            KinematicTrajectoryNode {
                epoch: t0,
                pos_m: DVec3::new(1.15, 0.0, 0.0),
                vel_m_s: DVec3::ZERO,
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            },
            KinematicTrajectoryNode {
                epoch: t1,
                pos_m: DVec3::new(1.15, 0.01, 0.0),
                vel_m_s: DVec3::ZERO,
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            },
        ],
    )
    .expect("cr3bp traj");

    let query = AccessQuerySpec {
        query_interval: interval,
        sample_step_s: 60.0,
        boundary_time_tolerance_s: 60.0,
        min_range_m: None,
        max_range_m: None,
        signal_time: sim_analysis::SignalTimeModel::Geometric,
    };
    let err_non_si = evaluate_station_access(
        &cr3bp_traj,
        None,
        &station,
        &omni_sensor,
        &occ_env,
        &query,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect_err("AutonomousCr3bp must be rejected as ModelNotApplicable");
    assert!(matches!(
        err_non_si,
        AccessAnalysisError::ModelNotApplicable(
            ModelNotApplicableReason::UnsupportedDynamicalModel { .. }
        )
    ));
}

#[cfg(test)]
fn build_antipodal_si_trajectory(t0: TdbEpoch, t1: TdbEpoch) -> SampledKinematicTrajectory {
    SampledKinematicTrajectory::from_icrf_nodes(
        100,
        NaifId::EARTH,
        DynamicalModelIdentity::PlanetaryKinematicOrFlight {
            body: NaifId::EARTH,
        },
        vec![
            KinematicTrajectoryNode {
                epoch: t0,
                pos_m: DVec3::new(-7_000_000.0, 0.0, 0.0),
                vel_m_s: DVec3::new(0.0, -7_500.0, 0.0),
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            },
            KinematicTrajectoryNode {
                epoch: t1,
                pos_m: DVec3::new(-6_500_000.0, -4_000_000.0, 0.0),
                vel_m_s: DVec3::new(0.0, -7_500.0, 0.0),
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            },
        ],
    )
    .expect("si traj")
}

#[test]
fn outcome_taxonomy_rejects_out_of_coverage_trajectory_and_attitude() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = TdbEpoch::new(840_000_000, 0);
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(600, 0));
    let interval = ValidEpochInterval::new(t0, t1);
    let station = sample_goldstone_station();
    let omni_sensor = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let occ_env = OccultationEnvironmentModel::earth_and_moon_spherical();
    let query = AccessQuerySpec {
        query_interval: interval,
        sample_step_s: 60.0,
        boundary_time_tolerance_s: 60.0,
        min_range_m: None,
        max_range_m: None,
        signal_time: sim_analysis::SignalTimeModel::Geometric,
    };
    let si_traj = build_antipodal_si_trajectory(t0, t1);

    let out_of_cov_query = AccessQuerySpec {
        query_interval: ValidEpochInterval::new(
            t0,
            t1.add_span(TickSpan::from_seconds_and_subsec_ticks(600, 0)),
        ),
        ..query
    };
    assert!(matches!(
        evaluate_station_access(
            &si_traj,
            None,
            &station,
            &omni_sensor,
            &occ_env,
            &out_of_cov_query,
            &solar.ephemeris,
            &solar.orientation,
        ),
        Err(AccessAnalysisError::DataCoverage(
            DataCoverageError::TrajectoryOutOfCoverage { .. }
        ))
    ));

    let dir_sensor = VehicleSensorModel::directional(
        2,
        "Narrow Camera",
        SensorMount::IDENTITY,
        DVec3::ZERO,
        SensorFovModel::ConicalHalfAngle {
            max_half_angle_rad: 0.1,
        },
    );
    assert!(matches!(
        evaluate_station_access(
            &si_traj,
            None,
            &station,
            &dir_sensor,
            &occ_env,
            &query,
            &solar.ephemeris,
            &solar.orientation,
        ),
        Err(AccessAnalysisError::DataCoverage(
            DataCoverageError::AttitudeOutOfCoverage { .. }
        ))
    ));
}

#[test]
fn outcome_taxonomy_reports_restrictions_and_sampled_grid_completeness() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = TdbEpoch::new(840_000_000, 0);
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(600, 0));
    let interval = ValidEpochInterval::new(t0, t1);
    let station = sample_goldstone_station();
    let omni_sensor = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let occ_env = OccultationEnvironmentModel::earth_and_moon_spherical();
    let query = AccessQuerySpec {
        query_interval: interval,
        sample_step_s: 60.0,
        boundary_time_tolerance_s: 60.0,
        min_range_m: None,
        max_range_m: None,
        signal_time: sim_analysis::SignalTimeModel::Geometric,
    };
    let si_traj = build_antipodal_si_trajectory(t0, t1);

    let grid_report = evaluate_station_access(
        &si_traj,
        None,
        &station,
        &omni_sensor,
        &occ_env,
        &query,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("grid report");
    assert_eq!(
        grid_report.completeness,
        WindowSearchCompleteness::SampledGridOnly {
            sample_step_s: 60.0
        }
    );
    assert!(grid_report.samples.iter().any(|s| {
        s.active_restrictions.iter().any(|r| {
            matches!(
                r,
                AccessRestrictionReason::BelowMinimumElevation { .. }
                    | AccessRestrictionReason::OccultedByBody { .. }
            )
        })
    }));

    let long_t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(250_000, 0));
    let long_traj = build_antipodal_si_trajectory(t0, long_t1);
    let excessive_grid_query = AccessQuerySpec {
        query_interval: ValidEpochInterval::new(t0, long_t1),
        sample_step_s: 1.0,
        boundary_time_tolerance_s: 1.0,
        min_range_m: None,
        max_range_m: None,
        signal_time: sim_analysis::SignalTimeModel::Geometric,
    };
    let step_cap_err = evaluate_station_access(
        &long_traj,
        None,
        &station,
        &omni_sensor,
        &occ_env,
        &excessive_grid_query,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect_err("query exceeding MAX_ACCESS_GRID_STEPS must fail closed instead of leaving gap");
    assert!(matches!(
        step_cap_err,
        AccessAnalysisError::ModelNotApplicable(ModelNotApplicableReason::QueryStepCountExceeded {
            requested_steps: 250_000,
            max_steps: 200_000,
            ..
        })
    ));
}
