//! S-band LEO downlink to a ground station: Doppler through a pass, occultation withholding
//! the budget, margin ordering, byte-for-byte determinism, and the sealed artifact path.

use glam::{DQuat, DVec3};
use sim_analysis::{
    AccessQuerySpec, AccessRestrictionReason, AnalysisEndpoint, AnalysisJobStatus,
    AnalysisOutcomePayload, AnalysisRequestArtifact, AnalysisWorkloadSpec, AntennaPointing,
    AntennaSpec, BudgetUnavailableReason, CoordinateFrameContract, DesignRevisionIdentity,
    GroundStationModel, LinkBudgetModels, LinkBudgetReport, LinkDirection,
    NumericalAccuracyContract, OccultationEnvironmentModel, ParabolicDishSpec, ReceiverNoise,
    RfLinkSpec, RfReceiverSpec, RfTransmitterSpec, SerializableGroundStationSpec,
    SerializablePlatformSpec, SignalTimeModel, TimeScaleContract, TrajectoryNodesFrame,
    VehicleSensorModel, evaluate_analysis_request_artifact, evaluate_endpoint_link_access,
    evaluate_endpoint_link_budget,
};
use sim_geo::Geodetic;
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{
    DynamicalModelIdentity, KinematicTrajectoryNode, NaifId, SPEED_OF_LIGHT_M_S,
    SampledKinematicTrajectory, TdbEpoch, ValidEpochInterval,
};

const MU_EARTH_M3_S2: f64 = 3.986_004_418e14;
const ORBIT_RADIUS_M: f64 = 6_878_137.0;
const CARRIER_HZ: f64 = 2.2e9;

fn day_start() -> TdbEpoch {
    TdbEpoch::new(820_497_600 + 424 * 86_400, 0)
}

/// Circular 500 km orbit inclined 51.6 degrees, sampled every ten seconds for a day.
fn leo_nodes() -> Vec<KinematicTrajectoryNode> {
    let n = (MU_EARTH_M3_S2 / ORBIT_RADIUS_M.powi(3)).sqrt();
    let inc = 51.6_f64.to_radians();
    (0..=8_640)
        .map(|k| {
            let t = f64::from(k) * 10.0;
            let (s, c) = (n * t).sin_cos();
            let (r, v) = (ORBIT_RADIUS_M, ORBIT_RADIUS_M * n);
            KinematicTrajectoryNode {
                epoch: day_start().advanced_seconds(t),
                pos_m: DVec3::new(r * c, r * s * inc.cos(), r * s * inc.sin()),
                vel_m_s: DVec3::new(-v * s, v * c * inc.cos(), v * c * inc.sin()),
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            }
        })
        .collect()
}

fn orbital_speed_m_s() -> f64 {
    (MU_EARTH_M3_S2 / ORBIT_RADIUS_M).sqrt()
}

fn station(min_elevation_rad: f64) -> GroundStationModel {
    GroundStationModel::earth_wgs84(
        14,
        "DSS-14 Goldstone",
        Geodetic::from_degrees(35.426, -116.89, 1000.0),
        min_elevation_rad,
    )
}

fn downlink() -> RfLinkSpec {
    RfLinkSpec {
        carrier_frequency_hz: CARRIER_HZ,
        bandwidth_hz: 2.0e6,
        data_rate_bps: 1.0e6,
        required_eb_n0_db: 9.6,
        polarization_mismatch_loss_db: 0.3,
        implementation_loss_db: 1.0,
        transmitter: RfTransmitterSpec {
            transmit_power_w: 5.0,
            antenna: AntennaSpec::Isotropic,
            pointing: AntennaPointing::FixedInMount,
            line_loss_db: 1.0,
        },
        receiver: RfReceiverSpec {
            antenna: AntennaSpec::ParabolicDish(ParabolicDishSpec {
                diameter_m: 11.0,
                aperture_efficiency: 0.6,
                sidelobe_floor_below_peak_db: 35.0,
            }),
            pointing: AntennaPointing::TrackingTarget {
                pointing_error_rad: 0.0,
            },
            line_loss_db: 0.5,
            noise: ReceiverNoise::SystemTemperature {
                system_noise_temperature_k: 120.0,
            },
        },
        propagation: Vec::new(),
    }
}

fn query(signal_time: SignalTimeModel) -> AccessQuerySpec {
    AccessQuerySpec {
        query_interval: ValidEpochInterval::new(
            day_start(),
            day_start().advanced_seconds(86_000.0),
        ),
        sample_step_s: 20.0,
        boundary_time_tolerance_s: 1.0,
        min_range_m: None,
        max_range_m: None,
        signal_time,
    }
}

struct Fixture {
    solar: BuiltinSolarSystem2026_2028,
    trajectory: SampledKinematicTrajectory,
    sensor: VehicleSensorModel,
}

#[cfg(test)]
impl Fixture {
    fn new() -> Self {
        Self {
            solar: BuiltinSolarSystem2026_2028::load_verified().expect("solar system"),
            trajectory: SampledKinematicTrajectory::from_icrf_nodes(
                7,
                NaifId::EARTH,
                DynamicalModelIdentity::PlanetaryKinematicOrFlight {
                    body: NaifId::EARTH,
                },
                leo_nodes(),
            )
            .expect("trajectory"),
            sensor: VehicleSensorModel::omnidirectional_at_cg(1, "S-band whip"),
        }
    }

    fn report(&self, station: &GroundStationModel) -> LinkBudgetReport {
        let link = downlink();
        let satellite = AnalysisEndpoint::MovingPlatform {
            endpoint_id: 7,
            label: "LEO-1",
            host_body: NaifId::EARTH,
            ellipsoid: sim_geo::BodyEllipsoid::WGS84,
            trajectory: &self.trajectory,
            attitude: Some(&self.trajectory),
            sensor: &self.sensor,
            min_local_elevation_rad: None,
        };
        let ground = AnalysisEndpoint::SurfaceStation {
            station,
            altitude_spec: None,
        };
        evaluate_endpoint_link_budget(
            satellite,
            ground,
            &link,
            &LinkBudgetModels::from_spec(&link),
            &OccultationEnvironmentModel::earth_wgs84(),
            None,
            &query(SignalTimeModel::Geometric),
            &self.solar.ephemeris,
            &self.solar.orientation,
        )
        .expect("link budget")
    }
}

/// The first run of consecutive epochs that carry a budget.
fn first_pass(report: &LinkBudgetReport) -> Vec<&sim_analysis::LinkBudget> {
    report
        .samples
        .iter()
        .skip_while(|s| s.budget.is_none())
        .map_while(|s| s.budget.as_ref())
        .collect()
}

#[test]
fn doppler_changes_sign_at_closest_approach_with_peak_near_orbital_radial_speed() {
    let fixture = Fixture::new();
    let report = fixture.report(&station(10.0_f64.to_radians()));
    let pass = first_pass(&report);
    assert!(pass.len() > 20, "pass has {} epochs", pass.len());

    let (first, last) = (pass[0], pass[pass.len() - 1]);
    assert!(first.range_rate_m_s < 0.0 && first.doppler_shift_hz > 0.0);
    assert!(last.range_rate_m_s > 0.0 && last.doppler_shift_hz < 0.0);

    let closest = pass
        .iter()
        .enumerate()
        .min_by(|a, b| a.1.slant_range_m.total_cmp(&b.1.slant_range_m))
        .map(|(i, _)| i)
        .expect("non-empty pass");
    let flips = pass
        .windows(2)
        .position(|w| w[0].range_rate_m_s < 0.0 && w[1].range_rate_m_s >= 0.0)
        .expect("range rate crosses zero");
    assert!(
        closest.abs_diff(flips) <= 1,
        "closest {closest}, flip {flips}"
    );

    let peak_hz = pass
        .iter()
        .map(|b| b.doppler_shift_hz.abs())
        .fold(0.0, f64::max);
    let bound_hz = CARRIER_HZ * orbital_speed_m_s() / SPEED_OF_LIGHT_M_S;
    assert!(
        peak_hz < 1.05 * bound_hz,
        "peak {peak_hz} Hz vs {bound_hz} Hz"
    );
    assert!(
        peak_hz > 0.5 * bound_hz,
        "peak {peak_hz} Hz vs {bound_hz} Hz"
    );
    for b in &pass {
        let expected = -CARRIER_HZ * b.range_rate_m_s / SPEED_OF_LIGHT_M_S;
        assert!((b.doppler_shift_hz - expected).abs() < 1e-6);
        assert!((b.received_frequency_hz - CARRIER_HZ - expected).abs() < 1e-6);
    }
}

#[test]
fn occulted_epochs_carry_no_budget_and_say_why() {
    let fixture = Fixture::new();
    let report = fixture.report(&station(-core::f64::consts::FRAC_PI_2));
    let mut occulted = 0;
    let mut budgeted = 0;
    for (access, budget) in report.access.samples.iter().zip(&report.samples) {
        assert_eq!(access.epoch, budget.epoch);
        let behind_earth = access
            .active_restrictions
            .iter()
            .any(|r| matches!(r, AccessRestrictionReason::OccultedByBody { .. }));
        if behind_earth {
            occulted += 1;
            assert!(budget.budget.is_none());
            assert_eq!(
                budget.unavailable,
                Some(BudgetUnavailableReason::NotAccessible)
            );
        }
        if access.accessible {
            budgeted += 1;
            assert!(budget.budget.is_some() && budget.unavailable.is_none());
        }
    }
    assert!(
        occulted > 100 && budgeted > 20,
        "{occulted} occulted, {budgeted} budgeted"
    );
}

#[test]
fn margin_is_monotone_in_range_over_the_accessible_epochs() {
    let fixture = Fixture::new();
    let report = fixture.report(&station(5.0_f64.to_radians()));
    let mut points: Vec<(f64, f64)> = report
        .samples
        .iter()
        .filter_map(|s| s.budget.as_ref())
        .map(|b| (b.slant_range_m, b.margin_db))
        .collect();
    assert!(points.len() > 50);
    points.sort_by(|a, b| a.0.total_cmp(&b.0));
    for pair in points.windows(2) {
        assert!(pair[1].1 <= pair[0].1 + 1e-9, "{pair:?}");
    }
}

#[test]
fn the_budget_is_stated_verified_only_where_it_was_evaluated() {
    let fixture = Fixture::new();
    let ground = station(10.0_f64.to_radians());
    let report = fixture.report(&ground);
    assert!(report.access.verification_scope.rf_link_budget_verified);
    let geometric_only = evaluate_endpoint_link_access(
        AnalysisEndpoint::MovingPlatform {
            endpoint_id: 7,
            label: "LEO-1",
            host_body: NaifId::EARTH,
            ellipsoid: sim_geo::BodyEllipsoid::WGS84,
            trajectory: &fixture.trajectory,
            attitude: Some(&fixture.trajectory),
            sensor: &fixture.sensor,
            min_local_elevation_rad: None,
        },
        AnalysisEndpoint::SurfaceStation {
            station: &ground,
            altitude_spec: None,
        },
        &OccultationEnvironmentModel::earth_wgs84(),
        None,
        &query(SignalTimeModel::Geometric),
        &fixture.solar.ephemeris,
        &fixture.solar.orientation,
    )
    .expect("access");
    assert!(!geometric_only.verification_scope.rf_link_budget_verified);
    assert_eq!(geometric_only.windows, report.access.windows);
}

#[test]
fn the_same_request_twice_gives_identical_bytes() {
    let fixture = Fixture::new();
    let ground = station(10.0_f64.to_radians());
    let first = serde_json::to_vec(&fixture.report(&ground)).expect("serialize");
    let second = serde_json::to_vec(&fixture.report(&ground)).expect("serialize");
    assert_eq!(first, second);
}

#[cfg(test)]
fn request(fixture: &Fixture) -> AnalysisRequestArtifact {
    let q = query(SignalTimeModel::Geometric);
    AnalysisRequestArtifact {
        schema_version: sim_analysis::ANALYSIS_CONTRACT_SCHEMA_VERSION,
        request_id: "req-leo-s-band".to_owned(),
        design_revision: DesignRevisionIdentity {
            design_id: "design-leo".to_owned(),
            revision_number: 1,
            design_digest_sha256: "00".repeat(32),
        },
        execution_candidate_digest: None,
        time_scale: TimeScaleContract::default(),
        coordinate_frames: CoordinateFrameContract::default(),
        numerical_accuracy: NumericalAccuracyContract {
            trajectory_interpolation_method: "CubicHermiteIcrf".to_owned(),
            requested_position_interpolation_error_m: None,
            requested_boundary_time_tolerance_s: None,
            window_boundary_time_tolerance_s: q.boundary_time_tolerance_s,
            terrain_chord_sample_step_m: None,
            max_terrain_ray_samples_per_chord: None,
            proven_position_interpolation_error_m: None,
            proven_window_boundary_time_tolerance_s: None,
        },
        query: q,
        occultation_env: OccultationEnvironmentModel::earth_wgs84(),
        terrain_dem: None,
        terrain_los_config: None,
        workload: AnalysisWorkloadSpec::EndpointPairLinkBudget {
            source_platform: SerializablePlatformSpec {
                endpoint_id: 7,
                label: "LEO-1".to_owned(),
                host_body: NaifId::EARTH,
                nodes_frame: TrajectoryNodesFrame::BodyCenteredIcrf {
                    body: NaifId::EARTH,
                },
                nodes: fixture.trajectory.nodes_icrf.clone(),
                min_local_elevation_rad: None,
            },
            source_sensor: fixture.sensor.clone(),
            target_station: SerializableGroundStationSpec {
                station: station(10.0_f64.to_radians()),
                altitude_spec: None,
            },
            direction: LinkDirection::Downlink,
            link: downlink(),
        },
    }
}

#[test]
fn link_budget_artifact_is_sealed_bound_and_states_rf_verification() {
    let fixture = Fixture::new();
    let req = request(&fixture);
    let json = serde_json::to_string(&req).expect("serialize request");
    let req: AnalysisRequestArtifact = serde_json::from_str(&json).expect("round trip");
    let run = || {
        evaluate_analysis_request_artifact(
            &req,
            "res-leo",
            &fixture.solar.ephemeris,
            &fixture.solar.orientation,
        )
    };
    let result = run();
    assert_eq!(result.job_status, AnalysisJobStatus::Completed);
    assert!(result.verification_scope.rf_link_budget_verified);
    assert_eq!(result.verify_against_request(&req), Ok(()));
    let AnalysisOutcomePayload::LinkBudgetReport(report) = &result.outcome else {
        panic!("expected a link budget report");
    };
    assert!(report.samples.iter().any(|s| s.budget.is_some()));
    assert_eq!(result, run());
    assert_eq!(
        serde_json::to_vec(&result).expect("bytes"),
        serde_json::to_vec(&run()).expect("bytes")
    );

    let mut invalid = req.clone();
    if let AnalysisWorkloadSpec::EndpointPairLinkBudget { link, .. } = &mut invalid.workload {
        link.transmitter.transmit_power_w = -1.0;
    }
    let refused = evaluate_analysis_request_artifact(
        &invalid,
        "res-bad",
        &fixture.solar.ephemeris,
        &fixture.solar.orientation,
    );
    assert!(matches!(
        refused.job_status,
        AnalysisJobStatus::Failed { .. }
    ));
    assert!(!refused.verification_scope.rf_link_budget_verified);
}
