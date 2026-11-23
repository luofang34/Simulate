//! Link budgets at interplanetary range with the signal-time model, and the reciprocity of the
//! path between two stations.

use glam::{DQuat, DVec3};
use sim_analysis::{
    AccessQuerySpec, AnalysisEndpoint, AntennaPointing, AntennaSpec, GroundStationModel,
    LinkBudgetModels, OccultationEnvironmentModel, ReceiverNoise, RfLinkSpec, RfReceiverSpec,
    RfTransmitterSpec, SensorFovModel, SignalTimeModel, VehicleSensorModel,
    evaluate_endpoint_link_budget,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{
    ASTRONOMICAL_UNIT_M, DerivativeOrder, DynamicalModelIdentity, EphemerisCatalog,
    KinematicTrajectoryNode, NaifId, SPEED_OF_LIGHT_M_S, SampledKinematicTrajectory, SensorMount,
    TdbEpoch, ValidEpochInterval,
};

fn link() -> RfLinkSpec {
    RfLinkSpec {
        carrier_frequency_hz: 8.4e9,
        bandwidth_hz: 1.0e4,
        data_rate_bps: 1.0e3,
        required_eb_n0_db: 2.0,
        polarization_mismatch_loss_db: 0.0,
        implementation_loss_db: 0.0,
        transmitter: RfTransmitterSpec {
            transmit_power_w: 20.0,
            antenna: AntennaSpec::Isotropic,
            pointing: AntennaPointing::FixedInMount,
            line_loss_db: 0.0,
        },
        receiver: RfReceiverSpec {
            antenna: AntennaSpec::Isotropic,
            pointing: AntennaPointing::FixedInMount,
            line_loss_db: 0.0,
            noise: ReceiverNoise::SystemTemperature {
                system_noise_temperature_k: 50.0,
            },
        },
        propagation: Vec::new(),
    }
}

fn query(start: TdbEpoch, signal_time: SignalTimeModel) -> AccessQuerySpec {
    AccessQuerySpec {
        query_interval: ValidEpochInterval::new(start, start.advanced_seconds(10.0)),
        sample_step_s: 10.0,
        boundary_time_tolerance_s: 1.0,
        min_range_m: None,
        max_range_m: None,
        signal_time,
    }
}

fn station(id: u32, lat_deg: f64) -> GroundStationModel {
    GroundStationModel {
        station_id: id,
        name: format!("station-{id}"),
        body: NaifId::EARTH,
        geodetic: Geodetic::from_degrees(lat_deg, -116.9, 3000.0),
        ellipsoid: BodyEllipsoid::WGS84,
        mount: SensorMount::IDENTITY,
        fov: SensorFovModel::ConicalHalfAngle {
            max_half_angle_rad: core::f64::consts::PI,
        },
        min_elevation_rad: -core::f64::consts::FRAC_PI_2,
    }
}

#[test]
fn swapping_transmitter_and_receiver_leaves_the_path_loss_unchanged() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar system");
    let (a, b) = (station(1, 35.0), station(2, 38.0));
    let spec = link();
    let start = TdbEpoch::new(820_497_600 + 424 * 86_400, 0);
    let run = |from: &GroundStationModel, to: &GroundStationModel| {
        let report = evaluate_endpoint_link_budget(
            AnalysisEndpoint::SurfaceStation {
                station: from,
                altitude_spec: None,
            },
            AnalysisEndpoint::SurfaceStation {
                station: to,
                altitude_spec: None,
            },
            &spec,
            &LinkBudgetModels::from_spec(&spec),
            &OccultationEnvironmentModel::earth_wgs84(),
            None,
            &query(start, SignalTimeModel::Geometric),
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("budget");
        report.samples[0].budget.clone().expect("accessible")
    };
    let forward = run(&a, &b);
    let reverse = run(&b, &a);
    assert!(forward.slant_range_m > 3.0e5);
    assert!((forward.slant_range_m - reverse.slant_range_m).abs() < 1e-6);
    assert!(
        (forward.free_space_path_loss_db - reverse.free_space_path_loss_db).abs() < 1e-9,
        "{} dB vs {} dB",
        forward.free_space_path_loss_db,
        reverse.free_space_path_loss_db
    );
    assert!((forward.margin_db - reverse.margin_db).abs() < 1e-9);
}

const PROBE_VELOCITY_M_S: DVec3 = DVec3::new(-8.0e3, 25.0e3, 4.0e3);

/// A probe one AU from Earth moving at constant velocity, so its position at any earlier
/// instant is known exactly.
#[cfg(test)]
fn probe_trajectory(
    solar: &BuiltinSolarSystem2026_2028,
    t0: TdbEpoch,
) -> SampledKinematicTrajectory {
    let velocity = PROBE_VELOCITY_M_S;
    let earth = solar
        .ephemeris
        .geometric_state_icrf(NaifId::EARTH, NaifId::SSB, t0, DerivativeOrder::Position)
        .expect("earth state")
        .pos_m;
    let start_pos = earth + DVec3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0);
    let node = |dt: f64| KinematicTrajectoryNode {
        epoch: t0.advanced_seconds(dt),
        pos_m: start_pos + velocity * dt,
        vel_m_s: velocity,
        q_body_to_axes: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
    };
    SampledKinematicTrajectory::from_icrf_nodes(
        9,
        NaifId::SSB,
        DynamicalModelIdentity::PlanetaryKinematicOrFlight { body: NaifId::SSB },
        vec![node(-2_000.0), node(0.0), node(2_000.0)],
    )
    .expect("trajectory")
}

#[test]
fn light_time_budget_uses_the_transmitter_where_it_was_at_emission() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar system");
    let t0 = TdbEpoch::new(820_497_600 + 424 * 86_400, 0);
    let velocity = PROBE_VELOCITY_M_S;
    let trajectory = probe_trajectory(&solar, t0);
    let sensor = VehicleSensorModel::omnidirectional_at_cg(1, "X-band");
    let ground = station(14, 35.4);
    let spec = link();
    let run = |signal_time| {
        // The probe transmits to the station on Earth.
        evaluate_endpoint_link_budget(
            AnalysisEndpoint::MovingPlatform {
                endpoint_id: 9,
                label: "probe",
                host_body: NaifId::EARTH,
                ellipsoid: BodyEllipsoid::WGS84,
                trajectory: &trajectory,
                attitude: Some(&trajectory),
                sensor: &sensor,
                min_local_elevation_rad: None,
            },
            AnalysisEndpoint::SurfaceStation {
                station: &ground,
                altitude_spec: None,
            },
            &spec,
            &LinkBudgetModels::from_spec(&spec),
            &OccultationEnvironmentModel { bodies: vec![] },
            None,
            &query(t0, signal_time),
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("budget")
    };
    let geometric = run(SignalTimeModel::Geometric);
    let light = run(SignalTimeModel::LightTime);
    let (g_sample, l_sample) = (&geometric.access.samples[0], &light.access.samples[0]);
    let (g, l) = (
        geometric.samples[0].budget.as_ref().expect("geometric"),
        light.samples[0].budget.as_ref().expect("light time"),
    );

    let tau = l.light_time_s.expect("light time reported");
    assert!(g.light_time_s.is_none());
    assert!((450.0..550.0).contains(&tau), "{tau} s at about 1 AU");
    assert!((tau - l.slant_range_m / SPEED_OF_LIGHT_M_S).abs() < 1e-6);
    assert!((l_sample.target_pos_icrf_m - g_sample.target_pos_icrf_m).length() < 1e-6);
    let expected_emission = g_sample.source_pos_icrf_m - velocity * tau;
    assert!(
        (l_sample.source_pos_icrf_m - expected_emission).length() < 1e-3,
        "emission position off by {} m",
        (l_sample.source_pos_icrf_m - expected_emission).length()
    );

    let moved_m = (velocity * tau).length();
    let range_change_m = (l.slant_range_m - g.slant_range_m).abs();
    assert!(moved_m > 1.0e7 && range_change_m <= moved_m + 1.0);
    let expected_db = 20.0 * (l.slant_range_m / g.slant_range_m).log10();
    let loss_change_db = l.free_space_path_loss_db - g.free_space_path_loss_db;
    assert!((loss_change_db - expected_db).abs() < 1e-9);
    assert!(g.free_space_path_loss_db > 270.0 && g.free_space_path_loss_db < 280.0);
}
