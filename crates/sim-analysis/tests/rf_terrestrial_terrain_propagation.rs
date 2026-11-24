//! End-to-end terrestrial link budget over DEM terrain with `world-propagation` diffraction,
//! NTIA/ITS ITM loss, and ITU-R F.1245 antenna patterns through `evaluate_endpoint_link_budget`.

use glam::{DQuat, DVec3};
use sim_analysis::{
    AccessQuerySpec, AccessRestrictionReason, AnalysisEndpoint, AntennaPointing, AntennaSpec,
    BudgetUnavailableReason, Climate, DEFAULT_EFFECTIVE_EARTH_RADIUS_M,
    DeltaBullingtonPropagationLoss, DemElevationGrid, ExceedancePercentages, F1245Pattern, Ground,
    GroundStationModel, ItmPropagationLoss, LinkBudgetModels, LinkBudgetReport,
    OccultationEnvironmentModel, OccultingBodySpec, OccultingShape, Polarisation, ReceiverNoise,
    RfLinkSpec, RfReceiverSpec, RfTransmitterSpec, SignalTimeModel, TerrainLosConfig,
    VariabilityMode, VehicleSensorModel, evaluate_endpoint_link_budget,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{
    BodyOrientationCatalog, DynamicalModelIdentity, KinematicTrajectoryNode, NaifId,
    SampledKinematicTrajectory, TdbEpoch, ValidEpochInterval,
};

fn earth_only_env() -> OccultationEnvironmentModel {
    OccultationEnvironmentModel {
        bodies: vec![OccultingBodySpec {
            body: NaifId::EARTH,
            shape: OccultingShape::Ellipsoid(BodyEllipsoid::WGS84),
        }],
    }
}

fn ridge_dem(peak_height_m: f64) -> DemElevationGrid {
    let mut cells = Vec::with_capacity(3 * 9);
    for _row in 0..3 {
        for col in 0..9 {
            let h = if col == 4 { peak_height_m } else { 100.0 };
            cells.push(Some(h));
        }
    }
    DemElevationGrid {
        dataset_id: "test-terrestrial-ridge".to_owned(),
        body: NaifId::EARTH,
        lat_min_rad: -0.002,
        lat_max_rad: 0.002,
        lon_min_rad: -0.004,
        lon_max_rad: 0.004,
        rows: 3,
        cols: 9,
        cells_m: cells,
    }
}

fn terrestrial_link(freq_hz: f64) -> RfLinkSpec {
    RfLinkSpec {
        carrier_frequency_hz: freq_hz,
        bandwidth_hz: 1.0e6,
        data_rate_bps: 1.0e6,
        required_eb_n0_db: 10.0,
        polarization_mismatch_loss_db: 0.0,
        implementation_loss_db: 0.0,
        transmitter: RfTransmitterSpec {
            transmit_power_w: 10.0,
            antenna: AntennaSpec::Isotropic,
            pointing: AntennaPointing::TrackingTarget {
                pointing_error_rad: 0.0,
            },
            line_loss_db: 0.5,
        },
        receiver: RfReceiverSpec {
            antenna: AntennaSpec::Isotropic,
            pointing: AntennaPointing::TrackingTarget {
                pointing_error_rad: 0.0,
            },
            line_loss_db: 0.5,
            noise: ReceiverNoise::SystemTemperature {
                system_noise_temperature_k: 290.0,
            },
        },
        propagation: Vec::new(),
    }
}

fn low_platform_trajectory(
    solar: &BuiltinSolarSystem2026_2028,
    epoch: TdbEpoch,
    dst: &Geodetic,
) -> Result<SampledKinematicTrajectory, Box<dyn std::error::Error>> {
    let fixed = BodyEllipsoid::WGS84.geodetic_to_fixed(dst);
    let nodes = (0..=2)
        .map(|k| {
            let ep = epoch.advanced_seconds(f64::from(k));
            let rot = solar.orientation.body_rotation(NaifId::EARTH, ep)?;
            Ok::<_, Box<dyn std::error::Error>>(KinematicTrajectoryNode {
                epoch: ep,
                pos_m: rot.r_icrf_to_fixed.transpose() * fixed,
                vel_m_s: DVec3::ZERO,
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(SampledKinematicTrajectory::from_icrf_nodes(
        2,
        NaifId::EARTH,
        DynamicalModelIdentity::PlanetaryKinematicOrFlight {
            body: NaifId::EARTH,
        },
        nodes,
    )?)
}

fn run_ridge_case(
    solar: &BuiltinSolarSystem2026_2028,
    link: &RfLinkSpec,
    models: &LinkBudgetModels<'_>,
    peak_height_m: f64,
) -> Result<LinkBudgetReport, Box<dyn std::error::Error>> {
    let epoch = TdbEpoch::new(840_000_000, 0);
    let tx_station = GroundStationModel::earth_wgs84(
        1,
        "West Ridge Terminal",
        Geodetic {
            lat_rad: 0.0,
            lon_rad: -0.001_5,
            height_m: 130.0,
        },
        0.0,
    );
    let trajectory = low_platform_trajectory(
        solar,
        epoch,
        &Geodetic {
            lat_rad: 0.0,
            lon_rad: 0.001_5,
            height_m: 250.0,
        },
    )?;
    let sensor = VehicleSensorModel::omnidirectional_at_cg(2, "UHF airborne relay");
    let terrain_cfg = TerrainLosConfig {
        sample_step_m: 150.0,
        min_clearance_m: 0.0,
        max_terrain_interaction_altitude_m: 5_000.0,
        max_ray_samples: 1_000,
    };
    let query = AccessQuerySpec {
        query_interval: ValidEpochInterval::new(epoch, epoch),
        sample_step_s: 1.0,
        boundary_time_tolerance_s: 0.5,
        min_range_m: None,
        max_range_m: Some(100_000.0),
        signal_time: SignalTimeModel::Geometric,
    };
    Ok(evaluate_endpoint_link_budget(
        AnalysisEndpoint::SurfaceStation {
            station: &tx_station,
            altitude_spec: None,
        },
        AnalysisEndpoint::MovingPlatform {
            endpoint_id: 2,
            label: "Low UAS Relay",
            host_body: NaifId::EARTH,
            ellipsoid: BodyEllipsoid::WGS84,
            trajectory: &trajectory,
            attitude: None,
            sensor: &sensor,
            min_local_elevation_rad: None,
        },
        link,
        models,
        &earth_only_env(),
        Some((&ridge_dem(peak_height_m), &terrain_cfg)),
        &query,
        &solar.ephemeris,
        &solar.orientation,
    )?)
}

#[test]
fn endpoint_link_budget_samples_dem_terrain_for_delta_bullington_and_itm()
-> Result<(), Box<dyn std::error::Error>> {
    let solar = BuiltinSolarSystem2026_2028::load_verified()?;
    let freq_hz = 1.2e9;
    let bullington =
        DeltaBullingtonPropagationLoss::new(DEFAULT_EFFECTIVE_EARTH_RADIUS_M, Ground::LAND)?;
    let itm = ItmPropagationLoss::new(
        Climate::ContinentalTemperate,
        301.0,
        Ground::LAND,
        VariabilityMode::SingleMessage,
    )?;
    let f1245 = F1245Pattern::new(0.9, freq_hz)?;

    let link = terrestrial_link(freq_hz);
    let mut models = LinkBudgetModels::from_spec(&link)
        .with_polarisation(Polarisation::Vertical)
        .with_exceedance(ExceedancePercentages::try_new(50.0, 50.0, 50.0)?);
    models.transmit_antenna = &f1245;
    models.receive_antenna = &f1245;
    models.propagation.push(&bullington);
    models.propagation.push(&itm);

    let flat = run_ridge_case(&solar, &link, &models, 100.0)?;
    let grazing = run_ridge_case(&solar, &link, &models, 180.0)?;
    let blocked = run_ridge_case(&solar, &link, &models, 280.0)?;

    let flat_b = flat
        .samples
        .first()
        .and_then(|s| s.budget.as_ref())
        .ok_or("missing flat budget")?;
    let graz_b = grazing
        .samples
        .first()
        .and_then(|s| s.budget.as_ref())
        .ok_or("missing grazing budget")?;
    assert_eq!(flat_b.propagation_losses.len(), 2);
    assert_eq!(graz_b.propagation_losses.len(), 2);
    assert!(
        graz_b.propagation_losses[0].loss_db > flat_b.propagation_losses[0].loss_db + 3.0,
        "Bullington loss did not reflect Fresnel ridge intrusion: flat={}, grazing={}",
        flat_b.propagation_losses[0].loss_db,
        graz_b.propagation_losses[0].loss_db
    );
    assert!(graz_b.margin_db < flat_b.margin_db - 3.0);

    let blocked_access = blocked
        .access
        .samples
        .first()
        .ok_or("missing blocked access")?;
    assert!(!blocked_access.accessible);
    assert!(
        blocked_access
            .active_restrictions
            .iter()
            .any(|r| matches!(r, AccessRestrictionReason::OccultedByTerrain { .. }))
    );
    assert_eq!(
        blocked
            .samples
            .first()
            .ok_or("missing blocked sample")?
            .unavailable,
        Some(BudgetUnavailableReason::NotAccessible)
    );
    Ok(())
}
