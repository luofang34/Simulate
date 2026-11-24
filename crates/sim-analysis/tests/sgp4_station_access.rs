//! End-to-end check that catalog-tier `Sgp4CatalogTrajectory` plugs directly into `sim-analysis`.

use sim_analysis::{
    AccessQuerySpec, GroundStationModel, OccultationEnvironmentModel, SignalTimeModel,
    VehicleSensorModel, WindowSearchCompleteness, evaluate_station_access,
};
use sim_geo::Geodetic;
use sim_orbital::{BuiltinSolarSystem2026_2028, Sgp4CatalogTrajectory, TdbEpoch};
use sim_spacetime::{TickSpan, TranslationalTrajectorySource, ValidEpochInterval};

const ISS_OMM: &str = r#"{
    "OBJECT_NAME": "ISS (ZARYA)",
    "OBJECT_ID": "1998-067A",
    "EPOCH": "2026-01-01T00:00:00.000000",
    "MEAN_MOTION": 15.49560532,
    "ECCENTRICITY": 0.0001771,
    "INCLINATION": 51.6435,
    "RA_OF_ASC_NODE": 225.4004,
    "ARG_OF_PERICENTER": 44.9625,
    "MEAN_ANOMALY": 5.1087,
    "EPHEMERIS_TYPE": 0,
    "CLASSIFICATION_TYPE": "U",
    "NORAD_CAT_ID": 25544,
    "ELEMENT_SET_NO": 999,
    "REV_AT_EPOCH": 23587,
    "BSTAR": 0.0049645,
    "MEAN_MOTION_DOT": 0.00289036,
    "MEAN_MOTION_DDOT": 0
}"#;

#[test]
fn sgp4_catalog_trajectory_drives_station_access_analysis() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = TdbEpoch::JAN_1_2026_00Z_TDB;
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(6_000, 0));
    let interval = ValidEpochInterval::new(t0, t1);
    let sgp4 =
        Sgp4CatalogTrajectory::from_omm_json(25544, ISS_OMM, interval, solar.time_scales.clone())
            .expect("sgp4 omm");
    assert!(sgp4.model_identity().is_catalog_tier());

    let goldstone = GroundStationModel::earth_wgs84(
        1,
        "Goldstone",
        Geodetic {
            lat_rad: 35.426_67_f64.to_radians(),
            lon_rad: (-116.890_0_f64).to_radians(),
            height_m: 1_001.0,
        },
        5.0_f64.to_radians(),
    );
    let omni = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let occ = OccultationEnvironmentModel::earth_wgs84_and_moon();
    let query = AccessQuerySpec {
        query_interval: interval,
        sample_step_s: 30.0,
        boundary_time_tolerance_s: 1.0,
        min_range_m: None,
        max_range_m: None,
        signal_time: SignalTimeModel::Geometric,
    };

    let report = evaluate_station_access(
        &sgp4,
        None,
        &goldstone,
        &omni,
        &occ,
        &query,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("SGP4 catalog trajectory is accepted by sim-analysis");
    assert!(matches!(
        report.completeness,
        WindowSearchCompleteness::BracketRefinedOnGrid { .. }
            | WindowSearchCompleteness::SampledGridOnly { .. }
    ));
    assert!(!report.samples.is_empty());
}
