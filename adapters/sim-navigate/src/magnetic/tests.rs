use std::sync::Arc;

use navigate_contract::{GeodeticPosition, SensorClass, SourceEpoch, SourceId};
use navigate_fusion::ObservationValue;
use sim_bodies::{
    AnomalyOverlay, GridGeometry, HarmonicMagnetism, MagnetismWithAnomaly, OutsideCoverage,
};
use sim_core::{SimInstant, VehicleState};
use sim_devices::MagnetometerSpec;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, MagneticField};

use super::{MagneticAnomalyConfig, MagneticAnomalyMap, SimulatedMagnetometer};
use crate::{Estimate, MasterClockBridge};

fn delaware_site() -> Geodetic {
    Geodetic::from_degrees(39.05, -75.50, 1_000.0)
}

fn planar_overlay(north_slope_nt_per_m: f64, east_slope_nt_per_m: f64) -> AnomalyOverlay {
    let origin = Geodetic::from_degrees(39.0, -75.6, 0.0);
    let origin_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&origin);
    let frame = LocalFrame::at(&origin);
    let geometry = GridGeometry {
        south_deg: 38.9,
        west_deg: -75.7,
        latitude_step_deg: 0.05,
        longitude_step_deg: 0.05,
        rows: 7,
        columns: 7,
    };
    let mut values = Vec::with_capacity(7 * 7 * 3);
    for row in 0..geometry.rows {
        let lat_deg = geometry.south_deg + row as f64 * geometry.latitude_step_deg;
        for col in 0..geometry.columns {
            let lon_deg = geometry.west_deg + col as f64 * geometry.longitude_step_deg;
            let p = BodyEllipsoid::WGS84
                .geodetic_to_fixed(&Geodetic::from_degrees(lat_deg, lon_deg, 0.0));
            let offset = frame.ecef_to_ned_vector(p - origin_ecef);
            let bn = north_slope_nt_per_m * offset.x + east_slope_nt_per_m * offset.y;
            values.extend_from_slice(&[bn, 0.0, 0.0]);
        }
    }
    AnomalyOverlay::new(
        geometry,
        values,
        &BodyEllipsoid::WGS84,
        OutsideCoverage::Refuse,
    )
    .unwrap()
}

#[test]
fn simulated_magnetometer_reads_world_with_anomaly_overlay() {
    let main: Arc<dyn MagneticField> = Arc::new(HarmonicMagnetism::igrf14().unwrap());
    let overlay = planar_overlay(0.5, -0.3);
    let world: Arc<dyn MagneticField> = Arc::new(MagnetismWithAnomaly {
        base: Arc::clone(&main),
        overlays: vec![overlay],
    });
    let state = VehicleState::level_flight(&delaware_site(), 0.7, 50.0);
    let mut main_sensor =
        SimulatedMagnetometer::new(MagnetometerSpec::IDEAL, 1, 2026.5, Arc::clone(&main)).unwrap();
    let mut world_sensor =
        SimulatedMagnetometer::new(MagnetometerSpec::IDEAL, 1, 2026.5, world).unwrap();
    let b_main = main_sensor.sense_body_ut(&state).unwrap();
    let b_world = world_sensor.sense_body_ut(&state).unwrap();
    let fixed = BodyEllipsoid::WGS84.geodetic_to_fixed(&delaware_site());
    let expected_delta = (state.attitude.inverse()
        * planar_overlay(0.5, -0.3).anomaly_fixed(fixed).unwrap())
        / 1_000.0;
    assert!(expected_delta.length() > 0.1, "{expected_delta}");
    assert!(((b_world - b_main) - expected_delta).length() < 1e-12);
}

#[test]
fn main_field_only_map_has_zero_crustal_gradient_and_outside_returns_none() {
    let main: Arc<dyn MagneticField> = Arc::new(HarmonicMagnetism::igrf14().unwrap());
    let mut flat_map = MagneticAnomalyMap::new(
        Arc::clone(&main),
        Vec::new(),
        MagneticAnomalyConfig::default(),
        MasterClockBridge::default(),
        SourceId::new(10),
        SourceEpoch::new(0),
    )
    .unwrap();
    let site = delaware_site();
    let est = Estimate::new(GeodeticPosition::new(
        site.lat_rad,
        site.lon_rad,
        site.height_m,
    ));
    let total_t = main
        .field_fixed_nt(BodyEllipsoid::WGS84.geodetic_to_fixed(&site), 2026.5)
        .unwrap()
        .length()
        * 1e-9;
    let obs = flat_map
        .observe_total_field(SimInstant::START, &est, total_t)
        .unwrap()
        .unwrap();
    let ObservationValue::MagneticAnomaly {
        measured_t,
        map_t_at_lin,
        gradient_ned_t_per_m,
        ..
    } = obs.value
    else {
        panic!("expected magnetic anomaly");
    };
    assert!(measured_t.abs() < 1e-18);
    assert!(map_t_at_lin.abs() < 1e-18);
    assert!(gradient_ned_t_per_m.iter().all(|g| g.abs() < 1e-18));
    assert!(obs.composition.contains(SensorClass::MagneticAnomaly));

    let mut covered_map = MagneticAnomalyMap::new(
        main,
        vec![planar_overlay(0.5, -0.3)],
        MagneticAnomalyConfig::default(),
        MasterClockBridge::default(),
        SourceId::new(11),
        SourceEpoch::new(0),
    )
    .unwrap();
    let inside = covered_map
        .observe_total_field(SimInstant::START, &est, total_t)
        .unwrap()
        .unwrap();
    let ObservationValue::MagneticAnomaly {
        gradient_ned_t_per_m: g,
        ..
    } = inside.value
    else {
        panic!("expected magnetic anomaly");
    };
    assert!(g[0].hypot(g[1]) > 1e-11, "{g:?}");
    let outside = Estimate::new(GeodeticPosition::new(
        10.0_f64.to_radians(),
        20.0_f64.to_radians(),
        1_000.0,
    ));
    assert!(
        covered_map
            .observe_total_field(SimInstant::START, &outside, total_t)
            .unwrap()
            .is_none()
    );
}
