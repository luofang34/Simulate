use glam::DQuat;
use sim_core::SimInstant;
use sim_flight_chain::{FcAirData, FcTargets};
use sim_geo::{Geodetic, geodetic_to_ecef};

use super::*;

fn estimate() -> VehicleState {
    let position = Geodetic::from_degrees(41.7, -1.07, 2051.0);
    let frame = LocalFrame::at(&position);
    // Nose east, level: body forward on local east, down on local down.
    let body_to_ecef = glam::DMat3::from_cols(frame.east(), -frame.north(), -frame.up());
    VehicleState {
        position_ecef: geodetic_to_ecef(&position),
        velocity_ecef: frame.east() * 60.0,
        attitude: DQuat::from_mat3(&body_to_ecef),
        angular_rate_body: DVec3::ZERO,
    }
}

fn avionics() -> FcAvionics {
    FcAvionics {
        at: SimInstant::from_ticks_u128(1_000),
        estimate: Some(estimate()),
        specific_force_body_m_s2: Some(DVec3::new(0.1, 0.0, -9.8)),
        air: Some(FcAirData {
            indicated_m_s: 50.0,
            true_m_s: 55.0,
            static_pressure_pa: Some(79_000.0),
            fresh: true,
        }),
        mode: ControlMode::VelocityControl,
        armed: true,
        targets: FcTargets {
            heading_rad: Some(1.2),
            altitude_m: Some(2051.0),
            airspeed_m_s: Some(55.0),
            vertical_speed_m_s: None,
        },
    }
}

#[test]
fn the_air_data_state_no_setting_so_the_altitude_is_pressure_altitude() {
    let reading = air_data_reading(&avionics()).unwrap();
    assert_eq!(reading.value.altimeter_setting_hpa, None);
    assert_eq!(reading.value.static_pressure_pa, Some(79_000.0));
    assert!((reading.value.calibrated_airspeed_m_s() - 50.0).abs() < 1e-9);
    assert_eq!(reading.validity, SourceValidity::Valid);
    let mut held = avionics();
    held.air = held.air.map(|air| FcAirData {
        fresh: false,
        ..air
    });
    assert_eq!(
        air_data_reading(&held).unwrap().validity,
        SourceValidity::Stale
    );
    held.air = None;
    assert!(air_data_reading(&held).is_none());
}

#[test]
fn the_guidance_is_the_flight_controllers_targets_above_mean_sea_level() {
    let fc = avionics();
    let shown = guidance(&fc, &estimate(), 4);
    let undulation = egm96_undulation_m(41.7_f64.to_radians(), (-1.07_f64).to_radians());
    let altitude = shown.selections.altitude_m.unwrap();
    assert!((altitude - (2051.0 - undulation)).abs() < 1e-9);
    assert_eq!(shown.selections.heading_rad, 1.2);
    assert_eq!(shown.selections.baro_setting_hpa, None);
    assert_eq!(shown.source_epoch, 4);
    assert!(shown.autopilot.engaged);
    assert_eq!(shown.autopilot.lateral, ident("HDG"));
    assert_eq!(shown.autopilot.vertical, ident("ALT"));
    let mut level = fc;
    level.targets = FcTargets::default();
    let held = guidance(&level, &estimate(), 4);
    assert!((held.selections.heading_rad - core::f64::consts::FRAC_PI_2).abs() < 1e-9);
    assert_eq!(held.selections.altitude_m, None);
}
