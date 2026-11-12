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
    let shown = guidance(&fc, &estimate(), None, 4);
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
    let held = guidance(&level, &estimate(), None, 4);
    assert!((held.selections.heading_rad - core::f64::consts::FRAC_PI_2).abs() < 1e-9);
    assert_eq!(held.selections.altitude_m, None);
}

fn fix(ident: &str, lat_deg: f64) -> sim_flight_chain::DisplayFix {
    sim_flight_chain::DisplayFix {
        ident: ident.to_owned(),
        position: navigate_contract::GeodeticPosition::new(lat_deg.to_radians(), -0.017, 2050.0),
    }
}

fn display(hold: bool) -> NavigationDisplay {
    use sim_flight_chain::{
        DisplayWaypoint, HoldingLeg, HoldingPattern, HoldingPatternStatus, HoldingTurn, LegDisplay,
        NavQuality, ToFrom, WaypointProgress,
    };
    let waypoints = ["WP1", "WP2", "WP3"]
        .iter()
        .enumerate()
        .map(|(i, name)| DisplayWaypoint {
            fix: fix(name, 41.7 + 0.1 * i as f64),
            progress: WaypointProgress::Ahead,
        })
        .collect();
    NavigationDisplay {
        waypoints,
        active_waypoint: (!hold).then_some(1),
        plan_complete: hold,
        leg: Some(LegDisplay {
            from: (!hold).then(|| fix("WP1", 41.7)),
            to: fix(if hold { "WP3" } else { "WP2" }, 41.8),
            desired_track_rad: 0.5,
            cross_track_m: Some(50.0),
            vertical_deviation_m: None,
            distance_to_go_m: 3_704.0,
            time_to_go_s: Some(61.0),
            to_from: ToFrom::To,
            quality: NavQuality::Degraded,
        }),
        holding_pattern: hold.then(|| HoldingPatternStatus {
            pattern: HoldingPattern {
                fix_ident: "WP3".to_owned(),
                fix: fix("WP3", 41.9).position,
                inbound_course_rad: 0.5,
                turn: HoldingTurn::Right,
                leg_time_s: 60.0,
                turn_radius_m: 900.0,
                leg_length_m: 3_600.0,
            },
            entry: None,
            leg: HoldingLeg::Inbound,
            circuits: 2,
            outbound_time_s: 60.0,
        }),
    }
}

/// The HSI's lane carries Pilotage's leg: the desired track as the course,
/// the cross-track deviation, the distance and the two fixes; in the hold,
/// the inbound course to the holding fix with `HOLD` for the FROM name.
#[test]
fn the_hsi_lane_is_pilotages_leg() {
    let at = SimInstant::from_ticks_u128(46_080_000_000 * 90);
    let HostNavGuidance::PilotageLane {
        stamp,
        guidance,
        now_ms,
    } = navigation_lane(&display(false), at, 3)
    else {
        panic!("no lane");
    };
    assert_eq!(guidance.course_rad, 0.5);
    assert_eq!(guidance.lateral_deviation_m, 50.0);
    assert!(guidance.vertical_deviation_m.is_nan());
    assert_eq!(guidance.distance_to_waypoint_m, 3_704.0);
    assert_eq!((guidance.leg_index, guidance.waypoint_count), (1, 3));
    assert_eq!(guidance.to_ident, ident("WP2").unwrap_or_default());
    assert_eq!(guidance.from_ident, ident("WP1").unwrap_or_default());
    assert_eq!(guidance.solution_quality, 1);
    assert_eq!((stamp.epoch, now_ms), (3, 90_000.0));
    let HostNavGuidance::PilotageLane { guidance, .. } = navigation_lane(&display(true), at, 3)
    else {
        panic!("no lane in the hold");
    };
    assert_eq!(guidance.from_ident, ident("HOLD").unwrap_or_default());
    assert_eq!(guidance.to_ident, ident("WP3").unwrap_or_default());
    let mut idle = display(false);
    idle.leg = None;
    assert_eq!(navigation_lane(&idle, at, 3), HostNavGuidance::None);
}
