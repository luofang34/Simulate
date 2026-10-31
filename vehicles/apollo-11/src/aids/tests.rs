use std::f64::consts::FRAC_PI_2;

use glam::{DQuat, DVec3};
use sim_core::{Block, CrewObservation, CrewStationPort, SimInstant, TICKS_PER_SECOND, TickSpan};
use sim_geo::BodyEllipsoid;

use super::*;
use crate::{InertialSiteSample, SurfacePlane};

const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 10);
const G: f64 = 9.806_65;

const SURFACE: SurfacePlane = SurfacePlane {
    point_fixed_m: DVec3::ZERO,
    normal_fixed: DVec3::Z,
};

fn tips(heights: [f64; 3]) -> LunarContactInput {
    LunarContactInput {
        tips_fixed_m: heights.map(|height| DVec3::new(1.0, 2.0, height)),
        surface: SURFACE,
    }
}

#[test]
fn any_one_probe_lights_the_lamp_and_it_stays_lit() {
    let mut probes = LunarContactProbeBlock::new(PERIOD).unwrap();
    probes.step(&tips([0.1, 0.2, 0.3])).unwrap();
    assert!(!probes.output().lunar_contact);
    probes.step(&tips([0.1, -0.01, 0.3])).unwrap();
    assert!(probes.output().lunar_contact);
    probes.step(&tips([5.0, 5.0, 5.0])).unwrap();
    assert!(probes.output().lunar_contact);

    let mut port = CrewStationPort::new();
    probes.publish_lamp(SimInstant::START, &mut port).unwrap();
    assert_eq!(
        port.latest_observations().first().unwrap().observation,
        CrewObservation::AnnunciatorLamp {
            lamp_id: CONTACT_LIGHT_LAMP,
            lit: true
        }
    );
}

#[test]
fn a_probe_snapshot_restores_the_latch_and_bad_input_is_refused() {
    let mut probes = LunarContactProbeBlock::new(PERIOD).unwrap();
    let saved = probes.snapshot();
    probes.step(&tips([-1.0, 1.0, 1.0])).unwrap();
    probes.restore(&saved);
    assert!(!probes.output().lunar_contact);
    assert!(probes.step(&tips([f64::NAN, 1.0, 1.0])).is_err());
    let bent = LunarContactInput {
        surface: SurfacePlane {
            normal_fixed: DVec3::ZERO,
            ..SURFACE
        },
        ..tips([1.0; 3])
    };
    assert!(probes.step(&bent).is_err());
    assert!(LunarContactProbeBlock::new(TickSpan::ZERO).is_err());
}

#[test]
fn the_plane_beneath_a_position_is_level_and_at_terrain_height() {
    let body = BodyEllipsoid::MOON_IAU;
    let position = DVec3::new(1.0, 2.0, 2.0).normalize() * (body.semi_major_m + 5_000.0);
    let plane = SurfacePlane::level_beneath(&body, position, -1_000.0);
    assert!((plane.normal_fixed - position.normalize()).length() < 1e-9);
    assert!((plane.clearance_m(position) - 6_000.0).abs() < 1e-6);
}

fn decelerating(g_load: f64) -> InertialSiteSample {
    InertialSiteSample {
        site_to_inertial: DQuat::IDENTITY,
        specific_force_site_m_s2: DVec3::new(-g_load * G, 0.0, 0.0),
        angular_rate_site_rad_s: DVec3::ZERO,
    }
}

fn monitor() -> EntryMonitorBlock {
    EntryMonitorBlock::new(EntryMonitorConfig {
        period: PERIOD,
        sensing_axis_site: DVec3::X,
        initial_delta_v_m_s: 8_000.0,
    })
    .unwrap()
}

fn run_ten_seconds_after_threshold(g_load: f64) -> EntryMonitorOutput {
    let mut ems = monitor();
    ems.step(&decelerating(0.06)).unwrap();
    for _ in 0..99 {
        ems.step(&decelerating(g_load)).unwrap();
        assert_eq!(ems.output().corridor, CorridorIndication::Pending);
    }
    ems.step(&decelerating(g_load)).unwrap();
    ems.output()
}

#[test]
fn the_threshold_latches_exactly_at_a_twentieth_of_a_g() {
    let mut ems = monitor();
    ems.step(&decelerating(0.049)).unwrap();
    assert!(!ems.output().threshold_reached);
    ems.step(&decelerating(0.05)).unwrap();
    assert!(ems.output().threshold_reached);
    ems.step(&decelerating(0.0)).unwrap();
    assert!(ems.output().threshold_reached, "latched");
}

#[test]
fn the_corridor_check_reads_the_deceleration_ten_seconds_on() {
    assert_eq!(
        run_ten_seconds_after_threshold(0.3).corridor,
        CorridorIndication::LiftUp
    );
    assert_eq!(
        run_ten_seconds_after_threshold(0.2).corridor,
        CorridorIndication::LiftDown
    );
}

#[test]
fn the_corridor_verdict_holds_once_given() {
    let mut ems = monitor();
    ems.step(&decelerating(0.06)).unwrap();
    for _ in 0..100 {
        ems.step(&decelerating(0.3)).unwrap();
    }
    ems.step(&decelerating(0.1)).unwrap();
    assert_eq!(ems.output().corridor, CorridorIndication::LiftUp);
}

#[test]
fn the_counter_winds_down_by_sensed_deceleration_and_the_peak_holds() {
    let mut ems = monitor();
    for g_load in [1.0, 3.0, 2.0] {
        ems.step(&decelerating(g_load)).unwrap();
    }
    let output = ems.output();
    assert!((output.delta_v_counter_m_s - (8_000.0 - 0.6 * G)).abs() < 1e-9);
    assert_eq!(output.peak_g_load, 3.0);
    assert_eq!(output.g_load, 2.0);
}

#[test]
fn a_snapshot_keeps_the_time_since_the_threshold() {
    let mut ems = monitor();
    ems.step(&decelerating(0.06)).unwrap();
    let saved = ems.snapshot();
    let finish = |ems: &mut EntryMonitorBlock| {
        for _ in 0..100 {
            ems.step(&decelerating(0.3)).unwrap();
        }
        ems.output()
    };
    let expected = finish(&mut ems);
    ems.restore(&saved);
    assert_eq!(finish(&mut ems), expected);
}

#[test]
fn a_crooked_sensing_axis_is_refused() {
    assert!(
        EntryMonitorBlock::new(EntryMonitorConfig {
            period: PERIOD,
            sensing_axis_site: DVec3::splat(1.0),
            initial_delta_v_m_s: 0.0,
        })
        .is_err()
    );
}

/// An LM with `+x` up the frame's `+z` and `+z` along the frame's `+x`,
/// pitched back from it by `pitch_rad`.
fn lm_pitched_back(pitch_rad: f64) -> DQuat {
    let upright = DQuat::from_mat3(&glam::DMat3::from_cols(DVec3::Z, DVec3::NEG_Y, DVec3::X));
    (upright * DQuat::from_rotation_y(pitch_rad)).normalize()
}

#[test]
fn a_target_ahead_reads_its_depression_on_the_scale() {
    let eye = DVec3::new(0.0, 0.0, 100.0);
    let reading = LpdProjector.project(eye, lm_pitched_back(0.0), DVec3::new(100.0, 0.0, 0.0));
    assert!((reading.elevation_rad - 45f64.to_radians()).abs() < 1e-12);
    assert!(reading.cross_track_rad.abs() < 1e-12);
    assert!(reading.on_scale);
}

#[test]
fn pitching_back_moves_the_target_down_the_scale() {
    let eye = DVec3::new(0.0, 0.0, 100.0);
    let target = DVec3::new(100.0, 0.0, 0.0);
    let reading = LpdProjector.project(eye, lm_pitched_back(0.3), target);
    assert!((reading.elevation_rad - (45f64.to_radians() + 0.3)).abs() < 1e-12);
}

#[test]
fn a_target_off_the_scale_or_off_the_plane_says_so() {
    let eye = DVec3::new(0.0, 0.0, 100.0);
    let straight_down = DVec3::new(1.0, 0.0, 0.0);
    let below = LpdProjector.project(eye, lm_pitched_back(0.0), straight_down);
    assert!(!below.on_scale);
    assert!(below.elevation_rad > 70f64.to_radians());
    let above_horizon =
        LpdProjector.project(eye, lm_pitched_back(0.0), DVec3::new(100.0, 0.0, 120.0));
    assert!(!above_horizon.on_scale);
    let aside = LpdProjector.project(eye, lm_pitched_back(0.0), DVec3::new(100.0, -100.0, 0.0));
    assert!(aside.cross_track_rad.abs() > 0.5);
}

#[test]
fn a_redesignation_returns_the_crater_the_crew_read_off() {
    let eye = DVec3::new(10.0, -20.0, 500.0);
    let attitude = lm_pitched_back(0.4);
    let crater = DVec3::new(600.0, 40.0, 0.0);
    let reading = LpdProjector.project(eye, attitude, crater);
    let point = LpdProjector
        .redesignate(eye, attitude, &reading, &SURFACE)
        .unwrap();
    assert!((point - crater).length() < 1e-6, "{point}");
}

#[test]
fn a_line_of_sight_above_the_horizon_reaches_no_surface() {
    let eye = DVec3::new(0.0, 0.0, 100.0);
    let up = LpdReading {
        elevation_rad: -FRAC_PI_2 / 2.0,
        cross_track_rad: 0.0,
        on_scale: false,
    };
    assert_eq!(
        LpdProjector.redesignate(eye, lm_pitched_back(0.0), &up, &SURFACE),
        None
    );
}
