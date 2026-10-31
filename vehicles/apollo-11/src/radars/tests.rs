use std::f64::consts::FRAC_PI_2;

use glam::{DQuat, DVec3};
use mbd_core::{KinematicsFrame, LocalSiteKinematics};
use sim_core::{Block, TICKS_PER_SECOND, TickSpan};

use super::*;
use crate::SurfacePlane;

const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 20);

/// LM `+x` straight up the frame's `+z`, `height_m` above a plane at the
/// origin.
fn upright(height_m: f64, velocity: DVec3) -> LocalSiteKinematics {
    LocalSiteKinematics {
        frame: KinematicsFrame::Ecef,
        position_m: DVec3::new(0.0, 0.0, height_m),
        velocity_m_s: velocity,
        attitude: DQuat::from_rotation_y(-FRAC_PI_2),
        specific_force_site_m_s2: DVec3::ZERO,
        angular_rate_site_rad_s: DVec3::ZERO,
    }
}

const SURFACE: SurfacePlane = SurfacePlane {
    point_fixed_m: DVec3::ZERO,
    normal_fixed: DVec3::Z,
};

fn landing_radar() -> LandingRadarBlock {
    LandingRadarBlock::new(LandingRadarConfig::nominal(PERIOD)).unwrap()
}

fn read(site: LocalSiteKinematics, antenna: LandingRadarAntenna) -> LandingRadarOutput {
    let mut radar = landing_radar();
    radar
        .step(&LandingRadarInput {
            site,
            surface: SURFACE,
            antenna,
        })
        .unwrap();
    radar.output()
}

#[test]
fn slant_range_is_the_height_over_the_boresight_cosine() {
    let hover = read(upright(1_000.0, DVec3::ZERO), LandingRadarAntenna::Hover);
    assert_eq!(hover.validity, RadarValidity::Valid);
    assert!((hover.slant_range_m - 1_000.0 / 6f64.to_radians().cos()).abs() < 1e-6);
    let descent = read(upright(1_000.0, DVec3::ZERO), LandingRadarAntenna::Descent);
    assert!((descent.slant_range_m - 1_000.0 / 24f64.to_radians().cos()).abs() < 1e-6);
}

#[test]
fn the_three_doppler_beams_solve_for_the_velocity_in_either_position() {
    let velocity = DVec3::new(40.0, -15.0, -80.0);
    for antenna in [LandingRadarAntenna::Descent, LandingRadarAntenna::Hover] {
        let site = upright(2_000.0, velocity);
        let reading = read(site, antenna);
        let truth = site.attitude.inverse() * velocity;
        assert!((reading.velocity_site_m_s - truth).length() < 1e-9);
        let [first, second, third] = velocity_beams_of(antenna).map(|beam| beam.dot(truth));
        assert_eq!(reading.beam_doppler_m_s, [first, second, third]);
    }
}

fn velocity_beams_of(antenna: LandingRadarAntenna) -> [DVec3; 3] {
    // The beams through the block itself: unit doppler per unit axis speed.
    let probe = |axis: DVec3| {
        let site = upright(2_000.0, DQuat::from_rotation_y(-FRAC_PI_2) * axis);
        read(site, antenna).beam_doppler_m_s
    };
    let [xa, xb, xc] = probe(DVec3::X);
    let [ya, yb, yc] = probe(DVec3::Y);
    let [za, zb, zc] = probe(DVec3::Z);
    [
        DVec3::new(xa, ya, za),
        DVec3::new(xb, yb, zb),
        DVec3::new(xc, yc, zc),
    ]
}

#[test]
fn the_beams_descend_towards_the_surface() {
    let descending = read(
        upright(2_000.0, DVec3::new(0.0, 0.0, -50.0)),
        LandingRadarAntenna::Hover,
    );
    assert!(descending.beam_doppler_m_s.iter().all(|speed| *speed > 0.0));
}

#[test]
fn above_the_limit_the_radar_loses_lock() {
    let high = read(upright(11_950.0, DVec3::ZERO), LandingRadarAntenna::Hover);
    assert_eq!(high.validity, RadarValidity::OutOfLimits);
    let in_range = read(upright(11_900.0, DVec3::ZERO), LandingRadarAntenna::Hover);
    assert_eq!(in_range.validity, RadarValidity::Valid);
}

#[test]
fn excessive_tilt_loses_lock() {
    let tilted = |degrees: f64| LocalSiteKinematics {
        attitude: DQuat::from_rotation_x(degrees.to_radians()) * upright(0.0, DVec3::ZERO).attitude,
        ..upright(1_000.0, DVec3::ZERO)
    };
    let mild = read(tilted(20.0), LandingRadarAntenna::Hover);
    assert_eq!(mild.validity, RadarValidity::Valid);
    let steep = read(tilted(70.0), LandingRadarAntenna::Hover);
    assert_eq!(steep.validity, RadarValidity::OutOfLimits);
    let inverted = read(tilted(170.0), LandingRadarAntenna::Hover);
    assert_eq!(inverted.validity, RadarValidity::OutOfLimits);
}

#[test]
fn below_the_surface_is_out_of_limits_and_a_body_frame_site_is_refused() {
    let buried = read(upright(-1.0, DVec3::ZERO), LandingRadarAntenna::Hover);
    assert_eq!(buried.validity, RadarValidity::OutOfLimits);

    let mut radar = landing_radar();
    let wrong_frame = LocalSiteKinematics {
        frame: KinematicsFrame::Body,
        ..upright(1_000.0, DVec3::ZERO)
    };
    assert!(
        radar
            .step(&LandingRadarInput {
                site: wrong_frame,
                surface: SURFACE,
                antenna: LandingRadarAntenna::Hover,
            })
            .is_err()
    );
}

fn track(target: DVec3, target_velocity: DVec3) -> RelativeTrackInput {
    RelativeTrackInput {
        site: LocalSiteKinematics {
            attitude: DQuat::IDENTITY,
            ..upright(0.0, DVec3::ZERO)
        },
        target_position_fixed_m: target,
        target_velocity_fixed_m_s: target_velocity,
    }
}

#[test]
fn the_rendezvous_radar_reads_range_rate_and_antenna_angles() {
    let mut radar = RendezvousRadarBlock::new(RendezvousRadarConfig::nominal(PERIOD)).unwrap();
    let target = DVec3::new(3_000.0, 4_000.0, 12_000.0);
    radar
        .step(&track(target, target.normalize() * -5.0))
        .unwrap();
    let output = radar.output();
    assert_eq!(output.validity, RadarValidity::Valid);
    assert!((output.range_m - 13_000.0).abs() < 1e-9);
    assert!((output.range_rate_m_s + 5.0).abs() < 1e-9);
    assert!((output.shaft_rad - 3_000f64.atan2(12_000.0)).abs() < 1e-12);
    assert!((output.trunnion_rad - (4_000.0 / 13_000f64).asin()).abs() < 1e-12);
}

#[test]
fn the_rendezvous_radar_loses_a_target_out_of_range_or_off_its_gimbals() {
    let mut radar = RendezvousRadarBlock::new(RendezvousRadarConfig::nominal(PERIOD)).unwrap();
    for target in [
        DVec3::new(0.0, 0.0, 800_000.0),
        DVec3::new(10_000.0, 0.0, 1_000.0),
        DVec3::new(0.0, 10_000.0, 1_000.0),
    ] {
        radar.step(&track(target, DVec3::ZERO)).unwrap();
        assert_eq!(
            radar.output().validity,
            RadarValidity::OutOfLimits,
            "{target}"
        );
    }
}

#[test]
fn vhf_ranging_is_independent_quantised_and_omnidirectional() {
    let mut vhf = VhfRangingBlock::new(VhfRangingConfig::nominal(PERIOD, 30.0)).unwrap();
    vhf.step(&track(DVec3::new(0.0, 0.0, -100_000.0), DVec3::ZERO))
        .unwrap();
    let output = vhf.output();
    assert_eq!(output.validity, RadarValidity::Valid);
    assert!((output.range_m - 100_030.0).abs() <= VHF_RESOLUTION_M_FOR_TEST / 2.0);
    let steps = output.range_m / VHF_RESOLUTION_M_FOR_TEST;
    assert!((steps - steps.round()).abs() < 1e-6);

    vhf.step(&track(DVec3::new(0.0, 0.0, -700_000.0), DVec3::ZERO))
        .unwrap();
    assert_eq!(vhf.output().validity, RadarValidity::OutOfLimits);
}

const VHF_RESOLUTION_M_FOR_TEST: f64 = 18.52;

#[test]
fn a_target_on_top_of_the_antenna_is_refused_and_snapshots_restore() {
    let mut radar = RendezvousRadarBlock::new(RendezvousRadarConfig::nominal(PERIOD)).unwrap();
    assert!(radar.step(&track(DVec3::ZERO, DVec3::ZERO)).is_err());
    radar
        .step(&track(DVec3::new(0.0, 0.0, 5_000.0), DVec3::ZERO))
        .unwrap();
    let saved = radar.snapshot();
    radar
        .step(&track(DVec3::new(0.0, 0.0, 6_000.0), DVec3::ZERO))
        .unwrap();
    radar.restore(&saved);
    assert_eq!(radar.output(), saved);
    assert!(
        VhfRangingBlock::new(VhfRangingConfig {
            resolution_m: 0.0,
            ..VhfRangingConfig::nominal(PERIOD, 0.0)
        })
        .is_err()
    );
}
