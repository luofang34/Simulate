use glam::{BVec3, DVec3};
use mbd_core::{KinematicsFrame, LocalSiteKinematics};
use sim_core::{Block, TICKS_PER_SECOND};

use super::*;
use crate::{
    InertialSiteSample, LandingRadarAntenna, LandingRadarBlock, LandingRadarConfig,
    LandingRadarInput, StrapdownAsaBlock, StrapdownAsaConfig, SurfacePlane, ThreeGimbalImuBlock,
    ThreeGimbalImuConfig,
};

const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 100);
const YAW_RATE_RAD_S: f64 = 0.5;

/// The vehicle turning about the platform's `z` axis, the way that drives the
/// middle gimbal into lock.
struct Slew {
    imu: ThreeGimbalImuBlock,
    asa: StrapdownAsaBlock,
    yaw_rad: f64,
    at: SimInstant,
}

impl Slew {
    fn new() -> Self {
        Self {
            imu: ThreeGimbalImuBlock::new(ThreeGimbalImuConfig {
                gyro_drift_rad_s: DVec3::ZERO,
                ..ThreeGimbalImuConfig::nominal(PERIOD, DQuat::IDENTITY)
            })
            .unwrap(),
            asa: StrapdownAsaBlock::new(StrapdownAsaConfig {
                gyro_bias_rad_s: DVec3::ZERO,
                ..StrapdownAsaConfig::nominal(PERIOD, DQuat::IDENTITY)
            })
            .unwrap(),
            yaw_rad: 0.0,
            at: SimInstant::START,
        }
    }

    fn step(&mut self) {
        self.yaw_rad += YAW_RATE_RAD_S * PERIOD.as_seconds();
        self.at = self.at.advanced(PERIOD);
        let sample = InertialSiteSample {
            site_to_inertial: DQuat::from_rotation_z(self.yaw_rad),
            specific_force_site_m_s2: DVec3::ZERO,
            angular_rate_site_rad_s: DVec3::Z * YAW_RATE_RAD_S,
        };
        self.imu.step(&sample).unwrap();
        self.asa.step(&sample).unwrap();
    }

    fn sources(&self) -> LunarModuleSources {
        LunarModuleSources {
            timestamp: self.at,
            imu: self.imu.output(),
            asa: self.asa.output(),
            rate_gyros: RateGyroOutput {
                rate_rad_s: DVec3::Z * YAW_RATE_RAD_S,
                saturated: BVec3::FALSE,
            },
            radar: None,
            contact: LunarContactOutput::default(),
            lpd: None,
            height_m: 1_000.0,
            speed_m_s: 50.0,
        }
    }

    /// Steps until `reached` holds of the panel, returning the panel's
    /// snapshots before and after.
    fn run_until(
        &mut self,
        panel: &mut CockpitInstrumentPanel,
        reached: impl Fn(&CockpitInstrumentSnapshot) -> bool,
    ) -> (CockpitInstrumentSnapshot, CockpitInstrumentSnapshot) {
        let mut previous = panel.snapshot();
        for _ in 0..1_000 {
            self.step();
            let current = panel.observe_lunar_module(&self.sources());
            if reached(&current) {
                return (previous, current);
            }
            previous = current;
        }
        panic!("the slew never reached the condition");
    }
}

#[test]
fn a_healthy_platform_flies_pgns_with_every_channel_valid() {
    let mut slew = Slew::new();
    let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
    slew.step();
    let snapshot = panel.observe_lunar_module(&slew.sources());

    assert_eq!(snapshot.guidance_source, Some(GuidanceSource::Pgns));
    assert_eq!(snapshot.attitude.validity, SourceValidity::Valid);
    assert_eq!(snapshot.pgns_attitude.validity, SourceValidity::Valid);
    assert_eq!(snapshot.ags_attitude.validity, SourceValidity::Valid);
    assert_eq!(snapshot.attitude.timestamp, slew.at);
    assert!(!snapshot.gimbal_lock_caution && !snapshot.gimbal_lock_captured);
}

#[test]
fn the_caution_comes_at_seventy_degrees_with_pgns_still_flying() {
    let mut slew = Slew::new();
    let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
    let (before, at) = slew.run_until(&mut panel, |s| s.gimbal_lock_caution);

    assert!(
        (slew.yaw_rad.to_degrees() - 70.0).abs() < 0.5,
        "{}",
        slew.yaw_rad
    );
    assert!(!at.gimbal_lock_captured);
    assert_eq!(at.guidance_source, Some(GuidanceSource::Pgns));
    assert_eq!(
        at.changes_since(&before),
        vec![InstrumentChange::GimbalLockCaution(true)]
    );
}

#[test]
fn capture_hands_over_to_ags_and_attitude_stays_on_the_panel() {
    let mut slew = Slew::new();
    let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
    let (before, at) = slew.run_until(&mut panel, |s| s.gimbal_lock_captured);

    assert!(
        (slew.yaw_rad.to_degrees() - 85.0).abs() < 0.5,
        "{}",
        slew.yaw_rad
    );
    assert_eq!(before.guidance_source, Some(GuidanceSource::Pgns));
    assert_eq!(at.guidance_source, Some(GuidanceSource::Ags));
    assert_eq!(at.pgns_attitude.validity, SourceValidity::Failed);
    assert_eq!(at.attitude.validity, SourceValidity::Degraded);
    assert!(at.attitude.validity.is_usable());
    // The AGS attitude is the truth, so the flown attitude did not jump.
    let truth = DQuat::from_rotation_z(slew.yaw_rad);
    assert!(at.attitude.value.angle_between(truth) < 1e-6);
    assert_eq!(
        at.changes_since(&before),
        vec![
            InstrumentChange::GuidanceSource(Some(GuidanceSource::Ags)),
            InstrumentChange::Attitude(SourceValidity::Degraded),
            InstrumentChange::PgnsAttitude(SourceValidity::Failed),
            InstrumentChange::GimbalLockCaptured(true),
        ]
    );
}

#[test]
fn the_transfer_latches_after_the_vehicle_turns_back() {
    let mut slew = Slew::new();
    let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
    slew.run_until(&mut panel, |s| s.gimbal_lock_captured);
    slew.imu.coarse_align(DQuat::from_rotation_z(slew.yaw_rad));
    slew.step();

    let snapshot = panel.observe_lunar_module(&slew.sources());
    assert_eq!(snapshot.pgns_attitude.validity, SourceValidity::Valid);
    assert_eq!(snapshot.guidance_source, Some(GuidanceSource::Ags));
    assert_eq!(snapshot.attitude.validity, SourceValidity::Degraded);

    panel.crew_select(GuidanceSource::Pgns);
    let snapshot = panel.observe_lunar_module(&slew.sources());
    assert_eq!(snapshot.guidance_source, Some(GuidanceSource::Pgns));
    assert_eq!(snapshot.attitude.validity, SourceValidity::Valid);
}

/// A real radar at `height_m` above a level plane, its antenna tilted by
/// `tilt_rad` off level.
fn radar_at(height_m: f64, tilt_rad: f64, antenna: LandingRadarAntenna) -> LandingRadarOutput {
    let mut radar = LandingRadarBlock::new(LandingRadarConfig::nominal(PERIOD)).unwrap();
    // LM +x points up the surface normal when level.
    radar
        .step(&LandingRadarInput {
            site: LocalSiteKinematics {
                frame: KinematicsFrame::Ecef,
                position_m: DVec3::X * height_m,
                velocity_m_s: DVec3::NEG_X,
                attitude: DQuat::from_rotation_y(tilt_rad),
                angular_rate_site_rad_s: DVec3::ZERO,
                specific_force_site_m_s2: DVec3::ZERO,
            },
            surface: SurfacePlane {
                point_fixed_m: DVec3::ZERO,
                normal_fixed: DVec3::X,
            },
            antenna,
        })
        .unwrap();
    radar.output()
}

#[test]
fn a_locked_radar_reads_valid_and_its_range_is_the_radars() {
    let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
    let mut slew = Slew::new();
    slew.step();
    let output = radar_at(1_000.0, 0.0, LandingRadarAntenna::Hover);
    assert_eq!(output.validity, RadarValidity::Valid);
    let snapshot = panel.observe_lunar_module(&LunarModuleSources {
        radar: Some(RadarSample {
            output,
            timestamp: slew.at,
        }),
        ..slew.sources()
    });

    assert_eq!(snapshot.radar_slant_range_m.validity, SourceValidity::Valid);
    assert_eq!(snapshot.radar_slant_range_m.value, output.slant_range_m);
    assert_eq!(snapshot.radar_velocity_m_s.value, output.velocity_site_m_s);
}

#[test]
fn a_radar_out_of_limits_reads_missing_and_holds_its_last_good_value() {
    let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
    let mut slew = Slew::new();
    slew.step();
    let good = radar_at(1_000.0, 0.0, LandingRadarAntenna::Hover);
    let sample = |output, timestamp| RadarSample { output, timestamp };
    panel.observe_lunar_module(&LunarModuleSources {
        radar: Some(sample(good, slew.at)),
        ..slew.sources()
    });

    let beyond = radar_at(50_000.0, 0.0, LandingRadarAntenna::Hover);
    assert_eq!(beyond.validity, RadarValidity::OutOfLimits);
    let snapshot = panel.observe_lunar_module(&LunarModuleSources {
        radar: Some(sample(beyond, slew.at)),
        ..slew.sources()
    });

    assert_eq!(
        snapshot.radar_slant_range_m.validity,
        SourceValidity::Missing
    );
    assert_eq!(
        snapshot.radar_velocity_m_s.validity,
        SourceValidity::Missing
    );
    assert_eq!(snapshot.radar_slant_range_m.value, good.slant_range_m);
    assert!(!snapshot.radar_slant_range_m.validity.is_usable());
}

#[test]
fn a_radar_that_stops_sampling_goes_stale() {
    let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
    let mut slew = Slew::new();
    slew.step();
    let radar = Some(RadarSample {
        output: radar_at(1_000.0, 0.0, LandingRadarAntenna::Hover),
        timestamp: slew.at,
    });
    for _ in 0..101 {
        slew.step();
    }
    let snapshot = panel.observe_lunar_module(&LunarModuleSources {
        radar,
        ..slew.sources()
    });
    assert_eq!(snapshot.radar_slant_range_m.validity, SourceValidity::Stale);
}

#[test]
fn the_aids_reach_the_panel() {
    let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
    let mut slew = Slew::new();
    slew.step();
    let reading = |elevation_rad, on_scale| LpdReading {
        elevation_rad,
        cross_track_rad: 0.0,
        on_scale,
    };
    let snapshot = panel.observe_lunar_module(&LunarModuleSources {
        contact: LunarContactOutput {
            lunar_contact: true,
        },
        lpd: Some(reading(40_f64.to_radians(), true)),
        ..slew.sources()
    });
    assert!(snapshot.contact_light);
    assert!((snapshot.lpd_elevation_deg.unwrap() - 40.0).abs() < 1e-9);

    let snapshot = panel.observe_lunar_module(&LunarModuleSources {
        lpd: Some(reading(80_f64.to_radians(), false)),
        ..slew.sources()
    });
    assert_eq!(snapshot.lpd_elevation_deg, None, "off the scale");
}

#[test]
fn saturated_rate_gyros_degrade_their_reading() {
    let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
    let mut slew = Slew::new();
    slew.step();
    let snapshot = panel.observe_lunar_module(&LunarModuleSources {
        rate_gyros: RateGyroOutput {
            rate_rad_s: DVec3::Z,
            saturated: BVec3::new(false, false, true),
        },
        ..slew.sources()
    });
    assert_eq!(snapshot.body_rate_rad_s.validity, SourceValidity::Degraded);
}

#[test]
fn an_earth_vehicle_has_attitude_and_nothing_from_apollo() {
    let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
    let vehicle = VehicleState {
        position_ecef: DVec3::X * 7e6,
        velocity_ecef: DVec3::Y * 60.0,
        attitude: DQuat::from_rotation_x(0.3),
        angular_rate_body: DVec3::Z * 0.1,
    };
    let snapshot = panel.observe_earth(&EarthVehicleSources {
        timestamp: SimInstant::START,
        vehicle,
        height_m: 900.0,
    });

    assert_eq!(snapshot.guidance_source, None);
    assert_eq!(snapshot.attitude.value, vehicle.attitude);
    assert_eq!(snapshot.attitude.validity, SourceValidity::Valid);
    assert_eq!(snapshot.pgns_attitude.validity, SourceValidity::Missing);
    assert_eq!(
        snapshot.radar_slant_range_m.validity,
        SourceValidity::Missing
    );
    assert_eq!(
        (snapshot.speed_m_s.value, snapshot.height_m.value),
        (60.0, 900.0)
    );
    assert_eq!(snapshot.ems_g_load, None);
    assert_eq!(snapshot.lpd_elevation_deg, None);
    assert!(!snapshot.contact_light);
}

#[test]
fn the_command_module_shows_the_entry_monitor_and_fails_with_its_platform() {
    let mut slew = Slew::new();
    let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
    let ems = EntryMonitorOutput {
        g_load: 4.2,
        peak_g_load: 4.2,
        delta_v_counter_m_s: 1_000.0,
        threshold_reached: true,
        corridor: crate::CorridorIndication::LiftUp,
    };
    let sources = |slew: &Slew| CommandModuleSources {
        timestamp: slew.at,
        imu: slew.imu.output(),
        ems,
        height_m: 60_000.0,
        speed_m_s: 7_000.0,
    };
    let healthy = panel.observe_command_module(&sources(&slew));
    assert_eq!(healthy.ems_g_load, Some(4.2));
    assert_eq!(healthy.attitude.validity, SourceValidity::Valid);

    while !slew.imu.is_gimbal_locked() {
        slew.step();
    }
    let captured = panel.observe_command_module(&sources(&slew));
    assert_eq!(captured.attitude.validity, SourceValidity::Failed);
    assert_eq!(captured.guidance_source, Some(GuidanceSource::Pgns));
}
