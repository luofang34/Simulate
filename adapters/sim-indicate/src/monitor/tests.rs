use glam::{BVec3, DVec3};
use indicate_instrument_state::{ComparisonState, SignalStatus, validate_quat};
use sim_apollo::{
    CockpitInstrumentPanel, GuidanceSource, InertialSiteSample, LunarContactOutput,
    LunarModuleSources, RateGyroOutput, StrapdownAsaBlock, StrapdownAsaConfig, ThreeGimbalImuBlock,
    ThreeGimbalImuConfig,
};
use sim_core::{Block, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, UtcTime};

use super::*;
use crate::{AltitudeSource, DisplayAxes, FeedConfig};

const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 50);
const STEPS_PER_SECOND: u32 = 50;
const YAW_RATE_RAD_S: f64 = 0.5;

fn site_of_landing() -> Geodetic {
    Geodetic::from_degrees(0.7, 23.5, 3_000.0)
}

/// An LM over the Moon, slewing about the platform's `z` axis: the way that
/// drives the middle gimbal into lock.
struct Lander {
    imu: ThreeGimbalImuBlock,
    asa: StrapdownAsaBlock,
    panel: CockpitInstrumentPanel,
    feed: StateFeed,
    monitor: SourceMonitor,
    yaw_rad: f64,
    rate_rad_s: f64,
    at: SimInstant,
}

impl Lander {
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
            panel: CockpitInstrumentPanel::new(SimInstant::START),
            feed: StateFeed::new(FeedConfig {
                body: BodyEllipsoid::MOON_IAU,
                axes: DisplayAxes::apollo_lm(),
                origin: site_of_landing(),
                origin_id: 11,
                altitude: AltitudeSource::OriginRelative,
                heading_mode: crate::HeadingMode::default(),
            }),
            monitor: SourceMonitor::new(),
            yaw_rad: 0.0,
            rate_rad_s: 0.0,
            at: SimInstant::START,
        }
    }

    fn sample(&mut self) -> FlightSample {
        let cockpit = self.panel.observe_lunar_module(&LunarModuleSources {
            timestamp: self.at,
            imu: self.imu.output(),
            asa: self.asa.output(),
            rate_gyros: RateGyroOutput {
                rate_rad_s: DVec3::Z * self.rate_rad_s,
                saturated: BVec3::FALSE,
            },
            radar: None,
            contact: LunarContactOutput::default(),
            lpd: None,
            height_m: 3_000.0,
            speed_m_s: 50.0,
        });
        let body = BodyEllipsoid::MOON_IAU;
        let position = body.geodetic_to_fixed(&site_of_landing());
        FlightSample {
            timestamp: self.at,
            vehicle: VehicleState {
                position_ecef: position,
                velocity_ecef: DVec3::ZERO,
                attitude: DQuat::IDENTITY,
                angular_rate_body: DVec3::ZERO,
            },
            cockpit: Some(cockpit),
            central_body: Some(InertialTransform::moon_at(UtcTime::from_unix_seconds(
                -14_182_940.0,
            ))),
            air_data: None,
            specific_force_body_m_s2: None,
            guidance: None,
        }
    }

    fn step(&mut self, rate_rad_s: f64) -> MonitoredFrame {
        self.rate_rad_s = rate_rad_s;
        self.yaw_rad += rate_rad_s * PERIOD.as_seconds();
        self.at = self.at.advanced(PERIOD);
        let site = InertialSiteSample {
            site_to_inertial: DQuat::from_rotation_z(self.yaw_rad),
            specific_force_site_m_s2: DVec3::ZERO,
            angular_rate_site_rad_s: DVec3::Z * rate_rad_s,
        };
        self.imu.step(&site).unwrap();
        self.asa.step(&site).unwrap();
        let sample = self.sample();
        self.monitor.step(&mut self.feed, &sample).unwrap()
    }

    /// Slews until the platform captures, returning the frames either side.
    fn slew_into_lock(&mut self) -> (MonitoredFrame, MonitoredFrame) {
        let mut before = self.step(YAW_RATE_RAD_S);
        loop {
            let frame = self.step(YAW_RATE_RAD_S);
            if self.imu.is_gimbal_locked() {
                return (before, frame);
            }
            before = frame;
            assert!(self.yaw_rad < 2.0, "the slew never reached capture");
        }
    }

    fn hold(&mut self, steps: u32) -> MonitoredFrame {
        let mut frame = self.step(0.0);
        for _ in 1..steps {
            frame = self.step(0.0);
        }
        frame
    }

    /// Makes the computer believe the platform sits `angle_rad` from where it
    /// does, with every health signal still clean.
    fn bias_pgns(&mut self, angle_rad: f64) {
        let output = self.imu.output();
        self.imu
            .fine_align(output.estimated_platform_to_inertial * DQuat::from_rotation_x(angle_rad));
    }
}

fn selected(frame: &MonitoredFrame) -> SourceId {
    frame.panel.sources.attitude.selected.unwrap().source()
}

#[test]
fn healthy_sources_agree_and_the_primary_is_shown() {
    let frame = Lander::new().step(YAW_RATE_RAD_S);
    assert_eq!(selected(&frame), PGNS_SOURCE);
    assert_eq!(frame.report.attitude.state, ComparisonState::Agree);
    assert!(!frame.panel.sources.attitude.reverted);
    assert_eq!(frame.panel.roll_rad.status, SignalStatus::Valid);
    assert_eq!(frame.alerts.active().len(), 0);
}

#[test]
fn indicate_rejects_a_captured_platform_on_the_frame_it_fails_and_shows_ags() {
    let mut lander = Lander::new();
    let (before, after) = lander.slew_into_lock();
    assert_eq!(selected(&before), PGNS_SOURCE);
    assert_eq!(before.report.attitude.state, ComparisonState::Agree);
    assert!(before.report.attitude.selected == Some(PGNS_SOURCE));

    assert!(lander.imu.output().mga_rad.to_degrees() >= sim_apollo::GIMBAL_LOCK_CAPTURE_DEG);
    assert!(!lander.imu.output().platform_valid);
    assert_eq!(selected(&after), AGS_SOURCE);
    assert!(after.panel.sources.attitude.reverted);
    assert_eq!(
        after.report.attitude.state,
        ComparisonState::InsufficientSources
    );
    assert!(after.panel.roll_rad.status.shows_value());
}

#[test]
fn the_value_shown_after_reversion_is_the_backups_own() {
    let mut lander = Lander::new();
    lander.slew_into_lock();
    let frame = lander.hold(50);
    let shown = frame.panel.sources.attitude.selected.unwrap().value();
    let sample = lander.sample();
    let local = lander.feed.local_level(&sample);
    let axes = lander.feed.config().axes;
    let cockpit = sample.cockpit.unwrap();
    let ags = sample
        .display_attitude(cockpit.ags_attitude.value, &local, &axes)
        .unwrap();
    assert_eq!(shown, indicate_quat(ags));
    let pgns = sample
        .display_attitude(cockpit.pgns_attitude.value, &local, &axes)
        .unwrap();
    assert!(
        ags.angle_between(pgns) > 3f64.to_radians(),
        "the captured platform has tumbled away by now"
    );
}

#[test]
fn the_rose_follows_the_source_indicate_selected() {
    let mut lander = Lander::new();
    lander.slew_into_lock();
    let frame = lander.hold(50);
    let heading = frame.panel.sources.heading.selected.unwrap();
    assert_eq!(heading.source(), AGS_SOURCE);
    assert!(frame.panel.sources.heading.reverted);
    assert_eq!(
        frame.panel.heading.value_rad.value,
        indicate_instrument_state::wrap_2pi(heading.value()),
    );
}

#[test]
fn a_sustained_silent_disagreement_alerts_but_never_picks_a_winner() {
    let mut lander = Lander::new();
    lander.hold(5);
    lander.bias_pgns(10f64.to_radians());
    let early = lander.hold(STEPS_PER_SECOND / 2);
    assert_ne!(early.report.attitude.state, ComparisonState::Miscompare);
    assert!(early.alerts.active().is_empty(), "a transient never alerts");

    let late = lander.hold(STEPS_PER_SECOND);
    assert_eq!(late.report.attitude.state, ComparisonState::Miscompare);
    assert_eq!(
        selected(&late),
        PGNS_SOURCE,
        "ambiguous, so the primary stays"
    );
    assert!(!late.panel.sources.attitude.reverted);
    // A bias about the LM's thrust axis is a yaw error, so the heading the two
    // sources imply disagrees as well, and the crew is told about both.
    assert_eq!(late.report.heading.state, ComparisonState::Miscompare);
    assert_eq!(late.alerts.active().len(), 2);
}

#[test]
fn the_miscompare_clears_when_the_sources_agree_again() {
    let mut lander = Lander::new();
    lander.hold(5);
    lander.bias_pgns(10f64.to_radians());
    lander.hold(STEPS_PER_SECOND * 2);
    lander.bias_pgns(-10f64.to_radians());
    let frame = lander.hold(5);
    assert_eq!(frame.report.attitude.state, ComparisonState::Agree);
    assert_eq!(frame.report.attitude.fault, None);
}

#[test]
fn a_realigned_platform_earns_its_way_back() {
    let mut lander = Lander::new();
    lander.slew_into_lock();
    lander.hold(5);
    lander
        .imu
        .coarse_align(lander.asa.output().site_to_inertial);
    lander.panel.crew_select(GuidanceSource::Pgns);
    let soon = lander.hold(STEPS_PER_SECOND);
    assert_eq!(selected(&soon), AGS_SOURCE, "no chatter straight back");
    let later = lander.hold(STEPS_PER_SECOND * 3);
    assert_eq!(selected(&later), PGNS_SOURCE);
    assert!(!later.panel.sources.attitude.reverted);
}

#[test]
fn a_source_that_stopped_updating_goes_stale_rather_than_agreeing() {
    let mut lander = Lander::new();
    lander.hold(5);
    let mut sample = lander.sample();
    let frozen = sample.cockpit.unwrap().ags_attitude;
    sample.timestamp = sample.timestamp.advanced(TickSpan::SECOND);
    let later = sample.timestamp;
    let mut cockpit = sample.cockpit.unwrap();
    cockpit.pgns_attitude.timestamp = later;
    cockpit.attitude.timestamp = later;
    cockpit.ags_attitude = frozen;
    sample.cockpit = Some(cockpit);
    let frame = lander.monitor.step(&mut lander.feed, &sample).unwrap();
    assert_eq!(selected(&frame), PGNS_SOURCE);
    assert_eq!(
        frame.report.attitude.state,
        ComparisonState::InsufficientSources
    );
}

#[test]
fn no_usable_source_fails_the_ball_instead_of_holding_the_last_picture() {
    let mut lander = Lander::new();
    lander.hold(5);
    let mut sample = lander.sample();
    let mut cockpit = sample.cockpit.unwrap();
    cockpit.pgns_attitude.validity = SourceValidity::Failed;
    cockpit.ags_attitude.validity = SourceValidity::Failed;
    sample.cockpit = Some(cockpit);
    sample.timestamp = sample.timestamp.advanced(PERIOD);
    let frame = lander.monitor.step(&mut lander.feed, &sample).unwrap();
    assert_eq!(frame.panel.sources.attitude.selected, None);
    assert_eq!(frame.panel.roll_rad.status, SignalStatus::Failed);
}

#[test]
fn a_vehicle_with_one_source_resolves_without_a_comparison() {
    let mut lander = Lander::new();
    let vehicle =
        VehicleState::level_flight(&Geodetic::from_degrees(39.1, -75.6, 900.0), 0.0, 50.0);
    let mut feed = StateFeed::new(FeedConfig {
        body: BodyEllipsoid::WGS84,
        axes: DisplayAxes::AEROSPACE,
        origin: Geodetic::from_degrees(39.1, -75.6, 0.0),
        origin_id: 1,
        altitude: AltitudeSource::OriginRelative,
        heading_mode: crate::HeadingMode::default(),
    });
    let snapshot = lander
        .panel
        .observe_earth(&sim_apollo::EarthVehicleSources {
            timestamp: SimInstant::START,
            vehicle,
            height_m: 900.0,
        });
    let sample = FlightSample {
        cockpit: Some(snapshot),
        ..FlightSample::truth(SimInstant::START, vehicle)
    };
    let frame = SourceMonitor::new().step(&mut feed, &sample).unwrap();
    assert_eq!(frame.panel.sources.attitude.selected, None);
    assert_eq!(frame.panel.roll_rad.status, SignalStatus::Valid);
    let no_cockpit = FlightSample::truth(SimInstant::START, vehicle);
    let frame = SourceMonitor::new().step(&mut feed, &no_cockpit).unwrap();
    assert_eq!(frame.panel.pitch_rad.status, SignalStatus::Valid);
}

#[test]
fn every_candidate_attitude_is_a_well_formed_rotation() {
    let mut lander = Lander::new();
    lander.slew_into_lock();
    let sample = lander.sample();
    let local = lander.feed.local_level(&sample);
    let axes = lander.feed.config().axes;
    for rotation in [
        sample.cockpit.unwrap().pgns_attitude.value,
        sample.cockpit.unwrap().ags_attitude.value,
    ] {
        let display = sample.display_attitude(rotation, &local, &axes).unwrap();
        assert!(validate_quat(indicate_quat(display)).is_ok());
    }
}

#[test]
fn two_identical_runs_make_identical_decisions() {
    let run = || {
        let mut lander = Lander::new();
        let (_, frame) = lander.slew_into_lock();
        (frame.panel, frame.report.attitude, lander.at)
    };
    assert_eq!(run(), run());
}
