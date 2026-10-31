//! An Apollo LM's real sensors through the public API, from the blocks to
//! pixels, across a gimbal capture.

use glam::{BVec3, DQuat, DVec3};
use indicate_instrument_panels::PfdConfig;
use indicate_instrument_raster::{FrameId, FramebufferDims};
use indicate_instrument_state::abi::v8::CAPACITY;
use sim_apollo::{
    CockpitInstrumentPanel, InertialSiteSample, LunarContactOutput, LunarModuleSources,
    RateGyroOutput, StrapdownAsaBlock, StrapdownAsaConfig, ThreeGimbalImuBlock,
    ThreeGimbalImuConfig,
};
use sim_core::{Block, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, UtcTime};
use sim_indicate::{
    AGS_SOURCE, AltitudeSource, DisplayAxes, FeedConfig, FlightSample, MonitoredFrame,
    PANEL_HEIGHT_PX, PANEL_RGBA_BYTES, PANEL_SCENE_BYTES, PANEL_WIDTH_PX, PGNS_SOURCE,
    PanelBuffers, SourceMonitor, StateFeed, decode_frame, encode_frame, render_hsi, render_pfd,
};

const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 50);
const YAW_RATE_RAD_S: f64 = 0.5;

#[cfg(test)]
struct Frames {
    pgns: Vec<MonitoredFrame>,
    ags: Vec<MonitoredFrame>,
}

/// Slews an LM into gimbal lock and on for a second, splitting the frames by
/// the source Indicate chose.
#[cfg(test)]
fn fly() -> Frames {
    let mut imu =
        ThreeGimbalImuBlock::new(ThreeGimbalImuConfig::nominal(PERIOD, DQuat::IDENTITY)).unwrap();
    let mut asa =
        StrapdownAsaBlock::new(StrapdownAsaConfig::nominal(PERIOD, DQuat::IDENTITY)).unwrap();
    let body = BodyEllipsoid::MOON_IAU;
    let site = Geodetic::from_degrees(0.7, 23.5, 3_000.0);
    let mut cockpit = CockpitInstrumentPanel::new(SimInstant::START);
    let mut feed = StateFeed::new(FeedConfig {
        body,
        axes: DisplayAxes::apollo_lm(),
        origin: site,
        origin_id: 11,
        altitude: AltitudeSource::OriginRelative,
    });
    let mut monitor = SourceMonitor::new();
    let moon = InertialTransform::moon_at(UtcTime::from_unix_seconds(-14_182_940.0));
    let vehicle = VehicleState {
        position_ecef: body.geodetic_to_fixed(&site),
        velocity_ecef: DVec3::new(10.0, 40.0, 0.0),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    };
    let (mut pgns, mut ags) = (Vec::new(), Vec::new());
    let mut at = SimInstant::START;
    for step in 1..=170 {
        at = at.advanced(PERIOD);
        let yaw_rad = YAW_RATE_RAD_S * PERIOD.as_seconds() * f64::from(step);
        let sensed = InertialSiteSample {
            site_to_inertial: DQuat::from_rotation_z(yaw_rad),
            specific_force_site_m_s2: DVec3::ZERO,
            angular_rate_site_rad_s: DVec3::Z * YAW_RATE_RAD_S,
        };
        imu.step(&sensed).unwrap();
        asa.step(&sensed).unwrap();
        let snapshot = cockpit.observe_lunar_module(&LunarModuleSources {
            timestamp: at,
            imu: imu.output(),
            asa: asa.output(),
            rate_gyros: RateGyroOutput {
                rate_rad_s: DVec3::Z * YAW_RATE_RAD_S,
                saturated: BVec3::FALSE,
            },
            radar: None,
            contact: LunarContactOutput::default(),
            lpd: None,
            height_m: 3_000.0,
            speed_m_s: 41.0,
        });
        let sample = FlightSample {
            cockpit: Some(snapshot),
            central_body: Some(moon),
            ..FlightSample::truth(at, vehicle)
        };
        let frame = monitor.step(&mut feed, &sample).unwrap();
        match frame.panel.sources.attitude.selected.map(|s| s.source()) {
            Some(PGNS_SOURCE) => pgns.push(frame),
            Some(AGS_SOURCE) => ags.push(frame),
            other => panic!("no source selected: {other:?}"),
        }
    }
    assert!(imu.is_gimbal_locked());
    Frames { pgns, ags }
}

#[cfg(test)]
fn pfd_pixels(frame: &MonitoredFrame) -> Vec<u8> {
    let mut scene = vec![0; PANEL_SCENE_BYTES];
    let mut pixels = vec![0; PANEL_RGBA_BYTES];
    render_pfd(
        &frame.panel,
        &PfdConfig::default(),
        Some(&frame.alerts),
        FrameId {
            frame_generation: frame.state.snapshot.generation,
            render_generation: 1,
        },
        &mut PanelBuffers {
            scene: &mut scene,
            pixels: &mut pixels,
            dims: FramebufferDims::tight(PANEL_WIDTH_PX, PANEL_HEIGHT_PX),
        },
    )
    .unwrap();
    pixels
}

#[test]
fn the_display_changes_source_once_and_only_at_the_capture() {
    let frames = fly();
    assert!(!frames.pgns.is_empty() && !frames.ags.is_empty());
    let first_on_ags = &frames.ags[0];
    assert!(first_on_ags.panel.sources.attitude.reverted);
    assert!(
        frames
            .pgns
            .iter()
            .all(|f| !f.panel.sources.attitude.reverted)
    );
    assert!(frames.ags.iter().all(|f| f.panel.sources.attitude.reverted));
}

#[test]
fn the_reversion_is_visible_on_the_glass() {
    let frames = fly();
    let (last_pgns, first_ags) = (frames.pgns.last().unwrap(), &frames.ags[0]);
    let mut annunciated = *first_ags;
    annunciated.panel.sources = last_pgns.panel.sources;
    assert_ne!(pfd_pixels(first_ags), pfd_pixels(&annunciated));
}

#[test]
fn identical_flights_paint_identical_panels() {
    let (first, second) = (fly(), fly());
    let frame = |frames: &Frames| *frames.ags.last().unwrap();
    assert_eq!(pfd_pixels(&frame(&first)), pfd_pixels(&frame(&second)));
}

#[test]
fn the_hsi_paints_from_a_frame_that_crossed_the_binary_abi() {
    let frames = fly();
    let frame = frames.ags.last().unwrap();
    let mut wire = [0u8; CAPACITY];
    let length = encode_frame(&frame.state, &mut wire).unwrap();
    let decoded = decode_frame(&wire[..length]).unwrap().state;
    assert_eq!(decoded, frame.state);
    let mut scene = vec![0; PANEL_SCENE_BYTES];
    let mut pixels = vec![0; PANEL_RGBA_BYTES];
    let painted = render_hsi(
        &frame.panel,
        None,
        FrameId::default(),
        &mut PanelBuffers {
            scene: &mut scene,
            pixels: &mut pixels,
            dims: FramebufferDims::tight(PANEL_WIDTH_PX, PANEL_HEIGHT_PX),
        },
    )
    .unwrap();
    assert!(painted.scene_bytes > 1);
    assert!(
        pixels
            .as_chunks::<4>()
            .0
            .iter()
            .all(|pixel| pixel[3] == 255)
    );
}
