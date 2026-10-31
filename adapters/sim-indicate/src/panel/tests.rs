use glam::DQuat;
use indicate_instrument_panels::PfdConfig;
use indicate_instrument_raster::{RasterError, RenderWork};
use indicate_instrument_state::{EstimateQuality, FreshnessPolicy, resolve};
use sim_core::{SimInstant, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic};

use super::*;
use crate::{AltitudeSource, DisplayAxes, FeedConfig, FlightSample, StateFeed};

struct Target {
    scene: Vec<u8>,
    pixels: Vec<u8>,
}

impl Target {
    fn new() -> Self {
        Self {
            scene: vec![0; PANEL_SCENE_BYTES],
            pixels: vec![0x55; PANEL_RGBA_BYTES],
        }
    }

    fn buffers(&mut self) -> PanelBuffers<'_> {
        PanelBuffers {
            scene: &mut self.scene,
            pixels: &mut self.pixels,
            dims: FramebufferDims::tight(PANEL_WIDTH_PX, PANEL_HEIGHT_PX),
        }
    }

    fn pixel(&self, x: usize, y: usize) -> [u8; 4] {
        let at = (y * PANEL_WIDTH_PX as usize + x) * 4;
        [
            self.pixels[at],
            self.pixels[at + 1],
            self.pixels[at + 2],
            self.pixels[at + 3],
        ]
    }

    fn count(&self, matches: impl Fn(&[u8]) -> bool) -> usize {
        self.pixels
            .as_chunks::<4>()
            .0
            .iter()
            .filter(|p| matches(p.as_slice()))
            .count()
    }
}

fn panel(pitch_rad: f64, heading_rad: f64) -> PanelData {
    let origin = Geodetic::from_degrees(39.1, -75.6, 100.0);
    let mut vehicle = VehicleState::level_flight(&origin, heading_rad, 50.0);
    vehicle.attitude *= DQuat::from_rotation_y(pitch_rad);
    let mut feed = StateFeed::new(FeedConfig {
        body: BodyEllipsoid::WGS84,
        axes: DisplayAxes::AEROSPACE,
        origin,
        origin_id: 1,
        altitude: AltitudeSource::OriginRelative,
    });
    let sample = FlightSample::truth(SimInstant::START, vehicle);
    resolve(&feed.state(&sample).unwrap(), &FreshnessPolicy::default())
}

fn failed_panel() -> PanelData {
    let origin = Geodetic::from_degrees(39.1, -75.6, 100.0);
    let mut feed = StateFeed::new(FeedConfig {
        body: BodyEllipsoid::WGS84,
        axes: DisplayAxes::AEROSPACE,
        origin,
        origin_id: 1,
        altitude: AltitudeSource::OriginRelative,
    });
    let sample = FlightSample::truth(
        SimInstant::START,
        VehicleState::level_flight(&origin, 0.0, 50.0),
    );
    let mut state = feed.state(&sample).unwrap();
    state.quality = EstimateQuality::Unusable;
    resolve(&state, &FreshnessPolicy::default())
}

fn pfd(target: &mut Target, data: &PanelData) -> PanelRender {
    render_pfd(
        data,
        &PfdConfig::default(),
        None,
        FrameId::default(),
        &mut target.buffers(),
    )
    .unwrap()
}

#[test]
fn the_panel_constants_are_one_full_frame() {
    assert_eq!((PANEL_WIDTH_PX, PANEL_HEIGHT_PX), (480, 360));
    assert_eq!(PANEL_RGBA_BYTES, 480 * 360 * 4);
}

#[test]
fn the_same_state_paints_the_same_pixels_and_echoes_the_frame_ids() {
    let data = panel(0.05, 1.0);
    let frame = FrameId {
        frame_generation: 7,
        render_generation: 9,
    };
    let (mut first, mut second) = (Target::new(), Target::new());
    let config = PfdConfig::default();
    let a = render_pfd(&data, &config, None, frame, &mut first.buffers()).unwrap();
    let b = render_pfd(&data, &config, None, frame, &mut second.buffers()).unwrap();
    assert_eq!(a, b);
    assert_eq!(first.pixels, second.pixels);
    assert_eq!(a.report.frame, frame);
    assert!(a.scene_bytes > 0);
}

#[test]
fn both_panels_stay_inside_indicates_work_budget() {
    let mut target = Target::new();
    let data = panel(0.05, 1.0);
    assert!(
        pfd(&mut target, &data)
            .report
            .work
            .within(&RenderWork::BUDGET)
    );
    let hsi = render_hsi(&data, None, FrameId::default(), &mut target.buffers()).unwrap();
    assert!(hsi.report.work.within(&RenderWork::BUDGET));
}

#[test]
fn nose_up_puts_sky_in_the_middle_of_the_horizon_and_nose_down_puts_ground() {
    // Left of the fixed aircraft symbol and clear of the pitch ladder's bars.
    let (x, y) = (215, 200);
    let mut target = Target::new();
    pfd(&mut target, &panel(20f64.to_radians(), 0.0));
    let [r, _, b, a] = target.pixel(x, y);
    assert_eq!(a, 255);
    assert!(b > r, "sky is blue: {:?}", target.pixel(x, y));
    pfd(&mut target, &panel(-20f64.to_radians(), 0.0));
    let [r, _, b, _] = target.pixel(x, y);
    assert!(r > b, "ground is brown: {:?}", target.pixel(x, y));
}

#[test]
fn a_failed_attitude_paints_the_red_cross_instead_of_a_plausible_ball() {
    let is_red = |p: &[u8]| p[0] > 180 && p[1] < 80 && p[2] < 80;
    let mut target = Target::new();
    pfd(&mut target, &panel(0.0, 0.0));
    let healthy = target.count(is_red);
    pfd(&mut target, &failed_panel());
    let failed = target.count(is_red);
    assert!(failed > healthy + 1_000, "{healthy} -> {failed}");
}

#[test]
fn the_hsi_turns_with_the_heading() {
    let mut target = Target::new();
    render_hsi(
        &panel(0.0, 0.0),
        None,
        FrameId::default(),
        &mut target.buffers(),
    )
    .unwrap();
    let north = target.pixels.clone();
    render_hsi(
        &panel(0.0, 1.0),
        None,
        FrameId::default(),
        &mut target.buffers(),
    )
    .unwrap();
    assert_ne!(north, target.pixels);
}

#[test]
fn a_framebuffer_that_would_clip_the_panel_is_refused_before_any_drawing() {
    let mut target = Target::new();
    let mut buffers = target.buffers();
    buffers.dims = FramebufferDims::tight(PANEL_WIDTH_PX - 1, PANEL_HEIGHT_PX);
    let refused = render_hsi(&panel(0.0, 0.0), None, FrameId::default(), &mut buffers);
    assert_eq!(
        refused,
        Err(PanelError::FramebufferClipsPanel {
            width: 479,
            height: 360,
            min_width: 480,
            min_height: 360
        }),
    );
    assert!(target.pixels.iter().all(|byte| *byte == 0x55));
}

#[test]
fn a_scene_buffer_too_small_names_the_panel() {
    let mut target = Target::new();
    let mut buffers = target.buffers();
    buffers.scene = &mut [];
    let refused = render_hsi(&panel(0.0, 0.0), None, FrameId::default(), &mut buffers);
    assert!(matches!(
        refused,
        Err(PanelError::Scene {
            panel: "HSI",
            capacity: 0,
            ..
        })
    ));
    let mut target = Target::new();
    let mut small = [0u8; 64];
    let mut buffers = target.buffers();
    buffers.scene = &mut small;
    let refused = render_pfd(
        &panel(0.0, 0.0),
        &PfdConfig::default(),
        None,
        FrameId::default(),
        &mut buffers,
    );
    assert!(matches!(
        refused,
        Err(PanelError::Scene {
            panel: "PFD",
            capacity: 64,
            ..
        })
    ));
}

#[test]
fn a_pixel_slice_shorter_than_its_shape_is_a_raster_failure_and_stays_untouched() {
    let mut target = Target::new();
    let mut short = vec![0x55; 16];
    let mut buffers = target.buffers();
    buffers.pixels = &mut short;
    let refused = render_hsi(&panel(0.0, 0.0), None, FrameId::default(), &mut buffers);
    assert!(matches!(
        refused,
        Err(PanelError::Raster {
            panel: "HSI",
            source: RasterError::FramebufferTooSmall { .. }
        })
    ));
    assert!(short.iter().all(|byte| *byte == 0x55));
}

#[test]
fn a_larger_framebuffer_keeps_the_panels_scale() {
    let data = panel(0.05, 1.0);
    let mut exact = Target::new();
    pfd(&mut exact, &data);
    let width = PANEL_WIDTH_PX + 20;
    let mut scene = vec![0; PANEL_SCENE_BYTES];
    let mut pixels = vec![0; width as usize * PANEL_HEIGHT_PX as usize * 4];
    let mut buffers = PanelBuffers {
        scene: &mut scene,
        pixels: &mut pixels,
        dims: FramebufferDims::tight(width, PANEL_HEIGHT_PX),
    };
    render_pfd(
        &data,
        &PfdConfig::default(),
        None,
        FrameId::default(),
        &mut buffers,
    )
    .unwrap();
    let row = PANEL_WIDTH_PX as usize * 4;
    assert_eq!(pixels[..row], exact.pixels[..row]);
}
