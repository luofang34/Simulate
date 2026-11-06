use std::f32::consts::{FRAC_PI_2, PI};

use glam::DQuat;
use indicate_instrument_hud::{HudContext, HudProfile, HudProjection, WorldSource};
use indicate_instrument_raster::{FrameId, FramebufferDims, render};
use indicate_instrument_scene::{Anchor, LayerId, PaintMode, Rgba8, SceneWriter};
use indicate_instrument_state::{FreshnessPolicy, PanelData, Quat, Sig, resolve};
use sim_core::{SimInstant, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_render::VectorVertex;

use super::*;
use crate::{
    AltitudeSource, DisplayAxes, FeedConfig, FlightSample, PANEL_SCENE_BYTES, PanelBuffers,
    StateFeed, render_hud,
};

const GREEN: Rgba8 = Rgba8::rgba(90, 255, 130, 255);
const WHITE: Rgba8 = Rgba8::rgba(255, 255, 255, 255);

/// Encodes `draw` inside one valid layer envelope.
fn layered(draw: impl FnOnce(&mut SceneWriter<'_>)) -> Vec<u8> {
    let mut buf = vec![0; PANEL_SCENE_BYTES];
    let mut w = SceneWriter::new(&mut buf).unwrap();
    w.begin_layer(LayerId::Attitude).unwrap();
    w.save().unwrap();
    draw(&mut w);
    w.restore().unwrap();
    w.end_layer(LayerId::Attitude).unwrap();
    let n = w.finish();
    buf.truncate(n);
    buf
}

/// Pixels the CPU rasterizer paints.
fn raster_mask(scene: &[u8], w: u32, h: u32) -> Vec<bool> {
    let mut px = vec![0; (w * h * 4) as usize];
    render(
        scene,
        &mut px,
        FramebufferDims::tight(w, h),
        FrameId::default(),
    )
    .unwrap();
    px.as_chunks::<4>().0.iter().map(|p| p[3] > 0).collect()
}

fn inside(p: [f32; 2], a: [f32; 2], b: [f32; 2], c: [f32; 2]) -> bool {
    let cross =
        |o: [f32; 2], u: [f32; 2]| (u[0] - o[0]) * (p[1] - o[1]) - (u[1] - o[1]) * (p[0] - o[0]);
    let (d0, d1, d2) = (cross(a, b), cross(b, c), cross(c, a));
    (d0 >= 0.0 && d1 >= 0.0 && d2 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0 && d2 <= 0.0)
}

/// Pixels whose centre a vector triangle covers inside its clip rectangle,
/// the coverage one MSAA sample at the centre would see.
fn vector_mask(vertices: &[VectorVertex], w: u32, h: u32) -> Vec<bool> {
    let mut mask = vec![false; (w * h) as usize];
    for t in vertices.as_chunks::<3>().0 {
        let [a, b, c] = [t[0].pos_px, t[1].pos_px, t[2].pos_px];
        let clip = t[0].clip_rect_px;
        let lo_x = a[0].min(b[0]).min(c[0]).max(clip[0]).floor().max(0.0) as u32;
        let hi_x = a[0].max(b[0]).max(c[0]).min(clip[2]).ceil().min(w as f32) as u32;
        let lo_y = a[1].min(b[1]).min(c[1]).max(clip[1]).floor().max(0.0) as u32;
        let hi_y = a[1].max(b[1]).max(c[1]).min(clip[3]).ceil().min(h as f32) as u32;
        for y in lo_y..hi_y {
            for x in lo_x..hi_x {
                let p = [x as f32 + 0.5, y as f32 + 0.5];
                let clipped =
                    p[0] < clip[0] || p[1] < clip[1] || p[0] >= clip[2] || p[1] >= clip[3];
                if !clipped && inside(p, a, b, c) {
                    mask[(y * w + x) as usize] = true;
                }
            }
        }
    }
    mask
}

/// Pixels where the two masks disagree although the raster mask is uniform
/// over the pixel's 3x3 neighbourhood: a disagreement away from any edge.
fn interior_mismatches(raster: &[bool], vector: &[bool], w: u32, h: u32) -> Vec<(u32, u32)> {
    let at = |x: i64, y: i64| {
        let (x, y) = (x.clamp(0, w as i64 - 1), y.clamp(0, h as i64 - 1));
        raster[(y as u32 * w + x as u32) as usize]
    };
    let mut out = Vec::new();
    for y in 0..h {
        for x in 0..w {
            let i = (y * w + x) as usize;
            if raster[i] == vector[i] {
                continue;
            }
            let uniform =
                (-1..=1).all(|dy| (-1..=1).all(|dx| at(x as i64 + dx, y as i64 + dy) == raster[i]));
            if uniform {
                out.push((x, y));
            }
        }
    }
    out
}

fn assert_parity(scene: &[u8], w: u32, h: u32) -> (usize, usize) {
    let raster = raster_mask(scene, w, h);
    let vertices = tessellate_scene_cmds(scene, w as f32, h as f32, w, h).unwrap();
    assert_eq!(vertices.len() % 3, 0);
    let vector = vector_mask(&vertices, w, h);
    let bad = interior_mismatches(&raster, &vector, w, h);
    assert!(
        bad.is_empty(),
        "interior coverage differs from raster at {bad:?}"
    );
    let count = |m: &[bool]| m.iter().filter(|&&b| b).count();
    let (r, v) = (count(&raster), count(&vector));
    assert!(r > 0, "scene painted nothing");
    let diff = r.abs_diff(v) as f32 / r as f32;
    assert!(diff < 0.08, "coverage {v} vs raster {r}");
    (r, v)
}

fn covered(scene: &[u8], w: u32, h: u32, x: u32, y: u32) -> (bool, bool) {
    let raster = raster_mask(scene, w, h);
    let vertices = tessellate_scene_cmds(scene, w as f32, h as f32, w, h).unwrap();
    let vector = vector_mask(&vertices, w, h);
    let i = (y * w + x) as usize;
    (raster[i], vector[i])
}

const CONCAVE: [[f32; 2]; 6] = [
    [10.0, 10.0],
    [70.0, 10.0],
    [70.0, 25.0],
    [30.0, 25.0],
    [30.0, 60.0],
    [10.0, 60.0],
];

#[test]
fn concave_polygon_fill_matches_raster_and_strokes_only_on_request() {
    let fill = layered(|w| {
        w.fill_color(GREEN).unwrap();
        w.stroke(WHITE, 3.0).unwrap();
        w.polygon(PaintMode::Fill, &CONCAVE).unwrap();
    });
    assert_parity(&fill, 80, 70);
    // Outside the fill, where only a stroke would reach.
    assert_eq!(covered(&fill, 80, 70, 71, 17), (false, false));
    // Inside the notch of the L, which a convex fan would wrongly fill.
    assert_eq!(covered(&fill, 80, 70, 50, 45), (false, false));

    let stroke = layered(|w| {
        w.stroke(WHITE, 3.0).unwrap();
        w.polygon(PaintMode::Stroke, &CONCAVE).unwrap();
    });
    assert_parity(&stroke, 80, 70);
    assert_eq!(covered(&stroke, 80, 70, 20, 40), (false, false));
    assert_eq!(covered(&stroke, 80, 70, 71, 17), (true, true));
}

#[test]
fn rotated_rect_with_fill_and_stroke_matches_raster() {
    let scene = layered(|w| {
        w.translate(40.0, 35.0).unwrap();
        w.rotate(0.4).unwrap();
        w.fill_color(GREEN).unwrap();
        w.stroke(WHITE, 2.5).unwrap();
        w.rect(PaintMode::FillStroke, -20.0, -12.0, 40.0, 24.0)
            .unwrap();
        w.rect(PaintMode::Stroke, -30.0, -25.0, 10.0, 6.0).unwrap();
    });
    assert_parity(&scene, 80, 70);
}

#[test]
fn circles_and_arcs_match_raster() {
    let scene = layered(|w| {
        w.fill_color(GREEN).unwrap();
        w.stroke(WHITE, 2.0).unwrap();
        w.circle(PaintMode::Fill, 20.0, 20.0, 12.0).unwrap();
        w.circle(PaintMode::Stroke, 60.0, 20.0, 12.0).unwrap();
        w.translate(40.0, 50.0).unwrap();
        w.rotate(FRAC_PI_2).unwrap();
        w.stroke(WHITE, 3.0).unwrap();
        w.arc(0.0, 0.0, 15.0, 0.0, -PI * 0.75).unwrap();
    });
    assert_parity(&scene, 80, 70);
    // The stroked circle leaves its centre open; the filled one does not.
    assert_eq!(covered(&scene, 80, 70, 60, 20), (false, false));
    assert_eq!(covered(&scene, 80, 70, 20, 20), (true, true));
}

#[test]
fn lines_polylines_text_and_clips_match_raster() {
    let scene = layered(|w| {
        w.stroke(GREEN, 2.0).unwrap();
        w.line(4.0, 4.0, 76.0, 30.0).unwrap();
        w.polyline(&[[5.0, 60.0], [20.0, 40.0], [35.0, 60.0], [50.0, 40.0]])
            .unwrap();
        w.fill_color(WHITE).unwrap();
        w.text(40.0, 20.0, 14.0, Anchor::CENTER, "ALT 120").unwrap();
        w.text(78.0, 68.0, 7.0, Anchor::BASELINE_LEFT, "N").unwrap();
        w.save().unwrap();
        w.clip_rect(55.0, 40.0, 10.0, 10.0).unwrap();
        w.rect(PaintMode::Fill, 50.0, 35.0, 30.0, 30.0).unwrap();
        w.restore().unwrap();
    });
    assert_parity(&scene, 80, 70);
    assert_eq!(
        covered(&scene, 80, 70, 70, 45),
        (false, false),
        "clip must cut the fill"
    );
    assert_eq!(covered(&scene, 80, 70, 60, 45), (true, true));
}

#[test]
fn a_transparent_paint_emits_no_geometry() {
    let scene = layered(|w| {
        w.fill_color(Rgba8::rgba(0, 0, 0, 0)).unwrap();
        w.rect(PaintMode::Fill, 0.0, 0.0, 10.0, 10.0).unwrap();
    });
    assert!(
        tessellate_scene_cmds(&scene, 20.0, 20.0, 20, 20)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn a_scene_outside_any_layer_or_with_a_missing_glyph_fails_closed() {
    let mut buf = vec![0; 256];
    let mut w = SceneWriter::new(&mut buf).unwrap();
    w.rect(PaintMode::Fill, 0.0, 0.0, 4.0, 4.0).unwrap();
    let n = w.finish();
    let res = tessellate_scene_cmds(&buf[..n], 20.0, 20.0, 20, 20);
    assert!(matches!(
        res,
        Err(PanelError::VectorScene {
            source: VectorSceneError::Layers(_)
        })
    ));

    let scene = layered(|w| {
        w.text(5.0, 10.0, 7.0, Anchor::BASELINE_LEFT, "A\u{263A}")
            .unwrap();
    });
    let res = tessellate_scene_cmds(&scene, 20.0, 20.0, 20, 20);
    assert!(matches!(
        res,
        Err(PanelError::VectorScene {
            source: VectorSceneError::Glyphs(_)
        })
    ));
    let res = tessellate_scene_cmds(&scene, 20.0, 20.0, 0, 20);
    assert!(matches!(
        res,
        Err(PanelError::VectorScene {
            source: VectorSceneError::EmptyFrame { .. }
        })
    ));
}

fn hud_inputs() -> (PanelData, HudContext) {
    let origin = Geodetic::from_degrees(39.1, -75.6, 100.0);
    let mut vehicle = VehicleState::level_flight(&origin, 1.0, 50.0);
    vehicle.attitude *= DQuat::from_rotation_y(0.05);
    let mut feed = StateFeed::new(FeedConfig {
        body: BodyEllipsoid::WGS84,
        axes: DisplayAxes::AEROSPACE,
        origin,
        origin_id: 1,
        altitude: AltitudeSource::OriginRelative,
        heading_mode: crate::HeadingMode::default(),
    });
    let sample = FlightSample::truth(SimInstant::START, vehicle);
    let data = resolve(&feed.state(&sample).unwrap(), &FreshnessPolicy::default());
    let context = HudContext {
        profile: HudProfile::Screen,
        projection: Sig::valid(HudProjection {
            vertical_fov_rad: 1.0,
            body_from_view: Quat::IDENTITY,
            source: WorldSource::Synthetic,
        }),
        ..HudContext::default()
    };
    (data, context)
}

#[test]
fn the_vector_hud_fills_a_retina_target_with_finite_in_bounds_geometry() {
    let (data, context) = hud_inputs();
    assert_eq!(
        VectorHudPanel::design_frame(2560, 1440).unwrap(),
        [1280.0, 720.0]
    );
    let mut hud = VectorHudPanel::new();
    let vertices = hud.tessellate(&data, &context, None, 2560, 1440).unwrap();
    assert!(!vertices.is_empty());
    assert_eq!(vertices.len() % 3, 0);
    for v in &vertices {
        assert!(
            v.pos_px
                .iter()
                .chain(&v.color_premul)
                .chain(&v.clip_rect_px)
                .all(|c| c.is_finite())
        );
        let [x0, y0, x1, y1] = v.clip_rect_px;
        assert!(
            x0 >= 0.0 && y0 >= 0.0 && x1 <= 2560.0 && y1 <= 1440.0,
            "clip {:?}",
            v.clip_rect_px
        );
        assert!(
            v.color_premul[..3]
                .iter()
                .all(|&c| c <= v.color_premul[3] + 1e-6)
        );
        assert!(v.depth > 0.0 && v.depth <= 1.0);
    }
    for t in vertices.as_chunks::<3>().0 {
        let [x0, y0, x1, y1] = t[0].clip_rect_px;
        let xs = t.iter().map(|v| v.pos_px[0]);
        let ys = t.iter().map(|v| v.pos_px[1]);
        let (lo_x, hi_x) = xs.fold((f32::MAX, f32::MIN), |(l, h), x| (l.min(x), h.max(x)));
        let (lo_y, hi_y) = ys.fold((f32::MAX, f32::MIN), |(l, h), y| (l.min(y), h.max(y)));
        assert!(
            hi_x > x0 && lo_x < x1 && hi_y > y0 && lo_y < y1,
            "triangle outside its clip"
        );
        assert!(
            t.iter()
                .all(|v| v.clip_rect_px == t[0].clip_rect_px && v.depth == t[0].depth)
        );
    }
    let depths: Vec<f32> = vertices.iter().map(|v| v.depth).collect();
    assert!(
        depths.windows(2).all(|d| d[0] <= d[1]),
        "paint order must not go backwards"
    );
}

#[test]
fn the_vector_hud_covers_what_the_raster_hud_paints() {
    let (data, context) = hud_inputs();
    let (w, h) = (1280_u32, 720_u32);
    let mut scene = vec![0; PANEL_SCENE_BYTES];
    let mut pixels = vec![0; (w * h * 4) as usize];
    let render = render_hud(
        &data,
        &context,
        None,
        FrameId::default(),
        &mut PanelBuffers {
            scene: &mut scene,
            pixels: &mut pixels,
            dims: FramebufferDims::tight(w, h),
        },
    )
    .unwrap();
    let raster: Vec<bool> = pixels.as_chunks::<4>().0.iter().map(|p| p[3] > 0).collect();
    let vertices =
        tessellate_scene_cmds(&scene[..render.scene_bytes], w as f32, h as f32, w, h).unwrap();
    let vector = vector_mask(&vertices, w, h);
    let bad = interior_mismatches(&raster, &vector, w, h);
    let painted = raster.iter().filter(|&&b| b).count();
    assert!(painted > 1000);
    assert!(
        bad.len() * 1000 <= painted,
        "{} interior mismatches of {painted}: {:?}",
        bad.len(),
        &bad[..bad.len().min(20)]
    );
}

#[test]
fn a_hud_target_outside_the_layout_aspect_is_refused() {
    assert!(matches!(
        VectorHudPanel::design_frame(3000, 500),
        Err(PanelError::HudFrameOutOfRange {
            width: 3000,
            height: 500
        })
    ));
    assert!(VectorHudPanel::design_frame(0, 500).is_err());
    let [w, h] = VectorHudPanel::design_frame(500, 1000).unwrap();
    assert!(w >= 390.0 && (w / h - 0.5).abs() < 0.01);
}

#[test]
fn vector_pfd_and_hsi_tessellate_into_physical_triangles() {
    let (data, _) = hud_inputs();
    let mut panels = VectorFlightPanels::new();
    let pfd = panels
        .tessellate_pfd(&data, &crate::PfdConfig::default(), None, 960, 720)
        .unwrap();
    let hsi = panels.tessellate_hsi(&data, None, 960, 720).unwrap();
    assert!(pfd.len() > 300 && pfd.len().is_multiple_of(3));
    assert!(hsi.len() > 300 && hsi.len().is_multiple_of(3));
}
