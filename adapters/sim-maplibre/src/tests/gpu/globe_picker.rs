//! The airport-selection globe draws, follows drag and zoom gestures, picks and projects
//! consistently, and draws its markers and runways.

use std::time::Duration;

use super::{sample_scenery_and_runtime, try_host_gpu};
use crate::{GlobeAirportMarker, GlobePickerSurface, GlobeRunwayEndOverlay};

const SIZE: [u32; 2] = [160, 120];
const CENTER: [f32; 2] = [80.0, 60.0];

fn picker() -> Option<GlobePickerSurface> {
    let gpu = try_host_gpu()?;
    let (_, _, runtime) = sample_scenery_and_runtime(gpu);
    Some(GlobePickerSurface::new(&runtime, SIZE, false).expect("picker"))
}

/// Draws frames until nothing is loading or waiting to be drawn; returns the last frame.
fn settle(picker: &mut GlobePickerSurface) -> Vec<u8> {
    for _ in 0..400 {
        assert!(picker.render().expect("frame").is_some(), "colour texture");
        if !picker.needs_redraw() {
            break;
        }
        std::thread::yield_now();
    }
    picker.read_rgba().expect("rgba")
}

fn pixel(rgba: &[u8], [x, y]: [f32; 2]) -> [u8; 4] {
    assert!(
        (0.0..SIZE[0] as f32).contains(&x) && (0.0..SIZE[1] as f32).contains(&y),
        "({x}, {y}) is outside the frame"
    );
    let at = ((y as usize) * SIZE[0] as usize + x as usize) * 4;
    [rgba[at], rgba[at + 1], rgba[at + 2], rgba[at + 3]]
}

fn near(a: [f32; 2], b: [f32; 2], tolerance: f32) -> bool {
    (a[0] - b[0]).abs() <= tolerance && (a[1] - b[1]).abs() <= tolerance
}

#[test]
fn the_globe_draws_and_picks_where_it_projects() {
    let Some(mut picker) = picker() else {
        return;
    };
    assert_eq!(picker.size_px(), SIZE);
    let rgba = settle(&mut picker);
    let [r, g, b, a] = pixel(&rgba, CENTER);
    assert!(
        a == 255 && b > r && b > g,
        "dark-ocean globe at the centre: {r},{g},{b},{a}"
    );
    let (lat, lon, zoom) = picker.camera_center_and_zoom();
    assert!(
        (lat - 39.5).abs() < 0.5 && (lon + 75.0).abs() < 0.5,
        "{lat} {lon}"
    );
    assert!((zoom - 2.6).abs() < 0.3, "{zoom}");
    let (pick_lat, pick_lon) = picker.screen_to_lat_lon(CENTER).expect("ground");
    assert!((pick_lat - lat).abs() < 1.0 && (pick_lon - lon).abs() < 1.0);
    let (drawn, occluded) = picker
        .project_lat_lon(pick_lat, pick_lon)
        .expect("projected");
    assert!(near(drawn, CENTER, 1.0) && !occluded, "{drawn:?}");
    let (_, antipode_hidden) = picker
        .project_lat_lon(-lat, lon + 180.0)
        .expect("projected");
    assert!(antipode_hidden, "the far side is behind the planet");
    // Zoomed out in a larger frame the whole globe shows, with space in the corners.
    picker.resize([480, 360]);
    assert_eq!(picker.size_px(), [480, 360]);
    picker
        .jump_or_fly_to(lat, lon, Some(0.5))
        .expect("zoom out");
    assert!(
        picker.screen_to_lat_lon([240.0, 180.0]).is_some(),
        "the globe"
    );
    assert!(
        picker.screen_to_lat_lon([2.0, 2.0]).is_none(),
        "the corner shows space"
    );
    picker.resize([0, 360]);
    assert_eq!(picker.size_px(), [480, 360], "a zero size is ignored");
}

#[test]
fn drag_and_zoom_move_projected_ground_deterministically() {
    let Some(mut picker) = picker() else {
        return;
    };
    let _first = settle(&mut picker);
    let (lat, lon) = picker.screen_to_lat_lon(CENTER).expect("ground");
    assert!(picker.drag_pixels(CENTER, [CENTER[0] + 30.0, CENTER[1]]));
    let (dragged, _) = picker.project_lat_lon(lat, lon).expect("projected");
    assert!(
        near(dragged, [CENTER[0] + 30.0, CENTER[1]], 1.5),
        "{dragged:?}"
    );
    let east = picker
        .screen_to_lat_lon([CENTER[0] + 20.0, CENTER[1]])
        .expect("east");
    let (before_zoom, _) = picker.project_lat_lon(east.0, east.1).expect("projected");
    let zoom_before = picker.camera_center_and_zoom().2;
    assert!(picker.zoom_at_pixel(CENTER, 1.0));
    let zoom_after = picker.camera_center_and_zoom().2;
    assert!(
        (zoom_after - zoom_before - 1.0).abs() < 0.1,
        "{zoom_before} -> {zoom_after}"
    );
    let (after_zoom, _) = picker.project_lat_lon(east.0, east.1).expect("projected");
    let spread = |p: [f32; 2]| p[0] - CENTER[0];
    let ratio = spread(after_zoom) / spread(before_zoom);
    assert!(
        (1.8..2.2).contains(&ratio),
        "one zoom level doubles distances: {ratio}"
    );
    let frame = settle(&mut picker);
    assert_eq!(frame.len(), (SIZE[0] * SIZE[1] * 4) as usize);
}

#[test]
fn markers_and_selected_runways_render_on_the_globe() {
    let Some(mut picker) = picker() else {
        return;
    };
    picker
        .jump_or_fly_to(40.69, -74.17, Some(4.5))
        .expect("jump");
    let marker = |code: &str, lat_deg, lon_deg, towered, selected| GlobeAirportMarker {
        code: code.to_owned(),
        label: code.to_owned(),
        lat_deg,
        lon_deg,
        tier: 1,
        towered,
        selected,
    };
    picker
        .set_airport_markers(&[
            marker("KEWR", 40.69, -74.17, true, true),
            marker("KABE", 40.65, -75.44, true, false),
            marker("N07", 40.98, -73.0, false, false),
        ])
        .expect("markers");
    let rgba = settle(&mut picker);
    let at = |lat, lon| {
        let (px, occluded) = picker.project_lat_lon(lat, lon).expect("projected");
        assert!(!occluded);
        pixel(&rgba, px)
    };
    let [r, g, b, _] = at(40.69, -74.17);
    assert!(
        r > 200 && g > 150 && b < 120,
        "gold selected KEWR: {r},{g},{b}"
    );
    let [r, g, b, _] = at(40.65, -75.44);
    assert!(
        b > 180 && g > 150 && r < 120,
        "cyan towered KABE: {r},{g},{b}"
    );
    let [r, g, b, _] = at(40.98, -73.0);
    assert!(
        r > 150 && b > 200 && g < 170,
        "violet untowered N07: {r},{g},{b}"
    );

    picker.set_airport_markers(&[]).expect("clear markers");
    picker
        .jump_or_fly_to(40.685, -74.18, Some(13.0))
        .expect("jump");
    let runway = GlobeRunwayEndOverlay {
        designator: "22L".to_owned(),
        threshold_lat_deg: 40.6995,
        threshold_lon_deg: -74.1625,
        opposite_lat_deg: 40.6745,
        opposite_lon_deg: -74.1878,
        width_m: 45.0,
        selected: true,
    };
    picker.set_selected_runways(&[runway]).expect("runways");
    let rgba = settle(&mut picker);
    let (mid, _) = picker.project_lat_lon(40.687, -74.17515).expect("mid");
    let [r, g, b, _] = pixel(&rgba, mid);
    assert!(
        r > 180 && g > 180 && b > 180,
        "pale pavement strip: {r},{g},{b}"
    );
}

#[test]
fn fly_to_animates_smoothly_across_frames_until_arriving() {
    let Some(mut picker) = picker() else {
        return;
    };
    picker
        .jump_or_fly_to(40.69, -74.17, Some(4.5))
        .expect("initial jump");
    let _settled = settle(&mut picker);

    picker
        .fly_to(51.47, -0.46, Some(4.5))
        .expect("smooth fly_to");
    assert!(
        picker.needs_redraw(),
        "active flight requests redraw frames"
    );

    for _ in 0..8 {
        assert!(
            picker
                .render_step(Duration::from_millis(100))
                .expect("step")
                .is_some()
        );
    }
    let (_, mid_lon, _) = picker.camera_center_and_zoom();
    assert!(
        mid_lon > -72.0 && mid_lon < -3.0,
        "camera moves smoothly through intermediate longitudes, got {mid_lon}"
    );

    for _ in 0..120 {
        let _ = picker
            .render_step(Duration::from_millis(100))
            .expect("step");
        if !picker.needs_redraw() {
            break;
        }
    }
    let (end_lat, end_lon, end_zoom) = picker.camera_center_and_zoom();
    assert!(
        (end_lat - 51.47).abs() < 0.25 && (end_lon + 0.46).abs() < 0.25,
        "arrived at EGLL: {end_lat}, {end_lon}"
    );
    assert!((end_zoom - 4.5).abs() < 0.2, "arrived zoom: {end_zoom}");
}
