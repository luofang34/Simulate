use std::f64::consts::FRAC_PI_2;
use std::io::Cursor;

use glam::DVec3;
use image::{ImageFormat, Luma, Rgb, RgbImage};
use navigate_data::{GroundHeight, LayerKind, MemoryStore, Sha256Digest};
use sim_core::VehicleState;
use sim_geo::Geodetic;
use sim_link::decode_fragment;
use sim_navdata::{
    HttpAnswer, PackagePlan, SourceSet, TileTransport, plan_package, prefetch, seal_package,
};

use super::{
    CameraWorld, MapView, WORLD_MEMORY_CEILING_BYTES, world_line, world_memory_bytes,
    world_memory_refusal,
};
use crate::{HostError, hosted_request, package_camera, package_route_msl};

const FIRST: &str = "v=1&r=41.700000,-1.050000,,,ZA1;41.700000,-0.800000,,,ZA2;41.800000,-0.800000,,,ZA3;\
41.800000,-1.000000,,,ZA4;41.740000,-1.050000,,,ZA5&h=2000.0&hd=msl&start=41.700000,-1.070000,,90.0\
&t=20260615T120400Z&veh=c172&seed=1&nav=ins.baro.visual;ins.baro;ins.baro.gnss&fly=ins.baro.gnss\
&go=1&mb=200";

struct Flat(f64);

impl GroundHeight for Flat {
    fn ground_m(&self, _point: [f64; 2]) -> Option<f64> {
        Some(self.0)
    }
}

fn plan(fragment: &str, ground_m: f64) -> PackagePlan {
    let link = decode_fragment(fragment).unwrap();
    let request = hosted_request(&link).unwrap();
    plan_package(
        &package_route_msl(&link).unwrap(),
        package_camera(&request).unwrap(),
        &SourceSet::default(),
        &Flat(ground_m),
    )
    .unwrap()
}

#[test]
fn the_world_costs_each_tile_whole_and_decoded() {
    let plan = plan(FIRST, 300.0);
    let tiles = |kind: LayerKind| {
        plan.size
            .layers
            .iter()
            .filter(|size| size.layer == kind)
            .map(|size| size.tiles)
            .sum::<u64>()
    };
    let expected =
        tiles(LayerKind::Imagery) * 256 * 256 * 4 + tiles(LayerKind::Elevation) * 256 * 256 * 8;
    let sources = SourceSet::default();
    assert_eq!(world_memory_bytes(&plan, &sources), expected);
    // The page measured 144 MB for the 517 + 15 tiles of this link.
    assert!((100_000_000..200_000_000).contains(&expected), "{expected}");
    assert!(world_memory_refusal(&plan, &sources).is_none());
    let said = world_line(&plan, &sources);
    assert_eq!(said.label, "Camera's world");
    assert!(said.text.starts_with(
        "the camera sees the same imagery the onboard map was built from, which favours visual navigation."
    ));
    assert!(
        said.text.ends_with("the page allows 1000 MB."),
        "{}",
        said.text
    );
}

#[test]
fn a_world_too_big_for_the_tab_is_refused_with_its_numbers() {
    // A long low flight with a fine camera: many fine tiles.
    let long = FIRST
        .replace("h=2000.0", "h=600.0")
        .replace("&go=1", "&cam=1600x1200,67.4,90,1&go=1");
    let plan = plan(&long, 300.0);
    let sources = SourceSet::default();
    let needed = world_memory_bytes(&plan, &sources);
    assert!(needed > WORLD_MEMORY_CEILING_BYTES, "{needed}");
    let refusal = world_memory_refusal(&plan, &sources).unwrap();
    assert_eq!(refusal[0].label, "Reason");
    assert!(
        refusal[0].text.contains("the page allows 1000 MB"),
        "{}",
        refusal[0].text
    );
    assert!(refusal[1].text.contains("does not lower the map detail"));
}

/// A short leg flown over a visual set.
const SHORT: &str = "v=1&r=41.700000,-1.050000,,,A;41.700000,-1.030000,,,B&h=2000.0&hd=msl\
&t=20260615T120400Z&veh=c172&seed=1&nav=ins.baro.visual&mb=200";
/// The grey of every imagery tile the fake server holds.
const GREY: u8 = 150;
/// The ground height of every elevation tile it holds, metres.
const GROUND_M: f64 = 300.0;

/// A tile server of uniform grey imagery over flat ground.
struct Uniform;

fn encoded(image: impl Into<image::DynamicImage>, format: ImageFormat) -> Vec<u8> {
    let mut bytes = Cursor::new(Vec::new());
    image.into().write_to(&mut bytes, format).unwrap();
    bytes.into_inner()
}

impl TileTransport for Uniform {
    async fn get(&self, url: &str) -> Result<HttpAnswer, String> {
        let body = if url.contains("arcgisonline") {
            let grey = image::ImageBuffer::from_pixel(256, 256, Luma([GREY]));
            encoded(
                image::DynamicImage::ImageLuma8(grey).to_rgb8(),
                ImageFormat::Jpeg,
            )
        } else {
            let value = (GROUND_M + 32768.0) as u32;
            let pixel = Rgb([(value / 256) as u8, (value % 256) as u8, 0]);
            encoded(RgbImage::from_pixel(256, 256, pixel), ImageFormat::Png)
        };
        Ok(HttpAnswer { status: 200, body })
    }
}

#[test]
fn the_camera_sees_the_sealed_package_the_way_the_trial_does() {
    let link = decode_fragment(SHORT).unwrap();
    let request = hosted_request(&link).unwrap();
    let sources = SourceSet::default();
    let plan = plan_package(
        &package_route_msl(&link).unwrap(),
        package_camera(&request).unwrap(),
        &sources,
        &Flat(GROUND_M),
    )
    .unwrap();
    let mut store = MemoryStore::default();
    let keys = plan.keys(&sources);
    let fetched = pollster::block_on(prefetch(
        &keys,
        &sources,
        u64::MAX,
        &Uniform,
        &mut store,
        &mut |_| {},
    ))
    .unwrap();
    let sealed = pollster::block_on(seal_package(
        &plan,
        &sources,
        "test",
        &fetched.absent,
        &mut store,
    ))
    .unwrap();
    let world = CameraWorld::open_blocking(sealed.memory.clone(), sealed.summary.identity).unwrap();

    let spec = request.trial.terrain_camera.unwrap();
    let first = &request.trial.waypoints[0];
    let over = Geodetic::from_degrees(first.lat_deg, first.lon_deg, first.altitude_m);
    let state = VehicleState::level_flight(&over, FRAC_PI_2, 55.0);
    let pose = spec.camera_mount().pose(&state, DVec3::ZERO);
    let frame = world
        .render(pose.position_ecef, pose.orientation, &spec.intrinsics())
        .unwrap();
    assert_eq!((frame.width(), frame.height()), (400, 300));
    let off = frame
        .pixels()
        .iter()
        .filter(|luma| luma.abs_diff(GREY) > 3)
        .count();
    assert_eq!(off, 0, "every pixel sees the package's ground");

    // From above: the route's ground in dimmed grey, far off it the fill.
    let points = request
        .trial
        .waypoints
        .iter()
        .map(|w| (w.lat_deg, w.lon_deg));
    let view = MapView::around(points, 0.0).unwrap().with_aspect(0.5);
    let rgba = world.overview_rgba(&view, 40, 20);
    assert_eq!(rgba.len(), 40 * 20 * 4);
    let (x, y) = view.pixel((first.lat_deg, first.lon_deg + 0.01), 40.0, 20.0);
    let at = ((y as usize) * 40 + x as usize) * 4;
    assert_eq!(rgba[at], (f64::from(GREY) * 0.6) as u8);
    let far = MapView::around([(10.0, 10.0)], 0.001).unwrap();
    assert_eq!(world.overview_rgba(&far, 2, 2)[..4], [11, 14, 23, 255]);

    let elsewhere = CameraWorld::open_blocking(sealed.memory, Sha256Digest::of(b"another"));
    assert!(matches!(elsewhere, Err(HostError::Trial { .. })));
}

#[test]
fn the_map_view_projects_and_keeps_its_shape() {
    let (x, y) = super::mercator(41.7, -1.05);
    let (lat, lon) = super::unmercator(x, y);
    assert!((lat - 41.7).abs() < 1e-9 && (lon + 1.05).abs() < 1e-9);
    let view = MapView::around([(41.7, -1.05), (41.8, -0.8)], 0.1).unwrap();
    let wide = view.with_aspect(0.25);
    assert!(wide.holds(&view));
    // A box already of the shape is the same box, so it holds itself.
    let ratio = (view.y1 - view.y0) / (view.x1 - view.x0);
    assert!(view.with_aspect(ratio).holds(&view));
    let ratio = (wide.y1 - wide.y0) / (wide.x1 - wide.x0);
    assert!((ratio - 0.25).abs() < 1e-9, "{ratio}");
    let (px, py) = wide.pixel((41.8, -0.8), 800.0, 200.0);
    assert!((0.0..800.0).contains(&px) && (0.0..200.0).contains(&py));
    // About 21 km across at 41.75° north on 800 pixels.
    let m = view.metres_per_pixel(800.0);
    assert!((20.0..40.0).contains(&m), "{m}");
}
