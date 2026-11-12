use std::sync::Arc;

use glam::DVec3;
use maplibre::projection::{ProjectionType, globe::scale};

use super::{
    camera::{EfbCamera, effective_globe_zoom},
    plate::{PLATE_PX, ProceduralPlateSpec, generate_procedural_approach_plate},
    *,
};
use crate::{
    ChartOwnship, ChartSymbologyInput, HybridTileLoader, belief_aviation_chart_style,
    geodesy::destination,
    tests::gpu::{sample_scenery_and_runtime, try_host_gpu},
};

fn kewr_22l() -> ProceduralPlateSpec {
    ProceduralPlateSpec {
        airport_code: "KEWR".to_owned(),
        airport_name: "Newark Liberty Intl".to_owned(),
        runway_designator: "22L".to_owned(),
        threshold_lat_deg: 40.6995,
        threshold_lon_deg: -74.1625,
        true_heading_deg: 207.0,
        elevation_ft: 18.0,
        ils_freq_khz: 108_700,
        glidepath_deg: 3.0,
    }
}

fn ownship(lat_deg: f64, lon_deg: f64, velocity: [f64; 2]) -> ChartOwnship {
    ChartOwnship {
        lat_deg,
        lon_deg,
        true_heading_deg: 90.0,
        ground_velocity_en_mps: velocity,
    }
}

fn unit_sphere(lat_deg: f64, lon_deg: f64) -> DVec3 {
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    DVec3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat)
}

fn screen_up_3d(lat_deg: f64, lon_deg: f64, bearing_deg: f64) -> DVec3 {
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    let east = DVec3::new(-sin_lon, cos_lon, 0.0);
    let north = DVec3::new(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    let (sin_b, cos_b) = bearing_deg.to_radians().sin_cos();
    north * cos_b + east * sin_b
}

#[test]
fn chart_style_uses_vertical_perspective_globe_projection() {
    let style = belief_aviation_chart_style(
        &SceneryEndpoints::catalog(0, 0),
        json!({ "type": "FeatureCollection", "features": [] }),
    )
    .expect("chart style");
    assert_eq!(
        style.projection.as_ref().map(|p| &p.projection_type),
        Some(&ProjectionType::VerticalPerspective)
    );
}

#[test]
fn polar_zoom_compensation_leaves_mid_latitudes_identical_and_matches_sixty_degree_scale() {
    for lat in [-60.0, -40.69, 0.0, 40.69, 60.0] {
        assert_eq!(effective_globe_zoom(10.2, lat), 10.2);
    }
    let ref_radius = scale::radius_pixels(10.2, 60.0);
    for lat in [60.001, 75.0, 85.051_129, 89.9975, 90.0, -89.9975, -90.0] {
        let eff = effective_globe_zoom(10.2, lat);
        assert!(eff < 10.2, "compensated zoom {eff} at {lat}");
        let radius = scale::radius_pixels(eff, lat);
        assert!(
            (radius - ref_radius).abs() / ref_radius < 1e-12,
            "radius {radius} vs {ref_radius} at {lat}"
        );
    }
}

#[test]
fn zoom_is_kept_per_display_mode_and_clamped() {
    let mut camera = EfbCamera::new(40.0, -74.0);
    assert_eq!(camera.zoom(), DEFAULT_COMPACT_ZOOM);
    camera.zoom_by(1.0);
    camera.mode = EfbDisplayMode::ExpandedEfb;
    assert_eq!(camera.zoom(), DEFAULT_EXPANDED_ZOOM);
    camera.zoom_by(100.0);
    assert_eq!(camera.zoom(), EFB_ZOOM_RANGE.1);
    camera.mode = EfbDisplayMode::CompactCorner;
    assert!((camera.zoom() - DEFAULT_COMPACT_ZOOM - 1.0).abs() < 1e-12);
    camera.zoom_by(-100.0);
    assert_eq!(camera.zoom(), EFB_ZOOM_RANGE.0);
}

#[test]
fn track_up_follows_the_ground_track_and_north_up_does_not_rotate() {
    let mut camera = EfbCamera::new(0.0, 0.0);
    camera.follow_ownship(Some(&ownship(40.5, -74.5, [-50.0, -50.0])));
    let spec = camera.spec();
    assert_eq!((spec.center_lat_deg, spec.center_lon_deg), (40.5, -74.5));
    assert!(
        (spec.bearing_deg - 225.0).abs() < 1e-9,
        "{}",
        spec.bearing_deg
    );
    camera.follow_ownship(Some(&ownship(40.5, -74.5, [0.5, 0.0])));
    assert!(
        (camera.spec().bearing_deg - 90.0).abs() < 1e-9,
        "slow: heading"
    );
    camera.set_follow(EfbFollowMode::FollowNorthUp);
    camera.follow_ownship(Some(&ownship(-90.0, 139.2729, [-50.0, -50.0])));
    assert_eq!(camera.spec().center_lat_deg, -90.0);
    assert_eq!(camera.spec().bearing_deg, 0.0);

    // A two-finger twist rotates the map while keeping ownship centred.
    camera.rotate_by_deg(35.0);
    assert_eq!(camera.follow, EfbFollowMode::FollowBearing);
    camera.follow_ownship(Some(&ownship(40.7, -74.1, [0.0, 80.0])));
    assert_eq!(
        (camera.spec().center_lat_deg, camera.spec().center_lon_deg),
        (40.7, -74.1)
    );
    assert!((camera.spec().bearing_deg - 35.0).abs() < 1e-9);
    camera.rotate_by_deg(-50.0);
    assert!((camera.spec().bearing_deg - 345.0).abs() < 1e-9);
}

#[test]
fn panning_moves_the_centre_by_screen_pixels_at_any_bearing_and_recentre_resumes() {
    let mut camera = EfbCamera::new(0.0, 0.0);
    camera.set_follow(EfbFollowMode::FollowTrackUp);
    camera.follow_ownship(Some(&ownship(40.69, -74.17, [0.0, 60.0])));
    let start = camera.spec();
    let radius_px = scale::radius_pixels(start.zoom, start.center_lat_deg);
    let u0 = unit_sphere(start.center_lat_deg, start.center_lon_deg);
    camera.pan_by_pixels(30.0, -40.0);
    assert_eq!(camera.follow, EfbFollowMode::FreePan);
    let after = camera.spec();
    let u1 = unit_sphere(after.center_lat_deg, after.center_lon_deg);
    let moved_px = u0.dot(u1).clamp(-1.0, 1.0).acos() * radius_px;
    assert!((moved_px - 50.0).abs() < 1e-6, "{moved_px} px");
    // Track north: screen right is east, screen up is north.
    assert!(after.center_lon_deg > start.center_lon_deg);
    assert!(after.center_lat_deg > start.center_lat_deg);
    camera.follow_ownship(Some(&ownship(10.0, 10.0, [0.0, 60.0])));
    assert_eq!(
        camera.spec().center_lat_deg,
        after.center_lat_deg,
        "free pan holds"
    );
    camera.recenter();
    assert_eq!(camera.follow, EfbFollowMode::FollowTrackUp);
    camera.follow_ownship(Some(&ownship(10.0, 10.0, [0.0, 60.0])));
    assert_eq!(camera.spec().center_lat_deg, 10.0);
}

#[test]
fn spherical_panning_crosses_the_south_pole_without_clamping_or_orientation_flip() {
    let mut camera = EfbCamera::new(-89.99, 0.0);
    camera.follow_ownship(Some(&ownship(-89.99, 0.0, [0.0, -40.0])));
    let before = camera.spec();
    assert!((before.bearing_deg - 180.0).abs() < 1e-9);
    let up_before = screen_up_3d(
        before.center_lat_deg,
        before.center_lon_deg,
        before.bearing_deg,
    );
    // Pan 120 px straight across 90° S along the 0° -> 180° meridian.
    camera.pan_by_pixels(0.0, -120.0);
    let after = camera.spec();
    assert!(
        after.center_lat_deg.is_finite()
            && after.center_lon_deg.is_finite()
            && after.bearing_deg.is_finite()
    );
    assert!(
        after.center_lat_deg < -89.9,
        "never clamped to Mercator -85.051°: {}",
        after.center_lat_deg
    );
    assert!(
        after.center_lon_deg.abs() > 179.0,
        "crossed 90° S onto opposite meridian: {}",
        after.center_lon_deg
    );
    let up_after = screen_up_3d(
        after.center_lat_deg,
        after.center_lon_deg,
        after.bearing_deg,
    );
    assert!(
        up_before.dot(up_after) > 0.9999,
        "screen orientation stayed steady across the pole: dot={}",
        up_before.dot(up_after)
    );
}

#[test]
fn the_procedural_plate_is_a_512_px_png_spanning_twelve_miles_around_the_final_course() {
    let plate = generate_procedural_approach_plate(&kewr_22l()).unwrap();
    assert_eq!(plate.chart_id, "kewr-ils-22l");
    assert_eq!(plate.title, "ILS OR LOC RWY 22L");
    let image = image::load_from_memory(&plate.png_bytes)
        .unwrap()
        .to_rgba8();
    assert_eq!(image.dimensions(), (PLATE_PX, PLATE_PX));
    let [nw, ne, se, sw] = plate.corners_lon_lat;
    assert_eq!((nw[1], ne[1]), (ne[1], nw[1]));
    assert_eq!((sw[0], se[1]), (nw[0], sw[1]));
    let height_nm = (nw[1] - sw[1]).to_radians() * 6_371_008.8 / 1_852.0;
    assert!((height_nm - 12.0).abs() < 0.01, "{height_nm}");
    let center = [(nw[0] + ne[0]) / 2.0, (nw[1] + sw[1]) / 2.0];
    let threshold = destination(center, 207.0, 3.0 * 1_852.0);
    assert!((threshold[0] + 74.1625).abs() < 1e-4 && (threshold[1] - 40.6995).abs() < 1e-4);
    let banner = image.get_pixel(PLATE_PX / 2, 2).0;
    let tint = image.get_pixel(40, PLATE_PX / 2).0;
    assert!(
        banner[3] > 200 && banner[0] < 80,
        "slate header banner {banner:?}"
    );
    assert!(
        tint[3] > 100 && tint[3] < 220 && tint[0] > 200,
        "translucent ivory {tint:?}"
    );
}

#[test]
fn the_final_course_and_fixes_are_inked_where_they_lie() {
    let plate = generate_procedural_approach_plate(&kewr_22l()).unwrap();
    let image = image::load_from_memory(&plate.png_bytes)
        .unwrap()
        .to_rgba8();
    let [nw, ne, _, sw] = plate.corners_lon_lat;
    let to_px = |[lon, lat]: [f64; 2]| {
        let x = (lon - nw[0]) / (ne[0] - nw[0]) * f64::from(PLATE_PX);
        let y = (nw[1] - lat) / (nw[1] - sw[1]) * f64::from(PLATE_PX);
        (x as u32, y as u32)
    };
    let threshold = [-74.1625, 40.6995];
    for nm in [5.2, 7.0] {
        let point = destination(threshold, 27.0, nm * 1_852.0);
        let (x, y) = to_px(point);
        let px = image.get_pixel(x, y).0;
        assert!(px[0] < 90 && px[3] > 200, "inked course at {nm} NM: {px:?}");
    }
}

#[test]
fn non_finite_plate_inputs_are_refused() {
    let mut spec = kewr_22l();
    spec.true_heading_deg = f64::NAN;
    assert!(matches!(
        generate_procedural_approach_plate(&spec),
        Err(MapAdapterError::ChartRaster { .. })
    ));
}

#[test]
fn a_flightbag_chart_is_placed_north_up_over_its_bounds() {
    let chart = flightbag::GeorefChartRef {
        chart_id: "kewr-22l".to_owned(),
        facility: None,
        kind: flightbag::ChartKind::ApproachPlate,
        title: "ILS OR LOC RWY 22L".to_owned(),
        bounds: flightbag::ChartBounds {
            south_lat_deg: 40.5,
            west_lon_deg: -74.4,
            north_lat_deg: 40.8,
            east_lon_deg: -74.0,
        },
        asset_sha256_hex: "0".repeat(64),
    };
    let plate = ApproachPlateOverlay::from_flightbag_georef(&chart, Arc::from(vec![1_u8]));
    assert_eq!(
        plate.corners_lon_lat,
        [[-74.4, 40.8], [-74.0, 40.8], [-74.0, 40.5], [-74.4, 40.5]]
    );
    assert_eq!(plate.title, chart.title);
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn polar_nzsp_globe_frame_renders_non_empty_chart() {
    let Some(gpu) = try_host_gpu() else {
        return;
    };
    let (scenery, _, catalog_runtime) = sample_scenery_and_runtime(gpu.clone());
    drop(catalog_runtime);
    let charts = ChartAssetStore::new();
    let loader =
        HybridTileLoader::new(Some(Arc::new(scenery)), None).with_chart_store(charts.clone());
    let runtime = SharedMapRuntime::new(
        gpu,
        crate::CockpitDeviceTier::Desktop,
        wgpu::TextureFormat::Rgba8Unorm,
        Some(loader.into_shared_loader()),
    )
    .expect("runtime");
    let diagram = sim_scenery::OsmAerodromeDiagram::builtin_all_merged_geojson().expect("osm");
    let mut efb = EfbMapSurface::new(
        &runtime,
        charts,
        &SceneryEndpoints::catalog(0, 0),
        diagram,
        EfbMapConfig {
            compact_size: [128, 96],
            expanded_size: [256, 192],
            initial_center: [-89.9975, 139.2729],
        },
    )
    .expect("polar efb");
    let rwy = flightbag::RunwayEndSpec {
        designator: flightbag::RunwayDesignator::parse("02").expect("02"),
        threshold: world::GeodeticPosition::wgs84(-89.9975, 139.2729, 2_835.0),
        true_bearing_rad: 20.0_f64.to_radians(),
        length_m: 3_658.0,
        width_m: 61.0,
    };
    let polar_own = ChartOwnship {
        lat_deg: -89.9975,
        lon_deg: 139.2729,
        true_heading_deg: 20.0,
        ground_velocity_en_mps: [
            20.0_f64.to_radians().sin() * 55.0,
            20.0_f64.to_radians().cos() * 55.0,
        ],
    };
    let rwy_geom = sim_scenery::LandingRunwayGeometry::from_runway_end_spec(&rwy);
    let sym = ChartSymbologyInput {
        camera: efb.camera().spec(),
        viewport_px: [128, 96],
        ownship: Some(polar_own),
        runway: Some(&rwy_geom),
        traffic: &[],
    };
    for _ in 0..120 {
        assert!(efb.render(&sym).expect("polar frame").is_some());
        if !efb.needs_redraw() {
            break;
        }
        std::thread::yield_now();
    }
    let (lat, lon, _) = efb.camera_center_and_bearing();
    assert!((lat - (-89.9975)).abs() < 1e-6 && (lon - 139.2729).abs() < 1e-6);
    let rgba = efb.read_active_rgba().expect("polar rgba");
    let (pixels, rem) = rgba.as_chunks::<4>();
    assert_eq!(pixels.len(), 128 * 96);
    assert!(rem.is_empty());
    assert!(pixels.iter().all(|&[_, _, _, a]| a > 0));
    let bright = pixels
        .iter()
        .filter(|&&[r, g, b, _]| u16::from(r) + u16::from(g) + u16::from(b) > 240)
        .count();
    let dark = pixels
        .iter()
        .filter(|&&[r, g, b, _]| u16::from(r) + u16::from(g) + u16::from(b) < 150)
        .count();
    assert!(bright > 20 && dark > 200, "bright={bright}, dark={dark}");
}

mod allocations;
mod plan;
