//! The EFB moving map: compact and expanded views over one tile world, zoom, pan and
//! recentre, and a georeferenced approach plate.

use std::{f64::consts::PI, sync::Arc};

use super::{sample_scenery_and_runtime, try_host_gpu};
use crate::{
    ChartAssetStore, ChartCameraSpec, ChartOwnship, ChartSymbologyInput, EfbDisplayMode,
    EfbFollowMode, EfbLayerFlags, EfbMapConfig, EfbMapSurface, HybridTileLoader,
    ProceduralPlateSpec, SceneryEndpoints, SharedMapRuntime, generate_procedural_approach_plate,
    geodesy::destination,
};

const COMPACT: [u32; 2] = [96, 72];
const EXPANDED: [u32; 2] = [256, 192];
const THRESHOLD: [f64; 2] = [-74.1625, 40.6995];
const COURSE_DEG: f64 = 207.0;
const NM: f64 = 1_852.0;

fn efb() -> Option<EfbMapSurface> {
    let gpu = try_host_gpu()?;
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
    let config = EfbMapConfig {
        compact_size: COMPACT,
        expanded_size: EXPANDED,
        initial_center: [THRESHOLD[1], THRESHOLD[0]],
    };
    Some(
        EfbMapSurface::new(
            &runtime,
            charts,
            &SceneryEndpoints::catalog(0, 0),
            diagram,
            config,
        )
        .expect("efb"),
    )
}

fn own(lon_lat: [f64; 2]) -> ChartOwnship {
    ChartOwnship {
        lat_deg: lon_lat[1],
        lon_deg: lon_lat[0],
        true_heading_deg: COURSE_DEG,
        ground_velocity_en_mps: [0.0, 0.0],
    }
}

fn input(ownship: Option<ChartOwnship>) -> ChartSymbologyInput<'static> {
    ChartSymbologyInput {
        camera: ChartCameraSpec {
            center_lat_deg: 0.0,
            center_lon_deg: 0.0,
            zoom: 1.0,
            bearing_deg: 0.0,
            pitch_deg: 0.0,
            surveillance_revision: 0,
            notices_revision: 0,
            symbology_revision: 0,
        },
        viewport_px: [1, 1],
        ownship,
        runway: None,
        traffic: &[],
    }
}

/// Draws until nothing is loading; returns the shown view's last frame.
fn settle(efb: &mut EfbMapSurface, ownship: Option<ChartOwnship>) -> Vec<u8> {
    for _ in 0..400 {
        assert!(efb.render(&input(ownship)).expect("frame").is_some());
        if !efb.needs_redraw() {
            break;
        }
        std::thread::yield_now();
    }
    efb.read_active_rgba().expect("rgba")
}

#[test]
fn compact_and_expanded_views_switch_without_reloading_tiles() {
    let Some(mut efb) = efb() else {
        return;
    };
    let ownship = Some(own(THRESHOLD));
    assert_eq!(efb.display_mode(), EfbDisplayMode::CompactCorner);
    let compact = settle(&mut efb, ownship);
    assert_eq!(compact.len(), (COMPACT[0] * COMPACT[1] * 4) as usize);
    assert_eq!(efb.toggle_expanded(), EfbDisplayMode::ExpandedEfb);
    let expanded = settle(&mut efb, ownship);
    assert_eq!(expanded.len(), (EXPANDED[0] * EXPANDED[1] * 4) as usize);
    assert_eq!(efb.toggle_expanded(), EfbDisplayMode::CompactCorner);
    assert!(efb.render(&input(ownship)).expect("compact").is_some());
    efb.set_display_mode(EfbDisplayMode::ExpandedEfb);
    assert!(efb.render(&input(ownship)).expect("expanded").is_some());
    // Both views share the tile world: the first frame back is already the settled one.
    assert_eq!(
        efb.read_active_rgba().expect("rgba"),
        expanded,
        "first frame back is complete"
    );
    efb.resize_expanded([200, 150]).expect("resize");
    assert_eq!(efb.active_size(), [200, 150]);
}

#[test]
fn zoom_pan_and_recentre_move_the_camera() {
    let Some(mut efb) = efb() else {
        return;
    };
    let start = destination(THRESHOLD, COURSE_DEG + 180.0, 3.0 * NM);
    let _first = settle(&mut efb, Some(own(start)));
    let zoom = efb.zoom();
    efb.zoom_by(1.5);
    assert!((efb.zoom() - zoom - 1.5).abs() < 1e-9);
    efb.set_follow_mode(EfbFollowMode::FollowNorthUp);
    let _north_up = settle(&mut efb, Some(own(start)));
    let (lat, lon, bearing) = efb.camera_center_and_bearing();
    assert_eq!((lat, lon, bearing), (start[1], start[0], 0.0));
    efb.pan_by_pixels(40.0, 0.0);
    assert_eq!(efb.follow_mode(), EfbFollowMode::FreePan);
    let panned = settle(&mut efb, Some(own(THRESHOLD)));
    let (_, panned_lon, _) = efb.camera_center_and_bearing();
    let metres_per_px = 40_075_016.7 * lat.to_radians().cos() / (512.0 * 2_f64.powf(efb.zoom()));
    let east_m = (panned_lon - lon).to_radians() * 6_378_137.0 * lat.to_radians().cos();
    assert!(
        (east_m / metres_per_px - 40.0).abs() < 0.5,
        "{} px",
        east_m / metres_per_px
    );
    assert!(!panned.is_empty());
    efb.recenter_ownship();
    assert_eq!(efb.follow_mode(), EfbFollowMode::FollowNorthUp);
    let _recentred = settle(&mut efb, Some(own(THRESHOLD)));
    let (lat, lon, _) = efb.camera_center_and_bearing();
    assert_eq!((lat, lon), (THRESHOLD[1], THRESHOLD[0]));
}

/// Screen pixel of `point` in a north-up view of `center` at `zoom`.
fn screen(point: [f64; 2], center: [f64; 2], zoom: f64) -> (usize, usize) {
    let world = 512.0 * 2_f64.powf(zoom);
    let mercator = |[lon, lat]: [f64; 2]| {
        let y = 0.5 - (PI / 4.0 + lat.to_radians() / 2.0).tan().ln() / (2.0 * PI);
        [(lon + 180.0) / 360.0 * world, y * world]
    };
    let ([px, py], [cx, cy]) = (mercator(point), mercator(center));
    let x = px - cx + f64::from(EXPANDED[0]) / 2.0;
    let y = py - cy + f64::from(EXPANDED[1]) / 2.0;
    (x as usize, y as usize)
}

fn luminance(rgba: &[u8], (x, y): (usize, usize)) -> u32 {
    let at = (y * EXPANDED[0] as usize + x) * 4;
    rgba[at..at + 3].iter().map(|c| u32::from(*c)).sum()
}

#[test]
fn an_approach_plate_drapes_its_course_where_it_lies() {
    let Some(mut efb) = efb() else {
        return;
    };
    efb.set_display_mode(EfbDisplayMode::ExpandedEfb);
    efb.set_follow_mode(EfbFollowMode::FollowNorthUp);
    efb.zoom_by(9.5 - efb.zoom());
    let center = destination(THRESHOLD, COURSE_DEG + 180.0, 3.0 * NM);
    let without = settle(&mut efb, Some(own(center)));
    let plate = generate_procedural_approach_plate(&ProceduralPlateSpec {
        airport_code: "KEWR".to_owned(),
        airport_name: "Newark".to_owned(),
        runway_designator: "22L".to_owned(),
        threshold_lat_deg: THRESHOLD[1],
        threshold_lon_deg: THRESHOLD[0],
        true_heading_deg: COURSE_DEG,
        elevation_ft: 18.0,
        ils_freq_khz: 108_700,
        glidepath_deg: 3.0,
    })
    .expect("plate");
    efb.set_approach_plate(Some(plate)).expect("set plate");
    assert_eq!(efb.plate_opacity(), 0.78);
    let with = settle(&mut efb, Some(own(center)));
    let faf = screen(
        destination(THRESHOLD, COURSE_DEG + 180.0, 5.2 * NM),
        center,
        9.5,
    );
    let sides = [COURSE_DEG + 90.0, COURSE_DEG - 90.0].map(|side| {
        let abeam = destination(
            destination(THRESHOLD, COURSE_DEG + 180.0, 5.2 * NM),
            side,
            1.5 * NM,
        );
        screen(abeam, center, 9.5)
    });
    for side in sides {
        assert!(
            luminance(&with, side) > luminance(&without, side) + 150,
            "the ivory plate brightens the chart off the course at {side:?}"
        );
        assert!(
            luminance(&with, side) > luminance(&with, faf) + 150,
            "the course at the FAF {faf:?} is inked darker than abeam {side:?}"
        );
    }
    efb.set_layers(EfbLayerFlags {
        approach_plate: false,
        ..EfbLayerFlags::default()
    })
    .expect("hide plate");
    let hidden = settle(&mut efb, Some(own(center)));
    assert!(luminance(&hidden, sides[0]).abs_diff(luminance(&without, sides[0])) < 12);
    efb.set_layers(EfbLayerFlags::default())
        .expect("show plate");
    efb.set_plate_opacity(0.0).expect("transparent");
    let transparent = settle(&mut efb, Some(own(center)));
    assert!(luminance(&transparent, sides[1]).abs_diff(luminance(&without, sides[1])) < 12);
    efb.set_approach_plate(None).expect("clear");
    assert!(efb.approach_plate().is_none());
}
