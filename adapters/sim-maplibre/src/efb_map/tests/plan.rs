use std::sync::Arc;

use super::super::*;
use crate::{
    ChartSymbologyInput, HybridTileLoader,
    tests::gpu::{sample_scenery_and_runtime, try_host_gpu},
};

fn fix(id: u32, ident: &str, lat_deg: f64, lon_deg: f64) -> RouteFix {
    RouteFix {
        id: FixId(id),
        ident: ident.to_owned(),
        lat_deg,
        lon_deg,
        altitude_m: Some(2_000.0),
    }
}

/// Three fixes east of the EFB's default centre, flown from it.
fn plan() -> FlightPlan {
    FlightPlan {
        origin: Some([40.69, -74.17]),
        fixes: vec![
            fix(10, "AAA", 40.69, -74.14),
            fix(11, "BBB", 40.71, -74.12),
            fix(12, "CCC", 40.73, -74.15),
        ],
    }
}

fn states(plan: &FlightPlan, progress: RouteProgress) -> Vec<LegState> {
    plan.legs(progress).iter().map(|leg| leg.state).collect()
}

#[test]
fn legs_before_the_flown_toward_fix_are_flown_and_after_it_future() {
    use LegState::{Active, Flown, Future};
    let plan = plan();
    let toward = |to_fix| RouteProgress {
        to_fix,
        complete: false,
    };
    assert_eq!(states(&plan, toward(None)), [Future, Future, Future]);
    assert_eq!(states(&plan, toward(Some(0))), [Active, Future, Future]);
    assert_eq!(states(&plan, toward(Some(2))), [Flown, Flown, Active]);
    let complete = RouteProgress {
        to_fix: Some(2),
        complete: true,
    };
    assert_eq!(states(&plan, complete), [Flown, Flown, Flown]);
}

#[test]
fn a_sequenced_fix_makes_the_leg_after_it_active() {
    // The host's `LegAdvanced { to_index: n }` sequences fix `n - 1`.
    assert_eq!(RouteProgress::after_sequenced(None).to_fix, Some(0));
    let after_first = RouteProgress::after_sequenced(Some((0, false)));
    assert_eq!(after_first.to_fix, Some(1));
    assert_eq!(leg_state(0, after_first), LegState::Flown);
    assert_eq!(leg_state(1, after_first), LegState::Active);
    let last = RouteProgress::after_sequenced(Some((2, true)));
    assert!(last.complete);
    assert_eq!(leg_state(2, last), LegState::Flown);
}

#[test]
fn the_first_leg_runs_from_the_origin_and_without_one_there_is_none() {
    let mut plan = plan();
    let legs = plan.legs(RouteProgress::default());
    assert_eq!(legs.len(), 3);
    assert_eq!(legs.first().map(|leg| leg.from_deg), Some([40.69, -74.17]));
    assert_eq!(legs.get(1).map(|leg| leg.from_deg), Some([40.69, -74.14]));
    plan.origin = None;
    let legs = plan.legs(RouteProgress::default());
    assert_eq!(legs.iter().map(|leg| leg.to).collect::<Vec<_>>(), [1, 2]);
}

#[test]
fn the_route_document_has_every_leg_and_fix_with_its_state() {
    let plan = plan();
    let progress = RouteProgress::after_sequenced(Some((0, false)));
    let document = plan.geojson(progress);
    let features = document["features"].as_array().expect("features");
    let of_kind = |kind: &str| -> Vec<(&str, &str)> {
        features
            .iter()
            .filter(|f| f["properties"]["kind"] == kind)
            .map(|f| {
                (
                    f["properties"]["state"].as_str().unwrap_or_default(),
                    f["geometry"]["type"].as_str().unwrap_or_default(),
                )
            })
            .collect()
    };
    assert_eq!(
        of_kind("leg"),
        [
            ("flown", "LineString"),
            ("active", "LineString"),
            ("future", "LineString")
        ]
    );
    assert_eq!(
        of_kind("fix"),
        [("flown", "Point"), ("active", "Point"), ("future", "Point")]
    );
    let first_leg = &features[0]["geometry"]["coordinates"];
    assert_eq!(
        first_leg[0][0].as_f64(),
        Some(-74.17),
        "GeoJSON is lon, lat"
    );
    assert_eq!(features[3]["properties"]["ident"], "AAA");
}

#[test]
fn tracks_and_marks_carry_their_colours_and_outlines_close() {
    let tracks = EfbTracks {
        lines: vec![
            TrackLine {
                colour: [0xff, 0xe0, 0x66],
                width_px: 2.0,
                points: vec![[40.0, -74.0], [40.1, -74.0]],
            },
            TrackLine {
                colour: [1, 2, 3],
                width_px: 1.5,
                points: vec![[40.0, -74.0]],
            },
        ],
        rings: Vec::new(),
        marks: vec![PointMark {
            at_deg: [40.0, -74.0],
            colour: [0x39, 0x87, 0xe5],
            filled: false,
        }],
        outlines: vec![vec![[40.0, -74.0], [40.0, -73.0], [41.0, -73.0]]],
    };
    let document = tracks.geojson();
    let features = document["features"].as_array().expect("features");
    assert_eq!(features.len(), 2, "a one-point track draws no line");
    assert_eq!(features[0]["properties"]["colour"], "#ffe066");
    assert_eq!(features[1]["properties"]["filled"], false);
    let outline = &tracks.outlines_geojson()["features"][0]["geometry"]["coordinates"];
    assert_eq!(outline.as_array().map(Vec::len), Some(4));
    assert_eq!(outline[0], outline[3]);
}

/// An EFB map over the sample catalog.
fn sample_efb() -> Option<EfbMapSurface> {
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
    let efb = EfbMapSurface::new(
        &runtime,
        charts,
        &SceneryEndpoints::catalog(0, 0),
        serde_json::json!({ "type": "FeatureCollection", "features": [] }),
        EfbMapConfig {
            compact_size: [160, 120],
            expanded_size: [320, 240],
            ..EfbMapConfig::default()
        },
    )
    .expect("efb");
    Some(efb)
}

fn magenta_pixels(rgba: &[u8]) -> usize {
    rgba.as_chunks::<4>()
        .0
        .iter()
        .filter(|&&[r, g, b, _]| r > 200 && g < 120 && b > 200)
        .count()
}

/// The inset and the expanded view are views of one map: shown together, both draw the
/// plan's active leg, and each places the fixes by its own camera.
#[cfg(not(target_arch = "wasm32"))]
#[test]
fn both_views_draw_the_same_plan_layers_with_their_own_cameras() {
    let Some(mut efb) = sample_efb() else {
        return;
    };
    efb.set_route(&plan(), RouteProgress::after_sequenced(Some((0, false))))
        .expect("route");
    efb.set_all_views_shown(true);
    efb.set_display_mode(EfbDisplayMode::ExpandedEfb);
    efb.set_follow_mode(EfbFollowMode::FollowNorthUp);
    efb.zoom_by(-1.0);
    let symbology = ChartSymbologyInput {
        camera: efb.camera().spec(),
        viewport_px: [1, 1],
        ownship: None,
        runway: None,
        traffic: &[],
    };
    let mut drawn = [0, 0];
    for mode in [EfbDisplayMode::ExpandedEfb, EfbDisplayMode::CompactCorner] {
        efb.set_display_mode(mode);
        for _ in 0..120 {
            efb.render(&symbology).expect("frame");
            if !efb.needs_redraw() {
                break;
            }
            std::thread::yield_now();
        }
        let index = super::super::mode_index(mode);
        drawn[index] = magenta_pixels(&efb.read_active_rgba().expect("rgba"));
    }
    assert!(drawn.iter().all(|&n| n > 20), "magenta pixels {drawn:?}");
    let zooms = [EfbDisplayMode::CompactCorner, EfbDisplayMode::ExpandedEfb]
        .map(|mode| efb.camera_spec(mode).map(|spec| spec.zoom));
    assert_ne!(zooms[0], zooms[1], "each view keeps its own zoom");
    for mode in [EfbDisplayMode::CompactCorner, EfbDisplayMode::ExpandedEfb] {
        let positions = efb.fix_positions(mode);
        assert_eq!(positions.len(), 3);
        let first = positions[0].expect("first fix on screen");
        let picked = efb.pick_fix(mode, [first[0] + 3.0, first[1]], 12.0);
        assert_eq!(picked.map(|p| p.id), Some(FixId(10)));
        assert_eq!(efb.pick_fix(mode, [-500.0, -500.0], 12.0), None);
    }
}
