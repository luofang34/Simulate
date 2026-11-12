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
        ..RouteProgress::default()
    };
    assert_eq!(states(&plan, toward(None)), [Future, Future, Future]);
    assert_eq!(states(&plan, toward(Some(0))), [Active, Future, Future]);
    assert_eq!(states(&plan, toward(Some(2))), [Flown, Flown, Active]);
    let complete = RouteProgress {
        to_fix: Some(2),
        complete: true,
        hold: None,
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
fn tracks_and_marks_carry_their_colours_and_outlines_are_lines() {
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
        outlines: vec![vec![[40.0, -74.0], [40.0, -73.0]], vec![[41.0, -73.0]]],
    };
    let document = tracks.geojson();
    let features = document["features"].as_array().expect("features");
    assert_eq!(features.len(), 2, "a one-point track draws no line");
    assert_eq!(features[0]["properties"]["colour"], "#ffe066");
    assert_eq!(features[1]["properties"]["filled"], false);
    let outlines = tracks.outlines_geojson();
    let lines = outlines["features"].as_array().expect("features");
    assert_eq!(lines.len(), 1, "a one-point outline draws nothing");
    assert_eq!(
        lines[0]["geometry"]["coordinates"][1][0].as_f64(),
        Some(-73.0)
    );
}

/// An EFB map over the sample catalog.
pub(super) fn sample_efb() -> Option<EfbMapSurface> {
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
            compact_size: [320, 240],
            expanded_size: [480, 360],
            ..EfbMapConfig::default()
        },
    )
    .expect("efb");
    Some(efb)
}

/// Whether a magenta pixel lies within 3 pixels of `at` in a `width`-wide frame.
fn magenta_near(rgba: &[u8], width: u32, [x, y]: [f32; 2]) -> bool {
    let pixels = rgba.as_chunks::<4>().0;
    (-3_i64..=3).any(|dy| {
        (-3_i64..=3).any(|dx| {
            let (px, py) = (x as i64 + dx, y as i64 + dy);
            px >= 0
                && py >= 0
                && usize::try_from(py * i64::from(width) + px)
                    .ok()
                    .and_then(|i| pixels.get(i))
                    .is_some_and(|&[r, g, b, _]| r > 200 && g < 120 && b > 200)
        })
    })
}

/// The inset and the expanded view are views of one map: shown together, each draws the
/// plan's active leg where its own camera puts it, though one is track up at the inset's
/// zoom and the other north up two levels out, as a page's map starts.
#[cfg(not(target_arch = "wasm32"))]
#[test]
fn both_views_draw_the_same_plan_layers_with_their_own_cameras() {
    let Some(mut efb) = sample_efb() else {
        return;
    };
    let long = FlightPlan {
        origin: Some([40.69, -74.40]),
        fixes: vec![
            fix(10, "AAA", 40.69, -74.38),
            fix(11, "BBB", 40.69, -74.08),
            fix(12, "CCC", 40.79, -74.08),
        ],
    };
    efb.set_route(&long, RouteProgress::after_sequenced(Some((0, false))))
        .expect("route");
    efb.set_all_views_shown(true);
    efb.set_display_mode(EfbDisplayMode::ExpandedEfb);
    efb.resize_expanded([1664, 1031]).expect("resize");
    efb.set_pixel_ratio(2.0);
    efb.set_follow_mode(EfbFollowMode::FollowNorthUp);
    let own = crate::ChartOwnship {
        lat_deg: 40.69,
        lon_deg: -74.23,
        true_heading_deg: 90.0,
        ground_velocity_en_mps: [50.0, 0.0],
    };
    let symbology = ChartSymbologyInput {
        camera: efb.camera().spec(),
        viewport_px: [1, 1],
        ownship: Some(own),
        runway: None,
        traffic: &[],
    };
    settle(&mut efb, &symbology);
    efb.zoom_by(-2.0);
    grow_tracks(&mut efb, &symbology);
    settle(&mut efb, &symbology);
    let specs = [EfbDisplayMode::CompactCorner, EfbDisplayMode::ExpandedEfb]
        .map(|mode| efb.camera_spec(mode).expect("camera"));
    assert!(
        (specs[0].zoom - specs[1].zoom).abs() > 0.5,
        "each view keeps its own zoom"
    );
    assert_ne!(
        specs[0].bearing_deg, specs[1].bearing_deg,
        "track up and north up"
    );
    for mode in [EfbDisplayMode::CompactCorner, EfbDisplayMode::ExpandedEfb] {
        let positions = efb.fix_positions(mode);
        let (Some(a), Some(b)) = (positions[0], positions[1]) else {
            panic!("the active leg's ends project in {mode:?}: {positions:?}");
        };
        // A point of the active leg (the first fix to the second) clear of ownship.
        let on_leg = [a[0] + (b[0] - a[0]) * 0.42, a[1] + (b[1] - a[1]) * 0.42];
        let rgba = efb_view_rgba(&mut efb, mode, &symbology);
        let [width, _] = efb.size_of(mode);
        assert!(
            magenta_near(&rgba, width, on_leg),
            "the active leg passes {on_leg:?} in {mode:?}"
        );
        let picked = efb.pick_fix(mode, [b[0] + 3.0, b[1]], 12.0);
        assert_eq!(picked.map(|p| p.id), Some(FixId(11)));
        assert_eq!(efb.pick_fix(mode, [-500.0, -500.0], 12.0), None);
    }
}

/// Draws until the map has nothing left to load.
fn settle(efb: &mut EfbMapSurface, symbology: &ChartSymbologyInput<'_>) {
    for _ in 0..200 {
        efb.render(symbology).expect("frame");
        if !efb.needs_redraw() {
            break;
        }
        std::thread::yield_now();
    }
}

/// Hands over a growing track while the views draw, as a page does.
fn grow_tracks(efb: &mut EfbMapSurface, symbology: &ChartSymbologyInput<'_>) {
    for step in 0..40_u32 {
        let points = (0..step.wrapping_add(2))
            .map(|i| [40.69, -74.38 + 0.0025 * f64::from(i)])
            .collect();
        let tracks = EfbTracks {
            lines: vec![TrackLine {
                colour: [0xff, 0xe0, 0x66],
                width_px: 2.0,
                points,
            }],
            ..EfbTracks::default()
        };
        efb.set_tracks(tracks).expect("tracks");
        efb.render(symbology).expect("frame");
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
}

/// `mode`'s last frame, read back with it focused and every view drawn once more.
fn efb_view_rgba(
    efb: &mut EfbMapSurface,
    mode: EfbDisplayMode,
    symbology: &ChartSymbologyInput<'_>,
) -> Vec<u8> {
    let focus = efb.display_mode();
    efb.focus = mode;
    efb.render(symbology).expect("frame");
    let rgba = efb.read_active_rgba().expect("rgba");
    efb.focus = focus;
    rgba
}
