//! Navigation HUD slot gating (`set_view_slot_navigation_hud`), Retina hit-testing, and
//! crisp screen-space route line rendering (`terrain-structure: "overlay"`).

use maplibre::{hud::NavigationHudState, render::camera::EdgeInsets};
use sim_geo::Geodetic;
use sim_scenery::{TerrariumTile, project_to_tile};

use super::plan::sample_efb_on;
use crate::{
    ChartSymbologyInput, EfbDisplayMode, EfbFollowMode, FixId, FlightPlan, HudButton, RouteFix,
    RouteProgress,
};

const DEM_ZOOM: u8 = 10;

fn hud_button_physical_px(size_px: [u32; 2], pixel_ratio: f64, rel_logical_y: f64) -> [f32; 2] {
    let origin = NavigationHudState::panel_origin(
        f64::from(size_px[0]) / pixel_ratio,
        f64::from(size_px[1]) / pixel_ratio,
        &EdgeInsets::default(),
    );
    [
        ((origin.x + 19.0) * pixel_ratio) as f32,
        ((origin.y + rel_logical_y) * pixel_ratio) as f32,
    ]
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn navigation_hud_shows_on_expanded_efb_and_stays_hidden_on_compact_inset() {
    let Some(mut efb) = sample_efb_on(|_| {}, 0) else {
        return;
    };
    efb.set_pixel_ratio(2.0);
    efb.resize_expanded([640, 480]).expect("resize expanded");
    let expanded_size = efb.size_of(EfbDisplayMode::ExpandedEfb);

    let compass_px = hud_button_physical_px(expanded_size, 2.0, 20.0);
    let mode3d_px = hud_button_physical_px(expanded_size, 2.0, 56.0);
    let zoom_in_px = hud_button_physical_px(expanded_size, 2.0, 88.0);
    let zoom_out_px = hud_button_physical_px(expanded_size, 2.0, 118.0);
    let home_px = hud_button_physical_px(expanded_size, 2.0, 146.0);

    // Compact cockpit minimap inset hides the HUD and ignores HUD hits.
    let compact_size = efb.size_of(EfbDisplayMode::CompactCorner);
    let compact_probe = hud_button_physical_px(compact_size, 2.0, 88.0);
    assert_eq!(
        efb.hit_navigation_hud(EfbDisplayMode::CompactCorner, compact_probe),
        None
    );

    // Expanded EFB / page map hits each capsule button at Retina 2.0x scale.
    assert_eq!(
        efb.hit_navigation_hud(EfbDisplayMode::ExpandedEfb, compass_px),
        Some(HudButton::Compass)
    );
    assert_eq!(
        efb.hit_navigation_hud(EfbDisplayMode::ExpandedEfb, mode3d_px),
        Some(HudButton::Mode3D)
    );
    assert_eq!(
        efb.hit_navigation_hud(EfbDisplayMode::ExpandedEfb, zoom_in_px),
        Some(HudButton::ZoomIn)
    );
    assert_eq!(
        efb.hit_navigation_hud(EfbDisplayMode::ExpandedEfb, zoom_out_px),
        Some(HudButton::ZoomOut)
    );
    assert_eq!(
        efb.hit_navigation_hud(EfbDisplayMode::ExpandedEfb, home_px),
        Some(HudButton::Home)
    );

    efb.set_display_mode(EfbDisplayMode::ExpandedEfb);
    let start_zoom = efb.zoom();
    assert_eq!(
        efb.activate_navigation_hud(EfbDisplayMode::ExpandedEfb, zoom_in_px),
        Some(HudButton::ZoomIn)
    );
    assert!((efb.zoom() - (start_zoom + 1.0)).abs() < 1e-9);

    assert_eq!(
        efb.activate_navigation_hud(EfbDisplayMode::ExpandedEfb, mode3d_px),
        Some(HudButton::Mode3D)
    );
    assert!(
        (efb.camera_spec(EfbDisplayMode::ExpandedEfb)
            .expect("expanded camera")
            .pitch_deg
            - 45.0)
            .abs()
            < 1e-9
    );

    assert_eq!(
        efb.activate_navigation_hud(EfbDisplayMode::ExpandedEfb, compass_px),
        Some(HudButton::Compass)
    );
    assert_eq!(efb.follow_mode(), EfbFollowMode::FollowNorthUp);
}

#[test]
fn route_line_layers_declare_screen_space_terrain_overlay_metadata() {
    let style = super::super::style::efb_style(
        &crate::SceneryEndpoints::catalog(0, DEM_ZOOM),
        serde_json::json!({ "type": "FeatureCollection", "features": [] }),
    )
    .expect("efb style");
    for layer_id in [
        "efb-outline-line",
        "efb-route-casing",
        "efb-route-line",
        "efb-track-line",
        "efb-airspace-line",
    ] {
        let layer = style
            .layers
            .iter()
            .find(|layer| layer.id == layer_id)
            .unwrap_or_else(|| panic!("missing layer {layer_id}"));
        assert_eq!(
            layer
                .metadata
                .as_ref()
                .and_then(|meta| meta.get("maplibre-rs:terrain-structure"))
                .and_then(serde_json::Value::as_str),
            Some("overlay"),
            "{layer_id} must render as a crisp screen-space terrain overlay rather than baking into 512x512 drapes"
        );
    }
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn route_lines_render_as_crisp_retina_overlays_over_tilted_terrain() {
    let Some(mut efb) = sample_efb_on(
        |scenery| {
            let tile = project_to_tile(&Geodetic::from_degrees(40.70, -74.16, 0.0), DEM_ZOOM)
                .expect("tile")
                .tile;
            let heights = (0..16 * 16).map(|i| f64::from(i % 16) * 80.0).collect();
            let dem = TerrariumTile::from_heights(16, 16, heights).expect("dem");
            scenery.insert_elevation(tile, dem).expect("insert");
        },
        DEM_ZOOM,
    ) else {
        return;
    };
    efb.set_pixel_ratio(2.0);
    efb.resize_expanded([640, 400]).expect("resize retina");
    efb.set_display_mode(EfbDisplayMode::ExpandedEfb);
    efb.set_follow_mode(EfbFollowMode::FollowNorthUp);
    efb.zoom_by(12.5 - efb.zoom());
    efb.pitch_by(45.0);

    let own = crate::ChartOwnship {
        lat_deg: 40.70,
        lon_deg: -74.16,
        true_heading_deg: 45.0,
        ground_velocity_en_mps: [35.0, 35.0],
    };
    let symbology = ChartSymbologyInput {
        camera: efb.camera().spec(),
        viewport_px: [640, 400],
        ownship: Some(own),
        runway: None,
        traffic: &[],
    };
    let fix = |id: u32, lat_deg: f64, lon_deg: f64| RouteFix {
        id: FixId(id),
        ident: format!("W{id}"),
        lat_deg,
        lon_deg,
        altitude_m: None,
    };
    let plan = FlightPlan {
        origin: Some([40.69, -74.17]),
        fixes: vec![fix(0, 40.71, -74.15), fix(1, 40.73, -74.13)],
    };
    efb.set_route(&plan, RouteProgress::after_sequenced(None))
        .expect("set route");
    for _ in 0..160 {
        efb.render(&symbology).expect("render route");
        if !efb.needs_redraw() {
            break;
        }
        std::thread::yield_now();
    }
    let rgba = efb.read_active_rgba().expect("rgba");
    let magenta_px = rgba
        .as_chunks::<4>()
        .0
        .iter()
        .filter(|&&[r, g, b, _]| r > 200 && g < 110 && b > 200)
        .count();
    assert!(
        magenta_px > 40,
        "active magenta leg rendered at Retina resolution ({magenta_px} px)"
    );
}
