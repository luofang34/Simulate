//! A page's map on a real GPU: the window's inset and the page's canvas are the two views
//! of the one EFB map, both drawn each frame, and the page's commands reach the canvas's.

use sim_geo::BodyEllipsoid;
use sim_maplibre::{
    EfbBasemap, EfbDisplayMode, FixId, FlightPlan, HostGpu, LegState, RouteFix, RouteProgress,
};
use sim_render::{HeadlessGpu, VectorScenePass};
use sim_scenario::OperatorViewportMode;

use super::super::CockpitRig;
use super::super::efb_chrome::efb_map_rect;
use crate::error::AppSimError;
use crate::flight::Flight;
use crate::map_rig::{AvionicsPreset, EfbCommand, ScenerySource};
use crate::page_map::PageMapLink;
use crate::sky_rig::SkyFrame;
use crate::world_surface::WorldSurfaceHandle;

/// Three fixes a few kilometres around `[lat, lon]`.
fn plan_around([lat, lon]: [f64; 2]) -> FlightPlan {
    let fix = |id: u32, ident: &str, dlat: f64, dlon: f64| RouteFix {
        id: FixId(id),
        ident: ident.to_owned(),
        lat_deg: lat + dlat,
        lon_deg: lon + dlon,
        altitude_m: Some(600.0),
    };
    FlightPlan {
        origin: Some([lat, lon]),
        fixes: vec![
            fix(0, "AAA", 0.01, 0.0),
            fix(1, "BBB", 0.01, 0.015),
            fix(2, "CCC", -0.01, 0.015),
        ],
    }
}

/// A cockpit on a real GPU with the moving-map preset, its expanded view sized to a
/// page's 400 × 248 canvas, and a flight; `None` without a WebGPU-class adapter.
fn page_rig() -> Result<Option<(HeadlessGpu, CockpitRig, Flight)>, AppSimError> {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return Ok(None);
    };
    if gpu.backend == wgpu::Backend::Gl {
        return Ok(None);
    }
    let host_gpu = HostGpu {
        instance: gpu.instance.clone(),
        adapter: gpu.adapter.clone(),
        device: gpu.device.clone(),
        queue: gpu.queue.clone(),
    };
    let hud_pass = pollster::block_on(VectorScenePass::new(&gpu.device))?;
    let surface = WorldSurfaceHandle::new();
    let scenery = Some(ScenerySource::Catalog);
    let mut rig = CockpitRig::new_with_gpu(host_gpu, SIZE, scenery, hud_pass, surface.clone())?;
    let flight = Flight::with_surface(surface)?;
    rig.supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    rig.set_avionics_preset(AvionicsPreset::SingleMovingMap)?;
    rig.set_page_map_px(Some(PAGE_PX));
    Ok(Some((gpu, rig, flight)))
}

/// The window.
const SIZE: [u32; 2] = [1280, 720];
/// The page's canvas.
const PAGE_PX: [u32; 2] = [400, 248];

/// One frame: the page's input taken, every map drawn.
fn frame(
    gpu: &HeadlessGpu,
    rig: &mut CockpitRig,
    flight: &Flight,
    link: &PageMapLink,
) -> Result<(), AppSimError> {
    rig.take_page_map(link, true);
    let cam = flight.display_camera(OperatorViewportMode::Cockpit)?;
    let sky = SkyFrame::of_world(flight.world());
    rig.prepare_frame(&gpu.queue, SIZE, flight, &cam, &sky, &[])?;
    Ok(())
}

#[test]
fn the_inset_and_the_page_map_are_one_map_drawn_together() -> Result<(), AppSimError> {
    let Some((gpu, mut rig, flight)) = page_rig()? else {
        return Ok(());
    };
    let own = BodyEllipsoid::WGS84.fixed_to_geodetic(flight.display_vehicle_state().position_ecef);
    let link = PageMapLink::default();
    let plan = plan_around([own.lat_deg(), own.lon_deg()]);
    link.publish_route(&plan, RouteProgress::after_sequenced(Some((0, false))));
    frame(&gpu, &mut rig, &flight, &link)?;
    frame(&gpu, &mut rig, &flight, &link)?;

    let maps = rig.maps().expect("maps");
    let status = maps.efb_status().expect("status");
    assert_eq!(
        status.mode,
        EfbDisplayMode::ExpandedEfb,
        "the page commands its view"
    );
    assert!(maps.efb_view_of(EfbDisplayMode::ExpandedEfb).is_some());
    assert!(maps.efb_view_of(EfbDisplayMode::CompactCorner).is_some());
    let chrome = rig.efb_chrome(flight.world()).expect("the inset shows");
    assert_eq!(chrome.mode, EfbDisplayMode::CompactCorner);
    assert!(
        chrome.buttons.is_empty(),
        "the page's buttons command the map"
    );
    assert!(rig.page_commands_efb());
    let corner = efb_map_rect(SIZE, EfbDisplayMode::CompactCorner);
    let quads = rig.overlay_quads(flight.world());
    assert_eq!(
        quads.last().map(|q| q.rect),
        Some(corner),
        "the inset, not the tablet"
    );

    let view = rig.maps().expect("maps").page_map_view();
    let states: Vec<(&str, LegState)> = view
        .labels
        .iter()
        .map(|label| (label.ident.as_str(), label.state))
        .collect();
    assert_eq!(
        states,
        [
            ("AAA", LegState::Flown),
            ("BBB", LegState::Active),
            ("CCC", LegState::Future)
        ]
    );
    for label in &view.labels {
        let [x, y] = label.at_px;
        let [w, h] = PAGE_PX.map(|v| v as f32);
        assert!((0.0..w).contains(&x) && (0.0..h).contains(&y), "{label:?}");
    }
    gpu.check_device()?;
    Ok(())
}

#[test]
fn a_tap_on_the_page_map_picks_a_fix_and_the_basemap_switches() -> Result<(), AppSimError> {
    let Some((gpu, mut rig, flight)) = page_rig()? else {
        return Ok(());
    };
    let own = BodyEllipsoid::WGS84.fixed_to_geodetic(flight.display_vehicle_state().position_ecef);
    let link = PageMapLink::default();
    let plan = plan_around([own.lat_deg(), own.lon_deg()]);
    link.publish_route(&plan, RouteProgress::after_sequenced(None));
    frame(&gpu, &mut rig, &flight, &link)?;
    let view = rig.maps().expect("maps").page_map_view();
    let bbb = view.labels.get(1).map(|label| label.at_px).expect("BBB");
    link.command(EfbCommand::Pick([bbb[0] + 4.0, bbb[1] - 3.0]));
    link.command(EfbCommand::SetBasemap(EfbBasemap::Satellite));
    frame(&gpu, &mut rig, &flight, &link)?;
    let view = rig.maps().expect("maps").page_map_view();
    let picked = view.picked.expect("a fix picked");
    assert_eq!(
        (picked.ident.as_str(), picked.altitude_m, picked.state),
        ("BBB", Some(600.0), LegState::Future)
    );
    assert!(view.status.is_some_and(|s| s.satellite));
    link.command(EfbCommand::Pick([-50.0, -50.0]));
    frame(&gpu, &mut rig, &flight, &link)?;
    assert!(rig.maps().expect("maps").page_map_view().picked.is_none());
    gpu.check_device()?;
    Ok(())
}
