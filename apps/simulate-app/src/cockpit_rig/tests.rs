//! The cockpit EFB on a real GPU: the corner map draws in the top-right corner with its
//! plate hidden, expands into a tablet over the dock with the plate shown, and goes away
//! with the map-less preset.

use sim_maplibre::{EfbDisplayMode, HostGpu};
use sim_render::{HeadlessGpu, VectorScenePass};
use sim_scenario::OperatorViewportMode;

use super::CockpitRig;
use super::efb_chrome::efb_map_rect;
use crate::error::AppSimError;
use crate::flight::Flight;
use crate::map_rig::{AvionicsPreset, EfbCommand, GlobeSelection, ScenerySource};
use crate::sky_rig::SkyFrame;
use crate::world_surface::WorldSurfaceHandle;

#[test]
fn the_efb_draws_top_right_then_expands_over_the_dock() -> Result<(), AppSimError> {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return Ok(());
    };
    if gpu.backend == wgpu::Backend::Gl {
        return Ok(());
    }
    let host_gpu = HostGpu {
        instance: gpu.instance.clone(),
        adapter: gpu.adapter.clone(),
        device: gpu.device.clone(),
        queue: gpu.queue.clone(),
    };
    let size = [1280_u32, 720_u32];
    let hud_pass = pollster::block_on(VectorScenePass::new(&gpu.device))?;
    let surface = WorldSurfaceHandle::new();
    let scenery = Some(ScenerySource::Catalog);
    let mut rig = CockpitRig::new_with_gpu(host_gpu, size, scenery, hud_pass, surface.clone())?;
    let flight = Flight::with_surface(surface)?;
    rig.supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    rig.set_avionics_preset(AvionicsPreset::SingleMovingMap)?;
    let cam = flight.display_camera(OperatorViewportMode::Cockpit)?;
    let sky = SkyFrame::of_world(flight.world());
    let _globe = rig.prepare_frame(&gpu.queue, size, &flight, &cam, &sky, &[])?;

    let chrome = rig.efb_chrome(flight.world()).expect("the EFB drew");
    assert_eq!(chrome.mode, EfbDisplayMode::CompactCorner);
    let status = rig.maps().and_then(|m| m.efb_status()).expect("status");
    let code = rig
        .supervisor()
        .effective_aerodrome(flight.profile())
        .code();
    assert_eq!(status.airport, code);
    assert!(status.plate_title.is_some(), "{code} has a plate");
    assert!(!status.plate_shown, "the corner map hides the plate");
    let quads = rig.overlay_quads(flight.world());
    let corner = efb_map_rect(size, EfbDisplayMode::CompactCorner);
    assert_eq!(quads.last().map(|q| q.rect), Some(corner));
    assert_eq!(quads.len(), 5, "scene, HUD, PFD, HSI, EFB");

    rig.efb_command(EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb))?;
    let _globe = rig.prepare_frame(&gpu.queue, size, &flight, &cam, &sky, &[])?;
    let status = rig.maps().and_then(|m| m.efb_status()).expect("status");
    assert_eq!(status.mode, EfbDisplayMode::ExpandedEfb);
    assert!(status.plate_shown, "the tablet shows the plate");
    let tablet = efb_map_rect(size, EfbDisplayMode::ExpandedEfb);
    let quads = rig.overlay_quads(flight.world());
    assert_eq!(
        quads.last().map(|q| q.rect),
        Some(tablet),
        "drawn over the dock"
    );

    rig.set_avionics_preset(AvionicsPreset::NoMapGauges)?;
    let _globe = rig.prepare_frame(&gpu.queue, size, &flight, &cam, &sky, &[])?;
    assert!(rig.efb_chrome(flight.world()).is_none());
    assert_eq!(rig.overlay_quads(flight.world()).len(), 4);
    gpu.check_device()?;
    Ok(())
}

#[test]
fn the_hub_globe_centres_on_the_selection_and_picks_it_back() -> Result<(), AppSimError> {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return Ok(());
    };
    if gpu.backend == wgpu::Backend::Gl {
        return Ok(());
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
    let mut rig = CockpitRig::new_with_gpu(host_gpu, [640, 360], scenery, hud_pass, surface)?;
    let database = sim_scenery::AirportDatabase::builtin();
    let (kewr, record) = database.by_code("KEWR").expect("KEWR");
    let selection = GlobeSelection {
        lat_deg: record.lat_deg,
        lon_deg: record.lon_deg,
        airport: Some(kewr),
        runway: Some("22L".to_owned()),
    };
    let size = [480_u32, 270_u32];
    let maps = rig.maps_mut().expect("maps");
    assert!(maps.globe_view().is_none(), "built on first use");
    maps.render_globe_picker(size, Some(selection.clone()))?;
    assert!(maps.globe_view().is_some());
    assert_eq!(maps.pick_airport_at_pixel([240.0, 135.0]), Some(kewr));

    maps.drag_globe([240.0, 135.0], [300.0, 135.0]);
    maps.zoom_globe([240.0, 135.0], -1.0);
    maps.render_globe_picker(size, Some(selection))?;
    assert!(maps.globe_view().is_some());
    assert_ne!(
        maps.pick_airport_at_pixel([240.0, 135.0]),
        Some(kewr),
        "the drag moved KEWR off the centre"
    );
    gpu.check_device()?;
    Ok(())
}

mod page_map;
