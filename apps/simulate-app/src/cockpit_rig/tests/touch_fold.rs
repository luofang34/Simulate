//! Folding from a touch overlay, on a real GPU: the instrument column folds without taking
//! an open corner map with it, the map opens beside a folded column, and the overlay is
//! told where the corner map is only while it is drawn.

use sim_maplibre::{EfbDisplayMode, HostGpu};
use sim_render::{HeadlessGpu, VectorScenePass};
use sim_scenario::OperatorViewportMode;

use super::super::efb_chrome::{EfbPlacement, efb_map_rect};
use super::super::{CockpitRig, CockpitViewMode};
use crate::error::AppSimError;
use crate::flight::Flight;
use crate::map_rig::{AvionicsPreset, EfbCommand, ScenerySource};
use crate::sky_rig::SkyFrame;
use crate::world_surface::WorldSurfaceHandle;

#[test]
fn the_column_folds_alone_and_the_corner_map_reports_where_it_is_drawn() -> Result<(), AppSimError>
{
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
    let world = flight.world();
    rig.supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    rig.set_avionics_preset(AvionicsPreset::SingleMovingMap)?;
    let cam = flight.display_camera(OperatorViewportMode::Cockpit)?;
    let sky = SkyFrame::of_world(world);
    let _globe = rig.prepare_frame(&gpu.queue, size, &flight, &cam, &sky, &[])?;

    let corner = efb_map_rect(size, EfbDisplayMode::CompactCorner, EfbPlacement::default());
    let anchors = rig.touch_anchors(size, world);
    assert!(anchors.rail.is_none(), "a desktop window docks a row");
    let [left, bottom] = anchors.corner_map.expect("the corner map is drawn");
    assert!((left - corner.x).abs() < 1e-5);
    assert!((bottom - corner.y - corner.height).abs() < 1e-5);

    // The column folds with the map open: the map stays, and the HUD takes the column's place.
    assert_eq!(rig.view_mode(), CockpitViewMode::HudAndEfis);
    rig.set_view_mode(CockpitViewMode::for_head_down(false));
    let _globe = rig.prepare_frame(&gpu.queue, size, &flight, &cam, &sky, &[])?;
    assert_eq!(rig.view_mode(), CockpitViewMode::HudOnly);
    assert!(
        rig.efb_chrome(world).is_some(),
        "the map outlives the column"
    );
    assert!(rig.touch_anchors(size, world).corner_map.is_some());

    // The map folds, then opens again beside the folded column.
    rig.efb_command(EfbCommand::SetCollapsed(true))?;
    let _globe = rig.prepare_frame(&gpu.queue, size, &flight, &cam, &sky, &[])?;
    assert!(rig.touch_anchors(size, world).corner_map.is_none());
    rig.efb_command(EfbCommand::CycleMobileState)?;
    let _globe = rig.prepare_frame(&gpu.queue, size, &flight, &cam, &sky, &[])?;
    assert!(rig.efb_chrome(world).is_some(), "opened without the column");
    assert!(rig.touch_anchors(size, world).corner_map.is_some());

    // The tablet is not the corner map: nothing hangs on it.
    rig.efb_command(EfbCommand::CycleMobileState)?;
    let _globe = rig.prepare_frame(&gpu.queue, size, &flight, &cam, &sky, &[])?;
    let chrome = rig.efb_chrome(world).expect("the tablet shows");
    assert_eq!(chrome.mode, EfbDisplayMode::ExpandedEfb);
    assert!(rig.touch_anchors(size, world).corner_map.is_none());

    rig.set_view_mode(CockpitViewMode::for_head_down(true));
    assert_eq!(rig.view_mode(), CockpitViewMode::EfisOnly);
    gpu.check_device()?;
    Ok(())
}
