use sim_core::{ControlInputs, MagnetoSwitch};
use sim_maplibre::{HostGpu, ViewSlotMask};
use sim_render::{
    CompositorPass, CompositorTarget, HeadlessGpu, MAX_COMPOSITE_QUADS, OverlayQuad,
    VectorScenePass, ViewportRect,
};
use sim_scenario::{OperatorViewportMode, ScenarioProfile, ScenarioWorld, SimTimeScale};

use crate::cockpit_rig::CockpitRig;
use crate::error::AppSimError;
use crate::esc_menu::{
    EscMenu, HubCatalog, HubView, MenuInput, MenuOutcome, MenuOverlay, MenuStatus, OverlayContent,
};
use crate::flight::Flight;
use crate::keyboard::{CockpitAction, apply_aircraft_system_action};
use crate::map_rig::{
    AIRPORT_SURFACE_MAP, AvionicsPreset, MFD_CHART_MAP, PFD_SVS_MAP, ScenerySource,
};
use crate::sky_rig::SkyFrame;
use crate::supervisor_rig::SupervisorRig;
use crate::telemetry;
use crate::vehicles::FreeFlightVehicle;

mod liveries;
mod polar_nzsp;
mod sensor_aperture;

#[test]
fn free_flight_vehicles_cycle_step_and_accept_system_controls() -> Result<(), AppSimError> {
    let mut flight = Flight::new()?;
    assert_eq!(flight.free_vehicle(), Some(FreeFlightVehicle::JsbsimC172));
    assert!(flight.engine_status().is_some_and(|e| e.running));

    apply_aircraft_system_action(CockpitAction::FlapsDown, &mut flight);
    apply_aircraft_system_action(CockpitAction::CycleMagnetos, &mut flight);
    apply_aircraft_system_action(CockpitAction::MixtureLean, &mut flight);
    let sys = flight.system_controls();
    assert!(sys.is_some_and(|s| s.flaps_norm > 0.30));
    assert_eq!(sys.map(|s| s.magnetos), Some(MagnetoSwitch::Left));
    assert!(sys.is_some_and(|s| s.mixture_norm < 0.95));

    let cruise = ControlInputs {
        throttle: 0.72,
        ..ControlInputs::default()
    };
    for _ in 0..7 {
        flight.advance_by(0.05, cruise, SimTimeScale::RealTime)?;
        let next = flight.cycle_vehicle()?;
        if next == FreeFlightVehicle::AviateX500Direct
            || next == FreeFlightVehicle::AviateFixedWingDirect
        {
            assert_eq!(flight.aviate_armed(), Some(true));
        }
    }
    assert_eq!(flight.free_vehicle(), Some(FreeFlightVehicle::JsbsimC172));

    let before_pause = flight.now();
    flight.advance_by(0.25, cruise, SimTimeScale::Paused)?;
    assert_eq!(flight.now(), before_pause);

    flight.advance_by(0.10, cruise, SimTimeScale::Warp(4.0))?;
    assert!(flight.now() > before_pause);
    Ok(())
}

#[test]
fn supervisor_isolates_truth_traffic_from_operator_belief() -> Result<(), AppSimError> {
    let mut sup = SupervisorRig::new();
    let flight = Flight::new()?;
    sup.sync(
        flight.vehicle_state().position_ecef,
        flight.now(),
        flight.sim_epoch().start_utc()?,
        flight.profile(),
    );

    let truth = sup.truth_entities(ViewSlotMask::ALL);
    let belief = sup.belief_entities(ViewSlotMask::ALL);
    assert_eq!(
        truth.len(),
        2,
        "truth world holds near ADS-B + far RemoteID"
    );
    assert_eq!(belief.len(), 1, "belief world receives only in-range ADS-B");
    assert_eq!(
        belief.first().map(|e| e.kind),
        Some(sim_maplibre::DynamicEntityKind::FixedWing)
    );
    let belief_address = sup
        .supervisor()
        .operator_view()
        .belief_world
        .surveillance
        .tracks()
        .first()
        .map(|t| t.key.address);
    assert_eq!(belief_address, Some(0xA1_B2_C3));

    assert_eq!(sup.cycle_viewport_mode(), OperatorViewportMode::Chase);
    assert_eq!(
        sup.cycle_viewport_mode(),
        OperatorViewportMode::SensorAperture
    );
    assert_eq!(sup.cycle_viewport_mode(), OperatorViewportMode::ChartMap);
    assert_eq!(
        sup.cycle_viewport_mode(),
        OperatorViewportMode::VolumetricScene
    );
    assert_eq!(sup.cycle_viewport_mode(), OperatorViewportMode::Cockpit);
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn headless_composite_cockpit_renders_all_viewports_and_apollo_fallback() -> Result<(), AppSimError>
{
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return Ok(());
    };
    let scenery = (gpu.backend != wgpu::Backend::Gl).then_some(ScenerySource::Catalog);
    let host_gpu = HostGpu {
        instance: gpu.instance.clone(),
        adapter: gpu.adapter.clone(),
        device: gpu.device.clone(),
        queue: gpu.queue.clone(),
    };
    let size = [640_u32, 360_u32];
    let hud_pass = pollster::block_on(VectorScenePass::new(&gpu.device))?;
    let surface = crate::world_surface::WorldSurfaceHandle::new();
    let mut rig = CockpitRig::new_with_gpu(host_gpu, size, scenery, hud_pass, surface.clone())?;
    let compositor = pollster::block_on(CompositorPass::new(
        &gpu.device,
        wgpu::TextureFormat::Rgba8UnormSrgb,
    ))?;
    let target_view = create_offscreen_srgb_target(&gpu.device, size);

    let mut flight = Flight::with_surface(surface.clone())?;
    assert!(!rig.toggle_conformal_hud(), "the HUD starts shown");
    assert!(rig.toggle_conformal_hud());
    step_all_earth_viewports(&gpu, &compositor, &target_view, &mut rig, &mut flight, size)?;
    verify_vector_hud_and_menu_overlay(&gpu, &compositor, &target_view, &mut rig, &flight, size)?;

    if let Some(maps) = rig.maps() {
        assert_eq!(maps.preset(), AvionicsPreset::GlassCockpit);
        assert_eq!(maps.truth().otw_has_readback_buffer(), Some(false));
        for id in [PFD_SVS_MAP, MFD_CHART_MAP, AIRPORT_SURFACE_MAP] {
            let surface = maps.avionics().get(id).expect("glass cockpit map");
            assert_eq!(surface.has_readback_buffer(), Some(false));
        }
        assert!(
            maps.truth()
                .virtual_camera_size(maps.aux_virtual_camera_slot().expect("aux sensor slot"))
                .is_some()
        );
        verify_avionics_presets(&gpu, &mut rig, &flight, size)?;
    }
    assert!(rig.avionics().last_volume_scene_bytes() > 0);

    verify_airport_patterns_and_virtual_cameras(&gpu, &mut rig, &mut flight, size)?;
    verify_apollo_lunar_fallback(&gpu, &mut rig, &mut flight, size)?;
    assert!(gpu.device.poll(wgpu::PollType::wait_indefinitely()).is_ok());
    gpu.check_device()?;
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
fn create_offscreen_srgb_target(device: &wgpu::Device, size: [u32; 2]) -> wgpu::TextureView {
    let [width, height] = size;
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some("cockpit offscreen srgb"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        })
        .create_view(&wgpu::TextureViewDescriptor::default())
}

#[cfg(not(target_arch = "wasm32"))]
fn step_all_earth_viewports(
    gpu: &HeadlessGpu,
    compositor: &CompositorPass,
    target_view: &wgpu::TextureView,
    rig: &mut CockpitRig,
    flight: &mut Flight,
    size: [u32; 2],
) -> Result<(), AppSimError> {
    let enable_maps = rig.maps().is_some();
    for mode in [
        OperatorViewportMode::Cockpit,
        OperatorViewportMode::Chase,
        OperatorViewportMode::SensorAperture,
        OperatorViewportMode::ChartMap,
        OperatorViewportMode::VolumetricScene,
    ] {
        rig.supervisor_mut().set_viewport_mode(mode);
        flight.advance_by(
            0.05,
            ControlInputs {
                throttle: 0.72,
                ..ControlInputs::default()
            },
            SimTimeScale::RealTime,
        )?;
        let cam = flight.display_camera(mode)?;
        let world = flight.world();
        let exposures = flight.drain_sensor_exposures();
        let need_globe = rig.prepare_frame(
            &gpu.queue,
            size,
            flight,
            &cam,
            &SkyFrame::of_world(world),
            &exposures,
        )?;
        if enable_maps {
            assert!(
                !need_globe,
                "{mode:?} on Earth uses full-window MapLibre view"
            );
        }
        let quads = rig.overlay_quads(world);
        assert!(!quads.is_empty());

        let mut encoder = gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("cockpit test frame"),
            });
        compositor.encode(
            &gpu.device,
            &gpu.queue,
            &mut encoder,
            CompositorTarget {
                view: target_view,
                clear: Some(wgpu::Color::BLACK),
                size_px: size,
            },
            &quads,
        );
        gpu.queue.submit([encoder.finish()]);
    }
    Ok(())
}

/// Draws the vector HUD and the open `Esc` menu on the GPU and composites both over the scene.
#[cfg(not(target_arch = "wasm32"))]
fn verify_vector_hud_and_menu_overlay(
    gpu: &HeadlessGpu,
    compositor: &CompositorPass,
    target_view: &wgpu::TextureView,
    rig: &mut CockpitRig,
    flight: &Flight,
    size: [u32; 2],
) -> Result<(), AppSimError> {
    rig.supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    let cam = flight.display_camera(OperatorViewportMode::Cockpit)?;
    let sky = SkyFrame::of_world(flight.world());
    let _need_globe = rig.prepare_frame(&gpu.queue, size, flight, &cam, &sky, &[])?;
    assert!(rig.avionics().hud_view().is_some(), "the vector HUD drew");

    let mut menu = EscMenu::default();
    let mut overlay = MenuOverlay::new(pollster::block_on(VectorScenePass::new(&gpu.device))?);
    let catalog = HubCatalog::new(
        sim_scenery::AerodromeId::ALL
            .into_iter()
            .filter_map(|id| rig.supervisor().diagram(id)),
    );
    let status = MenuStatus {
        pacing: flight.pacing_stats(),
        history_s: flight.history_span_s(),
        epoch: rig.guidance_epoch(),
        aircraft: flight.preferred_vehicle().title().to_owned(),
        ..MenuStatus::default()
    };
    overlay.render(&gpu.device, &gpu.queue, OverlayContent::Hidden, size)?;
    assert!(overlay.view().is_none(), "a closed hub draws nothing");
    let (hangar, preview) = draw_hub_tabs(gpu, &mut overlay, &mut menu, &catalog, &status, size)?;
    let menu_view = overlay.view().expect("banner drew");

    let mut quads = rig.overlay_quads(flight.world());
    quads.push(OverlayQuad::vector_overlay(ViewportRect::FULL, menu_view));
    quads.push(OverlayQuad::panel(
        preview,
        hangar.view().expect("preview drew"),
        true,
        false,
    ));
    assert!(quads.len() <= MAX_COMPOSITE_QUADS, "{}", quads.len());
    let mut encoder = gpu
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("hud and menu test frame"),
        });
    compositor.encode(
        &gpu.device,
        &gpu.queue,
        &mut encoder,
        CompositorTarget {
            view: target_view,
            clear: Some(wgpu::Color::BLACK),
            size_px: size,
        },
        &quads,
    );
    gpu.queue.submit([encoder.finish()]);
    gpu.check_device()?;
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
fn verify_avionics_presets(
    gpu: &HeadlessGpu,
    rig: &mut CockpitRig,
    flight: &Flight,
    size: [u32; 2],
) -> Result<(), AppSimError> {
    rig.supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    let cam = flight.display_camera(OperatorViewportMode::Cockpit)?;
    let sun = SkyFrame::of_world(flight.world());
    for (preset, maps_expected, dock_panels) in [
        (AvionicsPreset::NoMapGauges, 0, 2),
        (AvionicsPreset::SingleMovingMap, 1, 3),
        (AvionicsPreset::GlassCockpit, 3, 4),
    ] {
        rig.set_avionics_preset(preset)?;
        let _need_globe = rig.prepare_frame(&gpu.queue, size, flight, &cam, &sun, &[])?;
        let maps = rig.maps().expect("maps enabled");
        assert_eq!(maps.avionics().len(), maps_expected, "{preset:?}");
        // Full-window scene + full-window HUD + the dock.
        assert_eq!(
            rig.overlay_quads(flight.world()).len(),
            2 + dock_panels,
            "{preset:?}"
        );
    }
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
fn verify_airport_patterns_and_virtual_cameras(
    gpu: &HeadlessGpu,
    rig: &mut CockpitRig,
    flight: &mut Flight,
    size: [u32; 2],
) -> Result<(), AppSimError> {
    for profile in [
        ScenarioProfile::Airport47NUncontrolledPattern,
        ScenarioProfile::AirportKewrToweredPattern,
    ] {
        flight.select(Some(profile))?;
        flight.advance_by(0.15, ControlInputs::default(), SimTimeScale::RealTime)?;
        assert!(flight.pattern_leg().is_some());
        for _ in 0..5 {
            let cam_spec = rig.supervisor_mut().cycle_virtual_camera(flight.profile());
            let cam = flight.display_camera(rig.supervisor().viewport_mode())?;
            let world = flight.world();
            let _need_globe = rig.prepare_frame(
                &gpu.queue,
                size,
                flight,
                &cam,
                &SkyFrame::of_world(world),
                &[],
            )?;
            let title_str = telemetry::title(
                flight,
                rig.supervisor().viewport_mode(),
                Some(cam_spec.title.as_ref()),
                Some(rig.supervisor().effective_aerodrome(Some(profile)).code()),
                SimTimeScale::RealTime,
                &flight.cockpit(),
            );
            assert!(title_str.contains(cam_spec.title.as_ref()));
        }
    }
    let next_aero = rig.supervisor_mut().cycle_aerodrome();
    if let Some(diag) = rig.supervisor().diagram(next_aero).cloned() {
        flight.respawn_at_diagram(&diag)?;
        assert!(flight.profile().is_none());
    }
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
fn verify_apollo_lunar_fallback(
    gpu: &HeadlessGpu,
    rig: &mut CockpitRig,
    flight: &mut Flight,
    size: [u32; 2],
) -> Result<(), AppSimError> {
    rig.supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    flight.select(Some(ScenarioProfile::Apollo11LunarLanding))?;
    assert_eq!(flight.world(), ScenarioWorld::LunarTranquility);
    flight.advance_by(0.10, ControlInputs::default(), SimTimeScale::RealTime)?;
    let cam = flight.display_camera(OperatorViewportMode::Cockpit)?;
    let need_globe = rig.prepare_frame(
        &gpu.queue,
        size,
        flight,
        &cam,
        &SkyFrame::of_world(ScenarioWorld::LunarTranquility),
        &[],
    )?;
    assert!(
        need_globe,
        "LunarTranquility uses 3D GlobeScene + Indicate Horizon panels"
    );
    let title_str = telemetry::title(
        flight,
        OperatorViewportMode::Cockpit,
        None,
        None,
        SimTimeScale::RealTime,
        &flight.cockpit(),
    );
    assert!(title_str.contains("PGNS"));
    Ok(())
}

/// Opens the hub, draws every tab, then the hangar tab with its live preview and the
/// takeover banner; returns the preview and where it composites.
#[cfg(not(target_arch = "wasm32"))]
fn draw_hub_tabs(
    gpu: &HeadlessGpu,
    overlay: &mut MenuOverlay,
    menu: &mut EscMenu,
    catalog: &HubCatalog,
    status: &MenuStatus,
    size: [u32; 2],
) -> Result<(crate::hangar::HangarRig, ViewportRect), AppSimError> {
    let view = HubView::build(menu, catalog, status, size);
    assert_eq!(
        menu.handle(MenuInput::Toggle, &view, catalog),
        MenuOutcome::Opened
    );
    let mut hangar = crate::hangar::HangarRig::new(&gpu.device)?;
    for tab in crate::esc_menu::HubTab::ALL {
        let view = HubView::build(menu, catalog, status, size);
        let _ = menu.handle(MenuInput::Tab(tab), &view, catalog);
        let view = HubView::build(menu, catalog, status, size);
        let content = OverlayContent::Hub {
            view: &view,
            highlights: menu.highlights(),
        };
        overlay.render(&gpu.device, &gpu.queue, content, size)?;
        assert!(overlay.view().is_some(), "{tab:?} drew");
    }
    let view = HubView::build(menu, catalog, status, size);
    let _ = menu.handle(
        MenuInput::Tab(crate::esc_menu::HubTab::Hangar),
        &view,
        catalog,
    );
    let view = HubView::build(menu, catalog, status, size);
    let preview = view.preview_viewport().expect("hangar preview card");
    let preview_px = view.preview_px().expect("preview size");
    let model = catalog.vehicles.first().expect("vehicle").model;
    hangar.render(
        &gpu.device,
        &gpu.queue,
        model,
        preview_px,
        sim_core::ControlInputs {
            roll: 0.5,
            ..ControlInputs::default()
        },
        0.1,
    )?;
    let content = OverlayContent::Hub {
        view: &view,
        highlights: menu.highlights(),
    };
    overlay.render(&gpu.device, &gpu.queue, content, size)?;
    overlay.render(
        &gpu.device,
        &gpu.queue,
        OverlayContent::Banner(crate::app::TAKEOVER_HINT),
        size,
    )?;
    Ok((hangar, preview))
}
