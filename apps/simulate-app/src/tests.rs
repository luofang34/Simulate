use sim_core::{ControlInputs, MagnetoSwitch};
use sim_maplibre::{HostGpu, ViewSlotMask};
use sim_render::{CompositorPass, CompositorTarget, HeadlessGpu};
use sim_scenario::{OperatorViewportMode, ScenarioProfile, ScenarioWorld, SimTimeScale};

use crate::cockpit_rig::CockpitRig;
use crate::error::AppSimError;
use crate::flight::Flight;
use crate::keyboard::{CockpitAction, apply_aircraft_system_action};
use crate::supervisor_rig::SupervisorRig;
use crate::telemetry;
use crate::vehicles::FreeFlightVehicle;

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
    for _ in 0..5 {
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
    sup.sync(flight.vehicle_state().position_ecef, flight.now());

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
    let enable_maps = gpu.backend != wgpu::Backend::Gl;
    let host_gpu = HostGpu {
        instance: gpu.instance.clone(),
        adapter: gpu.adapter.clone(),
        device: gpu.device.clone(),
        queue: gpu.queue.clone(),
    };
    let size = [640_u32, 360_u32];
    let mut rig = CockpitRig::new_with_gpu(host_gpu, size, enable_maps)?;
    let compositor = pollster::block_on(CompositorPass::new(
        &gpu.device,
        wgpu::TextureFormat::Rgba8UnormSrgb,
    ))?;
    let target_view = create_offscreen_srgb_target(&gpu.device, size);

    let mut flight = Flight::new()?;
    assert!(rig.toggle_conformal_hud());
    step_all_earth_viewports(&gpu, &compositor, &target_view, &mut rig, &mut flight, size)?;

    if let Some(maps) = rig.maps() {
        assert_eq!(maps.truth().otw_has_readback_buffer(), Some(false));
        assert_eq!(maps.belief().svs_has_readback_buffer(), Some(false));
        assert_eq!(maps.belief().chart_has_readback_buffer(), Some(false));
    }
    assert!(rig.avionics().last_volume_scene_bytes() > 0);

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
        let cam = flight.camera(mode)?;
        let world = flight.world();
        let need_globe =
            rig.prepare_frame(&gpu.queue, size, flight, &cam, world.sun_direction_ecef())?;
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
    let cam = flight.camera(OperatorViewportMode::Cockpit)?;
    let need_globe = rig.prepare_frame(
        &gpu.queue,
        size,
        flight,
        &cam,
        ScenarioWorld::LunarTranquility.sun_direction_ecef(),
    )?;
    assert!(
        need_globe,
        "LunarTranquility uses 3D GlobeScene + Indicate Horizon panels"
    );
    let title_str = telemetry::title(
        flight,
        OperatorViewportMode::Cockpit,
        SimTimeScale::RealTime,
        &flight.cockpit(),
    );
    assert!(title_str.contains("PGNS"));
    Ok(())
}
