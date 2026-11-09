//! The window and the machine-vision sensor are separate cameras: every window view draws
//! on the full-resolution OTW slot, and the `800×600` sensor slot steps only on `30 Hz`
//! exposures at their exact truth state, whatever the window shows.

use sim_core::{ControlInputs, SensorExposureEvent};
use sim_maplibre::HostGpu;
use sim_render::{HeadlessGpu, VectorScenePass};
use sim_scenario::{OperatorViewportMode, SimTimeScale};

use crate::cockpit_rig::CockpitRig;
use crate::error::AppSimError;
use crate::flight::Flight;
use crate::map_rig::ScenerySource;
use crate::sky_rig::SkyFrame;

const CRUISE: ControlInputs = ControlInputs {
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0,
    throttle: 0.72,
};

fn frame(
    gpu: &HeadlessGpu,
    rig: &mut CockpitRig,
    flight: &mut Flight,
    mode: OperatorViewportMode,
    elapsed_s: f64,
    size: [u32; 2],
) -> Result<Vec<SensorExposureEvent>, AppSimError> {
    rig.supervisor_mut().set_viewport_mode(mode);
    flight.advance_by(elapsed_s, CRUISE, SimTimeScale::RealTime)?;
    let exposures = flight.drain_sensor_exposures();
    let camera = flight.display_camera(mode)?;
    let sky = SkyFrame::of_world(flight.world());
    let _need_globe = rig.prepare_frame(&gpu.queue, size, flight, &camera, &sky, &exposures)?;
    Ok(exposures)
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn sensor_aperture_window_is_full_resolution_otw_and_sensor_steps_on_exposures()
-> Result<(), AppSimError> {
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
    let size = [1280_u32, 720_u32];
    let hud_pass = pollster::block_on(VectorScenePass::new(&gpu.device))?;
    let surface = crate::world_surface::WorldSurfaceHandle::new();
    let mut rig = CockpitRig::new_with_gpu(host_gpu, size, scenery, hud_pass, surface.clone())?;
    if rig.maps().is_none() {
        return Ok(());
    }
    let mut flight = Flight::with_surface(surface.clone())?;
    let aperture = OperatorViewportMode::SensorAperture;

    // Shorter than one sensor period: the window draws, the sensor does not.
    let none = frame(&gpu, &mut rig, &mut flight, aperture, 0.01, size)?;
    assert!(none.is_empty());
    let maps = rig.maps().expect("maps");
    assert_eq!(maps.sensor_frame_count(), 0);
    assert!(maps.last_sensor_stamp().is_none());
    assert_eq!(
        maps.truth().otw_size(),
        size,
        "the window slot is full resolution"
    );
    let quads = rig.overlay_quads(flight.world());
    let shown = quads.first().expect("full-window scene").view;
    let otw = rig
        .maps()
        .and_then(|m| m.truth().otw_color_view())
        .expect("otw");
    assert!(
        std::ptr::eq(shown, otw),
        "the aperture shows the OTW slot, not the 4:3 sensor"
    );

    // Crossing a sensor instant steps the sensor at that exposure's exact truth.
    let one = frame(&gpu, &mut rig, &mut flight, aperture, 0.05, size)?;
    assert_eq!(one.len(), 1);
    let exposure = one.first().expect("exposure");
    let maps = rig.maps().expect("maps");
    assert_eq!(maps.sensor_frame_count(), 1);
    let stamp = maps.last_sensor_stamp().expect("sensor stamp");
    assert_eq!(stamp.instant, exposure.exposure_at);
    assert_ne!(
        exposure.exposure_at,
        flight.now(),
        "not the display instant"
    );
    assert_ne!(exposure.state, flight.display_vehicle_state());

    // The sensor keeps its rate while the operator looks out the cockpit or at the chart.
    for mode in [
        OperatorViewportMode::Cockpit,
        OperatorViewportMode::ChartMap,
    ] {
        let before = rig.maps().map_or(0, |m| m.sensor_frame_count());
        let exposures = frame(&gpu, &mut rig, &mut flight, mode, 0.1, size)?;
        assert!(!exposures.is_empty(), "{mode:?}");
        let maps = rig.maps().expect("maps");
        assert_eq!(maps.sensor_frame_count(), before + 1, "{mode:?}");
        let newest = exposures.last().expect("newest");
        assert_eq!(
            maps.last_sensor_stamp().map(|s| s.instant),
            Some(newest.exposure_at),
            "{mode:?}"
        );
    }
    gpu.check_device()?;
    Ok(())
}
