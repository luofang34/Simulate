use std::f64::consts::TAU;

use sim_core::AirframeArticulation;

use super::*;

const SIZE: [u32; 2] = [256, 160];

fn try_device() -> Option<(wgpu::Device, wgpu::Queue)> {
    pollster::block_on(async {
        let instance = wgpu::Instance::default();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions::default())
            .await
            .ok()?;
        adapter
            .request_device(&wgpu::DeviceDescriptor::default())
            .await
            .ok()
    })
}

fn read_rgba(device: &wgpu::Device, queue: &wgpu::Queue, texture: &wgpu::Texture) -> Vec<u8> {
    let size = texture.size();
    let row = size.width * 4;
    let padded =
        row.div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT) * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("hangar test readback"),
        size: u64::from(padded * size.height),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&Default::default());
    encoder.copy_texture_to_buffer(
        texture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &staging,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded),
                rows_per_image: None,
            },
        },
        size,
    );
    queue.submit([encoder.finish()]);
    staging.map_async(wgpu::MapMode::Read, .., |r| r.unwrap());
    device.poll(wgpu::PollType::wait_indefinitely()).unwrap();
    let raw = staging.get_mapped_range(..).unwrap().to_vec();
    raw.chunks(padded as usize)
        .take(size.height as usize)
        .flat_map(|r| r[..row as usize].to_vec())
        .collect()
}

fn fixed_orbit() -> HangarOrbit {
    HangarOrbit {
        auto_rotate: false,
        ..HangarOrbit::default()
    }
}

fn render(
    pass: &mut HangarPreviewPass,
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    articulation: &AirframeArticulation,
    orbit: &HangarOrbit,
) -> Vec<u8> {
    let mut encoder = device.create_command_encoder(&Default::default());
    pass.render(device, queue, &mut encoder, articulation, orbit)
        .unwrap();
    queue.submit([encoder.finish()]);
    read_rgba(device, queue, pass.texture())
}

/// Pixels inside the central half of the frame whose colour differs noticeably.
fn centre_differences(a: &[u8], b: &[u8]) -> (usize, usize) {
    let [w, h] = SIZE.map(|v| v as usize);
    let mut differing = 0;
    let mut total = 0;
    for y in h / 4..h * 3 / 4 {
        for x in w / 4..w * 3 / 4 {
            let i = (y * w + x) * 4;
            total += 1;
            if (0..3).any(|c| a[i + c].abs_diff(b[i + c]) > 12) {
                differing += 1;
            }
        }
    }
    (differing, total)
}

fn builtins() -> Vec<(&'static str, UsdAircraftAsset)> {
    vec![
        ("c172", UsdAircraftAsset::embedded_c172().unwrap()),
        ("x500", UsdAircraftAsset::x500_quadrotor().unwrap()),
        ("fixed_wing", UsdAircraftAsset::fixed_wing_uav().unwrap()),
        ("lander", UsdAircraftAsset::lunar_lander().unwrap()),
    ]
}

#[test]
fn every_builtin_draws_over_the_backdrop_without_validation_errors() {
    let Some((device, queue)) = try_device() else {
        return;
    };
    let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
    let mut pass = HangarPreviewPass::new(&device, SIZE).unwrap();
    let art = AirframeArticulation::default();
    let backdrop_only = render(&mut pass, &device, &queue, &art, &fixed_orbit());
    assert!(
        backdrop_only.chunks(4).all(|p| p[3] == 255),
        "output must be opaque"
    );
    for (name, asset) in builtins() {
        pass.set_asset(&device, &queue, &asset).unwrap();
        let with_vehicle = render(&mut pass, &device, &queue, &art, &fixed_orbit());
        // Same framing, meshes withheld: only the studio backdrop remains.
        let meshes = std::mem::take(&mut pass.scene.as_mut().unwrap().meshes);
        let backdrop = render(&mut pass, &device, &queue, &art, &fixed_orbit());
        pass.scene.as_mut().unwrap().meshes = meshes;
        let (differing, total) = centre_differences(&with_vehicle, &backdrop);
        assert!(
            differing * 20 > total,
            "{name}: {differing}/{total} centre pixels changed"
        );
        assert!(
            with_vehicle.chunks(4).all(|p| p[3] == 255),
            "{name} must stay opaque"
        );
    }
    let error = pollster::block_on(scope.pop());
    assert!(error.is_none(), "validation error: {error:?}");
}

#[test]
fn articulation_moves_rendered_pixels() {
    let Some((device, queue)) = try_device() else {
        return;
    };
    let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
    let mut pass = HangarPreviewPass::new(&device, SIZE).unwrap();
    let rest = AirframeArticulation::default();
    let deflected = AirframeArticulation {
        left_aileron_rad: 0.35,
        right_aileron_rad: -0.35,
        elevator_rad: 0.4,
        rudder_rad: 0.4,
        propeller_phase_rad: 1.3,
        ..rest
    };
    let spun = AirframeArticulation {
        propeller_phase_rad: 0.8,
        ..rest
    };
    let orbit = HangarOrbit {
        elevation_rad: 0.6,
        ..fixed_orbit()
    };
    for (name, asset, posed) in [
        (
            "c172",
            UsdAircraftAsset::embedded_c172().unwrap(),
            deflected,
        ),
        (
            "fixed_wing",
            UsdAircraftAsset::fixed_wing_uav().unwrap(),
            deflected,
        ),
        (
            "x500 rotors",
            UsdAircraftAsset::x500_quadrotor().unwrap(),
            spun,
        ),
    ] {
        pass.set_asset(&device, &queue, &asset).unwrap();
        let a = render(&mut pass, &device, &queue, &rest, &orbit);
        let b = render(&mut pass, &device, &queue, &posed, &orbit);
        let (differing, _) = centre_differences(&a, &b);
        assert!(differing > 20, "{name}: only {differing} pixels moved");
    }
    let error = pollster::block_on(scope.pop());
    assert!(error.is_none(), "validation error: {error:?}");
}

#[test]
fn set_asset_is_cached_and_targets_resize() {
    let Some((device, queue)) = try_device() else {
        return;
    };
    let mut pass = HangarPreviewPass::new(&device, SIZE).unwrap();
    let x500 = UsdAircraftAsset::x500_quadrotor().unwrap();
    pass.set_asset(&device, &queue, &x500).unwrap();
    let sentinel = orbit::Bounds {
        min: glam::DVec3::splat(-1.0),
        max: glam::DVec3::splat(1.0),
    };
    pass.scene.as_mut().unwrap().bounds = sentinel;
    pass.set_asset(&device, &queue, &x500.clone()).unwrap();
    assert_eq!(pass.scene.as_ref().unwrap().bounds, sentinel, "re-uploaded");
    pass.set_asset(&device, &queue, &UsdAircraftAsset::lunar_lander().unwrap())
        .unwrap();
    assert_ne!(pass.scene.as_ref().unwrap().bounds, sentinel, "stale asset");

    pass.ensure_size(&device, [300, 200]).unwrap();
    assert_eq!(pass.size(), [300, 200]);
    let extent = pass.texture().size();
    assert_eq!((extent.width, extent.height), (300, 200));
    assert_eq!(pass.texture().format(), wgpu::TextureFormat::Rgba8UnormSrgb);
    assert!(pass.ensure_size(&device, [0, 10]).is_err());
    assert_eq!(pass.size(), [300, 200]);
}

#[test]
fn orbit_turns_wraps_and_clamps() {
    let mut orbit = HangarOrbit::default();
    let start = orbit.azimuth_rad;
    orbit.advance(0.2);
    let turned = (orbit.azimuth_rad - start).rem_euclid(TAU);
    assert!((turned - HANGAR_AUTO_ROTATE_RAD_PER_S * 0.2).abs() < 1e-9);
    // A stalled frame advances by at most a quarter second.
    let before = orbit.azimuth_rad;
    orbit.advance(30.0);
    let jumped = (orbit.azimuth_rad - before).rem_euclid(TAU);
    assert!((jumped - HANGAR_AUTO_ROTATE_RAD_PER_S * 0.25).abs() < 1e-9);
    orbit.advance(10.0);
    assert!(orbit.azimuth_rad >= 0.0 && orbit.azimuth_rad < TAU);
    orbit.auto_rotate = false;
    let held = orbit.azimuth_rad;
    orbit.advance(1.0);
    assert_eq!(orbit.azimuth_rad, held);
    orbit.nudge(-10.0, 10.0);
    assert!(orbit.azimuth_rad >= 0.0 && orbit.azimuth_rad < TAU);
    assert!(orbit.elevation_rad < std::f64::consts::FRAC_PI_2);
    orbit.nudge(f64::NAN, -10.0);
    assert!(orbit.elevation_rad < 0.0 && orbit.elevation_rad > -0.1);
    orbit.zoom_by(100.0);
    assert!(orbit.zoom <= 4.0);
    orbit.zoom_by(-1.0);
    assert!(orbit.zoom <= 4.0 && orbit.zoom > 0.0);
}
