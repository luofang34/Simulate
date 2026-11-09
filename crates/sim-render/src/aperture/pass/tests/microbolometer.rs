use sim_geo::{BodyEllipsoid, Geodetic};
use sim_scenery::SurfaceRadiometry;

use super::{HEIGHT, UniformGround, WIDTH, block, gpu, nadir, render, thermal_frame};
use crate::aperture::{
    Microbolometer, MicrobolometerConfig, SceneTemperature, SensorAperturePass, ThermalScale,
    ThermalSceneTarget,
};
use crate::{CameraPose, GlobeMesh, PinholeProjection, TerrainSources};

const SCALE: ThermalScale = ThermalScale {
    black_k: 250.0,
    white_k: 350.0,
    nedt_k: 0.0,
    noise_seed: 0,
};

/// Uniform 300 K black-body ground under a nadir camera 2 km up.
fn warm_ground() -> (GlobeMesh, CameraPose) {
    let body = BodyEllipsoid::WGS84;
    let centre = Geodetic::from_degrees(39.1, -75.5, 0.0);
    let ground = UniformGround(SurfaceRadiometry {
        albedo: 0.2,
        emissivity: 1.0,
        temperature_k: 300.0,
    });
    let sources = TerrainSources {
        radiometry: Some(&ground),
        ..Default::default()
    };
    let mesh = GlobeMesh::build_region(&block(body, &centre, 0.002, 4), sources).unwrap();
    (mesh, nadir(&body, &centre, 2_000.0))
}

#[test]
fn the_kelvin_target_reads_the_scene_unquantised() {
    let Some(gpu) = gpu() else { return };
    let (mesh, nadir_pose) = warm_ground();
    let projection = PinholeProjection::new(super::intrinsics(200.0), super::NEAR_M).unwrap();
    let target = ThermalSceneTarget::new(&gpu.device, projection).unwrap();
    let mut pass = pollster::block_on(SensorAperturePass::new(
        &gpu.device,
        &gpu.queue,
        &mesh,
        None,
    ))
    .unwrap();
    pass.render_scene_temperature(&gpu.device, &gpu.queue, &target, &nadir_pose);
    let scene = target
        .read_blocking(&gpu.device, &gpu.queue, 220.0)
        .unwrap();
    let middle = scene.kelvin_at(WIDTH / 2, HEIGHT / 2).unwrap();
    assert!((middle - 300.0).abs() < 1e-3, "{middle} K");
    // Looking at the horizon, the upper half of the frame is open sky.
    let horizon = CameraPose::looking(
        nadir_pose.position_ecef,
        nadir_pose.orientation * glam::DVec3::Y,
        -(nadir_pose.orientation * glam::DVec3::NEG_Z),
    );
    pass.render_scene_temperature(&gpu.device, &gpu.queue, &target, &horizon);
    let sky = target
        .read_blocking(&gpu.device, &gpu.queue, 220.0)
        .unwrap();
    assert_eq!(sky.kelvin_at(WIDTH / 2, 2), Some(220.0));
    gpu.check_device().unwrap();
    let config = MicrobolometerConfig {
        width: WIDTH,
        height: HEIGHT,
        ..MicrobolometerConfig::vox_640x512()
    };
    let frame = Microbolometer::new(config)
        .unwrap()
        .capture(&scene, 0)
        .unwrap();
    assert_eq!(frame.truth, scene);
}

#[test]
fn a_rendered_thermal_frame_feeds_the_microbolometer() {
    let Some(gpu) = gpu() else { return };
    let (mesh, pose) = warm_ground();
    let image = render(&gpu, &mesh, None, 200.0, &thermal_frame(pose, SCALE));
    let scene = SceneTemperature::from_thermal_image(&image, &SCALE).unwrap();
    // One grey level is 0.4 K at 300 K on this scale.
    let middle = scene.kelvin_at(WIDTH / 2, HEIGHT / 2).unwrap();
    assert!((middle - 300.0).abs() < 0.25, "{middle} K");
    let config = MicrobolometerConfig {
        width: WIDTH,
        height: HEIGHT,
        ..MicrobolometerConfig::vox_640x512()
    };
    let frame = Microbolometer::new(config)
        .unwrap()
        .capture(&scene, 0)
        .unwrap();
    let n = frame.image.counts.len() as f64;
    let mean = frame
        .image
        .counts
        .iter()
        .map(|&c| f64::from(c))
        .sum::<f64>()
        / n;
    let expected = config.reference_counts
        + config.responsivity_counts_per_k * (f64::from(middle) - config.reference_temperature_k);
    // Vignetting lowers the mean contrast by under 2% of a fraction of a kelvin.
    assert!((mean - expected).abs() < 2.0, "{mean} vs {expected}");
    assert_eq!(frame.truth, scene);
}
