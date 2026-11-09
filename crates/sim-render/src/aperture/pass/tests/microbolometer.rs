use sim_geo::{BodyEllipsoid, Geodetic};
use sim_scenery::SurfaceRadiometry;

use super::{HEIGHT, UniformGround, WIDTH, block, gpu, nadir, render, thermal_frame};
use crate::aperture::{Microbolometer, MicrobolometerConfig, SceneTemperature, ThermalScale};
use crate::{GlobeMesh, TerrainSources};

const SCALE: ThermalScale = ThermalScale {
    black_k: 250.0,
    white_k: 350.0,
    nedt_k: 0.0,
    noise_seed: 0,
};

#[test]
fn a_rendered_thermal_frame_feeds_the_microbolometer() {
    let Some(gpu) = gpu() else { return };
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
    let pose = nadir(&body, &centre, 2_000.0);
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
