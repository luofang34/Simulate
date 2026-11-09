use super::*;
use crate::{ApertureImage, ThermalScale};

fn core(config: MicrobolometerConfig) -> Microbolometer {
    Microbolometer::new(config).unwrap()
}

fn flat(config: &MicrobolometerConfig, kelvin: f32) -> SceneTemperature {
    SceneTemperature::uniform(config.width, config.height, kelvin)
}

/// Per-pixel temporal standard deviation over `frames`, in counts, pooled
/// over the detector.
fn temporal_std_counts(frames: &[MicrobolometerFrame]) -> f64 {
    let n = frames.len() as f64;
    let pixels = frames[0].image.counts.len();
    let mut variance = 0.0;
    for pixel in 0..pixels {
        let values: Vec<f64> = frames
            .iter()
            .map(|f| f64::from(f.image.counts[pixel]))
            .collect();
        let mean = values.iter().sum::<f64>() / n;
        variance += values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n - 1.0);
    }
    (variance / pixels as f64).sqrt()
}

fn spatial_std(values: &[u16]) -> f64 {
    let n = values.len() as f64;
    let mean = values.iter().map(|&v| f64::from(v)).sum::<f64>() / n;
    (values
        .iter()
        .map(|&v| (f64::from(v) - mean).powi(2))
        .sum::<f64>()
        / n)
        .sqrt()
}

#[test]
fn measured_temporal_netd_is_within_ten_percent_of_the_configured() {
    for preset in [
        MicrobolometerConfig::vox_640x512(),
        MicrobolometerConfig::asi_640x512(),
    ] {
        let config = MicrobolometerConfig {
            column_noise_k: 0.0,
            seed: 3,
            ..preset
        };
        let core = core(config);
        let scene = flat(&config, 300.0);
        let frames: Vec<_> = (0..6)
            .map(|seq| core.capture(&scene, seq).unwrap())
            .collect();
        let measured_k = temporal_std_counts(&frames) / config.responsivity_counts_per_k;
        let error = measured_k / config.netd_k - 1.0;
        writeln_stderr(&format!(
            "NETD configured {} mK, measured {:.2} mK ({:+.1}%)",
            config.netd_k * 1e3,
            measured_k * 1e3,
            error * 100.0
        ));
        assert!(error.abs() < 0.10, "measured {measured_k} K");
    }
}

#[test]
fn column_noise_moves_whole_columns_from_frame_to_frame() {
    let config = MicrobolometerConfig {
        netd_k: 0.0,
        column_noise_k: 0.2,
        ..MicrobolometerConfig::vox_640x512()
    };
    let core = core(config);
    let scene = flat(&config, 300.0);
    let [a, b] = [0, 1].map(|seq| core.capture(&scene, seq).unwrap().image.counts);
    let width = config.width as usize;
    let column_shift: Vec<f64> = (0..width)
        .map(|x| f64::from(a[x]) - f64::from(b[x]))
        .collect();
    for row in 1..config.height as usize {
        for x in 0..width {
            let shift = f64::from(a[row * width + x]) - f64::from(b[row * width + x]);
            assert!(
                (shift - column_shift[x]).abs() <= 1.0,
                "row {row} column {x}"
            );
        }
    }
    let n = column_shift.len() as f64;
    let rms = (column_shift.iter().map(|s| s * s).sum::<f64>() / n).sqrt();
    let expected = 2.0_f64.sqrt() * 0.2 * config.responsivity_counts_per_k;
    assert!((rms / expected - 1.0).abs() < 0.15, "{rms} vs {expected}");
}

#[test]
fn fixed_pattern_holds_across_frames_and_changes_with_the_seed() {
    let quiet = MicrobolometerConfig {
        netd_k: 0.0,
        column_noise_k: 0.0,
        seed: 21,
        ..MicrobolometerConfig::vox_640x512()
    };
    let scene = flat(&quiet, 330.0);
    let core_a = core(quiet);
    let first = core_a.capture(&scene, 0).unwrap().image;
    assert_eq!(first, core_a.capture(&scene, 99).unwrap().image);
    assert_eq!(first, core(quiet).capture(&scene, 7).unwrap().image);
    let other = core(MicrobolometerConfig { seed: 22, ..quiet })
        .capture(&scene, 0)
        .unwrap()
        .image;
    assert_ne!(first, other);
    // 30 K above the reference: 1% gain spread is 0.3 K, plus 0.2 K offset
    // and 0.05 K column offset, then vignetting's gentle bowl.
    let pattern_k = spatial_std(&first.counts) / quiet.responsivity_counts_per_k;
    assert!(
        (0.3..0.6).contains(&pattern_k),
        "fixed pattern {pattern_k} K"
    );
}

#[test]
fn reruns_are_bit_identical_and_frames_differ() {
    let config = MicrobolometerConfig::asi_640x512();
    let scene = flat(&config, 295.0);
    let frame = core(config).capture(&scene, 4).unwrap();
    assert_eq!(frame, core(config).capture(&scene, 4).unwrap());
    assert_ne!(frame.image, core(config).capture(&scene, 5).unwrap().image);
    assert_eq!(frame.truth, scene);
}

#[test]
fn counts_stay_in_fourteen_bits_and_follow_the_responsivity() {
    let config = MicrobolometerConfig {
        gain_nonuniformity: 0.0,
        offset_nonuniformity_k: 0.0,
        column_offset_k: 0.0,
        column_noise_k: 0.0,
        netd_k: 0.0,
        vignetting: 0.0,
        ..MicrobolometerConfig::vox_640x512()
    };
    let core = core(config);
    let mut scene = flat(&config, 300.0);
    for (kelvin, ramp) in scene.kelvin.iter_mut().zip(0..) {
        *kelvin = 100.0 + 500.0 * (ramp % 640) as f32 / 639.0;
    }
    let image = core.capture(&scene, 0).unwrap().image;
    assert_eq!(image.significant_bits, 14);
    assert!(image.counts.iter().all(|&c| c <= 16_383));
    assert_eq!(image.count(0, 0), Some(0), "100 K clips at zero");
    assert_eq!(
        image.count(639, 0),
        Some(16_383),
        "600 K clips at full scale"
    );
    let at = |kelvin: f32| {
        let mut probe = flat(&config, kelvin);
        probe.kelvin[0] = kelvin;
        core.capture(&probe, 0).unwrap().image.counts[0]
    };
    assert_eq!(at(300.0), 6000);
    assert_eq!(at(310.0), 6500);
    assert_eq!(at(250.0), 3500);
}

#[test]
fn presets_state_their_optics() {
    let vox = MicrobolometerConfig::vox_640x512();
    let asi = MicrobolometerConfig::asi_640x512();
    assert!((vox.horizontal_fov_deg() - 32.9).abs() < 0.05);
    assert!((asi.horizontal_fov_deg() - 32.0).abs() < 0.05);
    let intrinsics = vox.intrinsics();
    assert_eq!((intrinsics.width, intrinsics.height), (640, 512));
    assert_eq!((intrinsics.cx, intrinsics.cy), (319.5, 255.5));
    assert!((intrinsics.fx - 13.0 / 0.012).abs() < 1e-9);
}

#[test]
fn bad_scenes_and_configurations_are_refused() {
    let config = MicrobolometerConfig::vox_640x512();
    let core = core(config);
    assert!(
        core.capture(&SceneTemperature::uniform(320, 256, 300.0), 0)
            .is_err()
    );
    let mut hole = flat(&config, 300.0);
    hole.kelvin[10] = f32::NAN;
    assert!(core.capture(&hole, 0).is_err());
    for bad in [
        MicrobolometerConfig {
            netd_k: -0.1,
            ..config
        },
        MicrobolometerConfig {
            vignetting: 1.0,
            ..config
        },
        MicrobolometerConfig {
            pixel_pitch_m: 0.0,
            ..config
        },
    ] {
        assert!(matches!(
            Microbolometer::new(bad),
            Err(RenderError::InvalidSensor { .. })
        ));
    }
}

#[test]
fn scene_from_a_thermal_frame_inverts_the_eight_bit_exitance_scale() {
    let scale = ThermalScale {
        black_k: 250.0,
        white_k: 350.0,
        nedt_k: 0.0,
        noise_seed: 0,
    };
    let greys = [0_u8, 255, 69, 128];
    let image = ApertureImage {
        width: 2,
        height: 2,
        rgba8: greys.iter().flat_map(|&g| [g, g, g, 255]).collect(),
        depth: vec![0.0; 4],
    };
    let scene = SceneTemperature::from_thermal_image(&image, &scale).unwrap();
    assert!((scene.kelvin[0] - 250.0).abs() < 1e-3);
    assert!((scene.kelvin[1] - 350.0).abs() < 1e-3);
    let sigma = 5.670_374e-8_f64;
    let exitance = |k: f64| sigma * k.powi(4);
    let expected = ((exitance(250.0) + 128.0 / 255.0 * (exitance(350.0) - exitance(250.0)))
        / sigma)
        .powf(0.25);
    assert!((f64::from(scene.kelvin[3]) - expected).abs() < 1e-3);
    assert_eq!(scene.kelvin_at(1, 1), Some(scene.kelvin[3]));
    let wrong = ApertureImage {
        rgba8: vec![0; 8],
        ..image
    };
    assert!(SceneTemperature::from_thermal_image(&wrong, &scale).is_err());
}

fn writeln_stderr(line: &str) {
    use std::io::Write;
    writeln!(std::io::stderr(), "{line}").ok();
}
