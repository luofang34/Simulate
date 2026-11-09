use std::io::Write;

use super::*;
use crate::aperture::MicrobolometerConfig;

const FRAME_S: f64 = 1.0 / 60.0;

/// A small, noiseless, perfectly uniform core, so counts show the lag alone.
fn quiet(width: u32) -> Microbolometer {
    Microbolometer::new(MicrobolometerConfig {
        width,
        height: 4,
        netd_k: 0.0,
        gain_nonuniformity: 0.0,
        offset_nonuniformity_k: 0.0,
        column_offset_k: 0.0,
        column_noise_k: 0.0,
        vignetting: 0.0,
        ..MicrobolometerConfig::vox_640x512()
    })
    .unwrap()
}

fn held(scene: &SceneTemperature, duration_s: f64) -> SceneInterval<'_> {
    SceneInterval { scene, duration_s }
}

#[test]
fn a_still_scene_reads_exactly_as_the_settled_capture() {
    let config = MicrobolometerConfig {
        seed: 4,
        ..MicrobolometerConfig::asi_640x512()
    };
    let core = Microbolometer::new(config).unwrap();
    let mut scene = SceneTemperature::uniform(640, 512, 290.0);
    for (kelvin, i) in scene.kelvin.iter_mut().zip(0..) {
        *kelvin += (i % 97) as f32 * 0.37;
    }
    let mut lag = ThermalLag::new();
    for sequence in 0..3 {
        let lagged = core.capture_lagged(&mut lag, &[held(&scene, FRAME_S)], sequence);
        assert_eq!(lagged.unwrap(), core.capture(&scene, sequence).unwrap());
    }
}

#[test]
fn a_step_is_followed_with_the_configured_time_constant() {
    let core = quiet(8);
    let tau = core.config().time_constant_s;
    let cold = SceneTemperature::uniform(8, 4, 300.0);
    let warm = SceneTemperature::uniform(8, 4, 310.0);
    let mut lag = ThermalLag::new();
    core.capture_lagged(&mut lag, &[held(&cold, FRAME_S)], 0)
        .unwrap();
    let frame = core
        .capture_lagged(&mut lag, &[held(&warm, tau)], 1)
        .unwrap();
    let reached = (lag.detector_kelvin()[0] - 300.0) / 10.0;
    let measured_tau = -tau / (1.0 - reached).ln();
    writeln!(
        std::io::stderr(),
        "step: {:.4} of the step after τ, measured τ {:.4} ms",
        reached,
        measured_tau * 1e3
    )
    .ok();
    assert!((measured_tau / tau - 1.0).abs() < 1e-9);
    assert_eq!(frame.image.counts[0], 6316, "6000 + 50·10·(1 − 1/e)");
    assert_eq!(frame.truth, warm);

    // At 60 Hz a 10 ms detector shows 81% of a step one frame later and
    // 96% two frames later.
    let next = core
        .capture_lagged(&mut lag, &[held(&warm, FRAME_S)], 2)
        .unwrap();
    let expected = 310.0 - 10.0 * (-1.0_f64).exp() * (-FRAME_S / tau).exp();
    assert!((lag.detector_kelvin()[0] - expected).abs() < 1e-9);
    assert_eq!(
        f64::from(next.image.counts[0]),
        (6000.0 + 50.0 * (expected - 300.0)).round()
    );
}

#[test]
fn a_moving_hot_bar_leaves_an_exponential_trail_behind_it() {
    const WIDTH: u32 = 64;
    const STEPS: u32 = 20;
    const STEP_S: f64 = 0.001;
    let core = quiet(WIDTH);
    let tau = core.config().time_constant_s;
    // A 100 K hotter column moves one pixel toward +x every millisecond.
    let scenes: Vec<SceneTemperature> = (0..STEPS)
        .map(|step| {
            let mut scene = SceneTemperature::uniform(WIDTH, 4, 300.0);
            for row in 0..4 {
                scene.kelvin[(row * WIDTH + 10 + step) as usize] = 400.0;
            }
            scene
        })
        .collect();
    let mut lag = ThermalLag::new();
    let background = SceneTemperature::uniform(WIDTH, 4, 300.0);
    core.capture_lagged(&mut lag, &[held(&background, FRAME_S)], 0)
        .unwrap();
    let intervals: Vec<_> = scenes.iter().map(|scene| held(scene, STEP_S)).collect();
    let frame = core.capture_lagged(&mut lag, &intervals, 1).unwrap();

    let head = (10 + STEPS - 1) as usize;
    let retained = (-STEP_S / tau).exp();
    let excess = |x: usize| lag.detector_kelvin()[x] - 300.0;
    for behind in 0..STEPS as usize {
        let expected = 100.0 * (1.0 - retained) * retained.powi(behind as i32);
        assert!(
            (excess(head - behind) - expected).abs() < 1e-9,
            "{behind} px behind"
        );
    }
    assert!(excess(head + 1).abs() < 1e-12, "nothing ahead of the bar");
    // The trail falls to 1/e over v·τ = 10 px.
    let trail = excess(head - 10) / excess(head);
    writeln!(
        std::io::stderr(),
        "smear: head +{:.2} K, 10 px behind {:.3} of it (1/e = {:.3})",
        excess(head),
        trail,
        (-1.0_f64).exp()
    )
    .ok();
    assert!((trail - (-1.0_f64).exp()).abs() < 1e-9);
    let counts_at = |x: usize| i32::from(frame.image.counts[x]);
    assert!(counts_at(head) > counts_at(head - 5) && counts_at(head - 5) > counts_at(head - 15));
    assert_eq!(counts_at(head + 1), 6000);
    assert_eq!(frame.truth, scenes[STEPS as usize - 1]);
}

#[test]
fn bad_intervals_are_refused_and_leave_the_state_alone() {
    let core = quiet(8);
    let scene = SceneTemperature::uniform(8, 4, 300.0);
    let mut lag = ThermalLag::new();
    assert!(core.capture_lagged(&mut lag, &[], 0).is_err());
    assert!(
        core.capture_lagged(&mut lag, &[held(&scene, -1.0)], 0)
            .is_err()
    );
    assert!(
        core.capture_lagged(&mut lag, &[held(&scene, f64::NAN)], 0)
            .is_err()
    );
    assert!(lag.detector_kelvin().is_empty());
    let wrong_size = SceneTemperature::uniform(9, 4, 300.0);
    assert!(
        core.capture_lagged(&mut lag, &[held(&wrong_size, 0.01)], 0)
            .is_err()
    );
    let mut other = ThermalLag::new();
    quiet(16)
        .capture_lagged(
            &mut other,
            &[held(&SceneTemperature::uniform(16, 4, 300.0), 0.01)],
            0,
        )
        .unwrap();
    assert!(
        core.capture_lagged(&mut other, &[held(&scene, 0.01)], 0)
            .is_err()
    );
    assert_eq!(other.detector_kelvin().len(), 64);
}
