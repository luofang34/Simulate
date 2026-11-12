use sim_navigate::trial::DirectoryStores;

use super::*;
use crate::{SpoofKind, SpoofWindowSpec, StepOutcome, first_link};

fn step_seconds(trial: &mut ChainTrial, until_s: f64) {
    match trial.step_until(until_s).unwrap() {
        StepOutcome::Advanced(_) => {}
        StepOutcome::FrameWanted(req, _) => panic!("unexpected frame request {req:?}"),
    }
}

fn assert_rnp_triad(samples: &[TelemetrySample]) {
    let guided_count = samples
        .iter()
        .filter(|s| s.guidance_rnp.tse_cross_track_m.is_some())
        .map(|s| {
            let rnp = &s.guidance_rnp;
            let tse = rnp.tse_cross_track_m.unwrap();
            let fte = rnp.fte_cross_track_m.unwrap();
            let nse = rnp.nse_cross_track_m.unwrap();
            assert!((tse - (fte + nse)).abs() < 1e-9);
            let nav = s
                .navigators
                .iter()
                .find(|n| n.set == "ins.baro.gnss")
                .unwrap();
            let expected_bound = nav.epu_95_m.unwrap().hypot(fte);
            assert!((rnp.rnp_95_bound_m.unwrap() - expected_bound).abs() < 1e-9);
            assert!((rnp.rnp_limit_m - 0.3 * METRES_PER_NM).abs() < 1e-9);
        })
        .count();
    assert!(guided_count >= 30);
}

#[test]
fn nominal_flight_records_pitot_nees_and_rnp_triad() {
    let spec = first_link().unwrap();
    let mut trial = ChainTrial::new(&spec, &DirectoryStores).unwrap();
    let mut recorder = TelemetryRecorder::new(
        &spec,
        TelemetryConfig {
            rnp_limit_nm: Some(0.3),
            seed: Some(7),
        },
    );

    let mut samples = Vec::new();
    for sec in 1..=35_u32 {
        step_seconds(&mut trial, f64::from(sec));
        let sample = recorder
            .sample_with_terrain(&mut trial, |_, _| Some(215.0))
            .unwrap();
        let encoded = serde_json::to_string(&sample).unwrap();
        let decoded: TelemetrySample = serde_json::from_str(&encoded).unwrap();
        assert_eq!((decoded.schema, decoded.seed), (TELEMETRY_SCHEMA, Some(7)));
        samples.push(sample);
    }

    let last = samples.last().unwrap();
    assert!((last.truth.terrain_elev_m - 215.0).abs() < 1e-9);
    assert!((last.truth.alt_agl_m - (last.truth.alt_msl_m - 215.0)).abs() < 1e-9);
    assert!(last.truth.tas_mps > 40.0);
    assert!(last.inputs.pitot_ok && last.inputs.baro_ok && last.inputs.imu_ok);
    assert!(last.inputs.mag_ok && last.inputs.gnss_available && !last.inputs.spoof_active);
    assert!(last.inputs.pitot_diff_pressure_pa.unwrap() > 800.0);
    assert!(last.inputs.ias_mps.unwrap() > 35.0);

    let steered = last
        .navigators
        .iter()
        .find(|n| n.set == "ins.baro.gnss")
        .unwrap();
    assert_eq!(
        (
            steered.position_source.as_str(),
            steered.gnss_integrity.as_str()
        ),
        ("gnss", "normal")
    );
    assert!(steered.nees_horiz_2d.unwrap() >= 0.0 && steered.nees_pos_3d.unwrap() >= 0.0);
    assert!(steered.epu_95_m.unwrap() > 0.0 && steered.vpu_95_m.unwrap() > 0.0);

    assert_rnp_triad(&samples);

    let summary = recorder.finish();
    assert_eq!(
        (summary.sample_count, summary.steered_set.as_str()),
        (35, "ins.baro.gnss")
    );
    assert!(summary.horizontal_error_m.p95 < 25.0);
    assert!(summary.nees_horiz_2d.unwrap().mean > 0.0);
    assert!((summary.rnp_containment_fraction - 1.0).abs() < 1e-9);
    assert!(!summary.spoofing.spoof_injected && summary.position_source_pct["gnss"] > 90.0);
}

#[test]
fn multi_window_spoofing_toggles_and_records_exclusion_metrics() {
    let mut spec = first_link().unwrap();
    spec.spoof_windows = vec![SpoofWindowSpec {
        start_s: 6.0,
        end_s: Some(32.0),
        kind: SpoofKind::Ramp,
        offset_enu_m: [120.0, 0.0, 0.0],
        rate_enu_mps: [18.0, 0.0, 0.0],
        velocity_enu_mps: None,
    }];

    let mut trial = ChainTrial::new(&spec, &DirectoryStores).unwrap();
    let mut recorder = TelemetryRecorder::new(&spec, TelemetryConfig::default());

    let mut samples = Vec::new();
    for sec in 1..=44_u32 {
        step_seconds(&mut trial, f64::from(sec));
        samples.push(recorder.sample(&mut trial).unwrap());
    }

    let pre = &samples[3];
    assert!(!pre.inputs.spoof_active && pre.inputs.spoof_kind == SpoofKind::None);
    assert_eq!(pre.inputs.spoof_offset_enu_m, [0.0, 0.0, 0.0]);

    let mid = &samples[15];
    assert!(mid.inputs.spoof_active && mid.inputs.spoof_kind == SpoofKind::Ramp);
    assert!(mid.inputs.spoof_offset_enu_m[0] > 250.0);
    assert!((mid.inputs.spoof_rate_enu_mps[0] - 18.0).abs() < 1e-6);
    assert!(
        mid.inputs.gnss_measured_lla.unwrap()[1] > mid.inputs.gnss_true_lla.unwrap()[1] + 0.002
    );

    let post = &samples[35];
    assert!(!post.inputs.spoof_active && post.inputs.spoof_kind == SpoofKind::None);
    assert_eq!(post.inputs.spoof_offset_enu_m, [0.0, 0.0, 0.0]);
    let raw_diff_deg =
        (post.inputs.gnss_measured_lla.unwrap()[1] - post.inputs.gnss_true_lla.unwrap()[1]).abs();
    assert!(raw_diff_deg < 0.0002);

    let saw_excluded = samples.iter().any(|s| {
        s.navigators
            .iter()
            .find(|n| n.set == "ins.baro.gnss")
            .is_some_and(|n| n.gnss_integrity == "excluded")
    });
    assert!(
        saw_excluded,
        "expected GNSS integrity monitor to exclude ramp spoof"
    );

    let summary = recorder.finish();
    assert!(summary.spoofing.spoof_injected);
    assert!(summary.spoofing.spoof_start_s.unwrap() >= 6.0);
    assert!(summary.spoofing.spoof_end_s.unwrap() >= 32.0);
    assert!(summary.spoofing.time_to_detect_s.is_some());
    assert!(summary.spoofing.time_to_exclude_s.is_some());
    assert!(summary.spoofing.max_error_before_exclusion_m.unwrap() > 0.0);
}
