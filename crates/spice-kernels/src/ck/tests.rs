use alloc::vec;
use alloc::vec::Vec;

use super::quaternion::{matrix_to_quaternion, propagate_constant_av, quaternion_to_matrix};
use super::{CkData, CkFile, CkInterval, CkPointingInstance, CkSegment, CkType2, CkType3};
use crate::daf::{Daf, Endian};
use crate::error::Error;
use crate::spk::FRAME_J2000;

fn mat_mul_vec(m: [[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

#[test]
fn spice_quaternion_and_c_matrix_sign_convention_is_exact() {
    // Body rotated +90 deg about +Z_ref: +X_body points along +Y_ref.
    // Therefore C = R_{ref -> body} maps (0, 1, 0)_ref -> (1, 0, 0)_body.
    let s = libm::sin(core::f64::consts::FRAC_PI_4);
    let c = libm::cos(core::f64::consts::FRAC_PI_4);
    let q_ck = [c, 0.0, 0.0, -s];
    let c_mat = quaternion_to_matrix(q_ck);
    let v_body = mat_mul_vec(c_mat, [0.0, 1.0, 0.0]);
    assert!((v_body[0] - 1.0).abs() < 1e-15);
    assert!(v_body[1].abs() < 1e-15);
    assert!(v_body[2].abs() < 1e-15);

    let q_back = matrix_to_quaternion(c_mat);
    for (a, b) in q_ck.iter().zip(&q_back) {
        assert!((a - b).abs() < 1e-15);
    }

    // Constant-rate propagation about +Z_ref at +1 rad/s for pi/2 from identity
    // arrives at the same +90 deg rotation.
    let q_spin = propagate_constant_av(
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        core::f64::consts::FRAC_PI_2,
    );
    for (a, b) in q_spin.iter().zip(&q_ck) {
        assert!((a - b).abs() < 1e-15);
    }
}

#[test]
fn ck_type2_directory_layout_round_trip_and_big_endian() {
    let n = 250_usize;
    let rate = 0.01_f64;
    let av = [0.1, -0.2, 0.3];
    let mut intervals = Vec::with_capacity(n);
    let mut q = [1.0, 0.0, 0.0, 0.0];
    for k in 0..n {
        let start = 1000.0 + (k as f64) * 100.0;
        let stop = start + 100.0;
        intervals.push(CkInterval {
            start_sclk: start,
            stop_sclk: stop,
            seconds_per_tick: rate,
            quaternion: q,
            angular_velocity: av,
        });
        q = propagate_constant_av(q, av, (stop - start) * rate);
    }
    let t2 = CkType2::new(intervals.clone()).unwrap();
    let words = t2.to_words();
    // 250 intervals -> 250*10 + (250-1)/100 = 2502 words, directory midpoints at 100th/200th.
    assert_eq!(words.len(), 2502);
    assert_eq!(
        words[2500],
        0.5 * (intervals[99].stop_sclk + intervals[100].start_sclk)
    );
    assert_eq!(
        words[2501],
        0.5 * (intervals[199].stop_sclk + intervals[200].start_sclk)
    );

    let seg = CkSegment::type2("type2-250", -82_000, FRAME_J2000, t2).unwrap();
    let mut ck = CkFile::new("ck type2 test").unwrap();
    ck.push(seg);
    let bytes = ck.to_bytes().unwrap();
    let parsed = CkFile::parse(&bytes).unwrap();
    assert_eq!(parsed, ck);
    assert_eq!(parsed.to_bytes().unwrap(), bytes);

    // Query at interior and at interval boundary (which selects the interval starting there).
    let sclk_mid = 1000.0 + 173.4 * 100.0;
    let pt = parsed
        .pointing_and_angular_velocity(-82_000, sclk_mid, 0.0)
        .unwrap();
    let want_q = propagate_constant_av([1.0, 0.0, 0.0, 0.0], av, (sclk_mid - 1000.0) * rate);
    for (a, b) in pt.quaternion.iter().zip(&want_q) {
        assert!((a - b).abs() < 1e-13);
    }
    assert_eq!(pt.angular_velocity, Some(av));
}

#[test]
fn ck_type2_tolerance_and_gaps() {
    let intervals = vec![
        CkInterval {
            start_sclk: 100.0,
            stop_sclk: 200.0,
            seconds_per_tick: 1.0,
            quaternion: [1.0, 0.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 0.05],
        },
        CkInterval {
            start_sclk: 300.0,
            stop_sclk: 400.0,
            seconds_per_tick: 1.0,
            quaternion: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0, 0.0, -0.05],
        },
    ];
    let mut ck = CkFile::new("gaps").unwrap();
    ck.push(
        CkSegment::type2(
            "gapped",
            -100,
            FRAME_J2000,
            CkType2::new(intervals).unwrap(),
        )
        .unwrap(),
    );
    let parsed = CkFile::parse(&ck.to_bytes().unwrap()).unwrap();

    assert!(matches!(
        parsed.pointing_and_angular_velocity(-100, 250.0, 10.0),
        Err(Error::NoCkCoverage { .. })
    ));
    let left = parsed
        .pointing_and_angular_velocity(-100, 205.0, 10.0)
        .unwrap();
    assert_eq!(left.sclk, 200.0);
    let right = parsed
        .pointing_and_angular_velocity(-100, 292.0, 10.0)
        .unwrap();
    assert_eq!(right.sclk, 300.0);
    assert_eq!(right.quaternion, [0.0, 0.0, 0.0, 1.0]);
}

#[test]
fn ck_type3_directories_slerp_and_avflag_variants() {
    // 250 instances across 120 intervals so BOTH SCLK directory (2 entries)
    // and STARTS directory (1 entry) are written and parsed back.
    let mut instances = Vec::with_capacity(250);
    let mut starts = Vec::with_capacity(120);
    let axis = [0.0, 0.0, 1.0];
    for k in 0..250_usize {
        let sclk = 1000.0 + (k as f64) * 10.0;
        let q = propagate_constant_av([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.02], k as f64 * 10.0);
        instances.push(CkPointingInstance {
            sclk,
            quaternion: q,
            angular_velocity: Some([0.0, 0.0, 0.02]),
        });
        if k < 120 {
            starts.push(sclk);
        }
    }
    let t3 = CkType3::new(instances.clone(), starts.clone(), true).unwrap();
    let words = t3.to_words();
    // 250*7 + 250 + 2 + 120 + 1 + 2 = 2125.
    assert_eq!(words.len(), 2125);

    let seg_with_av = CkSegment::type3("t3-av", -500, FRAME_J2000, t3).unwrap();
    let no_av_insts: Vec<_> = instances[..5]
        .iter()
        .map(|i| CkPointingInstance {
            sclk: i.sclk,
            quaternion: i.quaternion,
            angular_velocity: None,
        })
        .collect();
    let seg_no_av = CkSegment::type3(
        "t3-noav",
        -501,
        FRAME_J2000,
        CkType3::single_interval(no_av_insts, false).unwrap(),
    )
    .unwrap();

    let mut ck = CkFile::new("type3").unwrap();
    ck.push(seg_with_av);
    ck.push(seg_no_av);
    let parsed = CkFile::parse(&ck.to_bytes().unwrap()).unwrap();
    assert_eq!(parsed, ck);

    // Last interval (index 119) spans k=119..=249: SLERP at midpoint sclk = 1000 + 200.5*10.
    let mid_sclk = 3005.0;
    let pt = parsed
        .pointing_and_angular_velocity(-500, mid_sclk, 0.0)
        .unwrap();
    let want_q = propagate_constant_av([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.02], mid_sclk - 1000.0);
    for (a, b) in pt.quaternion.iter().zip(&want_q) {
        assert!((a - b).abs() < 1e-13);
    }
    assert_eq!(pt.angular_velocity, Some([0.0, 0.0, 0.02]));
    let pt_noav = parsed
        .pointing_and_angular_velocity(-501, 1015.0, 0.0)
        .unwrap();
    assert_eq!(pt_noav.angular_velocity, None);
    let _ = axis;
}

#[test]
fn ck_precedence_raw_passthrough_and_validation() {
    let q_a = [1.0, 0.0, 0.0, 0.0];
    let q_b = [0.0, 1.0, 0.0, 0.0];
    let s1 = CkSegment::type3(
        "early",
        -77,
        FRAME_J2000,
        CkType3::single_interval(
            vec![
                CkPointingInstance {
                    sclk: 0.0,
                    quaternion: q_a,
                    angular_velocity: None,
                },
                CkPointingInstance {
                    sclk: 100.0,
                    quaternion: q_a,
                    angular_velocity: None,
                },
            ],
            false,
        )
        .unwrap(),
    )
    .unwrap();
    let s2 = CkSegment::type3(
        "late",
        -77,
        FRAME_J2000,
        CkType3::single_interval(
            vec![
                CkPointingInstance {
                    sclk: 40.0,
                    quaternion: q_b,
                    angular_velocity: None,
                },
                CkPointingInstance {
                    sclk: 60.0,
                    quaternion: q_b,
                    angular_velocity: None,
                },
            ],
            false,
        )
        .unwrap(),
    )
    .unwrap();
    let raw = CkSegment::from_stored("raw", -78, 1, [0.0, 10.0], 5, 1, vec![1.0, 2.0]).unwrap();
    assert!(matches!(
        raw.data(),
        CkData::Raw {
            data_type: 5,
            avflag: 1,
            ..
        }
    ));
    assert!(matches!(
        raw.pointing(5.0, 0.0),
        Err(Error::UnsupportedCkDataType { data_type: 5 })
    ));

    let mut ck = CkFile::new("precedence").unwrap();
    ck.push(s1);
    ck.push(s2);
    ck.push(raw);
    let parsed = CkFile::parse(&ck.to_bytes().unwrap()).unwrap();
    assert_eq!(parsed.attitude_at(-77, 50.0, 0.0).unwrap().quaternion, q_b);
    assert_eq!(parsed.attitude_at(-77, 20.0, 0.0).unwrap().quaternion, q_a);

    // Ensure non-CK DAF is rejected.
    let spk_daf = Daf::new("SPK", 2, 6, "").unwrap();
    assert!(matches!(
        CkFile::from_daf(&spk_daf),
        Err(Error::WrongKernelType { .. })
    ));
    let _ = Endian::Big;
}
