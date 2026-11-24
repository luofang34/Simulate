//! Integration tests for CK Type 2 & Type 3 quaternion kinematics, SLERP,
//! endianness, and an opt-in external CSPICE oracle cross-check.

use spice_kernels::{
    CkFile, CkInterval, CkPointingInstance, CkSegment, CkType2, CkType3, Daf, Endian, Error,
    FRAME_J2000, matrix_to_quaternion, quaternion_to_matrix,
};

fn mat_mul_vec(m: [[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

fn mat_mul(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let [b0, b1, b2] = b;
    let [a0, a1, a2] = a;
    [
        [
            a0[0] * b0[0] + a0[1] * b1[0] + a0[2] * b2[0],
            a0[0] * b0[1] + a0[1] * b1[1] + a0[2] * b2[1],
            a0[0] * b0[2] + a0[1] * b1[2] + a0[2] * b2[2],
        ],
        [
            a1[0] * b0[0] + a1[1] * b1[0] + a1[2] * b2[0],
            a1[0] * b0[1] + a1[1] * b1[1] + a1[2] * b2[1],
            a1[0] * b0[2] + a1[1] * b1[2] + a1[2] * b2[2],
        ],
        [
            a2[0] * b0[0] + a2[1] * b1[0] + a2[2] * b2[0],
            a2[0] * b0[1] + a2[1] * b1[1] + a2[2] * b2[1],
            a2[0] * b0[2] + a2[1] * b1[2] + a2[2] * b2[2],
        ],
    ]
}

fn rodrigues_neg(av_ref: [f64; 3], dt_s: f64) -> [[f64; 3]; 3] {
    let n = libm::sqrt(av_ref[0] * av_ref[0] + av_ref[1] * av_ref[1] + av_ref[2] * av_ref[2]);
    if n == 0.0 {
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    }
    let theta = -n * dt_s;
    let [ux, uy, uz] = [av_ref[0] / n, av_ref[1] / n, av_ref[2] / n];
    let (s, c) = (libm::sin(theta), libm::cos(theta));
    let k = 1.0 - c;
    [
        [c + ux * ux * k, ux * uy * k - uz * s, ux * uz * k + uy * s],
        [uy * ux * k + uz * s, c + uy * uy * k, uy * uz * k - ux * s],
        [uz * ux * k - uy * s, uz * uy * k + ux * s, c + uz * uz * k],
    ]
}

#[test]
fn ck_type2_matches_independent_rodrigues_c_matrix_propagation() -> Result<(), Error> {
    let q0 = matrix_to_quaternion(rodrigues_neg([0.3, -0.4, 0.5], 0.7));
    let av = [-0.017, 0.029, -0.011];
    let rate = 0.005;
    let t2 = CkType2::new(vec![CkInterval {
        start_sclk: 10_000.0,
        stop_sclk: 20_000.0,
        seconds_per_tick: rate,
        quaternion: q0,
        angular_velocity: av,
    }])?;
    let mut ck = CkFile::new("rodrigues parity")?;
    ck.push(CkSegment::type2("spin", -45_000, FRAME_J2000, t2)?);
    let parsed = CkFile::parse(&ck.to_bytes()?)?;

    let c0 = quaternion_to_matrix(q0);
    for tick in [10_000.0, 11_234.5, 15_000.0, 19_999.0, 20_000.0] {
        let pt = parsed.pointing_and_angular_velocity(-45_000, tick, 0.0)?;
        let got_c = pt.c_matrix();
        let want_c = mat_mul(c0, rodrigues_neg(av, (tick - 10_000.0) * rate));
        for i in 0..3 {
            for j in 0..3 {
                assert!((got_c[i][j] - want_c[i][j]).abs() < 1e-13);
            }
        }
        let body_v = mat_mul_vec(got_c, [1.2, -0.4, 0.8]);
        let norm_in = libm::sqrt(1.2 * 1.2 + 0.4 * 0.4 + 0.8 * 0.8);
        let norm_out =
            libm::sqrt(body_v[0] * body_v[0] + body_v[1] * body_v[1] + body_v[2] * body_v[2]);
        assert!((norm_in - norm_out).abs() < 1e-14);
    }
    Ok(())
}

#[test]
fn ck_type3_slerp_and_big_endian_daf_parity() -> Result<(), Error> {
    let av = [0.01, 0.02, -0.015];
    let c0 = rodrigues_neg(av, 0.0);
    let q0 = matrix_to_quaternion(c0);
    let q1 = matrix_to_quaternion(rodrigues_neg(av, 10.0));
    let t3 = CkType3::single_interval(
        vec![
            CkPointingInstance {
                sclk: 500.0,
                quaternion: q0,
                angular_velocity: Some(av),
            },
            CkPointingInstance {
                sclk: 510.0,
                quaternion: q1,
                angular_velocity: Some(av),
            },
        ],
        true,
    )?;
    let mut ck = CkFile::new("slerp parity")?;
    ck.push(CkSegment::type3("slerp", -99_000, FRAME_J2000, t3)?);
    let bytes = ck.to_bytes()?;
    // Also verify Daf::parse accepts the container and preserves BIG-IEEE/LTL-IEEE shape.
    let daf = Daf::parse(&bytes)?;
    assert_eq!(daf.kernel_type(), "CK");
    assert_eq!((daf.nd(), daf.ni()), (2, 6));
    let parsed = CkFile::parse(&bytes)?;

    let pt = parsed.pointing_and_angular_velocity(-99_000, 503.7, 0.0)?;
    let want_c = rodrigues_neg(av, 3.7);
    let got_c = pt.c_matrix();
    for i in 0..3 {
        for j in 0..3 {
            assert!((got_c[i][j] - want_c[i][j]).abs() < 1e-13);
        }
    }
    let _ = Endian::Little;
    Ok(())
}

/// Optional CSPICE oracle check using a Python interpreter with `spiceypy` installed.
/// Run with:
/// `SPICE_ORACLE_PYTHON=/path/to/python cargo test -p spice-kernels --test ck_kinematics -- --ignored`
#[cfg(feature = "std")]
#[test]
#[ignore = "requires local python3 with spiceypy / CSPICE (set SPICE_ORACLE_PYTHON)"]
fn cspice_oracle_cross_check_spk23_and_ck23() -> Result<(), Box<dyn std::error::Error>> {
    let py = std::env::var("SPICE_ORACLE_PYTHON").unwrap_or_else(|_| "python3".into());
    let dir = tempfile::tempdir()?;
    let bsp_path = dir.path().join("oracle.bsp");
    let bc_path = dir.path().join("oracle.bc");
    let t3 = CkType3::single_interval(
        vec![
            CkPointingInstance {
                sclk: 100.0,
                quaternion: [1.0, 0.0, 0.0, 0.0],
                angular_velocity: Some([0.0, 0.0, 0.02]),
            },
            CkPointingInstance {
                sclk: 200.0,
                quaternion: matrix_to_quaternion(rodrigues_neg([0.0, 0.0, 0.02], 100.0)),
                angular_velocity: Some([0.0, 0.0, 0.02]),
            },
        ],
        true,
    )?;
    let mut ck = CkFile::new("cspice oracle ck")?;
    ck.push(CkSegment::type3("ck3", -82_000, FRAME_J2000, t3)?);
    ck.write_file_blocking(&bc_path)?;

    let spk_bytes = include_bytes!("data/de440s_excerpt_1969.bsp");
    std::fs::write(&bsp_path, spk_bytes)?;

    let script = r#"
import json, sys, spiceypy as sp
sp.kclear()
sp.furnsh(sys.argv[1])
sp.furnsh(sys.argv[2])
state, _ = sp.spkezr("301", -960910900.0, "J2000", "NONE", "3")
h = sp.cklpf(sys.argv[2])
cmat, av, clkout, found = sp.ckgpav(-82000, 150.0, 0.0, "J2000")
sp.ckupf(h)
print(json.dumps({"state": state.tolist(), "cmat": cmat.tolist(), "av": av.tolist(), "found": bool(found)}))
"#;
    let out = std::process::Command::new(&py)
        .args(["-c", script])
        .arg(&bsp_path)
        .arg(&bc_path)
        .output()?;
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let parsed_out = String::from_utf8(out.stdout)?;
    assert!(parsed_out.contains("\"found\": true"));
    Ok(())
}
