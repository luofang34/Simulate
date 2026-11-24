//! Unit tests for CCSDS NDM and SPICE kernel interchange wrappers.

use pyo3::prelude::*;

use super::{ccsds_adm, ccsds_odm, spice};

const OPM_SAMPLE: &str = "\
CCSDS_OPM_VERS = 3.0
CREATION_DATE = 2026-06-01T00:00:00
ORIGINATOR = SIMULATE
OBJECT_NAME = DEMO
OBJECT_ID = 2026-001A
CENTER_NAME = EARTH
REF_FRAME = ICRF
TIME_SYSTEM = UTC
EPOCH = 2026-06-01T12:00:00Z
X = 7000.0 [km]
Y = 0.0 [km]
Z = 0.0 [km]
X_DOT = 0.0 [km/s]
Y_DOT = 7.546 [km/s]
Z_DOT = 0.0 [km/s]
";

#[test]
fn opm_parse_and_si_state_extraction_succeed() {
    Python::attach(|py| {
        let parsed = ccsds_odm::parse_ccsds_opm(py, OPM_SAMPLE, true);
        assert!(parsed.is_ok());
        let state = ccsds_odm::ccsds_opm_to_state(py, OPM_SAMPLE, true);
        assert!(state.is_ok());
        if let Ok(obj) = state {
            let bound = obj.bind(py);
            let pos: [f64; 3] = bound.get_item("pos_m").unwrap().unwrap().extract().unwrap();
            assert!((pos[0] - 7_000_000.0).abs() < 1.0e-6);
        }
    });
}

#[test]
fn spice_spk_and_ck_build_inspect_evaluate_round_trip() {
    Python::attach(|py| {
        let spk_bytes = spice::build_hermite_spk(
            py,
            "PY-TEST-SPK",
            "SEG-1",
            -77,
            399,
            1,
            vec![0.0, 60.0, 120.0, 180.0],
            vec![
                [7000.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                [7060.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                [7120.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                [7180.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            ],
            2,
            Some(vec!["unit test".to_owned()]),
        )
        .unwrap();
        let raw = spk_bytes.as_bytes(py);
        assert!(spice::inspect_spice_spk(py, raw).is_ok());
        let (pos, _) = spice::evaluate_spice_spk(raw, -77, 399, 90.0).unwrap();
        assert!((pos[0] - 7090.0).abs() < 1.0e-9);

        let ck_bytes = spice::build_type3_ck(
            py,
            "PY-TEST-CK",
            "ATT-1",
            -77_000,
            1,
            vec![0.0, 100.0],
            vec![[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            Some(vec![[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            None,
        )
        .unwrap();
        let ck_raw = ck_bytes.as_bytes(py);
        assert!(spice::inspect_spice_ck(py, ck_raw).is_ok());
        assert!(spice::evaluate_spice_ck(py, ck_raw, -77_000, 50.0, 0.0).is_ok());
    });
}

#[test]
fn empty_ccsds_and_invalid_spice_fail_closed() {
    Python::attach(|py| {
        assert!(ccsds_odm::parse_ccsds_opm(py, "", true).is_err());
        assert!(ccsds_adm::parse_ccsds_tdm(py, "   ", true).is_err());
        assert!(spice::inspect_spice_spk(py, b"not-a-daf").is_err());
        assert!(spice::inspect_spice_ck(py, b"not-a-daf").is_err());
    });
}
