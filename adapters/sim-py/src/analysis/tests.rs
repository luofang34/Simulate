//! Unit tests for H3 coverage and Monte Carlo bindings.

use pyo3::{prelude::*, types::PyList};

use super::{coverage, monte_carlo};

#[test]
fn monte_carlo_sample_is_deterministic_and_sensitive_to_seed() {
    Python::attach(|py| {
        let a =
            monte_carlo::monte_carlo_sample(py, 42, "cd_scale", 3, 8, None, None, None).unwrap();
        let b =
            monte_carlo::monte_carlo_sample(py, 42, "cd_scale", 3, 8, None, None, None).unwrap();
        let c =
            monte_carlo::monte_carlo_sample(py, 43, "cd_scale", 3, 8, None, None, None).unwrap();

        let sa: Vec<f64> = a
            .bind(py)
            .get_item("samples")
            .unwrap()
            .unwrap()
            .extract()
            .unwrap();
        let sb: Vec<f64> = b
            .bind(py)
            .get_item("samples")
            .unwrap()
            .unwrap()
            .extract()
            .unwrap();
        let sc: Vec<f64> = c
            .bind(py)
            .get_item("samples")
            .unwrap()
            .unwrap()
            .extract()
            .unwrap();
        assert_eq!(sa, sb);
        assert_ne!(sa, sc);

        let ha: String = a
            .bind(py)
            .get_item("manifest_sha256")
            .unwrap()
            .unwrap()
            .extract()
            .unwrap();
        let hb: String = b
            .bind(py)
            .get_item("manifest_sha256")
            .unwrap()
            .unwrap()
            .extract()
            .unwrap();
        let hc: String = c
            .bind(py)
            .get_item("manifest_sha256")
            .unwrap()
            .unwrap()
            .extract()
            .unwrap();
        assert_eq!(ha, hb);
        assert_eq!(ha.len(), 64);
        assert_ne!(ha, hc);
    });
}

#[test]
fn h3_coverage_rejects_partial_box_and_evaluates_leo_pass() {
    Python::attach(|py| {
        let r_m = 6_378_137.0 + 1_200_000.0;
        let v_m_s = (3.986_004_418e14_f64 / r_m).sqrt();
        let sat = ([r_m, 0.0, 0.0], [0.0, v_m_s, 0.0])
            .into_pyobject(py)
            .unwrap()
            .into_any();
        let bad = coverage::evaluate_h3_coverage(
            py,
            vec![sat.clone()],
            "2026-06-01T00:00:00Z",
            60.0,
            60.0,
            1,
            10.0,
            1,
            Some(-10.0),
            None,
            None,
            None,
            1000,
        );
        assert!(bad.is_err());

        let sats = PyList::new(py, [sat]).unwrap();
        let sat_vec = vec![sats.get_item(0).unwrap()];
        let ok = coverage::evaluate_h3_coverage(
            py,
            sat_vec,
            "2026-06-01T00:00:00Z",
            120.0,
            60.0,
            1,
            10.0,
            1,
            None,
            None,
            None,
            None,
            5_000,
        )
        .unwrap();
        let total: usize = ok
            .bind(py)
            .get_item("total_cells")
            .unwrap()
            .unwrap()
            .extract()
            .unwrap();
        assert_eq!(total, 842);
    });
}
