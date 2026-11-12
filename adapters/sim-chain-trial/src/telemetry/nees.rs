//! Exact 2D/3D Normalized Estimation Error Squared (`NEES`), uncertainty
//! radii (`EPU_95`, `VPU_95`), and Wilson–Hilferty chi-square consistency bands.

use glam::DVec3;
use navigate_contract::{GeodeticPosition, NavigationSolution, SymmetricCov3};
use navigate_geodesy::{BodyEllipsoid, LocalTangentPlane};
use sim_core::VehicleState;
use sim_geo::{LocalFrame, ecef_to_geodetic};

use super::types::NeesConsistencySummary;

/// `sqrt(chi^2_{0.95}(df = 2)) = sqrt(5.991464547107979)`.
const CHI2_2D_95_SCALE: f64 = 2.447_746_830_680_816;
/// `Phi^{-1}(0.975)` for two-sided `95%` Gaussian / Wilson–Hilferty intervals.
const Z_95_TWO_SIDED: f64 = 1.959_963_984_540_054;

/// Computed estimation error, uncertainties, and NEES for one solution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ScoredSolution {
    pub(super) error_enu_m: [f64; 3],
    pub(super) horizontal_error_m: f64,
    pub(super) vertical_error_m: f64,
    pub(super) velocity_error_mps: f64,
    pub(super) sigma_east_m: f64,
    pub(super) sigma_north_m: f64,
    pub(super) sigma_up_m: f64,
    pub(super) sigma_horiz_m: f64,
    pub(super) epu_95_m: f64,
    pub(super) vpu_95_m: f64,
    pub(super) nees_horiz_2d: Option<f64>,
    pub(super) nees_pos_3d: Option<f64>,
    pub(super) calibration_ratio: Option<f64>,
}

/// Scores a [`NavigationSolution`] against exact simulation `truth`.
pub(super) fn score_solution(
    solution: &NavigationSolution,
    truth: &VehicleState,
) -> ScoredSolution {
    let truth_geo = ecef_to_geodetic(truth.position_ecef);
    let truth_frame = LocalFrame::at(&truth_geo);
    let truth_vel_ned = truth_frame.ecef_to_ned_vector(truth.velocity_ecef);
    let est_vel_ned = DVec3::new(
        solution.velocity.north_mps,
        solution.velocity.east_mps,
        solution.velocity.down_mps,
    );
    let velocity_error_mps = (est_vel_ned - truth_vel_ned).length();

    // Tangent-plane horizontal offset of truth from estimate, negated so
    // `error = estimate - truth`, with ellipsoidal height difference for up/down
    // so Earth curvature d^2/(2R) never leaks into vertical error.
    let (err_north_m, err_east_m) =
        match LocalTangentPlane::new(solution.position, BodyEllipsoid::WGS84) {
            Ok(plane) => {
                let truth_gp =
                    GeodeticPosition::new(truth_geo.lat_rad, truth_geo.lon_rad, truth_geo.height_m);
                let offset = plane.to_ned(&truth_gp);
                (-offset.north_m, -offset.east_m)
            }
            Err(_) => {
                let est_ecef = sim_navigate::solution_position_ecef(solution);
                let ned = truth_frame.ecef_to_ned_vector(est_ecef - truth.position_ecef);
                (ned.x, ned.y)
            }
        };
    let err_up_m = solution.position.altitude_m - truth_geo.height_m;
    let err_down_m = -err_up_m;
    let error_enu_m = [err_east_m, err_north_m, err_up_m];
    let horizontal_error_m = err_east_m.hypot(err_north_m);
    let vertical_error_m = err_up_m.abs();

    let [p_nn, p_ne, _, p_ee, _, p_dd] = solution.position_cov.upper_triangle();
    let sigma_north_m = p_nn.max(0.0).sqrt();
    let sigma_east_m = p_ee.max(0.0).sqrt();
    let sigma_up_m = p_dd.max(0.0).sqrt();
    let sigma_horiz_m = (p_nn.max(0.0) + p_ee.max(0.0)).sqrt();
    let epu_95_m = CHI2_2D_95_SCALE * max_eigenvalue_2d(p_nn, p_ne, p_ee).sqrt();
    let vpu_95_m = Z_95_TWO_SIDED * sigma_up_m;
    let nees_horiz_2d = nees_2d((err_north_m, err_east_m), (p_nn, p_ne, p_ee));
    let nees_pos_3d = nees_3d(
        [err_north_m, err_east_m, err_down_m],
        &solution.position_cov,
    );
    let calibration_ratio = (sigma_horiz_m > 1e-12).then_some(horizontal_error_m / sigma_horiz_m);

    ScoredSolution {
        error_enu_m,
        horizontal_error_m,
        vertical_error_m,
        velocity_error_mps,
        sigma_east_m,
        sigma_north_m,
        sigma_up_m,
        sigma_horiz_m,
        epu_95_m,
        vpu_95_m,
        nees_horiz_2d,
        nees_pos_3d,
        calibration_ratio,
    }
}

fn max_eigenvalue_2d(p_nn: f64, p_ne: f64, p_ee: f64) -> f64 {
    let trace = p_nn + p_ee;
    let diff = p_nn - p_ee;
    let disc = (0.25 * diff * diff + p_ne * p_ne).max(0.0).sqrt();
    (0.5 * trace + disc).max(0.0)
}

fn nees_2d((e_n, e_e): (f64, f64), (p_nn, p_ne, p_ee): (f64, f64, f64)) -> Option<f64> {
    if !(p_nn.is_finite() && p_ne.is_finite() && p_ee.is_finite() && p_nn > 0.0 && p_ee > 0.0) {
        return None;
    }
    let det = p_nn * p_ee - p_ne * p_ne;
    if !det.is_finite() || det <= 1e-18 * p_nn * p_ee {
        return None;
    }
    let quad = (p_ee * e_n * e_n - 2.0 * p_ne * e_n * e_e + p_nn * e_e * e_e) / det;
    (quad.is_finite() && quad >= 0.0).then_some(quad)
}

fn nees_3d([e_n, e_e, e_d]: [f64; 3], cov: &SymmetricCov3) -> Option<f64> {
    let [p_nn, p_ne, p_nd, p_ee, p_ed, p_dd] = cov.upper_triangle();
    if !(p_nn > 0.0 && p_nn.is_finite()) {
        return None;
    }
    let l00 = p_nn.sqrt();
    let l10 = p_ne / l00;
    let d1 = p_ee - l10 * l10;
    if !(d1 > 0.0 && d1.is_finite()) {
        return None;
    }
    let l11 = d1.sqrt();
    let l20 = p_nd / l00;
    let l21 = (p_ed - l20 * l10) / l11;
    let d2 = p_dd - l20 * l20 - l21 * l21;
    if !(d2 > 0.0 && d2.is_finite()) {
        return None;
    }
    let l22 = d2.sqrt();
    let y0 = e_n / l00;
    let y1 = (e_e - l10 * y0) / l11;
    let y2 = (e_d - l20 * y0 - l21 * y1) / l22;
    let quad = y0 * y0 + y1 * y1 + y2 * y2;
    (quad.is_finite() && quad >= 0.0).then_some(quad)
}

/// Summarizes a sequence of NEES values of dimension `dof` with its two-sided
/// `95%` sample-mean chi-square consistency interval.
pub(super) fn summarize_nees(
    values: &[f64],
    dof: usize,
    p95: f64,
) -> Option<NeesConsistencySummary> {
    if values.is_empty() || dof == 0 {
        return None;
    }
    let samples = values.len();
    let mean = values.iter().copied().sum::<f64>() / samples as f64;
    let total_dof = (samples.saturating_mul(dof)) as f64;
    // Wilson–Hilferty cube-root normal approximation for chi-square quantiles:
    // Q_p(k) ≈ k * (1 - 2/(9k) + z_p * sqrt(2/(9k)))^3.
    let term = 2.0 / (9.0 * total_dof);
    let root = term.sqrt();
    let cube = |x: f64| x.max(0.0).powi(3);
    let chi2_lower_95 = (total_dof * cube(1.0 - term - Z_95_TWO_SIDED * root)) / samples as f64;
    let chi2_upper_95 = (total_dof * cube(1.0 - term + Z_95_TWO_SIDED * root)) / samples as f64;
    Some(NeesConsistencySummary {
        dof,
        samples,
        mean,
        p95,
        chi2_lower_95,
        chi2_upper_95,
        within_95_band: mean >= chi2_lower_95 && mean <= chi2_upper_95,
    })
}
