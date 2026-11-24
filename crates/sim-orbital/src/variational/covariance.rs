//! 6x6 Cartesian state covariance, discrete process noise, and `ICRF` <-> `RIC` frame transforms.

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::variational::VariationalError;

/// Discrete integrated 6x6 process-noise covariance $Q_k$ over $(\mathbf{r}_{\text{ICRF}}, \mathbf{v}_{\text{ICRF}})$
/// ($\text{m}^2$, $\text{m}^2/\text{s}$, $\text{m}^2/\text{s}^2$).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProcessNoiseQ {
    matrix: [[f64; 6]; 6],
}

impl ProcessNoiseQ {
    /// Zero process noise ($Q = 0$).
    pub const ZERO: Self = Self {
        matrix: [[0.0; 6]; 6],
    };

    /// Wraps a `6x6` row-major process-noise matrix.
    #[must_use]
    pub const fn from_matrix(matrix: [[f64; 6]; 6]) -> Self {
        Self { matrix }
    }

    /// Borrows the underlying `6x6` matrix.
    #[must_use]
    pub const fn as_matrix(&self) -> &[[f64; 6]; 6] {
        &self.matrix
    }

    /// Diagonal process-noise variances `[rx, ry, rz, vx, vy, vz]` ($\text{m}^2, \text{m}^2/\text{s}^2$).
    #[must_use]
    pub fn from_diagonal(variances: [f64; 6]) -> Self {
        Self {
            matrix: diag_6x6(variances),
        }
    }

    /// Continuous white-noise acceleration (`CWNA` / state-noise compensation) discrete
    /// covariance for isotropic acceleration spectral density $q_a$ ($\text{m}^2/\text{s}^3$)
    /// over interval $\Delta t \ge 0$ (`s`):
    /// $Q_{rr} = \frac{1}{3} q_a \Delta t^3 I_3$,
    /// $Q_{rv} = \frac{1}{2} q_a \Delta t^2 I_3$,
    /// $Q_{vv} = q_a \Delta t\, I_3$.
    #[must_use]
    pub fn from_accel_psd(q_accel_m2_s3: f64, dt_s: f64) -> Self {
        let dt = dt_s.max(0.0);
        let q = q_accel_m2_s3.max(0.0);
        let dt2 = dt * dt;
        let q_rr = q * dt2 * dt / 3.0;
        let q_rv = 0.5 * q * dt2;
        let q_vv = q * dt;
        Self {
            matrix: [
                [q_rr, 0.0, 0.0, q_rv, 0.0, 0.0],
                [0.0, q_rr, 0.0, 0.0, q_rv, 0.0],
                [0.0, 0.0, q_rr, 0.0, 0.0, q_rv],
                [q_rv, 0.0, 0.0, q_vv, 0.0, 0.0],
                [0.0, q_rv, 0.0, 0.0, q_vv, 0.0],
                [0.0, 0.0, q_rv, 0.0, 0.0, q_vv],
            ],
        }
    }

    /// Returns the symmetrized matrix $\frac{1}{2}(Q + Q^T)$.
    #[must_use]
    pub fn symmetrize(&self) -> Self {
        Self {
            matrix: symmetrize_6x6(&self.matrix),
        }
    }

    /// Checks whether all entries are finite, symmetric, and positive semidefinite.
    #[must_use]
    pub fn is_positive_semidefinite(&self) -> bool {
        is_psd_6x6(&self.matrix)
    }
}

/// 6x6 Cartesian state covariance matrix over $(\mathbf{r}, \mathbf{v})$ in `ICRF` or `RIC`
/// ($\text{m}^2$, $\text{m}^2/\text{s}$, $\text{m}^2/\text{s}^2$).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StateCovariance6 {
    matrix: [[f64; 6]; 6],
}

impl StateCovariance6 {
    /// Zero covariance matrix ($P = 0$).
    pub const ZERO: Self = Self {
        matrix: [[0.0; 6]; 6],
    };

    /// Constructs a [`StateCovariance6`] from a row-major `6x6` matrix.
    #[must_use]
    pub const fn from_matrix(matrix: [[f64; 6]; 6]) -> Self {
        Self { matrix }
    }

    /// Borrows the underlying `6x6` matrix.
    #[must_use]
    pub const fn as_matrix(&self) -> &[[f64; 6]; 6] {
        &self.matrix
    }

    /// Constructs a diagonal covariance matrix from state variances `[rx, ry, rz, vx, vy, vz]`.
    #[must_use]
    pub fn from_diagonal(variances: [f64; 6]) -> Self {
        Self {
            matrix: diag_6x6(variances),
        }
    }

    /// Constructs a diagonal covariance matrix from $1\sigma$ standard deviations `[sx, sy, sz, svx, svy, svz]`.
    #[must_use]
    pub fn from_diagonal_sigmas(sigmas: [f64; 6]) -> Self {
        Self::from_diagonal(sigmas.map(|s| s * s))
    }

    /// Returns the $1\sigma$ diagonal standard deviations $\sqrt{\max(P_{ii}, 0)}$.
    #[must_use]
    pub fn diagonal_sigmas(&self) -> [f64; 6] {
        core::array::from_fn(|i| {
            let v = self
                .matrix
                .get(i)
                .and_then(|r| r.get(i))
                .copied()
                .unwrap_or(0.0);
            if v.is_nan() {
                f64::NAN
            } else if v <= 0.0 {
                0.0
            } else {
                libm::sqrt(v)
            }
        })
    }

    /// Returns the symmetrized matrix $\frac{1}{2}(P + P^T)$.
    #[must_use]
    pub fn symmetrize(&self) -> Self {
        Self {
            matrix: symmetrize_6x6(&self.matrix),
        }
    }

    /// Checks whether this matrix has finite entries, is symmetric within roundoff, and is
    /// positive semidefinite across both position and velocity blocks.
    #[must_use]
    pub fn is_positive_semidefinite(&self) -> bool {
        is_psd_6x6(&self.matrix)
    }

    /// Propagates this covariance across a state transition matrix $\Phi(t_f, t_0)$ with optional
    /// discrete process noise $Q$: $P(t_f) = \Phi P_0 \Phi^T + Q$ (symmetrized).
    #[must_use]
    pub fn propagate(&self, stm: &[[f64; 6]; 6], process_noise: Option<&ProcessNoiseQ>) -> Self {
        let phi_p = matmul_6x6(stm, &self.matrix);
        let mut next = matmul_6x6_transpose_right(&phi_p, stm);
        if let Some(q) = process_noise {
            for (row_out, row_q) in next.iter_mut().zip(q.matrix.iter()) {
                for (dst, &q_val) in row_out.iter_mut().zip(row_q.iter()) {
                    *dst += q_val;
                }
            }
        }
        Self {
            matrix: symmetrize_6x6(&next),
        }
    }

    /// Extracts the symmetrized $3 \times 3$ position covariance sub-block ($\text{m}^2$).
    #[must_use]
    pub fn position_cov_m2(&self) -> DMat3 {
        block_3x3_dmat3(&self.matrix, 0, 0)
    }

    /// Rotates this `ICRF` covariance into the instantaneous local `RIC` (`R`, `I`, `C`) axes
    /// via block-diagonal $\operatorname{diag}(R_{\text{ICRF}\to\text{RIC}}, R_{\text{ICRF}\to\text{RIC}})$
    /// (`CCSDS RTN / RIC` convention).
    pub fn in_ric_frame(&self, r_icrf: DVec3, v_icrf: DVec3) -> Result<Self, VariationalError> {
        let rot = icrf_to_ric_rotation(r_icrf, v_icrf)?;
        Ok(Self {
            matrix: rotate_cov_6x6(&self.matrix, rot),
        })
    }

    /// Rotates a `RIC`-frame covariance back into `ICRF` axes via
    /// $\operatorname{diag}(R_{\text{ICRF}\to\text{RIC}}^T, R_{\text{ICRF}\to\text{RIC}}^T)$.
    pub fn from_ric_frame(
        cov_ric: &Self,
        r_icrf: DVec3,
        v_icrf: DVec3,
    ) -> Result<Self, VariationalError> {
        let rot = icrf_to_ric_rotation(r_icrf, v_icrf)?;
        Ok(Self {
            matrix: rotate_cov_6x6(&cov_ric.matrix, rot.transpose()),
        })
    }
}

/// Constructs the right-handed orthonormal rotation matrix $R_{\text{ICRF}\to\text{RIC}}$
/// such that $\mathbf{v}_{\text{RIC}} = R_{\text{ICRF}\to\text{RIC}} \mathbf{v}_{\text{ICRF}}$,
/// with axes:
/// - `R` (radial): $\hat{\mathbf{r}} = \mathbf{r} / \|\mathbf{r}\|$,
/// - `I` (in-track): $\hat{\mathbf{i}} = \hat{\mathbf{c}} \times \hat{\mathbf{r}}$,
/// - `C` (cross-track): $\hat{\mathbf{c}} = (\mathbf{r} \times \mathbf{v}) / \|\mathbf{r} \times \mathbf{v}\|$.
pub fn icrf_to_ric_rotation(r_icrf: DVec3, v_icrf: DVec3) -> Result<DMat3, VariationalError> {
    if !r_icrf.is_finite() || !v_icrf.is_finite() {
        return Err(VariationalError::NonFiniteState);
    }
    let r_sq = r_icrf.length_squared();
    if r_sq <= 0.0 {
        return Err(VariationalError::DegenerateRicFrame);
    }
    let r_hat = r_icrf / libm::sqrt(r_sq);
    let h = r_icrf.cross(v_icrf);
    let h_sq = h.length_squared();
    if !h_sq.is_finite() || h_sq <= 0.0 {
        return Err(VariationalError::DegenerateRicFrame);
    }
    let c_hat = h / libm::sqrt(h_sq);
    let i_raw = c_hat.cross(r_hat);
    let i_sq = i_raw.length_squared();
    if !i_sq.is_finite() || i_sq <= 0.0 {
        return Err(VariationalError::DegenerateRicFrame);
    }
    let i_hat = i_raw / libm::sqrt(i_sq);
    Ok(DMat3::from_cols(r_hat, i_hat, c_hat).transpose())
}

fn diag_6x6(d: [f64; 6]) -> [[f64; 6]; 6] {
    core::array::from_fn(|i| {
        let val = d.get(i).copied().unwrap_or(0.0);
        core::array::from_fn(|j| if i == j { val } else { 0.0 })
    })
}

fn symmetrize_6x6(m: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    core::array::from_fn(|i| {
        core::array::from_fn(|j| {
            let a = m.get(i).and_then(|r| r.get(j)).copied().unwrap_or(0.0);
            if i == j {
                a
            } else {
                let b = m.get(j).and_then(|r| r.get(i)).copied().unwrap_or(0.0);
                0.5 * (a + b)
            }
        })
    })
}

fn matmul_6x6(a: &[[f64; 6]; 6], b: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    core::array::from_fn(|i| {
        let row_a = a.get(i).copied().unwrap_or([0.0; 6]);
        core::array::from_fn(|j| {
            let mut sum = 0.0_f64;
            for (a_ik, row_b) in row_a.iter().zip(b.iter()) {
                sum += a_ik * row_b.get(j).copied().unwrap_or(0.0);
            }
            sum
        })
    })
}

fn matmul_6x6_transpose_right(a: &[[f64; 6]; 6], b: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    core::array::from_fn(|i| {
        let row_a = a.get(i).copied().unwrap_or([0.0; 6]);
        core::array::from_fn(|j| {
            let row_b = b.get(j).copied().unwrap_or([0.0; 6]);
            let mut sum = 0.0_f64;
            for (&a_ik, &b_jk) in row_a.iter().zip(row_b.iter()) {
                sum += a_ik * b_jk;
            }
            sum
        })
    })
}

fn block_3x3_dmat3(m: &[[f64; 6]; 6], row_off: usize, col_off: usize) -> DMat3 {
    let get = |r: usize, c: usize| -> f64 {
        let a = m
            .get(row_off + r)
            .and_then(|row| row.get(col_off + c))
            .copied()
            .unwrap_or(0.0);
        let b = m
            .get(col_off + c)
            .and_then(|row| row.get(row_off + r))
            .copied()
            .unwrap_or(0.0);
        if row_off == col_off && r != c {
            0.5 * (a + b)
        } else {
            a
        }
    };
    DMat3::from_cols(
        DVec3::new(get(0, 0), get(1, 0), get(2, 0)),
        DVec3::new(get(0, 1), get(1, 1), get(2, 1)),
        DVec3::new(get(0, 2), get(1, 2), get(2, 2)),
    )
}

fn dmat3_row_major(m: DMat3) -> [[f64; 3]; 3] {
    [
        [m.x_axis.x, m.y_axis.x, m.z_axis.x],
        [m.x_axis.y, m.y_axis.y, m.z_axis.y],
        [m.x_axis.z, m.y_axis.z, m.z_axis.z],
    ]
}

fn rotate_cov_6x6(m: &[[f64; 6]; 6], rot: DMat3) -> [[f64; 6]; 6] {
    let r = dmat3_row_major(rot);
    let mut t = [[0.0_f64; 6]; 6];
    let mut diag: usize = 0;
    while diag < 2 {
        let off = 3 * diag;
        for (ri, r_row) in r.iter().enumerate() {
            if let Some(dst_row) = t.get_mut(off + ri) {
                for (cj, &r_val) in r_row.iter().enumerate() {
                    if let Some(slot) = dst_row.get_mut(off + cj) {
                        *slot = r_val;
                    }
                }
            }
        }
        diag = diag.wrapping_add(1);
    }
    let tm = matmul_6x6(&t, m);
    symmetrize_6x6(&matmul_6x6_transpose_right(&tm, &t))
}

fn is_psd_6x6(m: &[[f64; 6]; 6]) -> bool {
    if m.iter().flatten().any(|v| !v.is_finite()) {
        return false;
    }
    let s = symmetrize_6x6(m);
    let diag = |idx: usize| -> f64 { s.get(idx).and_then(|r| r.get(idx)).copied().unwrap_or(0.0) };
    let scale_pos = diag(0).abs().max(diag(1).abs()).max(diag(2).abs());
    let scale_vel = diag(3).abs().max(diag(4).abs()).max(diag(5).abs());
    let d: [f64; 6] = core::array::from_fn(|i| {
        let val = diag(i);
        let blk_scale = if i < 3 { scale_pos } else { scale_vel };
        if val > 0.0 {
            libm::sqrt(val)
        } else if blk_scale > 0.0 {
            libm::sqrt(blk_scale)
        } else {
            1.0
        }
    });
    for i in 0..6 {
        let blk_scale = if i < 3 { scale_pos } else { scale_vel };
        let floor = if blk_scale > 0.0 {
            -1.0e-13 * blk_scale
        } else {
            0.0
        };
        if diag(i) < floor {
            return false;
        }
        let di = d.get(i).copied().unwrap_or(1.0);
        for j in (i + 1)..6 {
            let dj = d.get(j).copied().unwrap_or(1.0);
            let a = m.get(i).and_then(|r| r.get(j)).copied().unwrap_or(0.0);
            let b = m.get(j).and_then(|r| r.get(i)).copied().unwrap_or(0.0);
            let bound = 1.0e-11 * (di * dj).max(a.abs()).max(b.abs());
            if (a - b).abs() > bound {
                return false;
            }
        }
    }
    let mut c: [[f64; 6]; 6] = core::array::from_fn(|i| {
        let di = d.get(i).copied().unwrap_or(1.0);
        core::array::from_fn(|j| {
            let dj = d.get(j).copied().unwrap_or(1.0);
            s.get(i).and_then(|r| r.get(j)).copied().unwrap_or(0.0) / (di * dj)
        })
    });
    jacobi_min_eigenvalue_6x6(&mut c) >= -1.0e-11
}

fn jacobi_min_eigenvalue_6x6(c: &mut [[f64; 6]; 6]) -> f64 {
    let mut sweep = 0_u32;
    while sweep < 15 {
        for p in 0..5 {
            for q in (p + 1)..6 {
                jacobi_rotate_pair(c, p, q);
            }
        }
        sweep = sweep.wrapping_add(1);
    }
    let mut min_eig = f64::INFINITY;
    for (i, row) in c.iter().enumerate() {
        let eig = row.get(i).copied().unwrap_or(0.0);
        if eig < min_eig {
            min_eig = eig;
        }
    }
    min_eig
}

fn jacobi_rotate_pair(c: &mut [[f64; 6]; 6], p: usize, q: usize) {
    let get = |mat: &[[f64; 6]; 6], r: usize, col: usize| -> f64 {
        mat.get(r)
            .and_then(|row| row.get(col))
            .copied()
            .unwrap_or(0.0)
    };
    let set = |mat: &mut [[f64; 6]; 6], r: usize, col: usize, val: f64| {
        if let Some(slot) = mat.get_mut(r).and_then(|row| row.get_mut(col)) {
            *slot = val;
        }
    };
    let g = get(c, p, q);
    if g.abs() <= 1.0e-18 {
        return;
    }
    let c_pp = get(c, p, p);
    let c_qq = get(c, q, q);
    let phi = 0.5 * libm::atan2(g, 0.5 * (c_qq - c_pp));
    let (s, cs) = libm::sincos(phi);
    set(c, p, p, cs * cs * c_pp - 2.0 * s * cs * g + s * s * c_qq);
    set(c, q, q, s * s * c_pp + 2.0 * s * cs * g + cs * cs * c_qq);
    set(c, p, q, 0.0);
    set(c, q, p, 0.0);
    for k in 0..6 {
        if k == p || k == q {
            continue;
        }
        let c_kp = get(c, k, p);
        let c_kq = get(c, k, q);
        let new_kp = cs * c_kp - s * c_kq;
        let new_kq = s * c_kp + cs * c_kq;
        set(c, k, p, new_kp);
        set(c, p, k, new_kp);
        set(c, k, q, new_kq);
        set(c, q, k, new_kq);
    }
}
