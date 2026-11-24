//! Pure-`libm` SPICE quaternion algebra, SLERP, and C-matrix conversions.
//!
//! # Conventions
//!
//! SPICE quaternions are stored scalar-first as `[q0, q1, q2, q3] = [w, x, y, z]`
//! and represent the **C-matrix** $C = R_{\text{ref}\to\text{inst}}$ satisfying
//! $\mathbf{v}_{\text{inst}} = C \, \mathbf{v}_{\text{ref}}$ (`Q2M` / `M2Q`).
//!
//! When an instrument/body frame is rotated from `ref` by a right-handed angle
//! $\theta$ about unit axis $\hat{u}$ (so the Hamilton active body-to-reference
//! quaternion is $(\cos(\theta/2), +\sin(\theta/2)\hat{u})$), the inverse C-matrix
//! quaternion stored in a CK file is its conjugate:
//! `q_ck = [cos(theta/2), -sin(theta/2) * u_x, -sin(theta/2) * u_y, -sin(theta/2) * u_z]`.

/// Normalizes a SPICE quaternion `[q0, q1, q2, q3]`. Returns `None` if zero or non-finite.
#[must_use]
pub(super) fn normalize_quaternion(q: [f64; 4]) -> Option<[f64; 4]> {
    if q.iter().any(|x| !x.is_finite()) {
        return None;
    }
    let [q0, q1, q2, q3] = q;
    let norm_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if norm_sq <= 0.0 || !norm_sq.is_finite() {
        return None;
    }
    let inv = 1.0 / libm::sqrt(norm_sq);
    Some([q0 * inv, q1 * inv, q2 * inv, q3 * inv])
}

/// Hamilton product of two scalar-first SPICE quaternions (`Q2M(a * b) == Q2M(a) * Q2M(b)`).
#[must_use]
pub(super) fn quaternion_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [a0, a1, a2, a3] = a;
    let [b0, b1, b2, b3] = b;
    [
        a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3,
        a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2,
        a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1,
        a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0,
    ]
}

/// Shortest-arc spherical linear interpolation (SLERP) between two unit SPICE
/// quaternions at fraction `w` in `[0, 1]`.
#[must_use]
pub(super) fn quaternion_slerp(q1: [f64; 4], q2: [f64; 4], w: f64) -> [f64; 4] {
    let [a0, a1, a2, a3] = q1;
    let [b0, b1, b2, b3] = q2;
    let mut dot = a0 * b0 + a1 * b1 + a2 * b2 + a3 * b3;
    let [c0, c1, c2, c3] = if dot < 0.0 {
        dot = -dot;
        [-b0, -b1, -b2, -b3]
    } else {
        [b0, b1, b2, b3]
    };
    let dot = dot.clamp(-1.0, 1.0);
    if dot > 1.0 - 1.0e-12 {
        let raw = [
            a0 + w * (c0 - a0),
            a1 + w * (c1 - a1),
            a2 + w * (c2 - a2),
            a3 + w * (c3 - a3),
        ];
        return normalize_quaternion(raw).unwrap_or(q1);
    }
    let half_angle = libm::acos(dot);
    let sin_half = libm::sin(half_angle);
    let s1 = libm::sin((1.0 - w) * half_angle) / sin_half;
    let s2 = libm::sin(w * half_angle) / sin_half;
    let raw = [
        s1 * a0 + s2 * c0,
        s1 * a1 + s2 * c1,
        s1 * a2 + s2 * c2,
        s1 * a3 + s2 * c3,
    ];
    normalize_quaternion(raw).unwrap_or(q1)
}

/// Propagates a SPICE CK quaternion `q0` forward by `dt_s` seconds under constant
/// reference-frame angular velocity `av_ref` (`rad/s`).
#[must_use]
pub(super) fn propagate_constant_av(q0: [f64; 4], av_ref: [f64; 3], dt_s: f64) -> [f64; 4] {
    let [wx, wy, wz] = av_ref;
    let w2 = wx * wx + wy * wy + wz * wz;
    let w_norm = libm::sqrt(w2);
    let half_theta = 0.5 * w_norm * libm::fabs(dt_s);
    let (cos_ht, sin_coeff) = if half_theta < 1.0e-6 {
        let ht2 = half_theta * half_theta;
        (1.0 - 0.5 * ht2, -0.5 * dt_s * (1.0 - ht2 / 6.0))
    } else {
        let signed_ht = 0.5 * w_norm * dt_s;
        (libm::cos(signed_ht), -libm::sin(signed_ht) / w_norm)
    };
    let q_delta = [cos_ht, sin_coeff * wx, sin_coeff * wy, sin_coeff * wz];
    let q_out = quaternion_mul(q0, q_delta);
    normalize_quaternion(q_out).unwrap_or(q0)
}

/// Converts a unit SPICE scalar-first quaternion `[q0, q1, q2, q3]` to its 3x3
/// row-major C-matrix `M` (`Q2M`, satisfying `v_inst = M * v_ref`).
#[must_use]
pub fn quaternion_to_matrix(q: [f64; 4]) -> [[f64; 3]; 3] {
    let [q0, q1, q2, q3] = normalize_quaternion(q).unwrap_or([1.0, 0.0, 0.0, 0.0]);
    let (q01, q02, q03) = (q0 * q1, q0 * q2, q0 * q3);
    let (q11, q12, q13) = (q1 * q1, q1 * q2, q1 * q3);
    let (q22, q23, q33) = (q2 * q2, q2 * q3, q3 * q3);
    [
        [
            1.0 - 2.0 * (q22 + q33),
            2.0 * (q12 - q03),
            2.0 * (q13 + q02),
        ],
        [
            2.0 * (q12 + q03),
            1.0 - 2.0 * (q11 + q33),
            2.0 * (q23 - q01),
        ],
        [
            2.0 * (q13 - q02),
            2.0 * (q23 + q01),
            1.0 - 2.0 * (q11 + q22),
        ],
    ]
}

/// Converts a 3x3 row-major orthonormal C-matrix `M` (`v_inst = M * v_ref`) to a
/// unit SPICE scalar-first quaternion `[q0, q1, q2, q3]` with `q0 >= 0` (`M2Q`).
#[must_use]
pub fn matrix_to_quaternion(m: [[f64; 3]; 3]) -> [f64; 4] {
    let [[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]] = m;
    let trace = m00 + m11 + m22;
    let raw = if trace > 0.0 {
        let s = 2.0 * libm::sqrt(1.0 + trace);
        [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]
    } else if m00 >= m11 && m00 >= m22 {
        let s = 2.0 * libm::sqrt((1.0 + m00 - m11 - m22).max(0.0));
        [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]
    } else if m11 >= m22 {
        let s = 2.0 * libm::sqrt((1.0 + m11 - m00 - m22).max(0.0));
        [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]
    } else {
        let s = 2.0 * libm::sqrt((1.0 + m22 - m00 - m11).max(0.0));
        [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]
    };
    let [u0, u1, u2, u3] = normalize_quaternion(raw).unwrap_or([1.0, 0.0, 0.0, 0.0]);
    if u0 < 0.0 {
        [-u0, -u1, -u2, -u3]
    } else {
        [u0, u1, u2, u3]
    }
}
