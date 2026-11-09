//! Portable quaternion kinematics, rotation, geodesic angle, and vector norm clamping.

use glam::{DQuat, DVec3};

use crate::scalar_math::{atan2, sqrt};

/// Clamps vector Euclidean norm to `max_norm >= 0` using portable `sqrt`.
#[must_use]
pub fn clamp_vec_norm(v: DVec3, max_norm: f64) -> DVec3 {
    if max_norm <= 0.0 {
        return DVec3::ZERO;
    }
    let n_sq = v.length_squared();
    if n_sq <= max_norm * max_norm {
        v
    } else {
        v * (max_norm / sqrt(n_sq))
    }
}

/// Preserves exact bit patterns of already-normalized wire quaternions (`|n^2 - 1| <= 1e-11`)
/// while normalizing untrusted inputs so serde round-trips are 0-ulp lossless.
#[inline]
#[must_use]
pub fn quat_from_wire_xyzw(xyzw: [f64; 4]) -> DQuat {
    let q = DQuat::from_array(xyzw);
    let n_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if n_sq.is_finite() && (n_sq - 1.0).abs() <= 1.0e-11 {
        q
    } else {
        quat_normalize_portable(q)
    }
}

/// Portable unit-quaternion normalization (falls back to identity if degenerate).
#[inline]
#[must_use]
pub fn quat_normalize_portable(q: DQuat) -> DQuat {
    let n_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if n_sq > 0.0 && n_sq.is_finite() {
        let inv = 1.0 / sqrt(n_sq);
        DQuat::from_xyzw(q.x * inv, q.y * inv, q.z * inv, q.w * inv)
    } else {
        DQuat::IDENTITY
    }
}

/// Portable quaternion vector rotation $\mathbf{v}' = q \otimes \mathbf{v} \otimes q^*$ for unit `q`.
#[inline]
#[must_use]
pub fn quat_rotate_vec_portable(q_unit: DQuat, v: DVec3) -> DVec3 {
    let qv = DVec3::new(q_unit.x, q_unit.y, q_unit.z);
    let t = 2.0 * qv.cross(v);
    v + q_unit.w * t + qv.cross(t)
}

/// Exact quaternion kinematic derivative $\dot{q}_{\text{B}\to\text{I}} = \frac{1}{2} q_{\text{B}\to\text{I}} \otimes (\boldsymbol{\omega}_{\text{body}}, 0)$.
#[inline]
#[must_use]
pub fn quat_omega_derivative(q: DQuat, omega_body: DVec3) -> DQuat {
    let (wx, wy, wz) = (omega_body.x, omega_body.y, omega_body.z);
    DQuat::from_xyzw(
        0.5 * (q.w * wx + q.y * wz - q.z * wy),
        0.5 * (q.w * wy + q.z * wx - q.x * wz),
        0.5 * (q.w * wz + q.x * wy - q.y * wx),
        -0.5 * (q.x * wx + q.y * wy + q.z * wz),
    )
}

/// Numerically well-conditioned $SO(3)$ geodesic angle $2\arccos(|\langle \hat{q}_a, \hat{q}_b\rangle|) \in [0, \pi]$
/// evaluated via $4\operatorname{atan2}(\|\hat{q}_a - s\hat{q}_b\|, \|\hat{q}_a + s\hat{q}_b\|)$ with zero precision loss near $0$.
#[inline]
#[must_use]
pub fn quat_geodesic_angle_rad(qa: DQuat, qb: DQuat) -> f64 {
    let a = quat_normalize_portable(qa);
    let b = quat_normalize_portable(qb);
    let dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    let s = if dot >= 0.0 { 1.0 } else { -1.0 };
    let (dx, dy, dz, dw) = (a.x - s * b.x, a.y - s * b.y, a.z - s * b.z, a.w - s * b.w);
    let (sx, sy, sz, sw) = (a.x + s * b.x, a.y + s * b.y, a.z + s * b.z, a.w + s * b.w);
    let diff = sqrt(dx * dx + dy * dy + dz * dz + dw * dw);
    let sum = sqrt(sx * sx + sy * sy + sz * sz + sw * sw);
    4.0 * atan2(diff, sum)
}

/// Lossless `[f64; 3]` serde adapter for `glam::DVec3`.
pub mod serde_dvec3 {
    use glam::DVec3;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    /// Serializes `DVec3` as `[f64; 3]`.
    pub fn serialize<S: Serializer>(v: &DVec3, s: S) -> Result<S::Ok, S::Error> {
        v.to_array().serialize(s)
    }

    /// Deserializes `DVec3` from `[f64; 3]`.
    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<DVec3, D::Error> {
        <[f64; 3]>::deserialize(d).map(DVec3::from_array)
    }
}

/// Lossless `[f64; 4]` (`[x, y, z, w]`) serde adapter for `glam::DQuat`.
pub mod serde_dquat {
    use glam::DQuat;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    use super::quat_from_wire_xyzw;

    /// Serializes `DQuat` as `[f64; 4]`.
    pub fn serialize<S: Serializer>(q: &DQuat, s: S) -> Result<S::Ok, S::Error> {
        q.to_array().serialize(s)
    }

    /// Deserializes `DQuat` from `[f64; 4]`.
    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<DQuat, D::Error> {
        <[f64; 4]>::deserialize(d).map(quat_from_wire_xyzw)
    }
}
