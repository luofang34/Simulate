//! Minimal `APAS-95` androgynous docking capture, rigid assembly merge, and
//! spring separation conserving linear momentum $\mathbf{P}$ and total inertial
//! angular momentum $\mathbf{H}_{\text{ICRF}}$ to machine precision.

use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    ids::VesselId,
    plant::{
        ContinuousPlantState, MassPropertiesModel, quat_normalize_portable,
        quat_rotate_vec_portable,
    },
    scalar_math::sqrt,
};

/// Default `APAS-95` soft-capture radial alignment tolerance (`m`).
pub const APAS95_MAX_RADIAL_OFFSET_M: f64 = 0.15;
/// Default `APAS-95` axial capture window (`m`).
pub const APAS95_MAX_AXIAL_GAP_M: f64 = 0.12;
/// Default `APAS-95` maximum relative capture speed (`m/s`).
pub const APAS95_MAX_CLOSING_SPEED_M_S: f64 = 0.35;
/// Default `APAS-95` maximum angular misalignment (`rad`, `~5 deg`).
pub const APAS95_MAX_ALIGNMENT_ANGLE_RAD: f64 = 0.087_266_462_599_716_47;
/// Default `APAS-95` maximum relative angular rate (`rad/s`, `~1.5 deg/s`).
pub const APAS95_MAX_REL_OMEGA_RAD_S: f64 = 0.026_179_938_779_914_94;

/// Errors returned when attempting `APAS-95` capture or separation.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum DockingError {
    /// Vessels do not share the same primary coordinate origin.
    #[error("docking vessels have mismatched primary bodies ({0:?} vs {1:?})")]
    MismatchedPrimaryBody(crate::ids::NaifId, crate::ids::NaifId),
    /// Axial gap outside `[0, max_axial_gap_m]`.
    #[error("APAS-95 axial gap {gap_m:.4} m outside capture window [0, {max_m:.4}] m")]
    AxialGapOutOfRange {
        /// Measured axial separation (`m`).
        gap_m: f64,
        /// Allowed capture window (`m`).
        max_m: f64,
    },
    /// Radial offset exceeds capture ring tolerance.
    #[error("APAS-95 radial offset {radial_m:.4} m exceeds {max_m:.4} m")]
    RadialMisalignment {
        /// Measured lateral offset (`m`).
        radial_m: f64,
        /// Allowed tolerance (`m`).
        max_m: f64,
    },
    /// Relative velocity exceeds capture latch structural rating.
    #[error("APAS-95 relative speed {speed_m_s:.4} m/s exceeds {max_m_s:.4} m/s")]
    ExcessiveRelativeSpeed {
        /// Measured port relative speed (`m/s`).
        speed_m_s: f64,
        /// Allowed limit (`m/s`).
        max_m_s: f64,
    },
    /// Port outward normals fail antiparallel alignment (`n_A · n_B > -cos(theta_max)`).
    #[error("APAS-95 angular misalignment {angle_rad:.5} rad exceeds {max_rad:.5} rad")]
    AngularMisalignment {
        /// Measured antiparallel error (`rad`).
        angle_rad: f64,
        /// Allowed limit (`rad`).
        max_rad: f64,
    },
    /// Relative angular velocity exceeds capture damper rating.
    #[error("APAS-95 relative angular rate {rate_rad_s:.5} rad/s exceeds {max_rad_s:.5} rad/s")]
    ExcessiveAngularRate {
        /// Measured relative rate (`rad/s`).
        rate_rad_s: f64,
        /// Allowed limit (`rad/s`).
        max_rad_s: f64,
    },
}

/// Body-frame geometry and capture envelope of an `APAS-95` androgynous docking port.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Apas95PortSpec {
    /// Port ring center offset from the vessel CG in body coordinates (`m`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub pos_body_m: DVec3,
    /// Unit outward docking axis in body coordinates (e.g. `+Y` at `/Rocket/APAS95Reference`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub outward_axis_body: DVec3,
    /// Maximum radial offset for soft capture (`m`).
    pub max_radial_offset_m: f64,
    /// Maximum axial gap for soft capture (`m`).
    pub max_axial_gap_m: f64,
    /// Maximum port-to-port relative speed (`m/s`).
    pub max_closing_speed_m_s: f64,
    /// Maximum angular misalignment between `n_A` and `-n_B` (`rad`).
    pub max_alignment_angle_rad: f64,
    /// Maximum relative angular speed (`rad/s`).
    pub max_rel_omega_rad_s: f64,
}

impl Apas95PortSpec {
    /// Standard `APAS-95` port mounted at `pos_body_m` facing along `outward_axis_body`.
    #[must_use]
    pub fn standard(pos_body_m: DVec3, outward_axis_body: DVec3) -> Self {
        let n = sqrt(outward_axis_body.length_squared());
        let axis = if n > 0.0 {
            outward_axis_body / n
        } else {
            DVec3::Y
        };
        Self {
            pos_body_m,
            outward_axis_body: axis,
            max_radial_offset_m: APAS95_MAX_RADIAL_OFFSET_M,
            max_axial_gap_m: APAS95_MAX_AXIAL_GAP_M,
            max_closing_speed_m_s: APAS95_MAX_CLOSING_SPEED_M_S,
            max_alignment_angle_rad: APAS95_MAX_ALIGNMENT_ANGLE_RAD,
            max_rel_omega_rad_s: APAS95_MAX_REL_OMEGA_RAD_S,
        }
    }

    /// Nose-mounted `Raketa` `APAS-95` port (`/Rocket/APAS95Reference` at `y = 10.68 m` above hull base, `+Y` axis).
    #[must_use]
    pub fn raketa_nose_port(y_cg_m: f64) -> Self {
        Self::standard(DVec3::new(0.0, 10.68 - y_cg_m, 0.0), DVec3::Y)
    }
}

/// Reversible constituent record enabling lossless undocking of a merged assembly.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DockedAssemblyRecord {
    /// Host / primary constituent `VesselId`.
    pub vessel_a: VesselId,
    /// Captured / secondary constituent `VesselId`.
    pub vessel_b: VesselId,
    /// Original mass model of constituent A.
    pub mass_a: MassPropertiesModel,
    /// Original propellant mass of constituent A at merge (`kg`).
    pub prop_a_kg: f64,
    /// Original wheel momentum of constituent A in A-body axes (`N*m*s`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub h_rw_a_body_nms: DVec3,
    /// Original mass model of constituent B.
    pub mass_b: MassPropertiesModel,
    /// Original propellant mass of constituent B at merge (`kg`).
    pub prop_b_kg: f64,
    /// Original wheel momentum of constituent B in B-body axes (`N*m*s`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub h_rw_b_body_nms: DVec3,
    /// Fixed offset $\mathbf{r}_A - \mathbf{r}_{\text{cg}}$ in merged (A-body) axes (`m`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub offset_a_merged_m: DVec3,
    /// Fixed offset $\mathbf{r}_B - \mathbf{r}_{\text{cg}}$ in merged (A-body) axes (`m`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub offset_b_merged_m: DVec3,
    /// Fixed relative orientation rotating B-body vectors into merged (A-body) axes.
    #[serde(with = "crate::plant::attitude_math::serde_dquat")]
    pub q_b_to_merged: DQuat,
    /// Docking interface ring midpoint in merged (A-body) axes (`m`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub port_point_merged_m: DVec3,
    /// Unit outward separation normal (from A toward B) in merged (A-body) axes.
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub sep_axis_merged: DVec3,
}

/// Evaluates total linear momentum $\mathbf{P}$ (`kg*m/s`) and total inertial angular momentum
/// $\mathbf{H}_{0,\text{ICRF}} = \mathbf{r}\times m\mathbf{v} + R_{\text{B}\to\text{I}}(\mathbf{I}\boldsymbol{\omega} + \mathbf{h}_{\text{rw}})$
/// about the primary `ICRF` origin for one vessel.
#[must_use]
pub fn vessel_momenta_icrf(
    state: &ContinuousPlantState,
    mass_model: &MassPropertiesModel,
) -> (DVec3, DVec3) {
    let m = mass_model.total_mass_kg(state.propellant_mass_kg);
    let p_lin = m * state.vel_rel_icrf_m_s;
    let h_spin_icrf = mass_model.total_angular_momentum_icrf(state);
    let h_orb_icrf = state.pos_rel_icrf_m.cross(p_lin);
    (p_lin, h_orb_icrf + h_spin_icrf)
}

/// Validates whether two vessels with `APAS-95` ports satisfy soft-capture alignment and rate gates.
pub fn check_apas95_capture_gate(
    state_a: &ContinuousPlantState,
    port_a: &Apas95PortSpec,
    state_b: &ContinuousPlantState,
    port_b: &Apas95PortSpec,
) -> Result<(), DockingError> {
    if state_a.primary_body != state_b.primary_body {
        return Err(DockingError::MismatchedPrimaryBody(
            state_a.primary_body,
            state_b.primary_body,
        ));
    }
    let pa = state_a.pos_rel_icrf_m
        + quat_rotate_vec_portable(state_a.q_body_to_icrf, port_a.pos_body_m);
    let pb = state_b.pos_rel_icrf_m
        + quat_rotate_vec_portable(state_b.q_body_to_icrf, port_b.pos_body_m);
    let na = quat_rotate_vec_portable(state_a.q_body_to_icrf, port_a.outward_axis_body);
    let nb = quat_rotate_vec_portable(state_b.q_body_to_icrf, port_b.outward_axis_body);

    let delta = pb - pa;
    let axial_gap = delta.dot(na);
    let max_ax = port_a.max_axial_gap_m.min(port_b.max_axial_gap_m);
    if axial_gap < -0.02 || axial_gap > max_ax {
        return Err(DockingError::AxialGapOutOfRange {
            gap_m: axial_gap,
            max_m: max_ax,
        });
    }
    let radial = sqrt((delta - axial_gap * na).length_squared());
    let max_rad = port_a.max_radial_offset_m.min(port_b.max_radial_offset_m);
    if radial > max_rad {
        return Err(DockingError::RadialMisalignment {
            radial_m: radial,
            max_m: max_rad,
        });
    }
    check_port_rates_and_alignment(state_a, port_a, na, state_b, port_b, nb)
}

fn check_port_rates_and_alignment(
    state_a: &ContinuousPlantState,
    port_a: &Apas95PortSpec,
    na: DVec3,
    state_b: &ContinuousPlantState,
    port_b: &Apas95PortSpec,
    nb: DVec3,
) -> Result<(), DockingError> {
    let va = state_a.vel_rel_icrf_m_s
        + quat_rotate_vec_portable(
            state_a.q_body_to_icrf,
            state_a.omega_body_rad_s.cross(port_a.pos_body_m),
        );
    let vb = state_b.vel_rel_icrf_m_s
        + quat_rotate_vec_portable(
            state_b.q_body_to_icrf,
            state_b.omega_body_rad_s.cross(port_b.pos_body_m),
        );
    let rel_speed = sqrt((vb - va).length_squared());
    let max_spd = port_a
        .max_closing_speed_m_s
        .min(port_b.max_closing_speed_m_s);
    if rel_speed > max_spd {
        return Err(DockingError::ExcessiveRelativeSpeed {
            speed_m_s: rel_speed,
            max_m_s: max_spd,
        });
    }
    let cos_anti = (-na.dot(nb)).clamp(-1.0, 1.0);
    let ang_err = sqrt((2.0 * (1.0 - cos_anti)).max(0.0));
    let max_ang = port_a
        .max_alignment_angle_rad
        .min(port_b.max_alignment_angle_rad);
    if ang_err > max_ang {
        return Err(DockingError::AngularMisalignment {
            angle_rad: ang_err,
            max_rad: max_ang,
        });
    }
    let wa_icrf = quat_rotate_vec_portable(state_a.q_body_to_icrf, state_a.omega_body_rad_s);
    let wb_icrf = quat_rotate_vec_portable(state_b.q_body_to_icrf, state_b.omega_body_rad_s);
    let d_om = sqrt((wb_icrf - wa_icrf).length_squared());
    let max_om = port_a.max_rel_omega_rad_s.min(port_b.max_rel_omega_rad_s);
    if d_om > max_om {
        return Err(DockingError::ExcessiveAngularRate {
            rate_rad_s: d_om,
            max_rad_s: max_om,
        });
    }
    Ok(())
}

/// Validates `APAS-95` capture gates and merges `(A, B)` into a single rigid composite vessel
/// conserving linear momentum $\mathbf{P}$ and inertial angular momentum $\mathbf{H}_{0,\text{ICRF}}$
/// to machine precision (`< 1e-14` relative).
#[allow(clippy::too_many_arguments)]
pub fn dock_vessels_conserving_momentum(
    vessel_a: VesselId,
    state_a: &ContinuousPlantState,
    mass_a: &MassPropertiesModel,
    port_a: &Apas95PortSpec,
    vessel_b: VesselId,
    state_b: &ContinuousPlantState,
    mass_b: &MassPropertiesModel,
    port_b: &Apas95PortSpec,
) -> Result<
    (
        ContinuousPlantState,
        MassPropertiesModel,
        DockedAssemblyRecord,
    ),
    DockingError,
> {
    check_apas95_capture_gate(state_a, port_a, state_b, port_b)?;

    let ma = mass_a.total_mass_kg(state_a.propellant_mass_kg);
    let mb = mass_b.total_mass_kg(state_b.propellant_mass_kg);
    let m_tot = ma + mb;
    let r_cg = (ma * state_a.pos_rel_icrf_m + mb * state_b.pos_rel_icrf_m) / m_tot;
    let v_cg = (ma * state_a.vel_rel_icrf_m_s + mb * state_b.vel_rel_icrf_m_s) / m_tot;

    let q_merged = state_a.q_body_to_icrf;
    let q_i2m = q_merged.conjugate();
    let q_b2m = quat_normalize_portable(q_i2m * state_b.q_body_to_icrf);
    let off_a_m = quat_rotate_vec_portable(q_i2m, state_a.pos_rel_icrf_m - r_cg);
    let off_b_m = quat_rotate_vec_portable(q_i2m, state_b.pos_rel_icrf_m - r_cg);

    let (i_merged, omega_merged, h_rw_merged) = compute_merged_spin(
        state_a, mass_a, ma, state_b, mass_b, mb, r_cg, v_cg, q_i2m, q_b2m, off_a_m, off_b_m,
    );
    let pa_icrf = state_a.pos_rel_icrf_m
        + quat_rotate_vec_portable(state_a.q_body_to_icrf, port_a.pos_body_m);
    let pb_icrf = state_b.pos_rel_icrf_m
        + quat_rotate_vec_portable(state_b.q_body_to_icrf, port_b.pos_body_m);
    let port_mid_m = quat_rotate_vec_portable(q_i2m, 0.5 * (pa_icrf + pb_icrf) - r_cg);

    let merged_mass = MassPropertiesModel {
        dry_mass_kg: m_tot,
        dry_inertia_cols: i_merged.to_cols_array_2d(),
        propellant_inertia_per_kg_cols: [[0.0; 3]; 3],
        uncompensated_idot_factor: 0.0,
    };
    let merged_state = ContinuousPlantState {
        primary_body: state_a.primary_body,
        pos_rel_icrf_m: r_cg,
        vel_rel_icrf_m_s: v_cg,
        q_body_to_icrf: q_merged,
        omega_body_rad_s: omega_merged,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: h_rw_merged,
    };
    let rec = DockedAssemblyRecord {
        vessel_a,
        vessel_b,
        mass_a: *mass_a,
        prop_a_kg: state_a.propellant_mass_kg,
        h_rw_a_body_nms: state_a.h_rw_body_nms,
        mass_b: *mass_b,
        prop_b_kg: state_b.propellant_mass_kg,
        h_rw_b_body_nms: state_b.h_rw_body_nms,
        offset_a_merged_m: off_a_m,
        offset_b_merged_m: off_b_m,
        q_b_to_merged: q_b2m,
        port_point_merged_m: port_mid_m,
        sep_axis_merged: port_a.outward_axis_body,
    };
    Ok((merged_state, merged_mass, rec))
}

#[allow(clippy::too_many_arguments)]
fn compute_merged_spin(
    state_a: &ContinuousPlantState,
    mass_a: &MassPropertiesModel,
    ma: f64,
    state_b: &ContinuousPlantState,
    mass_b: &MassPropertiesModel,
    mb: f64,
    r_cg: DVec3,
    v_cg: DVec3,
    q_i2m: DQuat,
    q_b2m: DQuat,
    off_a_m: DVec3,
    off_b_m: DVec3,
) -> (DMat3, DVec3, DVec3) {
    let dr_a_icrf = state_a.pos_rel_icrf_m - r_cg;
    let dr_b_icrf = state_b.pos_rel_icrf_m - r_cg;
    let h_cg_icrf = dr_a_icrf.cross(ma * (state_a.vel_rel_icrf_m_s - v_cg))
        + dr_b_icrf.cross(mb * (state_b.vel_rel_icrf_m_s - v_cg))
        + mass_a.total_angular_momentum_icrf(state_a)
        + mass_b.total_angular_momentum_icrf(state_b);

    let r_b2m = DMat3::from_quat(q_b2m);
    let i_a = mass_a.inertia_tensor(state_a.propellant_mass_kg);
    let i_b_in_m = r_b2m * mass_b.inertia_tensor(state_b.propellant_mass_kg) * r_b2m.transpose();
    let i_merged =
        i_a + parallel_axis_shift(ma, off_a_m) + i_b_in_m + parallel_axis_shift(mb, off_b_m);

    let h_rw_merged =
        state_a.h_rw_body_nms + quat_rotate_vec_portable(q_b2m, state_b.h_rw_body_nms);
    let h_cg_merged = quat_rotate_vec_portable(q_i2m, h_cg_icrf);
    let omega_merged = i_merged.inverse() * (h_cg_merged - h_rw_merged);
    (i_merged, omega_merged, h_rw_merged)
}

/// Splits a docked composite vessel back into `(state_a, state_b)` with an internal `APAS-95`
/// push-off spring impulse `separation_impulse_n_s >= 0` applied equal-and-opposite at the port
/// interface, conserving both $\mathbf{P}$ and $\mathbf{H}_{0,\text{ICRF}}$ to machine precision.
#[must_use]
pub fn undock_assembly_conserving_momentum(
    merged_state: &ContinuousPlantState,
    record: &DockedAssemblyRecord,
    separation_impulse_n_s: f64,
) -> (ContinuousPlantState, ContinuousPlantState) {
    let q_m2i = merged_state.q_body_to_icrf;
    let q_b2i = quat_normalize_portable(q_m2i * record.q_b_to_merged);
    let w_m = merged_state.omega_body_rad_s;

    let pos_a =
        merged_state.pos_rel_icrf_m + quat_rotate_vec_portable(q_m2i, record.offset_a_merged_m);
    let pos_b =
        merged_state.pos_rel_icrf_m + quat_rotate_vec_portable(q_m2i, record.offset_b_merged_m);
    let vel_a_pre = merged_state.vel_rel_icrf_m_s
        + quat_rotate_vec_portable(q_m2i, w_m.cross(record.offset_a_merged_m));
    let vel_b_pre = merged_state.vel_rel_icrf_m_s
        + quat_rotate_vec_portable(q_m2i, w_m.cross(record.offset_b_merged_m));

    let omega_a_pre = w_m;
    let omega_b_pre = quat_rotate_vec_portable(record.q_b_to_merged.conjugate(), w_m);

    let j_mag = separation_impulse_n_s.max(0.0);
    let j_icrf = quat_rotate_vec_portable(q_m2i, record.sep_axis_merged * j_mag);
    let r_port_icrf =
        merged_state.pos_rel_icrf_m + quat_rotate_vec_portable(q_m2i, record.port_point_merged_m);

    let ma = record.mass_a.total_mass_kg(record.prop_a_kg);
    let mb = record.mass_b.total_mass_kg(record.prop_b_kg);
    let vel_a = vel_a_pre - j_icrf / ma;
    let vel_b = vel_b_pre + j_icrf / mb;

    let inv_ia = record.mass_a.inertia_tensor(record.prop_a_kg).inverse();
    let inv_ib = record.mass_b.inertia_tensor(record.prop_b_kg).inverse();
    let tau_imp_a_icrf = (r_port_icrf - pos_a).cross(-j_icrf);
    let tau_imp_b_icrf = (r_port_icrf - pos_b).cross(j_icrf);
    let omega_a =
        omega_a_pre + inv_ia * quat_rotate_vec_portable(q_m2i.conjugate(), tau_imp_a_icrf);
    let omega_b =
        omega_b_pre + inv_ib * quat_rotate_vec_portable(q_b2i.conjugate(), tau_imp_b_icrf);

    let delta_h_rw_m = merged_state.h_rw_body_nms
        - (record.h_rw_a_body_nms
            + quat_rotate_vec_portable(record.q_b_to_merged, record.h_rw_b_body_nms));
    let h_rw_a = record.h_rw_a_body_nms + 0.5 * delta_h_rw_m;
    let h_rw_b = record.h_rw_b_body_nms
        + quat_rotate_vec_portable(record.q_b_to_merged.conjugate(), 0.5 * delta_h_rw_m);

    let st_a = ContinuousPlantState {
        primary_body: merged_state.primary_body,
        pos_rel_icrf_m: pos_a,
        vel_rel_icrf_m_s: vel_a,
        q_body_to_icrf: q_m2i,
        omega_body_rad_s: omega_a,
        propellant_mass_kg: record.prop_a_kg,
        h_rw_body_nms: h_rw_a,
    };
    let st_b = ContinuousPlantState {
        primary_body: merged_state.primary_body,
        pos_rel_icrf_m: pos_b,
        vel_rel_icrf_m_s: vel_b,
        q_body_to_icrf: q_b2i,
        omega_body_rad_s: omega_b,
        propellant_mass_kg: record.prop_b_kg,
        h_rw_body_nms: h_rw_b,
    };
    (st_a, st_b)
}

fn parallel_axis_shift(mass_kg: f64, r: DVec3) -> DMat3 {
    let r2 = r.length_squared();
    let outer = DMat3::from_cols(r * r.x, r * r.y, r * r.z);
    (DMat3::IDENTITY * r2 - outer) * mass_kg
}
