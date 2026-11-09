//! Substep force evaluation, anchor-point bristle stiction, and settle tracking.

use glam::{DMat3, DQuat, DVec3};
use sim_spacetime::{CollisionGroundSample, CollisionTerrain};

use crate::{
    contact::{
        BristleFootState, ContactFootSpec, PINNED_REST_MAX_OMEGA_RAD_S, PINNED_REST_MAX_SPEED_M_S,
        PINNED_REST_SETTLE_SUBSTEPS, PinnedSurfaceState, VesselContactState,
    },
    ephemeris::EphemerisCatalog,
    gravity::{ForceModel, GravityError},
    orientation::BodyOrientationCatalog,
    plant::{
        ContinuousControlLaw, ContinuousPlantState, MassPropertiesModel, PropagationAudit,
        PropellantMode, ReactionWheelMode, clamp_vec_norm, quat_normalize_portable,
        quat_omega_derivative, quat_rotate_vec_portable,
    },
    scalar_math::sqrt,
    time::TdbEpoch,
};

#[allow(clippy::too_many_arguments)]
pub(super) fn step_one_contact_substep(
    state: &mut ContinuousPlantState,
    contact: &mut VesselContactState,
    feet_specs: &[ContactFootSpec],
    tau_s: f64,
    epoch: TdbEpoch,
    dt: f64,
    mass_model: &MassPropertiesModel,
    propellant_mode: PropellantMode,
    rw_mode: ReactionWheelMode,
    force_model: &ForceModel,
    terrain: &dyn CollisionTerrain,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    control: &dyn ContinuousControlLaw,
) -> Result<(f64, f64, PropagationAudit), GravityError> {
    let rot = orientation.body_rotation(state.primary_body, epoch)?;
    let r_i2f = rot.r_icrf_to_fixed;
    let r_f2i = r_i2f.transpose();
    let omega_p_fixed = rot.omega_body_rad_s;

    let (f_contact_icrf, tau_contact_body, min_clear, any_contact, any_slip) = evaluate_feet_wrench(
        state,
        &mut contact.feet,
        feet_specs,
        &r_i2f,
        &r_f2i,
        omega_p_fixed,
        terrain,
    );

    let g_icrf = force_model.relative_gravitational_accel_icrf(
        state.primary_body,
        state.pos_rel_icrf_m,
        epoch,
        ephemeris,
        orientation,
    )?;
    let wrench = control.evaluate_stage(tau_s, epoch, state);
    let mut aud = wrench.stage_audit;
    let m_tot = mass_model.total_mass_kg(state.propellant_mass_kg);
    let f_ctrl_body = wrench.main_thrust_body_n + wrench.rcs_force_body_n + wrench.env_force_body_n;
    let f_ng_icrf = quat_rotate_vec_portable(state.q_body_to_icrf, f_ctrl_body) + f_contact_icrf;
    let a_proper = sqrt((f_ng_icrf / m_tot).length_squared());

    advance_semi_implicit_state(
        state,
        dt,
        g_icrf + f_ng_icrf / m_tot,
        wrench.ext_torque_body_nm + tau_contact_body,
        wrench.rw_torque_cmd_body_nm,
        &wrench,
        mass_model,
        propellant_mode,
        rw_mode,
        &mut aud,
    );

    update_settle_tracker(state, contact, &r_i2f, omega_p_fixed, any_contact, any_slip);
    Ok((min_clear, a_proper, aud))
}

fn evaluate_feet_wrench(
    state: &ContinuousPlantState,
    feet_state: &mut [BristleFootState],
    feet_specs: &[ContactFootSpec],
    r_i2f: &DMat3,
    r_f2i: &DMat3,
    omega_p_fixed: DVec3,
    terrain: &dyn CollisionTerrain,
) -> (DVec3, DVec3, f64, bool, bool) {
    let mut f_tot_icrf = DVec3::ZERO;
    let mut tau_tot_body = DVec3::ZERO;
    let mut min_clear = f64::INFINITY;
    let mut any_contact = false;
    let mut any_slip = false;
    let q_i2b = state.q_body_to_icrf.conjugate();

    for (st, spec) in feet_state.iter_mut().zip(feet_specs.iter()) {
        let r_lever_icrf = quat_rotate_vec_portable(state.q_body_to_icrf, spec.pos_body_m);
        let v_lever_body = state.omega_body_rad_s.cross(spec.pos_body_m);
        let p_foot_icrf = state.pos_rel_icrf_m + r_lever_icrf;
        let v_foot_icrf =
            state.vel_rel_icrf_m_s + quat_rotate_vec_portable(state.q_body_to_icrf, v_lever_body);

        let p_foot_fixed = (*r_i2f) * p_foot_icrf;
        let v_foot_fixed = (*r_i2f) * v_foot_icrf - omega_p_fixed.cross(p_foot_fixed);
        let (geo, ground) = terrain.sample_fixed(p_foot_fixed);
        let clearance = geo.height_m - spec.pad_radius_m - ground.ellipsoidal_height_m;
        min_clear = min_clear.min(clearance);

        if clearance > 0.0 {
            st.anchored = false;
            st.slipping = false;
            continue;
        }
        any_contact = true;
        let f_foot_fixed =
            compute_single_foot_force_fixed(st, p_foot_fixed, v_foot_fixed, -clearance, &ground);
        if st.slipping {
            any_slip = true;
        }
        let f_foot_icrf = (*r_f2i) * f_foot_fixed;
        let f_foot_body = quat_rotate_vec_portable(q_i2b, f_foot_icrf);
        f_tot_icrf += f_foot_icrf;
        let omega_icrf = quat_rotate_vec_portable(state.q_body_to_icrf, state.omega_body_rad_s);
        let omega_rel_icrf = omega_icrf - (*r_f2i) * omega_p_fixed;
        let omega_rel_body = quat_rotate_vec_portable(q_i2b, omega_rel_icrf);
        let tau_pad_damp_body = -2.2 * ground.normal_damping_n_s_m * omega_rel_body;
        tau_tot_body += spec.pos_body_m.cross(f_foot_body) + tau_pad_damp_body;
    }
    (f_tot_icrf, tau_tot_body, min_clear, any_contact, any_slip)
}

fn compute_single_foot_force_fixed(
    st: &mut BristleFootState,
    p_foot_fixed: DVec3,
    v_foot_fixed: DVec3,
    penetration_m: f64,
    ground: &CollisionGroundSample,
) -> DVec3 {
    let n_up = ground.normal_fixed;
    let v_into = -v_foot_fixed.dot(n_up);
    let f_n = (ground.normal_stiffness_n_m * penetration_m + ground.normal_damping_n_s_m * v_into)
        .max(0.0);
    if f_n <= 0.0 {
        st.anchored = false;
        st.slipping = false;
        return DVec3::ZERO;
    }
    if !st.anchored {
        st.anchored = true;
        st.anchor_fixed_m = p_foot_fixed.to_array();
    }
    let anchor = DVec3::from_array(st.anchor_fixed_m);
    let raw_disp = p_foot_fixed - anchor;
    let disp_tan = raw_disp - raw_disp.dot(n_up) * n_up;
    let v_tan = v_foot_fixed - v_foot_fixed.dot(n_up) * n_up;

    let f_tan_trial =
        -ground.tangential_stiffness_n_m * disp_tan - ground.tangential_damping_n_s_m * v_tan;
    let trial_norm = sqrt(f_tan_trial.length_squared());
    let static_cap = ground.mu_static * f_n;

    let f_tan = if trial_norm <= static_cap {
        st.slipping = false;
        f_tan_trial
    } else {
        st.slipping = true;
        let kin_cap = ground.mu_kinetic * f_n;
        let dir = f_tan_trial / trial_norm;
        let max_stretch = kin_cap / ground.tangential_stiffness_n_m.max(1.0);
        let new_anchor = p_foot_fixed + dir * max_stretch;
        st.anchor_fixed_m = new_anchor.to_array();
        dir * kin_cap
    };
    f_n * n_up + f_tan
}

#[allow(clippy::too_many_arguments)]
fn advance_semi_implicit_state(
    state: &mut ContinuousPlantState,
    dt: f64,
    accel_icrf: DVec3,
    ext_torque_body: DVec3,
    rw_cmd_body: DVec3,
    wrench: &crate::plant::PlantStageWrench,
    mass_model: &MassPropertiesModel,
    propellant_mode: PropellantMode,
    rw_mode: ReactionWheelMode,
    aud: &mut PropagationAudit,
) {
    state.vel_rel_icrf_m_s += accel_icrf * dt;
    state.pos_rel_icrf_m += state.vel_rel_icrf_m_s * dt;

    let (tau_rw, h_rw_dot) = match rw_mode {
        ReactionWheelMode::ConserveMomentum {
            h_max_nms,
            tau_max_nm,
        } => {
            let t = clamp_vec_norm(rw_cmd_body, tau_max_nm);
            if sqrt(state.h_rw_body_nms.length_squared()) >= h_max_nms
                && (-t).dot(state.h_rw_body_nms) > 0.0
            {
                (DVec3::ZERO, DVec3::ZERO)
            } else {
                (t, -t)
            }
        }
        ReactionWheelMode::ExternalMagicTorque { tau_max_nm } => {
            let t = clamp_vec_norm(rw_cmd_body, tau_max_nm);
            if t.length_squared() > 0.0 {
                aud.angular_momentum_exempt = true;
            }
            (t, DVec3::ZERO)
        }
    };

    let i_mat = mass_model.inertia_tensor(state.propellant_mass_kg);
    let i_inv = i_mat.inverse();
    let h_total_body = i_mat * state.omega_body_rad_s + state.h_rw_body_nms;
    let omega_dot = i_inv * (ext_torque_body + tau_rw - state.omega_body_rad_s.cross(h_total_body));
    state.omega_body_rad_s += omega_dot * dt;
    state.h_rw_body_nms += h_rw_dot * dt;

    let dq = quat_omega_derivative(state.q_body_to_icrf, state.omega_body_rad_s);
    state.q_body_to_icrf = quat_normalize_portable(DQuat::from_xyzw(
        state.q_body_to_icrf.x + dq.x * dt,
        state.q_body_to_icrf.y + dq.y * dt,
        state.q_body_to_icrf.z + dq.z * dt,
        state.q_body_to_icrf.w + dq.w * dt,
    ));

    apply_contact_propellant_step(state, dt, wrench, propellant_mode, aud);
}

fn apply_contact_propellant_step(
    state: &mut ContinuousPlantState,
    dt: f64,
    wrench: &crate::plant::PlantStageWrench,
    propellant_mode: PropellantMode,
    aud: &mut PropagationAudit,
) {
    match propellant_mode {
        PropellantMode::InfiniteConstantMass => {
            if wrench.main_thrust_body_n.length_squared() > 0.0
                || wrench.rcs_force_body_n.length_squared() > 0.0
            {
                aud.mass_conservation_exempt = true;
            }
        }
        PropellantMode::FiniteVariableMass => {
            let mut mdot = 0.0_f64;
            if wrench.main_exhaust_vel_m_s > 0.0 {
                mdot -=
                    sqrt(wrench.main_thrust_body_n.length_squared()) / wrench.main_exhaust_vel_m_s;
            }
            if wrench.rcs_exhaust_vel_m_s > 0.0 {
                mdot -= sqrt(wrench.rcs_force_body_n.length_squared()) / wrench.rcs_exhaust_vel_m_s;
            }
            state.propellant_mass_kg = (state.propellant_mass_kg + mdot * dt).max(0.0);
        }
    }
}

fn update_settle_tracker(
    state: &ContinuousPlantState,
    contact: &mut VesselContactState,
    r_i2f: &DMat3,
    omega_p_fixed: DVec3,
    any_contact: bool,
    any_slip: bool,
) {
    if !any_contact || any_slip {
        contact.settle_substeps = 0;
        return;
    }
    let p_fixed = (*r_i2f) * state.pos_rel_icrf_m;
    let v_fixed = (*r_i2f) * state.vel_rel_icrf_m_s - omega_p_fixed.cross(p_fixed);
    let omega_icrf = quat_rotate_vec_portable(state.q_body_to_icrf, state.omega_body_rad_s);
    let omega_rel_fixed = (*r_i2f) * omega_icrf - omega_p_fixed;

    if sqrt(v_fixed.length_squared()) <= PINNED_REST_MAX_SPEED_M_S
        && sqrt(omega_rel_fixed.length_squared()) <= PINNED_REST_MAX_OMEGA_RAD_S
    {
        contact.settle_substeps = contact.settle_substeps.wrapping_add(1);
        if contact.settle_substeps >= PINNED_REST_SETTLE_SUBSTEPS {
            let q_i2f = quat_normalize_portable(DQuat::from_mat3(r_i2f));
            let q_b2f = quat_normalize_portable(q_i2f * state.q_body_to_icrf);
            contact.pinned = Some(PinnedSurfaceState {
                body: state.primary_body,
                pos_fixed_m: p_fixed.to_array(),
                q_body_to_fixed_xyzw: q_b2f.to_array(),
            });
        }
    } else {
        contact.settle_substeps = 0;
    }
}
