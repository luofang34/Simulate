//! Powered descent from rest above the landing target down to the touchdown sink rate.
//!
//! The nose is held on the local vertical throughout: the attitude loop is far too slow
//! (about `0.4 rad/s` natural frequency) to steer by tilting, so the main engine flies the
//! vertical channel and the CG translational RCS flies the horizontal one. Above the
//! terminal gate the vertical channel follows a braking-distance velocity profile; below
//! it the terminal vertical mode of `evaluate_powered_descent_autoland` holds the sink rate.

use glam::DVec3;
use sim_geo::LocalFrame;
use sim_orbital::{
    AutolandGuidanceConfig, BodyOrientationCatalog, BodyRotationState, ContactFootSpec,
    ContinuousControlLaw, ContinuousPlantState, FOUR_G_PROPER_ACCEL_M_S2, NaifId, PlantStageWrench,
    STANDARD_GRAVITY_G0_M_S2, TdbEpoch, evaluate_powered_descent_autoland,
    quat_rotate_vec_portable,
};
use sim_scenery::PinnedCollisionTerrain;

use super::voyage::gyroscopic_torque_body_nm;
use crate::{
    aero_actuators::{RAKETA_CG_Y_M, RaketaStageLaw},
    gnc_boundary::allocate_translational_rcs_unit,
};

/// CG height above the ground at which an upright vessel's foot pads first touch (`m`).
const FOOT_DROP_M: f64 = RAKETA_CG_Y_M - 0.0369 + 0.04;
/// Design vertical braking deceleration of the descent profile (`m/s^2`).
const VERTICAL_DECEL_M_S2: f64 = 0.6 * STANDARD_GRAVITY_G0_M_S2;
/// Time constant of the low-altitude part of the vertical profile (`s`).
const VERTICAL_TAU_S: f64 = 12.0;
/// Fastest descent the profile asks for, keeping Martian drag small (`m/s`).
const VERTICAL_SPEED_CAP_M_S: f64 = 300.0;
/// Descent rate the profile reaches at the terminal gate (`m/s`).
const GATE_SINK_M_S: f64 = 1.5;
/// Vertical velocity-tracking gain (`1/s`).
const VERTICAL_GAIN_PER_S: f64 = 0.5;
/// Design horizontal braking deceleration, a third of the RCS authority (`m/s^2`).
const HORIZONTAL_DECEL_M_S2: f64 = 0.8;
/// Time constant of the final horizontal approach (`s`).
const HORIZONTAL_TAU_S: f64 = 15.0;
/// Fastest horizontal traverse the profile asks for (`m/s`).
const HORIZONTAL_SPEED_CAP_M_S: f64 = 60.0;
/// The vertical profile keeps the vessel above a cone of this slope round the target.
const CONE_SLOPE: f64 = 0.5;
/// Lateral error beyond which the profile stops adding the gate sink rate (`m`).
const SINK_LATERAL_LIMIT_M: f64 = 50.0;
/// Main-engine thrust is withheld while the nose is further than this off vertical (`rad`).
const THRUST_GATE_RAD: f64 = 0.35;

/// Continuous descent law towards the touchdown point `target_fixed_m` on `body`.
pub(super) struct SurfaceDescentLaw<'a> {
    pub(super) orientation: &'a dyn BodyOrientationCatalog,
    pub(super) body: NaifId,
    pub(super) terrain: &'a PinnedCollisionTerrain,
    pub(super) mu_m3_s2: f64,
    /// CG position at touchdown in body-fixed coordinates (`m`).
    pub(super) target_fixed_m: DVec3,
}

/// Lowest foot-pad clearance above the collision terrain, computed exactly as the contact
/// model computes it (`m`, negative when compressed).
pub(super) fn foot_clearance_m(
    terrain: &PinnedCollisionTerrain,
    rot: &BodyRotationState,
    state: &ContinuousPlantState,
) -> f64 {
    ContactFootSpec::raketa_four_fin_pads(RAKETA_CG_Y_M)
        .iter()
        .map(|foot| {
            let lever = quat_rotate_vec_portable(state.q_body_to_icrf, foot.pos_body_m);
            let p_fixed = rot.r_icrf_to_fixed * (state.pos_rel_icrf_m + lever);
            let (geo, ground) = terrain.sample_fixed(p_fixed);
            geo.height_m - foot.pad_radius_m - ground.ellipsoidal_height_m
        })
        .fold(f64::INFINITY, f64::min)
}

/// `(speed, d speed / d distance)` of a braking profile that decelerates at `decel` far
/// out and closes exponentially with time constant `tau` near zero, capped at `cap`.
fn braking_profile(distance_m: f64, decel_m_s2: f64, tau_s: f64, cap_m_s: f64) -> (f64, f64) {
    let d = distance_m.max(0.0);
    let linear = d / tau_s;
    let root = (2.0 * decel_m_s2 * d).sqrt();
    if linear <= root && linear <= cap_m_s {
        (linear, 1.0 / tau_s)
    } else if root <= cap_m_s {
        (root, decel_m_s2 / root.max(1.0e-6))
    } else {
        (cap_m_s, 0.0)
    }
}

impl<'a> SurfaceDescentLaw<'a> {
    /// The inner-loop stage law this descent law asks for at `state`.
    pub(super) fn stage_law(
        &self,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> RaketaStageLaw<'a> {
        let up_guess = state.pos_rel_icrf_m.normalize_or_zero();
        let Ok(rot) = self.orientation.body_rotation(self.body, epoch) else {
            return self.law(0.0, up_guess, DVec3::ZERO);
        };
        let r_f2i = rot.r_icrf_to_fixed.transpose();
        let (p_f, v_f) = rot.icrf_to_fixed_pv(state.pos_rel_icrf_m, state.vel_rel_icrf_m_s);
        let (geo, ground) = self.terrain.sample_fixed(p_f);
        let up_f = LocalFrame::at(&geo).up();
        let up_i = r_f2i * up_f;
        let agl_m = geo.height_m - ground.ellipsoidal_height_m - FOOT_DROP_M;
        let to_target = self.target_fixed_m - p_f;
        let e_h = to_target - up_f * to_target.dot(up_f);
        let lateral_m = e_h.length();
        let v_up = v_f.dot(up_f);
        let v_h = v_f - up_f * v_up;

        let a_up = if agl_m <= AutolandGuidanceConfig::default().terminal_vertical_alt_m {
            self.terminal_vertical_accel(&rot, state, up_i, agl_m)
        } else {
            let gravity = self.mu_m3_s2 / p_f.length_squared().max(1.0);
            let terminal = AutolandGuidanceConfig::default().terminal_vertical_alt_m;
            let h_eff = agl_m - terminal - CONE_SLOPE * lateral_m;
            let (speed, slope) = braking_profile(
                h_eff,
                VERTICAL_DECEL_M_S2,
                VERTICAL_TAU_S,
                VERTICAL_SPEED_CAP_M_S,
            );
            let sink = GATE_SINK_M_S * (1.0 - lateral_m / SINK_LATERAL_LIMIT_M).clamp(0.0, 1.0);
            let feed_forward = (-slope * v_up).clamp(0.0, 1.5 * VERTICAL_DECEL_M_S2);
            gravity + VERTICAL_GAIN_PER_S * (-(speed + sink) - v_up) + feed_forward
        };

        let h_cap = HORIZONTAL_SPEED_CAP_M_S.min(0.1 * agl_m.max(0.0) + 0.02);
        let (h_speed, _) =
            braking_profile(lateral_m, HORIZONTAL_DECEL_M_S2, HORIZONTAL_TAU_S, h_cap);
        let v_h_wanted = e_h.normalize_or_zero() * h_speed;
        let q = sim_orbital::quat_normalize_portable(state.q_body_to_icrf);
        let mut err_body = quat_rotate_vec_portable(q.conjugate(), r_f2i * (v_h_wanted - v_h));
        err_body.y = 0.0;
        let rcs = allocate_translational_rcs_unit(err_body);

        let cos_tilt = quat_rotate_vec_portable(q, DVec3::Y).dot(up_i);
        let gate =
            ((cos_tilt - THRUST_GATE_RAD.cos()) / (1.0 - THRUST_GATE_RAD.cos())).clamp(0.0, 1.0);
        let main = (a_up / cos_tilt.max(0.5)).clamp(0.0, FOUR_G_PROPER_ACCEL_M_S2) * gate;
        self.law(main, up_i, rcs)
    }

    /// Upward specific force the terminal vertical mode of the autoland guidance asks for.
    fn terminal_vertical_accel(
        &self,
        rot: &BodyRotationState,
        state: &ContinuousPlantState,
        up_icrf: DVec3,
        agl_m: f64,
    ) -> f64 {
        let (target_pos, target_vel) = rot.fixed_to_icrf_pv(self.target_fixed_m, DVec3::ZERO);
        let r = state.pos_rel_icrf_m;
        let gravity_icrf = -r * (self.mu_m3_s2 / r.length().max(1.0).powi(3));
        let out = evaluate_powered_descent_autoland(
            r,
            state.vel_rel_icrf_m_s,
            target_pos,
            target_vel,
            up_icrf,
            gravity_icrf,
            agl_m,
            &AutolandGuidanceConfig::default(),
        );
        (out.thrust_dir_icrf * out.commanded_accel_m_s2).dot(up_icrf)
    }

    const fn law(&self, main: f64, up_icrf: DVec3, rcs: DVec3) -> RaketaStageLaw<'a> {
        RaketaStageLaw {
            commanded_main_accel_m_s2: main,
            desired_body_y_icrf: up_icrf,
            rcs_cmd_unit_body: rcs,
            fin_cmd_deg: [0.0; 4],
            is_landing_or_pad_mode: true,
            orientation: self.orientation,
        }
    }
}

impl ContinuousControlLaw for SurfaceDescentLaw<'_> {
    fn evaluate_stage(
        &self,
        _tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        let mut wrench = self
            .stage_law(epoch, state)
            .evaluate_detailed(epoch, state)
            .0;
        wrench.ext_torque_body_nm += gyroscopic_torque_body_nm(state);
        wrench
    }
}
