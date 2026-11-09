//! High-warp propagation semantics (`ReducedManifoldThrust`), integral + feed-forward
//! 6-DoF SAS tracking, and conservative `WarpGovernor` with global bottleneck reporting.

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    ids::VesselId,
    plant::{
        ContinuousControlLaw, ContinuousPlantState, MassPropertiesModel, PlantStageWrench,
        PropagationAudit, quat_geodesic_angle_rad, quat_normalize_portable, quat_omega_derivative,
        quat_rotate_vec_portable,
    },
    scalar_math::{cos, sin, sqrt},
    time::TdbEpoch,
};

/// Default entry/exit gate thresholds for [`PropagatorKind::ReducedManifoldThrust`](crate::PropagatorKind::ReducedManifoldThrust).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReducedThrustGateConfig {
    /// Maximum dynamic pressure $q_\infty$ (`Pa`, default `1.0e-3`).
    pub max_q_inf_pa: f64,
    /// Maximum geodesic attitude error (`rad`, default `1.0e-4`).
    pub max_attitude_error_rad: f64,
    /// Maximum angular rate error (`rad/s`, default `1.0e-5`).
    pub max_rate_error_rad_s: f64,
    /// Maximum ratio of required feed-forward torque to $\tau_{\max}$ (default `0.70`).
    pub max_torque_fraction: f64,
    /// Maximum ratio of wheel momentum $\|\mathbf{h}_{\text{rw}}\|$ to $h_{\max}$ (default `0.85`).
    pub max_wheel_momentum_fraction: f64,
}

impl Default for ReducedThrustGateConfig {
    fn default() -> Self {
        Self {
            max_q_inf_pa: 1.0e-3,
            max_attitude_error_rad: 1.0e-4,
            max_rate_error_rad_s: 1.0e-5,
            max_torque_fraction: 0.70,
            max_wheel_momentum_fraction: 0.85,
        }
    }
}

/// Specific reason why a vessel cannot enter or remain in `ReducedManifoldThrust`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReducedThrustGateRejection {
    /// Dynamic pressure $q_\infty \ge 10^{-3}\text{ Pa}$.
    DynamicPressureExceeded,
    /// Vessel is inside a proximity `SweptReachabilityTube` or `RendezvousGroup`.
    ProximityEnvelopeActive,
    /// Geodesic attitude tracking error $\ge 10^{-4}\text{ rad}$.
    AttitudeErrorExceeded,
    /// Angular rate tracking error $\ge 10^{-5}\text{ rad/s}$.
    RateErrorExceeded,
    /// Required feed-forward torque exceeds $0.70\,\tau_{\max}$.
    FeedForwardTorqueExceeded,
    /// Reaction wheel momentum $\|\mathbf{h}_{\text{rw}}\| \ge 0.85\,h_{\max}$.
    WheelMomentumSaturationProximity,
}

/// Evaluates whether a vessel satisfies all entry/continuation gates for `ReducedManifoldThrust`.
#[allow(clippy::too_many_arguments)]
pub fn check_reduced_manifold_gate(
    state: &ContinuousPlantState,
    q_ref_body_to_icrf: DQuat,
    omega_ref_body_rad_s: DVec3,
    alpha_ref_body_rad_s2: DVec3,
    q_inf_pa: f64,
    in_proximity_envelope: bool,
    mass_model: &MassPropertiesModel,
    tau_max_nm: f64,
    h_max_nms: f64,
    config: &ReducedThrustGateConfig,
) -> Result<(), ReducedThrustGateRejection> {
    if q_inf_pa >= config.max_q_inf_pa {
        return Err(ReducedThrustGateRejection::DynamicPressureExceeded);
    }
    if in_proximity_envelope {
        return Err(ReducedThrustGateRejection::ProximityEnvelopeActive);
    }
    let att_err = quat_geodesic_angle_rad(state.q_body_to_icrf, q_ref_body_to_icrf);
    if att_err >= config.max_attitude_error_rad {
        return Err(ReducedThrustGateRejection::AttitudeErrorExceeded);
    }
    let rate_err = sqrt((state.omega_body_rad_s - omega_ref_body_rad_s).length_squared());
    if rate_err >= config.max_rate_error_rad_s {
        return Err(ReducedThrustGateRejection::RateErrorExceeded);
    }
    let i_mat = mass_model.inertia_tensor(state.propellant_mass_kg);
    let h_tot = i_mat * omega_ref_body_rad_s + state.h_rw_body_nms;
    let tau_ff = i_mat * alpha_ref_body_rad_s2 + omega_ref_body_rad_s.cross(h_tot);
    if sqrt(tau_ff.length_squared()) > config.max_torque_fraction * tau_max_nm.max(1.0e-9) {
        return Err(ReducedThrustGateRejection::FeedForwardTorqueExceeded);
    }
    if sqrt(state.h_rw_body_nms.length_squared())
        >= config.max_wheel_momentum_fraction * h_max_nms.max(1.0e-9)
    {
        return Err(ReducedThrustGateRejection::WheelMomentumSaturationProximity);
    }
    Ok(())
}

/// Smooth pitch/yaw steering profile with analytic $\mathbf{q}_{\text{ref}}(\tau)$,
/// $\boldsymbol{\omega}_{\text{ref}}(\tau)$, and $\dot{\boldsymbol{\omega}}_{\text{ref}}(\tau)$
/// evaluated at every Runge-Kutta stage.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SmoothSteeringProfile {
    /// Initial body-to-ICRF reference quaternion at $\tau = 0$.
    #[serde(with = "crate::plant::attitude_math::serde_dquat")]
    pub q0_body_to_icrf: DQuat,
    /// Constant base angular rate about body Z (`rad/s`).
    pub pitch_rate_rad_s: f64,
    /// Sinusoidal pitch modulation amplitude (`rad`).
    pub harmonic_amp_rad: f64,
    /// Sinusoidal modulation angular frequency (`rad/s`).
    pub harmonic_freq_rad_s: f64,
    /// Commanded main-engine thrust along body `+Y` (`N`).
    pub thrust_n: f64,
    /// Main-engine effective exhaust velocity (`m/s`).
    pub exhaust_vel_m_s: f64,
}

impl SmoothSteeringProfile {
    /// Evaluates `(q_ref, omega_ref_body, alpha_ref_body)` at segment-local offset `tau_s` (`s`).
    #[must_use]
    pub fn evaluate_reference(&self, tau_s: f64) -> (DQuat, DVec3, DVec3) {
        let w = self.harmonic_freq_rad_s;
        let wt = w * tau_s;
        let theta = self.pitch_rate_rad_s * tau_s + self.harmonic_amp_rad * sin(wt);
        let omega_z = self.pitch_rate_rad_s + self.harmonic_amp_rad * w * cos(wt);
        let alpha_z = -self.harmonic_amp_rad * w * w * sin(wt);
        let half = 0.5 * theta;
        let dq = DQuat::from_xyzw(0.0, 0.0, sin(half), cos(half));
        let q_ref = quat_normalize_portable(self.q0_body_to_icrf * dq);
        (
            q_ref,
            DVec3::new(0.0, 0.0, omega_z),
            DVec3::new(0.0, 0.0, alpha_z),
        )
    }
}

/// Stage-evaluated `ReducedManifoldThrust` control law that projects thrust onto the
/// smooth reference manifold $\mathbf{q}_{\text{ref}}(\tau)$ and integrates $\dot{\mathbf{h}}_{\text{rw}}$
/// continuously via exact feed-forward torque.
#[derive(Clone, Debug)]
pub struct ReducedManifoldSteeringLaw {
    /// Smooth reference steering profile.
    pub profile: SmoothSteeringProfile,
    /// Vessel mass properties used to compute $\boldsymbol{\tau}_{\text{ff}}(\tau)$.
    pub mass_model: MassPropertiesModel,
}

impl ContinuousControlLaw for ReducedManifoldSteeringLaw {
    fn evaluate_stage(
        &self,
        tau_s: f64,
        _epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        let (q_ref, omega_ref, alpha_ref) = self.profile.evaluate_reference(tau_s);
        let q_unit = quat_normalize_portable(state.q_body_to_icrf);
        // Project the reference +Y thrust direction into the current body frame so that
        // quat_rotate_vec_portable(q_unit, main_thrust_body_n) == q_ref * (0, T, 0)!
        let thrust_icrf =
            quat_rotate_vec_portable(q_ref, DVec3::new(0.0, self.profile.thrust_n, 0.0));
        let thrust_body = quat_rotate_vec_portable(q_unit.conjugate(), thrust_icrf);

        let i_mat = self.mass_model.inertia_tensor(state.propellant_mass_kg);
        let h_tot = i_mat * omega_ref + state.h_rw_body_nms;
        let tau_ff = i_mat * alpha_ref + omega_ref.cross(h_tot);
        let _ = quat_omega_derivative;

        PlantStageWrench {
            main_thrust_body_n: thrust_body,
            main_exhaust_vel_m_s: self.profile.exhaust_vel_m_s,
            rw_torque_cmd_body_nm: tau_ff,
            stage_audit: PropagationAudit::default(),
            ..PlantStageWrench::default()
        }
    }
}

/// Full 6-DoF Stability Augmentation System (`SAS`) control law combining exact feed-forward
/// torque $\mathbf{I}\dot{\boldsymbol{\omega}}_{\text{ref}} + \boldsymbol{\omega}\times(\mathbf{I}\boldsymbol{\omega} + \mathbf{h}_{\text{rw}})$
/// with proportional-derivative-integral tracking to eliminate steady-state pointing bias.
#[derive(Clone, Debug)]
pub struct FullCoupledPidFeedForwardSasLaw {
    /// Smooth reference steering profile.
    pub profile: SmoothSteeringProfile,
    /// Vessel mass properties.
    pub mass_model: MassPropertiesModel,
    /// Proportional attitude gain (`N*m/rad`).
    pub kp_nm_rad: f64,
    /// Derivative angular rate gain (`N*m*s/rad`).
    pub kd_nms_rad: f64,
    /// Latched integral torque bias compensation (`N*m`).
    pub integral_bias_nm: DVec3,
}

impl ContinuousControlLaw for FullCoupledPidFeedForwardSasLaw {
    fn evaluate_stage(
        &self,
        tau_s: f64,
        _epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        let (q_ref, omega_ref, alpha_ref) = self.profile.evaluate_reference(tau_s);
        let q_unit = quat_normalize_portable(state.q_body_to_icrf);
        let q_err = quat_normalize_portable(q_unit.conjugate() * q_ref);
        let sign = if q_err.w >= 0.0 { 1.0 } else { -1.0 };
        let err_vec = 2.0 * sign * DVec3::new(q_err.x, q_err.y, q_err.z);
        let omega_err = omega_ref - state.omega_body_rad_s;

        let i_mat = self.mass_model.inertia_tensor(state.propellant_mass_kg);
        let h_tot = i_mat * state.omega_body_rad_s + state.h_rw_body_nms;
        let tau_ff = i_mat * alpha_ref + state.omega_body_rad_s.cross(h_tot);
        let tau_cmd =
            tau_ff + self.kp_nm_rad * err_vec + self.kd_nms_rad * omega_err + self.integral_bias_nm;

        PlantStageWrench {
            main_thrust_body_n: DVec3::new(0.0, self.profile.thrust_n, 0.0),
            main_exhaust_vel_m_s: self.profile.exhaust_vel_m_s,
            rw_torque_cmd_body_nm: tau_cmd,
            stage_audit: PropagationAudit::default(),
            ..PlantStageWrench::default()
        }
    }
}

/// Computes the **provably conservative** hazard time-to-go (`s`), replacing the unsafe
/// $h/\max(v_{\text{down}}, 1)$ heuristic with:
/// $$t_{\text{min,kin}} = \frac{-v_d + \sqrt{v_d^2 + 2 a_{\text{in,max}} \max(h, 0)}}{a_{\text{in,max}}}$$
/// bounded above by any isolated Bernstein hazard crossing `earliest_bernstein_root_s`.
#[must_use]
pub fn conservative_hazard_time_to_go_s(
    altitude_clearance_m: f64,
    downward_speed_m_s: f64,
    max_inward_accel_m_s2: f64,
    earliest_bernstein_root_s: Option<f64>,
) -> f64 {
    let h = altitude_clearance_m.max(0.0);
    let vd = downward_speed_m_s.max(0.0);
    let a = max_inward_accel_m_s2.max(1.0e-6);
    let disc = vd * vd + 2.0 * a * h;
    let t_kin = (-vd + sqrt(disc)) / a;
    match earliest_bernstein_root_s {
        Some(t_root) => t_kin.min(t_root.max(0.0)),
        None => t_kin,
    }
}

/// Reason why the global universe warp factor is capped below `requested_warp`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum WarpBottleneckReason {
    /// No bottleneck; `achieved_warp == requested_warp`.
    Unconstrained,
    /// A vessel is actively executing `1 ms` compliant ground contact (`ContactSubstep`).
    ActiveGroundContact,
    /// A vessel is inside sensible atmosphere ($q_\infty \ge 10^{-3}\text{ Pa}$).
    AtmosphericFlight,
    /// A vessel is inside a proximity collision/rendezvous tube (`RendezvousGroup`).
    ProximityRendezvous,
    /// A vessel is within a short time-to-go of a terrain or collision hazard.
    ImminentHazardCrossing,
    /// A vessel requires high-rate `FullCoupled6Dof` attitude control outside the reduced manifold.
    FullCoupledAttitudeTransient,
}

/// Per-vessel constraint snapshot supplied to [`evaluate_global_warp_status`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VesselWarpConstraintInput {
    /// Vessel identifier.
    pub vessel_id: VesselId,
    /// Whether this vessel is settled in `PinnedSurfaceRest` (does not limit warp!).
    pub is_pinned_surface_rest: bool,
    /// Whether this vessel is actively in `ContactSubstep` ground contact.
    pub is_active_ground_contact: bool,
    /// Current dynamic pressure $q_\infty$ (`Pa`).
    pub q_inf_pa: f64,
    /// Whether this vessel is inside a multi-vessel proximity `RendezvousGroup`.
    pub in_proximity_group: bool,
    /// Conservative hazard time-to-go (`s`) from [`conservative_hazard_time_to_go_s`].
    pub hazard_tgo_s: f64,
    /// Whether this vessel satisfies the `ReducedManifoldThrust` / `Dormant` gate when thrusting.
    pub on_reduced_manifold: bool,
}

/// Authoritative global warp governor output for HUD/telemetry display.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct WarpStatus {
    /// Player-requested time warp multiplier (`1.0 ..= 100_000.0`).
    pub requested_warp: f64,
    /// Achieved global committed-time warp multiplier.
    pub achieved_warp: f64,
    /// Specific vessel capping `achieved_warp` (`None` if `Unconstrained`).
    pub limiting_vessel: Option<VesselId>,
    /// Specific physical bottleneck reason on `limiting_vessel`.
    pub bottleneck_reason: WarpBottleneckReason,
}

/// Evaluates the global committed-time [`WarpStatus`] across all vessels in the universe.
#[must_use]
pub fn evaluate_global_warp_status(
    requested_warp: f64,
    nominal_frame_wall_s: f64,
    vessels: &[VesselWarpConstraintInput],
) -> WarpStatus {
    let req = requested_warp.clamp(1.0, 100_000.0);
    let dt_wall = nominal_frame_wall_s.max(1.0 / 240.0);
    let mut achieved = req;
    let mut limiter: Option<VesselId> = None;
    let mut reason = WarpBottleneckReason::Unconstrained;

    for v in vessels {
        if v.is_pinned_surface_rest {
            continue;
        }
        let (v_cap, v_reason) = single_vessel_warp_cap(v, dt_wall);
        if v_cap < achieved {
            achieved = v_cap;
            limiter = Some(v.vessel_id);
            reason = v_reason;
        }
    }

    WarpStatus {
        requested_warp: req,
        achieved_warp: achieved,
        limiting_vessel: limiter,
        bottleneck_reason: reason,
    }
}

fn single_vessel_warp_cap(
    v: &VesselWarpConstraintInput,
    dt_wall: f64,
) -> (f64, WarpBottleneckReason) {
    if v.is_active_ground_contact {
        return (1.0, WarpBottleneckReason::ActiveGroundContact);
    }
    let mut cap = 100_000.0_f64;
    let mut reason = WarpBottleneckReason::Unconstrained;

    if v.q_inf_pa >= 1.0 {
        cap = 4.0;
        reason = WarpBottleneckReason::AtmosphericFlight;
    } else if v.q_inf_pa >= 1.0e-3 {
        cap = 20.0;
        reason = WarpBottleneckReason::AtmosphericFlight;
    }

    if v.in_proximity_group && 10.0 < cap {
        cap = 10.0;
        reason = WarpBottleneckReason::ProximityRendezvous;
    }

    if !v.on_reduced_manifold && 50.0 < cap {
        cap = 50.0;
        reason = WarpBottleneckReason::FullCoupledAttitudeTransient;
    }

    // Ensure a single wall-clock frame never steps more than 25% of hazard_tgo_s:
    if v.hazard_tgo_s.is_finite() {
        let hazard_cap = ((0.25 * v.hazard_tgo_s) / dt_wall).clamp(1.0, 100_000.0);
        if hazard_cap < cap {
            cap = hazard_cap;
            reason = WarpBottleneckReason::ImminentHazardCrossing;
        }
    }

    (cap, reason)
}
