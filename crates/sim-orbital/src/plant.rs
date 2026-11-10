//! Continuous 6-DoF variable-mass and reaction-wheel spacecraft plant equations.
//!
//! Separates continuous force/torque evaluation from discrete GNC state updates so
//! Runge–Kutta stages remain pure functions of `(tau_s, epoch, state)`.

pub mod attitude_math;
mod srp;

use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    ephemeris::EphemerisCatalog,
    gravity::{ForceModel, GravityError},
    ids::NaifId,
    orientation::BodyOrientationCatalog,
    scalar_math::sqrt,
    time::TdbEpoch,
};

pub use attitude_math::{
    clamp_vec_norm, quat_geodesic_angle_rad, quat_normalize_portable, quat_omega_derivative,
    quat_rotate_vec_portable, serde_dquat, serde_dvec3,
};
pub use srp::{SolarRadiationPressureLaw, SrpSurface};

/// Dimension of the packed continuous 6-DoF plant ODE state vector.
pub const PLANT_ODE_DIM: usize = 17;

/// Propellant mass depletion semantics, explicitly auditing non-physical idealizations.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PropellantMode {
    /// Physical rocket mass depletion $\dot{m}_p = -\|\mathbf{F}_T\| / v_e$ for $m_p > 0$.
    FiniteVariableMass,
    /// Test-vehicle idealization (`Raketa`): $\dot{m}_p \equiv 0$, setting
    /// [`PropagationAudit::mass_conservation_exempt`] whenever thrust is applied.
    InfiniteConstantMass,
}

/// Reaction-wheel torque and internal angular-momentum accounting mode.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ReactionWheelMode {
    /// Physical internal wheel bank conserving total inertial angular momentum
    /// $\mathbf{H}_{\text{ICRF}} = R_{\text{B}\to\text{I}}(\mathbf{I}\boldsymbol{\omega} + \mathbf{h}_{\text{rw}})$
    /// via $\dot{\mathbf{h}}_{\text{rw}} = -\boldsymbol{\tau}_{\text{rw}}$ up to `h_max_nms`.
    ConserveMomentum {
        /// Maximum wheel-bank angular momentum norm $h_{\max}$ ($\text{N}\cdot\text{m}\cdot\text{s}$).
        h_max_nms: f64,
        /// Maximum wheel control torque norm $\tau_{\max}$ ($\text{N}\cdot\text{m}$).
        tau_max_nm: f64,
    },
    /// Idealized external control torque ($\dot{\mathbf{h}}_{\text{rw}} \equiv \mathbf{0}$), setting
    /// [`PropagationAudit::angular_momentum_exempt`] whenever torque is applied.
    ExternalMagicTorque {
        /// Maximum control torque norm $\tau_{\max}$ ($\text{N}\cdot\text{m}$).
        tau_max_nm: f64,
    },
}

/// Audit flags recording whether non-physical test idealizations were active on a trajectory.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PropagationAudit {
    /// `true` if `PropellantMode::InfiniteConstantMass` produced thrust with $\dot{m} = 0$.
    pub mass_conservation_exempt: bool,
    /// `true` if `ReactionWheelMode::ExternalMagicTorque` applied torque without $\dot{\mathbf{h}}_{\text{rw}}$.
    pub angular_momentum_exempt: bool,
    /// `true` if proper-acceleration limiting hit an infeasible external-load condition.
    pub proper_accel_exceedance: bool,
}

impl PropagationAudit {
    /// Merges flags from another step or segment audit.
    #[must_use]
    pub const fn merged(self, other: Self) -> Self {
        Self {
            mass_conservation_exempt: self.mass_conservation_exempt
                || other.mass_conservation_exempt,
            angular_momentum_exempt: self.angular_momentum_exempt || other.angular_momentum_exempt,
            proper_accel_exceedance: self.proper_accel_exceedance || other.proper_accel_exceedance,
        }
    }
}

/// Continuous translational, rotational, mass, and wheel-momentum state of one vessel.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContinuousPlantState {
    /// Primary coordinate origin (`NaifId::SSB` or an attracting body $P \in \mathcal{B}$).
    pub primary_body: NaifId,
    /// Position $\mathbf{r}$ relative to `primary_body` in non-rotating `ICRF` axes ($\text{m}$).
    #[serde(with = "serde_dvec3")]
    pub pos_rel_icrf_m: DVec3,
    /// Velocity $\dot{\mathbf{r}}$ relative to `primary_body` in non-rotating `ICRF` axes ($\text{m}/\text{s}$).
    #[serde(with = "serde_dvec3")]
    pub vel_rel_icrf_m_s: DVec3,
    /// Unit quaternion rotating body-frame vectors to `ICRF` axes.
    #[serde(with = "serde_dquat", rename = "q_body_to_icrf_xyzw")]
    pub q_body_to_icrf: DQuat,
    /// Angular velocity $\boldsymbol{\omega}_{\text{body}}$ of body wrt `ICRF` in body axes ($\text{rad}/\text{s}$).
    #[serde(with = "serde_dvec3")]
    pub omega_body_rad_s: DVec3,
    /// Remaining usable propellant mass $m_p \ge 0$ ($\text{kg}$).
    pub propellant_mass_kg: f64,
    /// Internal reaction-wheel angular momentum $\mathbf{h}_{\text{rw}}$ in body axes ($\text{N}\cdot\text{m}\cdot\text{s}$).
    #[serde(with = "serde_dvec3")]
    pub h_rw_body_nms: DVec3,
}

impl ContinuousPlantState {
    /// Packs the continuous plant state into a 17-element `f64` array for `DOP853`.
    #[must_use]
    pub fn to_ode_array(&self) -> [f64; PLANT_ODE_DIM] {
        let p = self.pos_rel_icrf_m;
        let v = self.vel_rel_icrf_m_s;
        let q = self.q_body_to_icrf;
        let w = self.omega_body_rad_s;
        let h = self.h_rw_body_nms;
        [
            p.x,
            p.y,
            p.z,
            v.x,
            v.y,
            v.z,
            q.x,
            q.y,
            q.z,
            q.w,
            w.x,
            w.y,
            w.z,
            self.propellant_mass_kg,
            h.x,
            h.y,
            h.z,
        ]
    }

    /// Unpacks a 17-element `f64` array into a [`ContinuousPlantState`].
    ///
    /// Pass `normalize_quat = false` inside intermediate Runge–Kutta stages (preserving
    /// smooth 8th-order polynomial Taylor expansions) and `true` at accepted step endpoints.
    #[must_use]
    pub fn from_ode_array(
        primary_body: NaifId,
        y: &[f64; PLANT_ODE_DIM],
        normalize_quat: bool,
    ) -> Self {
        let [
            px,
            py,
            pz,
            vx,
            vy,
            vz,
            qx,
            qy,
            qz,
            qw,
            wx,
            wy,
            wz,
            mp,
            hx,
            hy,
            hz,
        ] = *y;
        let raw_q = DQuat::from_xyzw(qx, qy, qz, qw);
        let q = if normalize_quat {
            quat_normalize_portable(raw_q)
        } else {
            raw_q
        };
        Self {
            primary_body,
            pos_rel_icrf_m: DVec3::new(px, py, pz),
            vel_rel_icrf_m_s: DVec3::new(vx, vy, vz),
            q_body_to_icrf: q,
            omega_body_rad_s: DVec3::new(wx, wy, wz),
            propellant_mass_kg: mp,
            h_rw_body_nms: DVec3::new(hx, hy, hz),
        }
    }
}

/// Rigid + variable-propellant mass and inertia tensor specification in body coordinates.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MassPropertiesModel {
    /// Dry structural mass $m_{\text{dry}} > 0$ ($\text{kg}$).
    pub dry_mass_kg: f64,
    /// Principal or full symmetric dry inertia tensor rows `[[Ixx, Ixy, Ixz], ...]` ($\text{kg}\cdot\text{m}^2$).
    pub dry_inertia_cols: [[f64; 3]; 3],
    /// Per-kilogram propellant inertia contribution $\mathbf{J}_p$ such that $\mathbf{I}(m_p) = \mathbf{I}_{\text{dry}} + m_p \mathbf{J}_p$.
    pub propellant_inertia_per_kg_cols: [[f64; 3]; 3],
    /// Uncompensated internal $\dot{\mathbf{I}}$ factor (`0.0` when isotropic exhaust carries tank spin away, `1.0` for internal mass gain/shift).
    pub uncompensated_idot_factor: f64,
}

impl MassPropertiesModel {
    /// Diagonal dry + propellant inertia helper.
    #[must_use]
    pub const fn diagonal(
        dry_mass_kg: f64,
        dry_diag_kg_m2: [f64; 3],
        prop_per_kg_diag_m2: [f64; 3],
    ) -> Self {
        let [ixx, iyy, izz] = dry_diag_kg_m2;
        let [jxx, jyy, jzz] = prop_per_kg_diag_m2;
        Self {
            dry_mass_kg,
            dry_inertia_cols: [[ixx, 0.0, 0.0], [0.0, iyy, 0.0], [0.0, 0.0, izz]],
            propellant_inertia_per_kg_cols: [[jxx, 0.0, 0.0], [0.0, jyy, 0.0], [0.0, 0.0, jzz]],
            uncompensated_idot_factor: 0.0,
        }
    }

    /// Total vehicle mass $m = m_{\text{dry}} + \max(m_p, 0)$ ($\text{kg}$).
    #[must_use]
    pub fn total_mass_kg(&self, propellant_mass_kg: f64) -> f64 {
        self.dry_mass_kg + propellant_mass_kg.max(0.0)
    }

    /// Instantaneous body-frame inertia tensor $\mathbf{I}(m_p)$ ($\text{kg}\cdot\text{m}^2$).
    #[must_use]
    pub fn inertia_tensor(&self, propellant_mass_kg: f64) -> DMat3 {
        let mp = propellant_mass_kg.max(0.0);
        DMat3::from_cols_array_2d(&self.dry_inertia_cols)
            + DMat3::from_cols_array_2d(&self.propellant_inertia_per_kg_cols) * mp
    }

    /// Uncompensated body-frame inertia rate $\dot{\mathbf{I}}_{\text{eff}}(\dot{m}_p)$.
    #[must_use]
    pub fn uncompensated_inertia_rate(&self, dm_dt_kg_s: f64) -> DMat3 {
        DMat3::from_cols_array_2d(&self.propellant_inertia_per_kg_cols)
            * (dm_dt_kg_s * self.uncompensated_idot_factor)
    }

    /// Total inertial angular momentum $\mathbf{H}_{\text{ICRF}} = R_{\text{B}\to\text{I}}\bigl(\mathbf{I}(m_p)\boldsymbol{\omega} + \mathbf{h}_{\text{rw}}\bigr)$.
    #[must_use]
    pub fn total_angular_momentum_icrf(&self, state: &ContinuousPlantState) -> DVec3 {
        let i_mat = self.inertia_tensor(state.propellant_mass_kg);
        let h_total_body = i_mat * state.omega_body_rad_s + state.h_rw_body_nms;
        quat_rotate_vec_portable(quat_normalize_portable(state.q_body_to_icrf), h_total_body)
    }
}

/// Instantaneous body-frame forces, moments, and actuator commands evaluated at a single RK stage.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlantStageWrench {
    /// Main-engine thrust force in body coordinates ($\text{N}$).
    pub main_thrust_body_n: DVec3,
    /// Effective exhaust velocity $v_e = I_{\text{sp}} g_0$ ($\text{m}/\text{s}$) for main-engine mass flow.
    pub main_exhaust_vel_m_s: f64,
    /// Translational RCS force in body coordinates ($\text{N}$).
    pub rcs_force_body_n: DVec3,
    /// Effective exhaust velocity $v_e$ ($\text{m}/\text{s}$) for RCS mass flow.
    pub rcs_exhaust_vel_m_s: f64,
    /// Passive aerodynamic + environment force in body coordinates ($\text{N}$).
    pub env_force_body_n: DVec3,
    /// External aerodynamic / thruster-offset / gravity-gradient torque in body coordinates ($\text{N}\cdot\text{m}$).
    pub ext_torque_body_nm: DVec3,
    /// Commanded reaction-wheel torque on the spacecraft body ($\text{N}\cdot\text{m}$).
    pub rw_torque_cmd_body_nm: DVec3,
    /// Stage-level idealization / exceedance audit flags.
    pub stage_audit: PropagationAudit,
}

/// Pure continuous steering/actuator law evaluated inside integrator stages (`&self`, zero mutation).
pub trait ContinuousControlLaw: Send + Sync {
    /// Evaluates stage forces and moments at segment-local offset `tau_s` (`epoch = anchor + tau_s`).
    fn evaluate_stage(
        &self,
        tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench;

    /// Fallible stage evaluation returning [`GravityError`] when ephemeris or frame lookup
    /// inside the control law fails at `epoch`.
    fn evaluate_stage_checked(
        &self,
        tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> Result<PlantStageWrench, GravityError> {
        Ok(self.evaluate_stage(tau_s, epoch, state))
    }
}

/// Ballistic coast law applying zero thrust and zero control torque.
#[derive(Clone, Copy, Debug, Default)]
pub struct CoastControlLaw;

impl ContinuousControlLaw for CoastControlLaw {
    fn evaluate_stage(
        &self,
        _tau: f64,
        _ep: TdbEpoch,
        _st: &ContinuousPlantState,
    ) -> PlantStageWrench {
        PlantStageWrench::default()
    }
}

/// Constant body-frame thrust + wheel-torque law for verification and finite-burn segments.
#[derive(Clone, Copy, Debug)]
pub struct ConstantWrenchLaw {
    /// Constant wrench applied at every stage.
    pub wrench: PlantStageWrench,
}

impl ContinuousControlLaw for ConstantWrenchLaw {
    fn evaluate_stage(
        &self,
        _tau: f64,
        _ep: TdbEpoch,
        _st: &ContinuousPlantState,
    ) -> PlantStageWrench {
        self.wrench
    }
}

/// Evaluates the 17-component continuous plant derivative $\dot{\mathbf{y}}$ at `(tau_s, epoch)`.
#[allow(clippy::too_many_arguments)]
pub fn evaluate_plant_ode_rhs(
    tau_s: f64,
    epoch: TdbEpoch,
    state: &ContinuousPlantState,
    mass_model: &MassPropertiesModel,
    propellant_mode: PropellantMode,
    rw_mode: ReactionWheelMode,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    control: &dyn ContinuousControlLaw,
) -> Result<([f64; PLANT_ODE_DIM], PropagationAudit), GravityError> {
    let a_grav_icrf = force_model.relative_gravitational_accel_icrf(
        state.primary_body,
        state.pos_rel_icrf_m,
        epoch,
        ephemeris,
        orientation,
    )?;

    let w_stage = control.evaluate_stage_checked(tau_s, epoch, state)?;
    let mut audit = w_stage.stage_audit;
    let (main_f, rcs_f, dm_dt) = resolve_propellant_flow(
        state.propellant_mass_kg,
        propellant_mode,
        &w_stage,
        &mut audit,
    );

    let m_total = mass_model.total_mass_kg(state.propellant_mass_kg);
    let f_body_total = main_f + rcs_f + w_stage.env_force_body_n;
    let q_unit = quat_normalize_portable(state.q_body_to_icrf);
    let acc_rel_icrf = a_grav_icrf + quat_rotate_vec_portable(q_unit, f_body_total) / m_total;

    let (tau_ctrl_body, dh_rw_dt) = resolve_rw_torque(
        rw_mode,
        state.h_rw_body_nms,
        w_stage.rw_torque_cmd_body_nm,
        &mut audit,
    );
    let i_mat = mass_model.inertia_tensor(state.propellant_mass_kg);
    let i_dot = mass_model.uncompensated_inertia_rate(dm_dt);
    let omega = state.omega_body_rad_s;
    let gyro = omega.cross(i_mat * omega + state.h_rw_body_nms);
    let omega_dot =
        i_mat.inverse() * (w_stage.ext_torque_body_nm + tau_ctrl_body - i_dot * omega - gyro);
    let q_dot = quat_omega_derivative(state.q_body_to_icrf, omega);
    let v = state.vel_rel_icrf_m_s;

    Ok((
        [
            v.x,
            v.y,
            v.z,
            acc_rel_icrf.x,
            acc_rel_icrf.y,
            acc_rel_icrf.z,
            q_dot.x,
            q_dot.y,
            q_dot.z,
            q_dot.w,
            omega_dot.x,
            omega_dot.y,
            omega_dot.z,
            dm_dt,
            dh_rw_dt.x,
            dh_rw_dt.y,
            dh_rw_dt.z,
        ],
        audit,
    ))
}

fn resolve_propellant_flow(
    propellant_mass_kg: f64,
    mode: PropellantMode,
    w_stage: &PlantStageWrench,
    audit: &mut PropagationAudit,
) -> (DVec3, DVec3, f64) {
    let has_prop = propellant_mass_kg > 0.0 || mode == PropellantMode::InfiniteConstantMass;
    let main_f = if has_prop {
        w_stage.main_thrust_body_n
    } else {
        DVec3::ZERO
    };
    let rcs_f = if has_prop {
        w_stage.rcs_force_body_n
    } else {
        DVec3::ZERO
    };
    let thrust_mag = sqrt(main_f.length_squared());
    let rcs_mag = sqrt(rcs_f.length_squared());

    let dm_dt = match mode {
        PropellantMode::FiniteVariableMass => {
            if propellant_mass_kg <= 0.0 {
                0.0
            } else {
                let dm_m = if w_stage.main_exhaust_vel_m_s > 0.0 {
                    thrust_mag / w_stage.main_exhaust_vel_m_s
                } else {
                    0.0
                };
                let dm_r = if w_stage.rcs_exhaust_vel_m_s > 0.0 {
                    rcs_mag / w_stage.rcs_exhaust_vel_m_s
                } else {
                    0.0
                };
                -(dm_m + dm_r)
            }
        }
        PropellantMode::InfiniteConstantMass => {
            if thrust_mag > 0.0 || rcs_mag > 0.0 {
                audit.mass_conservation_exempt = true;
            }
            0.0
        }
    };
    (main_f, rcs_f, dm_dt)
}

fn resolve_rw_torque(
    mode: ReactionWheelMode,
    h_rw: DVec3,
    cmd: DVec3,
    audit: &mut PropagationAudit,
) -> (DVec3, DVec3) {
    match mode {
        ReactionWheelMode::ConserveMomentum {
            h_max_nms,
            tau_max_nm,
        } => {
            let mut tau = clamp_vec_norm(cmd, tau_max_nm);
            let h_norm = sqrt(h_rw.length_squared());
            let onset = 0.90 * h_max_nms;
            if h_max_nms > 0.0 && h_norm > onset {
                let h_hat = h_rw / h_norm;
                let dh_radial = (-tau).dot(h_hat);
                if dh_radial > 0.0 {
                    let u = ((h_norm - onset) / (0.10 * h_max_nms)).clamp(0.0, 1.0);
                    let smooth = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
                    tau += (smooth * dh_radial) * h_hat;
                }
            }
            (tau, -tau)
        }
        ReactionWheelMode::ExternalMagicTorque { tau_max_nm } => {
            let tau = clamp_vec_norm(cmd, tau_max_nm);
            if tau.length_squared() > 0.0 {
                audit.angular_momentum_exempt = true;
            }
            (tau, DVec3::ZERO)
        }
    }
}
