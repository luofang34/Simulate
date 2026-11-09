//! `Raketa` `[0°, 180°]` full-envelope aerodynamics, fin ground-lock (`±18°` -> `0°`),
//! 3-axis CG translational RCS (`±0.25G`), reaction wheels ($h_{\max} = 5\times 10^6\text{ N}\cdot\text{m}\cdot\text{s}$),
//! and `ContinuousControlLaw` plant integration.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_atmosphere::{AtmosphereField, MarsAtmosphere, ReentryAtmosphere, VacuumAtmosphere};
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime};
use sim_orbital::{
    BodyOrientationCatalog, ContinuousControlLaw, ContinuousPlantState, FOUR_G_PROPER_ACCEL_M_S2,
    MassPropertiesModel, NaifId, PlantStageWrench, PropagationAudit, ReactionWheelMode,
    STANDARD_GRAVITY_G0_M_S2, TdbEpoch, clamp_vec_norm, quat_rotate_vec_portable,
};

use crate::limiter::{
    ProperAccelLimiterInput, ProperAccelLimiterOutput, limit_main_thrust_proper_accel,
};

/// Constant total mass of `Raketa` (`kg`).
pub const RAKETA_MASS_KG: f64 = 12_000.0;

/// Principal moments of inertia `[Ixx, Iyy, Izz]` about `Raketa`'s CG (`kg*m^2`).
pub const RAKETA_INERTIA_DIAGONAL_KG_M2: [f64; 3] = [85_000.0, 18_000.0, 85_000.0];

/// Height of `Raketa`'s CG along body `+Y` above the USD origin (`m`).
pub const RAKETA_CG_Y_M: f64 = 4.2;

/// Aerodynamic reference cross-sectional area $S_{\text{ref}}$ (`m^2`).
pub const RAKETA_REF_AREA_M2: f64 = 3.80;

/// Aerodynamic reference hull length $L_{\text{ref}}$ (`m`).
pub const RAKETA_REF_LENGTH_M: f64 = 11.02;

/// Maximum per-axis CG translational RCS acceleration (`±0.25G = 2.4516625 m/s^2`).
pub const RAKETA_RCS_MAX_ACCEL_M_S2: f64 = 0.25 * STANDARD_GRAVITY_G0_M_S2;

/// Reaction-wheel angular-momentum capacity $h_{\max}$ (`N*m*s`).
pub const RAKETA_RW_MAX_MOMENTUM_NMS: f64 = 5.0e6;

/// Reaction-wheel peak control torque $\tau_{\max}$ (`N*m`).
pub const RAKETA_RW_MAX_TORQUE_NM: f64 = 5.0e4;

/// Maximum aerodynamic fin deflection (`deg`).
pub const RAKETA_FIN_MAX_DEFLECTION_DEG: f64 = 18.0;

/// Radar altitude (`m` AGL) below which landing mode locks all 4 fins to `0.0°`.
pub const RAKETA_FIN_LANDING_LOCK_ALT_M: f64 = 50.0;

/// Returns `Raketa`'s canonical [`MassPropertiesModel`].
#[must_use]
pub fn raketa_mass_model() -> MassPropertiesModel {
    MassPropertiesModel::diagonal(RAKETA_MASS_KG, RAKETA_INERTIA_DIAGONAL_KG_M2, [0.0; 3])
}

/// Returns `Raketa`'s default momentum-conserving [`ReactionWheelMode`].
#[must_use]
pub const fn raketa_default_rw_mode() -> ReactionWheelMode {
    ReactionWheelMode::ConserveMomentum {
        h_max_nms: RAKETA_RW_MAX_MOMENTUM_NMS,
        tau_max_nm: RAKETA_RW_MAX_TORQUE_NM,
    }
}

/// Resolves effective fin deflections (`deg` in `[-18.0, +18.0]`) and returns `(effective_deg, locked_neutral)`.
///
/// Fins center and lock strictly to `0.0°` whenever on pad (`is_on_pad`), weight-on-fins (`weight_on_fins`),
/// or in landing mode below `50.0 m` radar altitude (`is_landing_mode && altitude_agl_m < 50.0`).
#[must_use]
pub fn resolve_fin_deflections_deg(
    commanded_deg: [f64; 4],
    is_on_pad: bool,
    weight_on_fins: bool,
    is_landing_mode: bool,
    altitude_agl_m: f64,
) -> ([f64; 4], bool) {
    let locked = is_on_pad
        || weight_on_fins
        || (is_landing_mode && altitude_agl_m < RAKETA_FIN_LANDING_LOCK_ALT_M);
    if locked {
        return ([0.0; 4], true);
    }
    let mut out = [0.0_f64; 4];
    for (dst, &cmd) in out.iter_mut().zip(commanded_deg.iter()) {
        *dst = cmd.clamp(
            -RAKETA_FIN_MAX_DEFLECTION_DEG,
            RAKETA_FIN_MAX_DEFLECTION_DEG,
        );
    }
    (out, false)
}

/// Evaluates 3-axis CG translational RCS force (`N`, zero parasitic torque) from normalized
/// `[-1.0, +1.0]` body-axis commands.
#[must_use]
pub fn evaluate_cg_rcs_force_body_n(rcs_cmd_unit_body: DVec3) -> DVec3 {
    let clamped = DVec3::new(
        rcs_cmd_unit_body.x.clamp(-1.0, 1.0),
        rcs_cmd_unit_body.y.clamp(-1.0, 1.0),
        rcs_cmd_unit_body.z.clamp(-1.0, 1.0),
    );
    clamped * (RAKETA_MASS_KG * RAKETA_RCS_MAX_ACCEL_M_S2)
}

/// Aerodynamic force, moment, dynamic pressure, Mach number, and total angle of attack (`[0, pi]`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RaketaAeroOutput {
    /// Aerodynamic force in vessel body coordinates (`N`, `+Y` along nose).
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub force_body_n: DVec3,
    /// Aerodynamic moment about CG in vessel body coordinates (`N*m`).
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub torque_body_nm: DVec3,
    /// Dynamic pressure $q_\infty = \frac{1}{2}\rho V^2$ (`Pa`).
    pub q_inf_pa: f64,
    /// Freestream Mach number.
    pub mach: f64,
    /// Total angle of attack $\alpha_{\text{tot}} \in [0, \pi]$ (`rad`, `0` = nose-first, `pi` = tail-first).
    pub alpha_total_rad: f64,
}

/// Evaluates `Raketa`'s `[0°, 180°]` full-envelope axisymmetric + 4-fin aerodynamic wrench.
#[must_use]
pub fn evaluate_raketa_aerodynamics(
    vel_air_rel_body_m_s: DVec3,
    omega_body_rad_s: DVec3,
    density_kg_m3: f64,
    speed_of_sound_m_s: f64,
    effective_fin_angles_deg: [f64; 4],
) -> RaketaAeroOutput {
    let v_sq = vel_air_rel_body_m_s.length_squared();
    let v_mag = libm::sqrt(v_sq);
    let rho = density_kg_m3.max(0.0);
    let q_inf = 0.5 * rho * v_sq;
    if q_inf <= 1.0e-9 || v_mag <= 1.0e-6 {
        return RaketaAeroOutput {
            force_body_n: DVec3::ZERO,
            torque_body_nm: DVec3::ZERO,
            q_inf_pa: 0.0,
            mach: 0.0,
            alpha_total_rad: 0.0,
        };
    }

    let u_vel = vel_air_rel_body_m_s / v_mag;
    let cos_alpha = u_vel.y.clamp(-1.0, 1.0);
    let alpha_rad = libm::acos(cos_alpha);
    let sin_alpha = libm::sqrt((1.0 - cos_alpha * cos_alpha).max(0.0));
    let mach = v_mag / speed_of_sound_m_s.max(50.0);

    // Axial drag coefficient: streamlined nose-first (~0.28 subsonic, ~0.52 supersonic),
    // blunt truncated-plug base tail-first (~1.25 for retro-entry aerocapture).
    let wave_bump = 0.24 * (1.0 / (1.0 + (mach - 1.15) * (mach - 1.15)));
    let cd_nose = 0.26 + wave_bump;
    let cd_tail = 1.28 + 0.18 * wave_bump;
    let tail_blend = 0.5 * (1.0 - cos_alpha);
    let cd_axial = cd_nose * (1.0 - tail_blend) + cd_tail * tail_blend;
    let cn_crossflow = 1.85 * sin_alpha * sin_alpha.max(0.15);

    let f_drag = -q_inf * RAKETA_REF_AREA_M2 * (cd_axial + 0.65 * sin_alpha * sin_alpha) * u_vel;
    let u_lat = DVec3::new(u_vel.x, 0.0, u_vel.z);
    let u_lat_norm = libm::sqrt(u_lat.length_squared());
    let f_normal = if u_lat_norm > 1.0e-6 {
        -q_inf * RAKETA_REF_AREA_M2 * cn_crossflow * (u_lat / u_lat_norm)
    } else {
        DVec3::ZERO
    };

    // Fin control authority (Fin 0: +X, Fin 1: -Z, Fin 2: -X, Fin 3: +Z):
    let [d0, d1, d2, d3] = effective_fin_angles_deg;
    let pitch_fin_rad = 0.5 * (d0 - d2).to_radians();
    let yaw_fin_rad = 0.5 * (d1 - d3).to_radians();
    let roll_fin_rad = 0.25 * (d0 + d1 + d2 + d3).to_radians();

    let fin_q_scale = q_inf * RAKETA_REF_AREA_M2 * 0.42;
    let f_fin = DVec3::new(-fin_q_scale * yaw_fin_rad, 0.0, fin_q_scale * pitch_fin_rad);
    let f_total = f_drag + f_normal + f_fin;

    // Aerodynamic center at y = 2.76 m (1.44 m aft of CG at y = 4.20 m):
    let r_cp = DVec3::new(0.0, 2.764_897 - RAKETA_CG_Y_M, 0.0);
    let damp_scale =
        q_inf * RAKETA_REF_AREA_M2 * RAKETA_REF_LENGTH_M * (RAKETA_REF_LENGTH_M / v_mag.max(10.0));
    let tau_damp = -0.35 * damp_scale * omega_body_rad_s;
    let tau_fin = q_inf
        * RAKETA_REF_AREA_M2
        * RAKETA_REF_LENGTH_M
        * DVec3::new(
            0.28 * pitch_fin_rad,
            0.14 * roll_fin_rad,
            0.28 * yaw_fin_rad,
        );
    let tau_total = r_cp.cross(f_normal + f_fin) + tau_damp + tau_fin;

    RaketaAeroOutput {
        force_body_n: f_total,
        torque_body_nm: tau_total,
        q_inf_pa: q_inf,
        mach,
        alpha_total_rad: alpha_rad,
    }
}

/// Samples atmospheric `(density_kg_m3, pressure_pa, speed_of_sound_m_s)` for `body` (`EARTH`, `MARS`, or vacuum).
#[must_use]
pub fn sample_planetary_atmosphere(body: NaifId, geodetic: &Geodetic) -> (f64, f64, f64) {
    let t_utc = UtcTime::from_unix_seconds(1_780_000_000.0);
    let samp = match body {
        NaifId::EARTH => ReentryAtmosphere
            .sample(geodetic, t_utc)
            .unwrap_or_else(|_| {
                VacuumAtmosphere
                    .sample(geodetic, t_utc)
                    .unwrap_or_else(|_| unreachable_vacuum())
            }),
        NaifId::MARS | NaifId::MARS_BARYCENTER => MarsAtmosphere::new()
            .sample(geodetic, t_utc)
            .unwrap_or_else(|_| {
                VacuumAtmosphere
                    .sample(geodetic, t_utc)
                    .unwrap_or_else(|_| unreachable_vacuum())
            }),
        _ => VacuumAtmosphere
            .sample(geodetic, t_utc)
            .unwrap_or_else(|_| unreachable_vacuum()),
    };
    (
        samp.density_kg_m3,
        samp.pressure_pa,
        samp.speed_of_sound_m_s.max(50.0),
    )
}

const fn unreachable_vacuum() -> sim_atmosphere::AtmosphereSample {
    sim_atmosphere::AtmosphereSample {
        temperature_k: 2.725,
        pressure_pa: 0.0,
        density_kg_m3: 0.0,
        speed_of_sound_m_s: 0.0,
        wind_ned_m_s: DVec3::ZERO,
    }
}

/// Complete closed-loop stage control law for `Raketa` combining `[0°, 180°]` planetary aerodynamics,
/// fin ground-lock, 3-axis CG RCS, reaction wheels, and the exact quadratic `4.0G` limiter.
pub struct RaketaStageLaw<'a> {
    /// Commanded main-engine specific thrust (`m/s^2`, `0..=4.0G`).
    pub commanded_main_accel_m_s2: f64,
    /// Desired unit body `+Y` pointing direction in `ICRF`.
    pub desired_body_y_icrf: DVec3,
    /// Normalized `[-1, 1]` 3-axis CG translational RCS command in body axes.
    pub rcs_cmd_unit_body: DVec3,
    /// Commanded fin angles (`deg`).
    pub fin_cmd_deg: [f64; 4],
    /// Whether the vessel is on pad or in `< 50 m` landing mode.
    pub is_landing_or_pad_mode: bool,
    /// Planetary orientation catalog for wind-relative airspeed and geodetic altitude.
    pub orientation: &'a dyn BodyOrientationCatalog,
}

impl RaketaStageLaw<'_> {
    /// Evaluates the full `Raketa` stage wrench and returns `(PlantStageWrench, ProperAccelLimiterOutput, RaketaAeroOutput, bool)`.
    #[must_use]
    pub fn evaluate_detailed(
        &self,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> (
        PlantStageWrench,
        ProperAccelLimiterOutput,
        RaketaAeroOutput,
        bool,
    ) {
        let (geo, v_air_body) = compute_body_geodetic_and_air_vel(self.orientation, epoch, state);
        let (rho, _p_pa, a_sound) = sample_planetary_atmosphere(state.primary_body, &geo);
        let (eff_fins, fin_locked) = resolve_fin_deflections_deg(
            self.fin_cmd_deg,
            false,
            false,
            self.is_landing_or_pad_mode,
            geo.height_m,
        );
        let aero = evaluate_raketa_aerodynamics(
            v_air_body,
            state.omega_body_rad_s,
            rho,
            a_sound,
            eff_fins,
        );
        let rcs_f_body = evaluate_cg_rcs_force_body_n(self.rcs_cmd_unit_body);
        let ext_a_body = (rcs_f_body + aero.force_body_n) / RAKETA_MASS_KG;

        let lim = limit_main_thrust_proper_accel(&ProperAccelLimiterInput {
            commanded_thrust_accel_m_s2: self.commanded_main_accel_m_s2,
            unit_thrust_axis_body: DVec3::Y,
            ext_specific_force_body_m_s2: ext_a_body,
            max_proper_accel_m_s2: FOUR_G_PROPER_ACCEL_M_S2,
        });

        let tau_rw = compute_pointing_rw_torque(state, self.desired_body_y_icrf);
        let mut audit = PropagationAudit::default();
        if lim.effective_thrust_accel_m_s2 > 0.0 || rcs_f_body.length_squared() > 0.0 {
            audit.mass_conservation_exempt = true;
        }
        if lim.exceedance_infeasible {
            audit.proper_accel_exceedance = true;
        }

        let wrench = PlantStageWrench {
            main_thrust_body_n: DVec3::new(
                0.0,
                lim.effective_thrust_accel_m_s2 * RAKETA_MASS_KG,
                0.0,
            ),
            main_exhaust_vel_m_s: 4_500.0,
            rcs_force_body_n: rcs_f_body,
            rcs_exhaust_vel_m_s: 3_000.0,
            env_force_body_n: aero.force_body_n,
            ext_torque_body_nm: aero.torque_body_nm,
            rw_torque_cmd_body_nm: tau_rw,
            stage_audit: audit,
        };
        (wrench, lim, aero, fin_locked)
    }
}

fn compute_body_geodetic_and_air_vel(
    orientation: &dyn BodyOrientationCatalog,
    epoch: TdbEpoch,
    state: &ContinuousPlantState,
) -> (Geodetic, DVec3) {
    let ell = match state.primary_body {
        NaifId::EARTH => BodyEllipsoid::WGS84,
        NaifId::MARS | NaifId::MARS_BARYCENTER => BodyEllipsoid::MARS_IAU,
        _ => BodyEllipsoid::MOON_IAU,
    };
    if let Ok(rot) = orientation.body_rotation(state.primary_body, epoch) {
        let p_fixed = rot.r_icrf_to_fixed * state.pos_rel_icrf_m;
        let v_fixed =
            rot.r_icrf_to_fixed * state.vel_rel_icrf_m_s - rot.omega_body_rad_s.cross(p_fixed);
        let r_f2i = rot.r_icrf_to_fixed.transpose();
        let v_air_icrf = r_f2i * v_fixed;
        let q_unit = sim_orbital::quat_normalize_portable(state.q_body_to_icrf);
        let v_b = quat_rotate_vec_portable(q_unit.conjugate(), v_air_icrf);
        (ell.fixed_to_geodetic(p_fixed), v_b)
    } else {
        let r = libm::sqrt(state.pos_rel_icrf_m.length_squared());
        (
            Geodetic {
                lat_rad: 0.0,
                lon_rad: 0.0,
                height_m: r - ell.semi_major_m,
            },
            DVec3::ZERO,
        )
    }
}

fn compute_pointing_rw_torque(state: &ContinuousPlantState, desired_body_y_icrf: DVec3) -> DVec3 {
    let q_unit = sim_orbital::quat_normalize_portable(state.q_body_to_icrf);
    let cur_y_icrf = quat_rotate_vec_portable(q_unit, DVec3::Y);
    let des_norm = libm::sqrt(desired_body_y_icrf.length_squared());
    let des_y = if des_norm > 1.0e-9 {
        desired_body_y_icrf / des_norm
    } else {
        cur_y_icrf
    };
    let err_icrf = cur_y_icrf.cross(des_y);
    let err_body = quat_rotate_vec_portable(q_unit.conjugate(), err_icrf);
    clamp_vec_norm(
        1.5e4 * err_body - 1.8e4 * state.omega_body_rad_s,
        RAKETA_RW_MAX_TORQUE_NM,
    )
}

impl ContinuousControlLaw for RaketaStageLaw<'_> {
    fn evaluate_stage(
        &self,
        _tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        self.evaluate_detailed(epoch, state).0
    }
}
