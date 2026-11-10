//! Explicit `Navigate` (outer-loop `f64` trajectory guidance) to `Aviate` (inner-loop `f32`/`1 kHz`
//! attitude stabilization, proper-acceleration limiting, `< 50 m` fin ground-lock, and RW/RCS
//! allocation) GNC boundary for `Raketa` (`vehicles/raketa`).
//!
//! Mirrors the four-system architecture contract:
//! - **Outer loop (`Navigate`)**: computes [`RaketaOuterGuidanceSetpoint`] (`ThrustVector` for
//!   orbital burns, gravity-turn ascent, and ZEM/ZEV powered descent; `EntryCorridor` for
//!   atmospheric entry corridor guidance; `RendezvousTranslation` for 6-DOF proximity RCS).
//! - **Inner loop (`Aviate` `spacecraft` feature)**: converts the outer-loop setpoint into
//!   [`RaketaStageLaw`] and [`RaketaInnerAllocation`], enforcing `< 50 m` fin ground-lock,
//!   `<= 4.0G` quadratic proper-acceleration limiting, reaction-wheel saturation handoff to
//!   rotational RCS (`>= 85%` of $h_{\max}$), and deadbanded CG translational RCS allocation.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_orbital::{
    AscentGuidanceOutput, AutolandGuidanceOutput, BodyOrientationCatalog, EntryCorridorOutput,
    FOUR_G_PROPER_ACCEL_M_S2, FiniteBurnPlan,
};

use crate::aero_actuators::{RAKETA_RW_MAX_MOMENTUM_NMS, RAKETA_RW_MAX_TORQUE_NM, RaketaStageLaw};

/// Reaction-wheel momentum fraction above which rotational RCS assists with momentum dumping
/// (matches `Aviate`'s `RW_SATURATION_THRESHOLD = 0.85`).
pub const RAKETA_RW_DUMP_THRESHOLD_FRAC: f64 = 0.85;

/// Translational RCS velocity-error deadband (`m/s`, matches `Aviate`'s `RCS_VEL_DEADBAND_MPS = 0.02`).
pub const RAKETA_RCS_VEL_DEADBAND_M_S: f64 = 0.02;

/// Translational RCS proportional gain (`s^-1`, matches `Aviate`'s `RCS_VEL_KP = 1.2`).
pub const RAKETA_RCS_VEL_KP_PER_S: f64 = 1.2;

/// Outer-loop `f64` trajectory guidance setpoint emitted by `Navigate`-tier guidance laws
/// (`burn`, `rendezvous`, `entry`, `powered_descent`, and ascent).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum RaketaOuterGuidanceSetpoint {
    /// Directed specific-force burn or powered-descent setpoint (`Navigate` `GuidanceSetpoint::ThrustVector`).
    ThrustVector {
        /// Desired unit thrust direction in `ICRF`.
        #[serde(with = "sim_orbital::serde_dvec3")]
        direction_unit_icrf: DVec3,
        /// Commanded specific force (`m/s^2`, `>= 0.0`).
        specific_force_m_s2: f64,
        /// Remaining velocity-to-go (`m/s`, `>= 0.0`).
        velocity_to_go_m_s: f64,
        /// Whether engine cutoff has been reached.
        cutoff: bool,
        /// Nominal aerodynamic fin trim command (`deg`) when outside the `< 50 m` ground-lock regime.
        fin_trim_deg: [f64; 4],
        /// Whether the vehicle is on pad or in terminal landing mode (`< 50 m` fin ground-lock active).
        is_landing_or_pad_mode: bool,
    },
    /// Atmospheric entry corridor guidance setpoint (`Navigate` `GuidanceSetpoint::EntryBank`).
    EntryCorridor {
        /// Desired retro-entry attitude direction (`+Y` body axis in `ICRF`).
        #[serde(with = "sim_orbital::serde_dvec3")]
        attitude_dir_icrf: DVec3,
        /// Commanded bank angle (`rad` in `[-pi, pi]`).
        bank_rad: f64,
        /// Reference corridor drag deceleration (`m/s^2`).
        reference_drag_m_s2: f64,
        /// Sensed minus reference drag error (`m/s^2`).
        drag_error_m_s2: f64,
        /// Supersonic retro-propulsion specific thrust (`m/s^2`) when entry drag exceeds corridor limits.
        retro_thrust_accel_m_s2: f64,
        /// Aerodynamic fin trim deflections (`deg`).
        fin_trim_deg: [f64; 4],
    },
    /// 6-DOF proximity rendezvous velocity correction (`Navigate` `GuidanceSetpoint::CartesianVelocity`).
    RendezvousTranslation {
        /// Velocity error (`target - chaser`, `m/s`) expressed in `Raketa` body coordinates.
        #[serde(with = "sim_orbital::serde_dvec3")]
        velocity_error_body_m_s: DVec3,
        /// Desired docking/pointing axis (`+Y` body axis in `ICRF`).
        #[serde(with = "sim_orbital::serde_dvec3")]
        desired_body_y_icrf: DVec3,
    },
}

/// Inner-loop reaction-wheel and RCS allocation diagnostics matching `Aviate`'s `spacecraft` mixer.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RaketaInnerAllocation {
    /// Reaction-wheel torque command in body axes (`N*m`).
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub rw_torque_cmd_body_nm: DVec3,
    /// Normalized `[-1, 1]` rotational RCS momentum-dumping assist command in body axes.
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub rcs_rotational_dump_unit_body: DVec3,
    /// Normalized `[-1, 1]` 3-axis CG translational RCS command in body axes.
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub rcs_translational_unit_body: DVec3,
    /// Whether any reaction-wheel axis exceeded the `85%` saturation threshold.
    pub rw_desaturation_active: bool,
}

impl RaketaOuterGuidanceSetpoint {
    /// Builds an outer-loop [`RaketaOuterGuidanceSetpoint::ThrustVector`] from gravity-turn ascent guidance.
    #[must_use]
    pub const fn from_ascent(ascent: &AscentGuidanceOutput, fin_trim_deg: [f64; 4]) -> Self {
        Self::ThrustVector {
            direction_unit_icrf: ascent.thrust_dir_icrf,
            specific_force_m_s2: if ascent.orbit_achieved {
                0.0
            } else {
                ascent.commanded_accel_m_s2
            },
            velocity_to_go_m_s: 0.0,
            cutoff: ascent.orbit_achieved,
            fin_trim_deg,
            is_landing_or_pad_mode: false,
        }
    }

    /// Builds an outer-loop [`RaketaOuterGuidanceSetpoint::ThrustVector`] from a finite orbital burn plan.
    #[must_use]
    pub const fn from_finite_burn(plan: &FiniteBurnPlan) -> Self {
        Self::ThrustVector {
            direction_unit_icrf: plan.unit_dir_icrf,
            specific_force_m_s2: plan.accel_m_s2,
            velocity_to_go_m_s: plan.delta_v_m_s,
            cutoff: plan.duration_s <= 0.0 || plan.accel_m_s2 <= 0.0,
            fin_trim_deg: [0.0; 4],
            is_landing_or_pad_mode: false,
        }
    }

    /// Builds an outer-loop [`RaketaOuterGuidanceSetpoint::EntryCorridor`] from atmospheric entry guidance.
    #[must_use]
    pub const fn from_entry_corridor(
        entry: &EntryCorridorOutput,
        sensed_drag_m_s2: f64,
        reference_drag_m_s2: f64,
        fin_trim_deg: [f64; 4],
    ) -> Self {
        Self::EntryCorridor {
            attitude_dir_icrf: entry.attitude_dir_icrf,
            bank_rad: entry.bank_angle_rad,
            reference_drag_m_s2,
            drag_error_m_s2: sensed_drag_m_s2 - reference_drag_m_s2,
            retro_thrust_accel_m_s2: entry.retro_thrust_accel_m_s2,
            fin_trim_deg,
        }
    }

    /// Builds an outer-loop [`RaketaOuterGuidanceSetpoint::ThrustVector`] from ZEM/ZEV powered-descent autoland.
    #[must_use]
    pub fn from_powered_descent(
        autoland: &AutolandGuidanceOutput,
        velocity_to_go_m_s: f64,
        fin_trim_deg: [f64; 4],
    ) -> Self {
        Self::ThrustVector {
            direction_unit_icrf: autoland.thrust_dir_icrf,
            specific_force_m_s2: autoland.commanded_accel_m_s2.min(FOUR_G_PROPER_ACCEL_M_S2),
            velocity_to_go_m_s: velocity_to_go_m_s.max(0.0),
            cutoff: autoland.commanded_accel_m_s2 <= 0.0,
            fin_trim_deg,
            is_landing_or_pad_mode: true,
        }
    }

    /// Projects this outer-loop `Navigate` guidance setpoint into `Raketa`'s inner-loop
    /// [`RaketaStageLaw`] (`Aviate` `spacecraft` control/mixing boundary).
    #[must_use]
    pub fn into_stage_law<'a>(
        self,
        orientation: &'a dyn BodyOrientationCatalog,
    ) -> RaketaStageLaw<'a> {
        match self {
            Self::ThrustVector {
                direction_unit_icrf,
                specific_force_m_s2,
                cutoff,
                fin_trim_deg,
                is_landing_or_pad_mode,
                ..
            } => RaketaStageLaw {
                commanded_main_accel_m_s2: if cutoff {
                    0.0
                } else {
                    specific_force_m_s2.max(0.0)
                },
                desired_body_y_icrf: direction_unit_icrf,
                rcs_cmd_unit_body: DVec3::ZERO,
                fin_cmd_deg: fin_trim_deg,
                is_landing_or_pad_mode,
                orientation,
            },
            Self::EntryCorridor {
                attitude_dir_icrf,
                retro_thrust_accel_m_s2,
                fin_trim_deg,
                ..
            } => RaketaStageLaw {
                commanded_main_accel_m_s2: retro_thrust_accel_m_s2.max(0.0),
                desired_body_y_icrf: attitude_dir_icrf,
                rcs_cmd_unit_body: DVec3::ZERO,
                fin_cmd_deg: fin_trim_deg,
                is_landing_or_pad_mode: false,
                orientation,
            },
            Self::RendezvousTranslation {
                velocity_error_body_m_s,
                desired_body_y_icrf,
            } => RaketaStageLaw {
                commanded_main_accel_m_s2: 0.0,
                desired_body_y_icrf,
                rcs_cmd_unit_body: allocate_translational_rcs_unit(velocity_error_body_m_s),
                fin_cmd_deg: [0.0; 4],
                is_landing_or_pad_mode: false,
                orientation,
            },
        }
    }
}

/// Allocates normalized `[-1.0, +1.0]` 3-axis CG translational RCS commands from body-frame
/// velocity error (`m/s`), matching `Aviate`'s `allocate_translational_rcs`.
#[must_use]
pub fn allocate_translational_rcs_unit(velocity_error_body_m_s: DVec3) -> DVec3 {
    let map_axis = |dv: f64| -> f64 {
        if !dv.is_finite() || dv.abs() < RAKETA_RCS_VEL_DEADBAND_M_S {
            0.0
        } else {
            (dv * RAKETA_RCS_VEL_KP_PER_S).clamp(-1.0, 1.0)
        }
    };
    DVec3::new(
        map_axis(velocity_error_body_m_s.x),
        map_axis(velocity_error_body_m_s.y),
        map_axis(velocity_error_body_m_s.z),
    )
}

/// Allocates reaction-wheel control torque (`N*m`) vs. rotational RCS momentum-dumping assist
/// (`[-1.0, +1.0]`), matching `Aviate`'s `allocate_rw_and_rotational_rcs`.
#[must_use]
pub fn allocate_rw_and_rotational_rcs_dump(
    desired_torque_body_nm: DVec3,
    rw_momentum_body_nms: DVec3,
    velocity_error_body_m_s: DVec3,
) -> RaketaInnerAllocation {
    let threshold_nms = RAKETA_RW_DUMP_THRESHOLD_FRAC * RAKETA_RW_MAX_MOMENTUM_NMS;
    let map_axis = |tau_nm: f64, h_nms: f64| -> (f64, f64, bool) {
        if h_nms.abs() >= threshold_nms {
            let rcs_unit =
                (tau_nm / RAKETA_RW_MAX_TORQUE_NM - 0.25 * h_nms.signum()).clamp(-1.0, 1.0);
            (0.0, rcs_unit, true)
        } else {
            (
                tau_nm.clamp(-RAKETA_RW_MAX_TORQUE_NM, RAKETA_RW_MAX_TORQUE_NM),
                0.0,
                false,
            )
        }
    };
    let (rw_x, rcs_x, sat_x) = map_axis(desired_torque_body_nm.x, rw_momentum_body_nms.x);
    let (rw_y, rcs_y, sat_y) = map_axis(desired_torque_body_nm.y, rw_momentum_body_nms.y);
    let (rw_z, rcs_z, sat_z) = map_axis(desired_torque_body_nm.z, rw_momentum_body_nms.z);
    RaketaInnerAllocation {
        rw_torque_cmd_body_nm: DVec3::new(rw_x, rw_y, rw_z),
        rcs_rotational_dump_unit_body: DVec3::new(rcs_x, rcs_y, rcs_z),
        rcs_translational_unit_body: allocate_translational_rcs_unit(velocity_error_body_m_s),
        rw_desaturation_active: sat_x || sat_y || sat_z,
    }
}

#[cfg(test)]
mod tests {
    use glam::{DQuat, DVec3};
    use sim_orbital::{
        AscentGuidanceOutput, AutolandGuidanceOutput, BuiltinSolarSystem2026_2028,
        ContinuousPlantState, EntryCorridorOutput, FOUR_G_PROPER_ACCEL_M_S2, FiniteBurnPlan,
        NaifId, STANDARD_GRAVITY_G0_M_S2, TdbEpoch,
    };

    use crate::aero_actuators::RAKETA_RW_MAX_MOMENTUM_NMS;

    use super::{
        RaketaOuterGuidanceSetpoint, allocate_rw_and_rotational_rcs_dump,
        allocate_translational_rcs_unit,
    };

    #[test]
    fn outer_guidance_setpoints_project_to_inner_stage_law_with_fin_lock_and_4g_limit() {
        let Ok(solar) = BuiltinSolarSystem2026_2028::load_verified() else {
            return;
        };
        let t0 = TdbEpoch::new(835_000_000, 0);
        let state = ContinuousPlantState {
            primary_body: NaifId::EARTH,
            pos_rel_icrf_m: DVec3::new(6_378_140.0, 0.0, 0.0),
            vel_rel_icrf_m_s: DVec3::new(-2.0, 0.0, 0.0),
            q_body_to_icrf: DQuat::IDENTITY,
            omega_body_rad_s: DVec3::ZERO,
            propellant_mass_kg: 0.0,
            h_rw_body_nms: DVec3::ZERO,
        };

        let autoland = AutolandGuidanceOutput {
            thrust_dir_icrf: DVec3::X,
            commanded_accel_m_s2: 5.0 * STANDARD_GRAVITY_G0_M_S2,
            lock_fins_neutral: true,
            time_to_go_s: 4.0,
        };
        let sp = RaketaOuterGuidanceSetpoint::from_powered_descent(
            &autoland,
            2.0,
            [15.0, -15.0, 15.0, -15.0],
        );
        let law = sp.into_stage_law(&solar.orientation);
        let (_wrench, lim, _aero, fin_locked) = law.evaluate_detailed(t0, &state);
        assert!(fin_locked);
        assert!(lim.effective_thrust_accel_m_s2 <= FOUR_G_PROPER_ACCEL_M_S2 + 1.0e-9);

        let burn = FiniteBurnPlan::from_impulsive(
            DVec3::new(0.0, 1_200.0, 0.0),
            2.0 * STANDARD_GRAVITY_G0_M_S2,
        );
        let burn_law =
            RaketaOuterGuidanceSetpoint::from_finite_burn(&burn).into_stage_law(&solar.orientation);
        assert!((burn_law.commanded_main_accel_m_s2 - 2.0 * STANDARD_GRAVITY_G0_M_S2).abs() < 1e-9);
        assert!(!burn_law.is_landing_or_pad_mode);

        let ascent = AscentGuidanceOutput {
            thrust_dir_icrf: DVec3::Y,
            commanded_accel_m_s2: 2.5 * STANDARD_GRAVITY_G0_M_S2,
            orbit_achieved: true,
        };
        let cutoff_law = RaketaOuterGuidanceSetpoint::from_ascent(&ascent, [2.0, 0.0, -2.0, 0.0])
            .into_stage_law(&solar.orientation);
        assert_eq!(cutoff_law.commanded_main_accel_m_s2, 0.0);

        let entry = EntryCorridorOutput {
            attitude_dir_icrf: -DVec3::Y,
            retro_thrust_accel_m_s2: 4.0,
            bank_angle_rad: 0.5,
            ready_for_autoland: false,
        };
        let entry_law = RaketaOuterGuidanceSetpoint::from_entry_corridor(
            &entry,
            37.0,
            25.0,
            [4.0, -4.0, 4.0, -4.0],
        )
        .into_stage_law(&solar.orientation);
        assert_eq!(entry_law.commanded_main_accel_m_s2, 4.0);
        assert_eq!(entry_law.desired_body_y_icrf, -DVec3::Y);
    }

    #[test]
    fn rendezvous_rcs_deadband_and_rw_momentum_dump_match_aviate_spacecraft_contract() {
        let Ok(solar) = BuiltinSolarSystem2026_2028::load_verified() else {
            return;
        };
        let rdv_sp = RaketaOuterGuidanceSetpoint::RendezvousTranslation {
            velocity_error_body_m_s: DVec3::new(0.005, 0.5, -1.5),
            desired_body_y_icrf: DVec3::Y,
        };
        let law = rdv_sp.into_stage_law(&solar.orientation);
        assert_eq!(law.commanded_main_accel_m_s2, 0.0);
        assert_eq!(law.rcs_cmd_unit_body.x, 0.0);
        assert!((law.rcs_cmd_unit_body.y - 0.6).abs() < 1e-9);
        assert!((law.rcs_cmd_unit_body.z + 1.0).abs() < 1e-9);

        let unsat = allocate_rw_and_rotational_rcs_dump(
            DVec3::new(10_000.0, -5_000.0, 0.0),
            DVec3::new(0.2 * RAKETA_RW_MAX_MOMENTUM_NMS, 0.0, 0.0),
            DVec3::ZERO,
        );
        assert!(!unsat.rw_desaturation_active);
        assert_eq!(unsat.rw_torque_cmd_body_nm.x, 10_000.0);
        assert_eq!(unsat.rcs_rotational_dump_unit_body, DVec3::ZERO);

        let sat = allocate_rw_and_rotational_rcs_dump(
            DVec3::new(20_000.0, 0.0, 0.0),
            DVec3::new(0.90 * RAKETA_RW_MAX_MOMENTUM_NMS, 0.0, 0.0),
            DVec3::new(0.0, 0.1, 0.0),
        );
        assert!(sat.rw_desaturation_active);
        assert_eq!(sat.rw_torque_cmd_body_nm.x, 0.0);
        assert!(sat.rcs_rotational_dump_unit_body.x != 0.0);
        assert!(sat.rcs_translational_unit_body.y > 0.0);
        assert_eq!(
            allocate_translational_rcs_unit(DVec3::splat(f64::NAN)),
            DVec3::ZERO
        );
    }
}
