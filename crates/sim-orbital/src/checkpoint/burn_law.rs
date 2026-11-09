//! Pure, serializable latched continuous burn/wrench laws and actuator states.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    hash::DomainHasher,
    plant::{
        ContinuousControlLaw, ContinuousPlantState, PlantStageWrench, quat_normalize_portable,
        quat_rotate_vec_portable,
    },
    time::TdbEpoch,
};

/// Pure, serializable latched continuous burn/wrench law evaluated inside RK stages (`&self`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum LatchedBurnLaw {
    /// Ballistic coast (zero control thrust and zero control torque).
    #[default]
    Coast,
    /// Constant body-frame thrust, RCS force, and reaction-wheel torque.
    ConstantBody {
        /// Main-engine thrust in body axes ($\text{N}$).
        main_thrust_body_n: [f64; 3],
        /// Effective main exhaust velocity $v_e$ ($\text{m}/\text{s}$).
        main_exhaust_vel_m_s: f64,
        /// Translational RCS force in body axes ($\text{N}$).
        rcs_force_body_n: [f64; 3],
        /// Effective RCS exhaust velocity $v_e$ ($\text{m}/\text{s}$).
        rcs_exhaust_vel_m_s: f64,
        /// Commanded reaction-wheel torque on the body ($\text{N}\cdot\text{m}$).
        rw_torque_cmd_body_nm: [f64; 3],
    },
    /// Inertially pointed thrust vector rotated into the instantaneous body frame at every RK stage.
    InertialThrust {
        /// Desired unit thrust direction in `ICRF` axes.
        dir_icrf: [f64; 3],
        /// Thrust magnitude ($\text{N}$).
        thrust_n: f64,
        /// Effective exhaust velocity $v_e$ ($\text{m}/\text{s}$).
        exhaust_vel_m_s: f64,
        /// Commanded reaction-wheel torque on the body ($\text{N}\cdot\text{m}$).
        rw_torque_cmd_body_nm: [f64; 3],
    },
}

impl LatchedBurnLaw {
    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        match *self {
            Self::Coast => h.write_u8(0),
            Self::ConstantBody {
                main_thrust_body_n,
                main_exhaust_vel_m_s,
                rcs_force_body_n,
                rcs_exhaust_vel_m_s,
                rw_torque_cmd_body_nm,
            } => {
                h.write_u8(1);
                for v in main_thrust_body_n {
                    h.write_f64_canonical(v);
                }
                h.write_f64_canonical(main_exhaust_vel_m_s);
                for v in rcs_force_body_n {
                    h.write_f64_canonical(v);
                }
                h.write_f64_canonical(rcs_exhaust_vel_m_s);
                for v in rw_torque_cmd_body_nm {
                    h.write_f64_canonical(v);
                }
            }
            Self::InertialThrust {
                dir_icrf,
                thrust_n,
                exhaust_vel_m_s,
                rw_torque_cmd_body_nm,
            } => {
                h.write_u8(2);
                for v in dir_icrf {
                    h.write_f64_canonical(v);
                }
                h.write_f64_canonical(thrust_n);
                h.write_f64_canonical(exhaust_vel_m_s);
                for v in rw_torque_cmd_body_nm {
                    h.write_f64_canonical(v);
                }
            }
        }
    }
}

impl ContinuousControlLaw for LatchedBurnLaw {
    fn evaluate_stage(
        &self,
        _tau_s: f64,
        _epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        match *self {
            Self::Coast => PlantStageWrench::default(),
            Self::ConstantBody {
                main_thrust_body_n,
                main_exhaust_vel_m_s,
                rcs_force_body_n,
                rcs_exhaust_vel_m_s,
                rw_torque_cmd_body_nm,
            } => PlantStageWrench {
                main_thrust_body_n: DVec3::from_array(main_thrust_body_n),
                main_exhaust_vel_m_s,
                rcs_force_body_n: DVec3::from_array(rcs_force_body_n),
                rcs_exhaust_vel_m_s,
                rw_torque_cmd_body_nm: DVec3::from_array(rw_torque_cmd_body_nm),
                ..PlantStageWrench::default()
            },
            Self::InertialThrust {
                dir_icrf,
                thrust_n,
                exhaust_vel_m_s,
                rw_torque_cmd_body_nm,
            } => {
                let q_inv = quat_normalize_portable(state.q_body_to_icrf).conjugate();
                let u_icrf = DVec3::from_array(dir_icrf).normalize_or_zero();
                let f_body = quat_rotate_vec_portable(q_inv, u_icrf * thrust_n.max(0.0));
                PlantStageWrench {
                    main_thrust_body_n: f_body,
                    main_exhaust_vel_m_s: exhaust_vel_m_s,
                    rw_torque_cmd_body_nm: DVec3::from_array(rw_torque_cmd_body_nm),
                    ..PlantStageWrench::default()
                }
            }
        }
    }
}

/// Control-surface, conformal nosecone, and throttle actuator states.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ActuatorState {
    /// Aerodynamic fin deflections `Fin_0..3/Rudder` ($\text{rad}$).
    pub fin_angle_rad: [f64; 4],
    /// Aerodynamic fin angular rates ($\text{rad}/\text{s}$).
    pub fin_rate_rad_s: [f64; 4],
    /// Whether fins are mechanically centered and locked to `0°` (pad / WOF / `< 50 m` landing).
    pub fins_locked_zero: bool,
    /// Conformal nosecone `Carrier` hinge angle ($\text{rad}$).
    pub nose_carrier_rad: f64,
    /// Conformal nosecone `Wrist` hinge angle ($\text{rad}$).
    pub nose_wrist_rad: f64,
    /// Normalized main-engine throttle setting `[0.0, 1.0]`.
    pub throttle_0_to_1: f64,
}

impl ActuatorState {
    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        for v in self.fin_angle_rad {
            h.write_f64_canonical(v);
        }
        for v in self.fin_rate_rad_s {
            h.write_f64_canonical(v);
        }
        h.write_u8(u8::from(self.fins_locked_zero));
        h.write_f64_canonical(self.nose_carrier_rad);
        h.write_f64_canonical(self.nose_wrist_rad);
        h.write_f64_canonical(self.throttle_0_to_1);
    }
}
