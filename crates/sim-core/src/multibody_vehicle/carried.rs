//! Stepping a vehicle's internals while another integrator owns its motion.
//!
//! Orbital propagation carries a vehicle as one rigid body. Its tanks, bus,
//! thrusters and thermal network still have to run, so this advances exactly
//! those with the base state prescribed from outside, and leaves joints where
//! they are: it is only valid while nothing internal reacts on the base
//! faster than the prescribing integrator hears about it.

use glam::DVec3;
use mbd_multibody::PinnedSurfaceRest;

use super::step::{block_mbd, invalid_input, sample_env_and_aero};
use super::step_contacts::{build_telemetry, merge_dynamic_masses};
use super::step_subsystems::run_subsystems_step;
use super::subsystems::{fixed_base_from_vehicle, validate_input};
use super::{MultibodyVehicleBlock, MultibodyVehicleInput};
use crate::{BlockError, VehicleState};

impl MultibodyVehicleBlock {
    /// Advances the subsystems (fluids, electrical bus, propulsion, thermal,
    /// surface erosion) one period with the vehicle held at `prescribed`, the
    /// state of its centre of gravity at the end of the period as another
    /// integrator flew it. Joints do not move and nothing is integrated for
    /// the base; [`Self::telemetry`] reports the thrust and its moment that
    /// the other integrator has to apply. A failed step changes nothing.
    pub fn step_carried(
        &mut self,
        input: &MultibodyVehicleInput,
        prescribed: &VehicleState,
    ) -> Result<(), BlockError> {
        let saved = self.snapshot_for_coord();
        let res = self.run_carried_step(input, prescribed);
        if res.is_err()
            && let Err(error) = self.try_restore(&saved)
        {
            tracing::error!(%error, "multibody rollback failed");
        }
        res
    }

    fn run_carried_step(
        &mut self,
        input: &MultibodyVehicleInput,
        prescribed: &VehicleState,
    ) -> Result<(), BlockError> {
        validate_input(input)?;
        check_prescribed(prescribed)?;
        let body = self.config.body;
        let base = fixed_base_from_vehicle(prescribed, &self.mass);
        let poses = self.tree.evaluate_pose_table().map_err(block_mbd)?;
        let env = sample_env_and_aero(
            &body,
            self.atmosphere.as_ref(),
            &self.aero,
            &poses,
            &base,
            &self.mass,
            self.current_utc(),
            input.control_surfaces_rad,
        )?;
        let sub = run_subsystems_step(
            self.config.period.as_seconds(),
            &mut self.tree,
            &poses,
            &mut self.subsystems,
            &env,
            base,
            &self.mass,
            self.telemetry.specific_force_cg_body,
            DVec3::ZERO,
            &body,
            input,
        )?;
        let dyn_masses =
            merge_dynamic_masses(&sub.fluid_out.mass_contributions, &sub.grain_contribs);
        self.mass = self
            .tree
            .mass_properties_with_contributions(&dyn_masses)
            .map_err(block_mbd)?;
        self.poses = self.tree.evaluate_pose_table().map_err(block_mbd)?;
        // The prescribed point is the centre of gravity, which moves in the
        // root frame as tanks drain, so the root is placed from the new mass.
        self.base_fixed = fixed_base_from_vehicle(prescribed, &self.mass);
        self.pinned_rest = PinnedSurfaceRest::Free;
        self.last_alpha_root_rad_s2 = DVec3::ZERO;
        let specific_force =
            (sub.thrust_body_n + env.aero_root.force_body_n + sub.ext_force_root_n)
                / self.mass.total_mass_kg;
        self.telemetry = build_telemetry(
            &self.subsystems,
            &env,
            &sub,
            specific_force,
            false,
            false,
            false,
            0.0,
            self.telemetry.used_rigid_fast_path,
        );
        self.now = self.now.advanced(self.config.period);
        Ok(())
    }

    /// Takes `state` as the vehicle's centre-of-gravity state at
    /// [`Self::now`], leaving every subsystem and joint as it is: how a
    /// vehicle flown elsewhere comes back to this block's own integration.
    pub fn adopt_base_state(&mut self, state: &VehicleState) -> Result<(), BlockError> {
        check_prescribed(state)?;
        self.base_fixed = fixed_base_from_vehicle(state, &self.mass);
        self.pinned_rest = PinnedSurfaceRest::Free;
        self.last_alpha_root_rad_s2 = DVec3::ZERO;
        self.telemetry.in_ground_contact = false;
        self.telemetry.is_pinned_rest = false;
        self.telemetry.is_splashed_down = false;
        self.telemetry.total_buoyancy_n = 0.0;
        Ok(())
    }

    /// Whether any joint is moving, or `input` drives one: motion that reacts
    /// on the base and that [`Self::step_carried`] does not integrate.
    #[must_use]
    pub fn joints_in_motion(&self, input: &MultibodyVehicleInput) -> bool {
        let driven = input.joint_drives.iter().any(|drive| {
            drive.effort_scalar != 0.0
                || drive.torque_3d_nm != DVec3::ZERO
                || drive.force_3d_n != DVec3::ZERO
        });
        let moving = self.tree.states().iter().any(|state| {
            state.joint.q_vel_scalar != 0.0
                || state.joint.linear_vel_m_s != DVec3::ZERO
                || state.joint.angular_vel_rad_s != DVec3::ZERO
        });
        driven || moving
    }
}

fn check_prescribed(state: &VehicleState) -> Result<(), BlockError> {
    if state.position_ecef.is_finite()
        && state.velocity_ecef.is_finite()
        && state.attitude.is_finite()
        && state.angular_rate_body.is_finite()
    {
        Ok(())
    } else {
        Err(invalid_input("prescribed_state", format!("{state:?}")))
    }
}
