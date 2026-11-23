//! Stepping a vehicle's internals while another integrator owns its motion.
//!
//! Orbital propagation carries a vehicle as one rigid body. Its tanks, bus,
//! thrusters and thermal network still have to run, so this advances exactly
//! those with the base state prescribed from outside, and leaves joints where
//! they are: it is only valid while nothing internal reacts on the base
//! faster than the prescribing integrator hears about it.

use glam::DVec3;
use mbd_multibody::PinnedSurfaceRest;

use super::step::invalid_input;
use super::subsystems::validate_input;
use super::telemetry::build_telemetry;
use super::{MultibodyVehicleBlock, MultibodyVehicleInput};
use crate::flight_model::loads::LoadReadout;
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
        self.sync_elements()?;
        let dt_s = self.config.period.as_seconds();
        self.state
            .set_base(&self.mount.base_of(prescribed, self.mass.cg_body_m));
        let (env, sub) = self.step_subsystems(dt_s, input)?;
        self.take_up_masses(&sub)?;
        // The prescribed point is the centre of gravity, which moves in the
        // root frame as tanks drain, so the root is placed from the new mass.
        self.state
            .set_base(&self.mount.base_of(prescribed, self.mass.cg_body_m));
        self.poses = self
            .tree
            .evaluate_pose_table()
            .map_err(super::step::block_mbd)?;
        self.hold_loads(&sub, input);
        // A carried vehicle's far ends are not followed over the period, so
        // its mechanisms stay out of the loads read at its end.
        self.elements.mechanisms.clear();
        self.mechanism_states.clear();
        let state = self.state.clone();
        let loads = self.read_loads(&state, input)?;
        self.pinned_rest = PinnedSurfaceRest::Free;
        self.last_alpha_root_rad_s2 = DVec3::ZERO;
        // What the other integrator has to apply: thrust, air and outside
        // loads, without the surface the vehicle is carried clear of.
        let applied = loads.external_force_body_n - loads.support_force_body_n;
        let carried = LoadReadout {
            any_contact: false,
            buoyancy_n: 0.0,
            ..loads
        };
        self.telemetry = build_telemetry(
            &self.subsystems,
            &env,
            &sub,
            &carried,
            applied / self.mass.total_mass_kg,
            false,
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
        self.state
            .set_base(&self.mount.base_of(state, self.mass.cg_body_m));
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
