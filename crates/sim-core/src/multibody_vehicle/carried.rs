//! Stepping a vehicle's internals while another integrator owns its motion.
//!
//! Orbital propagation carries a vehicle as one rigid body. Its tanks, bus,
//! thrusters, thermal network and joints still have to run, so this advances
//! exactly those with the motion of its centre of gravity and of its body
//! axes prescribed from outside. Joints move under their drives, stops and
//! on-vehicle mechanisms; what their motion does to the vehicle's angular
//! momentum is handed back as a [`CarriedReaction`] for the prescribing
//! integrator to apply, since it flies the base as if nothing inside moved.

use std::sync::Arc;

use glam::DVec3;
use mbd_multibody::{FloatingBaseState, PinnedSurfaceRest};
use sim_atmosphere::AtmosphereField;
use sim_geo::{BodyEllipsoid, GravityField};

use super::step::{block_multi, invalid_input};
use super::subsystems::validate_input;
use super::telemetry::build_telemetry;
use super::{MultibodyVehicleBlock, MultibodyVehicleInput, SurfaceContactProvider, VehicleWorld};
use crate::flight_model::coupled_mechanism::CoupledMechanism;
use crate::flight_model::loads::LoadReadout;
use crate::flight_model::planet_frame::fixed_to_instantaneous_inertial;
use crate::{BlockError, VehicleState};

/// What the motion inside a vehicle over one carried step asks of the
/// integrator that flies its base.
///
/// About the centre of gravity the vehicle's angular momentum in body axes
/// is `I ω + h`: its inertia with the joints where they are times the
/// inertial rate of its body axes, plus the momentum `h` of its parts
/// moving relative to them. A carried step moves the joints with `ω`
/// prescribed, which changes `I` and `h`; an integrator that applies
/// [`Self::angular_impulse_cg_body_nms`] to the base and holds
/// [`Self::internal_momentum_cg_body_nms`] keeps `I ω + h` continuous
/// across the step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarriedReaction {
    /// Angular impulse on the base about the centre of gravity, body axes,
    /// N·m·s: minus the change the step's joint motion made to `I ω + h` at
    /// the prescribed `ω`. Mass that left the vehicle during the step is
    /// not in it.
    pub angular_impulse_cg_body_nms: DVec3,
    /// Angular momentum `h` of the vehicle's parts moving relative to its
    /// body axes at the end of the step, about the centre of gravity, body
    /// axes, N·m·s.
    pub internal_momentum_cg_body_nms: DVec3,
}

impl CarriedReaction {
    /// Nothing inside moved.
    pub const NONE: Self = Self {
        angular_impulse_cg_body_nms: DVec3::ZERO,
        internal_momentum_cg_body_nms: DVec3::ZERO,
    };
}

impl MultibodyVehicleBlock {
    /// Advances the subsystems (fluids, electrical bus, propulsion, thermal,
    /// surface erosion) and the joints one period with the vehicle carried
    /// at `prescribed`, the state of its centre of gravity and body axes at
    /// the end of the period as another integrator flew it.
    ///
    /// Joints are integrated with the whole vehicle floating free from
    /// `prescribed` under the loads held over the period, so a drive
    /// accelerates its joint against the vehicle's own inertia; the base
    /// motion that integration finds is then replaced by `prescribed` and
    /// returned as the [`CarriedReaction`] the other integrator owes the
    /// vehicle. [`Self::telemetry`] reports the thrust and its moment that
    /// integrator has to apply. A tether or docking ring to another vehicle
    /// is refused: its far end is not followed while the vehicle is carried.
    /// A failed step changes nothing.
    pub fn step_carried(
        &mut self,
        input: &MultibodyVehicleInput,
        prescribed: &VehicleState,
    ) -> Result<CarriedReaction, BlockError> {
        let saved = self.snapshot_for_coord();
        let res = self.run_carried_step(input, prescribed);
        if res.is_err()
            && let Err(error) = self.try_restore(&saved)
        {
            tracing::error!(%error, "multibody rollback failed");
        }
        res
    }

    /// Puts the root where a vehicle whose centre of gravity is at
    /// `prescribed` has it, with the mass the vehicle has now.
    fn place_at(&mut self, prescribed: &VehicleState) {
        self.state
            .set_base(&self.mount.base_of(prescribed, self.mass.cg_body_m));
    }

    /// Whether any joint has a degree of freedom.
    fn has_moving_joints(&self) -> bool {
        self.model.layout().velocity_len() > 6
    }

    /// The vehicle's angular momentum about its centre of gravity in body
    /// axes with its joints as they are: in whole, at the inertial rate its
    /// body axes have now, and the part its joints' own motion carries.
    fn angular_momentum_cg_body(&mut self) -> Result<(DVec3, DVec3), BlockError> {
        let base = self.state.base();
        let to_body = self.mount.attitude(&base).conjugate();
        let mut at = self.state.clone();
        at.set_base(&fixed_to_instantaneous_inertial(&base, &self.config.body));
        let whole = self
            .model
            .system_momentum(&at, &mut self.work)
            .map_err(block_multi)?;
        at.set_base(&FloatingBaseState {
            linear_vel_world_m_s: DVec3::ZERO,
            angular_vel_root_rad_s: DVec3::ZERO,
            ..base
        });
        let internal = self
            .model
            .system_momentum(&at, &mut self.work)
            .map_err(block_multi)?;
        Ok((
            to_body * whole.angular_about_com_world_nms,
            to_body * internal.angular_about_com_world_nms,
        ))
    }

    /// Angular momentum of the vehicle's parts moving relative to its body
    /// axes now, about the centre of gravity, body axes, N·m·s: what a
    /// vehicle handed to another integrator with its joints in motion
    /// reports before its first carried step.
    pub fn internal_momentum_cg_body(&mut self) -> Result<DVec3, BlockError> {
        if !self.has_moving_joints() {
            return Ok(DVec3::ZERO);
        }
        Ok(self.angular_momentum_cg_body()?.1)
    }

    /// Moves the vehicle to another central body: from now on it flies
    /// about `body`, in `gravity` and `atmosphere` and over `surface`, with
    /// `state` as the state of its centre of gravity in that body's fixed
    /// axes at [`Self::now`]. Every subsystem, joint and element state is
    /// left as it is, and the model keeps the scale it was built with.
    pub fn rehome(
        &mut self,
        body: BodyEllipsoid,
        gravity: Arc<dyn GravityField>,
        atmosphere: Arc<dyn AtmosphereField>,
        surface: Arc<dyn SurfaceContactProvider>,
        state: &VehicleState,
    ) -> Result<(), BlockError> {
        check_prescribed(state)?;
        self.config.body = body;
        self.world = VehicleWorld {
            atmosphere,
            gravity,
            surface,
        };
        self.adopt_base_state(state)
    }

    fn run_carried_step(
        &mut self,
        input: &MultibodyVehicleInput,
        prescribed: &VehicleState,
    ) -> Result<CarriedReaction, BlockError> {
        validate_input(input)?;
        check_prescribed(prescribed)?;
        check_mechanisms(input)?;
        self.sync_elements()?;
        let dt_s = self.config.period.as_seconds();
        self.place_at(prescribed);
        let (env, sub) = self.step_subsystems(dt_s, input)?;
        self.take_up_masses(&sub)?;
        // The prescribed point is the centre of gravity, which moves in the
        // root frame as tanks drain, so the root is placed from the new mass.
        self.place_at(prescribed);
        self.hold_loads(&sub, input);
        let reaction = if self.has_moving_joints() {
            let (before, _) = self.angular_momentum_cg_body()?;
            self.integrate_held(&sub, input)?;
            // The joints have moved the centre of gravity in the root frame.
            self.place_at(prescribed);
            let (after, internal) = self.angular_momentum_cg_body()?;
            CarriedReaction {
                angular_impulse_cg_body_nms: before - after,
                internal_momentum_cg_body_nms: internal,
            }
        } else {
            self.elements.mechanisms.clear();
            self.mechanism_states.clear();
            CarriedReaction::NONE
        };
        self.poses = self
            .tree
            .evaluate_pose_table()
            .map_err(super::step::block_mbd)?;
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
        );
        self.now = self.now.advanced(self.config.period);
        Ok(reaction)
    }

    /// Takes `state` as the vehicle's centre-of-gravity state at
    /// [`Self::now`], leaving every subsystem and joint as it is: how a
    /// vehicle flown elsewhere comes back to this block's own integration.
    pub fn adopt_base_state(&mut self, state: &VehicleState) -> Result<(), BlockError> {
        check_prescribed(state)?;
        self.place_at(state);
        self.pinned_rest = PinnedSurfaceRest::Free;
        self.last_alpha_root_rad_s2 = DVec3::ZERO;
        self.telemetry.in_ground_contact = false;
        self.telemetry.is_pinned_rest = false;
        self.telemetry.is_splashed_down = false;
        self.telemetry.total_buoyancy_n = 0.0;
        Ok(())
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

/// Refuses a mechanism whose other end is on another vehicle.
fn check_mechanisms(input: &MultibodyVehicleInput) -> Result<(), BlockError> {
    let reaches_out = input.mechanisms.iter().any(|mechanism| {
        matches!(
            mechanism,
            CoupledMechanism::Tether(..) | CoupledMechanism::Docking(..)
        )
    });
    if reaches_out {
        return Err(invalid_input(
            "mechanisms",
            "a carried vehicle cannot follow a tether or docking ring to another".to_owned(),
        ));
    }
    Ok(())
}
