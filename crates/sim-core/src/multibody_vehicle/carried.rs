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

use glam::{DMat3, DVec3};
use mbd_core::{ComponentTree, JointKind};
use mbd_multibody::{FloatingBaseState, PinnedSurfaceRest};
use sim_atmosphere::AtmosphereField;
use sim_geo::{BodyEllipsoid, GravityField};

use super::step::{block_multi, invalid_input};
use super::subsystems::validate_input;
use super::telemetry::build_telemetry;
use super::{MultibodyVehicleBlock, MultibodyVehicleInput, SurfaceContactProvider, VehicleWorld};
use crate::flight_model::coupled_mechanism::CoupledMechanism;
use crate::flight_model::loads::LoadReadout;
use crate::flight_model::planet_frame::{fixed_to_instantaneous_inertial, spin};
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

/// Carried-step result exposing the free-joint effective (Schur-complement)
/// base inertia and canonical joint momentum alongside [`CarriedReaction`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarriedDecoupledStep {
    /// Standard [`CarriedReaction`] measured at `reaction_base`.
    pub reaction: CarriedReaction,
    /// Free-joint effective rotational inertia about CG before joint motion,
    /// using post-drain masses, body axes, kg·m².
    pub effective_inertia_before_cg_body: DMat3,
    /// Free-joint effective rotational inertia about CG after joint motion,
    /// body axes, kg·m².
    pub effective_inertia_after_cg_body: DMat3,
    /// Canonical internal angular momentum `L_cg - I_eff ω` after the step at
    /// `reaction_base`, body axes, N·m·s.
    pub canonical_momentum_after_cg_body_nms: DVec3,
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
        self.step_carried_between(input, prescribed, prescribed, prescribed)
    }

    /// Advances subsystems and joints one period from `start` to `end` while
    /// another integrator holds the base trajectory, returning the reaction
    /// owed at `reaction_base`.
    pub fn step_carried_between(
        &mut self,
        input: &MultibodyVehicleInput,
        start: &VehicleState,
        end: &VehicleState,
        reaction_base: &VehicleState,
    ) -> Result<CarriedReaction, BlockError> {
        self.step_carried_decoupled(input, start, end, reaction_base)
            .map(|out| out.reaction)
    }

    /// Advances subsystems and joints one period from `start` to `end`, also
    /// returning the free-joint Schur-complement inertia and canonical
    /// momentum at `reaction_base`.
    pub fn step_carried_decoupled(
        &mut self,
        input: &MultibodyVehicleInput,
        start: &VehicleState,
        end: &VehicleState,
        reaction_base: &VehicleState,
    ) -> Result<CarriedDecoupledStep, BlockError> {
        let saved = self.snapshot_for_coord();
        let res = self.run_carried_step(input, start, end, reaction_base);
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
    pub fn has_moving_joints(&self) -> bool {
        self.model.layout().velocity_len() > 6
    }

    /// Computes the free-joint effective (Schur-complement) inertia tensor about
    /// CG in body axes and the canonical joint momentum `L_cg - I_eff ω` at
    /// `prescribed`.
    pub fn effective_inertia_and_momentum_cg_body(
        &mut self,
        prescribed: &VehicleState,
    ) -> Result<(DMat3, DVec3), BlockError> {
        if !self.has_moving_joints() {
            return Ok((self.mass.inertia_cg_body, DVec3::ZERO));
        }
        let saved = self.state.base();
        self.place_at(prescribed);
        let res = self.compute_effective_at(prescribed);
        self.state.set_base(&saved);
        res
    }

    fn compute_effective_at(
        &mut self,
        prescribed: &VehicleState,
    ) -> Result<(DMat3, DVec3), BlockError> {
        let (whole_body, _) = self.angular_momentum_cg_body()?;
        let i_eff = self.schur_inertia_cg_body()?;
        let w_inertial = prescribed.angular_rate_body
            + prescribed.attitude.conjugate() * spin(&self.config.body);
        Ok((i_eff, whole_body - i_eff * w_inertial))
    }

    fn schur_inertia_cg_body(&mut self) -> Result<DMat3, BlockError> {
        let nv = self.model.layout().velocity_len();
        let mut tau = vec![0.0; nv];
        let mut accel = vec![0.0; nv];
        let mut cols = [DVec3::ZERO; 3];
        for (axis, col) in [DVec3::X, DVec3::Y, DVec3::Z].into_iter().zip(&mut cols) {
            tau.fill(0.0);
            let tau_link = self.mount.body_from_link.conjugate() * axis;
            if let Some(dst) = tau.get_mut(3..6) {
                dst.copy_from_slice(&[tau_link.x, tau_link.y, tau_link.z]);
            }
            self.model
                .apply_inverse_mass(self.state.as_slice(), &tau, &mut self.work, &mut accel)
                .map_err(super::step::block_multi)?;
            if let [_, _, _, ax, ay, az, ..] = *accel.as_slice() {
                *col = self.mount.body_from_link * DVec3::new(ax, ay, az);
            }
        }
        let [c0, c1, c2] = cols;
        let inv = DMat3::from_cols(c0, c1, c2);
        let sym = 0.5 * (inv + inv.transpose());
        let det = sym.determinant();
        if det.is_finite() && det > 0.0 {
            Ok(sym.inverse())
        } else {
            Ok(self.mass.inertia_cg_body)
        }
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

    fn step_carried_joints(
        &mut self,
        sub: &super::step_subsystems::SubsystemStepOut,
        input: &MultibodyVehicleInput,
        start: &VehicleState,
        end: &VehicleState,
        reaction_base: &VehicleState,
    ) -> Result<CarriedDecoupledStep, BlockError> {
        if !self.has_moving_joints() {
            self.place_at(end);
            self.elements.mechanisms.clear();
            self.mechanism_states.clear();
            return Ok(CarriedDecoupledStep {
                reaction: CarriedReaction::NONE,
                effective_inertia_before_cg_body: self.mass.inertia_cg_body,
                effective_inertia_after_cg_body: self.mass.inertia_cg_body,
                canonical_momentum_after_cg_body_nms: DVec3::ZERO,
            });
        }
        let dt_s = self.config.period.as_seconds();
        self.place_at(reaction_base);
        let (before, _) = self.angular_momentum_cg_body()?;
        let (i_eff_before, _) = self.compute_effective_at(reaction_base)?;
        self.place_at(start);
        let omega_inertial =
            start.angular_rate_body + start.attitude.conjugate() * spin(&self.config.body);
        let near_stop = any_joint_near_stop(&self.tree, dt_s);
        let mut override_stepper = self
            .stepper
            .carried_for_spin(omega_inertial, dt_s, near_stop);
        self.integrate_held_with(sub, input, override_stepper.as_mut())?;
        self.place_at(reaction_base);
        let (after, internal) = self.angular_momentum_cg_body()?;
        let (i_eff_after, h_after) = self.compute_effective_at(reaction_base)?;
        self.place_at(end);
        Ok(CarriedDecoupledStep {
            reaction: CarriedReaction {
                angular_impulse_cg_body_nms: before - after,
                internal_momentum_cg_body_nms: internal,
            },
            effective_inertia_before_cg_body: i_eff_before,
            effective_inertia_after_cg_body: i_eff_after,
            canonical_momentum_after_cg_body_nms: h_after,
        })
    }

    fn run_carried_step(
        &mut self,
        input: &MultibodyVehicleInput,
        start: &VehicleState,
        end: &VehicleState,
        reaction_base: &VehicleState,
    ) -> Result<CarriedDecoupledStep, BlockError> {
        validate_input(input)?;
        check_prescribed(start)?;
        check_prescribed(end)?;
        check_prescribed(reaction_base)?;
        check_mechanisms(input)?;
        self.sync_elements()?;
        let dt_s = self.config.period.as_seconds();
        self.place_at(start);
        let (env, sub) = self.step_subsystems(dt_s, input)?;
        self.take_up_masses(&sub)?;
        self.hold_loads(&sub, input);
        let decoupled = self.step_carried_joints(&sub, input, start, end, reaction_base)?;
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
        Ok(decoupled)
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

/// Switches carried fixed-step RK4 to adaptive integration near a finite travel
/// stop whose penalty stiffness (`~omega_stop = 45 rad/s`) would destabilize `h ~ 10 s`.
fn any_joint_near_stop(tree: &ComponentTree, dt_s: f64) -> bool {
    const STOP_MARGIN: f64 = 0.15;
    tree.ids().any(|id| {
        let (Ok(spec), Ok(state)) = (tree.spec(id), tree.state(id)) else {
            return false;
        };
        if !state.attached {
            return false;
        }
        let js = state.joint;
        match spec.joint.kind {
            JointKind::Fixed | JointKind::Compliant6Dof { .. } => false,
            JointKind::Revolute {
                min_rad: min,
                max_rad: max,
                ..
            }
            | JointKind::Prismatic {
                min_m: min,
                max_m: max,
                ..
            } => {
                let q0 = js.q_scalar;
                let q1 = q0 + js.q_vel_scalar * dt_s;
                (min.is_finite() && (q0 <= min + STOP_MARGIN || q1 <= min + STOP_MARGIN))
                    || (max.is_finite() && (q0 >= max - STOP_MARGIN || q1 >= max - STOP_MARGIN))
            }
            JointKind::Spherical {
                cone_half_angle_rad,
            } => {
                if !cone_half_angle_rad.is_finite() {
                    return false;
                }
                let r = js.rotation;
                let angle = 2.0 * sim_spacetime::atan2(r.xyz().length(), r.w.abs());
                let speed = js.angular_vel_rad_s.length();
                angle + STOP_MARGIN >= cone_half_angle_rad
                    || angle + speed * dt_s + STOP_MARGIN >= cone_half_angle_rad
            }
        }
    })
}
