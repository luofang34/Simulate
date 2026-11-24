//! Unified `OdeSystem` coupling all free vessels and active `Docking` /
//! `WinchTether` elements in one proximity group at every integrator stage.

use core::ops::Range;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, PoseTable, WrenchAtSite};
use mbd_multibody::{
    DockingSpec, ElementContext, FloatingBaseState, ForceElement, Loads, ModelScale, ModelState,
    MultibodyError, MultibodyModel, OdeSystem, PortKinematics, StateKind, TetherCommand,
    WinchTetherSpec, Workspace,
};
use sim_geo::BodyEllipsoid;

use super::links::{describe_link, describe_vessel, evaluate_one_link, transition_one_link};

use crate::flight_model::elements::VehicleElements;
use crate::flight_model::environment::FlightEnvironment;
use crate::flight_model::planet_frame::{fixed_from_inertial, spin};
use crate::multi_vessel::plume::site_world_kinematics;

/// Step-constant host environment paired with live stage inter-vessel wrenches.
pub(super) struct StageInput<'a> {
    pub(super) env: &'a FlightEnvironment<'a>,
    pub(super) link_wrenches: &'a [WrenchAtSite],
}

/// Wrapper presenting a vessel's own `VehicleElements` plus live inter-vessel
/// wrenches evaluated at the current stage.
pub(super) struct StageElements<'a> {
    pub(super) own: &'a VehicleElements,
}

impl<'a> StageElements<'a> {
    fn inner_ctx<'c>(
        &self,
        ctx: &'c ElementContext<'c, StageInput<'a>>,
    ) -> ElementContext<'c, FlightEnvironment<'a>> {
        ElementContext {
            time_s: ctx.time_s,
            model: ctx.model,
            state: ctx.state,
            kinematics: ctx.kinematics,
            inputs: ctx.inputs.env,
        }
    }
}

impl<'a> ForceElement<StageInput<'a>> for StageElements<'a> {
    fn state_len(&self) -> usize {
        ForceElement::<FlightEnvironment<'_>>::state_len(self.own)
    }

    fn mode_len(&self) -> usize {
        ForceElement::<FlightEnvironment<'_>>::mode_len(self.own)
    }

    fn has_transitions(&self) -> bool {
        ForceElement::<FlightEnvironment<'_>>::has_transitions(self.own)
    }

    fn initial_state(&self, state: &mut [f64], modes: &mut [f64]) {
        ForceElement::<FlightEnvironment<'_>>::initial_state(self.own, state, modes);
    }

    fn state_scale(&self, model: &ModelScale, scale: &mut [f64]) {
        ForceElement::<FlightEnvironment<'_>>::state_scale(self.own, model, scale);
    }

    fn evaluate(
        &self,
        ctx: &ElementContext<'_, StageInput<'a>>,
        state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let inner = self.inner_ctx(ctx);
        ForceElement::<FlightEnvironment<'_>>::evaluate(
            self.own, &inner, state, modes, loads, state_dot,
        )?;
        for wrench in ctx.inputs.link_wrenches {
            loads.add_wrench(wrench)?;
        }
        Ok(())
    }

    fn has_resolution(&self) -> bool {
        ForceElement::<FlightEnvironment<'_>>::has_resolution(self.own)
    }

    fn resolve(
        &self,
        ctx: &ElementContext<'_, StageInput<'a>>,
        state: &[f64],
        modes: &[f64],
        resolution: &mut mbd_multibody::Resolution<'_>,
        loads: &mut Loads<'_>,
    ) -> Result<(), MultibodyError> {
        let inner = self.inner_ctx(ctx);
        ForceElement::<FlightEnvironment<'_>>::resolve(
            self.own, &inner, state, modes, resolution, loads,
        )
    }

    fn transition(
        &self,
        ctx: &ElementContext<'_, StageInput<'a>>,
        state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        let inner = self.inner_ctx(ctx);
        ForceElement::<FlightEnvironment<'_>>::transition(self.own, &inner, state, modes)
    }
}

#[derive(Clone, Copy)]
pub(super) enum VesselTarget {
    Free(usize),
    Pinned(usize),
}

#[derive(Clone, Copy)]
pub(super) struct EndpointRef {
    pub(super) target: VesselTarget,
    pub(super) component: ComponentId,
    pub(super) offset_local_m: DVec3,
}

#[derive(Clone, Copy)]
pub(super) struct EndpointPose {
    pub(super) port: PortKinematics,
    pub(super) rot_world_from_link: DMat3,
}

impl EndpointPose {
    pub(super) const ZERO: Self = Self {
        port: PortKinematics {
            position_world_m: DVec3::ZERO,
            rotation_world: DQuat::IDENTITY,
            velocity_world_m_s: DVec3::ZERO,
            angular_vel_world_rad_s: DVec3::ZERO,
        },
        rot_world_from_link: DMat3::IDENTITY,
    };
}

pub(super) enum GroupLinkKind {
    Docking {
        spec: DockingSpec,
        axis_a_local: DVec3,
    },
    Tether {
        spec: WinchTetherSpec,
        command: TetherCommand,
    },
}

pub(super) struct GroupLinkSlot {
    pub(super) link_index: usize,
    pub(super) end_a: EndpointRef,
    pub(super) end_b: EndpointRef,
    pub(super) kind: GroupLinkKind,
    pub(super) modes: Vec<f64>,
    pub(super) range: Range<usize>,
    pub(super) scale: ModelScale,
}

pub(super) struct PinnedVesselSlot {
    pub(super) poses: PoseTable,
    pub(super) fixed_base: FloatingBaseState,
    pub(super) body: BodyEllipsoid,
}

pub(super) struct GroupVesselSlot<'a> {
    pub(super) model: &'a MultibodyModel,
    pub(super) elements: &'a VehicleElements,
    pub(super) env: FlightEnvironment<'a>,
    pub(super) modes: Vec<f64>,
    pub(super) work: Workspace,
    pub(super) stage_state: ModelState,
    pub(super) link_wrenches: Vec<WrenchAtSite>,
    pub(super) range: Range<usize>,
}

pub(super) struct ProximityGroupOde<'a> {
    pub(super) vessels: Vec<GroupVesselSlot<'a>>,
    pub(super) pinned: Vec<PinnedVesselSlot>,
    pub(super) links: Vec<GroupLinkSlot>,
    pub(super) total_dim: usize,
    pub(super) cache_a: Vec<EndpointPose>,
    pub(super) cache_b: Vec<EndpointPose>,
}

fn len_err(expected: usize, actual: usize) -> MultibodyError {
    MultibodyError::StateLengthMismatch { expected, actual }
}

fn pinned_endpoint_pose(
    slot: &PinnedVesselSlot,
    end: EndpointRef,
    time_s: f64,
) -> Result<EndpointPose, MultibodyError> {
    let (pos_f, rot_f, _, _) = site_world_kinematics(
        &slot.poses,
        &slot.fixed_base,
        end.component,
        end.offset_local_m,
    )
    .map_err(|_| MultibodyError::DetachedComponent { id: end.component })?;
    let q_i_from_f = fixed_from_inertial(&slot.body, time_s).conjugate();
    let pos_i = q_i_from_f * pos_f;
    let rot_i = (q_i_from_f * rot_f).normalize();
    let omega_i = spin(&slot.body);
    Ok(EndpointPose {
        port: PortKinematics {
            position_world_m: pos_i,
            rotation_world: rot_i,
            velocity_world_m_s: omega_i.cross(pos_i),
            angular_vel_world_rad_s: omega_i,
        },
        rot_world_from_link: DMat3::from_quat(rot_i),
    })
}

fn free_endpoint_pose(
    kin: &mbd_multibody::Kinematics<'_>,
    end: EndpointRef,
) -> Result<EndpointPose, MultibodyError> {
    let p = kin.point(end.component, end.offset_local_m)?;
    Ok(EndpointPose {
        port: PortKinematics {
            position_world_m: p.position_world_m,
            rotation_world: p.rotation_quat(),
            velocity_world_m_s: p.velocity_world_m_s,
            angular_vel_world_rad_s: p.angular_vel_world_rad_s,
        },
        rot_world_from_link: p.rotation_world_from_link,
    })
}

impl ProximityGroupOde<'_> {
    pub(super) fn eval_endpoint_poses(&mut self, t: f64, x: &[f64]) -> Result<(), MultibodyError> {
        for (v_idx, vessel) in self.vessels.iter_mut().enumerate() {
            let x_v = x
                .get(vessel.range.clone())
                .ok_or_else(|| len_err(vessel.range.end, x.len()))?;
            let dst = vessel.stage_state.as_mut_slice();
            if dst.len() != x_v.len() {
                return Err(len_err(dst.len(), x_v.len()));
            }
            dst.copy_from_slice(x_v);
            vessel.model.centre_of_mass_to_root(dst, &mut vessel.work)?;
            let kin = vessel
                .model
                .kinematics(&vessel.stage_state, &mut vessel.work)?;
            for (l_idx, link) in self.links.iter().enumerate() {
                if let VesselTarget::Free(i) = link.end_a.target
                    && i == v_idx
                    && let Some(slot) = self.cache_a.get_mut(l_idx)
                {
                    *slot = free_endpoint_pose(&kin, link.end_a)?;
                }
                if let VesselTarget::Free(i) = link.end_b.target
                    && i == v_idx
                    && let Some(slot) = self.cache_b.get_mut(l_idx)
                {
                    *slot = free_endpoint_pose(&kin, link.end_b)?;
                }
            }
        }
        for (l_idx, link) in self.links.iter().enumerate() {
            if let VesselTarget::Pinned(p_idx) = link.end_a.target
                && let Some(p_slot) = self.pinned.get(p_idx)
                && let Some(dst) = self.cache_a.get_mut(l_idx)
            {
                *dst = pinned_endpoint_pose(p_slot, link.end_a, t)?;
            }
            if let VesselTarget::Pinned(p_idx) = link.end_b.target
                && let Some(p_slot) = self.pinned.get(p_idx)
                && let Some(dst) = self.cache_b.get_mut(l_idx)
            {
                *dst = pinned_endpoint_pose(p_slot, link.end_b, t)?;
            }
        }
        Ok(())
    }

    pub(super) fn apply_link_evaluations(
        &mut self,
        x: &[f64],
        mut x_dot: Option<&mut [f64]>,
    ) -> Result<(), MultibodyError> {
        for v in &mut self.vessels {
            v.link_wrenches.clear();
        }
        for (idx, link) in self.links.iter().enumerate() {
            let pa = *self
                .cache_a
                .get(idx)
                .ok_or_else(|| len_err(idx, self.cache_a.len()))?;
            let pb = *self
                .cache_b
                .get(idx)
                .ok_or_else(|| len_err(idx, self.cache_b.len()))?;
            let x_l = x
                .get(link.range.clone())
                .ok_or_else(|| len_err(link.range.end, x.len()))?;
            let xd_l = match x_dot.as_deref_mut() {
                Some(xd) => {
                    let xd_len = xd.len();
                    Some(
                        xd.get_mut(link.range.clone())
                            .ok_or_else(|| len_err(link.range.end, xd_len))?,
                    )
                }
                None => None,
            };
            evaluate_one_link(&mut self.vessels, link, pa, pb, x_l, xd_l);
        }
        Ok(())
    }
}

impl OdeSystem for ProximityGroupOde<'_> {
    fn dim(&self) -> usize {
        self.total_dim
    }

    fn derivative(&mut self, t: f64, x: &[f64], x_dot: &mut [f64]) -> Result<(), MultibodyError> {
        self.eval_endpoint_poses(t, x)?;
        self.apply_link_evaluations(x, Some(x_dot))?;
        for vessel in &mut self.vessels {
            let x_v = x
                .get(vessel.range.clone())
                .ok_or_else(|| len_err(vessel.range.end, x.len()))?;
            let xd_len = x_dot.len();
            let xd_v = x_dot
                .get_mut(vessel.range.clone())
                .ok_or_else(|| len_err(vessel.range.end, xd_len))?;
            let stage_elements = StageElements {
                own: vessel.elements,
            };
            let stage_input = StageInput {
                env: &vessel.env,
                link_wrenches: &vessel.link_wrenches,
            };
            let mut ode = mbd_multibody::CentreOfMassOde {
                model: vessel.model,
                elements: &stage_elements,
                inputs: &stage_input,
                modes: &mut vessel.modes,
                work: &mut vessel.work,
            };
            ode.derivative(t, x_v, xd_v)?;
        }
        Ok(())
    }

    fn retract(&self, x: &mut [f64]) {
        for vessel in &self.vessels {
            if let Some(x_v) = x.get_mut(vessel.range.clone()) {
                vessel.model.layout().retract(x_v);
            }
        }
    }

    fn describe(&self, kinds: &mut [StateKind], magnitudes: &mut [f64]) {
        kinds.fill(StateKind::Auxiliary);
        magnitudes.fill(1.0);
        for vessel in &self.vessels {
            if let (Some(kv), Some(mv)) = (
                kinds.get_mut(vessel.range.clone()),
                magnitudes.get_mut(vessel.range.clone()),
            ) {
                describe_vessel(vessel.model, vessel.elements, kv, mv);
            }
        }
        for link in &self.links {
            if let Some(ml) = magnitudes.get_mut(link.range.clone()) {
                describe_link(link, ml);
            }
        }
    }

    fn accept(&mut self, t: f64, x: &mut [f64]) -> Result<bool, MultibodyError> {
        let mut event = false;
        for vessel in &mut self.vessels {
            if !ForceElement::<FlightEnvironment<'_>>::has_transitions(vessel.elements) {
                continue;
            }
            let x_len = x.len();
            let x_v = x
                .get_mut(vessel.range.clone())
                .ok_or_else(|| len_err(vessel.range.end, x_len))?;
            let stage_elements = StageElements {
                own: vessel.elements,
            };
            let stage_input = StageInput {
                env: &vessel.env,
                link_wrenches: &[],
            };
            let mut ode = mbd_multibody::CentreOfMassOde {
                model: vessel.model,
                elements: &stage_elements,
                inputs: &stage_input,
                modes: &mut vessel.modes,
                work: &mut vessel.work,
            };
            event |= ode.accept(t, x_v)?;
        }
        self.eval_endpoint_poses(t, x)?;
        for (idx, link) in self.links.iter_mut().enumerate() {
            let pa = *self
                .cache_a
                .get(idx)
                .ok_or_else(|| len_err(idx, self.cache_a.len()))?;
            let pb = *self
                .cache_b
                .get(idx)
                .ok_or_else(|| len_err(idx, self.cache_b.len()))?;
            let x_len = x.len();
            let x_l = x
                .get_mut(link.range.clone())
                .ok_or_else(|| len_err(link.range.end, x_len))?;
            event |= transition_one_link(link, pa, pb, x_l);
        }
        Ok(event)
    }
}
