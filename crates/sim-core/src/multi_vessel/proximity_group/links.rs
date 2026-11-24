//! Stage evaluation, state scaling, and mode transitions for coupled
//! `Docking` and `WinchTether` links inside [`super::ode::ProximityGroupOde`].

use glam::DVec3;
use mbd_core::WrenchAtSite;
use mbd_multibody::{
    DockingCapture, DockingState, ForceElement, MultibodyModel, PortAnchor, StateKind, Tether,
    TetherAnchor, TetherEnds, WinchTetherState,
};

use super::ode::{EndpointPose, GroupLinkKind, GroupLinkSlot, GroupVesselSlot, VesselTarget};
use crate::flight_model::elements::VehicleElements;
use crate::flight_model::environment::FlightEnvironment;

fn push_wrench(vessels: &mut [GroupVesselSlot<'_>], target: VesselTarget, wrench: WrenchAtSite) {
    if let VesselTarget::Free(idx) = target
        && let Some(slot) = vessels.get_mut(idx)
    {
        slot.link_wrenches.push(wrench);
    }
}

fn write_link_rates(dst: Option<&mut [f64]>, vals: &[f64]) {
    if let Some(out) = dst {
        for (slot, &val) in out.iter_mut().zip(vals) {
            *slot = val;
        }
    }
}

pub(super) fn evaluate_one_link(
    vessels: &mut [GroupVesselSlot<'_>],
    link: &GroupLinkSlot,
    pa: EndpointPose,
    pb: EndpointPose,
    x_l: &[f64],
    xd_l: Option<&mut [f64]>,
) {
    match &link.kind {
        GroupLinkKind::Docking { spec, axis_a_local } => {
            let st = DockingState::from_blocks(x_l, &link.modes);
            let ev = spec.evaluate(&st, &pa.port, &pb.port, *axis_a_local);
            write_link_rates(xd_l, &[ev.standoff_rate_m_s, ev.dwell_rate]);
            push_wrench(
                vessels,
                link.end_a.target,
                WrenchAtSite {
                    site: link.end_a.component,
                    point_local_m: link.end_a.offset_local_m,
                    force_local_n: pa.rot_world_from_link.transpose() * ev.world_wrench_on_a.force,
                    torque_local_nm: pa.rot_world_from_link.transpose()
                        * ev.world_wrench_on_a.torque,
                },
            );
            push_wrench(
                vessels,
                link.end_b.target,
                WrenchAtSite {
                    site: link.end_b.component,
                    point_local_m: link.end_b.offset_local_m,
                    force_local_n: pb.rot_world_from_link.transpose() * ev.world_wrench_on_b.force,
                    torque_local_nm: pb.rot_world_from_link.transpose()
                        * ev.world_wrench_on_b.torque,
                },
            );
        }
        GroupLinkKind::Tether { spec, command } => {
            let st = WinchTetherState::from_blocks(x_l, &link.modes);
            let ends = TetherEnds {
                anchor_a_world_m: pa.port.position_world_m,
                vel_a_world_m_s: pa.port.velocity_world_m_s,
                anchor_b_world_m: pb.port.position_world_m,
                vel_b_world_m_s: pb.port.velocity_world_m_s,
            };
            let ev = spec.evaluate(&st, command, &ends);
            write_link_rates(
                xd_l,
                &[
                    ev.deployed_length_rate_m_s,
                    ev.absorbed_thermal_rate_w,
                    ev.low_tension_timer_rate,
                ],
            );
            push_wrench(
                vessels,
                link.end_a.target,
                WrenchAtSite {
                    site: link.end_a.component,
                    point_local_m: link.end_a.offset_local_m,
                    force_local_n: pa.rot_world_from_link.transpose() * ev.force_on_a_world_n,
                    torque_local_nm: DVec3::ZERO,
                },
            );
            push_wrench(
                vessels,
                link.end_b.target,
                WrenchAtSite {
                    site: link.end_b.component,
                    point_local_m: link.end_b.offset_local_m,
                    force_local_n: pb.rot_world_from_link.transpose() * ev.force_on_b_world_n,
                    torque_local_nm: DVec3::ZERO,
                },
            );
        }
    }
}

pub(super) fn describe_vessel(
    model: &MultibodyModel,
    elements: &VehicleElements,
    kinds: &mut [StateKind],
    magnitudes: &mut [f64],
) {
    let scale = model.scale();
    for (slot, kind) in kinds.iter_mut().zip(model.layout().kinds()) {
        *slot = *kind;
    }
    for (slot, kind) in magnitudes.iter_mut().zip(model.layout().kinds()) {
        *slot = match kind {
            StateKind::Length => scale.length_m(),
            StateKind::LinearRate => scale.speed_m_s(),
            StateKind::AngularRate => scale.angular_rate_rad_s(),
            StateKind::Attitude | StateKind::Angle | StateKind::Auxiliary => 1.0,
        };
    }
    if let Some(aux) = magnitudes.get_mut(model.layout().len()..) {
        ForceElement::<FlightEnvironment<'_>>::state_scale(elements, &scale, aux);
    }
}

pub(super) fn describe_link(link: &GroupLinkSlot, magnitudes: &mut [f64]) {
    match &link.kind {
        GroupLinkKind::Docking { spec, axis_a_local } => {
            let elem = DockingCapture {
                spec: *spec,
                port_a: PortAnchor::Link {
                    component: link.end_a.component,
                    offset_local_m: link.end_a.offset_local_m,
                },
                port_b: PortAnchor::Link {
                    component: link.end_b.component,
                    offset_local_m: link.end_b.offset_local_m,
                },
                axis_a_local: *axis_a_local,
            };
            ForceElement::<()>::state_scale(&elem, &link.scale, magnitudes);
        }
        GroupLinkKind::Tether { spec, command } => {
            let elem = Tether {
                spec: *spec,
                anchor_a: TetherAnchor::Link {
                    component: link.end_a.component,
                    offset_local_m: link.end_a.offset_local_m,
                },
                anchor_b: TetherAnchor::Link {
                    component: link.end_b.component,
                    offset_local_m: link.end_b.offset_local_m,
                },
                initial_length_m: 0.0,
                command: *command,
            };
            ForceElement::<()>::state_scale(&elem, &link.scale, magnitudes);
        }
    }
}

pub(super) fn transition_one_link(
    link: &mut GroupLinkSlot,
    pa: EndpointPose,
    pb: EndpointPose,
    x_l: &mut [f64],
) -> bool {
    match &link.kind {
        GroupLinkKind::Docking { spec, axis_a_local } => {
            let mut st = DockingState::from_blocks(x_l, &link.modes);
            let tr = spec.transition(&mut st, &pa.port, &pb.port, *axis_a_local);
            st.write_blocks(x_l, &mut link.modes);
            tr.changed
        }
        GroupLinkKind::Tether { spec, command } => {
            let before = WinchTetherState::from_blocks(x_l, &link.modes);
            let mut st = before;
            let ends = TetherEnds {
                anchor_a_world_m: pa.port.position_world_m,
                vel_a_world_m_s: pa.port.velocity_world_m_s,
                anchor_b_world_m: pb.port.position_world_m,
                vel_b_world_m_s: pb.port.velocity_world_m_s,
            };
            spec.transition(&mut st, command, &ends);
            st.write_blocks(x_l, &mut link.modes);
            st != before
        }
    }
}
