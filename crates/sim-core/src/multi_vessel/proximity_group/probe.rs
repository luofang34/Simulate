//! Component validation, rollback snapshots, and initial link-wrench probing
//! for `PinnedSurfaceRest` vessels in a proximity group.

use std::collections::BTreeMap;

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, WrenchAtSite};
use mbd_multibody::PortKinematics;
use sim_geo::BodyEllipsoid;

use super::ode::EndpointPose;
use crate::flight_model::planet_frame::fixed_to_instantaneous_inertial;
use crate::multi_vessel::plume::site_world_kinematics;
use crate::multi_vessel::{
    COORD_BLOCK, InterVesselLink, MultiVesselCoordinator, SimVesselId, missing_vessel,
};
use crate::multibody_vehicle::{MultibodyVehicleBlock, MultibodyVehicleSnapshot};
use crate::{BlockError, TickSpan};

pub(super) struct ComponentBackup {
    vessels: Vec<(SimVesselId, MultibodyVehicleSnapshot)>,
    links: Vec<(usize, InterVesselLink)>,
}

pub(super) fn capture_backup(
    coord: &MultiVesselCoordinator,
    vessels: &[SimVesselId],
    link_indices: &[usize],
) -> ComponentBackup {
    let vs = vessels
        .iter()
        .filter_map(|id| coord.vessels.get(id).map(|v| (*id, v.snapshot_for_coord())))
        .collect();
    let ls = link_indices
        .iter()
        .filter_map(|&idx| coord.links.get(idx).map(|l| (idx, l.clone())))
        .collect();
    ComponentBackup {
        vessels: vs,
        links: ls,
    }
}

pub(super) fn restore_backup(
    coord: &mut MultiVesselCoordinator,
    mut freed: Vec<(SimVesselId, MultibodyVehicleBlock)>,
    backup: ComponentBackup,
) {
    for (id, block) in freed.drain(..) {
        coord.vessels.insert(id, block);
    }
    for (id, snap) in backup.vessels {
        if let Some(v) = coord.vessels.get_mut(&id) {
            v.try_restore(&snap).ok();
        }
    }
    for (idx, link) in backup.links {
        if let Some(slot) = coord.links.get_mut(idx) {
            *slot = link;
        }
    }
}

pub(super) fn validate_component(
    coord: &MultiVesselCoordinator,
    vessels: &[SimVesselId],
) -> Result<(TickSpan, BodyEllipsoid), BlockError> {
    let first_id = vessels
        .first()
        .copied()
        .ok_or_else(|| missing_vessel(SimVesselId(0)))?;
    let first = coord
        .vessels
        .get(&first_id)
        .ok_or_else(|| missing_vessel(first_id))?;
    let period = first.config().period;
    let body = first.body();
    for &id in vessels {
        let v = coord.vessels.get(&id).ok_or_else(|| missing_vessel(id))?;
        if v.config().period != period {
            return Err(BlockError::InputOutOfRange {
                block: COORD_BLOCK,
                input: "period",
                detail: format!(
                    "coupled vessel #{} period {:?} differs from {:?}",
                    id.0,
                    v.config().period,
                    period
                ),
            });
        }
        if v.body() != body {
            return Err(BlockError::InputOutOfRange {
                block: COORD_BLOCK,
                input: "body",
                detail: format!("coupled vessel #{} central body differs", id.0),
            });
        }
    }
    Ok((period, body))
}

fn start_endpoint_pose(
    vessel: &MultibodyVehicleBlock,
    comp: ComponentId,
    offset_local_m: DVec3,
) -> Result<EndpointPose, BlockError> {
    let base_i = fixed_to_instantaneous_inertial(&vessel.floating_base_fixed(), &vessel.body());
    let (pos, rot, vel, omega) =
        site_world_kinematics(vessel.pose_table(), &base_i, comp, offset_local_m)?;
    Ok(EndpointPose {
        port: PortKinematics {
            position_world_m: pos,
            rotation_world: rot,
            velocity_world_m_s: vel,
            angular_vel_world_rad_s: omega,
        },
        rot_world_from_link: DMat3::from_quat(rot),
    })
}

pub(super) fn initial_link_wrenches(
    coord: &MultiVesselCoordinator,
    link_indices: &[usize],
) -> Result<BTreeMap<SimVesselId, Vec<WrenchAtSite>>, BlockError> {
    let mut out: BTreeMap<SimVesselId, Vec<WrenchAtSite>> = BTreeMap::new();
    for &idx in link_indices {
        let Some(link) = coord.links.get(idx) else {
            continue;
        };
        add_initial_wrenches_for_link(coord, link, &mut out)?;
    }
    Ok(out)
}

fn push_world_wrench(
    out: &mut BTreeMap<SimVesselId, Vec<WrenchAtSite>>,
    vessel: SimVesselId,
    site: ComponentId,
    offset: DVec3,
    rot_world_from_link: glam::DMat3,
    force_world_n: DVec3,
    torque_world_nm: DVec3,
) {
    out.entry(vessel).or_default().push(WrenchAtSite {
        site,
        point_local_m: offset,
        force_local_n: rot_world_from_link.transpose() * force_world_n,
        torque_local_nm: rot_world_from_link.transpose() * torque_world_nm,
    });
}

fn add_initial_wrenches_for_link(
    coord: &MultiVesselCoordinator,
    link: &InterVesselLink,
    out: &mut BTreeMap<SimVesselId, Vec<WrenchAtSite>>,
) -> Result<(), BlockError> {
    match link {
        InterVesselLink::Docking { .. } => add_docking_wrenches(coord, link, out),
        InterVesselLink::WinchTether { .. } => add_tether_wrenches(coord, link, out),
        InterVesselLink::RailSlide { .. }
        | InterVesselLink::HingeSwing { .. }
        | InterVesselLink::PlumeLink { .. } => Ok(()),
    }
}

fn add_docking_wrenches(
    coord: &MultiVesselCoordinator,
    link: &InterVesselLink,
    out: &mut BTreeMap<SimVesselId, Vec<WrenchAtSite>>,
) -> Result<(), BlockError> {
    let InterVesselLink::Docking {
        chaser,
        port_a,
        offset_a_local_m,
        axis_a_local,
        target,
        port_b,
        offset_b_local_m,
        spec,
        state,
        ..
    } = link
    else {
        return Ok(());
    };
    let (Some(va), Some(vb)) = (coord.vessels.get(chaser), coord.vessels.get(target)) else {
        return Ok(());
    };
    let pa = start_endpoint_pose(va, *port_a, *offset_a_local_m)?;
    let pb = start_endpoint_pose(vb, *port_b, *offset_b_local_m)?;
    let ev = spec.evaluate(state, &pa.port, &pb.port, *axis_a_local);
    push_world_wrench(
        out,
        *chaser,
        *port_a,
        *offset_a_local_m,
        pa.rot_world_from_link,
        ev.world_wrench_on_a.force,
        ev.world_wrench_on_a.torque,
    );
    push_world_wrench(
        out,
        *target,
        *port_b,
        *offset_b_local_m,
        pb.rot_world_from_link,
        ev.world_wrench_on_b.force,
        ev.world_wrench_on_b.torque,
    );
    Ok(())
}

fn add_tether_wrenches(
    coord: &MultiVesselCoordinator,
    link: &InterVesselLink,
    out: &mut BTreeMap<SimVesselId, Vec<WrenchAtSite>>,
) -> Result<(), BlockError> {
    let InterVesselLink::WinchTether {
        carrier,
        anchor_carrier,
        offset_carrier_m,
        payload,
        anchor_payload,
        offset_payload_m,
        spec,
        state,
        command,
        ..
    } = link
    else {
        return Ok(());
    };
    let (Some(va), Some(vb)) = (coord.vessels.get(carrier), coord.vessels.get(payload)) else {
        return Ok(());
    };
    let pa = start_endpoint_pose(va, *anchor_carrier, *offset_carrier_m)?;
    let pb = start_endpoint_pose(vb, *anchor_payload, *offset_payload_m)?;
    let ends = mbd_multibody::TetherEnds {
        anchor_a_world_m: pa.port.position_world_m,
        vel_a_world_m_s: pa.port.velocity_world_m_s,
        anchor_b_world_m: pb.port.position_world_m,
        vel_b_world_m_s: pb.port.velocity_world_m_s,
    };
    let ev = spec.evaluate(state, command, &ends);
    push_world_wrench(
        out,
        *carrier,
        *anchor_carrier,
        *offset_carrier_m,
        pa.rot_world_from_link,
        ev.force_on_a_world_n,
        DVec3::ZERO,
    );
    push_world_wrench(
        out,
        *payload,
        *anchor_payload,
        *offset_payload_m,
        pb.rot_world_from_link,
        ev.force_on_b_world_n,
        DVec3::ZERO,
    );
    Ok(())
}
