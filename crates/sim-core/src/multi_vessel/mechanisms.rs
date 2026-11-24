use std::collections::BTreeMap;

use glam::DVec3;
use mbd_core::ComponentId;
use mbd_multibody::{
    DockingCapture, DockingPhase, HardMateResult, HingeRelease, ImpulseReleaseResult,
    PinnedSurfaceRest, PortAnchor, PortKinematics, RailSlide, Tether, TetherAnchor,
};

use super::plume::{eval_plume_link, site_world_kinematics};
use super::{
    InterVesselLink, MultiVesselCoordinator, MultiVesselStepReport, SimVesselId, map_sys_coord,
    missing_vessel,
};
use crate::BlockError;
use crate::flight_model::coupled_mechanism::{CoupledMechanism, CoupledMechanismState};
use crate::flight_model::planet_frame::{
    fixed_to_instantaneous_inertial, inertial_to_fixed_after_step,
};
use crate::multibody_vehicle::{
    MultibodyVehicleBlock, MultibodyVehicleInput, merge_aero_remapped, merge_subsystems_remapped,
    remap_aero_subset, remap_subsystems_subset,
};

/// Which vessel integrates a link's state this step, and where in that
/// vessel's mechanisms it sits.
pub(super) type LinkOwner = Option<(SimVesselId, usize)>;

/// Appends `mechanism` to the step input of `vessel` and returns where it
/// landed.
fn couple(
    step_inputs: &mut BTreeMap<SimVesselId, MultibodyVehicleInput>,
    vessel: SimVesselId,
    mechanism: CoupledMechanism,
) -> LinkOwner {
    let input = step_inputs.get_mut(&vessel)?;
    input.mechanisms.push(mechanism);
    Some((vessel, input.mechanisms.len().saturating_sub(1)))
}

/// Builds each vessel's input for the step: the caller's commands plus the
/// mechanisms and loads of every link, with the far end of a two-vessel link
/// given as it is at the start of the step. Returns, per link, the vessel
/// that carries its state.
pub(super) fn prepare_step_inputs(
    coord: &MultiVesselCoordinator,
    inputs: &BTreeMap<SimVesselId, MultibodyVehicleInput>,
) -> Result<(BTreeMap<SimVesselId, MultibodyVehicleInput>, Vec<LinkOwner>), BlockError> {
    let mut step_inputs = BTreeMap::new();
    for id in coord.vessel_ids() {
        step_inputs.insert(id, inputs.get(&id).cloned().unwrap_or_default());
    }
    let mut owners = Vec::with_capacity(coord.links.len());
    for link in &coord.links {
        owners.push(couple_link(&coord.vessels, link, &mut step_inputs)?);
    }
    Ok((step_inputs, owners))
}

fn couple_link(
    vessels: &BTreeMap<SimVesselId, MultibodyVehicleBlock>,
    link: &InterVesselLink,
    step_inputs: &mut BTreeMap<SimVesselId, MultibodyVehicleInput>,
) -> Result<LinkOwner, BlockError> {
    match link {
        InterVesselLink::RailSlide {
            vessel,
            spec,
            state,
            ..
        } => {
            let element = RailSlide {
                spec: *spec,
                fire: state.fired,
            };
            Ok(couple(
                step_inputs,
                *vessel,
                CoupledMechanism::Rail(element, *state),
            ))
        }
        InterVesselLink::HingeSwing {
            vessel,
            spec,
            state,
            ..
        } => {
            let element = HingeRelease {
                spec: *spec,
                unlatch: state.unlatched,
            };
            Ok(couple(
                step_inputs,
                *vessel,
                CoupledMechanism::Hinge(element, *state),
            ))
        }
        InterVesselLink::Docking { .. } => couple_docking(vessels, link, step_inputs),
        InterVesselLink::WinchTether { .. } => couple_tether(vessels, link, step_inputs),
        InterVesselLink::PlumeLink { .. } => {
            eval_plume_link(vessels, link, step_inputs)?;
            Ok(None)
        }
    }
}

/// Which of the two copies of a link's state is kept: the first vessel's,
/// unless that vessel is latched at rest on a surface and so does not
/// integrate its mechanisms this step.
fn stepping_owner(
    first: &MultibodyVehicleBlock,
    on_first: LinkOwner,
    on_second: LinkOwner,
) -> LinkOwner {
    if matches!(first.pinned_rest(), PinnedSurfaceRest::Pinned { .. }) {
        on_second.or(on_first)
    } else {
        on_first.or(on_second)
    }
}

/// A port of `vessel` at the start of the coming step, in the inertial axes
/// that coincide with the fixed ones then: what the other end of a link is
/// given, to carry on at this velocity over the step.
fn port_at_step_start(
    vessel: &MultibodyVehicleBlock,
    port: ComponentId,
    offset_local_m: DVec3,
) -> Result<PortKinematics, BlockError> {
    let axes = fixed_to_instantaneous_inertial(&vessel.body_axes(), &vessel.body());
    let (position_world_m, rotation_world, velocity_world_m_s, angular_vel_world_rad_s) =
        site_world_kinematics(vessel.pose_table(), &axes, port, offset_local_m)?;
    Ok(PortKinematics {
        position_world_m,
        rotation_world,
        velocity_world_m_s,
        angular_vel_world_rad_s,
    })
}

/// Gives each vessel its end of the ring and the other vessel's port as it
/// starts the step. The chaser's copy of the ring's state is the one kept,
/// unless the chaser is resting on a surface.
fn couple_docking(
    vessels: &BTreeMap<SimVesselId, MultibodyVehicleBlock>,
    link: &InterVesselLink,
    step_inputs: &mut BTreeMap<SimVesselId, MultibodyVehicleInput>,
) -> Result<LinkOwner, BlockError> {
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
        return Ok(None);
    };
    let (Some(va), Some(vb)) = (vessels.get(chaser), vessels.get(target)) else {
        return Ok(None);
    };
    let on_chaser = PortAnchor::Link {
        component: *port_a,
        offset_local_m: *offset_a_local_m,
    };
    let on_target = PortAnchor::Link {
        component: *port_b,
        offset_local_m: *offset_b_local_m,
    };
    let chaser_port = port_at_step_start(va, *port_a, *offset_a_local_m)?;
    let target_port = port_at_step_start(vb, *port_b, *offset_b_local_m)?;
    let ring = |port_a, port_b| DockingCapture {
        spec: *spec,
        port_a,
        port_b,
        axis_a_local: *axis_a_local,
    };
    let on_first = couple(
        step_inputs,
        *chaser,
        CoupledMechanism::Docking(ring(on_chaser, PortAnchor::World(target_port)), *state),
    );
    let on_second = couple(
        step_inputs,
        *target,
        CoupledMechanism::Docking(ring(PortAnchor::World(chaser_port), on_target), *state),
    );
    Ok(stepping_owner(va, on_first, on_second))
}

/// Gives each vessel its end of the line and the other vessel's anchor as
/// it starts the step. The carrier's copy of the line's state is the one
/// kept, unless the carrier is resting on a surface.
fn couple_tether(
    vessels: &BTreeMap<SimVesselId, MultibodyVehicleBlock>,
    link: &InterVesselLink,
    step_inputs: &mut BTreeMap<SimVesselId, MultibodyVehicleInput>,
) -> Result<LinkOwner, BlockError> {
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
        return Ok(None);
    };
    let (Some(va), Some(vb)) = (vessels.get(carrier), vessels.get(payload)) else {
        return Ok(None);
    };
    let held = |port: PortKinematics| TetherAnchor::World {
        position_world_m: port.position_world_m,
        velocity_world_m_s: port.velocity_world_m_s,
    };
    let carrier_end = held(port_at_step_start(va, *anchor_carrier, *offset_carrier_m)?);
    let payload_end = held(port_at_step_start(vb, *anchor_payload, *offset_payload_m)?);
    let line = |anchor_a, anchor_b| Tether {
        spec: *spec,
        anchor_a,
        anchor_b,
        initial_length_m: state.deployed_length_m,
        command: *command,
    };
    let on_carrier = TetherAnchor::Link {
        component: *anchor_carrier,
        offset_local_m: *offset_carrier_m,
    };
    let on_payload = TetherAnchor::Link {
        component: *anchor_payload,
        offset_local_m: *offset_payload_m,
    };
    let on_first = couple(
        step_inputs,
        *carrier,
        CoupledMechanism::Tether(line(on_carrier, payload_end), *state),
    );
    let on_second = couple(
        step_inputs,
        *payload,
        CoupledMechanism::Tether(line(carrier_end, on_payload), *state),
    );
    Ok(stepping_owner(va, on_first, on_second))
}

/// Takes the state each link ended the step in from the vessel that carried
/// it, and queues the releases and hard mates those states call for.
pub(super) fn take_link_states(coord: &mut MultiVesselCoordinator, owners: &[LinkOwner]) {
    coord.pending_releases.clear();
    coord.pending_hard_mates.clear();
    for (link, owner) in coord.links.iter_mut().zip(owners) {
        let Some((vessel, index)) = owner else {
            continue;
        };
        let Some(ended) = coord
            .vessels
            .get(vessel)
            .and_then(|v| v.mechanism_states().get(*index))
        else {
            continue;
        };
        match (link, ended) {
            (
                InterVesselLink::RailSlide {
                    vessel,
                    spec,
                    state,
                    ..
                },
                CoupledMechanismState::Rail(ended),
            ) => {
                *state = *ended;
                if ended.ready_to_release {
                    coord
                        .pending_releases
                        .push((*vessel, spec.component, 0.0, DVec3::X));
                }
            }
            (
                InterVesselLink::HingeSwing {
                    vessel,
                    spec,
                    state,
                    ..
                },
                CoupledMechanismState::Hinge(ended),
            ) => {
                *state = *ended;
                if ended.ready_to_release {
                    coord
                        .pending_releases
                        .push((*vessel, spec.component, 0.0, DVec3::X));
                }
            }
            (
                InterVesselLink::Docking {
                    chaser,
                    port_a,
                    target,
                    port_b,
                    mate_placement,
                    state,
                    ..
                },
                CoupledMechanismState::Docking(ended),
            ) => {
                *state = *ended;
                if ended.phase == DockingPhase::HardLatched {
                    coord.pending_hard_mates.push((
                        *chaser,
                        *port_a,
                        *target,
                        *port_b,
                        *mate_placement,
                    ));
                }
            }
            (InterVesselLink::WinchTether { state, .. }, CoupledMechanismState::Tether(ended)) => {
                *state = *ended;
            }
            _ => {}
        }
    }
}

pub(super) fn process_post_step_links(
    coord: &mut MultiVesselCoordinator,
) -> Result<MultiVesselStepReport, BlockError> {
    let mut report = MultiVesselStepReport::default();
    let releases = core::mem::take(&mut coord.pending_releases);
    for (parent_id, child_root, impulse, axis) in releases {
        if coord.vessels.contains_key(&parent_id) {
            let child_id = coord.separate_stage_impulse(parent_id, child_root, impulse, axis)?;
            report.spawned_vessels.push((parent_id, child_id));
        }
    }
    let mates = core::mem::take(&mut coord.pending_hard_mates);
    for (chaser, port_a, target, port_b, mate_place) in mates {
        if coord.vessels.contains_key(&chaser) && coord.vessels.contains_key(&target) {
            coord.hard_mate(chaser, port_a, target, port_b, mate_place)?;
            report.merged_vessels.push((chaser, target));
        }
    }
    for link in &coord.links {
        if let InterVesselLink::WinchTether { id, state, .. } = link
            && !state.intact
            && let Some(reason) = state.cut_reason
        {
            report.cut_tethers.push((*id, reason));
        }
    }
    coord.links.retain(|l| match l {
        InterVesselLink::Docking { state, .. } => !matches!(state.phase, DockingPhase::HardLatched),
        InterVesselLink::WinchTether { state, .. } => state.intact,
        InterVesselLink::RailSlide { state, .. } => !state.ready_to_release,
        InterVesselLink::HingeSwing { state, .. } => !state.ready_to_release,
        InterVesselLink::PlumeLink { .. } => true,
    });
    Ok(report)
}

pub(super) fn split_vessel_with_release(
    coord: &mut MultiVesselCoordinator,
    parent_id: SimVesselId,
    release: ImpulseReleaseResult,
) -> Result<SimVesselId, BlockError> {
    let parent = coord
        .vessels
        .get(&parent_id)
        .ok_or_else(|| missing_vessel(parent_id))?;
    let body = parent.body();
    let p_fixed = inertial_to_fixed_after_step(&release.parent_state, &body, 0.0);
    let c_fixed = inertial_to_fixed_after_step(&release.child_state, &body, 0.0);

    let p_subs = remap_subsystems_subset(
        parent.subsystems(),
        &release.parent_tree,
        &release.parent_id_map,
    )
    .map_err(map_sys_coord)?;
    let c_subs = remap_subsystems_subset(
        parent.subsystems(),
        &release.child_tree,
        &release.child_id_map,
    )
    .map_err(map_sys_coord)?;
    let aero = parent.aero_with_states();
    let p_aero = remap_aero_subset(&aero, &release.parent_id_map);
    let c_aero = remap_aero_subset(&aero, &release.child_id_map);

    let child_block = parent.spawn_detached(release.child_tree, c_fixed, c_subs, c_aero)?;
    let parent_mut = coord
        .vessels
        .get_mut(&parent_id)
        .ok_or_else(|| missing_vessel(parent_id))?;
    parent_mut.replace_topology_state(release.parent_tree, p_fixed, p_subs, p_aero)?;

    Ok(coord.spawn_independent(child_block))
}

pub(super) fn mate_vessels_from_result(
    coord: &mut MultiVesselCoordinator,
    chaser_id: SimVesselId,
    target_id: SimVesselId,
    res: HardMateResult,
) -> Result<(), BlockError> {
    let chaser = coord
        .vessels
        .get(&chaser_id)
        .ok_or_else(|| missing_vessel(chaser_id))?;
    let target = coord
        .vessels
        .get(&target_id)
        .ok_or_else(|| missing_vessel(target_id))?;
    let body = chaser.body();
    let merged_fixed = inertial_to_fixed_after_step(&res.base_state, &body, 0.0);
    let merged_subs = merge_subsystems_remapped(
        chaser.subsystems(),
        &res.a_id_map,
        target.subsystems(),
        &res.b_id_map,
        &res.tree,
    )
    .map_err(map_sys_coord)?;
    let merged_aero = merge_aero_remapped(
        &chaser.aero_with_states(),
        &res.a_id_map,
        &target.aero_with_states(),
        &res.b_id_map,
    );

    let chaser_mut = coord
        .vessels
        .get_mut(&chaser_id)
        .ok_or_else(|| missing_vessel(chaser_id))?;
    chaser_mut.replace_topology_state(res.tree, merged_fixed, merged_subs, merged_aero)?;
    coord.vessels.remove(&target_id);
    if coord.active_vessel == target_id {
        coord.active_vessel = chaser_id;
    }
    Ok(())
}
