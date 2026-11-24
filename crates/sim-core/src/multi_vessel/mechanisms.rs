use std::collections::BTreeMap;

use glam::DVec3;
use mbd_core::{ComponentId, Placement};
use mbd_multibody::{
    DockingPhase, GrappleLockSpec, HardMateResult, HingeRelease, ImpulseReleaseResult, RailSlide,
    grapple_lock,
};

use super::plume::site_world_kinematics;
use super::{
    COORD_BLOCK, InterVesselLink, MultiVesselCoordinator, MultiVesselStepReport, SimVesselId,
    map_multi_coord, map_sys_coord, missing_vessel,
};
use crate::BlockError;
use crate::flight_model::coupled_mechanism::{CoupledMechanism, CoupledMechanismState};
use crate::flight_model::planet_frame::{
    fixed_to_instantaneous_inertial, inertial_to_fixed_after_step,
};
use crate::multibody_vehicle::{
    MultibodyVehicleInput, merge_aero_remapped, merge_subsystems_remapped, remap_aero_subset,
    remap_subsystems_subset,
};

/// Which vessel integrates a single-vessel mechanism's state this step, and
/// where in that vessel's `mechanisms` input it sits.
pub(super) type LinkOwner = Option<(SimVesselId, usize)>;

fn couple(
    step_inputs: &mut BTreeMap<SimVesselId, MultibodyVehicleInput>,
    vessel: SimVesselId,
    mechanism: CoupledMechanism,
) -> LinkOwner {
    let input = step_inputs.get_mut(&vessel)?;
    input.mechanisms.push(mechanism);
    Some((vessel, input.mechanisms.len().saturating_sub(1)))
}

/// Attaches single-vessel `RailSlide` and `HingeSwing` mechanisms onto their
/// owner vessels' `MultibodyVehicleInput::mechanisms` for the upcoming step.
pub(super) fn couple_single_vessel_mechanisms(
    coord: &MultiVesselCoordinator,
    step_inputs: &mut BTreeMap<SimVesselId, MultibodyVehicleInput>,
) -> Vec<LinkOwner> {
    let mut owners = Vec::with_capacity(coord.links.len());
    for link in &coord.links {
        let owner = match link {
            InterVesselLink::RailSlide {
                vessel,
                spec,
                state,
                ..
            } => couple(
                step_inputs,
                *vessel,
                CoupledMechanism::Rail(
                    RailSlide {
                        spec: *spec,
                        fire: state.fired,
                    },
                    *state,
                ),
            ),
            InterVesselLink::HingeSwing {
                vessel,
                spec,
                state,
                ..
            } => couple(
                step_inputs,
                *vessel,
                CoupledMechanism::Hinge(
                    HingeRelease {
                        spec: *spec,
                        unlatch: state.unlatched,
                    },
                    *state,
                ),
            ),
            InterVesselLink::Docking { .. }
            | InterVesselLink::WinchTether { .. }
            | InterVesselLink::PlumeLink { .. } => None,
        };
        owners.push(owner);
    }
    owners
}

/// Reads back ended states of single-vessel mechanisms (`RailSlide` and
/// `HingeSwing`) and queues triggered separations.
pub(super) fn take_link_states(coord: &mut MultiVesselCoordinator, owners: &[LinkOwner]) {
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

fn lookup_comp(map: &[(ComponentId, ComponentId)], old: ComponentId) -> Option<ComponentId> {
    map.iter().find(|&&(k, _)| k == old).map(|&(_, v)| v)
}

fn remap_links_after_split(
    coord: &mut MultiVesselCoordinator,
    parent_id: SimVesselId,
    child_id: SimVesselId,
    release: &ImpulseReleaseResult,
) {
    let map_ep = |vessel: SimVesselId, comp: ComponentId| -> Option<(SimVesselId, ComponentId)> {
        if vessel != parent_id {
            return Some((vessel, comp));
        }
        if let Some(mapped) = lookup_comp(&release.parent_id_map, comp) {
            Some((parent_id, mapped))
        } else {
            lookup_comp(&release.child_id_map, comp).map(|mapped| (child_id, mapped))
        }
    };
    remap_link_endpoints(&mut coord.links, map_ep, false);
    for (v, comp, _, _) in &mut coord.pending_releases {
        if let Some((nv, nc)) = map_ep(*v, *comp) {
            (*v, *comp) = (nv, nc);
        }
    }
    for (va, ca, vb, cb, _) in &mut coord.pending_hard_mates {
        if let Some((nva, nca)) = map_ep(*va, *ca) {
            (*va, *ca) = (nva, nca);
        }
        if let Some((nvb, ncb)) = map_ep(*vb, *cb) {
            (*vb, *cb) = (nvb, ncb);
        }
    }
}

fn remap_links_after_mate(
    coord: &mut MultiVesselCoordinator,
    chaser_id: SimVesselId,
    target_id: SimVesselId,
    res: &HardMateResult,
) {
    let map_ep = |vessel: SimVesselId, comp: ComponentId| -> Option<(SimVesselId, ComponentId)> {
        if vessel == chaser_id {
            lookup_comp(&res.a_id_map, comp).map(|nc| (chaser_id, nc))
        } else if vessel == target_id {
            lookup_comp(&res.b_id_map, comp).map(|nc| (chaser_id, nc))
        } else {
            Some((vessel, comp))
        }
    };
    remap_link_endpoints(&mut coord.links, map_ep, true);
    for (v, comp, _, _) in &mut coord.pending_releases {
        if let Some((nv, nc)) = map_ep(*v, *comp) {
            (*v, *comp) = (nv, nc);
        }
    }
    for (va, ca, vb, cb, _) in &mut coord.pending_hard_mates {
        if let Some((nva, nca)) = map_ep(*va, *ca) {
            (*va, *ca) = (nva, nca);
        }
        if let Some((nvb, ncb)) = map_ep(*vb, *cb) {
            (*vb, *cb) = (nvb, ncb);
        }
    }
}

fn remap_pair(
    va: &mut SimVesselId,
    ca: &mut ComponentId,
    vb: &mut SimVesselId,
    cb: &mut ComponentId,
    map_ep: &impl Fn(SimVesselId, ComponentId) -> Option<(SimVesselId, ComponentId)>,
    drop_self_two_vessel: bool,
) -> bool {
    let Some((na, nca)) = map_ep(*va, *ca) else {
        return false;
    };
    let Some((nb, ncb)) = map_ep(*vb, *cb) else {
        return false;
    };
    (*va, *ca, *vb, *cb) = (na, nca, nb, ncb);
    !(drop_self_two_vessel && na == nb)
}

fn remap_single(
    vessel: &mut SimVesselId,
    component: &mut ComponentId,
    map_ep: &impl Fn(SimVesselId, ComponentId) -> Option<(SimVesselId, ComponentId)>,
) -> bool {
    let Some((nv, nc)) = map_ep(*vessel, *component) else {
        return false;
    };
    (*vessel, *component) = (nv, nc);
    true
}

fn remap_link_endpoints(
    links: &mut Vec<InterVesselLink>,
    map_ep: impl Fn(SimVesselId, ComponentId) -> Option<(SimVesselId, ComponentId)>,
    drop_self_two_vessel: bool,
) {
    let taken = core::mem::take(links);
    for mut link in taken {
        let keep = match &mut link {
            InterVesselLink::RailSlide { vessel, spec, .. } => {
                remap_single(vessel, &mut spec.component, &map_ep)
            }
            InterVesselLink::HingeSwing { vessel, spec, .. } => {
                remap_single(vessel, &mut spec.component, &map_ep)
            }
            InterVesselLink::Docking {
                chaser,
                port_a,
                target,
                port_b,
                ..
            } => remap_pair(
                chaser,
                port_a,
                target,
                port_b,
                &map_ep,
                drop_self_two_vessel,
            ),
            InterVesselLink::WinchTether {
                carrier,
                anchor_carrier,
                payload,
                anchor_payload,
                ..
            } => remap_pair(
                carrier,
                anchor_carrier,
                payload,
                anchor_payload,
                &map_ep,
                drop_self_two_vessel,
            ),
            InterVesselLink::PlumeLink {
                source_vessel,
                nozzle_site,
                target_vessel,
                target_site,
                ..
            } => remap_pair(
                source_vessel,
                nozzle_site,
                target_vessel,
                target_site,
                &map_ep,
                drop_self_two_vessel,
            ),
        };
        if keep {
            links.push(link);
        }
    }
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

    let child_block = parent.spawn_detached(release.child_tree.clone(), c_fixed, c_subs, c_aero)?;
    let parent_mut = coord
        .vessels
        .get_mut(&parent_id)
        .ok_or_else(|| missing_vessel(parent_id))?;
    parent_mut.replace_topology_state(release.parent_tree.clone(), p_fixed, p_subs, p_aero)?;

    let child_id = coord.spawn_independent(child_block);
    remap_links_after_split(coord, parent_id, child_id, &release);
    Ok(child_id)
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
    chaser_mut.replace_topology_state(res.tree.clone(), merged_fixed, merged_subs, merged_aero)?;
    coord.vessels.remove(&target_id);
    if coord.active_vessel == target_id {
        coord.active_vessel = chaser_id;
    }
    remap_links_after_mate(coord, chaser_id, target_id, &res);
    Ok(())
}

pub(super) fn grapple_lock_impl(
    coord: &mut MultiVesselCoordinator,
    spec: &GrappleLockSpec,
    servicer_id: SimVesselId,
    manipulator_name: &str,
    target_id: SimVesselId,
    fixture_component: ComponentId,
    mate_placement: Placement,
) -> Result<(), BlockError> {
    let servicer = coord
        .vessels
        .get(&servicer_id)
        .ok_or_else(|| missing_vessel(servicer_id))?;
    let target = coord
        .vessels
        .get(&target_id)
        .ok_or_else(|| missing_vessel(target_id))?;
    let chain = servicer
        .subsystems()
        .manipulators
        .iter()
        .find(|(n, _)| n == manipulator_name)
        .map(|(_, c)| c)
        .ok_or_else(|| BlockError::InputOutOfRange {
            block: COORD_BLOCK,
            input: "manipulator_name",
            detail: format!("unknown manipulator {manipulator_name:?}"),
        })?;
    let ee_comp = chain.ee_component();
    let servicer_i =
        fixed_to_instantaneous_inertial(&servicer.floating_base_fixed(), &servicer.body());
    let target_i = fixed_to_instantaneous_inertial(&target.floating_base_fixed(), &target.body());
    let (ee_pos, ee_rot, ee_vel, _) = site_world_kinematics(
        servicer.pose_table(),
        &servicer_i,
        ee_comp,
        chain.ee_offset_local_m(),
    )?;
    let (fix_pos, fix_rot, fix_vel, _) = site_world_kinematics(
        target.pose_table(),
        &target_i,
        fixture_component,
        DVec3::ZERO,
    )?;
    let ma = servicer.dynamic_mass_contributions()?;
    let mb = target.dynamic_mass_contributions()?;
    let res = grapple_lock(
        spec,
        ee_pos,
        ee_rot,
        ee_vel,
        fix_pos,
        fix_rot,
        fix_vel,
        servicer.tree(),
        &ma,
        &servicer_i,
        ee_comp,
        target.tree(),
        &mb,
        &target_i,
        fixture_component,
        mate_placement,
    )
    .map_err(map_multi_coord)?;
    mate_vessels_from_result(coord, servicer_id, target_id, res)
}
