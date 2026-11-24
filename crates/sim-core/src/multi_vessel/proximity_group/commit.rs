//! Link slot construction and post-step commit helpers for proximity groups.

use std::collections::BTreeMap;

use mbd_multibody::{DockingPhase, DockingState, ModelScale, MultibodyError, WinchTetherState};

use super::ode::{EndpointRef, GroupLinkKind, GroupLinkSlot, VesselTarget};
use super::setup::{LinkOutcome, PreparedFree};
use crate::BlockError;
use crate::flight_model::integration::step_error;
use crate::flight_model::loads::LoadReadout;
use crate::multi_vessel::{
    COORD_BLOCK, InterVesselLink, MultiVesselCoordinator, SimVesselId, missing_vessel,
};

pub(super) fn build_group_links(
    coord: &MultiVesselCoordinator,
    link_indices: &[usize],
    targets: &BTreeMap<SimVesselId, VesselTarget>,
    free: &[PreparedFree],
    x: &mut Vec<f64>,
) -> Result<Vec<GroupLinkSlot>, BlockError> {
    let default_scale = ModelScale::new(1.0, 1.0, 1.0).map_err(|s| step_error(COORD_BLOCK, s))?;
    let scale_of = |target: VesselTarget| match target {
        VesselTarget::Free(idx) => free.get(idx).map(|f| f.block.model().scale()),
        VesselTarget::Pinned(_) => None,
    };
    let mut slots = Vec::with_capacity(link_indices.len());
    for &link_index in link_indices {
        let Some(link) = coord.links.get(link_index) else {
            continue;
        };
        if let Some(slot) = build_one_slot(link_index, link, targets, &scale_of, default_scale, x)?
        {
            slots.push(slot);
        }
    }
    Ok(slots)
}

fn build_one_slot(
    link_index: usize,
    link: &InterVesselLink,
    targets: &BTreeMap<SimVesselId, VesselTarget>,
    scale_of: &impl Fn(VesselTarget) -> Option<ModelScale>,
    default_scale: ModelScale,
    x: &mut Vec<f64>,
) -> Result<Option<GroupLinkSlot>, BlockError> {
    match *link {
        InterVesselLink::Docking { .. } => {
            build_docking_slot(link_index, link, targets, scale_of, default_scale, x)
        }
        InterVesselLink::WinchTether { .. } => {
            build_tether_slot(link_index, link, targets, scale_of, default_scale, x)
        }
        InterVesselLink::RailSlide { .. }
        | InterVesselLink::HingeSwing { .. }
        | InterVesselLink::PlumeLink { .. } => Ok(None),
    }
}

fn build_docking_slot(
    link_index: usize,
    link: &InterVesselLink,
    targets: &BTreeMap<SimVesselId, VesselTarget>,
    scale_of: &impl Fn(VesselTarget) -> Option<ModelScale>,
    default_scale: ModelScale,
    x: &mut Vec<f64>,
) -> Result<Option<GroupLinkSlot>, BlockError> {
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
    } = *link
    else {
        return Ok(None);
    };
    let ta = *targets.get(&chaser).ok_or_else(|| missing_vessel(chaser))?;
    let tb = *targets.get(&target).ok_or_else(|| missing_vessel(target))?;
    let start = x.len();
    let (mut blk, mut modes) = ([0.0; 2], vec![0.0; 1]);
    state.write_blocks(&mut blk, &mut modes);
    x.extend_from_slice(&blk);
    Ok(Some(GroupLinkSlot {
        link_index,
        end_a: EndpointRef {
            target: ta,
            component: port_a,
            offset_local_m: offset_a_local_m,
        },
        end_b: EndpointRef {
            target: tb,
            component: port_b,
            offset_local_m: offset_b_local_m,
        },
        kind: GroupLinkKind::Docking { spec, axis_a_local },
        modes,
        range: start..x.len(),
        scale: scale_of(ta)
            .or_else(|| scale_of(tb))
            .unwrap_or(default_scale),
    }))
}

fn build_tether_slot(
    link_index: usize,
    link: &InterVesselLink,
    targets: &BTreeMap<SimVesselId, VesselTarget>,
    scale_of: &impl Fn(VesselTarget) -> Option<ModelScale>,
    default_scale: ModelScale,
    x: &mut Vec<f64>,
) -> Result<Option<GroupLinkSlot>, BlockError> {
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
    } = *link
    else {
        return Ok(None);
    };
    let ta = *targets
        .get(&carrier)
        .ok_or_else(|| missing_vessel(carrier))?;
    let tb = *targets
        .get(&payload)
        .ok_or_else(|| missing_vessel(payload))?;
    let start = x.len();
    let (mut blk, mut modes) = ([0.0; 3], vec![0.0; 2]);
    state.write_blocks(&mut blk, &mut modes);
    x.extend_from_slice(&blk);
    Ok(Some(GroupLinkSlot {
        link_index,
        end_a: EndpointRef {
            target: ta,
            component: anchor_carrier,
            offset_local_m: offset_carrier_m,
        },
        end_b: EndpointRef {
            target: tb,
            component: anchor_payload,
            offset_local_m: offset_payload_m,
        },
        kind: GroupLinkKind::Tether { spec, command },
        modes,
        range: start..x.len(),
        scale: scale_of(ta)
            .or_else(|| scale_of(tb))
            .unwrap_or(default_scale),
    }))
}

pub(super) fn extract_link_outcomes(
    links: &[GroupLinkSlot],
    x: &[f64],
) -> Result<Vec<LinkOutcome>, BlockError> {
    let mut out = Vec::with_capacity(links.len());
    for link in links {
        let x_l = x.get(link.range.clone()).ok_or_else(|| {
            step_error(
                COORD_BLOCK,
                MultibodyError::StateLengthMismatch {
                    expected: link.range.end,
                    actual: x.len(),
                },
            )
        })?;
        match link.kind {
            GroupLinkKind::Docking { .. } => out.push(LinkOutcome::Docking {
                index: link.link_index,
                state: DockingState::from_blocks(x_l, &link.modes),
            }),
            GroupLinkKind::Tether { .. } => out.push(LinkOutcome::Tether {
                index: link.link_index,
                state: WinchTetherState::from_blocks(x_l, &link.modes),
            }),
        }
    }
    Ok(out)
}

pub(super) fn commit_group_results(
    coord: &mut MultiVesselCoordinator,
    free: &mut Vec<PreparedFree>,
    readouts: &[LoadReadout],
    outcomes: &[LinkOutcome],
) -> Result<(), BlockError> {
    for (item, &readout) in free.iter_mut().zip(readouts) {
        item.block
            .finish_group_step(&item.env_sample, &item.sub_out, &item.state, readout)?;
    }
    for item in free.drain(..) {
        coord.vessels.insert(item.id, item.block);
    }
    for outcome in outcomes {
        match *outcome {
            LinkOutcome::Docking { index, state } => {
                if let Some(InterVesselLink::Docking {
                    chaser,
                    port_a,
                    target,
                    port_b,
                    mate_placement,
                    state: dst,
                    ..
                }) = coord.links.get_mut(index)
                {
                    *dst = state;
                    if state.phase == DockingPhase::HardLatched {
                        coord.pending_hard_mates.push((
                            *chaser,
                            *port_a,
                            *target,
                            *port_b,
                            *mate_placement,
                        ));
                    }
                }
            }
            LinkOutcome::Tether { index, state } => {
                if let Some(InterVesselLink::WinchTether { state: dst, .. }) =
                    coord.links.get_mut(index)
                {
                    *dst = state;
                }
            }
        }
    }
    Ok(())
}
