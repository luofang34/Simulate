//! Connected-component partitioning and shared-integrator stepping for coupled
//! proximity groups (`Docking` and `WinchTether` links).

use std::collections::{BTreeMap, BTreeSet};

use super::mechanisms::{
    couple_single_vessel_mechanisms, process_post_step_links, take_link_states,
};
use super::plume::eval_plume_link;
use super::{InterVesselLink, MultiVesselCoordinator, MultiVesselStepReport, SimVesselId};
use crate::BlockError;
use crate::multibody_vehicle::MultibodyVehicleInput;

mod commit;
mod links;
mod ode;
mod probe;
mod setup;

use setup::step_proximity_group;

/// Active two-vessel link kinds that couple vessels inside a shared integrator.
fn two_vessel_endpoints(
    coord: &MultiVesselCoordinator,
    link: &InterVesselLink,
) -> Option<(SimVesselId, SimVesselId)> {
    let (a, b) = match *link {
        InterVesselLink::Docking { chaser, target, .. } => (chaser, target),
        InterVesselLink::WinchTether {
            carrier, payload, ..
        } => (carrier, payload),
        InterVesselLink::RailSlide { .. }
        | InterVesselLink::HingeSwing { .. }
        | InterVesselLink::PlumeLink { .. } => return None,
    };
    (a != b && coord.vessels.contains_key(&a) && coord.vessels.contains_key(&b)).then_some((a, b))
}

struct CoupledComponent {
    vessels: Vec<SimVesselId>,
    link_indices: Vec<usize>,
}

/// Partitions vessels connected by active `Docking` / `WinchTether` links into
/// deterministic sorted components, leaving uncoupled vessels as singletons.
fn partition_components(coord: &MultiVesselCoordinator) -> Vec<CoupledComponent> {
    let mut adjacency: BTreeMap<SimVesselId, Vec<(SimVesselId, usize)>> = BTreeMap::new();
    for (idx, link) in coord.links.iter().enumerate() {
        let Some((a, b)) = two_vessel_endpoints(coord, link) else {
            continue;
        };
        adjacency.entry(a).or_default().push((b, idx));
        adjacency.entry(b).or_default().push((a, idx));
    }
    let mut visited = BTreeSet::new();
    let mut components = Vec::new();
    let nodes: Vec<SimVesselId> = adjacency.keys().copied().collect();
    for start in nodes {
        if !visited.insert(start) {
            continue;
        }
        components.push(collect_component(start, &adjacency, &mut visited));
    }
    components
}

fn collect_component(
    start: SimVesselId,
    adjacency: &BTreeMap<SimVesselId, Vec<(SimVesselId, usize)>>,
    visited: &mut BTreeSet<SimVesselId>,
) -> CoupledComponent {
    let mut queue = vec![start];
    let mut vessels = BTreeSet::new();
    let mut link_set = BTreeSet::new();
    while let Some(cur) = queue.pop() {
        vessels.insert(cur);
        if let Some(neighbors) = adjacency.get(&cur) {
            for &(next, link_idx) in neighbors {
                link_set.insert(link_idx);
                if visited.insert(next) {
                    queue.push(next);
                }
            }
        }
    }
    CoupledComponent {
        vessels: vessels.into_iter().collect(),
        link_indices: link_set.into_iter().collect(),
    }
}

/// Steps all vessels in `coord`: uncoupled singletons take their individual
/// `Block::step` path, while coupled components integrate together inside one
/// unified ODE.
pub(super) fn step_all_grouped(
    coord: &mut MultiVesselCoordinator,
    inputs: &BTreeMap<SimVesselId, MultibodyVehicleInput>,
) -> Result<MultiVesselStepReport, BlockError> {
    coord.pending_releases.clear();
    coord.pending_hard_mates.clear();
    let mut step_inputs = BTreeMap::new();
    for id in coord.vessel_ids() {
        step_inputs.insert(id, inputs.get(&id).cloned().unwrap_or_default());
    }
    for link in &coord.links {
        if matches!(link, InterVesselLink::PlumeLink { .. }) {
            eval_plume_link(&coord.vessels, link, &mut step_inputs)?;
        }
    }
    let single_owners = couple_single_vessel_mechanisms(coord, &mut step_inputs);
    let components = partition_components(coord);
    let mut coupled = BTreeSet::new();
    for comp in &components {
        coupled.extend(comp.vessels.iter().copied());
    }
    for id in coord.vessel_ids() {
        if coupled.contains(&id) {
            continue;
        }
        if let Some(vessel) = coord.vessels.get_mut(&id) {
            let input = step_inputs.remove(&id).unwrap_or_default();
            crate::Block::step(vessel, &input)?;
        }
    }
    for comp in components {
        step_proximity_group(coord, &comp.vessels, &comp.link_indices, &step_inputs)?;
    }
    take_link_states(coord, &single_owners);
    process_post_step_links(coord)
}
