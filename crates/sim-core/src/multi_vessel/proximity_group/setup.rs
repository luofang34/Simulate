//! Pre-step setup, unified ODE integration, and post-step readout for
//! coupled proximity groups.

use std::collections::BTreeMap;

use mbd_core::WrenchAtSite;
use mbd_multibody::{
    DockingState, DormandPrince54, Integrator, ModelState, PinnedSurfaceRest, Rk4, Rosenbrock23,
    WinchTetherState, Workspace,
};
use sim_geo::BodyEllipsoid;

use super::commit::{build_group_links, commit_group_results, extract_link_outcomes};
use super::ode::{
    EndpointPose, GroupLinkSlot, GroupVesselSlot, PinnedVesselSlot, ProximityGroupOde, VesselTarget,
};
use super::probe::{capture_backup, initial_link_wrenches, restore_backup, validate_component};
use crate::flight_model::integration::{StepSetup, step_error, tightest_tolerances};
use crate::flight_model::loads::LoadReadout;
use crate::flight_model::planet_frame::{
    fixed_to_instantaneous_inertial, inertial_to_fixed_after_step,
};
use crate::multi_vessel::{COORD_BLOCK, MultiVesselCoordinator, SimVesselId, missing_vessel};
use crate::multibody_vehicle::{
    EnvSample, MultibodyVehicleBlock, MultibodyVehicleInput, SubsystemStepOut,
};
use crate::{BlockError, VehicleIntegrator};

enum GroupIntegrator {
    FixedRk4(Rk4),
    Adaptive(DormandPrince54),
    Stiff(Rosenbrock23),
}

pub(super) struct PreparedFree {
    pub(super) id: SimVesselId,
    pub(super) block: MultibodyVehicleBlock,
    input: MultibodyVehicleInput,
    pub(super) env_sample: EnvSample,
    pub(super) sub_out: SubsystemStepOut,
    pub(super) state: ModelState,
    x_init: Vec<f64>,
    modes: Vec<f64>,
}

pub(super) enum LinkOutcome {
    Docking {
        index: usize,
        state: DockingState,
    },
    Tether {
        index: usize,
        state: WinchTetherState,
    },
}

struct GroupIntegrationOut {
    readouts: Vec<LoadReadout>,
    outcomes: Vec<LinkOutcome>,
}

pub(super) fn step_proximity_group(
    coord: &mut MultiVesselCoordinator,
    vessels: &[SimVesselId],
    link_indices: &[usize],
    step_inputs: &BTreeMap<SimVesselId, MultibodyVehicleInput>,
) -> Result<(), BlockError> {
    let (period, body) = validate_component(coord, vessels)?;
    let dt_s = period.as_seconds();
    let backup = capture_backup(coord, vessels, link_indices);
    let mut free = Vec::new();
    let res = run_proximity_group(
        coord,
        vessels,
        link_indices,
        step_inputs,
        body,
        dt_s,
        &mut free,
    );
    if res.is_err() {
        let taken = free.into_iter().map(|f| (f.id, f.block)).collect();
        restore_backup(coord, taken, backup);
    }
    res
}

fn run_proximity_group(
    coord: &mut MultiVesselCoordinator,
    vessels: &[SimVesselId],
    link_indices: &[usize],
    step_inputs: &BTreeMap<SimVesselId, MultibodyVehicleInput>,
    body: BodyEllipsoid,
    dt_s: f64,
    free: &mut Vec<PreparedFree>,
) -> Result<(), BlockError> {
    let (pinned, targets) =
        prepare_free_and_pinned(coord, vessels, link_indices, step_inputs, body, dt_s, free)?;
    let mut x = Vec::new();
    let mut vessel_ranges = Vec::with_capacity(free.len());
    for item in free.iter() {
        let start = x.len();
        x.extend_from_slice(&item.x_init);
        vessel_ranges.push(start..x.len());
    }
    let link_slots = build_group_links(coord, link_indices, &targets, free, &mut x)?;
    let mut integrator = pick_group_integrator(free);

    let out = integrate_and_read(
        free,
        &vessel_ranges,
        pinned,
        link_slots,
        &mut integrator,
        &mut x,
        body,
        dt_s,
    )?;
    commit_group_results(coord, free, &out.readouts, &out.outcomes)?;
    Ok(())
}

fn prepare_free_and_pinned(
    coord: &mut MultiVesselCoordinator,
    vessels: &[SimVesselId],
    link_indices: &[usize],
    step_inputs: &BTreeMap<SimVesselId, MultibodyVehicleInput>,
    body: BodyEllipsoid,
    dt_s: f64,
    free: &mut Vec<PreparedFree>,
) -> Result<(Vec<PinnedVesselSlot>, BTreeMap<SimVesselId, VesselTarget>), BlockError> {
    let any_pinned = vessels.iter().any(|id| {
        coord
            .vessels
            .get(id)
            .is_some_and(|v| matches!(v.pinned_rest(), PinnedSurfaceRest::Pinned { .. }))
    });
    let init_wrenches = if any_pinned {
        initial_link_wrenches(coord, link_indices)?
    } else {
        BTreeMap::new()
    };
    let mut pinned = Vec::new();
    let mut targets = BTreeMap::new();
    for &id in vessels {
        let input = step_inputs.get(&id).cloned().unwrap_or_default();
        let extra = init_wrenches.get(&id).map_or(&[][..], Vec::as_slice);
        let stayed_pinned = {
            let v = coord
                .vessels
                .get_mut(&id)
                .ok_or_else(|| missing_vessel(id))?;
            v.probe_pinned_for_group(&input, extra)?
        };
        if stayed_pinned {
            let v = coord.vessels.get(&id).ok_or_else(|| missing_vessel(id))?;
            let p_idx = pinned.len();
            pinned.push(PinnedVesselSlot {
                poses: v.pose_table().clone(),
                fixed_base: v.floating_base_fixed(),
                body: v.body(),
            });
            targets.insert(id, VesselTarget::Pinned(p_idx));
            continue;
        }
        let mut block = coord
            .vessels
            .remove(&id)
            .ok_or_else(|| missing_vessel(id))?;
        let (env_sample, sub_out) = block.step_subsystems(dt_s, &input)?;
        block.take_up_masses(&sub_out)?;
        block.hold_loads(&sub_out, &input);
        let mut state = block.step_state(&input)?;
        state.set_base(&fixed_to_instantaneous_inertial(&state.base(), &body));
        let mut x_init = state.as_slice().to_vec();
        let mut work = Workspace::new();
        block
            .model()
            .root_to_centre_of_mass(&mut x_init, &mut work)
            .map_err(|s| step_error(COORD_BLOCK, s))?;
        let modes = state.modes().to_vec();
        let f_idx = free.len();
        free.push(PreparedFree {
            id,
            block,
            input,
            env_sample,
            sub_out,
            state,
            x_init,
            modes,
        });
        targets.insert(id, VesselTarget::Free(f_idx));
    }
    Ok((pinned, targets))
}

fn pick_group_integrator(free: &[PreparedFree]) -> GroupIntegrator {
    let mut stiff: Option<mbd_multibody::Tolerances> = None;
    let mut adaptive: Option<mbd_multibody::Tolerances> = None;
    let mut rk4_substeps = 1_u32;
    for item in free {
        match item.block.config().integrator {
            VehicleIntegrator::Stiff(tol) => {
                stiff = Some(stiff.map_or(tol, |cur| tightest_tolerances(cur, tol)));
            }
            VehicleIntegrator::Adaptive(tol) => {
                adaptive = Some(adaptive.map_or(tol, |cur| tightest_tolerances(cur, tol)));
            }
            VehicleIntegrator::FixedRk4 { substeps } => {
                rk4_substeps = rk4_substeps.max(substeps.max(1));
            }
        }
    }
    if let Some(tol) = stiff {
        GroupIntegrator::Stiff(Rosenbrock23::new(tol))
    } else if let Some(tol) = adaptive {
        GroupIntegrator::Adaptive(DormandPrince54::new(tol))
    } else {
        GroupIntegrator::FixedRk4(Rk4::with_substeps(rk4_substeps))
    }
}

#[allow(clippy::too_many_arguments)]
fn integrate_and_read(
    free: &mut [PreparedFree],
    vessel_ranges: &[core::ops::Range<usize>],
    pinned: Vec<PinnedVesselSlot>,
    link_slots: Vec<GroupLinkSlot>,
    integrator: &mut GroupIntegrator,
    x: &mut [f64],
    body: BodyEllipsoid,
    dt_s: f64,
) -> Result<GroupIntegrationOut, BlockError> {
    let mut vessel_slots = Vec::with_capacity(free.len());
    for (item, range) in free.iter().zip(vessel_ranges) {
        vessel_slots.push(GroupVesselSlot {
            model: item.block.model(),
            elements: item.block.elements(),
            env: item.block.world_environment(&item.input),
            modes: item.modes.clone(),
            work: Workspace::new(),
            stage_state: item.state.clone(),
            link_wrenches: Vec::new(),
            range: range.clone(),
        });
    }
    let link_len = link_slots.len();
    let mut ode = ProximityGroupOde {
        vessels: vessel_slots,
        pinned,
        links: link_slots,
        total_dim: x.len(),
        cache_a: vec![EndpointPose::ZERO; link_len],
        cache_b: vec![EndpointPose::ZERO; link_len],
    };
    let step_res = match integrator {
        GroupIntegrator::FixedRk4(rk4) => rk4.advance(&mut ode, 0.0, dt_s, x),
        GroupIntegrator::Adaptive(dp54) => dp54.advance(&mut ode, 0.0, dt_s, x),
        GroupIntegrator::Stiff(ros23) => ros23.advance(&mut ode, 0.0, dt_s, x),
    };
    step_res.map_err(|s| step_error(COORD_BLOCK, s))?;
    ode.eval_endpoint_poses(dt_s, x)
        .map_err(|s| step_error(COORD_BLOCK, s))?;
    ode.apply_link_evaluations(x, None)
        .map_err(|s| step_error(COORD_BLOCK, s))?;

    let mut link_wrenches = Vec::with_capacity(ode.vessels.len());
    let mut end_states = Vec::with_capacity(ode.vessels.len());
    for slot in &mut ode.vessels {
        link_wrenches.push(core::mem::take(&mut slot.link_wrenches));
        let mut st = slot.stage_state.clone();
        for (dst, &src) in st.modes_mut().iter_mut().zip(&slot.modes) {
            *dst = src;
        }
        end_states.push(st);
    }
    let outcomes = extract_link_outcomes(&ode.links, x)?;
    drop(ode);

    let mut readouts = Vec::with_capacity(free.len());
    for ((item, end_state), wrenches) in free.iter_mut().zip(end_states).zip(link_wrenches) {
        item.state = end_state;
        let readout =
            read_vessel_end_loads(&mut item.block, &item.input, &item.state, &wrenches, dt_s)?;
        let ended = inertial_to_fixed_after_step(&item.state.base(), &body, dt_s);
        item.state.set_base(&ended);
        readouts.push(readout);
    }
    Ok(GroupIntegrationOut { readouts, outcomes })
}

fn read_vessel_end_loads(
    block: &mut MultibodyVehicleBlock,
    input: &MultibodyVehicleInput,
    state: &ModelState,
    link_wrenches: &[WrenchAtSite],
    dt_s: f64,
) -> Result<LoadReadout, BlockError> {
    let base_len = block.elements().wrenches.len();
    block
        .elements_mut()
        .wrenches
        .extend_from_slice(link_wrenches);
    let readout = {
        let env = block.world_environment(input);
        let setup = StepSetup {
            model: block.model(),
            elements: block.elements(),
            environment: &env,
            dt_s,
        };
        let mut work = Workspace::new();
        let mut rates = vec![0.0; state.as_slice().len()];
        setup.loads_at(block.mount(), state, dt_s, &mut work, &mut rates)
    };
    block.elements_mut().wrenches.truncate(base_len);
    readout.map_err(|s| step_error(COORD_BLOCK, s))
}
