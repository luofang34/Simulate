use std::collections::BTreeMap;

use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, PoseTable, WrenchAtSite};
use mbd_multibody::{
    FloatingBaseState, HardMateResult, ImpulseReleaseResult, SpatialForce, step_docking_capture,
    step_hinge_release, step_rail_drive, step_tether,
};
use mbd_systems::{PlumeImpingementSpec, PlumeSurfaceTarget, evaluate_plume_impingement};

use super::{
    COORD_BLOCK, InterVesselLink, MultiVesselCoordinator, MultiVesselStepReport, SimVesselId,
    map_multi_coord, map_sys_coord, missing_vessel,
};
use crate::multibody_vehicle::{
    MultibodyVehicleBlock, MultibodyVehicleInput, fixed_to_instantaneous_inertial,
    inertial_to_fixed_after_step, merge_aero_remapped, merge_subsystems_remapped,
    remap_aero_subset, remap_subsystems_subset,
};
use crate::{Block, BlockError};

pub(super) fn site_world_kinematics(
    poses: &PoseTable,
    base: &FloatingBaseState,
    site: ComponentId,
    offset_local_m: DVec3,
) -> Result<(DVec3, DQuat, DVec3, DVec3), BlockError> {
    let pose = poses.get(site).map_err(|source| BlockError::Mbd {
        block: COORD_BLOCK,
        source,
    })?;
    let rot_link = (base.rotation_world_from_root * pose.rotation_root_from_link).normalize();
    let pos = base.position_world_m
        + base.rotation_world_from_root * pose.position_root_m
        + rot_link * offset_local_m;
    let vel_root = base.linear_vel_root_m_s
        + base.angular_vel_root_rad_s.cross(pose.position_root_m)
        + pose.linear_vel_root_m_s;
    let omega_root = base.angular_vel_root_rad_s + pose.angular_vel_root_rad_s;
    let omega_world = base.rotation_world_from_root * omega_root;
    let vel =
        base.rotation_world_from_root * vel_root + omega_world.cross(rot_link * offset_local_m);
    Ok((pos, rot_link, vel, omega_world))
}

fn world_force_to_local_wrench(
    poses: &PoseTable,
    base: &FloatingBaseState,
    site: ComponentId,
    offset_local_m: DVec3,
    spatial_world: SpatialForce,
) -> Result<WrenchAtSite, BlockError> {
    let pose = poses.get(site).map_err(|source| BlockError::Mbd {
        block: COORD_BLOCK,
        source,
    })?;
    let rot_inv = (base.rotation_world_from_root * pose.rotation_root_from_link)
        .normalize()
        .conjugate();
    Ok(WrenchAtSite {
        site,
        point_local_m: offset_local_m,
        force_local_n: rot_inv * spatial_world.force,
        torque_local_nm: rot_inv * spatial_world.torque,
    })
}

pub(super) fn prepare_step_inputs(
    coord: &mut MultiVesselCoordinator,
    inputs: &BTreeMap<SimVesselId, MultibodyVehicleInput>,
) -> Result<BTreeMap<SimVesselId, MultibodyVehicleInput>, BlockError> {
    let mut step_inputs = BTreeMap::new();
    for id in coord.vessel_ids() {
        step_inputs.insert(id, inputs.get(&id).cloned().unwrap_or_default());
    }
    coord.pending_releases.clear();
    coord.pending_hard_mates.clear();

    let link_count = coord.links.len();
    for idx in 0..link_count {
        eval_single_link(coord, idx, &mut step_inputs)?;
    }
    Ok(step_inputs)
}

fn eval_single_link(
    coord: &mut MultiVesselCoordinator,
    idx: usize,
    step_inputs: &mut BTreeMap<SimVesselId, MultibodyVehicleInput>,
) -> Result<(), BlockError> {
    let Some(link) = coord.links.get_mut(idx) else {
        return Ok(());
    };
    match link {
        InterVesselLink::RailSlide {
            vessel,
            spec,
            state,
            ..
        } => {
            if let Some(v) = coord.vessels.get(vessel) {
                let out = step_rail_drive(spec, state, v.tree()).map_err(map_multi_coord)?;
                if let Some(drive) = out.drive
                    && let Some(inp) = step_inputs.get_mut(vessel)
                {
                    inp.joint_drives.push(drive);
                }
                if out.ready_to_release && !state.released {
                    state.released = true;
                    coord
                        .pending_releases
                        .push((*vessel, spec.component, 0.0, DVec3::X));
                }
            }
        }
        InterVesselLink::HingeSwing {
            vessel,
            spec,
            state,
            ..
        } => {
            if let Some(v) = coord.vessels.get(vessel) {
                let (drive_opt, ready) =
                    step_hinge_release(spec, state, v.tree()).map_err(map_multi_coord)?;
                if let Some(drive) = drive_opt
                    && let Some(inp) = step_inputs.get_mut(vessel)
                {
                    inp.joint_drives.push(drive);
                }
                if ready && !state.released {
                    state.released = true;
                    coord
                        .pending_releases
                        .push((*vessel, spec.component, 0.0, DVec3::X));
                }
            }
        }
        InterVesselLink::Docking { .. } => eval_docking_link(
            &coord.vessels,
            link,
            &mut coord.pending_hard_mates,
            step_inputs,
        )?,
        InterVesselLink::WinchTether { .. } => eval_tether_link(&coord.vessels, link, step_inputs)?,
        InterVesselLink::PlumeLink { .. } => eval_plume_link(&coord.vessels, link, step_inputs)?,
    }
    Ok(())
}

fn eval_docking_link(
    vessels: &BTreeMap<SimVesselId, MultibodyVehicleBlock>,
    link: &mut InterVesselLink,
    pending_mates: &mut Vec<(
        SimVesselId,
        ComponentId,
        SimVesselId,
        ComponentId,
        mbd_core::Placement,
    )>,
    step_inputs: &mut BTreeMap<SimVesselId, MultibodyVehicleInput>,
) -> Result<(), BlockError> {
    let InterVesselLink::Docking {
        chaser,
        port_a,
        offset_a_local_m,
        axis_a_local,
        target,
        port_b,
        offset_b_local_m,
        mate_placement,
        spec,
        state,
        ..
    } = link
    else {
        return Ok(());
    };
    let (Some(va), Some(vb)) = (vessels.get(chaser), vessels.get(target)) else {
        return Ok(());
    };
    let dt_s = va.period().as_seconds();
    let ba_i = fixed_to_instantaneous_inertial(va.floating_base_fixed(), &va.body());
    let bb_i = fixed_to_instantaneous_inertial(vb.floating_base_fixed(), &vb.body());
    let (pa, ra, va_w, wa_w) =
        site_world_kinematics(va.pose_table(), &ba_i, *port_a, *offset_a_local_m)?;
    let (pb, rb, vb_w, wb_w) =
        site_world_kinematics(vb.pose_table(), &bb_i, *port_b, *offset_b_local_m)?;
    let out = step_docking_capture(
        spec,
        state,
        pa,
        ra,
        va_w,
        wa_w,
        pb,
        rb,
        vb_w,
        wb_w,
        *axis_a_local,
        dt_s,
    );
    if out.world_wrench_on_a != SpatialForce::ZERO
        && let Some(inp) = step_inputs.get_mut(chaser)
    {
        inp.external_wrenches.push(world_force_to_local_wrench(
            va.pose_table(),
            va.floating_base_fixed(),
            *port_a,
            *offset_a_local_m,
            out.world_wrench_on_a,
        )?);
    }
    if out.world_wrench_on_b != SpatialForce::ZERO
        && let Some(inp) = step_inputs.get_mut(target)
    {
        inp.external_wrenches.push(world_force_to_local_wrench(
            vb.pose_table(),
            vb.floating_base_fixed(),
            *port_b,
            *offset_b_local_m,
            out.world_wrench_on_b,
        )?);
    }
    if out.ready_for_hard_mate {
        pending_mates.push((*chaser, *port_a, *target, *port_b, *mate_placement));
    }
    Ok(())
}

fn eval_tether_link(
    vessels: &BTreeMap<SimVesselId, MultibodyVehicleBlock>,
    link: &mut InterVesselLink,
    step_inputs: &mut BTreeMap<SimVesselId, MultibodyVehicleInput>,
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
        umbilical_heat_rate_w,
        ..
    } = link
    else {
        return Ok(());
    };
    let (Some(va), Some(vb)) = (vessels.get(carrier), vessels.get(payload)) else {
        return Ok(());
    };
    let dt_s = va.period().as_seconds();
    let ba_i = fixed_to_instantaneous_inertial(va.floating_base_fixed(), &va.body());
    let bb_i = fixed_to_instantaneous_inertial(vb.floating_base_fixed(), &vb.body());
    let (pa, _, va_w, _) =
        site_world_kinematics(va.pose_table(), &ba_i, *anchor_carrier, *offset_carrier_m)?;
    let (pb, _, vb_w, _) =
        site_world_kinematics(vb.pose_table(), &bb_i, *anchor_payload, *offset_payload_m)?;
    let out = step_tether(
        spec,
        state,
        pa,
        va_w,
        pb,
        vb_w,
        *umbilical_heat_rate_w,
        dt_s,
    )
    .map_err(map_multi_coord)?;
    if out.force_on_a_world_n.length_squared() > 0.0
        && let Some(inp) = step_inputs.get_mut(carrier)
    {
        inp.external_wrenches.push(world_force_to_local_wrench(
            va.pose_table(),
            va.floating_base_fixed(),
            *anchor_carrier,
            *offset_carrier_m,
            SpatialForce::new(DVec3::ZERO, out.force_on_a_world_n),
        )?);
    }
    if out.force_on_b_world_n.length_squared() > 0.0
        && let Some(inp) = step_inputs.get_mut(payload)
    {
        inp.external_wrenches.push(world_force_to_local_wrench(
            vb.pose_table(),
            vb.floating_base_fixed(),
            *anchor_payload,
            *offset_payload_m,
            SpatialForce::new(DVec3::ZERO, out.force_on_b_world_n),
        )?);
    }
    Ok(())
}

fn eval_plume_link(
    vessels: &BTreeMap<SimVesselId, MultibodyVehicleBlock>,
    link: &InterVesselLink,
    step_inputs: &mut BTreeMap<SimVesselId, MultibodyVehicleInput>,
) -> Result<(), BlockError> {
    let InterVesselLink::PlumeLink {
        source_vessel,
        nozzle_site,
        nozzle_offset_local_m,
        exhaust_dir_local,
        nozzle_radius_m,
        cone_half_angle_rad,
        exhaust_thermal_w_per_n,
        target_vessel,
        target_site,
        target_point_local_m,
        target_normal_local,
        target_area_m2,
        thermal_absorptivity,
        ..
    } = link
    else {
        return Ok(());
    };
    let (Some(src), Some(dst)) = (vessels.get(source_vessel), vessels.get(target_vessel)) else {
        return Ok(());
    };
    let thrust_n = src.telemetry().total_thrust_body_n.length();
    if thrust_n <= 0.0 {
        return Ok(());
    }
    let (nozzle_origin_m, rot_src, _, _) = site_world_kinematics(
        src.pose_table(),
        src.floating_base_fixed(),
        *nozzle_site,
        *nozzle_offset_local_m,
    )?;
    let (target_center_m, rot_dst, _, _) = site_world_kinematics(
        dst.pose_table(),
        dst.floating_base_fixed(),
        *target_site,
        *target_point_local_m,
    )?;
    let plume = PlumeImpingementSpec {
        nozzle_origin_m,
        exhaust_dir: (rot_src * *exhaust_dir_local).normalize_or(DVec3::NEG_X),
        nozzle_radius_m: *nozzle_radius_m,
        cone_half_angle_rad: *cone_half_angle_rad,
        thrust_n,
        exhaust_thermal_power_w: thrust_n * *exhaust_thermal_w_per_n,
    };
    let target = PlumeSurfaceTarget {
        target_site: *target_site,
        surface_center_m: target_center_m,
        point_local_m: *target_point_local_m,
        rotation_root_from_target: rot_dst,
        outward_normal: (rot_dst * *target_normal_local).normalize_or(DVec3::Z),
        area_m2: *target_area_m2,
        thermal_absorptivity: *thermal_absorptivity,
    };
    let res = evaluate_plume_impingement(&plume, &[target], None).map_err(map_sys_coord)?;
    if let Some(inp) = step_inputs.get_mut(target_vessel) {
        inp.external_wrenches.extend(res.target_wrenches);
        inp.extra_thermal_loads.extend(res.target_thermal_loads);
    }
    Ok(())
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
        InterVesselLink::Docking { state, .. } => {
            !matches!(state.phase, mbd_multibody::DockingPhase::HardLatched)
        }
        InterVesselLink::WinchTether { state, .. } => state.intact,
        _ => true,
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
    let p_aero = remap_aero_subset(parent.aero_storage(), &release.parent_id_map);
    let c_aero = remap_aero_subset(parent.aero_storage(), &release.child_id_map);

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
        chaser.aero_storage(),
        &res.a_id_map,
        target.aero_storage(),
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
