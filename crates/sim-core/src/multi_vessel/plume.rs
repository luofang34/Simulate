//! Where a site of a vessel is in the world, and the plume of one vessel's
//! thruster striking another.

use std::collections::BTreeMap;

use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, PoseTable};
use mbd_multibody::FloatingBaseState;
use mbd_systems::{PlumeImpingementSpec, PlumeSurfaceTarget, evaluate_plume_impingement};

use super::{COORD_BLOCK, InterVesselLink, SimVesselId, map_sys_coord};
use crate::BlockError;
use crate::multibody_vehicle::{MultibodyVehicleBlock, MultibodyVehicleInput};

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
    let vel_from_root =
        base.angular_vel_root_rad_s.cross(pose.position_root_m) + pose.linear_vel_root_m_s;
    let omega_root = base.angular_vel_root_rad_s + pose.angular_vel_root_rad_s;
    let omega_world = base.rotation_world_from_root * omega_root;
    let vel = base.linear_vel_world_m_s
        + base.rotation_world_from_root * vel_from_root
        + omega_world.cross(rot_link * offset_local_m);
    Ok((pos, rot_link, vel, omega_world))
}

pub(super) fn eval_plume_link(
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
        &src.floating_base_fixed(),
        *nozzle_site,
        *nozzle_offset_local_m,
    )?;
    let (target_center_m, rot_dst, _, _) = site_world_kinematics(
        dst.pose_table(),
        &dst.floating_base_fixed(),
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
