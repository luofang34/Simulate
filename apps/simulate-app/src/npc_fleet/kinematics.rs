//! Deterministic per-frame pose and articulation evaluation for scripted and apron NPCs.

use std::sync::Arc;

use glam::{DQuat, DVec3, EulerRot};
use sim_core::{AirframeArticulation, AirframeLights};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_maplibre::{
    AltitudeReference, LatLon, Placement3d, UsdAircraftAsset, ViewSlotMask,
    orientation_from_ned_body_quat,
};

use super::script::{NpcModeSpec, NpcSpawnSpec, NpcWaypoint};
use super::{EvaluatedNpc, GRAVITY_MPS2, KNOTS_TO_MPS};
use crate::world_surface::WorldSurfaceHandle;

const PARKED_PROP_IDLE_RPM: f64 = 650.0;
const CRUISE_PROP_RPM: f64 = 1_950.0;

pub(super) fn evaluate_spawn(
    spawn: &NpcSpawnSpec,
    model_key: (String, String),
    asset: &Arc<UsdAircraftAsset>,
    t_s: f64,
    anchor_msl: Geodetic,
    surface: &WorldSurfaceHandle,
) -> Option<EvaluatedNpc> {
    let callsign = spawn
        .callsign
        .clone()
        .unwrap_or_else(|| default_callsign(&spawn.id));
    let (transponder, msl) = (spawn.transponder, anchor_msl.height_m);
    match &spawn.mode {
        NpcModeSpec::ParkedApron {
            offset_east_m,
            offset_north_m,
            heading_deg,
            props_idle,
        } => {
            let geo = apron_offset_geodetic(anchor_msl, *offset_east_m, *offset_north_m);
            Some(evaluate_parked(
                callsign,
                model_key,
                asset,
                t_s,
                (transponder, geo, *heading_deg, *props_idle, msl),
                surface,
            ))
        }
        NpcModeSpec::ParkedGeodetic {
            lat_deg,
            lon_deg,
            heading_deg,
            props_idle,
        } => {
            let geo = Geodetic::from_degrees(*lat_deg, *lon_deg, msl);
            Some(evaluate_parked(
                callsign,
                model_key,
                asset,
                t_s,
                (transponder, geo, *heading_deg, *props_idle, msl),
                surface,
            ))
        }
        NpcModeSpec::Orbit {
            center_lat_deg,
            center_lon_deg,
            altitude_msl_m,
            radius_m,
            speed_kt,
            clockwise,
            phase_deg,
        } => Some(evaluate_orbit(
            spawn,
            callsign,
            model_key,
            asset,
            t_s,
            (*center_lat_deg, *center_lon_deg, *altitude_msl_m),
            (*radius_m, *speed_kt, *clockwise, *phase_deg),
        )),
        NpcModeSpec::Waypoints {
            loop_route,
            speed_kt,
            points,
        } => evaluate_waypoints(
            spawn,
            callsign,
            model_key,
            asset,
            t_s,
            *loop_route,
            *speed_kt,
            points,
        ),
    }
}

fn apron_offset_geodetic(anchor: Geodetic, east_m: f64, north_m: f64) -> Geodetic {
    let anchor_ecef = geodetic_to_ecef(&anchor);
    let frame = LocalFrame::at(&anchor);
    let ecef = anchor_ecef + frame.east() * east_m + frame.north() * north_m;
    ecef_to_geodetic(ecef)
}

fn evaluate_parked(
    callsign: String,
    model_key: (String, String),
    asset: &Arc<UsdAircraftAsset>,
    t_s: f64,
    (transponder, geo, heading_deg, props_idle, fallback_msl_m): (bool, Geodetic, f64, bool, f64),
    surface: &WorldSurfaceHandle,
) -> EvaluatedNpc {
    let ground_msl = surface.orthometric_ground_m(&geo).unwrap_or(fallback_msl_m);
    let lift_m = -f64::from(asset.lowest_point_z_m()).min(0.0);
    let ned_quat = DQuat::from_euler(EulerRot::ZYX, heading_deg.to_radians(), 0.0, 0.0).normalize();
    let mut articulation = flying_or_idle_articulation(props_idle, props_idle, t_s, 1.0);
    articulation.gear_wow = [true; 3];
    EvaluatedNpc {
        callsign,
        transponder,
        model_key,
        asset: Arc::clone(asset),
        placement: Placement3d {
            position: LatLon::new(geo.lat_deg(), geo.lon_deg()),
            altitude_meters: ground_msl + lift_m,
            altitude_reference: AltitudeReference::AboveMsl,
            orientation: orientation_from_ned_body_quat(ned_quat),
            view_mask: ViewSlotMask::ALL,
        },
        ground_height_msl_m: Some(ground_msl),
        articulation,
        velocity_ned_mps: [0.0; 3],
        heading_rad: heading_deg.to_radians().rem_euclid(std::f64::consts::TAU),
    }
}

fn evaluate_orbit(
    spawn: &NpcSpawnSpec,
    callsign: String,
    model_key: (String, String),
    asset: &Arc<UsdAircraftAsset>,
    t_s: f64,
    (center_lat_deg, center_lon_deg, altitude_msl_m): (f64, f64, f64),
    (radius_m, speed_kt, clockwise, phase_deg): (f64, f64, bool, f64),
) -> EvaluatedNpc {
    let radius = radius_m.max(50.0);
    let speed_mps = (speed_kt * KNOTS_TO_MPS).max(10.0);
    let dir = if clockwise { 1.0 } else { -1.0 };
    let omega = dir * speed_mps / radius;
    let theta = phase_deg.to_radians() + omega * t_s;
    let center = Geodetic::from_degrees(center_lat_deg, center_lon_deg, altitude_msl_m);
    let frame = LocalFrame::at(&center);
    let pos_ecef = geodetic_to_ecef(&center)
        + frame.north() * (radius * theta.cos())
        + frame.east() * (radius * theta.sin());
    let geo = ecef_to_geodetic(pos_ecef);
    let vn = -radius * omega * theta.sin();
    let ve = radius * omega * theta.cos();
    let heading_rad = ve.atan2(vn);
    let bank_rad = (dir * speed_mps * speed_mps / (GRAVITY_MPS2 * radius))
        .atan()
        .clamp(-0.65, 0.65);
    let ned_quat = DQuat::from_euler(EulerRot::ZYX, heading_rad, 0.0, bank_rad).normalize();
    EvaluatedNpc {
        callsign,
        transponder: spawn.transponder,
        model_key,
        asset: Arc::clone(asset),
        placement: Placement3d {
            position: LatLon::new(geo.lat_deg(), geo.lon_deg()),
            altitude_meters: altitude_msl_m,
            altitude_reference: AltitudeReference::AboveMsl,
            orientation: orientation_from_ned_body_quat(ned_quat),
            view_mask: ViewSlotMask::ALL,
        },
        ground_height_msl_m: None,
        articulation: flying_or_idle_articulation(true, false, t_s, 0.0),
        velocity_ned_mps: [vn, ve, 0.0],
        heading_rad: heading_rad.rem_euclid(std::f64::consts::TAU),
    }
}

#[allow(clippy::too_many_arguments)]
fn evaluate_waypoints(
    spawn: &NpcSpawnSpec,
    callsign: String,
    model_key: (String, String),
    asset: &Arc<UsdAircraftAsset>,
    t_s: f64,
    loop_route: bool,
    speed_kt: f64,
    points: &[NpcWaypoint],
) -> Option<EvaluatedNpc> {
    let first = points.first()?;
    if points.len() < 2 {
        return Some(single_waypoint_npc(
            spawn, callsign, model_key, asset, t_s, first,
        ));
    }
    let speed_mps = (speed_kt * KNOTS_TO_MPS).max(10.0);
    let ecefs: Vec<DVec3> = points
        .iter()
        .map(|pt| {
            geodetic_to_ecef(&Geodetic::from_degrees(
                pt.lat_deg,
                pt.lon_deg,
                pt.altitude_msl_m,
            ))
        })
        .collect();
    let leg_count = if loop_route {
        ecefs.len()
    } else {
        ecefs.len().saturating_sub(1)
    };
    let mut legs = Vec::with_capacity(leg_count);
    let mut total_len = 0.0;
    for i in 0..leg_count {
        let a = ecefs.get(i)?;
        let b = ecefs.get((i.wrapping_add(1)) % ecefs.len())?;
        let d = (*b - *a).length();
        total_len += d;
        legs.push((*a, *b, d));
    }
    let dist = if loop_route && total_len > 1.0 {
        (t_s * speed_mps).rem_euclid(total_len)
    } else {
        (t_s * speed_mps).clamp(0.0, total_len)
    };
    let (pos_ecef, vel_ecef) = sample_polyline(&legs, dist, speed_mps)?;
    let geo = ecef_to_geodetic(pos_ecef);
    let ned = LocalFrame::at(&geo).ecef_to_ned_vector(vel_ecef);
    let h_speed = (ned.x * ned.x + ned.y * ned.y).sqrt();
    let heading = if h_speed > 0.1 {
        ned.y.atan2(ned.x)
    } else {
        0.0
    };
    let pitch = (-ned.z).atan2(h_speed.max(0.1)).clamp(-0.5, 0.5);
    let quat = DQuat::from_euler(EulerRot::ZYX, heading, pitch, 0.0).normalize();
    Some(EvaluatedNpc {
        callsign,
        transponder: spawn.transponder,
        model_key,
        asset: Arc::clone(asset),
        placement: Placement3d {
            position: LatLon::new(geo.lat_deg(), geo.lon_deg()),
            altitude_meters: geo.height_m,
            altitude_reference: AltitudeReference::AboveMsl,
            orientation: orientation_from_ned_body_quat(quat),
            view_mask: ViewSlotMask::ALL,
        },
        ground_height_msl_m: None,
        articulation: flying_or_idle_articulation(true, false, t_s, 0.0),
        velocity_ned_mps: [ned.x, ned.y, ned.z],
        heading_rad: heading.rem_euclid(std::f64::consts::TAU),
    })
}

fn single_waypoint_npc(
    spawn: &NpcSpawnSpec,
    callsign: String,
    model_key: (String, String),
    asset: &Arc<UsdAircraftAsset>,
    t_s: f64,
    first: &NpcWaypoint,
) -> EvaluatedNpc {
    EvaluatedNpc {
        callsign,
        transponder: spawn.transponder,
        model_key,
        asset: Arc::clone(asset),
        placement: Placement3d {
            position: LatLon::new(first.lat_deg, first.lon_deg),
            altitude_meters: first.altitude_msl_m,
            altitude_reference: AltitudeReference::AboveMsl,
            orientation: orientation_from_ned_body_quat(DQuat::IDENTITY),
            view_mask: ViewSlotMask::ALL,
        },
        ground_height_msl_m: None,
        articulation: flying_or_idle_articulation(true, false, t_s, 0.0),
        velocity_ned_mps: [0.0; 3],
        heading_rad: 0.0,
    }
}

fn sample_polyline(
    legs: &[(DVec3, DVec3, f64)],
    dist: f64,
    speed_mps: f64,
) -> Option<(DVec3, DVec3)> {
    let mut rem = dist;
    for &(a, b, len) in legs {
        if len > 1e-3 && rem <= len {
            let frac = rem / len;
            let pos = a + (b - a) * frac;
            let vel = (b - a) / len * speed_mps;
            return Some((pos, vel));
        }
        rem -= len;
    }
    let (a, b, len) = legs.last().copied()?;
    let vel = if len > 1e-3 {
        (b - a) / len * speed_mps
    } else {
        DVec3::ZERO
    };
    Some((b, vel))
}

fn flying_or_idle_articulation(
    running: bool,
    idle: bool,
    t_s: f64,
    gear_pos_norm: f64,
) -> AirframeArticulation {
    let rpm = if !running {
        0.0
    } else if idle {
        PARKED_PROP_IDLE_RPM
    } else {
        CRUISE_PROP_RPM
    };
    let phase = if running {
        (t_s * rpm / 60.0 * std::f64::consts::TAU).rem_euclid(std::f64::consts::TAU)
    } else {
        0.0
    };
    let mut out = AirframeArticulation {
        engine_running: running,
        engine_rpm: rpm,
        propeller_rpm: rpm,
        propeller_phase_rad: phase,
        gear_pos_norm,
        lights: AirframeLights {
            beacon_flash_norm: if running && (t_s % 1.2) < 0.12 {
                1.0
            } else {
                0.0
            },
            ..AirframeLights::default()
        },
        ..AirframeArticulation::default()
    };
    out.propeller_right.rpm = rpm;
    out.propeller_right.sense = -1.0;
    out.propeller_right.phase_rad = (-phase).rem_euclid(std::f64::consts::TAU);
    out
}

fn default_callsign(id: &str) -> String {
    let clean: String = id
        .chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .take(8)
        .collect::<String>()
        .to_ascii_uppercase();
    if clean.is_empty() {
        "NPC0001".to_owned()
    } else {
        clean
    }
}
