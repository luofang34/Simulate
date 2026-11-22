//! Declarative JSON schema and parser for scripted ground and flying NPC fleets.

use serde_json::Value;

use crate::error::AppSimError;

/// Optional path to a JSON file listing scripted NPC aircraft spawns.
#[cfg(not(any(target_arch = "wasm32", test)))]
pub const ENV_NPC_SCRIPT: &str = "SIMULATE_NPC_SCRIPT";

/// Top-level JSON document specifying non-player aircraft spawns.
#[derive(Debug, Clone, Default)]
pub struct NpcFleetSpec {
    /// Ordered list of ground or flying NPC definitions.
    pub spawns: Vec<NpcSpawnSpec>,
}

/// Single scripted NPC aircraft entry.
#[derive(Debug, Clone)]
pub struct NpcSpawnSpec {
    /// Stable identifier used for cache keys, logs, and optional callsign fallback.
    pub id: String,
    /// Airframe identifier resolved via [`crate::livery_session::LiverySession`] or built-ins.
    pub aircraft_id: String,
    /// Optional livery identifier within the airframe's catalog.
    pub livery_id: Option<String>,
    /// Optional ATC/transponder callsign override.
    pub callsign: Option<String>,
    /// Whether this NPC transmits ADS-B traffic tracks to `SupervisorRig` when active.
    pub transponder: bool,
    /// Placement and kinematic behavior specification.
    pub mode: NpcModeSpec,
}

/// Movement or parked placement mode for a scripted NPC.
#[derive(Debug, Clone)]
pub enum NpcModeSpec {
    /// Parked on the active airport apron relative to the airport reference point.
    ParkedApron {
        /// East offset from the active aerodrome reference anchor, metres.
        offset_east_m: f64,
        /// North offset from the active aerodrome reference anchor, metres.
        offset_north_m: f64,
        /// True nose heading, degrees (`0` = north, `90` = east).
        heading_deg: f64,
        /// Whether engines/propellers idle while parked.
        props_idle: bool,
    },
    /// Parked at fixed WGS-84 latitude/longitude on the authoritative ground surface.
    ParkedGeodetic {
        /// Geodetic latitude, degrees.
        lat_deg: f64,
        /// Geodetic longitude, degrees.
        lon_deg: f64,
        /// True nose heading, degrees.
        heading_deg: f64,
        /// Whether engines/propellers idle while parked.
        props_idle: bool,
    },
    /// Continuous level circular orbit around a geodetic center.
    Orbit {
        /// Orbit center latitude, degrees.
        center_lat_deg: f64,
        /// Orbit center longitude, degrees.
        center_lon_deg: f64,
        /// Orthometric altitude above MSL, metres.
        altitude_msl_m: f64,
        /// Orbit turn radius, metres.
        radius_m: f64,
        /// True groundspeed, knots.
        speed_kt: f64,
        /// Turn sense (`true` for right/clockwise orbit from above).
        clockwise: bool,
        /// Initial angular phase along the circle at `SimInstant::START`, degrees.
        phase_deg: f64,
    },
    /// Deterministic great-circle/ENU piecewise-linear path across geodetic waypoints.
    Waypoints {
        /// Whether the route wraps back from the last waypoint to the first.
        loop_route: bool,
        /// Groundspeed along legs, knots.
        speed_kt: f64,
        /// Ordered route waypoints (`>= 2` required for motion).
        points: Vec<NpcWaypoint>,
    },
}

/// One 3D waypoint in a scripted route.
#[derive(Debug, Clone)]
pub struct NpcWaypoint {
    /// Geodetic latitude, degrees.
    pub lat_deg: f64,
    /// Geodetic longitude, degrees.
    pub lon_deg: f64,
    /// Orthometric altitude above MSL, metres.
    pub altitude_msl_m: f64,
}

/// Parses an [`NpcFleetSpec`] from raw UTF-8 JSON bytes.
pub fn parse_npc_fleet_json(bytes: &[u8]) -> Result<NpcFleetSpec, AppSimError> {
    let root: Value = serde_json::from_slice(bytes).map_err(|err| AppSimError::NpcScript {
        detail: err.to_string(),
    })?;
    let schema_version = root
        .get("schema_version")
        .and_then(Value::as_u64)
        .map_or(1, |v| v as u32);
    if schema_version != 1 {
        return Err(AppSimError::NpcScript {
            detail: format!("unsupported NPC script schema_version {schema_version}"),
        });
    }
    let raw_spawns = root
        .get("spawns")
        .or_else(|| root.get("npcs"))
        .and_then(Value::as_array)
        .ok_or_else(|| AppSimError::NpcScript {
            detail: "missing 'spawns' array".to_owned(),
        })?;
    let mut spawns = Vec::with_capacity(raw_spawns.len());
    for (idx, item) in raw_spawns.iter().enumerate() {
        spawns.push(parse_spawn(item, idx)?);
    }
    Ok(NpcFleetSpec { spawns })
}

fn parse_spawn(val: &Value, idx: usize) -> Result<NpcSpawnSpec, AppSimError> {
    let id = opt_str(val, "id").unwrap_or_else(|| format!("npc-{idx}"));
    let aircraft_id = req_str(val, "aircraft_id", &id)?;
    let livery_id = opt_str(val, "livery_id");
    let callsign = opt_str(val, "callsign");
    let transponder = val
        .get("transponder")
        .and_then(Value::as_bool)
        .unwrap_or(false);
    let mode_val = val.get("mode").ok_or_else(|| AppSimError::NpcScript {
        detail: format!("spawn {id:?} missing 'mode' object"),
    })?;
    let mode = parse_mode(mode_val, &id)?;
    Ok(NpcSpawnSpec {
        id,
        aircraft_id,
        livery_id,
        callsign,
        transponder,
        mode,
    })
}

fn parse_mode(val: &Value, spawn_id: &str) -> Result<NpcModeSpec, AppSimError> {
    let kind = val
        .get("kind")
        .or_else(|| val.get("type"))
        .and_then(Value::as_str)
        .ok_or_else(|| AppSimError::NpcScript {
            detail: format!("spawn {spawn_id:?} missing mode 'kind'"),
        })?;
    match kind {
        "parked_apron" => Ok(NpcModeSpec::ParkedApron {
            offset_east_m: req_f64_any(val, &["offset_east_m", "east_m"], spawn_id)?,
            offset_north_m: req_f64_any(val, &["offset_north_m", "north_m"], spawn_id)?,
            heading_deg: opt_f64(val, "heading_deg").unwrap_or(0.0),
            props_idle: opt_bool_any(val, &["props_idle", "engines_running"]).unwrap_or(false),
        }),
        "parked_geodetic" => Ok(NpcModeSpec::ParkedGeodetic {
            lat_deg: req_f64_any(val, &["lat_deg"], spawn_id)?,
            lon_deg: req_f64_any(val, &["lon_deg"], spawn_id)?,
            heading_deg: opt_f64(val, "heading_deg").unwrap_or(0.0),
            props_idle: opt_bool_any(val, &["props_idle", "engines_running"]).unwrap_or(false),
        }),
        "orbit" => Ok(NpcModeSpec::Orbit {
            center_lat_deg: req_f64_any(val, &["center_lat_deg"], spawn_id)?,
            center_lon_deg: req_f64_any(val, &["center_lon_deg"], spawn_id)?,
            altitude_msl_m: req_f64_any(val, &["altitude_msl_m"], spawn_id)?,
            radius_m: req_f64_any(val, &["radius_m"], spawn_id)?,
            speed_kt: req_f64_any(val, &["speed_kt"], spawn_id)?,
            clockwise: opt_bool_any(val, &["clockwise"]).unwrap_or(false),
            phase_deg: opt_f64(val, "phase_deg").unwrap_or(0.0),
        }),
        "waypoints" => parse_waypoints(val, spawn_id),
        other => Err(AppSimError::NpcScript {
            detail: format!("unknown mode kind {other:?} in spawn {spawn_id:?}"),
        }),
    }
}

fn parse_waypoints(val: &Value, spawn_id: &str) -> Result<NpcModeSpec, AppSimError> {
    let raw_pts =
        val.get("points")
            .and_then(Value::as_array)
            .ok_or_else(|| AppSimError::NpcScript {
                detail: format!("spawn {spawn_id:?} waypoints missing 'points' array"),
            })?;
    let mut points = Vec::with_capacity(raw_pts.len());
    for pt in raw_pts {
        points.push(NpcWaypoint {
            lat_deg: req_f64_any(pt, &["lat_deg"], spawn_id)?,
            lon_deg: req_f64_any(pt, &["lon_deg"], spawn_id)?,
            altitude_msl_m: req_f64_any(pt, &["altitude_msl_m"], spawn_id)?,
        });
    }
    Ok(NpcModeSpec::Waypoints {
        loop_route: opt_bool_any(val, &["loop_route"]).unwrap_or(true),
        speed_kt: req_f64_any(val, &["speed_kt"], spawn_id)?,
        points,
    })
}

fn opt_str(val: &Value, key: &str) -> Option<String> {
    val.get(key)
        .and_then(Value::as_str)
        .filter(|s| !s.is_empty())
        .map(str::to_owned)
}

fn req_str(val: &Value, key: &str, spawn_id: &str) -> Result<String, AppSimError> {
    opt_str(val, key).ok_or_else(|| AppSimError::NpcScript {
        detail: format!("spawn {spawn_id:?} missing string field {key:?}"),
    })
}

fn opt_f64(val: &Value, key: &str) -> Option<f64> {
    val.get(key)
        .and_then(Value::as_f64)
        .filter(|v| v.is_finite())
}

fn req_f64_any(val: &Value, keys: &[&str], spawn_id: &str) -> Result<f64, AppSimError> {
    for &key in keys {
        if let Some(num) = opt_f64(val, key) {
            return Ok(num);
        }
    }
    Err(AppSimError::NpcScript {
        detail: format!("spawn {spawn_id:?} missing finite number for {keys:?}"),
    })
}

fn opt_bool_any(val: &Value, keys: &[&str]) -> Option<bool> {
    for &key in keys {
        if let Some(b) = val.get(key).and_then(Value::as_bool) {
            return Some(b);
        }
    }
    None
}
