//! Non-player aircraft fleet management: ground ramp spawns, scripted orbits/routes, and
//! deterministic per-frame evaluation for `TruthSceneMap` rendering.

use std::collections::HashMap;
use std::sync::Arc;

use sim_core::{AirframeArticulation, SimInstant};
use sim_geo::Geodetic;
use sim_maplibre::{BuiltinAirframe, Placement3d, UsdAircraftAsset};
#[cfg(not(any(target_arch = "wasm32", test)))]
use tracing::warn;

use crate::error::AppSimError;
use crate::livery_session::LiverySession;
use crate::world_surface::WorldSurfaceHandle;

mod kinematics;
pub mod script;

#[cfg(not(any(target_arch = "wasm32", test)))]
use script::ENV_NPC_SCRIPT;
pub use script::{NpcModeSpec, NpcSpawnSpec, parse_npc_fleet_json};

const KNOTS_TO_MPS: f64 = 1852.0 / 3600.0;
const GRAVITY_MPS2: f64 = 9.80665;
const APRON_COLUMNS: u32 = 6;
const APRON_START_EAST_M: f64 = 140.0;
const APRON_START_NORTH_M: f64 = -160.0;
const APRON_ROW_SPACING_M: f64 = 80.0;

/// Evaluated pose, articulation, and cached OpenUSD model of one active NPC for one frame.
#[derive(Debug, Clone)]
pub struct EvaluatedNpc {
    /// Transponder/radar callsign label.
    pub callsign: String,
    /// Whether this NPC radiates ADS-B tracks into `SupervisorRig`.
    pub transponder: bool,
    /// Cache key `(aircraft_id, resolved_livery_id)` for [`sim_maplibre::TrafficModelId`] lookup.
    pub model_key: (String, String),
    /// Resolved and AO-baked OpenUSD airframe stage.
    pub asset: Arc<UsdAircraftAsset>,
    /// 3D world placement (`AboveMsl` orthometric height matching `Ownship` datum).
    pub placement: Placement3d,
    /// Authoritative orthometric ground height under parked aircraft for contact shadows.
    pub ground_height_msl_m: Option<f64>,
    /// Control-surface, gear, light, and propeller articulation state.
    pub articulation: AirframeArticulation,
    /// Local NED velocity vector (`[north, east, down]` in `m/s`) for ADS-B reports.
    pub velocity_ned_mps: [f64; 3],
    /// True heading in radians `[0, 2π)` for surveillance transponder state.
    pub heading_rad: f64,
}

/// Active non-player aircraft collection synchronized across desktop, headless, and web builds.
#[derive(Debug, Clone, Default)]
pub struct NpcFleet {
    spawns: Vec<NpcSpawnSpec>,
    cached_assets: HashMap<(String, String), Arc<UsdAircraftAsset>>,
    next_apron_seq: u32,
}

impl NpcFleet {
    /// Initializes an [`NpcFleet`], loading `SIMULATE_NPC_SCRIPT` when set on native builds.
    #[must_use]
    pub fn from_environment(_liveries: &mut LiverySession) -> Self {
        #[cfg(not(any(target_arch = "wasm32", test)))]
        {
            if let Some(raw_path) = std::env::var_os(ENV_NPC_SCRIPT)
                && !raw_path.is_empty()
            {
                let mut fleet = Self::default();
                let path = std::path::PathBuf::from(raw_path);
                if let Err(error) = fleet.load_json_path(_liveries, &path) {
                    warn!(?path, %error, "ignoring invalid SIMULATE_NPC_SCRIPT");
                }
                return fleet;
            }
        }
        Self::default()
    }

    /// Number of active scripted or ramp-spawned NPCs.
    #[must_use]
    pub fn count(&self) -> usize {
        self.spawns.len()
    }

    /// Removes all NPCs and resets stand numbering (cached assets stay reusable).
    pub fn clear(&mut self) {
        self.spawns.clear();
        self.next_apron_seq = 0;
    }

    /// Spawns one aircraft on the next unoccupied apron stand south of the active runway.
    pub fn spawn_ground_apron(
        &mut self,
        liveries: &mut LiverySession,
        aircraft_id: &str,
        livery_id: Option<&str>,
    ) -> Result<bool, AppSimError> {
        let Some(resolved_livery) =
            self.resolve_and_cache_asset(liveries, aircraft_id, livery_id)?
        else {
            return Ok(false);
        };
        let spacing_m = liveries
            .external_slot_by_id(aircraft_id)
            .and_then(|slot| liveries.external_meta(slot))
            .map_or(72.0, |meta| (meta.wingspan_m + 18.0).max(72.0));
        let seq = self.next_apron_seq;
        self.next_apron_seq = self.next_apron_seq.wrapping_add(1);
        let col = f64::from(seq % APRON_COLUMNS);
        let row = f64::from(seq / APRON_COLUMNS);
        let spec = NpcSpawnSpec {
            id: format!("apron-{aircraft_id}-{seq}"),
            aircraft_id: aircraft_id.to_owned(),
            livery_id: (!resolved_livery.is_empty()).then_some(resolved_livery),
            callsign: None,
            transponder: false,
            mode: NpcModeSpec::ParkedApron {
                offset_east_m: APRON_START_EAST_M + col * spacing_m,
                offset_north_m: APRON_START_NORTH_M - row * APRON_ROW_SPACING_M,
                heading_deg: 0.0,
                props_idle: false,
            },
        };
        self.spawns.push(spec);
        Ok(true)
    }

    /// Replaces the fleet with the parsed JSON script and pre-resolves every referenced asset.
    pub fn load_json_bytes(
        &mut self,
        liveries: &mut LiverySession,
        bytes: &[u8],
    ) -> Result<usize, AppSimError> {
        let parsed = parse_npc_fleet_json(bytes)?;
        let mut next_spawns = Vec::with_capacity(parsed.spawns.len());
        for mut spawn in parsed.spawns {
            let resolved = self.resolve_and_cache_asset(
                liveries,
                &spawn.aircraft_id,
                spawn.livery_id.as_deref(),
            )?;
            if let Some(resolved_livery) = resolved {
                if spawn.livery_id.is_none() && !resolved_livery.is_empty() {
                    spawn.livery_id = Some(resolved_livery);
                }
                next_spawns.push(spawn);
            }
        }
        let count = next_spawns.len();
        self.spawns = next_spawns;
        self.next_apron_seq = 0;
        Ok(count)
    }

    /// Reads a declarative JSON script from `path` on desktop hosts and pre-resolves its models.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn load_json_path(
        &mut self,
        liveries: &mut LiverySession,
        path: &std::path::Path,
    ) -> Result<usize, AppSimError> {
        let bytes = std::fs::read(path).map_err(|err| AppSimError::NpcScript {
            detail: format!("failed to read {}: {err}", path.display()),
        })?;
        self.load_json_bytes(liveries, &bytes)
    }

    /// Evaluates every active NPC deterministically at simulation instant `now`.
    #[must_use]
    pub fn evaluate(
        &self,
        now: SimInstant,
        anchor_msl: Geodetic,
        surface: &WorldSurfaceHandle,
    ) -> Vec<EvaluatedNpc> {
        let t_s = now.duration_since(SimInstant::START).as_seconds().max(0.0);
        let mut out = Vec::with_capacity(self.spawns.len());
        for spawn in &self.spawns {
            let key = cache_key(&spawn.aircraft_id, spawn.livery_id.as_deref());
            let Some(asset) = self.cached_assets.get(&key) else {
                continue;
            };
            if let Some(evaluated) =
                kinematics::evaluate_spawn(spawn, key, asset, t_s, anchor_msl, surface)
            {
                out.push(evaluated);
            }
        }
        out
    }

    fn resolve_and_cache_asset(
        &mut self,
        liveries: &mut LiverySession,
        aircraft_id: &str,
        livery_id: Option<&str>,
    ) -> Result<Option<String>, AppSimError> {
        let resolved_livery = resolved_livery_name(liveries, aircraft_id, livery_id);
        let key = cache_key(aircraft_id, Some(&resolved_livery));
        if self.cached_assets.contains_key(&key) {
            return Ok(Some(resolved_livery));
        }
        let asset = if let Some(found) = liveries.usd_asset_for(aircraft_id, livery_id)? {
            found
        } else if let Some(builtin) = fallback_builtin_asset(aircraft_id)? {
            builtin
        } else {
            return Ok(None);
        };
        self.cached_assets.insert(key, asset);
        Ok(Some(resolved_livery))
    }
}

fn cache_key(aircraft_id: &str, livery_id: Option<&str>) -> (String, String) {
    (
        aircraft_id.trim().to_ascii_lowercase(),
        livery_id.unwrap_or("default").trim().to_ascii_lowercase(),
    )
}

fn resolved_livery_name(
    liveries: &LiverySession,
    aircraft_id: &str,
    explicit: Option<&str>,
) -> String {
    if let Some(id) = explicit {
        return id.to_owned();
    }
    if let Some((_, active)) = liveries.active_livery(aircraft_id) {
        return active.id.clone();
    }
    liveries
        .variant(aircraft_id)
        .and_then(|v| v.default_livery_id())
        .unwrap_or("default")
        .to_owned()
}

fn fallback_builtin_asset(aircraft_id: &str) -> Result<Option<Arc<UsdAircraftAsset>>, AppSimError> {
    let built = match aircraft_id.trim().to_ascii_lowercase().as_str() {
        "c172s" | "c172" | "skyhawk" => {
            return Ok(Some(UsdAircraftAsset::builtin(BuiltinAirframe::N642sp)?));
        }
        "c172p" => return Ok(Some(UsdAircraftAsset::builtin(BuiltinAirframe::DEcjb)?)),
        "raketa" => return Ok(Some(UsdAircraftAsset::builtin(BuiltinAirframe::Raketa)?)),
        "p38g" | "p38" => UsdAircraftAsset::p38g()?,
        "x500" => UsdAircraftAsset::x500_quadrotor()?,
        "lunar_lander" => UsdAircraftAsset::lunar_lander()?,
        "fixed_wing_uav" => UsdAircraftAsset::fixed_wing_uav()?,
        _ => return Ok(None),
    };
    Ok(Some(Arc::new(built)))
}
