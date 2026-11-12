//! Modernized Cockpit & Web Electronic Flight Bag (`EFB`) workspace state, modelled on
//! `Pilotage`'s chip-based mission planner (`MissionPlannerBar`, `MissionPlannerSheet`,
//! `RouteEntryEditor`, and `RouteChipCell`) plus two-point ad-hoc runway delineation and
//! coupled approach/bridge autoland.

mod actions;
pub mod adhoc_builder;
pub mod autoland;
pub mod navlog;
pub mod presets;
pub mod tokens;

#[cfg(test)]
mod tests;

pub use adhoc_builder::{AdHocBuilder, AdHocPickMode};
pub use autoland::{EfbAutopilot, EfbAutopilotPhase};
pub use navlog::{ActiveLegSummary, NavLogRow};
pub use presets::EfbPresetId;
pub use tokens::{
    ChipBadge, RouteToken, RouteTokenKind, autocomplete_candidates, parse_single_token,
};

use sim_maplibre::{FixId, FlightPlan, RouteProgress};
use sim_scenery::{AirportDatabase, AirportRecord, LandingRunwayGeometry};

use crate::esc_menu::catalog::HubCatalog;
use crate::world_surface::WorldSurfaceHandle;

/// Active dock tab inside the Expanded EFB workspace.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EfbTab {
    /// Interactive route chip editor, omni-search, Direct-To, and holds (`ROUTE`).
    #[default]
    Route,
    /// Leg-by-leg courses, distances, ETE, altitudes, and RNP scale (`NAVLOG`).
    NavLog,
    /// Two-point 3D/map ad-hoc runway delineation & bridge autoland (`AD-HOC RWY`).
    AdHocRwy,
    /// One-click curated routes including Mathias Rust 1987 `EFHF -> ZMRS` (`PRESETS`).
    Presets,
}

impl EfbTab {
    /// All tabs in dock header order.
    pub const ALL: [Self; 4] = [Self::Route, Self::NavLog, Self::AdHocRwy, Self::Presets];

    /// Short uppercase tab title.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Route => "ROUTE",
            Self::NavLog => "NAVLOG",
            Self::AdHocRwy => "AD-HOC RWY",
            Self::Presets => "PRESETS",
        }
    }
}

/// Copyable UI action dispatched from EFB chrome buttons.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EfbPlanAction {
    /// Toggles the left Mission Planner dock open or collapsed on the Expanded EFB.
    ToggleDock,
    /// Switches the left mission dock tab.
    SelectTab(EfbTab),
    /// Focuses or unfocuses the omni-search text input box.
    FocusOmni(bool),
    /// Commits the current omni-search text into route token(s).
    SubmitOmni,
    /// Picks the `idx`-th autocomplete suggestion row (`0..6`).
    PickSuggestion(usize),
    /// Selects the `idx`-th route chip.
    SelectChip(usize),
    /// Executes Direct-To (`D->`) from ownship's live position to the selected chip.
    DirectToSelected,
    /// Moves the selected chip earlier (`-1`) or later (`+1`) in the route.
    MoveSelected(i8),
    /// Toggles or cycles holding pattern (`Off -> Right -> Left -> Off`) on selected chip.
    CycleHoldSelected,
    /// Adjusts crossing altitude on the selected chip by `delta_hundreds * 100 ft`.
    AdjustAltSelected(i16),
    /// Cycles runway end designator on the selected airport chip (`18 -> 36`, `15 -> 33`).
    CycleRunwaySelected,
    /// Deletes the selected route chip.
    DeleteSelected,
    /// Clears all route chips (pushing an undo snapshot first).
    ClearRoute,
    /// Restores the previous route snapshot (`16`-deep undo ring).
    UndoRoute,
    /// Toggles whether clicking empty space on the EFB map inserts a `MapPin` waypoint.
    ToggleMapClickInsert,
    /// Centers the EFB map on the selected chip (or destination airport).
    FocusSelectedOnMap,
    /// Toggles the EFB LNAV/VNAV + Approach/Autoland autopilot.
    ToggleAutopilot,
    /// Exports the current route to a `#v=2` `SimulationLink` fragment.
    ExportLink,
    /// Loads a 1-click preset mission (`MathiasRust1987Corridor`, `RedSquareBridgeFinal`, ...).
    LoadPreset(EfbPresetId),
    /// Starts two-click runway delineation on the Expanded EFB map.
    AdHocPickEfbMap,
    /// Starts two-click runway delineation directly in the 3D viewport.
    AdHocPickWorld3d,
    /// Cycles ad-hoc strip width (`18 / 24 / 30 / 45 m`).
    AdHocCycleWidth,
    /// Cycles ad-hoc strip surface (`ASPH / TURF / GRVL`).
    AdHocCycleSurface,
    /// Swaps ad-hoc Threshold A and Threshold B.
    AdHocSwapThresholds,
    /// Commits the draft ad-hoc runway into the world and appends it to the route.
    AdHocCreateAndAppend,
    /// Spawns ownship lined up on the active ad-hoc runway threshold.
    AdHocSpawnThreshold,
    /// Spawns ownship on stabilized short final to the active ad-hoc runway with autoland.
    AdHocSpawnFinalAutoland,
}

/// Side-effect request returned by [`EfbWorkspace::apply_action`] for `FlightApp` to execute
/// on `Flight`, `SupervisorRig`, and `MapRig`.
#[derive(Debug, Clone, PartialEq)]
pub enum EfbHostEffect {
    /// No world/spawn side effect beyond refreshing the EFB route portrayal.
    RouteUpdated,
    /// Center EFB map at `[lat_deg, lon_deg]` with optional zoom.
    JumpMap {
        /// Target latitude, degrees.
        lat_deg: f64,
        /// Target longitude, degrees.
        lon_deg: f64,
        /// Optional zoom override.
        zoom: Option<f64>,
    },
    /// Minimize the Expanded EFB tablet temporarily so the pilot can click 3D terrain.
    EnterWorld3dPicker,
    /// Newly created ad-hoc aerodrome must be synced into `MapRig` and `SupervisorRig`.
    CreatedAdHoc {
        /// Newly delineated aerodrome code (`ZZ01`, ...).
        code: String,
        /// Focus point `[lat_deg, lon_deg]`.
        focus_deg: [f64; 2],
    },
    /// Spawn ownship at `code` (`threshold` if `station_m == 0.0`, or short final `< 0.0`),
    /// optionally engaging autoland immediately.
    SpawnAtRunway {
        /// Airport or ad-hoc code (`ZMRS`, `EFHF`, `ZZ01`).
        code: String,
        /// Runway designator (`15`, `18`, ...).
        designator: String,
        /// Longitudinal station relative to threshold (`0.0` threshold, `-1650.0` short final).
        station_m: f64,
        /// Engage `EfbAutopilot` right after spawning.
        engage_autoland: bool,
    },
}

/// Complete portable state of the Simulator's modernized EFB workspace.
#[derive(Debug, Clone)]
pub struct EfbWorkspace {
    /// Whether the left Mission Planner dock is expanded (`true`) or collapsed (`false`).
    pub dock_open: bool,
    /// Active left-dock tab.
    pub tab: EfbTab,
    /// Ordered sequence of route chips.
    pub tokens: Vec<RouteToken>,
    /// Selected chip index in `tokens`.
    pub selected_chip: Option<usize>,
    /// Active leg and hold progress.
    pub progress: RouteProgress,
    /// Omni-search input buffer.
    pub omni_query: String,
    /// Whether keyboard typing is captured by `omni_query`.
    pub omni_focused: bool,
    /// Whether clicking empty map space inserts a `MapPin` waypoint.
    pub map_click_insert: bool,
    /// Undo stack of prior `tokens` snapshots (bounded to `16`).
    pub(super) undo_stack: Vec<Vec<RouteToken>>,
    /// Monotonic [`FixId`] generator.
    pub(super) next_fix_id: u32,
    /// Interactive ad-hoc runway delineation state.
    pub ad_hoc: AdHocBuilder,
    /// Coupled LNAV/VNAV + approach/flare/rollout autopilot.
    pub autopilot: EfbAutopilot,
    /// Latest exported `#v=2` link fragment or status note.
    pub last_exported_link: Option<String>,
    /// Short status banner at the bottom of the EFB dock.
    pub status_banner: String,
}

impl Default for EfbWorkspace {
    fn default() -> Self {
        Self {
            dock_open: true,
            tab: EfbTab::Route,
            tokens: Vec::new(),
            selected_chip: None,
            progress: RouteProgress::default(),
            omni_query: String::new(),
            omni_focused: false,
            map_click_insert: true,
            undo_stack: Vec::new(),
            next_fix_id: 1,
            ad_hoc: AdHocBuilder::default(),
            autopilot: EfbAutopilot::default(),
            last_exported_link: None,
            status_banner: "EFB READY - OPEN PRESETS FOR MATHIAS RUST 1987 OR TYPE ICAO/FIX"
                .to_owned(),
        }
    }
}

impl EfbWorkspace {
    /// Allocates a fresh [`FixId`].
    pub fn alloc_fix_id(&mut self) -> FixId {
        let id = FixId(self.next_fix_id);
        self.next_fix_id = self.next_fix_id.wrapping_add(1);
        id
    }

    pub(super) fn push_undo(&mut self) {
        if self.undo_stack.len() >= 16 {
            let _oldest = self.undo_stack.remove(0);
        }
        self.undo_stack.push(self.tokens.clone());
    }

    /// Builds the `sim_maplibre::FlightPlan` and current `RouteProgress` for map portrayal.
    #[must_use]
    pub fn maplibre_plan(&self, cruise_kt: f64) -> (FlightPlan, RouteProgress) {
        navlog::build_maplibre_plan(&self.tokens, self.progress, self.selected_chip, cruise_kt)
    }

    /// Zero-based active leg index (`tokens[k] -> tokens[k + 1]`).
    #[must_use]
    pub fn active_leg_index(&self) -> usize {
        navlog::active_leg_of(self.progress, self.tokens.len())
    }

    /// Computes leg-by-leg `NavLogRow`s at `ground_speed_kt`.
    #[must_use]
    pub fn navlog_rows(&self, ground_speed_kt: f64) -> Vec<NavLogRow> {
        navlog::compute_navlog(&self.tokens, ground_speed_kt)
    }

    /// Computes live active-leg summary (`TO` fix, bearing, distance, XTK, ETE, RNP).
    #[must_use]
    pub fn active_summary(
        &self,
        own_deg: [f64; 2],
        ground_speed_kt: f64,
    ) -> Option<ActiveLegSummary> {
        navlog::active_leg_summary(&self.tokens, self.progress, own_deg, ground_speed_kt)
    }

    /// Compact 1-line summary (`EFHF->ZMRS/15 · ->KREML 142 12.4NM · AP APP`) for the
    /// corner minimap glance pill.
    #[must_use]
    pub fn glance_pill_text(&self, own_deg: [f64; 2], ground_speed_kt: f64) -> Option<String> {
        if self.tokens.is_empty() && !self.autopilot.is_engaged() {
            return None;
        }
        let ends = match (self.tokens.first(), self.tokens.last()) {
            (Some(a), Some(b)) if self.tokens.len() >= 2 => {
                format!("{}-{}", a.ident(), b.chip_label())
            }
            (Some(only), _) => only.chip_label(),
            _ => "FREE".to_owned(),
        };
        if let Some(sum) = self.active_summary(own_deg, ground_speed_kt) {
            Some(format!(
                "{ends}  TO {} {:03.0}T {:.1}NM {}  {}",
                sum.to_ident,
                sum.bearing_deg,
                sum.distance_nm,
                sum.phase_badge,
                self.autopilot.phase().badge()
            ))
        } else {
            Some(format!("{ends}  {}", self.autopilot.phase().badge()))
        }
    }

    /// Appends a character to `omni_query` when focused.
    pub fn push_omni_char(&mut self, ch: char) {
        if self.omni_focused && !ch.is_control() && self.omni_query.len() < 64 {
            self.omni_query.push(ch.to_ascii_uppercase());
        }
    }

    /// Deletes the last character from `omni_query`.
    pub fn backspace_omni(&mut self) -> bool {
        self.omni_focused && self.omni_query.pop().is_some()
    }

    /// Commits space-separated tokens (or a `#v=2` link fragment) from `omni_query` into `tokens`.
    pub fn submit_omni_tokens(&mut self, catalog: &HubCatalog) -> usize {
        let raw = self.omni_query.trim().to_owned();
        if raw.is_empty() {
            return 0;
        }
        if (raw.starts_with("#v=2") || raw.starts_with("v=2&"))
            && let Ok(imported) =
                navlog::import_simulation_link_fragment(&raw, &mut self.next_fix_id, catalog)
            && !imported.is_empty()
        {
            self.push_undo();
            let count = imported.len();
            self.tokens = imported;
            self.selected_chip = self.tokens.len().checked_sub(1);
            self.omni_query.clear();
            self.clamp_progress();
            self.status_banner = format!("IMPORTED {count} FIXES FROM #V=2 LINK");
            return count;
        }
        let mut parsed = Vec::new();
        for part in raw.split_whitespace() {
            let id = self.alloc_fix_id();
            if let Some(tok) = parse_single_token(part, id, catalog) {
                parsed.push(tok);
            }
        }
        if parsed.is_empty() {
            self.status_banner = format!("NO FIX OR AIRPORT MATCHING '{raw}'");
            return 0;
        }
        self.push_undo();
        let count = parsed.len();
        let mut insert_idx = self.selected_chip.map_or(self.tokens.len(), |sel| {
            sel.wrapping_add(1).min(self.tokens.len())
        });
        for tok in parsed {
            self.tokens.insert(insert_idx, tok);
            self.selected_chip = Some(insert_idx);
            insert_idx = insert_idx.wrapping_add(1);
        }
        self.omni_query.clear();
        self.clamp_progress();
        self.status_banner = format!("ADDED {count} ROUTE FIX(ES)");
        count
    }

    /// Handles a click on the Expanded EFB moving map: either captures an ad-hoc runway
    /// threshold, selects a clicked route fix, or inserts a `MapPin` waypoint.
    pub fn handle_efb_map_click(
        &mut self,
        picked_fix: Option<FixId>,
        geodetic_deg: Option<[f64; 2]>,
    ) -> bool {
        if self.ad_hoc.pick_mode.is_efb_map() {
            if let Some([lat, lon]) = geodetic_deg {
                let _done = self.ad_hoc.record_point(lat, lon);
                self.status_banner.clone_from(&self.ad_hoc.status);
                return true;
            }
            return false;
        }
        if let Some(fix_id) = picked_fix
            && let Some(pos) = self.tokens.iter().position(|t| t.id == fix_id)
        {
            self.selected_chip = Some(pos);
            self.tab = EfbTab::Route;
            return true;
        }
        if self.map_click_insert
            && matches!(self.tab, EfbTab::Route | EfbTab::NavLog)
            && let Some([lat_deg, lon_deg]) = geodetic_deg
        {
            self.push_undo();
            let id = self.alloc_fix_id();
            let tok = RouteToken {
                id,
                kind: RouteTokenKind::MapPin {
                    ident: format!("WP{:02}", id.0 % 100),
                },
                lat_deg,
                lon_deg,
                altitude_ft: Some(2_500),
                fly_over: false,
                hold: None,
            };
            let ident = tok.ident();
            let insert_at = self.selected_chip.map_or(self.tokens.len(), |s| {
                s.wrapping_add(1).min(self.tokens.len())
            });
            self.tokens.insert(insert_at, tok);
            self.selected_chip = Some(insert_at);
            self.clamp_progress();
            self.status_banner = format!("INSERTED {ident} ({lat_deg:.3}, {lon_deg:.3})");
            return true;
        }
        false
    }

    /// Resolves the destination [`AirportRecord`] and [`LandingRunwayGeometry`] when the
    /// final route chip is an airport with a selected runway end.
    #[must_use]
    pub fn destination_landing_runway(
        &self,
        catalog: &HubCatalog,
        surface: &WorldSurfaceHandle,
    ) -> Option<(AirportRecord, LandingRunwayGeometry)> {
        let last = self.tokens.last()?;
        let (code, Some(rwy)) = last.airport_and_runway()? else {
            return None;
        };
        let record = catalog
            .ad_hoc_by_code(code)
            .map(|a| a.airport.clone())
            .or_else(|| {
                AirportDatabase::builtin()
                    .by_code(code)
                    .map(|(_, r)| r.clone())
            })?;
        let geom = record.landing_runway(rwy, surface.vertical_datum())?;
        Some((record, geom))
    }

    /// Advances lateral leg sequencing at `own_deg`.
    pub fn update_sequencing(&mut self, own_deg: [f64; 2]) {
        self.progress = navlog::sequence_progress(&self.tokens, self.progress, own_deg);
    }

    pub(super) fn clamp_progress(&mut self) {
        if self.tokens.len() < 2 {
            self.progress = RouteProgress::default();
        } else {
            let max_to = self.tokens.len().saturating_sub(1);
            let cur_to = self.progress.to_fix.unwrap_or(1).clamp(1, max_to);
            self.progress.to_fix = Some(cur_to);
            if self.tokens.get(cur_to).and_then(|t| t.hold).is_none() {
                self.progress.hold = None;
            }
        }
    }
}
