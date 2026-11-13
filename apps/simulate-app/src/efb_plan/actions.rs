//! Reducer methods executing [`super::EfbPlanAction`] on [`super::EfbWorkspace`].

use sim_maplibre::RouteProgress;

use super::presets::EfbPresetId;
use super::tokens::{
    HoldSpec, HoldTurn, RouteToken, RouteTokenKind, autocomplete_candidates, parse_single_token,
};
use super::{EfbHostEffect, EfbPlanAction, EfbTab, EfbWorkspace, navlog};
use crate::esc_menu::catalog::HubCatalog;
use crate::world_surface::WorldSurfaceHandle;

impl EfbWorkspace {
    /// Applies an [`EfbPlanAction`] and returns any host side effect to execute.
    pub fn apply_action(
        &mut self,
        action: EfbPlanAction,
        own_deg: [f64; 2],
        catalog: &mut HubCatalog,
        surface: &WorldSurfaceHandle,
    ) -> EfbHostEffect {
        match action {
            EfbPlanAction::ToggleDock => {
                self.dock_open = !self.dock_open;
                self.omni_focused = false;
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::SelectTab(tab) => {
                self.tab = tab;
                self.omni_focused = false;
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::FocusOmni(focused) => {
                self.omni_focused = focused;
                if focused {
                    self.tab = EfbTab::Route;
                }
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::SubmitOmni => {
                let _added = self.submit_omni_tokens(catalog);
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::PickSuggestion(idx) => {
                let items = autocomplete_candidates(&self.omni_query, Some(own_deg), catalog, 6);
                if let Some(chosen) = items.get(idx) {
                    self.omni_query.clone_from(&chosen.insert_text);
                    let _added = self.submit_omni_tokens(catalog);
                }
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::SelectChip(idx) => {
                if idx < self.tokens.len() {
                    self.selected_chip = Some(idx);
                }
                EfbHostEffect::RouteUpdated
            }
            other => self.apply_edit_or_adhoc(other, own_deg, catalog, surface),
        }
    }

    fn apply_edit_or_adhoc(
        &mut self,
        action: EfbPlanAction,
        own_deg: [f64; 2],
        catalog: &mut HubCatalog,
        surface: &WorldSurfaceHandle,
    ) -> EfbHostEffect {
        match action {
            EfbPlanAction::DirectToSelected => self.exec_direct_to(own_deg),
            EfbPlanAction::MoveSelected(dir) => self.exec_move_selected(dir),
            EfbPlanAction::CycleHoldSelected => self.exec_cycle_hold(),
            EfbPlanAction::AdjustAltSelected(delta) => self.exec_adjust_alt(delta),
            EfbPlanAction::CycleRunwaySelected => self.exec_cycle_runway(catalog),
            EfbPlanAction::DeleteSelected => self.exec_delete_selected(),
            EfbPlanAction::ClearRoute => {
                if !self.tokens.is_empty() {
                    self.push_undo();
                    self.tokens.clear();
                    self.selected_chip = None;
                    self.progress = RouteProgress::default();
                    self.status_banner = "CLEARED ROUTE".to_owned();
                }
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::UndoRoute => {
                if let Some(prev) = self.undo_stack.pop() {
                    self.tokens = prev;
                    self.selected_chip = self.tokens.len().checked_sub(1);
                    self.clamp_progress();
                    self.status_banner = "RESTORED PREVIOUS ROUTE".to_owned();
                }
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::ToggleMapClickInsert => {
                self.map_click_insert = !self.map_click_insert;
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::FocusSelectedOnMap => {
                if let Some(tok) = self
                    .selected_chip
                    .and_then(|i| self.tokens.get(i))
                    .or_else(|| self.tokens.last())
                {
                    return EfbHostEffect::JumpMap {
                        lat_deg: tok.lat_deg,
                        lon_deg: tok.lon_deg,
                        zoom: Some(12.5),
                    };
                }
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::ToggleAutopilot => {
                self.autopilot.toggle();
                self.status_banner = format!("GUIDANCE: {}", self.autopilot.phase().badge());
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::ExportLink => {
                match navlog::export_simulation_link_fragment(&self.tokens, 750) {
                    Ok(frag) => {
                        self.status_banner = format!("EXPORTED #{frag}");
                        self.last_exported_link = Some(format!("#{frag}"));
                    }
                    Err(err) => self.status_banner = err.to_ascii_uppercase(),
                }
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::LoadPreset(preset) => self.exec_load_preset(preset, catalog),
            adhoc => self.apply_adhoc_action(adhoc, catalog, surface),
        }
    }

    fn exec_direct_to(&mut self, own_deg: [f64; 2]) -> EfbHostEffect {
        let Some(sel) = self.selected_chip.filter(|&i| i < self.tokens.len()) else {
            return EfbHostEffect::RouteUpdated;
        };
        self.push_undo();
        let ppos_id = self.alloc_fix_id();
        let ppos = RouteToken {
            id: ppos_id,
            kind: RouteTokenKind::MapPin {
                ident: "PPOS".to_owned(),
            },
            lat_deg: own_deg.first().copied().unwrap_or(0.0),
            lon_deg: own_deg.get(1).copied().unwrap_or(0.0),
            altitude_ft: None,
            fly_over: false,
            hold: None,
        };
        let tail: Vec<RouteToken> = self.tokens.get(sel..).unwrap_or(&[]).to_vec();
        let target_name = tail
            .first()
            .map_or_else(String::new, RouteToken::chip_label);
        self.tokens.clear();
        self.tokens.push(ppos);
        self.tokens.extend(tail);
        self.selected_chip = Some(1);
        self.progress = RouteProgress {
            to_fix: (self.tokens.len() >= 2).then_some(1),
            complete: false,
            hold: None,
        };
        self.status_banner = format!("DIRECT-TO {target_name} ACTIVATED");
        EfbHostEffect::RouteUpdated
    }

    fn exec_move_selected(&mut self, dir: i8) -> EfbHostEffect {
        let Some(sel) = self.selected_chip else {
            return EfbHostEffect::RouteUpdated;
        };
        let target = if dir < 0 {
            sel.checked_sub(1)
        } else {
            let next = sel.wrapping_add(1);
            (next < self.tokens.len()).then_some(next)
        };
        if let Some(dst) = target {
            self.push_undo();
            self.tokens.swap(sel, dst);
            self.selected_chip = Some(dst);
        }
        EfbHostEffect::RouteUpdated
    }

    fn exec_cycle_hold(&mut self) -> EfbHostEffect {
        let Some(tok) = self.selected_chip.and_then(|i| self.tokens.get_mut(i)) else {
            return EfbHostEffect::RouteUpdated;
        };
        let next_hold = match tok.hold.map(|h| h.turn) {
            None => Some(HoldSpec {
                turn: HoldTurn::Right,
                ..HoldSpec::default()
            }),
            Some(HoldTurn::Right) => Some(HoldSpec {
                turn: HoldTurn::Left,
                ..HoldSpec::default()
            }),
            Some(HoldTurn::Left) => None,
        };
        tok.hold = next_hold;
        EfbHostEffect::RouteUpdated
    }

    fn exec_adjust_alt(&mut self, delta_hundreds: i16) -> EfbHostEffect {
        if let Some(tok) = self.selected_chip.and_then(|i| self.tokens.get_mut(i)) {
            let current = tok.altitude_ft.unwrap_or(2_500);
            let next = (current + i32::from(delta_hundreds) * 100).clamp(0, 18_000);
            tok.altitude_ft = (next > 0).then_some(next);
        }
        EfbHostEffect::RouteUpdated
    }

    fn exec_cycle_runway(&mut self, catalog: &HubCatalog) -> EfbHostEffect {
        let Some(tok) = self.selected_chip.and_then(|i| self.tokens.get_mut(i)) else {
            return EfbHostEffect::RouteUpdated;
        };
        if let RouteTokenKind::Airport { code, runway, .. } = &mut tok.kind
            && let Some(entry) = catalog.aerodromes.iter().find(|a| a.code == *code)
            && !entry.runways.is_empty()
        {
            let cur_pos = runway
                .as_deref()
                .and_then(|r| entry.runways.iter().position(|e| e.designator == r))
                .unwrap_or(0);
            let next_pos = cur_pos.wrapping_add(1) % entry.runways.len();
            if let Some(next_rwy) = entry.runways.get(next_pos) {
                *runway = Some(next_rwy.designator.clone());
            }
        }
        EfbHostEffect::RouteUpdated
    }

    fn exec_delete_selected(&mut self) -> EfbHostEffect {
        if let Some(sel) = self.selected_chip
            && sel < self.tokens.len()
        {
            self.push_undo();
            let _removed = self.tokens.remove(sel);
            self.selected_chip = if self.tokens.is_empty() {
                None
            } else {
                Some(sel.min(self.tokens.len().saturating_sub(1)))
            };
            self.clamp_progress();
        }
        EfbHostEffect::RouteUpdated
    }

    fn exec_load_preset(&mut self, preset: EfbPresetId, catalog: &HubCatalog) -> EfbHostEffect {
        self.push_undo();
        self.tokens = preset.build_tokens(&mut self.next_fix_id, catalog);
        self.selected_chip = self.tokens.len().checked_sub(1);
        self.progress = RouteProgress {
            to_fix: (self.tokens.len() >= 2).then_some(1),
            complete: false,
            hold: None,
        };
        self.tab = EfbTab::Route;
        self.status_banner = format!("LOADED {}", preset.title());
        if preset == EfbPresetId::RedSquareBridgeFinal {
            self.ad_hoc.active_code = "ZMRS".to_owned();
            return EfbHostEffect::SpawnAtRunway {
                code: "ZMRS".to_owned(),
                designator: "33".to_owned(),
                station_m: -1_650.0,
                engage_autoland: true,
            };
        }
        if preset == EfbPresetId::RedSquarePlazaFinal {
            self.ad_hoc.active_code = "ZMRQ".to_owned();
            return EfbHostEffect::SpawnAtRunway {
                code: "ZMRQ".to_owned(),
                designator: "30".to_owned(),
                station_m: -1_450.0,
                engage_autoland: true,
            };
        }
        if let Some(first) = self.tokens.first() {
            return EfbHostEffect::JumpMap {
                lat_deg: first.lat_deg,
                lon_deg: first.lon_deg,
                zoom: Some(if preset == EfbPresetId::MathiasRust1987Corridor {
                    5.5
                } else {
                    9.5
                }),
            };
        }
        EfbHostEffect::RouteUpdated
    }

    fn apply_adhoc_action(
        &mut self,
        action: EfbPlanAction,
        catalog: &mut HubCatalog,
        surface: &WorldSurfaceHandle,
    ) -> EfbHostEffect {
        match action {
            EfbPlanAction::AdHocPickEfbMap => {
                self.ad_hoc.start_efb_pick();
                self.status_banner.clone_from(&self.ad_hoc.status);
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::AdHocPickWorld3d => {
                self.ad_hoc.start_world_pick();
                self.status_banner.clone_from(&self.ad_hoc.status);
                EfbHostEffect::EnterWorld3dPicker
            }
            EfbPlanAction::AdHocCycleWidth => {
                self.ad_hoc.cycle_width();
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::AdHocCycleSurface => {
                self.ad_hoc.cycle_surface();
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::AdHocSwapThresholds => {
                self.ad_hoc.swap_thresholds();
                EfbHostEffect::RouteUpdated
            }
            EfbPlanAction::AdHocCreateAndAppend => match self.ad_hoc.commit(catalog, surface) {
                Ok((_, created)) => {
                    let code = created.code().to_owned();
                    let focus_deg = [created.airport.lat_deg, created.airport.lon_deg];
                    let id = self.alloc_fix_id();
                    if let Some(mut tok) = parse_single_token(&code, id, catalog) {
                        tok.fly_over = true;
                        self.push_undo();
                        self.tokens.push(tok);
                        self.selected_chip = self.tokens.len().checked_sub(1);
                    }
                    self.status_banner.clone_from(&self.ad_hoc.status);
                    EfbHostEffect::CreatedAdHoc { code, focus_deg }
                }
                Err(err) => {
                    self.status_banner = err.to_string().to_ascii_uppercase();
                    EfbHostEffect::RouteUpdated
                }
            },
            EfbPlanAction::AdHocSpawnThreshold | EfbPlanAction::AdHocSpawnFinalAutoland => {
                let code = self.ad_hoc.active_code.clone();
                let designator = catalog
                    .ad_hoc_by_code(&code)
                    .and_then(|a| a.airport.runways.first())
                    .and_then(|r| r.ends.first())
                    .map_or_else(|| "15".to_owned(), |e| e.designator.as_str().to_owned());
                let is_final = matches!(action, EfbPlanAction::AdHocSpawnFinalAutoland);
                EfbHostEffect::SpawnAtRunway {
                    code,
                    designator,
                    station_m: if is_final { -1_650.0 } else { 0.0 },
                    engage_autoland: is_final,
                }
            }
            _ => EfbHostEffect::RouteUpdated,
        }
    }
}
