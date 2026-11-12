//! Modern `Pilotage`-inspired Mission Planner, NavLog, Ad-Hoc Runway Delineation, and
//! Presets dock laid onto [`super::EfbChrome`].

mod adhoc_presets;

use sim_maplibre::EfbDisplayMode;

use super::{EfbButton, EfbButtonTone, EfbChrome, EfbLabel, EfbPanel};
use crate::efb_plan::{ChipBadge, EfbPlanAction, EfbTab, EfbWorkspace, autocomplete_candidates};
use crate::esc_menu::catalog::HubCatalog;
use crate::esc_menu::view::Palette;
use crate::map_rig::EfbCommand;
use crate::world_surface::WorldSurfaceHandle;

const DOCK_W: f32 = 308.0;

impl EfbChrome {
    /// Enriches `self` with the cockpit [`EfbWorkspace`]: a compact active-leg glance pill
    /// in `CompactCorner` mode, or the full interactive Mission Planner & Ad-Hoc Runway dock
    /// in `ExpandedEfb` mode.
    pub fn attach_workspace(
        &mut self,
        ws: &EfbWorkspace,
        own_deg: [f64; 2],
        ground_speed_kt: f64,
        catalog: &HubCatalog,
        surface: &WorldSurfaceHandle,
    ) {
        if self.map == [0.0; 4] {
            return;
        }
        match self.mode {
            EfbDisplayMode::CompactCorner => {
                if let Some(glance) = ws.glance_pill_text(own_deg, ground_speed_kt) {
                    let [mx, my, mw, _] = self.map;
                    let pill_h = 18.0;
                    self.panels.push(EfbPanel {
                        rect: [mx, my + 26.0, mw, pill_h],
                        fill: Palette::PANEL,
                        edge: None,
                    });
                    self.labels.push(EfbLabel {
                        at: [mx + 5.0, my + 26.0 + pill_h * 0.5],
                        size: 7.8,
                        color: Palette::READOUT,
                        text: glance,
                        max_chars: 42,
                    });
                }
            }
            EfbDisplayMode::ExpandedEfb => {
                attach_expanded_dock(self, ws, own_deg, ground_speed_kt, catalog, surface);
            }
        }
    }
}

fn attach_expanded_dock(
    chrome: &mut EfbChrome,
    ws: &EfbWorkspace,
    own_deg: [f64; 2],
    gs_kt: f64,
    catalog: &HubCatalog,
    surface: &WorldSurfaceHandle,
) {
    let [mx, my, _, mh] = chrome.map;
    let dx = (mx - DOCK_W - 8.0).max(10.0);
    let top_y = my - 52.0;
    let total_h = mh + 60.0;

    if !ws.dock_open {
        chrome.buttons.push(EfbButton {
            command: EfbCommand::Plan(EfbPlanAction::ToggleDock),
            rect: [dx.max(12.0), top_y + 4.0, 92.0, 22.0],
            label: "FMS PLANNER".to_owned(),
            active: false,
            tone: EfbButtonTone::Emerald,
        });
        return;
    }

    chrome.panels.push(EfbPanel {
        rect: [dx, top_y, DOCK_W, total_h],
        fill: Palette::CARD,
        edge: Some(Palette::EDGE),
    });
    layout_dock_header_and_tabs(chrome, ws, own_deg, gs_kt, [dx, top_y]);

    let body = [dx + 8.0, top_y + 81.0, DOCK_W - 16.0, total_h - 106.0];
    match ws.tab {
        EfbTab::Route => layout_route_tab(chrome, ws, own_deg, catalog, body),
        EfbTab::NavLog => layout_navlog_tab(chrome, ws, gs_kt, body),
        EfbTab::AdHocRwy => adhoc_presets::layout_adhoc_tab(chrome, ws, surface, body),
        EfbTab::Presets => adhoc_presets::layout_presets_tab(chrome, body),
    }

    chrome.labels.push(EfbLabel {
        at: [dx + 8.0, top_y + total_h - 13.0],
        size: 7.6,
        color: Palette::VALUE,
        text: ws.status_banner.clone(),
        max_chars: 46,
    });
}

fn layout_dock_header_and_tabs(
    chrome: &mut EfbChrome,
    ws: &EfbWorkspace,
    own_deg: [f64; 2],
    gs_kt: f64,
    [dx, top_y]: [f32; 2],
) {
    chrome.labels.push(EfbLabel {
        at: [dx + 8.0, top_y + 15.0],
        size: 9.8,
        color: Palette::TITLE,
        text: "EFB FMS".to_owned(),
        max_chars: 12,
    });
    let ap_on = ws.autopilot.is_engaged();
    chrome.buttons.push(EfbButton {
        command: EfbCommand::Plan(EfbPlanAction::ToggleAutopilot),
        rect: [dx + 74.0, top_y + 5.0, 86.0, 20.0],
        label: ws.autopilot.phase().badge().to_owned(),
        active: ap_on,
        tone: if ap_on {
            EfbButtonTone::Emerald
        } else {
            EfbButtonTone::Standard
        },
    });
    chrome.buttons.push(EfbButton::std_btn(
        EfbCommand::Plan(EfbPlanAction::ExportLink),
        [dx + 164.0, top_y + 5.0, 74.0, 20.0],
        "LINK V2",
        ws.last_exported_link.is_some(),
    ));
    chrome.buttons.push(EfbButton::std_btn(
        EfbCommand::Plan(EfbPlanAction::ToggleDock),
        [dx + 242.0, top_y + 5.0, 58.0, 20.0],
        "FOLD",
        false,
    ));

    let tab_y = top_y + 29.0;
    let tab_w = (DOCK_W - 16.0 - 9.0) / 4.0;
    for (idx, tab) in EfbTab::ALL.into_iter().enumerate() {
        let tx = dx + 8.0 + idx as f32 * (tab_w + 3.0);
        chrome.buttons.push(EfbButton::std_btn(
            EfbCommand::Plan(EfbPlanAction::SelectTab(tab)),
            [tx, tab_y, tab_w, 20.0],
            tab.label(),
            ws.tab == tab,
        ));
    }

    let sum_y = top_y + 53.0;
    chrome.panels.push(EfbPanel {
        rect: [dx + 8.0, sum_y, DOCK_W - 16.0, 22.0],
        fill: Palette::PANEL,
        edge: None,
    });
    let summary_line = match ws.active_summary(own_deg, gs_kt) {
        Some(s) => format!(
            "TO {}  {:03.0}T  {:.1}NM  XTK {:+.0}M  RNP {:.1}",
            s.to_ident, s.bearing_deg, s.distance_nm, s.cross_track_m, s.rnp_nm
        ),
        None => format!(
            "{} FIXES - SELECT PRESETS OR ADD WAYPOINTS",
            ws.tokens.len()
        ),
    };
    chrome.labels.push(EfbLabel {
        at: [dx + 13.0, sum_y + 11.0],
        size: 8.0,
        color: Palette::READOUT,
        text: summary_line,
        max_chars: 44,
    });
}

fn layout_route_tab(
    chrome: &mut EfbChrome,
    ws: &EfbWorkspace,
    own_deg: [f64; 2],
    catalog: &HubCatalog,
    [bx, by, bw, _bh]: [f32; 4],
) {
    let cursor = if ws.omni_focused { "-" } else { "" };
    let prompt = if ws.omni_query.is_empty() && !ws.omni_focused {
        "TYPE ICAO/FIX (EFHF KREML ZMRS/15)".to_owned()
    } else {
        format!("ADD: {}{cursor}", ws.omni_query)
    };
    chrome.buttons.push(EfbButton {
        command: EfbCommand::Plan(EfbPlanAction::FocusOmni(!ws.omni_focused)),
        rect: [bx, by, bw - 48.0, 24.0],
        label: prompt,
        active: ws.omni_focused,
        tone: EfbButtonTone::Field,
    });
    chrome.buttons.push(EfbButton {
        command: EfbCommand::Plan(EfbPlanAction::SubmitOmni),
        rect: [bx + bw - 44.0, by, 44.0, 24.0],
        label: "ADD".to_owned(),
        active: !ws.omni_query.is_empty(),
        tone: EfbButtonTone::Emerald,
    });

    let mut y = by + 28.0;
    if ws.omni_focused || !ws.omni_query.is_empty() {
        let items = autocomplete_candidates(&ws.omni_query, Some(own_deg), catalog, 4);
        for (i, item) in items.iter().enumerate() {
            chrome.buttons.push(EfbButton {
                command: EfbCommand::Plan(EfbPlanAction::PickSuggestion(i)),
                rect: [bx, y, bw, 20.0],
                label: item.label.clone(),
                active: false,
                tone: if item.badge == ChipBadge::AdHocRunway {
                    EfbButtonTone::Emerald
                } else {
                    EfbButtonTone::Standard
                },
            });
            y += 22.0;
        }
        y += 2.0;
    }

    layout_route_chips_and_actions(chrome, ws, [bx, by, bw, y]);
}

fn layout_route_chips_and_actions(
    chrome: &mut EfbChrome,
    ws: &EfbWorkspace,
    [bx, by, bw, chips_y]: [f32; 4],
) {
    let chip_cols = 3usize;
    let chip_w = (bw - 8.0) / 3.0;
    let total = ws.tokens.len();
    let active_leg = ws.active_leg_index();
    let start = ws
        .selected_chip
        .unwrap_or(active_leg)
        .saturating_sub(6)
        .min(total.saturating_sub(15));
    for (slot, (idx, tok)) in ws
        .tokens
        .iter()
        .enumerate()
        .skip(start)
        .take(15)
        .enumerate()
    {
        let col = (slot % chip_cols) as f32;
        let row = (slot / chip_cols) as f32;
        let is_active_to = idx == active_leg.wrapping_add(1);
        let tone = if is_active_to || tok.hold.is_some() {
            EfbButtonTone::Amber
        } else if tok.badge() == ChipBadge::AdHocRunway {
            EfbButtonTone::Emerald
        } else {
            EfbButtonTone::Standard
        };
        chrome.buttons.push(EfbButton {
            command: EfbCommand::Plan(EfbPlanAction::SelectChip(idx)),
            rect: [
                bx + col * (chip_w + 4.0),
                chips_y + row * 24.0,
                chip_w,
                21.0,
            ],
            label: tok.chip_label(),
            active: ws.selected_chip == Some(idx),
            tone,
        });
    }
    layout_route_action_rows(chrome, ws, [bx, by + 308.0, bw]);
}

fn layout_route_action_rows(chrome: &mut EfbChrome, ws: &EfbWorkspace, [bx, act_y, bw]: [f32; 3]) {
    let w5 = (bw - 16.0) / 5.0;
    for (i, (act, lbl, em)) in [
        (EfbPlanAction::DirectToSelected, "D-TO", true),
        (EfbPlanAction::MoveSelected(-1), "PREV", false),
        (EfbPlanAction::MoveSelected(1), "NEXT", false),
        (EfbPlanAction::CycleHoldSelected, "HOLD", false),
        (EfbPlanAction::CycleRunwaySelected, "RWY", false),
    ]
    .into_iter()
    .enumerate()
    {
        chrome.buttons.push(EfbButton {
            command: EfbCommand::Plan(act),
            rect: [bx + i as f32 * (w5 + 4.0), act_y, w5, 21.0],
            label: lbl.to_owned(),
            active: false,
            tone: if em {
                EfbButtonTone::Emerald
            } else {
                EfbButtonTone::Standard
            },
        });
    }
    for (i, (act, lbl)) in [
        (EfbPlanAction::AdjustAltSelected(5), "ALT+"),
        (EfbPlanAction::AdjustAltSelected(-5), "ALT-"),
        (EfbPlanAction::FocusSelectedOnMap, "GOTO"),
        (EfbPlanAction::DeleteSelected, "DEL"),
        (EfbPlanAction::UndoRoute, "UNDO"),
    ]
    .into_iter()
    .enumerate()
    {
        chrome.buttons.push(EfbButton::std_btn(
            EfbCommand::Plan(act),
            [bx + i as f32 * (w5 + 4.0), act_y + 25.0, w5, 21.0],
            lbl,
            false,
        ));
    }
    let half_w = (bw - 4.0) * 0.5;
    chrome.buttons.push(EfbButton::std_btn(
        EfbCommand::Plan(EfbPlanAction::ToggleMapClickInsert),
        [bx, act_y + 50.0, half_w, 20.0],
        if ws.map_click_insert {
            "MAP CLICK: +WPT"
        } else {
            "MAP CLICK: PAN"
        },
        ws.map_click_insert,
    ));
    chrome.buttons.push(EfbButton::std_btn(
        EfbCommand::Plan(EfbPlanAction::ClearRoute),
        [bx + half_w + 4.0, act_y + 50.0, half_w, 20.0],
        "CLEAR PLAN",
        false,
    ));
}

fn layout_navlog_tab(
    chrome: &mut EfbChrome,
    ws: &EfbWorkspace,
    gs_kt: f64,
    [bx, by, bw, _]: [f32; 4],
) {
    chrome.labels.push(EfbLabel {
        at: [bx + 4.0, by + 8.0],
        size: 8.0,
        color: Palette::HINT,
        text: "LEG              CRS   DIST   CUM    ETE   ALT".to_owned(),
        max_chars: 48,
    });
    let rows = ws.navlog_rows(gs_kt);
    let active_leg = ws.active_leg_index();
    let start = active_leg.saturating_sub(2);
    for (slot, row) in rows.iter().skip(start).take(12).enumerate() {
        let ry = by + 22.0 + slot as f32 * 24.0;
        let active = row.leg_index == active_leg;
        chrome.panels.push(EfbPanel {
            rect: [bx, ry, bw, 21.0],
            fill: if active {
                Palette::ACTIVE
            } else {
                Palette::PANEL
            },
            edge: None,
        });
        let mins = (row.ete_s / 60.0).round() as u32;
        let alt = row
            .altitude_ft
            .map_or_else(|| "----".to_owned(), |ft| format!("{ft:4}"));
        let line = format!(
            "{:<15} {:03.0}T {:5.1} {:5.0} {:3}M  {alt}",
            format!("{}-{}", row.from_ident, row.to_ident),
            row.course_true_deg,
            row.leg_nm,
            row.cumulative_nm,
            mins
        );
        chrome.labels.push(EfbLabel {
            at: [bx + 5.0, ry + 10.5],
            size: 7.8,
            color: if active {
                Palette::TITLE
            } else {
                Palette::LABEL
            },
            text: line,
            max_chars: 48,
        });
    }
    if let Some(last) = rows.last() {
        let tot_min = (rows.iter().map(|r| r.ete_s).sum::<f64>() / 60.0).round() as u32;
        chrome.labels.push(EfbLabel {
            at: [bx + 4.0, by + 330.0],
            size: 8.8,
            color: Palette::READOUT,
            text: format!(
                "TOTAL {:.1} NM   ETE {}H{:02}M   GS {:.0} KT",
                last.cumulative_nm,
                tot_min / 60,
                tot_min % 60,
                gs_kt.max(40.0)
            ),
            max_chars: 46,
        });
    }
}
