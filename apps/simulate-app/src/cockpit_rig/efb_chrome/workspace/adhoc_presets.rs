//! Ad-Hoc Runway Delineation (`AD-HOC RWY`) and Curated Mission Presets (`PRESETS`) dock
//! tab layouts for [`crate::cockpit_rig::efb_chrome::EfbChrome`].

use super::super::{EfbButton, EfbButtonTone, EfbChrome, EfbLabel, EfbPanel};
use crate::efb_plan::{EfbPlanAction, EfbPresetId, EfbWorkspace};
use crate::esc_menu::view::Palette;
use crate::map_rig::EfbCommand;
use crate::world_surface::WorldSurfaceHandle;

pub(super) fn layout_adhoc_tab(
    chrome: &mut EfbChrome,
    ws: &EfbWorkspace,
    surface: &WorldSurfaceHandle,
    [bx, by, bw, _]: [f32; 4],
) {
    let half = (bw - 6.0) * 0.5;
    chrome.buttons.push(EfbButton {
        command: EfbCommand::Plan(EfbPlanAction::AdHocPickWorld3d),
        rect: [bx, by, half, 28.0],
        label: "PICK ON 3D WORLD".to_owned(),
        active: ws.ad_hoc.pick_mode.is_world_3d(),
        tone: EfbButtonTone::Emerald,
    });
    chrome.buttons.push(EfbButton {
        command: EfbCommand::Plan(EfbPlanAction::AdHocPickEfbMap),
        rect: [bx + half + 6.0, by, half, 28.0],
        label: "PICK ON EFB MAP".to_owned(),
        active: ws.ad_hoc.pick_mode.is_efb_map(),
        tone: EfbButtonTone::Emerald,
    });

    let w3 = (bw - 8.0) / 3.0;
    chrome.buttons.push(EfbButton::std_btn(
        EfbCommand::Plan(EfbPlanAction::AdHocCycleWidth),
        [bx, by + 34.0, w3, 22.0],
        format!("W {:.0}M", ws.ad_hoc.width_m),
        false,
    ));
    chrome.buttons.push(EfbButton::std_btn(
        EfbCommand::Plan(EfbPlanAction::AdHocCycleSurface),
        [bx + w3 + 4.0, by + 34.0, w3, 22.0],
        format!("SURF {}", ws.ad_hoc.surface_label()),
        false,
    ));
    chrome.buttons.push(EfbButton::std_btn(
        EfbCommand::Plan(EfbPlanAction::AdHocSwapThresholds),
        [bx + 2.0 * (w3 + 4.0), by + 34.0, w3, 22.0],
        "SWAP A/B",
        false,
    ));

    layout_adhoc_preview_and_spawn(chrome, ws, surface, [bx, by, bw, half]);
}

fn layout_adhoc_preview_and_spawn(
    chrome: &mut EfbChrome,
    ws: &EfbWorkspace,
    surface: &WorldSurfaceHandle,
    [bx, by, bw, half]: [f32; 4],
) {
    chrome.panels.push(EfbPanel {
        rect: [bx, by + 64.0, bw, 92.0],
        fill: Palette::PANEL,
        edge: Some(Palette::EDGE),
    });
    let a_txt = ws.ad_hoc.threshold_a_deg.map_or_else(
        || "THR A: UNSET".to_owned(),
        |[la, lo]| format!("THR A: {la:8.4}  {lo:9.4}"),
    );
    let b_txt = ws.ad_hoc.threshold_b_deg.map_or_else(
        || "THR B: UNSET".to_owned(),
        |[la, lo]| format!("THR B: {la:8.4}  {lo:9.4}"),
    );
    for (dy, txt) in [(80.0, a_txt), (98.0, b_txt)] {
        chrome.labels.push(EfbLabel {
            at: [bx + 8.0, by + dy],
            size: 8.2,
            color: Palette::LABEL,
            text: txt,
            max_chars: 42,
        });
    }
    if let Some(p) = ws.ad_hoc.preview(surface) {
        chrome.labels.push(EfbLabel {
            at: [bx + 8.0, by + 118.0],
            size: 8.4,
            color: Palette::READOUT,
            text: format!(
                "{} RWY {}  {:.0}X{:.0}M {}",
                p.code, p.designator_pair, p.length_m, p.width_m, p.surface_label
            ),
            max_chars: 44,
        });
        chrome.labels.push(EfbLabel {
            at: [bx + 8.0, by + 136.0],
            size: 8.0,
            color: Palette::VALUE,
            text: format!(
                "HDG {:03}M/{:03}T   ELEV {:.0}M MSL",
                p.mag_bearing_deg, p.true_bearing_deg, p.elevation_msl_m
            ),
            max_chars: 44,
        });
    }
    layout_adhoc_action_buttons(chrome, ws, [bx, by, bw, half]);
}

fn layout_adhoc_action_buttons(
    chrome: &mut EfbChrome,
    ws: &EfbWorkspace,
    [bx, by, bw, half]: [f32; 4],
) {
    chrome.buttons.push(EfbButton {
        command: EfbCommand::Plan(EfbPlanAction::AdHocCreateAndAppend),
        rect: [bx, by + 166.0, bw, 28.0],
        label: "CREATE RUNWAY + ADD TO ROUTE".to_owned(),
        active: false,
        tone: EfbButtonTone::Emerald,
    });
    chrome.labels.push(EfbLabel {
        at: [bx + 4.0, by + 212.0],
        size: 8.2,
        color: Palette::HINT,
        text: format!("ACTIVE STRIP: {}", ws.ad_hoc.active_code),
        max_chars: 40,
    });
    chrome.buttons.push(EfbButton::std_btn(
        EfbCommand::Plan(EfbPlanAction::AdHocSpawnThreshold),
        [bx, by + 226.0, half, 28.0],
        "SPAWN THRESHOLD",
        false,
    ));
    chrome.buttons.push(EfbButton {
        command: EfbCommand::Plan(EfbPlanAction::AdHocSpawnFinalAutoland),
        rect: [bx + half + 6.0, by + 226.0, half, 28.0],
        label: "FINAL + AUTOLAND".to_owned(),
        active: false,
        tone: EfbButtonTone::Amber,
    });
    chrome.buttons.push(EfbButton {
        command: EfbCommand::Plan(EfbPlanAction::LoadPreset(EfbPresetId::RedSquareBridgeFinal)),
        rect: [bx, by + 264.0, bw, 30.0],
        label: "DEMO: ZMRS RED SQUARE BRIDGE AUTOLAND".to_owned(),
        active: false,
        tone: EfbButtonTone::Emerald,
    });
}

pub(super) fn layout_presets_tab(chrome: &mut EfbChrome, [bx, by, bw, _]: [f32; 4]) {
    for (i, preset) in EfbPresetId::ALL.into_iter().enumerate() {
        let py = by + i as f32 * 58.0;
        chrome.panels.push(EfbPanel {
            rect: [bx, py, bw, 52.0],
            fill: Palette::PANEL,
            edge: Some(Palette::EDGE),
        });
        chrome.buttons.push(EfbButton {
            command: EfbCommand::Plan(EfbPlanAction::LoadPreset(preset)),
            rect: [bx + 4.0, py + 4.0, bw - 8.0, 24.0],
            label: preset.title().to_owned(),
            active: false,
            tone: if matches!(
                preset,
                EfbPresetId::MathiasRust1987Corridor
                    | EfbPresetId::RedSquareBridgeFinal
                    | EfbPresetId::RedSquarePlazaFinal
            ) {
                EfbButtonTone::Emerald
            } else {
                EfbButtonTone::Standard
            },
        });
        chrome.labels.push(EfbLabel {
            at: [bx + 8.0, py + 39.0],
            size: 7.4,
            color: Palette::HINT,
            text: preset.subtitle().to_owned(),
            max_chars: 46,
        });
    }
}
