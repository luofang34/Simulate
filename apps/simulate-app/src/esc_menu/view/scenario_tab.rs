//! `4 SCENARIOS & MISSIONS`: the categorized browser and the mission briefing card.

use indicate_instrument_scene::HAlign;

use super::{HubView, MenuStatus, Palette, SCENARIO_ROWS, WidgetId, WidgetStyle, fit_chars};
use crate::esc_menu::EscMenu;
use crate::esc_menu::catalog::{HubCatalog, ScenarioEntry};

const LIST_W: f32 = 440.0;
const ROW_H: f32 = 44.0;

pub(super) fn build(
    view: &mut HubView,
    menu: &EscMenu,
    catalog: &HubCatalog,
    status: &MenuStatus,
    [x, y, w, h]: [f32; 4],
) {
    view.panel([x, y, LIST_W, h], Palette::PANEL, None);
    let rows = catalog
        .scenarios
        .iter()
        .enumerate()
        .skip(menu.scenario_scroll())
        .take(SCENARIO_ROWS);
    for (slot, (index, entry)) in rows.enumerate() {
        let ry = y + 4.0 + slot as f32 * ROW_H;
        let rect = [x + 4.0, ry, LIST_W - 8.0, ROW_H - 6.0];
        view.widget(
            WidgetId::ScenarioRow(index),
            rect,
            entry.title(),
            WidgetStyle::Row,
            index == menu.scenario(),
        );
        view.text_fit(
            [x + 14.0, ry + ROW_H - 14.0],
            9.0,
            Palette::HINT,
            LIST_W - 28.0,
            entry.category.title(),
        );
    }
    let bx = x + LIST_W + 24.0;
    let bw = w - LIST_W - 24.0;
    if let Some(entry) = catalog.scenarios.get(menu.scenario()) {
        briefing(view, entry, status, [bx, y, bw]);
    }
    let button = [bx + bw - 280.0, y + h - 48.0, 280.0, 48.0];
    view.widget(
        WidgetId::Launch,
        button,
        "Launch Scenario",
        WidgetStyle::Primary,
        false,
    );
}

fn briefing(view: &mut HubView, entry: &ScenarioEntry, status: &MenuStatus, [x, y, w]: [f32; 3]) {
    view.panel([x, y, w, 300.0], Palette::PANEL, Some(Palette::EDGE));
    view.text(
        [x + 16.0, y + 22.0],
        10.0,
        HAlign::Left,
        Palette::VALUE,
        entry.category.title(),
    );
    view.text_fit(
        [x + 16.0, y + 48.0],
        16.0,
        Palette::TITLE,
        w - 32.0,
        entry.title(),
    );
    let mut line_y = y + 82.0;
    for line in wrap(entry.briefing, fit_chars(w - 32.0, 11.5)) {
        view.text(
            [x + 16.0, line_y],
            11.5,
            HAlign::Left,
            Palette::LABEL,
            &line,
        );
        line_y += 20.0;
    }
    let world = entry
        .profile
        .map_or("SELECTED SPAWN", |p| world_title(p.world()));
    let facts = [
        format!("WORLD {world}"),
        format!("SKY EPHEMERIS {}", status.sky),
        "WEATHER ISA STANDARD DAY, CALM".to_owned(),
    ];
    for (i, fact) in facts.iter().enumerate() {
        let at = [x + 16.0, y + 210.0 + i as f32 * 22.0];
        view.text_fit(at, 10.5, Palette::READOUT, w - 32.0, fact);
    }
}

const fn world_title(world: sim_scenario::ScenarioWorld) -> &'static str {
    match world {
        sim_scenario::ScenarioWorld::EarthGlobe => "EARTH GLOBE",
        sim_scenario::ScenarioWorld::DelawareCorridor => "DELAWARE CORRIDOR",
        sim_scenario::ScenarioWorld::CentralJersey47N => "CENTRAL JERSEY 47N",
        sim_scenario::ScenarioWorld::NewarkKewr => "NEWARK KEWR",
        sim_scenario::ScenarioWorld::LunarTranquility => "MOON, SEA OF TRANQUILITY",
    }
}

/// Greedy word wrap at `width` characters.
fn wrap(text: &str, width: usize) -> Vec<String> {
    let mut lines = Vec::new();
    let mut line = String::new();
    for word in text.split_whitespace() {
        if !line.is_empty() && line.len().saturating_add(word.len()) >= width {
            lines.push(std::mem::take(&mut line));
        }
        if !line.is_empty() {
            line.push(' ');
        }
        line.push_str(word);
    }
    if !line.is_empty() {
        lines.push(line);
    }
    lines
}
