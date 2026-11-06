//! `3 WORLD & AERODROMES`: full-height ranked aerodrome list on the left; on the right an
//! enlarged interactive 3D globe with an overlaid corner search box, aviation `M/T` runway
//! chips (with `M`-only fallback when tight), and a single-row `SPAWN MODE` bar.

use indicate_instrument_scene::HAlign;

use super::{AERODROME_ROWS, HubView, Palette, WidgetId, WidgetStyle, fit_chars, label_size};
use crate::esc_menu::EscMenu;
use crate::esc_menu::catalog::{AerodromeEntry, HubCatalog, SpawnMode};

const LIST_W: f32 = 320.0;
const COLUMN_GAP: f32 = 20.0;
const ROW_H: f32 = 44.0;
const GLOBE_H: f32 = 350.0;
const SEARCH_W: f32 = 316.0;
const SEARCH_H: f32 = 34.0;
const CHIP_H: f32 = 27.0;
const GAP: f32 = 6.0;
/// Most runway-end chips shown (two rows of 4..=6 ends).
const RUNWAY_CHIPS: usize = 12;
const SPAWN_BTN_W: f32 = 240.0;
const SPAWN_BTN_H: f32 = 38.0;

pub(super) fn build(
    view: &mut HubView,
    menu: &EscMenu,
    catalog: &HubCatalog,
    [x, y, w, h]: [f32; 4],
) {
    let rx = x + LIST_W + COLUMN_GAP;
    let rw = w - LIST_W - COLUMN_GAP;
    globe_with_corner_search(view, menu, [rx, y, rw, GLOBE_H]);
    list(view, menu, catalog, [x, y, h]);
    if let Some(entry) = catalog.aerodromes.get(menu.aerodrome()) {
        details(view, menu, entry, [rx, y + GLOBE_H, rw, h - GLOBE_H]);
    }
}

/// Enlarged 3D globe frame with the search field tucked into its top-left corner and the
/// interaction legend tucked into its bottom-right corner.
fn globe_with_corner_search(view: &mut HubView, menu: &EscMenu, [rx, y, rw, gh]: [f32; 4]) {
    view.panel([rx, y, rw, gh], Palette::STUDIO, Some(Palette::EDGE));
    view.set_preview([rx + 2.0, y + 2.0, rw - 4.0, gh - 4.0]);
    let field = format!("SEARCH  {}-", menu.search());
    view.widget(
        WidgetId::Search,
        [rx + 12.0, y + 12.0, SEARCH_W, SEARCH_H],
        &field,
        WidgetStyle::Field,
        !menu.search().is_empty(),
    );
    let [bx, by, bw, bh] = [rx + rw - 294.0, y + gh - 28.0, 282.0, 18.0];
    view.panel([bx, by, bw, bh], Palette::CARD, None);
    view.text(
        [bx + bw * 0.5, by + bh * 0.5],
        9.0,
        HAlign::Center,
        Palette::HINT,
        "DRAG SPIN   WHEEL ZOOM   CLICK PICK",
    );
}

/// Full-height ranked aerodrome list on the left column.
fn list(view: &mut HubView, menu: &EscMenu, catalog: &HubCatalog, [x, y, h]: [f32; 3]) {
    let matches = catalog.filter_aerodromes(menu.search());
    view.panel([x, y, LIST_W, h], Palette::PANEL, None);
    if matches.is_empty() {
        view.text(
            [x + 14.0, y + 22.0],
            11.0,
            HAlign::Left,
            Palette::WARN,
            "NO MATCHING AERODROME",
        );
    }
    for (slot, &index) in matches
        .iter()
        .skip(menu.aerodrome_scroll())
        .take(AERODROME_ROWS)
        .enumerate()
    {
        let Some(entry) = catalog.aerodromes.get(index) else {
            continue;
        };
        let row_y = y + 4.0 + slot as f32 * ROW_H;
        let rect = [x + 4.0, row_y, LIST_W - 8.0, ROW_H - 6.0];
        let label = format!("{}   {}", entry.code, entry.name);
        view.widget(
            WidgetId::AerodromeRow(index),
            rect,
            &label,
            WidgetStyle::Row,
            index == menu.aerodrome(),
        );
        if !entry.location_hint.is_empty() {
            view.text_fit(
                [x + 14.0, row_y + 31.0],
                9.0,
                Palette::HINT,
                LIST_W - 28.0,
                &entry.location_hint,
            );
        }
    }
}

fn details(view: &mut HubView, menu: &EscMenu, entry: &AerodromeEntry, [x, y, w, _h]: [f32; 4]) {
    let text_w = (w - SPAWN_BTN_W - 14.0).max(80.0);
    view.text_fit([x, y + 17.0], 14.5, Palette::TITLE, text_w, &entry.name);
    let facts = format_aerodrome_facts(entry);
    view.text_fit([x, y + 36.0], 10.0, Palette::READOUT, text_w, &facts);
    view.widget(
        WidgetId::SpawnHere,
        [x + w - SPAWN_BTN_W, y + 8.0, SPAWN_BTN_W, SPAWN_BTN_H],
        "Spawn At Selected",
        WidgetStyle::Primary,
        false,
    );
    let spawn_y = if entry.runways.is_empty() {
        y + 56.0
    } else {
        runway_chips(view, menu, entry, [x, y + 54.0, w])
    };
    spawn_chips_single_row(view, menu, entry, [x, spawn_y, w]);
}

fn format_aerodrome_facts(entry: &AerodromeEntry) -> String {
    let elev_ft = entry.field_elevation_m * 3.280_84;
    let var = match entry.mag_var_deg {
        Some(deg) if deg.abs() >= 0.05 => {
            let ew = if deg >= 0.0 { 'E' } else { 'W' };
            format!("   VAR {:.0}{ew}", deg.abs())
        }
        Some(_) => "   VAR 0".to_owned(),
        None => String::new(),
    };
    let loc = if entry.location_hint.is_empty() {
        String::new()
    } else {
        format!("   {}", entry.location_hint)
    };
    format!(
        "{}   {elev_ft:.0} FT / {:.0} M{var}{loc}",
        entry.code, entry.field_elevation_m
    )
}

/// Runway-end chips from `y` (at most two rows); returns the next section's `y`.
fn runway_chips(
    view: &mut HubView,
    menu: &EscMenu,
    entry: &AerodromeEntry,
    [x, y, w]: [f32; 3],
) -> f32 {
    view.text(
        [x, y + 6.0],
        10.5,
        HAlign::Left,
        Palette::LABEL,
        "RUNWAY END   MAG / TRUE",
    );
    let top = y + 16.0;
    let shown = entry.runways.len().min(RUNWAY_CHIPS);
    let per_row = if shown <= 8 {
        4
    } else if shown <= 10 {
        5
    } else {
        6
    };
    let chip_w = (w - GAP * (per_row as f32 - 1.0)) / per_row as f32;
    let max_chars = fit_chars(chip_w - 16.0, label_size(WidgetStyle::Chip));
    for (i, rwy) in entry.runways.iter().enumerate().take(RUNWAY_CHIPS) {
        let col = (i % per_row) as f32;
        let row = (i / per_row) as f32;
        let rect = [
            x + col * (chip_w + GAP),
            top + row * (CHIP_H + GAP),
            chip_w,
            CHIP_H,
        ];
        let label = rwy.chip_label(max_chars);
        view.widget(
            WidgetId::RunwayEnd(i),
            rect,
            &label,
            WidgetStyle::Chip,
            i == menu.runway(),
        );
    }
    let rows = shown.div_ceil(per_row) as f32;
    top + rows * (CHIP_H + GAP) + 4.0
}

/// All supported spawn-mode chips laid out on one single horizontal row.
fn spawn_chips_single_row(
    view: &mut HubView,
    menu: &EscMenu,
    entry: &AerodromeEntry,
    [x, y, w]: [f32; 3],
) {
    view.text(
        [x, y + 6.0],
        10.5,
        HAlign::Left,
        Palette::LABEL,
        "SPAWN MODE",
    );
    let modes = entry.spawn_modes();
    let chosen = menu
        .spawn_mode()
        .filter(|m| modes.contains(m))
        .or_else(|| modes.first().copied());
    if let Some(stand) = entry
        .stands
        .first()
        .filter(|_| chosen == Some(SpawnMode::Ramp))
    {
        view.text(
            [x + w, y + 6.0],
            9.5,
            HAlign::Right,
            Palette::HINT,
            &format!("PARKED AT {stand} - ENGINE OFF"),
        );
    }
    let top = y + 16.0;
    let slots = SpawnMode::ALL.len() as f32;
    let chip_w = (w - GAP * (slots - 1.0)) / slots;
    for (i, mode) in modes.iter().enumerate() {
        let rect = [x + i as f32 * (chip_w + GAP), top, chip_w, CHIP_H + 1.0];
        view.widget(
            WidgetId::SpawnChoice(*mode),
            rect,
            mode.title(),
            WidgetStyle::Chip,
            Some(*mode) == chosen,
        );
    }
}

#[cfg(test)]
mod tests;
