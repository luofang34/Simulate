//! `3 WORLD & AERODROMES` over the global catalog: enlarged globe with corner search,
//! non-overlapping list/details, aviation `M/T` (`M`-only fallback) runway chips, and
//! single-row `SPAWN MODE` chips.

use crate::esc_menu::catalog::{HubCatalog, RunwayEntry, aviation_heading_deg};
use crate::esc_menu::tests::{Hub, status};
use crate::esc_menu::view::tests::{intersects, text_boxes};
use crate::esc_menu::view::{HubView, WidgetStyle};
use crate::esc_menu::{EscMenu, HubTab, MenuInput, MenuOutcome, SpawnMode, WidgetId};

fn world_hub() -> Hub {
    let mut hub = Hub {
        menu: EscMenu::default(),
        catalog: HubCatalog::with_global_airports(),
        status: status(),
    };
    assert_eq!(hub.input(MenuInput::Toggle), MenuOutcome::Opened);
    hub.input(MenuInput::Tab(HubTab::World));
    hub
}

fn select(hub: &mut Hub, code: &str) {
    let index = hub
        .catalog
        .aerodromes
        .iter()
        .position(|a| a.code == code)
        .unwrap_or_else(|| panic!("{code} listed"));
    hub.menu.select_aerodrome(index);
}

fn corners([x, y, w, h]: [f32; 4]) -> [f32; 4] {
    [x, y, x + w, y + h]
}

fn contained_in(inner: [f32; 4], outer: [f32; 4]) -> bool {
    inner[0] >= outer[0] + 4.0
        && inner[1] >= outer[1] + 4.0
        && inner[2] <= outer[2] - 4.0
        && inner[3] <= outer[3] - 4.0
}

fn is_globe_corner_text(text: &str) -> bool {
    text.starts_with("SEARCH  ") || text == "DRAG SPIN   WHEEL ZOOM   CLICK PICK"
}

fn assert_clean_layout(view: &HubView, what: &str) {
    let globe_rect = view.preview_rect().expect("the globe preview");
    assert!(
        globe_rect[3] >= 340.0,
        "{what}: globe height {}",
        globe_rect[3]
    );
    let globe = corners(globe_rect);
    let search = view.find(WidgetId::Search).expect("corner search widget");
    assert!(
        contained_in(corners(search.rect), globe),
        "{what}: search {:?} not tucked inside globe {globe:?}",
        search.rect
    );
    let widgets = view.widgets();
    for (i, a) in widgets.iter().enumerate() {
        for b in widgets.iter().skip(i + 1) {
            assert!(
                !intersects(corners(a.rect), corners(b.rect)),
                "{what}: {:?} overlaps {:?}",
                a.id,
                b.id
            );
        }
        if a.id != WidgetId::Search {
            assert!(
                !intersects(corners(a.rect), globe),
                "{what}: {:?} collides with the globe",
                a.id
            );
        }
        let clipped_ok = a.style == WidgetStyle::Row;
        assert!(
            !a.clipped || clipped_ok,
            "{what}: {:?} shows {:?}",
            a.id,
            a.label
        );
    }
    let boxes = text_boxes(view);
    for (i, (a, ta)) in boxes.iter().enumerate() {
        if is_globe_corner_text(ta) {
            assert!(
                contained_in(*a, globe),
                "{what}: {ta:?} spills outside globe"
            );
        } else {
            assert!(!intersects(*a, globe), "{what}: {ta:?} under the globe");
        }
        for (b, tb) in boxes.iter().skip(i + 1) {
            assert!(!intersects(*a, *b), "{what}: {ta:?} overlaps {tb:?}");
        }
    }
    let spawn_ys: Vec<f32> = widgets
        .iter()
        .filter(|w| matches!(w.id, WidgetId::SpawnChoice(_)))
        .map(|w| w.rect[1])
        .collect();
    if let Some(&first_y) = spawn_ys.first() {
        assert!(
            spawn_ys.iter().all(|&y| (y - first_y).abs() < 1.0e-3),
            "{what}: spawn modes split across rows: {spawn_ys:?}"
        );
    }
}

#[test]
fn the_world_tab_lays_out_the_globe_list_and_chips_without_overlap() {
    let mut hub = world_hub();
    for code in ["KEWR", "KORD", "EGLL", "47N", "DELAWARE", "TRANQUILITY"] {
        select(&mut hub, code);
        let check = |hub: &Hub, what: &str| {
            for size in [[1280, 720], [2560, 1440]] {
                let view = HubView::build(&hub.menu, &hub.catalog, &hub.status, size);
                assert_clean_layout(&view, &format!("{code} {size:?} {what}"));
            }
        };
        check(&hub, "default spawn");
        let ramp = WidgetId::SpawnChoice(SpawnMode::Ramp);
        if hub.view().find(ramp).is_some() {
            let _outcome = hub.click(ramp);
            check(&hub, "ramp with the stand note");
        }
    }
}

#[test]
fn runway_chips_show_magnetic_and_true_then_fall_back_to_magnetic_only() {
    assert_eq!(aviation_heading_deg(0.0), 360);
    assert_eq!(aviation_heading_deg(359.7), 360);
    assert_eq!(aviation_heading_deg(218.8), 219);
    let rwy = RunwayEntry {
        designator: "22L".to_owned(),
        magnetic_bearing_deg: 218.8,
        true_bearing_deg: 205.8,
        length_m: 3_353.0,
    };
    assert_eq!(rwy.chip_label(18), "RWY 22L 219M/206T");
    assert_eq!(rwy.chip_label(14), "RWY 22L 219M");
    assert_eq!(rwy.chip_label(11), "22L 219M");

    let polar_rwy = RunwayEntry {
        designator: "02T".to_owned(),
        magnetic_bearing_deg: 20.0,
        true_bearing_deg: 20.0,
        length_m: 3_658.0,
    };
    assert_eq!(polar_rwy.chip_label(18), "RWY 02T 020T");
    assert_eq!(polar_rwy.chip_label(10), "02T 020T");
    assert_eq!(polar_rwy.chip_label(6), "02T");

    let mut hub = world_hub();
    select(&mut hub, "KEWR");
    let kewr_rwy = hub
        .view()
        .find(WidgetId::RunwayEnd(0))
        .expect("KEWR runway")
        .label
        .clone();
    assert!(
        kewr_rwy.contains("M/") && kewr_rwy.ends_with('T'),
        "{kewr_rwy}"
    );

    select(&mut hub, "KORD");
    let kord_rwy = hub
        .view()
        .find(WidgetId::RunwayEnd(0))
        .expect("KORD runway")
        .label
        .clone();
    assert!(
        kord_rwy.ends_with('M') && !kord_rwy.contains('/'),
        "{kord_rwy}"
    );
}

#[test]
fn typing_a_code_selects_and_lists_that_airport_first() {
    let mut hub = world_hub();
    for (query, code) in [("KJFK", "KJFK"), ("RJTT", "RJTT"), ("EGLL", "EGLL")] {
        for _ in 0..8 {
            hub.input(MenuInput::Backspace);
        }
        for ch in query.chars() {
            hub.input(MenuInput::Char(ch));
        }
        let selected = hub.catalog.aerodromes.get(hub.menu.aerodrome());
        assert_eq!(selected.map(|a| a.code.as_str()), Some(code));
        let first_row = hub.view().widgets().iter().find_map(|w| match w.id {
            WidgetId::AerodromeRow(i) => Some(i),
            _ => None,
        });
        assert_eq!(first_row, Some(hub.menu.aerodrome()), "{query}");
    }
}
