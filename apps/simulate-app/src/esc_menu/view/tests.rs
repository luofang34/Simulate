//! Hub layout regressions: text never overlaps text or bars, every widget reads in full,
//! hub strings stay inside the glyph pack, and the rewind timeline's transport.

use indicate_instrument_scene::HAlign;

use super::{DecorKind, HubView, TimelineScrub, WidgetIcon, WidgetId, WidgetStyle};
use crate::controls::{ControlScheme, KeyAction, VehicleClass, key_label};
use crate::esc_menu::catalog::{AerodromeKind, mode_label};
use crate::esc_menu::tests::{Hub, catalog, status};
use crate::esc_menu::{HubTab, MenuCommand, MenuInput, MenuOutcome, MenuStatus, scene, view};

/// `status()` with the Controls tab's rows and keys filled in from a fixed-wing scheme.
fn populated_status() -> MenuStatus {
    let scheme = ControlScheme::new(VehicleClass::FixedWing);
    let mut status = status();
    status.controls.rows = scheme.axis_rows();
    status.controls.keys = KeyAction::ALL
        .into_iter()
        .map(|a| (a, key_label(scheme.keys().key_for(a))))
        .collect();
    status
}

/// Drawn glyph height over cell height: the panel pack's glyphs overhang their cell.
const GLYPH_HEIGHT_PER_SIZE: f32 = 1.2;

/// Every drawn text run's `[x0, y0, x1, y1]` box, placed as `scene` paints it.
pub(super) fn text_boxes(view: &HubView) -> Vec<([f32; 4], String)> {
    let boxed = |[x, y]: [f32; 2], size: f32, align: HAlign, text: &str| {
        let w = text.chars().count() as f32 * size * view::ADVANCE_PER_SIZE;
        let x0 = match align {
            HAlign::Left => x,
            HAlign::Center => x - w * 0.5,
            HAlign::Right => x - w,
        };
        let half_h = size * GLYPH_HEIGHT_PER_SIZE * 0.5;
        ([x0, y - half_h, x0 + w, y + half_h], text.to_owned())
    };
    let labels = view
        .labels()
        .iter()
        .filter(|l| !l.text.is_empty())
        .map(|l| boxed(l.at, l.size, l.align, &l.text));
    let widgets = view
        .widgets()
        .iter()
        .filter(|w| !w.label.is_empty())
        .map(|w| {
            let place = scene::label_placement(w);
            boxed(place.at, place.size, place.align, &w.label)
        });
    labels.chain(widgets).collect()
}

pub(super) fn intersects(a: [f32; 4], b: [f32; 4]) -> bool {
    a[0] < b[2] - 0.5 && b[0] < a[2] - 0.5 && a[1] < b[3] - 0.5 && b[1] < a[3] - 0.5
}

#[test]
fn hub_text_never_overlaps_and_every_widget_reads_in_full() {
    let mut hub = Hub::opened();
    hub.status = populated_status();
    hub.status.rewind_offset_s = 15.0;
    for tab in HubTab::ALL {
        hub.input(MenuInput::Tab(tab));
        let view = hub.view();
        let boxes = text_boxes(&view);
        for (i, (a, ta)) in boxes.iter().enumerate() {
            for (b, tb) in boxes.iter().skip(i + 1) {
                assert!(
                    !intersects(*a, *b),
                    "{tab:?}: {ta:?} {a:?} overlaps {tb:?} {b:?}"
                );
            }
            for decor in view.decor() {
                let [x, y, w, h] = decor.rect;
                let bar = matches!(decor.kind, DecorKind::Bar { .. });
                let rect = [x, y, x + w, y + h];
                assert!(
                    !(bar && intersects(*a, rect)),
                    "{tab:?}: {ta:?} runs into a bar"
                );
            }
        }
        for widget in view.widgets() {
            // A label must show in full (list rows may clip long data names; the
            // selection's details show them); only label-less widgets rely on an icon.
            let ok = if widget.label.is_empty() {
                widget.style == WidgetStyle::Track || widget.icon != WidgetIcon::None
            } else {
                !widget.clipped || widget.style == WidgetStyle::Row
            };
            assert!(
                ok,
                "{tab:?}: {:?} shows {:?} (clipped {})",
                widget.id, widget.label, widget.clipped
            );
        }
    }
}

/// Whether the panel glyph pack draws every character of `text` (case aside).
fn draws_in_full(text: &str) -> bool {
    text.chars()
        .all(|c| c == ' ' || !view::glyph_safe(&c.to_string(), 1).is_empty())
}

#[test]
fn hub_strings_use_only_characters_the_glyph_pack_draws() {
    let catalog = catalog();
    let mut texts: Vec<String> = Vec::new();
    texts.extend(catalog.vehicles.iter().map(|v| v.title.clone()));
    texts.extend(catalog.vehicles.iter().map(|v| v.role.to_owned()));
    texts.extend(
        catalog
            .vehicles
            .iter()
            .flat_map(|v| v.modes().iter().map(|&m| mode_label(m).to_owned())),
    );
    texts.extend(catalog.scenarios.iter().map(|s| s.briefing.to_owned()));
    texts.extend(
        catalog
            .aerodromes
            .iter()
            .filter(|a| !matches!(a.kind, AerodromeKind::Osm(_)))
            .map(|a| a.name.clone()),
    );
    texts.extend(HubTab::ALL.iter().map(|t| t.title().to_owned()));
    let lossy: Vec<String> = texts.into_iter().filter(|t| !draws_in_full(t)).collect();
    assert!(lossy.is_empty(), "{lossy:#?} lose characters");
}

#[test]
fn timeline_scrubs_both_ways_and_only_live_is_highlighted() {
    let mut hub = Hub::opened();
    let texts =
        |hub: &Hub| -> Vec<String> { hub.view().labels().iter().map(|l| l.text.clone()).collect() };
    assert!(
        texts(&hub).iter().any(|t| t == "LIVE   40 S BUFFERED"),
        "{:?}",
        texts(&hub)
    );
    let active: Vec<WidgetId> = hub
        .view()
        .widgets()
        .iter()
        .filter(|w| w.active && !matches!(w.id, WidgetId::Tab(_) | WidgetId::TakeControl))
        .map(|w| w.id)
        .filter(|id| matches!(id, WidgetId::Rewind(_) | WidgetId::Scrub(_)))
        .collect();
    assert_eq!(active, [WidgetId::Scrub(TimelineScrub::Live)]);
    let style = |hub: &Hub, id| hub.view().find(id).map(|w| w.style);
    let forward = WidgetId::Scrub(TimelineScrub::Forward(5));
    assert_eq!(
        style(&hub, forward),
        Some(WidgetStyle::Muted),
        "nothing ahead"
    );
    assert_eq!(style(&hub, WidgetId::Rewind(30)), Some(WidgetStyle::Chip));
    for (id, command) in [
        (WidgetId::Rewind(30), MenuCommand::Rewind { seconds: 30 }),
        (WidgetId::Rewind(5), MenuCommand::Rewind { seconds: 5 }),
        (forward, MenuCommand::Scrub(TimelineScrub::Forward(5))),
        (
            WidgetId::Scrub(TimelineScrub::Forward(15)),
            MenuCommand::Scrub(TimelineScrub::Forward(15)),
        ),
        (
            WidgetId::Scrub(TimelineScrub::Live),
            MenuCommand::Scrub(TimelineScrub::Live),
        ),
        (
            WidgetId::TimelineBar,
            MenuCommand::Scrub(TimelineScrub::Permille(500)),
        ),
    ] {
        assert_eq!(hub.click(id), MenuOutcome::Command(command), "{id:?}");
    }

    hub.status.rewind_offset_s = 15.0;
    assert!(
        texts(&hub)
            .iter()
            .any(|t| t == "REWOUND -15.0 S   40 S BUFFERED"),
        "{:?}",
        texts(&hub)
    );
    let live = hub
        .view()
        .find(WidgetId::Scrub(TimelineScrub::Live))
        .map(|w| w.active);
    assert_eq!(live, Some(false));
    assert_eq!(style(&hub, forward), Some(WidgetStyle::Chip));
    hub.status.rewind_offset_s = 40.0;
    assert_eq!(
        style(&hub, WidgetId::Rewind(5)),
        Some(WidgetStyle::Muted),
        "at the oldest"
    );
}
