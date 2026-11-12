//! Modernized web `#v=2` Route & Presets Editor (`EFB`) mounted on the link-plan surface,
//! sharing [`crate::efb_plan`]'s chip model, omni-search parser, holds, and `EFHF -> ZMRS`
//! presets so editing or switching routes on the web page takes one click.

use std::cell::RefCell;
use std::rc::Rc;

use sim_link::SimulationLink;
use wasm_bindgen::JsCast;
use wasm_bindgen::closure::Closure;
use web_sys::{Document, Element, Event, HtmlInputElement};

use super::progress_surface::{ProgressSurface, attribute};
use super::section::append;
use crate::efb_plan::navlog::{build_simulation_link_legs, import_simulation_link_fragment};
use crate::efb_plan::tokens::{HoldSpec, HoldTurn};
use crate::efb_plan::{ChipBadge, EfbPresetId, RouteToken, parse_single_token};
use crate::esc_menu::HubCatalog;
use crate::world_surface::WorldSurfaceHandle;

struct WebEditorState {
    document: Document,
    chips_box: Element,
    status_el: Element,
    input_el: HtmlInputElement,
    catalog: HubCatalog,
    template: Option<SimulationLink>,
    tokens: Vec<RouteToken>,
    selected: Option<usize>,
    next_id: u32,
    closures: Vec<Closure<dyn FnMut(Event)>>,
}

/// Mounts a collapsible `Route & Mission Editor (EFB)` drawer onto `surface`, pre-loaded
/// from `initial_fragment` (and preserving `template`'s sensor sets, faults, height, and
/// camera settings when applying route edits).
pub fn mount_route_editor(
    surface: &ProgressSurface,
    initial_fragment: &str,
    template: Option<&SimulationLink>,
) {
    let Some(section) = surface.section("Route & mission editor (EFB)", 1) else {
        return;
    };
    section.settled(&[
        "Edit route chips, toggle holds, load Mathias Rust 1987 (EFHF -> ZMRS), or open interactive cockpit."
            .to_owned(),
    ]);
    let doc = section.document().clone();
    let Some(drawer) = append(&doc, section.root(), "div", "efb-web-drawer") else {
        return;
    };
    let Some((omni_row, input_el)) = build_omni_bar(&doc, &drawer) else {
        return;
    };
    let Some(chips_box) = append(&doc, &drawer, "div", "efb-web-chips") else {
        return;
    };
    let Some(actions_row) = append(&doc, &drawer, "div", "efb-web-actions") else {
        return;
    };
    let Some(presets_row) = append(&doc, &drawer, "div", "efb-web-presets") else {
        return;
    };
    let Some(status_el) = append(&doc, &drawer, "p", "detail efb-web-status") else {
        return;
    };

    let mut catalog = HubCatalog::with_global_airports();
    let world_surface = WorldSurfaceHandle::new();
    catalog.seed_ad_hoc_presets(&world_surface);

    let mut next_id = 1u32;
    let tokens = import_simulation_link_fragment(initial_fragment, &mut next_id, &catalog)
        .unwrap_or_default();
    let selected = tokens.len().checked_sub(1);

    let state = Rc::new(RefCell::new(WebEditorState {
        document: doc,
        chips_box,
        status_el,
        input_el,
        catalog,
        template: template.cloned(),
        tokens,
        selected,
        next_id,
        closures: Vec::new(),
    }));

    wire_omni_and_actions(&state, &omni_row, &actions_row);
    wire_presets_and_apply(&state, &presets_row);
    refresh_chips(&state);
}

fn build_omni_bar(doc: &Document, parent: &Element) -> Option<(Element, HtmlInputElement)> {
    let row = append(doc, parent, "div", "efb-web-omni")?;
    let raw_input = append(doc, &row, "input", "efb-web-input")?;
    attribute(&raw_input, "type", Some("text"));
    attribute(
        &raw_input,
        "placeholder",
        Some("Add ICAO, fix, or lat,lon (e.g. EFHF KREML ZMRS/15 or 55.749,37.625)"),
    );
    let input_el = raw_input.dyn_into::<HtmlInputElement>().ok()?;
    Some((row, input_el))
}

fn wire_omni_and_actions(
    state: &Rc<RefCell<WebEditorState>>,
    omni_row: &Element,
    actions_row: &Element,
) {
    add_btn(state, omni_row, "Add fix", "efb-web-btn primary", |st| {
        submit_input(st);
    });
    for (label, delta) in [("<- Earlier", -1i8), ("Later ->", 1i8)] {
        add_btn(state, actions_row, label, "efb-web-btn", move |st| {
            move_selected(st, delta);
        });
    }
    add_btn(state, actions_row, "Hold R/L", "efb-web-btn", cycle_hold);
    add_btn(state, actions_row, "Delete", "efb-web-btn", delete_selected);
    add_btn(state, actions_row, "Clear", "efb-web-btn", |st| {
        st.tokens.clear();
        st.selected = None;
        st.status_el.set_text_content(Some("Cleared route."));
    });
}

fn wire_presets_and_apply(state: &Rc<RefCell<WebEditorState>>, presets_row: &Element) {
    add_btn(
        state,
        presets_row,
        "Preset: Mathias Rust 1987 (EFHF -> ZMRS)",
        "efb-web-btn",
        |st| {
            st.tokens =
                EfbPresetId::MathiasRust1987Corridor.build_tokens(&mut st.next_id, &st.catalog);
            st.selected = st.tokens.len().checked_sub(1);
            st.status_el.set_text_content(Some(
                "Loaded Mathias Rust 1987 corridor (Helsinki-Malmi EFHF -> Red Square Bridge ZMRS).",
            ));
        },
    );
    add_btn(
        state,
        presets_row,
        "Preset: Delaware Corridor",
        "efb-web-btn",
        |st| {
            st.tokens = EfbPresetId::DelawareCorridor.build_tokens(&mut st.next_id, &st.catalog);
            st.selected = st.tokens.len().checked_sub(1);
            st.status_el
                .set_text_content(Some("Loaded Delaware visual corridor."));
        },
    );
    add_btn(
        state,
        presets_row,
        "Apply & Fly Link (#v=2)",
        "efb-web-btn primary",
        apply_and_navigate_link,
    );
    add_btn(
        state,
        presets_row,
        "Open Interactive 3D Cockpit + Bridge Autoland",
        "efb-web-btn emerald",
        |_| open_interactive_cockpit(),
    );
}

fn add_btn(
    state: &Rc<RefCell<WebEditorState>>,
    parent: &Element,
    label: &str,
    class: &str,
    mut handler: impl FnMut(&mut WebEditorState) + 'static,
) {
    let doc = state.borrow().document.clone();
    let Some(btn) = append(&doc, parent, "button", class) else {
        return;
    };
    attribute(&btn, "type", Some("button"));
    btn.set_text_content(Some(label));
    let weak = Rc::downgrade(state);
    let cb = Closure::<dyn FnMut(Event)>::new(move |_ev: Event| {
        if let Some(rc) = weak.upgrade() {
            {
                let mut guard = rc.borrow_mut();
                handler(&mut guard);
            }
            refresh_chips(&rc);
        }
    });
    btn.add_event_listener_with_callback("click", cb.as_ref().unchecked_ref())
        .ok();
    state.borrow_mut().closures.push(cb);
}

fn refresh_chips(state: &Rc<RefCell<WebEditorState>>) {
    let (doc, chips_box, items, selected) = {
        let guard = state.borrow();
        let snap: Vec<(String, ChipBadge, bool)> = guard
            .tokens
            .iter()
            .map(|t| (t.chip_label(), t.badge(), t.hold.is_some()))
            .collect();
        (
            guard.document.clone(),
            guard.chips_box.clone(),
            snap,
            guard.selected,
        )
    };
    chips_box.set_text_content(None);
    if items.is_empty() {
        if let Some(empty) = append(&doc, &chips_box, "span", "detail") {
            empty.set_text_content(Some("No route chips — add fixes above or load a preset."));
        }
        return;
    }
    for (idx, (label, badge, has_hold)) in items.into_iter().enumerate() {
        let tone = if has_hold {
            "hold"
        } else if badge == ChipBadge::AdHocRunway {
            "adhoc"
        } else {
            "std"
        };
        let cls = if selected == Some(idx) {
            format!("efb-web-chip {tone} selected")
        } else {
            format!("efb-web-chip {tone}")
        };
        let Some(chip) = append(&doc, &chips_box, "button", &cls) else {
            continue;
        };
        attribute(&chip, "type", Some("button"));
        chip.set_text_content(Some(&label));
        let weak = Rc::downgrade(state);
        let cb = Closure::<dyn FnMut(Event)>::new(move |_ev: Event| {
            if let Some(rc) = weak.upgrade() {
                rc.borrow_mut().selected = Some(idx);
                refresh_chips(&rc);
            }
        });
        chip.add_event_listener_with_callback("click", cb.as_ref().unchecked_ref())
            .ok();
        state.borrow_mut().closures.push(cb);
    }
}

fn submit_input(st: &mut WebEditorState) {
    let raw = st.input_el.value();
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return;
    }
    let mut added = 0usize;
    let mut insert_at = st
        .selected
        .map_or(st.tokens.len(), |s| s.wrapping_add(1).min(st.tokens.len()));
    for part in trimmed.split_whitespace() {
        let id = sim_maplibre::FixId(st.next_id);
        st.next_id = st.next_id.wrapping_add(1);
        if let Some(tok) = parse_single_token(part, id, &st.catalog) {
            st.tokens.insert(insert_at, tok);
            st.selected = Some(insert_at);
            insert_at = insert_at.wrapping_add(1);
            added = added.wrapping_add(1);
        }
    }
    if added > 0 {
        st.input_el.set_value("");
        st.status_el
            .set_text_content(Some(&format!("Added {added} fix(es).")));
    } else {
        st.status_el
            .set_text_content(Some(&format!("No airport or fix matching '{trimmed}'.")));
    }
}

fn move_selected(st: &mut WebEditorState, dir: i8) {
    let Some(sel) = st.selected else {
        return;
    };
    let dst = if dir < 0 {
        sel.checked_sub(1)
    } else {
        let next = sel.wrapping_add(1);
        (next < st.tokens.len()).then_some(next)
    };
    if let Some(d) = dst {
        st.tokens.swap(sel, d);
        st.selected = Some(d);
    }
}

fn cycle_hold(st: &mut WebEditorState) {
    if let Some(tok) = st.selected.and_then(|i| st.tokens.get_mut(i)) {
        tok.hold = match tok.hold.map(|h| h.turn) {
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
    }
}

fn delete_selected(st: &mut WebEditorState) {
    if let Some(sel) = st.selected
        && sel < st.tokens.len()
    {
        let _ = st.tokens.remove(sel);
        st.selected = st.tokens.len().checked_sub(1).map(|last| sel.min(last));
    }
}

fn apply_and_navigate_link(st: &mut WebEditorState) {
    match build_updated_fragment(&st.tokens, st.template.as_ref()) {
        Ok(fragment) => {
            if let Some(window) = web_sys::window() {
                let loc = window.location();
                if loc.set_hash(&fragment).is_ok() {
                    loc.reload().ok();
                }
            }
        }
        Err(err) => st.status_el.set_text_content(Some(&err)),
    }
}

fn open_interactive_cockpit() {
    if let Some(window) = web_sys::window() {
        let loc = window.location();
        if let Ok(pathname) = loc.pathname() {
            loc.set_href(&pathname).ok();
        }
    }
}

fn build_updated_fragment(
    tokens: &[RouteToken],
    template: Option<&SimulationLink>,
) -> Result<String, String> {
    if tokens.len() < 2 {
        return Err("Add at least 2 waypoints before applying link.".to_owned());
    }
    if let Some(base) = template {
        let mut updated = base.clone();
        updated.route = build_simulation_link_legs(tokens)?;
        return updated.fragment().map_err(|e| e.to_string());
    }
    crate::efb_plan::navlog::export_simulation_link_fragment(tokens, 750)
}
