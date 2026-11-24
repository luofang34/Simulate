//! The tabs that fold and unfold what the window draws under the overlay: one on the
//! instrument column's edge, one where a folded column was, and one on the corner map.
//! The style sheet places each from the edges the window reports, so a tab sits on the
//! thing it folds wherever the layout puts that.

use std::cell::RefCell;
use std::rc::Rc;

use wasm_bindgen::JsCast;
use web_sys::{Document, Element, HtmlElement};

use super::ListenerHandle;
use super::dom::{add_click, create_button};
use crate::cockpit_rig::TouchAnchors;
use crate::controls::touch::{TouchAction, TouchFlightState};

/// Each tab's style class, face, spoken label and action.
const TABS: [(&str, &str, &str, TouchAction); 3] = [
    (
        "tc-rail-fold",
        "\u{2039}",
        "Hide the flight instruments",
        TouchAction::SetInstrumentsShown(false),
    ),
    (
        "tc-rail-unfold",
        "PFD \u{203A}",
        "Show the flight instruments",
        TouchAction::SetInstrumentsShown(true),
    ),
    (
        "tc-map-fold",
        "\u{00D7}",
        "Hide the moving map",
        TouchAction::CollapseMap,
    ),
];

/// Adds the tabs under `root`; the style sheet shows each only while it has something
/// to fold or unfold.
pub(super) fn mount(
    doc: &Document,
    root: &Element,
    state: &Rc<RefCell<TouchFlightState>>,
    clicks: &mut Vec<ListenerHandle>,
) -> Option<()> {
    for (class, face, label, action) in TABS {
        let tab = create_button(
            doc,
            &format!("tc-btn tc-tab tc-flight-only {class}"),
            face,
            label,
        )?;
        let st = Rc::clone(state);
        add_click(&tab, clicks, move || st.borrow_mut().actions.push(action));
        root.append_child(&tab).ok()?;
    }
    Some(())
}

/// Tells the style sheet on `root` where the column and the corner map are drawn, and
/// which of them are.
pub(super) fn sync(root: &Element, anchors: &TouchAnchors) {
    let flag = |on: bool| if on { "true" } else { "false" };
    let folded = anchors.rail.is_some() && !anchors.rail_shown;
    for (name, on) in [
        ("data-has-rail", anchors.rail_shown),
        ("data-rail-folded", folded),
        ("data-has-map", anchors.corner_map.is_some()),
    ] {
        root.set_attribute(name, flag(on)).ok();
    }
    let Some(style) = root.dyn_ref::<HtmlElement>().map(HtmlElement::style) else {
        return;
    };
    let [rail_left, rail_top, _] = anchors.rail.unwrap_or_default();
    let [map_left, map_bottom] = anchors.corner_map.unwrap_or([1.0, 0.0]);
    for (name, fraction) in [
        ("--tc-efis-left", anchors.rail_reach()),
        ("--tc-rail-left", rail_left),
        ("--tc-rail-top", rail_top),
        ("--tc-map-left", map_left),
        ("--tc-map-bottom", map_bottom),
    ] {
        style.set_property(name, &format!("{fraction:.4}")).ok();
    }
}
