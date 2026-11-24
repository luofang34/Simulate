//! DOM builders and captured-pointer wiring for the mobile `#touch-cockpit` overlay.

use std::cell::RefCell;
use std::rc::Rc;

use wasm_bindgen::JsCast;
use wasm_bindgen::closure::Closure;
use web_sys::{Document, Element, EventTarget, HtmlElement, PointerEvent};

use super::{ListenerHandle, PointerListenerHandle};
use crate::controls::touch::{
    TouchAction, TouchFlightState, map_rudder_track, map_stick_pad, map_throttle_track,
};

pub(super) fn browser_now_s() -> f64 {
    web_sys::window()
        .and_then(|w| w.performance())
        .map_or(0.0, |p| p.now() / 1000.0)
}

pub(super) fn create_el(
    doc: &Document,
    tag: &str,
    class: &str,
    text: Option<&str>,
) -> Option<Element> {
    let el = doc.create_element(tag).ok()?;
    el.set_class_name(class);
    if let Some(txt) = text {
        el.set_text_content(Some(txt));
    }
    Some(el)
}

pub(super) fn create_button(
    doc: &Document,
    class: &str,
    text: &str,
    label: &str,
) -> Option<Element> {
    let btn = create_el(doc, "button", class, Some(text))?;
    btn.set_attribute("type", "button").ok()?;
    btn.set_attribute("aria-label", label).ok()?;
    Some(btn)
}

/// Adds the compact top-right bar (`MAP`, `DIAL`, `···`, pause, `MENU`) plus the hidden
/// `FLY` unhide pill when the control deck is collapsed. Returns `(map_btn, fly_btn, menu_btn)`.
pub(super) fn add_top_buttons(
    doc: &Document,
    top: &Element,
    state: &Rc<RefCell<TouchFlightState>>,
    listeners: &mut Vec<ListenerHandle>,
) -> Option<(Element, Element, Element)> {
    let map_btn = create_button(
        doc,
        "tc-btn tc-flight-only",
        "MAP",
        "Show the moving map, then the EFB tablet, then neither",
    )?;
    let st_map = Rc::clone(state);
    add_click(&map_btn, listeners, move || {
        st_map.borrow_mut().actions.push(TouchAction::CycleEfb);
    });

    let dial_btn = create_button(
        doc,
        "tc-btn tc-flight-only tc-anchor-dial",
        "DIAL",
        "Dial heading bug, altitude preselect, and baro QNH",
    )?;
    dial_btn
        .set_attribute("popovertarget", "tc-dial-popover")
        .ok()?;
    add_click(&dial_btn, listeners, || {});

    let more_btn = create_button(
        doc,
        "tc-btn tc-flight-only tc-anchor-more",
        "\u{00B7}\u{00B7}\u{00B7}",
        "Open display and flight controls drawer",
    )?;
    more_btn
        .set_attribute("popovertarget", "tc-more-popover")
        .ok()?;
    add_click(&more_btn, listeners, || {});

    let pause_btn = create_button(doc, "tc-btn tc-flight-only", "II", "Pause or resume")?;
    let st_pause = Rc::clone(state);
    add_click(&pause_btn, listeners, move || {
        st_pause.borrow_mut().actions.push(TouchAction::TogglePause);
    });

    let fly_btn = create_button(
        doc,
        "tc-btn tc-collapsed-only",
        "FLY",
        "Show touch controls",
    )?;
    let menu_btn = create_button(doc, "tc-btn", "MENU", "Open or close Flight Hub")?;
    let st_menu = Rc::clone(state);
    add_click(&menu_btn, listeners, move || {
        st_menu.borrow_mut().actions.push(TouchAction::ToggleHub);
    });
    for el in [
        &map_btn, &dial_btn, &more_btn, &pause_btn, &fly_btn, &menu_btn,
    ] {
        top.append_child(el).ok()?;
    }
    Some((map_btn, fly_btn, menu_btn))
}

pub(super) fn build_throttle(doc: &Document) -> Option<(Element, HtmlElement, Element)> {
    let track = create_el(doc, "div", "tc-throttle", None)?;
    track.set_attribute("aria-label", "Throttle slider").ok()?;
    let fill = create_el(doc, "div", "tc-throttle-fill", None)?
        .dyn_into::<HtmlElement>()
        .ok()?;
    let label = create_el(doc, "span", "tc-throttle-label", Some("THR"))?;
    track.append_child(&fill).ok()?;
    track.append_child(&label).ok()?;
    Some((track, fill, label))
}

pub(super) fn build_rudder(doc: &Document) -> Option<(Element, HtmlElement)> {
    let track = create_el(doc, "div", "tc-rudder", None)?;
    track.set_attribute("aria-label", "Rudder slider").ok()?;
    let thumb = create_el(doc, "div", "tc-rudder-thumb", None)?
        .dyn_into::<HtmlElement>()
        .ok()?;
    track.append_child(&thumb).ok()?;
    Some((track, thumb))
}

pub(super) fn build_stick(doc: &Document) -> Option<(Element, HtmlElement)> {
    let pad = create_el(doc, "div", "tc-stick", None)?;
    pad.set_attribute("aria-label", "Virtual flight stick")
        .ok()?;
    let puck = create_el(doc, "div", "tc-stick-puck", None)?
        .dyn_into::<HtmlElement>()
        .ok()?;
    pad.append_child(&puck).ok()?;
    Some((pad, puck))
}

pub(super) fn add_click<F: FnMut() + 'static>(
    el: &Element,
    list: &mut Vec<ListenerHandle>,
    mut cb: F,
) {
    let target: EventTarget = el.clone().into();
    let closure = Closure::<dyn FnMut(web_sys::Event)>::new(move |ev: web_sys::Event| {
        ev.stop_propagation();
        cb();
    });
    if target
        .add_event_listener_with_callback("click", closure.as_ref().unchecked_ref())
        .is_ok()
    {
        list.push(ListenerHandle {
            target,
            event_name: "click",
            closure,
        });
    }
}

fn add_pointer<F: FnMut(PointerEvent) + 'static>(
    el: &Element,
    event_name: &'static str,
    list: &mut Vec<PointerListenerHandle>,
    cb: F,
) {
    let target: EventTarget = el.clone().into();
    let closure = Closure::<dyn FnMut(PointerEvent)>::new(cb);
    if target
        .add_event_listener_with_callback(event_name, closure.as_ref().unchecked_ref())
        .is_ok()
    {
        list.push(PointerListenerHandle {
            target,
            event_name,
            closure,
        });
    }
}

pub(super) fn wire_captured_pointer<U, E>(
    el: &Element,
    list: &mut Vec<PointerListenerHandle>,
    on_move: U,
    on_end: E,
) where
    U: Fn(&PointerEvent) + 'static,
    E: Fn() + 'static,
{
    let captured = Rc::new(RefCell::new(None::<i32>));
    let on_move = Rc::new(on_move);
    let on_end = Rc::new(on_end);
    let cap_down = Rc::clone(&captured);
    let el_down = el.clone();
    let move_down = Rc::clone(&on_move);
    add_pointer(el, "pointerdown", list, move |ev| {
        ev.prevent_default();
        ev.stop_propagation();
        if cap_down.borrow().is_none() {
            *cap_down.borrow_mut() = Some(ev.pointer_id());
            el_down.set_pointer_capture(ev.pointer_id()).ok();
            move_down(&ev);
        }
    });
    let cap_move = Rc::clone(&captured);
    add_pointer(el, "pointermove", list, move |ev| {
        if *cap_move.borrow() == Some(ev.pointer_id()) {
            ev.prevent_default();
            on_move(&ev);
        }
    });
    for end_ev in ["pointerup", "pointercancel"] {
        let cap_end = Rc::clone(&captured);
        let end_cb = Rc::clone(&on_end);
        add_pointer(el, end_ev, list, move |ev| {
            if *cap_end.borrow() == Some(ev.pointer_id()) {
                *cap_end.borrow_mut() = None;
                end_cb();
            }
        });
    }
}

pub(super) fn wire_stick_pad(
    pad: &Element,
    puck: &HtmlElement,
    state: &Rc<RefCell<TouchFlightState>>,
    list: &mut Vec<PointerListenerHandle>,
) {
    let pad_el = pad.clone();
    let puck_move = puck.clone();
    let st_move = Rc::clone(state);
    let st_end = Rc::clone(state);
    let puck_end = puck.clone();
    wire_captured_pointer(
        pad,
        list,
        move |ev| {
            let rect = pad_el.get_bounding_client_rect();
            let radius = (rect.width().min(rect.height()) * 0.42).max(16.0);
            let dx = f64::from(ev.client_x()) - (rect.left() + rect.width() * 0.5);
            let dy = f64::from(ev.client_y()) - (rect.top() + rect.height() * 0.5);
            let axes = map_stick_pad(dx, dy, radius);
            {
                let mut s = st_move.borrow_mut();
                s.stick_active = true;
                s.stick_axes = axes;
                s.note_deck_interaction(browser_now_s());
            }
            position_puck(&puck_move, axes);
        },
        move || {
            let mut s = st_end.borrow_mut();
            s.stick_active = false;
            s.stick_axes = [0.0, 0.0];
            s.note_deck_interaction(browser_now_s());
            position_puck(&puck_end, s.shown_stick_axes());
        },
    );
}

pub(super) fn wire_throttle_slider(
    track: &Element,
    fill: &HtmlElement,
    label: &Element,
    state: &Rc<RefCell<TouchFlightState>>,
    list: &mut Vec<PointerListenerHandle>,
) {
    let track_el = track.clone();
    let fill = fill.clone();
    let label = label.clone();
    let st = Rc::clone(state);
    let st_end = Rc::clone(state);
    wire_captured_pointer(
        track,
        list,
        move |ev| {
            let rect = track_el.get_bounding_client_rect();
            let t = map_throttle_track(f64::from(ev.client_y()) - rect.top(), rect.height());
            let mut s = st.borrow_mut();
            s.throttle_value = Some(t);
            s.note_deck_interaction(browser_now_s());
            update_throttle_visual(&fill, &label, t);
        },
        move || {
            st_end.borrow_mut().note_deck_interaction(browser_now_s());
        },
    );
}

pub(super) fn wire_rudder_slider(
    track: &Element,
    thumb: &HtmlElement,
    state: &Rc<RefCell<TouchFlightState>>,
    list: &mut Vec<PointerListenerHandle>,
) {
    let track_el = track.clone();
    let thumb_move = thumb.clone();
    let st_move = Rc::clone(state);
    let st_end = Rc::clone(state);
    let thumb_end = thumb.clone();
    wire_captured_pointer(
        track,
        list,
        move |ev| {
            let rect = track_el.get_bounding_client_rect();
            let r = map_rudder_track(f64::from(ev.client_x()) - rect.left(), rect.width());
            {
                let mut s = st_move.borrow_mut();
                s.rudder_active = true;
                s.rudder_axis = r;
                s.note_deck_interaction(browser_now_s());
            }
            position_rudder(&thumb_move, r);
        },
        move || {
            let mut s = st_end.borrow_mut();
            s.rudder_active = false;
            s.rudder_axis = 0.0;
            s.note_deck_interaction(browser_now_s());
            position_rudder(&thumb_end, 0.0);
        },
    );
}

pub(super) fn wire_brake_button(
    btn: &Element,
    state: &Rc<RefCell<TouchFlightState>>,
    list: &mut Vec<PointerListenerHandle>,
) {
    let btn_down = btn.clone();
    let st_down = Rc::clone(state);
    let st_end = Rc::clone(state);
    let btn_end = btn.clone();
    wire_captured_pointer(
        btn,
        list,
        move |_| {
            let mut s = st_down.borrow_mut();
            s.brakes_held = true;
            s.note_deck_interaction(browser_now_s());
            btn_down.set_attribute("data-active", "true").ok();
        },
        move || {
            let mut s = st_end.borrow_mut();
            s.brakes_held = false;
            s.note_deck_interaction(browser_now_s());
            btn_end.set_attribute("data-active", "false").ok();
        },
    );
}

pub(super) fn position_puck(puck: &HtmlElement, [roll, pitch]: [f64; 2]) {
    let x_pct = 50.0 + roll.clamp(-1.0, 1.0) * 34.0;
    let y_pct = 50.0 + pitch.clamp(-1.0, 1.0) * 34.0;
    let style = puck.style();
    style.set_property("left", &format!("{x_pct:.1}%")).ok();
    style.set_property("top", &format!("{y_pct:.1}%")).ok();
}

pub(super) fn position_rudder(thumb: &HtmlElement, yaw: f64) {
    let x_pct = 50.0 + yaw.clamp(-1.0, 1.0) * 36.0;
    thumb
        .style()
        .set_property("left", &format!("{x_pct:.1}%"))
        .ok();
}

pub(super) fn update_throttle_visual(fill: &HtmlElement, label: &Element, throttle_norm: f64) {
    let pct = (throttle_norm.clamp(0.0, 1.0) * 100.0).round();
    fill.style()
        .set_property("height", &format!("{pct:.0}%"))
        .ok();
    label.set_text_content(Some(&format!("{pct:.0}%")));
}
