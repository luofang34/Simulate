//! Popover drawers (`DIAL` and `···`) and portrait rotate reminder for `#touch-cockpit`.

use std::cell::RefCell;
use std::rc::Rc;

use web_sys::{Document, Element};

use super::dom::{add_click, create_button, create_el, wire_captured_pointer};
use super::tilt::TiltController;
use super::{ListenerHandle, PointerListenerHandle};
use crate::cockpit_rig::CockpitViewMode;
use crate::controls::touch::{TouchAction, TouchFlightState, TouchUiSnapshot};

/// Mutable references into the `DIAL` and `···` popovers synchronized from frame UI snapshots.
pub(super) struct PopoverUi {
    summary_el: Element,
    hdg_val_el: Element,
    alt_val_el: Element,
    baro_val_el: Element,
    view_btns: Vec<(CockpitViewMode, Element)>,
    yoke_btn: Element,
}

pub(super) struct PopoverBuild {
    pub dial_popover: Element,
    pub more_popover: Element,
    pub rotate_banner: Element,
    pub ui: PopoverUi,
}

struct MorePopoverParts {
    popover: Element,
    view_btns: Vec<(CockpitViewMode, Element)>,
    yoke_btn: Element,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn build_popovers_and_banner(
    doc: &Document,
    root: &Element,
    state: &Rc<RefCell<TouchFlightState>>,
    tilt: &Rc<RefCell<TiltController>>,
    tilt_btn: &Element,
    collapsed: &Rc<RefCell<bool>>,
    fly_btn: &Element,
    clicks: &mut Vec<ListenerHandle>,
    pointers: &mut Vec<PointerListenerHandle>,
) -> Option<PopoverBuild> {
    let rotate_banner = build_rotate_banner(doc, root, clicks)?;
    let (dial_popover, summary_el, hdg_val_el, alt_val_el, baro_val_el) =
        build_dial_popover(doc, state, clicks, pointers)?;
    let tilt_ctrl = Rc::clone(tilt);
    add_click(tilt_btn, clicks, move || TiltController::toggle(&tilt_ctrl));
    let MorePopoverParts {
        popover: more_popover,
        view_btns,
        yoke_btn,
    } = build_more_popover(doc, root, state, tilt, tilt_btn, collapsed, fly_btn, clicks)?;
    Some(PopoverBuild {
        dial_popover,
        more_popover,
        rotate_banner,
        ui: PopoverUi {
            summary_el,
            hdg_val_el,
            alt_val_el,
            baro_val_el,
            view_btns,
            yoke_btn,
        },
    })
}

pub(super) fn sync_popovers(ui: &PopoverUi, snap: &TouchUiSnapshot) {
    let hdg_str = snap
        .dial_readout
        .heading_bug_deg
        .map_or_else(|| "---".to_owned(), |d| format!("{d:03}\u{00B0}"));
    let alt_str = snap
        .dial_readout
        .altitude_bug_ft
        .map_or_else(|| "---- ft".to_owned(), |ft| format!("{ft} ft"));
    let baro_str = format!(
        "{:.0} hPa / {:.2}",
        snap.dial_readout.baro_hpa, snap.dial_readout.baro_inhg
    );
    ui.summary_el
        .set_text_content(Some(&format!("HDG {hdg_str} · ALT {alt_str} · {baro_str}")));
    ui.hdg_val_el.set_text_content(Some(&hdg_str));
    ui.alt_val_el.set_text_content(Some(&alt_str));
    ui.baro_val_el.set_text_content(Some(&baro_str));
    for (mode, btn) in &ui.view_btns {
        let active = if *mode == snap.view_mode {
            "true"
        } else {
            "false"
        };
        btn.set_attribute("data-active", active).ok();
    }
    ui.yoke_btn.set_text_content(Some(if snap.hides_yoke {
        "YOKE: HIDDEN"
    } else {
        "YOKE: SHOWN"
    }));
    ui.yoke_btn
        .set_attribute(
            "data-active",
            if snap.hides_yoke { "true" } else { "false" },
        )
        .ok();
}

fn build_rotate_banner(
    doc: &Document,
    root: &Element,
    clicks: &mut Vec<ListenerHandle>,
) -> Option<Element> {
    let banner = create_el(doc, "div", "", None)?;
    banner.set_id("tc-rotate-banner");
    banner.set_attribute("role", "status").ok()?;
    let msg = create_el(
        doc,
        "span",
        "",
        Some("Rotate phone to landscape for wide cockpit & EFIS view"),
    )?;
    let close = create_button(
        doc,
        "tc-rotate-close",
        "\u{00D7}",
        "Dismiss rotate reminder",
    )?;
    let root_clone = root.clone();
    add_click(&close, clicks, move || {
        root_clone
            .set_attribute("data-rotate-dismissed", "true")
            .ok();
    });
    banner.append_child(&msg).ok()?;
    banner.append_child(&close).ok()?;
    Some(banner)
}

fn build_dial_popover(
    doc: &Document,
    state: &Rc<RefCell<TouchFlightState>>,
    clicks: &mut Vec<ListenerHandle>,
    pointers: &mut Vec<PointerListenerHandle>,
) -> Option<(Element, Element, Element, Element, Element)> {
    let pop = create_el(doc, "div", "tc-popover", None)?;
    pop.set_id("tc-dial-popover");
    pop.set_attribute("popover", "auto").ok()?;
    let summary = create_el(doc, "div", "tc-dial-summary", Some("HDG --- · ALT ----"))?;
    pop.append_child(&summary).ok()?;

    let hdg_val = build_dial_group(
        doc,
        &pop,
        "HSI HEADING BUG",
        "tc-dial-row",
        "DRAG TO ROTATE HDG",
        &[
            ("-10", TouchAction::AdjustHeadingBug(-10)),
            ("-1", TouchAction::AdjustHeadingBug(-1)),
            ("SYNC", TouchAction::SyncHeadingBug),
            ("+1", TouchAction::AdjustHeadingBug(1)),
            ("+10", TouchAction::AdjustHeadingBug(10)),
        ],
        6.0,
        |steps| TouchAction::AdjustHeadingBug(steps as i16),
        state,
        clicks,
        pointers,
    )?;
    let alt_val = build_dial_group(
        doc,
        &pop,
        "ALT PRESELECT",
        "tc-dial-row",
        "DRAG TO DIAL ALT",
        &[
            ("-500", TouchAction::AdjustAltitudeBug(-500)),
            ("-100", TouchAction::AdjustAltitudeBug(-100)),
            ("SYNC", TouchAction::SyncAltitudeBug),
            ("+100", TouchAction::AdjustAltitudeBug(100)),
            ("+500", TouchAction::AdjustAltitudeBug(500)),
        ],
        8.0,
        |steps| TouchAction::AdjustAltitudeBug(steps * 100),
        state,
        clicks,
        pointers,
    )?;
    let baro_val = build_dial_group(
        doc,
        &pop,
        "BARO ALTIMETER",
        "tc-dial-row baro",
        "DRAG TO DIAL BARO",
        &[
            ("-1 hPa", TouchAction::AdjustBaroTenthsHpa(-10)),
            ("STD", TouchAction::SetBaroStd),
            ("+1 hPa", TouchAction::AdjustBaroTenthsHpa(10)),
        ],
        10.0,
        |steps| TouchAction::AdjustBaroTenthsHpa(steps as i16),
        state,
        clicks,
        pointers,
    )?;
    Some((pop, summary, hdg_val, alt_val, baro_val))
}

#[allow(clippy::too_many_arguments)]
fn build_dial_group<F>(
    doc: &Document,
    parent: &Element,
    title: &str,
    row_class: &str,
    scrub_hint: &str,
    buttons: &[(&str, TouchAction)],
    px_per_step: f64,
    step_action: F,
    state: &Rc<RefCell<TouchFlightState>>,
    clicks: &mut Vec<ListenerHandle>,
    pointers: &mut Vec<PointerListenerHandle>,
) -> Option<Element>
where
    F: Fn(i32) -> TouchAction + 'static,
{
    let group = create_el(doc, "div", "tc-dial-group", None)?;
    let head = create_el(doc, "div", "tc-dial-head", None)?;
    let lbl = create_el(doc, "span", "", Some(title))?;
    let val = create_el(doc, "span", "tc-dial-val", Some("---"))?;
    head.append_child(&lbl).ok()?;
    head.append_child(&val).ok()?;
    group.append_child(&head).ok()?;

    let row = create_el(doc, "div", row_class, None)?;
    for &(label, action) in buttons {
        let btn = create_button(doc, "tc-dial-btn", label, title)?;
        let st = Rc::clone(state);
        add_click(&btn, clicks, move || st.borrow_mut().actions.push(action));
        row.append_child(&btn).ok()?;
    }
    group.append_child(&row).ok()?;

    let scrub = create_el(doc, "div", "tc-scrub", Some(scrub_hint))?;
    scrub.set_attribute("aria-label", scrub_hint).ok()?;
    wire_rotary_scrub(&scrub, px_per_step, step_action, state, pointers);
    group.append_child(&scrub).ok()?;
    parent.append_child(&group).ok()?;
    Some(val)
}

fn wire_rotary_scrub<F>(
    el: &Element,
    px_per_step: f64,
    step_action: F,
    state: &Rc<RefCell<TouchFlightState>>,
    pointers: &mut Vec<PointerListenerHandle>,
) where
    F: Fn(i32) -> TouchAction + 'static,
{
    let anchor_x = Rc::new(RefCell::new(None::<f64>));
    let ax_move = Rc::clone(&anchor_x);
    let ax_end = Rc::clone(&anchor_x);
    let st = Rc::clone(state);
    wire_captured_pointer(
        el,
        pointers,
        move |ev| {
            let x = f64::from(ev.client_x());
            let mut slot = ax_move.borrow_mut();
            let prev = *slot.get_or_insert(x);
            let delta = x - prev;
            let steps = (delta / px_per_step).trunc() as i32;
            if steps != 0 {
                *slot = Some(prev + f64::from(steps) * px_per_step);
                st.borrow_mut().actions.push(step_action(steps));
            }
        },
        move || {
            *ax_end.borrow_mut() = None;
        },
    );
}

#[allow(clippy::too_many_arguments)]
fn build_more_popover(
    doc: &Document,
    root: &Element,
    state: &Rc<RefCell<TouchFlightState>>,
    tilt: &Rc<RefCell<TiltController>>,
    tilt_btn: &Element,
    collapsed: &Rc<RefCell<bool>>,
    fly_btn: &Element,
    clicks: &mut Vec<ListenerHandle>,
) -> Option<MorePopoverParts> {
    let pop = create_el(doc, "div", "tc-popover", None)?;
    pop.set_id("tc-more-popover");
    pop.set_attribute("popover", "auto").ok()?;
    let (views_sec, view_btns) = build_views_section(doc, state, clicks)?;
    let (cockpit_sec, yoke_btn) = build_cockpit_section(doc, state, tilt, tilt_btn, clicks)?;
    let flight_sec = build_systems_section(doc, root, collapsed, fly_btn, clicks)?;
    pop.append_child(&views_sec).ok()?;
    pop.append_child(&cockpit_sec).ok()?;
    pop.append_child(&flight_sec).ok()?;
    Some(MorePopoverParts {
        popover: pop,
        view_btns,
        yoke_btn,
    })
}

fn build_views_section(
    doc: &Document,
    state: &Rc<RefCell<TouchFlightState>>,
    clicks: &mut Vec<ListenerHandle>,
) -> Option<(Element, Vec<(CockpitViewMode, Element)>)> {
    let sec = create_el(doc, "div", "tc-more-section", None)?;
    let hdr = create_el(doc, "div", "tc-more-label", Some("DISPLAY VIEW"))?;
    sec.append_child(&hdr).ok()?;
    let grid = create_el(doc, "div", "tc-more-grid views", None)?;
    let mut view_btns = Vec::new();
    for mode in CockpitViewMode::ALL {
        let btn = create_button(doc, "tc-more-btn", mode.label(), mode.label())?;
        let st = Rc::clone(state);
        add_click(&btn, clicks, move || {
            st.borrow_mut().actions.push(TouchAction::SetViewMode(mode));
        });
        grid.append_child(&btn).ok()?;
        view_btns.push((mode, btn));
    }
    sec.append_child(&grid).ok()?;
    Some((sec, view_btns))
}

fn build_cockpit_section(
    doc: &Document,
    state: &Rc<RefCell<TouchFlightState>>,
    tilt: &Rc<RefCell<TiltController>>,
    tilt_btn: &Element,
    clicks: &mut Vec<ListenerHandle>,
) -> Option<(Element, Element)> {
    let sec = create_el(doc, "div", "tc-more-section", None)?;
    let hdr = create_el(doc, "div", "tc-more-label", Some("COCKPIT & SENSORS"))?;
    sec.append_child(&hdr).ok()?;
    let grid = create_el(doc, "div", "tc-more-grid", None)?;
    let yoke_btn = create_button(doc, "tc-more-btn", "YOKE: SHOWN", "Toggle 3D cockpit yoke")?;
    let st_yoke = Rc::clone(state);
    add_click(&yoke_btn, clicks, move || {
        st_yoke.borrow_mut().actions.push(TouchAction::ToggleYoke);
    });
    let tilt_el = tilt_btn.clone();
    let tilt_zero = Rc::clone(tilt);
    let st_zero = Rc::clone(state);
    let zero_btn = create_button(doc, "tc-more-btn", "ZERO", "Recenter view and tilt")?;
    add_click(&zero_btn, clicks, move || {
        tilt_zero.borrow_mut().recalibrate();
        st_zero.borrow_mut().actions.push(TouchAction::ResetLook);
    });
    for btn in [&yoke_btn, &tilt_el, &zero_btn] {
        grid.append_child(btn).ok()?;
    }
    sec.append_child(&grid).ok()?;
    Some((sec, yoke_btn))
}

fn build_systems_section(
    doc: &Document,
    root: &Element,
    collapsed: &Rc<RefCell<bool>>,
    fly_btn: &Element,
    clicks: &mut Vec<ListenerHandle>,
) -> Option<Element> {
    let sec = create_el(doc, "div", "tc-more-section", None)?;
    let hdr = create_el(doc, "div", "tc-more-label", Some("CONTROLS"))?;
    sec.append_child(&hdr).ok()?;
    let grid = create_el(doc, "div", "tc-more-grid", None)?;
    let hide_btn = create_button(doc, "tc-more-btn", "HIDE DECK", "Hide touch controls")?;
    let root_c = root.clone();
    let collapsed_c = Rc::clone(collapsed);
    add_click(&hide_btn, clicks, move || {
        *collapsed_c.borrow_mut() = true;
        root_c.set_attribute("data-collapsed", "true").ok();
    });
    let root_f = root.clone();
    let collapsed_f = Rc::clone(collapsed);
    add_click(fly_btn, clicks, move || {
        *collapsed_f.borrow_mut() = false;
        root_f.set_attribute("data-collapsed", "false").ok();
    });
    grid.append_child(&hide_btn).ok()?;
    sec.append_child(&grid).ok()?;
    Some(sec)
}
