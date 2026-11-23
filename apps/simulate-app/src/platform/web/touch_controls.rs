//! Responsive DOM `#touch-cockpit` overlay for mobile browsers (`pointer: coarse` / iPhone):
//! compact 4-pill top bar (`MAP | DIAL | ··· | MENU`), popover `DIAL` & display/system drawers,
//! portrait rotate reminder, virtual stick, throttle, rudder, wheel brakes, and optional gyro tilt.

mod dial_dom;
mod dom;
mod safe_area;
mod tilt;

use std::cell::RefCell;
use std::rc::Rc;

use sim_input::InputMapper;
use wasm_bindgen::JsCast;
use wasm_bindgen::closure::Closure;
use web_sys::{Element, EventTarget, HtmlElement, PointerEvent};

use crate::controls::ControlScheme;
use crate::controls::touch::{TouchAction, TouchFlightState, TouchUiSnapshot};
use crate::flight::Flight;
use dial_dom::{PopoverBuild, PopoverUi, build_popovers_and_banner, sync_popovers};
use dom::{
    add_top_buttons, build_rudder, build_stick, build_throttle, create_button, create_el,
    position_puck, position_rudder, update_throttle_visual, wire_brake_button, wire_rudder_slider,
    wire_stick_pad, wire_throttle_slider,
};
use safe_area::SafeAreaProbe;
use tilt::TiltController;

struct ListenerHandle {
    target: EventTarget,
    event_name: &'static str,
    closure: Closure<dyn FnMut(web_sys::Event)>,
}

impl Drop for ListenerHandle {
    fn drop(&mut self) {
        self.target
            .remove_event_listener_with_callback(
                self.event_name,
                self.closure.as_ref().unchecked_ref(),
            )
            .ok();
    }
}

struct PointerListenerHandle {
    target: EventTarget,
    event_name: &'static str,
    closure: Closure<dyn FnMut(PointerEvent)>,
}

impl Drop for PointerListenerHandle {
    fn drop(&mut self) {
        self.target
            .remove_event_listener_with_callback(
                self.event_name,
                self.closure.as_ref().unchecked_ref(),
            )
            .ok();
    }
}

/// Live touch cockpit mounted over the simulation canvas for handheld browsers.
pub struct TouchCockpit {
    root: Element,
    map_btn: Element,
    menu_btn: Element,
    stick_puck: HtmlElement,
    throttle_fill: HtmlElement,
    throttle_label: Element,
    rudder_thumb: HtmlElement,
    brake_btn: Element,
    popovers: PopoverUi,
    safe_area: SafeAreaProbe,
    state: Rc<RefCell<TouchFlightState>>,
    tilt: Rc<RefCell<TiltController>>,
    collapsed: Rc<RefCell<bool>>,
    _pointer_listeners: Vec<PointerListenerHandle>,
    _click_listeners: Vec<ListenerHandle>,
}

impl TouchCockpit {
    /// Builds and mounts `#touch-cockpit` into `document.body`. Returns `None` if DOM is unavailable.
    #[must_use]
    pub fn mount() -> Option<Self> {
        let document = web_sys::window()?.document()?;
        let body = document.body()?;
        let root = create_el(&document, "div", "tc-root", None)?;
        root.set_id("touch-cockpit");
        root.set_attribute("data-hub-open", "false").ok()?;
        root.set_attribute("data-collapsed", "false").ok()?;
        root.set_attribute("data-rotate-dismissed", "false").ok()?;
        root.set_attribute("data-has-rail", "false").ok()?;
        root.set_attribute("data-deck-idle", "false").ok()?;

        let state = Rc::new(RefCell::new(TouchFlightState::default()));
        let mut click_listeners = Vec::new();
        let mut pointer_listeners = Vec::new();
        let top_wrap = create_el(&document, "div", "tc-top-wrap", None)?;
        let top = create_el(&document, "div", "tc-top", None)?;
        let (map_btn, fly_btn, menu_btn) =
            add_top_buttons(&document, &top, &state, &mut click_listeners)?;
        let tilt_btn = create_button(
            &document,
            "tc-more-btn",
            "TILT",
            "Toggle gyro tilt steering",
        )?;
        let tilt = Rc::new(RefCell::new(TiltController::new(
            Rc::clone(&state),
            tilt_btn.clone(),
        )));
        let collapsed = Rc::new(RefCell::new(false));
        let PopoverBuild {
            dial_popover,
            more_popover,
            rotate_banner,
            ui: popovers,
        } = build_popovers_and_banner(
            &document,
            &root,
            &state,
            &tilt,
            &tilt_btn,
            &collapsed,
            &fly_btn,
            &mut click_listeners,
            &mut pointer_listeners,
        )?;
        top_wrap.append_child(&top).ok()?;
        top_wrap.append_child(&rotate_banner).ok()?;

        let (deck, stick_puck, throttle_fill, throttle_label, rudder_thumb, brake_btn) =
            mount_deck(&document, &state, &mut pointer_listeners)?;
        root.append_child(&top_wrap).ok()?;
        root.append_child(&dial_popover).ok()?;
        root.append_child(&more_popover).ok()?;
        root.append_child(&deck).ok()?;
        let safe_area = SafeAreaProbe::mount(&document, &root)?;
        body.append_child(&root).ok()?;

        Some(Self {
            root,
            map_btn,
            menu_btn,
            stick_puck,
            throttle_fill,
            throttle_label,
            rudder_thumb,
            brake_btn,
            popovers,
            safe_area,
            state,
            tilt,
            collapsed,
            _pointer_listeners: pointer_listeners,
            _click_listeners: click_listeners,
        })
    }

    /// Synchronizes DOM attributes, deck clearances, auto-hide fade, throttle bar, stick puck,
    /// and `DIAL` / `···` popover readouts from the active frame snapshot.
    pub fn sync_ui(&self, snap: &TouchUiSnapshot) {
        let open_attr = if snap.hub_open { "true" } else { "false" };
        self.root.set_attribute("data-hub-open", open_attr).ok();
        self.menu_btn
            .set_text_content(Some(if snap.hub_open { "CLOSE" } else { "MENU" }));
        self.map_btn.set_text_content(Some(snap.efb_label));
        self.map_btn
            .set_attribute("data-active", if snap.efb_open { "true" } else { "false" })
            .ok();
        let has_rail = snap.efis_left_fraction > 0.01;
        self.root
            .set_attribute("data-has-rail", if has_rail { "true" } else { "false" })
            .ok();
        if let Some(root_html) = self.root.dyn_ref::<HtmlElement>() {
            root_html
                .style()
                .set_property("--tc-efis-left", &format!("{:.4}", snap.efis_left_fraction))
                .ok();
        }
        let (stick_axes, rudder, brake, idle) = {
            let mut s = self.state.borrow_mut();
            let idle = s.touch_deck_idle(snap.now_s);
            (s.shown_stick_axes(), s.rudder_axis, s.brakes_held, idle)
        };
        self.root
            .set_attribute("data-deck-idle", if idle { "true" } else { "false" })
            .ok();
        position_puck(&self.stick_puck, stick_axes);
        position_rudder(&self.rudder_thumb, rudder);
        update_throttle_visual(
            &self.throttle_fill,
            &self.throttle_label,
            snap.throttle_norm,
        );
        self.brake_btn
            .set_attribute("data-active", if brake { "true" } else { "false" })
            .ok();
        self.root
            .set_attribute(
                "data-collapsed",
                if *self.collapsed.borrow() {
                    "true"
                } else {
                    "false"
                },
            )
            .ok();
        sync_popovers(&self.popovers, snap);
    }

    /// The window edges the system covers, when a fresh reading is due at `now_s`.
    #[must_use]
    pub fn poll_safe_area(&self, now_s: f64) -> Option<crate::cockpit_rig::SafeAreaInsets> {
        self.safe_area.poll(now_s)
    }

    /// Releases held touch axes/brakes cleanly (when opening the Flight Hub).
    pub fn release_all(&self, mapper: &mut InputMapper, flight: &mut Flight) {
        self.state.borrow_mut().release_flight_inputs();
        mapper.set(sim_input::Control::Roll, 0.0);
        mapper.set(sim_input::Control::Pitch, 0.0);
        mapper.set(sim_input::Control::Yaw, 0.0);
        flight.update_system_controls(|sys| sys.wheel_brake_norm = 0.0);
    }

    /// Writes active touch/gyro controls into `mapper` and drains queued UI actions.
    pub fn apply_to(
        &self,
        mapper: &mut InputMapper,
        controls: &mut ControlScheme,
        flight: &mut Flight,
        allow_flight: bool,
    ) -> Vec<TouchAction> {
        self.state
            .borrow_mut()
            .apply_to(mapper, controls, flight, allow_flight)
    }
}

impl Drop for TouchCockpit {
    fn drop(&mut self) {
        self.tilt.borrow_mut().disable();
        self.root.remove();
    }
}

type DeckBuild = (
    Element,
    HtmlElement,
    HtmlElement,
    Element,
    HtmlElement,
    Element,
);

fn mount_deck(
    document: &web_sys::Document,
    state: &Rc<RefCell<TouchFlightState>>,
    pointer_listeners: &mut Vec<PointerListenerHandle>,
) -> Option<DeckBuild> {
    let deck = create_el(document, "div", "tc-deck", None)?;
    let left = create_el(document, "div", "tc-left", None)?;
    let right = create_el(document, "div", "tc-right", None)?;
    let (throttle, throttle_fill, throttle_label) = build_throttle(document)?;
    let (rudder, rudder_thumb) = build_rudder(document)?;
    let (stick, stick_puck) = build_stick(document)?;
    let brake_btn = create_button(document, "tc-btn tc-brake", "BRK", "Hold wheel brakes")?;
    left.append_child(&throttle).ok()?;
    left.append_child(&rudder).ok()?;
    right.append_child(&brake_btn).ok()?;
    right.append_child(&stick).ok()?;
    deck.append_child(&left).ok()?;
    deck.append_child(&right).ok()?;

    wire_stick_pad(&stick, &stick_puck, state, pointer_listeners);
    wire_throttle_slider(
        &throttle,
        &throttle_fill,
        &throttle_label,
        state,
        pointer_listeners,
    );
    wire_rudder_slider(&rudder, &rudder_thumb, state, pointer_listeners);
    wire_brake_button(&brake_btn, state, pointer_listeners);
    Some((
        deck,
        stick_puck,
        throttle_fill,
        throttle_label,
        rudder_thumb,
        brake_btn,
    ))
}
