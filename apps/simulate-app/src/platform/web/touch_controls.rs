//! Responsive DOM `#touch-cockpit` overlay for mobile browsers (`pointer: coarse` / iPhone):
//! virtual flight stick, throttle slider, spring-centered rudder, hold-to-brake, action bar,
//! and optional gyro-tilt steering.

mod dom;
mod tilt;

use std::cell::RefCell;
use std::rc::Rc;

use sim_input::InputMapper;
use wasm_bindgen::JsCast;
use wasm_bindgen::closure::Closure;
use web_sys::{Element, EventTarget, HtmlElement, PointerEvent};

use crate::controls::ControlScheme;
use crate::controls::touch::{TouchAction, TouchFlightState};
use crate::flight::Flight;
use dom::{
    add_top_buttons, build_rudder, build_stick, build_throttle, create_button, create_el,
    position_puck, position_rudder, update_throttle_visual, wire_brake_button, wire_rudder_slider,
    wire_stick_pad, wire_throttle_slider, wire_tilt_zero_hide,
};
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
    menu_btn: Element,
    stick_puck: HtmlElement,
    throttle_fill: HtmlElement,
    throttle_label: Element,
    rudder_thumb: HtmlElement,
    brake_btn: Element,
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

        let state = Rc::new(RefCell::new(TouchFlightState::default()));
        let mut click_listeners = Vec::new();
        let top = create_el(&document, "div", "tc-top", None)?;
        let (tilt_btn, zero_btn, hide_btn, menu_btn) =
            add_top_buttons(&document, &top, &state, &mut click_listeners)?;
        let tilt = Rc::new(RefCell::new(TiltController::new(
            Rc::clone(&state),
            tilt_btn.clone(),
        )));
        let collapsed = Rc::new(RefCell::new(false));
        wire_tilt_zero_hide(
            &root,
            (&tilt_btn, &zero_btn, &hide_btn),
            &state,
            &tilt,
            &collapsed,
            &mut click_listeners,
        );

        let deck = create_el(&document, "div", "tc-deck", None)?;
        let left = create_el(&document, "div", "tc-left", None)?;
        let right = create_el(&document, "div", "tc-right", None)?;
        let (throttle, throttle_fill, throttle_label) = build_throttle(&document)?;
        let (rudder, rudder_thumb) = build_rudder(&document)?;
        let (stick, stick_puck) = build_stick(&document)?;
        let brake_btn = create_button(&document, "tc-btn tc-brake", "BRK", "Hold wheel brakes")?;
        left.append_child(&throttle).ok()?;
        left.append_child(&rudder).ok()?;
        right.append_child(&brake_btn).ok()?;
        right.append_child(&stick).ok()?;
        deck.append_child(&left).ok()?;
        deck.append_child(&right).ok()?;
        root.append_child(&top).ok()?;
        root.append_child(&deck).ok()?;
        body.append_child(&root).ok()?;

        let mut pointer_listeners = Vec::new();
        wire_stick_pad(&stick, &stick_puck, &state, &mut pointer_listeners);
        wire_throttle_slider(
            &throttle,
            &throttle_fill,
            &throttle_label,
            &state,
            &mut pointer_listeners,
        );
        wire_rudder_slider(&rudder, &rudder_thumb, &state, &mut pointer_listeners);
        wire_brake_button(&brake_btn, &state, &mut pointer_listeners);

        Some(Self {
            root,
            menu_btn,
            stick_puck,
            throttle_fill,
            throttle_label,
            rudder_thumb,
            brake_btn,
            state,
            tilt,
            collapsed,
            _pointer_listeners: pointer_listeners,
            _click_listeners: click_listeners,
        })
    }

    /// Synchronizes DOM attributes, throttle bar, and virtual stick puck (showing gyro tilt
    /// deflection when active).
    pub fn sync_ui(&self, hub_open: bool, throttle_norm: f64) {
        let open_attr = if hub_open { "true" } else { "false" };
        self.root.set_attribute("data-hub-open", open_attr).ok();
        self.menu_btn
            .set_text_content(Some(if hub_open { "CLOSE" } else { "MENU" }));
        let (stick_axes, rudder, brake) = {
            let s = self.state.borrow();
            (s.shown_stick_axes(), s.rudder_axis, s.brakes_held)
        };
        position_puck(&self.stick_puck, stick_axes);
        position_rudder(&self.rudder_thumb, rudder);
        update_throttle_visual(&self.throttle_fill, &self.throttle_label, throttle_norm);
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
