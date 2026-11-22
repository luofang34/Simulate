//! Mobile gyro-tilt bridge using `DeviceOrientationEvent`, including iOS 13+ Safari's
//! click-triggered `requestPermission()` prompt and orientation-aware axis mapping.

use std::cell::RefCell;
use std::rc::Rc;

use js_sys::{Function, Promise, Reflect};
use wasm_bindgen::closure::Closure;
use wasm_bindgen::{JsCast, JsValue};
use wasm_bindgen_futures::{JsFuture, spawn_local};
use web_sys::{DeviceOrientationEvent, Element, EventTarget};

use crate::controls::touch::{TouchFlightState, tilt_axes_for_orientation};

type OrientationListener = (EventTarget, Closure<dyn FnMut(DeviceOrientationEvent)>);

/// Tracks live `DeviceOrientationEvent` readings, neutral hold calibration, and the
/// attached `window` listener.
#[derive(Default)]
pub(super) struct TiltController {
    state: Rc<RefCell<TouchFlightState>>,
    neutral: Rc<RefCell<Option<(f64, f64)>>>,
    button: Option<Element>,
    listener: Option<OrientationListener>,
    generation: Rc<RefCell<u64>>,
}

impl TiltController {
    pub(super) fn new(state: Rc<RefCell<TouchFlightState>>, button: Element) -> Self {
        Self {
            state,
            neutral: Rc::new(RefCell::new(None)),
            button: Some(button),
            listener: None,
            generation: Rc::new(RefCell::new(0)),
        }
    }

    pub(super) fn recalibrate(&mut self) {
        *self.neutral.borrow_mut() = None;
    }

    pub(super) fn set_active_flag(&self, active: bool) {
        if let Some(btn) = &self.button {
            btn.set_attribute("data-active", if active { "true" } else { "false" })
                .ok();
        }
    }

    pub(super) fn disable(&mut self) {
        *self.generation.borrow_mut() = self.generation.borrow().wrapping_add(1);
        self.state.borrow_mut().tilt_enabled = false;
        self.state.borrow_mut().tilt_axes = [0.0, 0.0];
        *self.neutral.borrow_mut() = None;
        self.set_active_flag(false);
        if let Some((target, closure)) = self.listener.take() {
            target
                .remove_event_listener_with_callback(
                    "deviceorientation",
                    closure.as_ref().unchecked_ref(),
                )
                .ok();
        }
    }

    pub(super) fn attach_listener(&mut self) {
        if self.listener.is_some() {
            return;
        }
        let Some(window) = web_sys::window() else {
            return;
        };
        let target: EventTarget = window.into();
        let state = Rc::clone(&self.state);
        let neutral = Rc::clone(&self.neutral);
        let closure =
            Closure::<dyn FnMut(DeviceOrientationEvent)>::new(move |ev: DeviceOrientationEvent| {
                let (Some(beta), Some(gamma)) = (ev.beta(), ev.gamma()) else {
                    return;
                };
                if !(beta.is_finite() && gamma.is_finite()) {
                    return;
                }
                let (nb, ng) = {
                    let mut guard = neutral.borrow_mut();
                    *guard.get_or_insert((beta, gamma))
                };
                let axes = tilt_axes_for_orientation(beta, gamma, nb, ng, screen_angle_deg());
                state.borrow_mut().tilt_axes = axes;
            });
        if target
            .add_event_listener_with_callback("deviceorientation", closure.as_ref().unchecked_ref())
            .is_ok()
        {
            self.listener = Some((target, closure));
        }
    }

    /// Toggles gyro steering inside a user click gesture (invoking iOS `requestPermission()`
    /// synchronously when required).
    pub(super) fn toggle(controller: &Rc<RefCell<Self>>) {
        let currently_enabled = controller.borrow().state.borrow().tilt_enabled;
        if currently_enabled {
            controller.borrow_mut().disable();
            return;
        }
        let permission_promise = request_ios_orientation_permission();
        let epoch = {
            let ctrl = controller.borrow_mut();
            let next = ctrl.generation.borrow().wrapping_add(1);
            *ctrl.generation.borrow_mut() = next;
            next
        };
        match permission_promise {
            Some(promise) => {
                let weak = Rc::clone(controller);
                spawn_local(async move {
                    let granted = JsFuture::from(promise)
                        .await
                        .ok()
                        .and_then(|v| v.as_string())
                        .is_some_and(|s| s == "granted");
                    let mut ctrl = weak.borrow_mut();
                    if *ctrl.generation.borrow() != epoch {
                        return;
                    }
                    if granted {
                        enable_tilt(&mut ctrl);
                    } else {
                        ctrl.disable();
                    }
                });
            }
            None => enable_tilt(&mut controller.borrow_mut()),
        }
    }
}

impl Drop for TiltController {
    fn drop(&mut self) {
        self.disable();
    }
}

fn enable_tilt(ctrl: &mut TiltController) {
    ctrl.recalibrate();
    ctrl.state.borrow_mut().tilt_enabled = true;
    ctrl.set_active_flag(true);
    ctrl.attach_listener();
}

fn request_ios_orientation_permission() -> Option<Promise> {
    let global = js_sys::global();
    let ctor = Reflect::get(&global, &JsValue::from_str("DeviceOrientationEvent")).ok()?;
    if ctor.is_undefined() || ctor.is_null() {
        return None;
    }
    let req = Reflect::get(&ctor, &JsValue::from_str("requestPermission")).ok()?;
    let func = req.dyn_into::<Function>().ok()?;
    func.call0(&ctor).ok()?.dyn_into::<Promise>().ok()
}

fn screen_angle_deg() -> i32 {
    let Some(window) = web_sys::window() else {
        return 0;
    };
    if let Ok(screen) = window.screen()
        && let Ok(angle) = screen.orientation().angle()
    {
        return i32::from(angle);
    }
    Reflect::get(&window, &JsValue::from_str("orientation"))
        .ok()
        .and_then(|v| v.as_f64())
        .map_or(0, |a| a.round() as i32)
}
