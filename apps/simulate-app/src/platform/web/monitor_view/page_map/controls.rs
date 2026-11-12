//! The page map's buttons and its pointer and wheel listeners, each giving the app's map
//! an EFB command through the link.

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use wasm_bindgen::JsCast;
use wasm_bindgen::closure::Closure;
use web_sys::{
    AddEventListenerOptions, Document, Element, Event, HtmlCanvasElement, MouseEvent, PointerEvent,
    WheelEvent,
};

use super::super::super::section::append;
use super::basemap_command;
use crate::map_rig::{EfbCommand, EfbStatus};
use crate::page_map::PageMapLink;
use crate::page_map::gesture::{Gestures, PointerInput};

/// A listener the page keeps alive for as long as the map.
type Listener = Closure<dyn FnMut(Event)>;

/// The buttons over the map.
pub(super) struct Controls {
    north: Element,
    recenter: Element,
    _listeners: Vec<Listener>,
}

impl Controls {
    /// The buttons at the top right of `frame`.
    pub(super) fn under(document: &Document, frame: &Element, link: &PageMapLink) -> Option<Self> {
        let bar = append(document, frame, "div", "page-map-buttons")?;
        let mut listeners = Vec::new();
        let mut button = |label: &str, title: &str, command: Box<dyn Fn() -> EfbCommand>| {
            let element = append(document, &bar, "button", "")?;
            element.set_text_content(Some(label));
            element.set_attribute("type", "button").ok();
            element.set_attribute("title", title).ok();
            let link = link.clone();
            listeners.push(on(&element, "click", move |_| link.command(command())));
            Some(element)
        };
        let zoom_in = button("+", "Zoom in", Box::new(|| EfbCommand::ZoomBy(1.0)));
        let zoom_out = button("−", "Zoom out", Box::new(|| EfbCommand::ZoomBy(-1.0)));
        let north = button(
            "Track up",
            "Switch between track up and north up",
            Box::new(|| EfbCommand::ToggleNorthUp),
        )?;
        let recenter = button(
            "Recenter",
            "Follow the aircraft again",
            Box::new(|| EfbCommand::Recenter),
        )?;
        let satellite = Rc::new(Cell::new(false));
        let shown = Rc::clone(&satellite);
        let basemap = button(
            "Satellite",
            "Switch between the chart and satellite imagery",
            Box::new(move || {
                shown.set(!shown.get());
                basemap_command(shown.get())
            }),
        )?;
        zoom_in?;
        zoom_out?;
        let label = basemap.clone();
        listeners.push(on(&basemap, "click", move |_| {
            label.set_text_content(Some(if satellite.get() {
                "Chart"
            } else {
                "Satellite"
            }));
        }));
        Some(Self {
            north,
            recenter,
            _listeners: listeners,
        })
    }

    /// Shows what the map reports: which way is up, and whether it was panned away.
    pub(super) fn show(&self, status: Option<&EfbStatus>) {
        let Some(status) = status else {
            return;
        };
        let up = if status.north_up {
            "North up"
        } else if status.custom_bearing {
            "Rotated"
        } else {
            "Track up"
        };
        if self.north.text_content().as_deref() != Some(up) {
            self.north.set_text_content(Some(up));
        }
        if status.free_pan {
            self.recenter.remove_attribute("disabled").ok();
        } else {
            self.recenter.set_attribute("disabled", "").ok();
        }
    }
}

/// Listens on `canvas` for pointers and the wheel, turning them into commands on `link`.
pub(super) fn listen(
    canvas: &HtmlCanvasElement,
    link: &PageMapLink,
    gestures: &Rc<RefCell<Gestures>>,
) -> Vec<Listener> {
    let pointer = |kind: &'static str| {
        let (link, gestures, target) = (link.clone(), Rc::clone(gestures), canvas.clone());
        on(canvas.as_ref(), kind, move |event| {
            let canvas = &target;
            let Some(event) = event.dyn_ref::<PointerEvent>() else {
                return;
            };
            let (id, at) = (event.pointer_id(), offset(event, canvas));
            let input = match kind {
                "pointerdown" => {
                    canvas.set_pointer_capture(id).ok();
                    // The right button, or any with Shift, tilts and turns.
                    let tilts = event.button() == 2 || event.shift_key();
                    PointerInput::Down { id, at, tilts }
                }
                "pointermove" => PointerInput::Move { id, at },
                "pointerup" => PointerInput::Up { id, at },
                _ => PointerInput::Cancel { id },
            };
            apply(&link, &gestures, canvas, input);
        })
    };
    let mut listeners: Vec<Listener> = ["pointerdown", "pointermove", "pointerup", "pointercancel"]
        .into_iter()
        .map(pointer)
        .collect();
    let (link, gestures, target) = (link.clone(), Rc::clone(gestures), canvas.clone());
    let wheel = Closure::<dyn FnMut(Event)>::new(move |event: Event| {
        let Some(wheel) = event.dyn_ref::<WheelEvent>() else {
            return;
        };
        event.prevent_default();
        // Line and page deltas are rare; a line reads as a few dozen pixels.
        let delta_y = match wheel.delta_mode() {
            0 => wheel.delta_y(),
            1 => wheel.delta_y() * 40.0,
            _ => wheel.delta_y() * 400.0,
        };
        apply(&link, &gestures, &target, PointerInput::Wheel { delta_y });
    });
    let options = AddEventListenerOptions::new();
    options.set_passive(false);
    canvas
        .add_event_listener_with_callback_and_add_event_listener_options(
            "wheel",
            wheel.as_ref().unchecked_ref(),
            &options,
        )
        .ok();
    listeners.push(wheel);
    // The right button tilts the map, so it opens no menu over it.
    listeners.push(on(canvas.as_ref(), "contextmenu", |event| {
        event.prevent_default()
    }));
    listeners
}

fn apply(
    link: &PageMapLink,
    gestures: &RefCell<Gestures>,
    canvas: &HtmlCanvasElement,
    input: PointerInput,
) {
    let ratio = f64::from(canvas.width()) / f64::from(canvas.client_width().max(1));
    for command in gestures.borrow_mut().handle(input, ratio) {
        link.command(command);
    }
}

/// Where `event` is on `canvas`, CSS pixels from its top left.
fn offset(event: &MouseEvent, canvas: &HtmlCanvasElement) -> [f64; 2] {
    let rect = canvas.get_bounding_client_rect();
    [
        f64::from(event.client_x()) - rect.left(),
        f64::from(event.client_y()) - rect.top(),
    ]
}

/// Calls `handler` for each `kind` event on `target`.
fn on(target: &Element, kind: &str, handler: impl FnMut(Event) + 'static) -> Listener {
    let closure = Closure::<dyn FnMut(Event)>::new(handler);
    target
        .add_event_listener_with_callback(kind, closure.as_ref().unchecked_ref())
        .ok();
    closure
}
