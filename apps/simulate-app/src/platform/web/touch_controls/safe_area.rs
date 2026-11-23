//! Reads the window edges the browser says the system covers, from an element whose
//! padding the page's style sheet sets to the `safe-area-inset-*` environment values.

use std::cell::Cell;

use web_sys::{Document, Element};

use super::dom::create_el;
use crate::cockpit_rig::SafeAreaInsets;

/// Seconds between readings: the insets change only when the device turns.
const READ_INTERVAL_S: f64 = 0.5;

/// The element that carries the insets, and when it was last read.
pub(super) struct SafeAreaProbe {
    element: Element,
    read_at_s: Cell<Option<f64>>,
}

impl SafeAreaProbe {
    /// Adds the probe under `root`.
    pub(super) fn mount(doc: &Document, root: &Element) -> Option<Self> {
        let element = create_el(doc, "div", "tc-safe-probe", None)?;
        element.set_attribute("aria-hidden", "true").ok()?;
        root.append_child(&element).ok()?;
        Some(Self {
            element,
            read_at_s: Cell::new(None),
        })
    }

    /// The insets now, when a reading is due at `now_s`.
    pub(super) fn poll(&self, now_s: f64) -> Option<SafeAreaInsets> {
        if self
            .read_at_s
            .get()
            .is_some_and(|at| (now_s - at).abs() < READ_INTERVAL_S)
        {
            return None;
        }
        self.read_at_s.set(Some(now_s));
        let window = web_sys::window()?;
        let style = window.get_computed_style(&self.element).ok().flatten()?;
        let px = |name: &str| {
            style
                .get_property_value(name)
                .ok()
                .and_then(|value| value.trim_end_matches("px").trim().parse::<f64>().ok())
                .unwrap_or(0.0)
        };
        let width = window.inner_width().ok().and_then(|v| v.as_f64())?;
        let height = window.inner_height().ok().and_then(|v| v.as_f64())?;
        Some(SafeAreaInsets::from_lengths(
            [
                px("padding-left"),
                px("padding-top"),
                px("padding-right"),
                px("padding-bottom"),
            ],
            [width, height],
        ))
    }
}
