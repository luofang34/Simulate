//! The sensor controls: one switch for each fault the visitor can give a simulated
//! sensor, with what the fault is and what every consumer of the sensor reports of it.
//!
//! A switch is a checkbox drawn as a button, read when the page draws like the
//! monitor's other controls. It says what the visitor asks for; the line beside it says
//! what the flight reports, which a link's own outages and spoofing change too.

use std::collections::BTreeSet;

use sim_link_host::{
    Monitor, SensorControl, SensorFault, SpoofDrag, gnss_check_lines, sensor_controls,
};
use sim_navigate::trial::SensorKind;
use wasm_bindgen::JsCast;
use web_sys::{Document, Element, HtmlInputElement};

use super::super::section::{append, write_lines};
use super::{inputs, values};

/// What heads the controls.
const HEAD: &str = "Sensor faults · each acts on the simulated device, so every navigation \
    set that uses it and the flight controller get the same signal";
/// The spoofer's name among the switches' `data-fault` values.
const SPOOF: &str = "spoof_gnss";

/// What the visitor asks of the sensors.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct FaultsAsked {
    /// Sensors asked to stop, by their trial names.
    pub stopped: BTreeSet<&'static str>,
    /// Whether the GNSS receiver is asked to be spoofed.
    pub spoof_gnss: bool,
}

/// The controls on the page.
pub struct SensorPanel {
    rows: Element,
    /// Each GNSS set's check of its receiver, as its navigator states it.
    checks: Element,
    checks_shown: Vec<sim_link_host::GnssCheckLine>,
    /// The faults in force when the flight starts: the link's open spans.
    on_at_start: FaultsAsked,
    built: Vec<SensorFault>,
    can_switch: bool,
    by_hand: SpoofDrag,
}

impl SensorPanel {
    /// The controls at the end of `parent`; a recorded run's cannot be switched.
    /// `by_hand` is the drag a spoofer switched on here applies, and `on_at_start`
    /// the faults the link leaves in force to the end of the flight, whose switches
    /// start on so the visitor can end them.
    pub fn under(
        document: &Document,
        parent: &Element,
        can_switch: bool,
        (by_hand, on_at_start): (SpoofDrag, FaultsAsked),
    ) -> Option<Self> {
        let block = append(document, parent, "div", "sensor-controls")?;
        append(document, &block, "p", "detail")?.set_text_content(Some(HEAD));
        let rows = append(document, &block, "div", "sensor-rows")?;
        Some(Self {
            rows,
            checks: append(document, &block, "dl", "lines gnss-checks")?,
            checks_shown: Vec::new(),
            on_at_start,
            built: Vec::new(),
            can_switch,
            by_hand,
        })
    }

    /// What the switches say now.
    pub fn asked(&self) -> FaultsAsked {
        let on: Vec<String> = inputs(&self.rows)
            .filter(HtmlInputElement::checked)
            .filter_map(|input| input.get_attribute("data-fault"))
            .collect();
        FaultsAsked {
            stopped: SensorKind::ALL
                .into_iter()
                .map(SensorKind::name)
                .filter(|name| on.iter().any(|fault| fault == name))
                .collect(),
            spoof_gnss: on.iter().any(|fault| fault == SPOOF),
        }
    }

    /// Shows the controls of `monitor`'s run and what each sensor's consumers report.
    pub fn show(&mut self, document: &Document, monitor: &Monitor) {
        let controls = sensor_controls(monitor, self.by_hand);
        let faults: Vec<SensorFault> = controls.iter().map(|control| control.fault).collect();
        if faults != self.built {
            self.rows.set_text_content(None);
            for control in &controls {
                let on = match control.fault {
                    SensorFault::Stop(sensor) => self.on_at_start.stopped.contains(sensor.name()),
                    SensorFault::SpoofGnss => self.on_at_start.spoof_gnss,
                };
                row(document, &self.rows, control, (self.can_switch, on));
            }
            self.built = faults;
        }
        for (control, value) in controls.iter().zip(values(&self.rows)) {
            let class = if control.active {
                "value active"
            } else {
                "value"
            };
            if value.class_name() != class {
                value.set_class_name(class);
            }
            if value.text_content().as_deref() != Some(control.state.as_str()) {
                value.set_text_content(Some(&control.state));
                value.set_attribute("title", &control.state).ok();
            }
        }
        let checks = gnss_check_lines(monitor);
        if checks != self.checks_shown {
            let lines: Vec<sim_link_host::PlanLine> = checks
                .iter()
                .map(|check| sim_link_host::PlanLine {
                    label: check.set.clone(),
                    text: check.words.clone(),
                })
                .collect();
            write_lines(document, &self.checks, &lines);
            let alert = checks.iter().any(|check| check.alert);
            self.checks.set_class_name(if alert {
                "lines gnss-checks alert"
            } else {
                "lines gnss-checks"
            });
            self.checks_shown = checks;
        }
    }
}

/// The fault's name among the switches' `data-fault` values.
const fn fault_name(fault: SensorFault) -> &'static str {
    match fault {
        SensorFault::Stop(sensor) => sensor.name(),
        SensorFault::SpoofGnss => SPOOF,
    }
}

fn row(
    document: &Document,
    rows: &Element,
    control: &SensorControl,
    (can_switch, on): (bool, bool),
) {
    let Some(label) = append(document, rows, "label", "fault-switch") else {
        return;
    };
    if let Some(input) = append(document, &label, "input", "") {
        input.set_attribute("type", "checkbox").ok();
        input
            .set_attribute("data-fault", fault_name(control.fault))
            .ok();
        if !can_switch {
            input.set_attribute("disabled", "").ok();
            input
                .set_attribute("title", "A recorded run cannot be changed.")
                .ok();
        }
        if let Ok(input) = input.dyn_into::<HtmlInputElement>() {
            input.set_checked(on);
        }
    }
    if let Some(text) = append(document, &label, "span", "") {
        text.set_text_content(Some(control.label));
    }
    append(document, rows, "span", "value");
}
