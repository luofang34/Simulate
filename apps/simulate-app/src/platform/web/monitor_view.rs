//! The flight monitor on the page: the status line, the error plot, a
//! legend that shows or hides each set, a switch for each means the run
//! uses, the track map, the terrain camera's latest frame and what the visual
//! navigator matched it against, the pace, the
//! link of the run, the terrain camera's tallies, the mission host's
//! reports and the latest events.
//!
//! The controls are read when the page draws, not listened to: the flight
//! loop runs every animation frame anyway, so a click shows within a frame.

mod frame_panel;
mod match_panel;
mod page_map;
mod plot;
mod track_map;

use std::collections::BTreeSet;

use sim_link_host::{
    CameraWorld, MeansSwitch, Monitor, PlanLine, TracePoint, clock_words, event_words,
    sensor_words, state_words, status_words, terrain_words,
};
use sim_navigate::NavMeans;
use sim_navigate::trial::SensorKind;
use wasm_bindgen::JsCast;
use web_sys::{Document, Element, HtmlInputElement};

use super::section::{Section, append, write_lines};
use frame_panel::FramePanel;
use match_panel::MatchPanel;
use page_map::PageMap;
pub use page_map::PageMapSetup;
use plot::Plot;
use track_map::TrackMap;

/// The colours of the sets, in the run's order, for the page's dark
/// surface. None is magenta, which the map keeps for the active leg, nor the
/// map's pale yellow true track.
const SERIES: [&str; 8] = [
    "#3987e5", "#d95926", "#199e70", "#c98500", "#2bb5c9", "#008300", "#9085e9", "#e66767",
];
/// The colour of a set past the eighth, which gets no hue of its own.
const SERIES_OTHER: &str = "#8f98ad";

/// The paces offered, route seconds per wall second; `None` is as fast as
/// the page can step.
const SPEEDS: [(&str, Option<f64>); 5] = [
    ("1×", Some(1.0)),
    ("4×", Some(4.0)),
    ("16×", Some(16.0)),
    ("64×", Some(64.0)),
    ("as fast as possible", None),
];

/// The colour of the set at `index` in the run's order.
pub fn series_colour(index: usize) -> &'static str {
    SERIES.get(index).copied().unwrap_or(SERIES_OTHER)
}

/// What the visitor has set on the controls.
#[derive(Debug, Clone, PartialEq)]
pub struct Controls {
    /// Sets whose line is hidden.
    pub hidden: BTreeSet<String>,
    /// Sensors the visitor asks to withhold.
    pub withheld: BTreeSet<&'static str>,
    /// The pace.
    pub speed: Option<f64>,
}

/// The monitor's part of the page.
pub struct MonitorView {
    section: Section,
    plot: Plot,
    map: TrackMap,
    page_map: Option<PageMap>,
    frame: FramePanel,
    matching: MatchPanel,
    legend: Element,
    switches: Element,
    speeds: Element,
    terrain: Element,
    mission: Element,
    events: Element,
    run_link: Element,
    built_for: Vec<String>,
    can_switch: bool,
    world_note: Option<String>,
}

impl MonitorView {
    /// The monitor in `section`, starting at `speed`. `can_switch` says
    /// whether the flight takes the visitor's switches; a recorded run
    /// cannot. `caveat` stands under the sets' results, `world_note` beside
    /// the terrain camera's, and `about` closes the section. The track map
    /// lies over `world` when there is one.
    pub fn inside(
        section: Section,
        speed: Option<f64>,
        can_switch: bool,
        caveat: Option<&str>,
        (world, world_note): (Option<CameraWorld>, Option<String>),
        about: &str,
        page_map: Option<PageMapSetup>,
    ) -> Option<Self> {
        let document = section.document().clone();
        let root = section.root().clone();
        let plot = Plot::under(&document, &root)?;
        let legend = append(&document, &root, "div", "legend")?;
        if let Some(caveat) = caveat
            && let Some(line) = append(&document, &root, "p", "detail")
        {
            line.set_text_content(Some(caveat));
        }
        let page_map = page_map.and_then(|setup| PageMap::under(&document, &root, setup));
        let map = TrackMap::under(&document, &root, world)?;
        let frame = FramePanel::under(&document, &root)?;
        let matching = MatchPanel::under(&document, &root)?;
        let switches = append(&document, &root, "div", "switches")?;
        let speeds = append(&document, &root, "div", "speeds")?;
        speed_choices(&document, &speeds, speed);
        let run_link = append(&document, &root, "p", "detail")?;
        let terrain = append(&document, &root, "dl", "lines")?;
        let mission = append(&document, &root, "dl", "lines")?;
        let events = append(&document, &root, "dl", "lines")?;
        if let Some(closing) = append(&document, &root, "p", "detail") {
            closing.set_text_content(Some(about));
        }
        Some(Self {
            section,
            plot,
            map,
            page_map,
            frame,
            matching,
            legend,
            switches,
            speeds,
            terrain,
            mission,
            events,
            run_link,
            built_for: Vec::new(),
            can_switch,
            world_note,
        })
    }

    /// Offers `url`, the link of the run as flown so far, to open or copy;
    /// the page's own address stays as it is, since a changed address
    /// starts the page again.
    pub fn show_run_link(&self, url: Option<&str>) {
        self.run_link.set_text_content(None);
        let Some(url) = url else {
            return;
        };
        let document = self.section.document();
        if let Some(words) = append(document, &self.run_link, "span", "") {
            words.set_text_content(Some("Link to this run, with the switches so far: "));
        }
        if let Some(anchor) = append(document, &self.run_link, "a", "") {
            anchor.set_attribute("href", url).ok();
            anchor.set_attribute("target", "_blank").ok();
            anchor.set_attribute("rel", "noopener").ok();
            anchor.set_text_content(Some(url));
        }
    }

    /// The section the monitor is in.
    pub const fn section(&self) -> &Section {
        &self.section
    }

    /// What the controls say now.
    pub fn controls(&self) -> Controls {
        let hidden = inputs(&self.legend)
            .filter(|input| !input.checked())
            .filter_map(|input| input.get_attribute("data-set"))
            .collect();
        let withheld = inputs(&self.switches)
            .filter(HtmlInputElement::checked)
            .filter_map(|input| input.get_attribute("data-sensor"))
            .filter_map(|name| SensorKind::ALL.into_iter().find(|kind| kind.name() == name))
            .map(|kind| kind.name())
            .collect();
        let speed = inputs(&self.speeds)
            .find(HtmlInputElement::checked)
            .and_then(|input| input.get_attribute("data-index"))
            .and_then(|index| index.parse::<usize>().ok())
            .and_then(|index| SPEEDS.get(index))
            .map_or(Some(1.0), |(_, speed)| *speed);
        Controls {
            hidden,
            withheld,
            speed,
        }
    }

    /// Shows `monitor` under `controls`; `note` heads the status line.
    pub fn draw(&mut self, monitor: &Monitor, controls: &Controls, note: &str) {
        self.rebuild_if_new(monitor);
        self.section
            .settled(&format!("{note}{}", status_words(monitor)));
        self.plot
            .draw(monitor, &|set| !controls.hidden.contains(set));
        self.legend_values(monitor);
        self.draw_map(monitor, &controls.hidden);
        self.frame.draw(monitor);
        self.matching.draw(monitor);
        self.switch_states(monitor);
        let mut visual: Vec<PlanLine> = monitor
            .traces()
            .iter()
            .filter(|trace| trace.means.contains(&NavMeans::Visual))
            .map(|trace| PlanLine {
                label: format!("Terrain camera, {}", trace.name),
                text: terrain_words(monitor, &trace.name),
            })
            .collect();
        if let Some(note) = self.world_note.as_ref().filter(|_| !visual.is_empty()) {
            visual.insert(
                0,
                PlanLine {
                    label: "Camera's world".to_owned(),
                    text: note.clone(),
                },
            );
        }
        write_lines(self.section.document(), &self.terrain, &visual);
        let mission: Vec<PlanLine> = monitor
            .mission_events()
            .iter()
            .rev()
            .map(|(t_s, event)| PlanLine {
                label: format!("Mission {}", clock_words(*t_s)),
                text: event.clone(),
            })
            .collect();
        write_lines(self.section.document(), &self.mission, &mission);
        let events: Vec<PlanLine> = monitor
            .events()
            .iter()
            .rev()
            .map(|event| PlanLine {
                label: clock_words(event.t_s),
                text: event_words(event),
            })
            .collect();
        write_lines(self.section.document(), &self.events, &events);
    }

    /// The app's map once it draws, the page's own drawing until then.
    fn draw_map(&mut self, monitor: &Monitor, hidden: &BTreeSet<String>) {
        if let Some(page_map) = self.page_map.as_mut() {
            page_map.draw(monitor, hidden);
            if page_map.live() {
                self.map.retire();
                return;
            }
        }
        self.map.draw(monitor, hidden);
    }

    /// Builds the legend and the switches when the run's sets change.
    fn rebuild_if_new(&mut self, monitor: &Monitor) {
        let names: Vec<String> = monitor.traces().iter().map(|t| t.name.clone()).collect();
        if names == self.built_for {
            return;
        }
        let document = self.section.document().clone();
        self.legend.set_text_content(None);
        for (index, name) in names.iter().enumerate() {
            legend_row(&document, &self.legend, name, series_colour(index));
        }
        self.switches.set_text_content(None);
        for switch in monitor.switches() {
            switch_row(&document, &self.switches, switch, self.can_switch);
        }
        self.built_for = names;
    }

    fn legend_values(&self, monitor: &Monitor) {
        let ends = monitor.route_end().map(|end| end.samples.as_slice());
        for (index, (trace, value)) in monitor
            .traces()
            .iter()
            .zip(values(&self.legend))
            .enumerate()
        {
            let now = trace.latest().map(sample_words);
            let at_end = ends
                .and_then(|samples| samples.get(index).copied().flatten())
                .map(|sample| sample_words(&sample));
            let text = match (at_end, now) {
                (Some(end), Some(now)) => format!("at the route's end {end}; now {now}"),
                (Some(end), None) => format!("at the route's end {end}"),
                (None, now) => now.unwrap_or_default(),
            };
            value.set_text_content(Some(&text));
        }
    }

    /// Each switch's sensor as the stream reports it, per set that has it.
    fn switch_states(&self, monitor: &Monitor) {
        for (switch, value) in monitor.switches().into_iter().zip(values(&self.switches)) {
            let states: Vec<String> = monitor
                .traces()
                .iter()
                .filter_map(|trace| {
                    let sensors = monitor.sensors(&trace.name)?;
                    let state = match switch.sensor {
                        SensorKind::Gnss => sensors.gnss,
                        SensorKind::Baro => sensors.baro,
                        SensorKind::Stars => sensors.stars,
                        SensorKind::TerrainCamera => sensors.terrain_camera,
                    };
                    Some(format!("{} in {}", state_words(state)?, trace.name))
                })
                .collect();
            value.set_text_content(Some(&states.join("; ")));
        }
    }
}

/// The checkboxes and radio buttons under `parent`.
fn inputs(parent: &Element) -> impl Iterator<Item = HtmlInputElement> {
    let list = parent.query_selector_all("input").ok();
    let count = list.as_ref().map_or(0, web_sys::NodeList::length);
    (0..count).filter_map(move |index| list.as_ref()?.item(index)?.dyn_into().ok())
}

/// The value cells under `parent`, one a row.
fn values(parent: &Element) -> impl Iterator<Item = Element> {
    let list = parent.query_selector_all(".value").ok();
    let count = list.as_ref().map_or(0, web_sys::NodeList::length);
    (0..count).filter_map(move |index| list.as_ref()?.item(index)?.dyn_into().ok())
}

fn checkbox(document: &Document, label: &Element, data: (&str, &str), checked: bool) {
    let Some(input) = append(document, label, "input", "") else {
        return;
    };
    input.set_attribute("type", "checkbox").ok();
    input.set_attribute(data.0, data.1).ok();
    if let Ok(input) = input.dyn_into::<HtmlInputElement>() {
        input.set_checked(checked);
    }
}

fn legend_row(document: &Document, legend: &Element, name: &str, colour: &str) {
    let Some(label) = append(document, legend, "label", "series") else {
        return;
    };
    checkbox(document, &label, ("data-set", name), true);
    if let Some(swatch) = append(document, &label, "span", "swatch") {
        swatch
            .set_attribute("style", &format!("background: {colour}"))
            .ok();
    }
    if let Some(text) = append(document, &label, "span", "") {
        text.set_text_content(Some(name));
    }
    append(document, legend, "span", "value");
}

fn switch_row(document: &Document, switches: &Element, switch: MeansSwitch, can_switch: bool) {
    let Some(label) = append(document, switches, "label", "series") else {
        return;
    };
    checkbox(
        document,
        &label,
        ("data-sensor", switch.sensor.name()),
        false,
    );
    if !can_switch && let Some(input) = label.query_selector("input").ok().flatten() {
        input.set_attribute("disabled", "").ok();
        input
            .set_attribute("title", "A recorded run cannot be changed.")
            .ok();
    }
    if let Some(text) = append(document, &label, "span", "") {
        text.set_text_content(Some(&format!("Withhold {}", sensor_words(switch.sensor))));
    }
    append(document, switches, "span", "value");
}

fn speed_choices(document: &Document, speeds: &Element, speed: Option<f64>) {
    if let Some(title) = append(document, speeds, "span", "") {
        title.set_text_content(Some("Pace"));
    }
    for (index, (words, pace)) in SPEEDS.iter().enumerate() {
        let Some(label) = append(document, speeds, "label", "") else {
            continue;
        };
        if let Some(input) = append(document, &label, "input", "") {
            input.set_attribute("type", "radio").ok();
            input.set_attribute("name", "pace").ok();
            input.set_attribute("data-index", &index.to_string()).ok();
            if let Ok(input) = input.dyn_into::<HtmlInputElement>() {
                input.set_checked(*pace == speed);
            }
        }
        if let Some(text) = append(document, &label, "span", "") {
            text.set_text_content(Some(words));
        }
    }
}

/// One sample of a set in words: its error and the uncertainty it states.
fn sample_words(sample: &TracePoint) -> String {
    match sample.sigma_horizontal_m {
        Some(sigma) => format!(
            "{:.1} m off the truth, {sigma:.1} m one-sigma",
            sample.horizontal_error_m
        ),
        None => format!(
            "{:.1} m off the truth, no uncertainty stated",
            sample.horizontal_error_m
        ),
    }
}
