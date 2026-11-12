//! The flight monitor on the page: the status line, whose navigation the
//! flight is flown on, the sensors' fault switches, the pace, the error plot,
//! a legend that shows or hides each set, the track map, the terrain camera's
//! latest frame and what the visual navigator matched it against, the link of
//! the run, the terrain camera's tallies, the mission host's reports and the
//! latest events.
//!
//! The controls are read when the page draws, not listened to: the flight
//! loop runs every animation frame anyway, so a click shows within a frame.

mod frame_panel;
mod match_panel;
mod page_map;
mod plot;
mod sensor_panel;
mod steering;
mod track_map;

use std::collections::BTreeSet;

use sim_chain_trial::FLIGHT_CONTROLLER_SET;
use sim_link_host::{
    ABSENT, CameraWorld, Monitor, PlanLine, SpoofDrag, TracePoint, clock_words, event_words,
    terrain_words,
};
use sim_navigate::NavMeans;
use wasm_bindgen::JsCast;
use web_sys::{Document, Element, HtmlInputElement};

use super::section::{Section, append, write_lines};
use frame_panel::FramePanel;
use match_panel::MatchPanel;
use page_map::PageMap;
pub use page_map::PageMapSetup;
use plot::Plot;
pub use sensor_panel::FaultsAsked;
use sensor_panel::SensorPanel;
pub use steering::Steering;
use track_map::TrackMap;

/// The colours of the sets, in the run's order, for the page's dark
/// surface. None is magenta, which the map keeps for the active leg, nor the
/// map's pale yellow true track.
const SERIES: [&str; 8] = [
    "#3987e5", "#d95926", "#199e70", "#c98500", "#2bb5c9", "#008300", "#9085e9", "#e66767",
];
/// The name the page gives the simulator's true state, wherever it marks it.
const GROUNDTRUTH: &str = "simulator groundtruth";
/// Its colour on the maps.
const GROUNDTRUTH_COLOUR: &str = "#ffe066";
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
    /// The sensor faults the visitor asks for.
    pub faults: FaultsAsked,
    /// The pace.
    pub speed: Option<f64>,
}

/// What a monitor is built from.
pub struct MonitorSetup<'a> {
    /// The pace the flight starts at.
    pub speed: Option<f64>,
    /// Whether the flight takes the visitor's switches; a recorded run cannot.
    pub can_switch: bool,
    /// The set the flight steers on and the links that fly on another.
    pub steering: Option<Steering>,
    /// The drag a spoofer switched on here applies.
    pub spoof: SpoofDrag,
    /// The faults the link leaves in force to the end of the flight.
    pub faults_at_start: FaultsAsked,
    /// What stands under the sets' results.
    pub caveat: Option<&'a str>,
    /// What the terrain camera sees, which the track map lies over.
    pub world: Option<CameraWorld>,
    /// What stands beside the terrain camera's results.
    pub world_note: Option<String>,
    /// What closes the section.
    pub about: &'a str,
    /// The page's map, when the app draws one.
    pub page_map: Option<PageMapSetup>,
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
    steering: Option<Steering>,
    steering_block: Element,
    sensors: SensorPanel,
    speeds: Element,
    terrain: Element,
    mission: Element,
    events: Element,
    run_link: Element,
    built_for: Vec<String>,
}

impl MonitorView {
    /// The monitor of `setup` in `section`.
    pub fn inside(section: Section, setup: MonitorSetup<'_>) -> Option<Self> {
        let document = section.document().clone();
        let root = section.root().clone();
        // What the flight is flown on and the visitor's controls stand first, next to
        // the views they change.
        let steering_block = append(&document, &root, "div", "")?;
        let sensors = SensorPanel::under(
            &document,
            &root,
            setup.can_switch,
            (setup.spoof, setup.faults_at_start),
        )?;
        let speeds = append(&document, &root, "div", "speeds")?;
        speed_choices(&document, &speeds, setup.speed);
        let plot = Plot::under(&document, &root)?;
        let legend = append(&document, &root, "div", "legend")?;
        if let Some(caveat) = setup.caveat
            && let Some(line) = append(&document, &root, "p", "detail")
        {
            line.set_text_content(Some(caveat));
        }
        let page_map = setup
            .page_map
            .and_then(|setup| PageMap::under(&document, &root, setup));
        let map = TrackMap::under(&document, &root, setup.world)?;
        let frame = FramePanel::under(&document, &root)?;
        let matching = MatchPanel::under(&document, &root)?;
        let run_link = append(&document, &root, "p", "detail run-link")?;
        if let Some(note) = setup.world_note
            && let Some(line) = append(&document, &root, "p", "detail")
        {
            line.set_text_content(Some(&format!("The camera's world: {note}")));
        }
        let terrain = append(&document, &root, "dl", "lines live-rows")?;
        let mission = append(&document, &root, "dl", "lines live-rows")?;
        let events = append(&document, &root, "dl", "lines live-rows")?;
        if let Some(closing) = append(&document, &root, "p", "detail") {
            closing.set_text_content(Some(setup.about));
        }
        Some(Self {
            section,
            plot,
            map,
            page_map,
            frame,
            matching,
            legend,
            steering: setup.steering,
            steering_block,
            sensors,
            speeds,
            terrain,
            mission,
            events,
            run_link,
            built_for: Vec::new(),
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
            words.set_text_content(Some("This run, switches included: "));
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
        let speed = inputs(&self.speeds)
            .find(HtmlInputElement::checked)
            .and_then(|input| input.get_attribute("data-index"))
            .and_then(|index| index.parse::<usize>().ok())
            .and_then(|index| SPEEDS.get(index))
            .map_or(Some(1.0), |(_, speed)| *speed);
        Controls {
            hidden,
            faults: self.sensors.asked(),
            speed,
        }
    }

    /// Shows `monitor` under `controls`, with `status` as the section's status.
    pub fn draw(&mut self, monitor: &Monitor, controls: &Controls, status: &[String]) {
        self.rebuild_if_new(monitor);
        self.section.settled(status);
        self.plot
            .draw(monitor, &|set| !controls.hidden.contains(set));
        self.legend_values(monitor);
        self.draw_map(monitor, &controls.hidden);
        self.frame.draw(monitor);
        self.matching.draw(monitor);
        self.sensors.show(self.section.document(), monitor);
        let visual: Vec<PlanLine> = monitor
            .traces()
            .iter()
            .filter(|trace| trace.means.contains(&NavMeans::Visual))
            .map(|trace| PlanLine {
                label: format!("Camera, {}", trace.name),
                text: terrain_words(monitor, &trace.name),
            })
            .collect();
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
            let steered = self.steering.as_ref().map(|steering| steering.set.as_str());
            page_map.draw(monitor, hidden, steered);
            if page_map.live() {
                self.map.retire();
                return;
            }
        }
        self.map.draw(monitor, hidden);
    }

    /// Builds the legend and the steering block when the run's sets change.
    fn rebuild_if_new(&mut self, monitor: &Monitor) {
        let names: Vec<String> = monitor.traces().iter().map(|t| t.name.clone()).collect();
        if names == self.built_for {
            return;
        }
        let document = self.section.document().clone();
        self.legend.set_text_content(None);
        let steered = self.steering.as_ref().map(|steering| steering.set.as_str());
        for (index, name) in names.iter().enumerate() {
            let role = match name.as_str() {
                set if Some(set) == steered => Some("flying on · the map's aircraft"),
                FLIGHT_CONTROLLER_SET => Some("Aviate's own estimate · compared only"),
                _ => None,
            };
            legend_row(&document, &self.legend, (name, role), series_colour(index));
        }
        groundtruth_row(&document, &self.legend);
        self.steering_block.set_text_content(None);
        if let Some(steering) = &self.steering {
            let index = names.iter().position(|name| *name == steering.set);
            let colour = index.map_or(SERIES_OTHER, series_colour);
            steering::steering_block(&document, &self.steering_block, steering, colour);
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
            let now = trace
                .latest()
                .map_or_else(|| ABSENT.to_owned(), sample_words);
            let at_end = ends
                .and_then(|samples| samples.get(index).copied().flatten())
                .map_or_else(|| ABSENT.to_owned(), |sample| sample_words(&sample));
            let text = format!("now {now} · end {at_end}");
            if value.text_content().as_deref() != Some(text.as_str()) {
                value.set_text_content(Some(&text));
                value.set_attribute("title", &text).ok();
            }
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

/// The legend's last row: the simulator groundtruth's colour and name. It has no
/// switch, since the maps always mark it, and its value cell keeps the rows paired.
fn groundtruth_row(document: &Document, legend: &Element) {
    let Some(label) = append(document, legend, "span", "series groundtruth") else {
        return;
    };
    if let Some(swatch) = append(document, &label, "span", "swatch") {
        swatch
            .set_attribute("style", &format!("background: {GROUNDTRUTH_COLOUR}"))
            .ok();
    }
    if let Some(text) = append(document, &label, "span", "") {
        text.set_text_content(Some(GROUNDTRUTH));
    }
    if let Some(value) = append(document, legend, "span", "value") {
        value.set_text_content(Some("known only to the simulator"));
    }
}

/// A set's row: its switch, colour and name, and its `role` in the flight when it has
/// one.
fn legend_row(
    document: &Document,
    legend: &Element,
    (name, role): (&str, Option<&str>),
    colour: &str,
) {
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
    if let Some(role) = role
        && let Some(badge) = append(document, &label, "span", "role")
    {
        badge.set_text_content(Some(role));
    }
    append(document, legend, "span", "value");
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

/// One sample of a set: its error against the truth and the one-sigma it
/// states, "12.3 m, σ 8.1".
fn sample_words(sample: &TracePoint) -> String {
    let sigma = sample
        .sigma_horizontal_m
        .map_or_else(|| ABSENT.to_owned(), |sigma| format!("{sigma:.1}"));
    format!("{:.1} m, σ {sigma}", sample.horizontal_error_m)
}
