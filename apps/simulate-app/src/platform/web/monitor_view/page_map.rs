//! The page's map: the app's EFB map, expanded onto a canvas of the page, with the
//! flight plan coloured by the mission host's own sequencing, the true track, each set's
//! estimated track with a ring of its stated one-sigma, the terrain camera's fix marks
//! and the sealed package's coverage. The page lays the labels, the buttons and the
//! picked fix's card over the canvas; the app draws everything else.
//!
//! Until the map first draws, the page keeps its own drawing of the track map, which is
//! all a browser without WebGPU gets.

use std::cell::RefCell;
use std::collections::BTreeSet;
use std::rc::Rc;
use std::sync::Arc;

use sim_link_host::Monitor;
use sim_maplibre::{
    EfbBasemap, EfbTracks, FixId, FlightPlan, HoldPattern, LegState, PointMark, RouteFix,
    RouteProgress, SigmaRing, TrackLine,
};
use sim_navigate::trial::{HoldingPatternRecord, HoldingTurnName};
use wasm_bindgen::closure::Closure;
use web_sys::{Document, Element, HtmlCanvasElement};

use super::super::section::append;
use super::series_colour;
use crate::map_rig::EfbCommand;
use crate::page_map::gesture::Gestures;
use crate::page_map::{PageMapLink, PageMapView, PickedFix};

mod controls;

/// The true track: pale yellow, since white means the legs to come.
const TRUTH: [u8; 3] = [0xff, 0xe0, 0x66];
/// Least time between two hand-overs of growing tracks, milliseconds: each one is
/// tessellated again.
const TRACKS_INTERVAL_MS: f64 = 500.0;
/// The page map's height over its width.
const ASPECT: f64 = 0.62;

/// What the page knows of the flight's map before it flies.
#[derive(Debug, Clone)]
pub struct PageMapSetup {
    /// The canvas the app draws the map on.
    pub canvas: HtmlCanvasElement,
    /// The link to the app's map.
    pub link: PageMapLink,
    /// `[lat, lon]` the flight starts at, degrees: the first leg runs from there.
    pub origin: Option<[f64; 2]>,
    /// The sealed package's coverage, as lines of `[lat, lon]`.
    pub coverage: Vec<Vec<[f64; 2]>>,
}

/// What was last handed over, so unchanged tracks are not tessellated again.
#[derive(Debug, Clone, PartialEq)]
struct Handed {
    samples: usize,
    marks: usize,
    hidden: BTreeSet<String>,
}

/// The page's map and what it lays over it.
pub struct PageMap {
    frame: Element,
    canvas: HtmlCanvasElement,
    labels: Element,
    card: Element,
    controls: controls::Controls,
    link: PageMapLink,
    origin: Option<[f64; 2]>,
    coverage: Vec<Vec<[f64; 2]>>,
    handed: Option<(Handed, f64)>,
    _listeners: Vec<Closure<dyn FnMut(web_sys::Event)>>,
}

impl PageMap {
    /// The map at the end of `parent`.
    pub fn under(document: &Document, parent: &Element, setup: PageMapSetup) -> Option<Self> {
        let frame = append(document, parent, "div", "page-map-frame")?;
        let canvas = setup.canvas;
        canvas.set_class_name("page-map");
        canvas
            .set_attribute(
                "aria-label",
                "The flight plan and each set's track on the moving map",
            )
            .ok();
        frame.append_child(&canvas).ok()?;
        let labels = append(document, &frame, "div", "page-map-labels")?;
        let card = append(document, &frame, "div", "page-map-card")?;
        card.set_attribute("hidden", "").ok();
        let controls = controls::Controls::under(document, &frame, &setup.link)?;
        if let Some(key) = append(document, parent, "p", "detail page-map-key") {
            key.set_text_content(Some(KEY));
        }
        let gestures = Rc::new(RefCell::new(Gestures::default()));
        let listeners = controls::listen(&canvas, &setup.link, &gestures);
        let map = Self {
            frame,
            canvas,
            labels,
            card,
            controls,
            link: setup.link,
            origin: setup.origin,
            coverage: setup.coverage,
            handed: None,
            _listeners: listeners,
        };
        // Sized while shown, then hidden until the app's map first draws.
        map.size_buffer();
        map.frame.set_attribute("hidden", "").ok();
        Some(map)
    }

    /// Whether the app's map has drawn on the canvas.
    pub fn live(&self) -> bool {
        self.link.view().live
    }

    /// Hands `monitor`'s plan and tracks, with the sets `hidden` names left out, to the
    /// app's map, and lays what it reports over the canvas.
    pub fn draw(&mut self, monitor: &Monitor, hidden: &BTreeSet<String>) {
        let now_ms = web_sys::window()
            .and_then(|window| window.performance())
            .map_or(0.0, |performance| performance.now());
        let (plan, progress) = plan_of(monitor, self.origin);
        self.link.publish_route(&plan, progress);
        let handed = Handed {
            samples: monitor.traces().iter().map(|t| t.points().len()).sum(),
            marks: monitor.camera().marks().len(),
            hidden: hidden.clone(),
        };
        let due = self.handed.as_ref().is_none_or(|(last, at_ms)| {
            *last != handed
                && (last.hidden != handed.hidden || now_ms - at_ms >= TRACKS_INTERVAL_MS)
        });
        if due {
            let tracks = tracks_of(monitor, hidden, &self.coverage);
            self.link.publish_tracks(Arc::new(tracks));
            self.handed = Some((handed, now_ms));
        }
        self.follow_width();
        let view = self.link.view();
        self.link.set_in_sight(!view.live || self.in_sight());
        if view.live {
            self.frame.remove_attribute("hidden").ok();
        }
        self.lay_labels(&view);
        self.lay_card(&view);
        self.controls.show(view.status.as_ref());
    }

    /// Sizes the canvas's drawing buffer to its width on screen in physical pixels.
    fn size_buffer(&self) {
        let ratio = web_sys::window().map_or(1.0, |w| w.device_pixel_ratio().max(1.0));
        let width = f64::from(self.canvas.client_width()).max(320.0) * ratio;
        self.canvas.set_width(width as u32);
        self.canvas.set_height((width * ASPECT) as u32);
    }

    /// Sizes the drawing buffer again when the canvas's width on screen changed, and
    /// tells the app's map.
    fn follow_width(&self) {
        let ratio = web_sys::window().map_or(1.0, |w| w.device_pixel_ratio().max(1.0));
        let css = self.canvas.client_width();
        if css <= 0 {
            return;
        }
        let width = (f64::from(css) * ratio) as u32;
        if width.abs_diff(self.canvas.width()) <= 1 {
            return;
        }
        let height = (f64::from(width) * ASPECT) as u32;
        self.canvas.set_width(width);
        self.canvas.set_height(height);
        self.link.resize([width, height]);
    }

    /// Whether any of the map is within the browser's window.
    fn in_sight(&self) -> bool {
        let rect = self.frame.get_bounding_client_rect();
        let height = web_sys::window()
            .and_then(|window| window.inner_height().ok())
            .and_then(|height| height.as_f64())
            .unwrap_or(f64::INFINITY);
        rect.bottom() > 0.0 && rect.top() < height
    }

    /// CSS pixels per drawing-buffer pixel.
    fn css_per_px(&self) -> f64 {
        f64::from(self.canvas.client_width()) / f64::from(self.canvas.width().max(1))
    }

    fn lay_labels(&self, view: &PageMapView) {
        let Some(document) = self.labels.owner_document() else {
            return;
        };
        let children = self.labels.children();
        while children.length() as usize > view.labels.len() {
            if let Some(last) = self.labels.last_element_child() {
                last.remove();
            }
        }
        while (children.length() as usize) < view.labels.len() {
            if append(&document, &self.labels, "span", "").is_none() {
                return;
            }
        }
        let scale = self.css_per_px();
        for (index, label) in view.labels.iter().enumerate() {
            let Some(element) = children.item(index as u32) else {
                continue;
            };
            let [x, y] = label.at_px.map(|v| f64::from(v) * scale);
            element.set_class_name(&format!("fix-label {}", label.state.name()));
            element.set_text_content(Some(&label.ident));
            element
                .set_attribute("style", &format!("left: {x:.1}px; top: {y:.1}px"))
                .ok();
        }
    }

    fn lay_card(&self, view: &PageMapView) {
        let Some(picked) = &view.picked else {
            self.card.set_attribute("hidden", "").ok();
            return;
        };
        let at = view
            .labels
            .iter()
            .find(|label| label.index == picked.index)
            .map(|label| label.at_px.map(|v| f64::from(v) * self.css_per_px()));
        let Some([x, y]) = at else {
            self.card.set_attribute("hidden", "").ok();
            return;
        };
        self.card.set_inner_html("");
        if let Some(document) = self.card.owner_document() {
            for (class, text) in card_lines(picked) {
                if let Some(line) = append(&document, &self.card, "div", class) {
                    line.set_text_content(Some(&text));
                }
            }
        }
        self.card
            .set_attribute("style", &format!("left: {x:.1}px; top: {y:.1}px"))
            .ok();
        self.card.remove_attribute("hidden").ok();
    }
}

/// What the map's marks mean.
const KEY: &str = "The cockpit's moving map, expanded. Magenta: the active leg; white: legs \
    to come; grey: legs flown. Yellow: the true track. Coloured: each set's estimate with its \
    one-sigma ring; dots and rings: camera fixes accepted and rejected. Dashed green: the map \
    package's imagery. Drag, zoom or tap a fix.";

/// The plan from `monitor`'s route, flown from `origin`, at the mission host's progress.
fn plan_of(monitor: &Monitor, origin: Option<[f64; 2]>) -> (FlightPlan, RouteProgress) {
    let altitudes = monitor.waypoint_altitudes_m();
    let fixes = monitor
        .waypoints()
        .iter()
        .enumerate()
        .map(|(index, (ident, (lat_deg, lon_deg)))| RouteFix {
            id: FixId(u32::try_from(index).unwrap_or(u32::MAX)),
            ident: ident.clone(),
            lat_deg: *lat_deg,
            lon_deg: *lon_deg,
            altitude_m: altitudes.get(index).copied(),
        })
        .collect();
    let progress = RouteProgress {
        hold: monitor.hold_pattern().map(hold_of),
        ..RouteProgress::after_sequenced(monitor.sequenced())
    };
    (FlightPlan { origin, fixes }, progress)
}

/// The map's drawing of the pattern the mission host reported.
fn hold_of(pattern: &HoldingPatternRecord) -> HoldPattern {
    HoldPattern {
        fix_deg: [
            pattern.fix.latitude_rad.to_degrees(),
            pattern.fix.longitude_rad.to_degrees(),
        ],
        inbound_course_deg: pattern.inbound_course_rad.to_degrees(),
        right_turns: matches!(pattern.turn, HoldingTurnName::Right),
        turn_radius_m: pattern.turn_radius_m,
        leg_length_m: pattern.leg_length_m,
    }
}

/// The true track, each shown set's track, ring and fix marks, and the coverage.
fn tracks_of(
    monitor: &Monitor,
    hidden: &BTreeSet<String>,
    coverage: &[Vec<[f64; 2]>],
) -> EfbTracks {
    let shown = |index: usize| {
        monitor
            .traces()
            .get(index)
            .is_some_and(|trace| !hidden.contains(&trace.name))
    };
    let mut tracks = EfbTracks {
        outlines: coverage.to_vec(),
        ..EfbTracks::default()
    };
    for (index, trace) in monitor.traces().iter().enumerate() {
        if !shown(index) {
            continue;
        }
        let colour = rgb(series_colour(index));
        tracks.lines.push(TrackLine {
            colour,
            width_px: 1.5,
            points: trace
                .points()
                .iter()
                .map(|p| [p.estimate_deg.0, p.estimate_deg.1])
                .collect(),
        });
        if let Some((latest, sigma_m)) = trace
            .latest()
            .and_then(|latest| Some((latest, latest.sigma_horizontal_m?)))
        {
            tracks.rings.push(SigmaRing {
                centre_deg: [latest.estimate_deg.0, latest.estimate_deg.1],
                sigma_m,
                colour,
            });
        }
    }
    tracks.lines.push(TrackLine {
        colour: TRUTH,
        width_px: 2.0,
        points: monitor
            .truth()
            .iter()
            .map(|(_, (lat, lon))| [*lat, *lon])
            .collect(),
    });
    tracks.marks = monitor
        .camera()
        .marks()
        .iter()
        .filter(|mark| shown(mark.set))
        .map(|mark| PointMark {
            at_deg: [mark.at_deg.0, mark.at_deg.1],
            colour: rgb(series_colour(mark.set)),
            filled: mark.accepted,
        })
        .collect();
    tracks
}

/// `[r, g, b]` of a `#rrggbb` colour; grey for anything else.
fn rgb(css: &str) -> [u8; 3] {
    let channel = |range: std::ops::Range<usize>| {
        css.get(range)
            .and_then(|hex| u8::from_str_radix(hex, 16).ok())
    };
    match (channel(1..3), channel(3..5), channel(5..7)) {
        (Some(r), Some(g), Some(b)) if css.len() == 7 => [r, g, b],
        _ => [0x8f, 0x98, 0xad],
    }
}

/// The picked fix's card, line by line with each line's class: always four
/// lines, so the card keeps its size from fix to fix.
fn card_lines(picked: &PickedFix) -> Vec<(&'static str, String)> {
    let [lat, lon] = picked.at_deg;
    let place = match picked.state {
        LegState::Flown => "sequenced",
        LegState::Active => "flown toward now",
        LegState::Future => "to come",
    };
    let mut lines = vec![
        ("ident", picked.ident.clone()),
        ("", format!("{} {}", dms(lat, 'N', 'S'), dms(lon, 'E', 'W'))),
    ];
    lines.push((
        "",
        picked.altitude_m.map_or_else(
            || "Altitude —".to_owned(),
            |altitude_m| {
                format!(
                    "{altitude_m:.0} m ({:.0} ft) ellipsoid",
                    altitude_m / 0.3048
                )
            },
        ),
    ));
    lines.push((
        "",
        format!("Waypoint {} · {place}", picked.index.saturating_add(1)),
    ));
    lines
}

/// `value` degrees as degrees and decimal minutes with its hemisphere.
fn dms(value: f64, positive: char, negative: char) -> String {
    let hemisphere = if value < 0.0 { negative } else { positive };
    let value = value.abs();
    let degrees = value.trunc();
    let minutes = (value - degrees) * 60.0;
    format!("{hemisphere}{degrees:.0}°{minutes:06.3}′")
}

/// Commands the page's buttons give the map.
pub(super) fn basemap_command(satellite: bool) -> EfbCommand {
    EfbCommand::SetBasemap(if satellite {
        EfbBasemap::Satellite
    } else {
        EfbBasemap::Chart
    })
}
