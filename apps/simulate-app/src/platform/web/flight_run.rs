//! Flying a flight in the page: each animation frame the flight steps for
//! one slice toward the pace's target, the monitor takes what it produced,
//! and the page draws when anything changed. The page draws between slices,
//! so it stays responsive; a hidden tab gets no frames, so the flight waits
//! for it instead of running up a debt.

use std::fmt::Display;

use sim_link::SimulationLink;
use sim_link_host::{
    Clock, FlightStepper, Monitor, MonitorFeed, Pacer, SLICE_BUDGET_S, SpoofRun, SwitchLog,
    run_lines, run_slice, stopped_lines,
};
use sim_navigate::trial::SensorKind;
use tracing::{info, warn};
use wasm_bindgen_futures::JsFuture;

use super::monitor_view::{Controls, MonitorView};
use crate::app::cadence::FrameCadence;

/// The wall clock of the page.
#[derive(Clone)]
pub struct PageClock(Option<web_sys::Performance>);

impl PageClock {
    /// The page's clock; one that stays at zero in a page without a
    /// window.
    pub fn of_page() -> Self {
        Self(web_sys::window().and_then(|window| window.performance()))
    }
}

impl Clock for PageClock {
    fn now_s(&self) -> f64 {
        self.0.as_ref().map_or(0.0, web_sys::Performance::now) / 1000.0
    }
}

/// Waits for the page's next animation frame. A page without a window
/// does not wait.
async fn animation_frame() {
    let Some(window) = web_sys::window() else {
        return;
    };
    let frame = js_sys::Promise::new(&mut |resolve, _reject| {
        window.request_animation_frame(&resolve).ok();
    });
    JsFuture::from(frame).await.ok();
}

/// The link of the run as flown, on the page's own address: `link` with
/// the flight's outages, the stated and the switched, in place of its own.
/// Waits for the next animation frame at most every 60 Hz refresh, as the world view
/// draws, so a frame's slice and its drawing fit one steady frame.
async fn steady_frame(cadence: &mut FrameCadence, clock: &PageClock) {
    loop {
        animation_frame().await;
        if cadence.due(clock.now_s()) {
            return;
        }
    }
}

/// The faults a flight has flown so far: its outages and its spoofer.
type Flown = (Vec<(SensorKind, f64, Option<f64>)>, Option<SpoofRun>);

fn flown<F: FlightStepper>(flight: &F) -> Flown {
    (flight.outages(), flight.gnss_spoof())
}

fn run_url(link: &SimulationLink, (outages, spoof): &Flown) -> Option<String> {
    let base = web_sys::window()?.location().href().ok()?;
    let mut bare = link.clone();
    bare.outages.clear();
    bare.spoof = None;
    let log = SwitchLog::of_outages(outages).with_spoof(spoof.as_ref());
    match log.url(&bare, &base) {
        Ok(url) => Some(url),
        Err(refusal) => {
            warn!(%refusal, "the run's link cannot be written");
            None
        }
    }
}

/// Hands the flight the switches the visitor changed since `before`.
fn apply_switches<F: FlightStepper>(flight: &mut F, before: &Controls, now: &Controls) {
    for sensor in SensorKind::ALL {
        let (was, is) = (
            before.faults.stopped.contains(sensor.name()),
            now.faults.stopped.contains(sensor.name()),
        );
        if was != is {
            info!(
                sensor = sensor.name(),
                on = !is,
                "a switch the visitor changed"
            );
            flight.switch(sensor, !is);
        }
    }
    if before.faults.spoof_gnss != now.faults.spoof_gnss {
        info!(
            on = now.faults.spoof_gnss,
            "the GNSS spoofer the visitor switched"
        );
        flight.spoof_gnss(now.faults.spoof_gnss);
    }
}

/// What share of the page's frames the flight loop itself takes, measured
/// over a few seconds: the rest goes to the page's other drawing, such as
/// the world view, which slows the pace without the flight costing more.
#[derive(Debug, Default)]
struct FrameShare {
    frame_started_s: Option<f64>,
    own_s: f64,
    wall_s: f64,
    others: Option<f64>,
}

/// Wall time a share is measured over, seconds.
const SHARE_WINDOW_S: f64 = 3.0;

impl FrameShare {
    /// A frame began at `now_s`.
    fn frame(&mut self, now_s: f64) {
        if let Some(last) = self.frame_started_s.replace(now_s) {
            self.wall_s += (now_s - last).max(0.0);
        }
        if self.wall_s >= SHARE_WINDOW_S {
            self.others = Some((1.0 - self.own_s / self.wall_s).clamp(0.0, 1.0));
            (self.own_s, self.wall_s) = (0.0, 0.0);
        }
    }

    /// The loop's own work this frame ended at `now_s`.
    fn own_until(&mut self, now_s: f64) {
        if let Some(started) = self.frame_started_s {
            self.own_s += (now_s - started).max(0.0);
        }
    }

    /// The page's other drawing, named `others`, with its share of the
    /// page's time once measured.
    fn of<'a>(&self, others: Option<&'a str>) -> Option<(&'a str, f64)> {
        others.zip(self.others)
    }
}

/// Steps `flight` for one slice toward the pace and hands the monitor what
/// it produced. Returns whether the flight goes on, and whether anything
/// the page shows changed.
fn slice<F>(
    flight: &mut F,
    (clock, pacer): (&PageClock, &mut Pacer),
    monitor: &mut Monitor,
    out: &mut Vec<MonitorFeed>,
) -> Result<(bool, bool), F::Error>
where
    F: FlightStepper,
{
    let target_s = pacer.frame(clock.now_s(), flight.sim_time_s());
    let slice = run_slice(flight, clock, SLICE_BUDGET_S, target_s, out)?;
    let held = pacer.limit();
    pacer.observe(&slice, flight.take_camera_wall_s());
    let fed = !out.is_empty();
    for feed in out.drain(..) {
        monitor.apply(feed);
    }
    Ok((!slice.finished, fed || pacer.limit() != held))
}

/// Flies `flight` with `view` showing it, for as long as the page is open.
/// `note` heads the status: what is flying. With `link`, the view
/// offers the link of the run as flown. `publish` sees the flight after
/// every slice, for `others`, the page's other views of it.
pub async fn fly<F>(
    mut view: MonitorView,
    mut flight: F,
    (note, link): (&str, Option<&SimulationLink>),
    publish: &mut dyn FnMut(&F, &Monitor),
    others: Option<&str>,
) where
    F: FlightStepper,
    F::Error: Display,
{
    let clock = PageClock::of_page();
    let mut faults = flown(&flight);
    if let Some(link) = link {
        view.show_run_link(run_url(link, &faults).as_deref());
    }
    let (mut monitor, mut share) = (Monitor::default(), FrameShare::default());
    let mut controls = view.controls();
    let mut pacer = Pacer::new(controls.speed);
    let mut out = Vec::new();
    let (mut flying, mut dirty) = (true, true);
    let started_s = clock.now_s();
    let mut cadence = FrameCadence::default();
    loop {
        steady_frame(&mut cadence, &clock).await;
        share.frame(clock.now_s());
        let now = view.controls();
        if now != controls {
            if flying {
                apply_switches(&mut flight, &controls, &now);
            }
            if now.speed != controls.speed {
                pacer.set_speed(now.speed);
            }
            controls = now;
            dirty = true;
        }
        if flying {
            match slice(&mut flight, (&clock, &mut pacer), &mut monitor, &mut out) {
                Ok((goes_on, changed)) => {
                    flying = goes_on;
                    dirty |= changed || !goes_on;
                    if !goes_on {
                        info!(wall_s = clock.now_s() - started_s, "the flight has ended");
                    }
                }
                Err(error) => {
                    warn!(%error, "the flight failed");
                    view.section().refuse(
                        "The flight stopped",
                        &stopped_lines(note),
                        &[sim_link_host::PlanLine {
                            label: "Reason".to_owned(),
                            text: error.to_string(),
                        }],
                    );
                    return;
                }
            }
            publish(&flight, &monitor);
            let now = flown(&flight);
            if now != faults {
                faults = now;
                if let Some(link) = link {
                    view.show_run_link(run_url(link, &faults).as_deref());
                }
            }
        }
        if dirty {
            view.draw(
                &monitor,
                &controls,
                &run_lines(note, &monitor, &pacer, share.of(others)),
            );
            dirty = false;
        }
        share.own_until(clock.now_s());
    }
}
