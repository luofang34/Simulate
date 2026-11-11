//! Flying a flight in the page: each animation frame the flight steps for
//! one slice toward the pace's target, the monitor takes what it produced,
//! and the page draws when anything changed. The page draws between slices,
//! so it stays responsive; a hidden tab gets no frames, so the flight waits
//! for it instead of running up a debt.

use std::fmt::Display;

use sim_link::SimulationLink;
use sim_link_host::{
    Clock, FlightStepper, Monitor, Pacer, SLICE_BUDGET_S, SwitchLog, pace_words, run_slice,
};
use sim_navigate::trial::SensorKind;
use tracing::{info, warn};
use wasm_bindgen_futures::JsFuture;

use super::monitor_view::{Controls, MonitorView};

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
fn run_url(link: &SimulationLink, outages: &[(SensorKind, f64, Option<f64>)]) -> Option<String> {
    let base = web_sys::window()?.location().href().ok()?;
    let mut bare = link.clone();
    bare.outages.clear();
    match SwitchLog::of_outages(outages).url(&bare, &base) {
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
            before.withheld.contains(sensor.name()),
            now.withheld.contains(sensor.name()),
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
}

/// Flies `flight` with `view` showing it, for as long as the page is open.
/// `note` heads the status line: what is flying. With `link`, the view
/// offers the link of the run as flown.
pub async fn fly<F>(mut view: MonitorView, mut flight: F, note: &str, link: Option<&SimulationLink>)
where
    F: FlightStepper,
    F::Error: Display,
{
    let clock = PageClock::of_page();
    let mut outages = flight.outages();
    if let Some(link) = link {
        view.show_run_link(run_url(link, &outages).as_deref());
    }
    let mut monitor = Monitor::default();
    let mut controls = view.controls();
    let mut pacer = Pacer::new(controls.speed);
    let mut out = Vec::new();
    let (mut flying, mut dirty) = (true, true);
    let started_s = clock.now_s();
    let (mut dumped_at_s, mut next_dump_s) = (0.0_f64, 30.0_f64);
    loop {
        animation_frame().await;
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
            let target_s = pacer.frame(clock.now_s(), flight.sim_time_s());
            let probe = sim_flight_chain::profile::start();
            let sliced = run_slice(&mut flight, &clock, SLICE_BUDGET_S, target_s, &mut out);
            sim_flight_chain::profile::stop("page: slice total", probe);
            if let Ok(slice) = &sliced {
                let held = pacer.limit();
                pacer.observe(slice, flight.take_camera_wall_s());
                dirty |= pacer.limit() != held;
            }
            match sliced {
                Ok(slice) if slice.finished => {
                    flying = false;
                    info!(wall_s = clock.now_s() - started_s, "the flight has ended");
                }
                Ok(_) => {}
                Err(error) => {
                    warn!(%error, "the flight failed");
                    view.section().refuse(
                        "The flight stopped",
                        &[sim_link_host::PlanLine {
                            label: "Reason".to_owned(),
                            text: error.to_string(),
                        }],
                    );
                    return;
                }
            }
            dirty |= !out.is_empty();
            let probe = sim_flight_chain::profile::start();
            for feed in out.drain(..) {
                monitor.apply(feed);
            }
            sim_flight_chain::profile::stop("page: monitor apply", probe);
            if flight.sim_time_s() >= next_dump_s {
                let route_s = flight.sim_time_s() - dumped_at_s;
                let mut rows = sim_flight_chain::profile::take();
                rows.sort_by(|a, b| b.1.total_cmp(&a.1));
                let table: Vec<String> = rows
                    .iter()
                    .map(|(label, seconds, calls)| {
                        format!("{label}={:.1}ms/{calls}", 1000.0 * seconds / route_s)
                    })
                    .collect();
                info!(route_s, table = %table.join(" | "), "PROFILE per route second");
                dumped_at_s = flight.sim_time_s();
                next_dump_s = dumped_at_s + 30.0;
            }
            let flown = flight.outages();
            if flown != outages {
                outages = flown;
                if let Some(link) = link {
                    view.show_run_link(run_url(link, &outages).as_deref());
                }
            }
        }
        if dirty {
            let note = match pacer.limit() {
                Some(limit) => format!("{note}{}. ", pace_words(&limit)),
                None => note.to_owned(),
            };
            let probe = sim_flight_chain::profile::start();
            view.draw(&monitor, &controls, &note);
            sim_flight_chain::profile::stop("page: monitor draw", probe);
            dirty = false;
        }
    }
}
