//! Flying a flight in the page: each animation frame the flight steps for
//! one slice toward the pace's target, the monitor takes what it produced,
//! and the page draws when anything changed. The page draws between slices,
//! so it stays responsive; a hidden tab gets no frames, so the flight waits
//! for it instead of running up a debt.

use std::fmt::Display;

use sim_link_host::{Clock, FlightStepper, Monitor, Pacer, SLICE_BUDGET_S, run_slice};
use tracing::{info, warn};
use wasm_bindgen_futures::JsFuture;

use super::monitor_view::MonitorView;

/// The wall clock of the page.
struct PageClock(Option<web_sys::Performance>);

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

/// Flies `flight` with `view` showing it, for as long as the page is open.
/// `note` heads the status line: what is flying.
pub async fn fly<F>(mut view: MonitorView, mut flight: F, note: &str)
where
    F: FlightStepper,
    F::Error: Display,
{
    let clock = PageClock(web_sys::window().and_then(|window| window.performance()));
    let mut monitor = Monitor::default();
    let mut controls = view.controls();
    let mut pacer = Pacer::new(controls.speed);
    let mut out = Vec::new();
    let (mut flying, mut dirty) = (true, true);
    let started_s = clock.now_s();
    loop {
        animation_frame().await;
        let now = view.controls();
        if now != controls {
            pacer.set_speed(now.speed);
            controls = now;
            dirty = true;
        }
        if flying {
            let target_s = pacer.frame(clock.now_s(), flight.sim_time_s());
            match run_slice(&mut flight, &clock, SLICE_BUDGET_S, target_s, &mut out) {
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
            for feed in out.drain(..) {
                monitor.apply(feed);
            }
        }
        if dirty {
            view.draw(&monitor, &controls, note);
            dirty = false;
        }
    }
}
