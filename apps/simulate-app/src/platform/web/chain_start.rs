//! The link flown by the vehicle's own software in the page: Pilotage's
//! mission host, Aviate's flight controller for the C172 and Navigate's
//! navigators, stepped in the page's slices, the terrain camera rendered
//! by the trial's own renderer from the sealed package.

use navigate_data::MemoryStore;
use sim_chain_trial::ChainTrial;
use sim_link_host::{
    CameraWorld, ChainFlight, Clock, HostError, Monitor, PlanLine, chain_spec, page_stores,
};
use sim_navdata::SealedPackage;
use sim_navigate::trial::parse_utc;
use tracing::{info, warn};

use super::PageView;
use super::flight_run::{PageClock, fly};
use super::monitor_view::MonitorView;
use super::package_run::LinkPlan;
use super::section::{Section, append};
use crate::vehicles::{PageFeed, PageSnapshot};
use wasm_bindgen::JsCast;
use web_sys::{Element, HtmlCanvasElement};

/// The pace a flight starts at, route seconds per wall second; the page
/// holds it lower when it cannot afford it.
const START_SPEED: f64 = 4.0;
/// What heads the status line.
const NOTE: &str = "Flown by the vehicle's own software: Pilotage's mission host, Aviate's \
    flight controller for the C172 and Navigate's navigators. ";
/// What the page says of the camera's world beside the visual results.
const WORLD_NOTE: &str = "the camera sees the same imagery the onboard map was built from, \
    which favours visual navigation.";
/// What the page says of the results: a set with nothing to fix its
/// position drifts, and says so through its own uncertainty.
const CAVEAT: &str = "ins.baro has nothing to fix its horizontal position, so its error grows \
    as the inertial sensors drift: kilometres within minutes on this vehicle's consumer-grade \
    unit, with its stated uncertainty growing alongside. Hide a set in the legend to fit the \
    map to the others.";
/// What closes the section.
const ABOUT: &str = "About this run: the same link flies the same flight in the same build of \
    this page, switches included once they are in the link. A desktop build of Simulate may \
    differ in the last digits, because the browser and the desktop compute some mathematical \
    functions differently.";

fn now_ms() -> f64 {
    web_sys::window()
        .and_then(|window| window.performance())
        .map_or(0.0, |performance| performance.now())
}

/// Lets the page paint before a long piece of work.
async fn yield_to_page() {
    let Some(window) = web_sys::window() else {
        return;
    };
    let later = js_sys::Promise::new(&mut |resolve, _reject| {
        window
            .set_timeout_with_callback_and_timeout_and_arguments_0(&resolve, 30)
            .ok();
    });
    wasm_bindgen_futures::JsFuture::from(later).await.ok();
}

/// The chain trial of the link and, when a set is visual, the camera's
/// world, both from the sealed package.
fn prepare(
    plan: &LinkPlan,
    sealed: Option<&SealedPackage>,
) -> Result<(ChainTrial, Option<CameraWorld>), HostError> {
    let identity = sealed.map(|sealed| sealed.summary.identity);
    let memory = sealed.map_or_else(MemoryStore::default, |sealed| sealed.memory.clone());
    let spec = chain_spec(&plan.link, &plan.request, identity)?;
    let started = now_ms();
    let trial = ChainTrial::new(&spec, &page_stores(memory.clone()))?;
    info!(ms = now_ms() - started, "the vehicle's software is ready");
    let world = match (plan.request.camera, identity) {
        (Some(_), Some(identity)) => {
            let started = now_ms();
            let world = CameraWorld::open_blocking(memory, identity)?;
            info!(ms = now_ms() - started, "the camera's world is decoded");
            Some(world)
        }
        _ => None,
    };
    Ok((trial, world))
}

/// Starts the app's own world view at the top of `section`, above the
/// monitor, drawing the flight `feed` publishes. Whatever becomes of it, the
/// flight goes on. Returns where the page states the tab's memory.
fn start_world_view(section: &Section, feed: PageFeed) -> Option<Element> {
    let document = section.document();
    let canvas = append(document, section.root(), "canvas", "world-view")
        .and_then(|canvas| canvas.dyn_into::<HtmlCanvasElement>().ok());
    let line = append(document, section.root(), "p", "detail");
    let notes = append(document, section.root(), "p", "detail");
    let (Some(canvas), Some(line), Some(notes)) = (canvas, line, notes) else {
        warn!("the page has no room for the world view");
        return None;
    };
    let diagnostics = append(document, &notes, "span", "")?;
    let memory = append(document, &notes, "span", "")?;
    canvas
        .set_attribute("aria-label", "The world outside the aircraft")
        .ok();
    line.set_text_content(Some("Starting the world view."));
    super::show_world(
        PageView {
            canvas,
            line,
            diagnostics,
        },
        feed,
    );
    Some(memory)
}

/// The tab's WebAssembly memory in words: it only grows, so it is the most
/// the run has needed, against the 4 GiB a 32-bit module can address.
fn memory_words() -> String {
    let bytes = core::arch::wasm32::memory_size(0) as u64 * 65_536;
    format!("; WebAssembly memory {} MB of 4096", bytes >> 20)
}

/// Flies the link of `plan` in `section`.
pub async fn fly_link(section: Section, plan: &LinkPlan, sealed: Option<&SealedPackage>) {
    section.working(
        "Starting the vehicle's software: trimming the C172, aligning every navigator and \
         decoding the camera's world.",
    );
    yield_to_page().await;
    let (trial, world) = match prepare(plan, sealed) {
        Ok(ready) => ready,
        Err(error) => {
            warn!(%error, "the flight cannot start");
            section.refuse(
                "The flight cannot start",
                &[PlanLine {
                    label: "Reason".to_owned(),
                    text: error.to_string(),
                }],
            );
            return;
        }
    };
    let world_note = world.as_ref().map(|_| WORLD_NOTE.to_owned());
    let feed = match parse_utc(&plan.request.trial.epoch_utc) {
        Ok(epoch) => Some(PageFeed::new(
            epoch.unix_seconds(),
            PageSnapshot::of(&trial, None),
        )),
        Err(error) => {
            warn!(%error, "the link's epoch does not parse; no world view");
            None
        }
    };
    let mut memory = None;
    if let Some(feed) = &feed {
        memory = start_world_view(&section, feed.clone());
    }
    let Some(view) = MonitorView::inside(
        section,
        Some(START_SPEED),
        true,
        Some(CAVEAT),
        (world.clone(), world_note),
        ABOUT,
    ) else {
        return;
    };
    let flight = ChainFlight::new(trial, world, PageClock::of_page());
    let clock = PageClock::of_page();
    let mut memory_at_s = f64::NEG_INFINITY;
    let mut publish = |flight: &ChainFlight<PageClock>, monitor: &Monitor| {
        if let Some(line) = memory
            .as_ref()
            .filter(|_| clock.now_s() - memory_at_s >= 2.0)
        {
            memory_at_s = clock.now_s();
            line.set_text_content(Some(&memory_words()));
        }
        if let Some(feed) = &feed {
            feed.publish(PageSnapshot::of(
                flight.trial(),
                monitor.phase().map(str::to_owned),
            ));
        }
    };
    let others = feed.as_ref().map(|_| "the world view");
    fly(view, flight, (NOTE, Some(&plan.link)), &mut publish, others).await;
}
