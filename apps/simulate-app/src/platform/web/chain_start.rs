//! The link flown by the vehicle's own software in the page: Pilotage's
//! mission host, Aviate's flight controller for the C172 and Navigate's
//! navigators, stepped in the page's slices, the terrain camera rendered
//! by the trial's own renderer from the sealed package.

use navigate_data::MemoryStore;
use sim_chain_trial::ChainTrial;
use sim_link_host::{CameraWorld, ChainFlight, HostError, PlanLine, chain_spec, page_stores};
use sim_navdata::SealedPackage;
use tracing::{info, warn};

use super::flight_run::{PageClock, fly};
use super::monitor_view::MonitorView;
use super::package_run::LinkPlan;
use super::section::Section;

/// The pace a flight starts at, route seconds per wall second; the page
/// holds it lower when it cannot afford it.
const START_SPEED: f64 = 4.0;
/// What heads the status line.
const NOTE: &str = "Flown by the vehicle's own software: Pilotage's mission host, Aviate's \
    flight controller for the C172 and Navigate's navigators. ";
/// What the page says of the camera's world beside the visual results.
const WORLD_NOTE: &str = "the camera sees the same imagery the onboard map was built from, \
    which favours visual navigation.";
/// What the page says of the results while a known defect stands.
const CAVEAT: &str = "The barometric and visual sets currently drift faster than expected in \
    this page; under investigation.";
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
    fly(view, flight, NOTE, Some(&plan.link)).await;
}
