//! What the page does with a simulation link in its address: decode it,
//! show its plan or why it is refused, build its map package, and say what
//! of the flight this build of the page cannot do yet.

use sim_link::decode_url;
use sim_link_host::{HostError, hosted_request, plan_lines, plan_summary, refusal_lines};
use tracing::{info, warn};

use super::package_run::{LinkPlan, build_package};
use super::progress_surface::ProgressSurface;

/// What stands in the flight's place until the vehicle's own software runs
/// in the browser.
const FLIGHT_NOT_YET: &str = "The flight is flown by the vehicle's own software: Pilotage's \
    mission host, Aviate's flight controller for the C172 and Navigate's navigator. They are \
    not linked into this build of the page, so it stops here. The plan and the map package \
    above are what they are handed.";

/// Shows the link in the page's address on `surface`. Returns whether the
/// address carried one, valid or not: such a page is that simulation's, and
/// the free-flight cockpit does not start on it.
pub fn show_link(surface: ProgressSurface) -> Result<(), ProgressSurface> {
    let Some(href) = web_sys::window().and_then(|window| window.location().href().ok()) else {
        return Err(surface);
    };
    match decode_url(&href) {
        Ok(None) => return Err(surface),
        Ok(Some(link)) => match planned(link) {
            Ok(plan) => {
                info!(
                    waypoints = plan.link.route.len(),
                    "showing the plan of the link"
                );
                let lines = plan_lines(&plan.link, &plan.request, &plan.summary);
                surface.lines("plan", "Flight plan from the link", &lines);
                wasm_bindgen_futures::spawn_local(continue_with(surface, plan));
            }
            Err(error) => {
                warn!(%error, "the link decodes but cannot be planned");
                surface.fail("This link cannot be flown", &error.to_string());
            }
        },
        Err(refusal) => {
            warn!(%refusal, "the link in the address is refused");
            let fragment = href.split_once('#').map_or("", |(_, fragment)| fragment);
            surface.lines(
                "refused",
                "This link is refused",
                &refusal_lines(fragment, &refusal),
            );
        }
    }
    Ok(())
}

fn planned(link: sim_link::SimulationLink) -> Result<LinkPlan, HostError> {
    let request = hosted_request(&link)?;
    let summary = plan_summary(&link, &request)?;
    Ok(LinkPlan {
        link,
        request,
        summary,
    })
}

/// After the plan: the map package, then the flight.
async fn continue_with(surface: ProgressSurface, plan: LinkPlan) {
    let Some(package) = surface.section("Map package") else {
        return;
    };
    let sealed = build_package(&package, &plan).await;
    let needs_package = plan.request.camera.is_some();
    if needs_package && sealed.is_none() {
        return;
    }
    if let Some(flight) = surface.section("Flight") {
        flight.settled(FLIGHT_NOT_YET);
    }
}
