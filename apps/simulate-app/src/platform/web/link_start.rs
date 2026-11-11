//! What the page does with a simulation link in its address: decode it and
//! show its plan, or show why it is refused.

use sim_link::decode_url;
use sim_link_host::{HostError, PlanLine, hosted_request, plan_lines, plan_summary, refusal_lines};
use tracing::{info, warn};

use super::progress_surface::ProgressSurface;

/// What a visitor is told under the plan: this page reads the link and
/// stops there.
const NOT_YET: &str = "shows the plan and stops here. Building the map package and flying the \
                       route are not in this build of the page.";

/// Shows the link in the page's address on `surface`. Returns whether the
/// address carried one, valid or not: such a page is that simulation's, and
/// the free-flight cockpit does not start on it.
pub fn show_link(surface: &ProgressSurface) -> bool {
    let Some(href) = web_sys::window().and_then(|window| window.location().href().ok()) else {
        return false;
    };
    match decode_url(&href) {
        Ok(None) => false,
        Ok(Some(link)) => {
            match planned(&link) {
                Ok(mut lines) => {
                    info!(waypoints = link.route.len(), "showing the plan of the link");
                    lines.push(PlanLine {
                        label: "This page".to_owned(),
                        text: NOT_YET.to_owned(),
                    });
                    surface.lines("plan", "Flight plan from the link", &lines);
                }
                Err(error) => {
                    warn!(%error, "the link decodes but cannot be planned");
                    surface.fail("This link cannot be flown", &error.to_string());
                }
            }
            true
        }
        Err(refusal) => {
            warn!(%refusal, "the link in the address is refused");
            let fragment = href.split_once('#').map_or("", |(_, fragment)| fragment);
            surface.lines(
                "refused",
                "This link is refused",
                &refusal_lines(fragment, &refusal),
            );
            true
        }
    }
}

fn planned(link: &sim_link::SimulationLink) -> Result<Vec<PlanLine>, HostError> {
    let request = hosted_request(link)?;
    let summary = plan_summary(link, &request)?;
    Ok(plan_lines(link, &request, &summary))
}
