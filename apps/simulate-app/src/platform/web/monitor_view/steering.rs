//! Whose navigation the flight is flown on, stated once and plainly: the set Pilotage's
//! mission steers on, that the map's aircraft is that set's estimate, what marks the
//! simulator's truth, and the links that fly the same plan on another set.
//!
//! The set is fixed when the flight starts: the link names it, and a link flies the
//! same flight every time. So another set is offered as another link, not as a switch.

use web_sys::{Document, Element};

use super::super::section::append;

/// The set a flight steers on and the other sets it could.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Steering {
    /// The steering set's name.
    pub set: String,
    /// The run's other sets, each with the link that flies on it.
    pub others: Vec<(String, String)>,
}

/// What the steering block says after the set's name.
const MEANS: &str = "Pilotage's mission steers on this set's position. The map's aircraft is \
    this estimate, inside the ring of the one-sigma the set states for it. When the set's \
    solution degrades, Pilotage flies the same solution on as dead reckoning and reports \
    it; the map's aircraft stays that solution.";
/// What it says of the other marks.
const OTHERS: &str = "The yellow crossed circle is the simulator groundtruth. The simulator \
    groundtruth is known only to the simulator; the onboard software never sees or uses \
    it. The flight \
    controller's own estimate is drawn as flight-controller, one more track to compare: \
    Aviate flies the surfaces on it, the mission does not steer on it.";
/// What it says of the HSI's label.
const HSI: &str = "The HSI's course, deviation and distance are Pilotage's, computed from \
    this set. Its source reads FMS with the position Pilotage says it steers on: GPS, VIS \
    for the terrain camera, INS, BLND for several together, or DR in amber for dead \
    reckoning, and DGRD or EXCL beside it when Pilotage grades the position so.";

/// The block at the end of `parent`, the steering set drawn in `colour`.
pub fn steering_block(
    document: &Document,
    parent: &Element,
    steering: &Steering,
    colour: &str,
) -> Option<Element> {
    let block = append(document, parent, "div", "steering")?;
    let head = append(document, &block, "p", "steering-head")?;
    append(document, &head, "span", "")?.set_text_content(Some("Flying on "));
    let swatch = append(document, &head, "span", "swatch")?;
    swatch
        .set_attribute("style", &format!("background: {colour}"))
        .ok();
    append(document, &head, "strong", "")?.set_text_content(Some(&steering.set));
    for text in [MEANS, OTHERS, HSI] {
        append(document, &block, "p", "detail")?.set_text_content(Some(text));
    }
    if !steering.others.is_empty() {
        let line = append(document, &block, "p", "detail steering-others")?;
        append(document, &line, "span", "")?
            .set_text_content(Some("Fly the same plan on another set, as a new flight: "));
        for (set, url) in &steering.others {
            let anchor = append(document, &line, "a", "")?;
            anchor.set_attribute("href", url).ok();
            anchor.set_attribute("target", "_blank").ok();
            anchor.set_attribute("rel", "noopener").ok();
            anchor.set_text_content(Some(set));
        }
    }
    Some(block)
}
