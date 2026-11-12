//! The link and its plan in plain language, for a page or a terminal.

use sim_link::{
    HeightDatum, Means, NavSet, Outage, Refusal, Sensor, SimulationLink, Spoof, TurnDirection,
    resolve_escapes,
};

use crate::request::stated_height_m;
use crate::{CameraGeometry, HostedRequest, Light, PlanSummary};

#[cfg(test)]
mod tests;

/// Characters of the link shown each side of the one a refusal names.
const EXCERPT_REACH: usize = 36;

/// One labelled line of a plan.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlanLine {
    /// What the line is about.
    pub label: String,
    /// What it says.
    pub text: String,
}

pub(crate) fn line(label: &str, text: String) -> PlanLine {
    PlanLine {
        label: label.to_owned(),
        text,
    }
}

/// What a sensor in outage does not give.
const fn withheld(sensor: Sensor) -> &'static str {
    match sensor {
        Sensor::Gnss => "satellite receiver gives no position",
        Sensor::Baro => "barometer gives no altitude",
        Sensor::Stars => "star camera gives no frames",
        Sensor::Cam => "terrain camera gives no frames",
    }
}

/// Each span of `outage` as a sentence: what the sensor does not give, from
/// when to when, in seconds from the start of the flight.
pub fn outage_words(outage: &Outage) -> Vec<String> {
    outage
        .spans
        .iter()
        .map(|span| {
            let until = span.end.map_or_else(
                || "the end of the flight".to_owned(),
                |end| format!("{} s", end.text(0)),
            );
            format!(
                "{} from {} s to {until}",
                withheld(outage.sensor),
                span.start.text(0)
            )
        })
        .collect()
}

/// Each span of `spoof` as a sentence: what the spoofer does to the
/// receiver's fixes, from when to when.
pub fn spoof_words(spoof: &Spoof) -> Vec<String> {
    spoof
        .spans
        .iter()
        .map(|span| {
            let until = span.end.map_or_else(
                || "the end of the flight".to_owned(),
                |end| format!("{} s", end.text(0)),
            );
            format!(
                "satellite receiver's position dragged {} m/s toward {}° true from {} s to {until}",
                spoof.rate.text(0),
                spoof.bearing.text(0),
                span.start.text(0)
            )
        })
        .collect()
}

fn set_words(set: &NavSet) -> String {
    let means: Vec<&str> = set.0.iter().map(|means| means.key()).collect();
    means.join(" + ")
}

const fn datum_words(datum: HeightDatum) -> &'static str {
    match datum {
        HeightDatum::MeanSeaLevel => "above mean sea level",
        HeightDatum::Ellipsoid => "above the ellipsoid",
    }
}

/// The plan of `link` in plain language: the route, each leg, the hold, the
/// start, the sets compared, the outages, the Sun and the map package.
pub fn plan_lines(
    link: &SimulationLink,
    request: &HostedRequest,
    summary: &PlanSummary,
) -> Vec<PlanLine> {
    let mut lines = vec![line(
        "Route",
        format!(
            "{} legs to {} fixes, {:.1} km, about {:.0} min at {} m/s ({})",
            link.route.len(),
            request.route.fixes.len(),
            summary.length_m / 1000.0,
            summary.duration_s / 60.0,
            request.trial.vehicle.cruise_speed_m_s,
            link.vehicle.key()
        ),
    )];
    lines.extend(leg_lines(link, request));
    lines.push(line("Hold", hold_words(request)));
    if link.route.iter().any(|leg| leg.rnp.is_some()) {
        let rnp = "stated on the route; the vehicle's software does not monitor it yet";
        lines.push(line("RNP", rnp.to_owned()));
    }
    lines.push(line("Starts", start_words(link)));
    let sets: Vec<String> = link.sets.iter().map(set_words).collect();
    lines.push(line("Compares", sets.join(";  ")));
    lines.push(line(
        "Flies on",
        link.steering().map(set_words).unwrap_or_default(),
    ));
    if link.outages.is_empty() {
        let none = "none: every sensor is available for the whole flight";
        lines.push(line("Outage", none.to_owned()));
    }
    for outage in &link.outages {
        lines.extend(
            outage_words(outage)
                .into_iter()
                .map(|words| line("Outage", words)),
        );
    }
    if let Some(spoof) = &link.spoof {
        lines.extend(spoof_words(spoof).into_iter().map(|w| line("Spoofing", w)));
    }
    lines.extend(sun_words(link, summary).map(|words| line("Sun", words)));
    if let Some(camera) = request.camera {
        lines.push(line("Camera", camera_words(camera, link.camera.is_none())));
    }
    lines.push(line("Sensor seed", link.seed.to_string()));
    let then = if link.auto_start {
        "built and flown without asking"
    } else {
        "shown as a plan first, built on a click"
    };
    lines.push(line(
        "Map package",
        format!("at most {} MB, {then}", link.budget_mb),
    ));
    lines
}

fn camera_words(camera: CameraGeometry, vehicles_own: bool) -> String {
    let looking = if camera.depression_deg == 90.0 {
        "looking straight down".to_owned()
    } else {
        format!("looking {}° below the horizon", camera.depression_deg)
    };
    let rate = if camera.rate_hz == 1.0 {
        "1 frame a second".to_owned()
    } else {
        format!("{} frames a second", camera.rate_hz)
    };
    let whose = if vehicles_own {
        " (the vehicle's own)"
    } else {
        ""
    };
    format!(
        "{} x {} pixels, {}° across, {looking}, {rate}{whose}",
        camera.width_px, camera.height_px, camera.horizontal_fov_deg
    )
}

fn leg_lines<'a>(
    link: &'a SimulationLink,
    request: &'a HostedRequest,
) -> impl Iterator<Item = PlanLine> + 'a {
    request
        .route
        .fixes
        .iter()
        .zip(&request.trial.waypoints)
        .map(|(fix, spec)| {
            let code = link
                .route
                .get(fix.leg.saturating_sub(1))
                .map_or("", |leg| leg.path.code());
            let height = stated_height_m(link, fix.height_m).map_or_else(String::new, metres_text);
            let turn = if fix.fly_over { "fly-over" } else { "fly-by" };
            let roles: Vec<&str> = fix.roles.iter().map(|role| role.word()).collect();
            let roles = if roles.is_empty() {
                String::new()
            } else {
                format!(", {}", roles.join(" "))
            };
            line(
                &format!("Leg {}", fix.leg),
                format!(
                    "{code} {}  {}, {}  {height} m {}, {turn}{roles}",
                    spec.ident,
                    fix.lat.text(6),
                    fix.lon.text(6),
                    datum_words(link.datum)
                ),
            )
        })
}

/// Metres with at most one decimal and no trailing `.0`.
fn metres_text(metres: f64) -> String {
    let text = format!("{metres:.1}");
    text.strip_suffix(".0")
        .map_or_else(|| text.clone(), str::to_owned)
}

fn hold_words(request: &HostedRequest) -> String {
    let Some(hold) = request.hold else {
        return "the standard right-hand hold at the last fix, flown when the route ends"
            .to_owned();
    };
    let turns = match hold.stated.turn {
        TurnDirection::Left => "left",
        TurnDirection::Right => "right",
    };
    let at = hold.stated.height_m.map_or_else(String::new, |height| {
        format!(", at {} m", metres_text(height))
    });
    format!(
        "at the last fix, {turns} turns, inbound course {:.1}° true, {} s legs{at}",
        hold.stated.inbound_course_deg, hold.stated.leg_time_s
    )
}

fn start_words(link: &SimulationLink) -> String {
    let time = link.time.iso8601().replace('T', " ").replace('Z', " UTC");
    match link.start {
        None => format!("{time}, over the first fix, heading for the second"),
        Some(start) => {
            let height = stated_height_m(link, start.height.map(sim_link::Milli::as_f64))
                .map_or_else(String::new, metres_text);
            let heading = start.heading.map_or_else(
                || "heading for the first fix".to_owned(),
                |heading| format!("heading {}° true", heading.text(0)),
            );
            format!(
                "{time}, at {}, {}  {height} m {}, {heading}",
                start.lat.text(6),
                start.lon.text(6),
                datum_words(link.datum)
            )
        }
    }
}

/// The Sun along the route, and what it means for the means of the link
/// that need daylight or a dark sky.
fn sun_words(link: &SimulationLink, summary: &PlanSummary) -> Option<String> {
    let elevations = || summary.sun.iter().map(|sample| sample.elevation_deg);
    let low = elevations().reduce(f64::min)?;
    let high = elevations().reduce(f64::max)?;
    let (darkest, brightest) = summary.light_range()?;
    let sky = match (darkest, brightest) {
        (Light::Day, _) => "daylight for the whole route",
        (_, Light::Night) => "night for the whole route",
        (Light::Twilight, Light::Twilight) => "twilight for the whole route",
        (Light::Night, Light::Twilight) => "night and twilight along the route",
        (_, Light::Day) => "the Sun crosses the horizon along the route",
    };
    let uses = |means: Means| link.sets.iter().any(|set| set.contains(means));
    let mut words = format!("{low:.0}° to {high:.0}° above the horizon: {sky}.");
    // Daylight and night are stated, not each sensor's own threshold: that
    // is the sensor model's, judged frame by frame in flight.
    if uses(Means::Visual) {
        words.push_str(match (darkest, brightest) {
            (Light::Day, _) => " The terrain camera needs daylight and has it.",
            (_, Light::Day) => {
                " The terrain camera needs daylight and has it on part of the route."
            }
            _ => " The terrain camera needs daylight and has none.",
        });
    }
    if uses(Means::Stars) {
        words.push_str(match (darkest, brightest) {
            (_, Light::Night) => " The star camera needs night and has it.",
            (Light::Night, _) => " The star camera needs night and has it on part of the route.",
            _ => " The star camera needs night and has none.",
        });
    }
    Some(words)
}

/// A refusal of the link `fragment` (the text after `#`) for a reader: what
/// is refused, and the stretch of the link around the character at fault
/// with a caret under it.
pub fn refusal_lines(fragment: &str, refusal: &Refusal) -> Vec<PlanLine> {
    let mut lines = vec![
        line("Parameter", refusal.parameter.to_string()),
        line("Reason", refusal.reason.to_string()),
    ];
    if let Some(item) = refusal.item {
        lines.insert(1, line("Entry", item.to_string()));
    }
    if let Ok(resolved) = resolve_escapes(fragment) {
        let from = refusal.offset.saturating_sub(EXCERPT_REACH);
        let excerpt: String = resolved
            .chars()
            .skip(from)
            .take(EXCERPT_REACH.saturating_mul(2))
            .collect();
        let caret = " ".repeat(refusal.offset.saturating_sub(from));
        lines.push(line(
            "Where",
            format!(
                "character {}\n{excerpt}\n{caret}^",
                refusal.offset.saturating_add(1)
            ),
        ));
    }
    lines
}
