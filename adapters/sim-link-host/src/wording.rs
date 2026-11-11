//! The link and its plan in plain language, for a page or a terminal.

use sim_link::{
    HeightDatum, Means, NavSet, Outage, Refusal, Sensor, SimulationLink, Turn, resolve_escapes,
};

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

/// The plan of `link` in plain language: the route, each waypoint, the
/// start, the sets compared, the outages, the Sun and the map package.
pub fn plan_lines(
    link: &SimulationLink,
    request: &HostedRequest,
    summary: &PlanSummary,
) -> Vec<PlanLine> {
    let mut lines = vec![line(
        "Route",
        format!(
            "{} waypoints, {:.1} km, about {:.0} min at {} m/s ({})",
            link.route.len(),
            summary.length_m / 1000.0,
            summary.duration_s / 60.0,
            request.trial.vehicle.cruise_speed_m_s,
            link.vehicle.key()
        ),
    )];
    lines.extend(waypoint_lines(link, request));
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

fn waypoint_lines<'a>(
    link: &'a SimulationLink,
    request: &'a HostedRequest,
) -> impl Iterator<Item = PlanLine> + 'a {
    link.route
        .iter()
        .zip(&request.trial.waypoints)
        .enumerate()
        .map(|(index, (waypoint, spec))| {
            let height = link
                .height_or_default(waypoint.height)
                .map_or_else(String::new, |height| height.text(0));
            let turn = match waypoint.turn {
                Turn::FlyBy => "fly-by",
                Turn::FlyOver => "fly-over",
            };
            line(
                &format!("Waypoint {}", index.saturating_add(1)),
                format!(
                    "{}  {}, {}  {height} m {}, {turn}",
                    spec.ident,
                    waypoint.lat.text(6),
                    waypoint.lon.text(6),
                    datum_words(link.datum)
                ),
            )
        })
}

fn start_words(link: &SimulationLink) -> String {
    let time = link.time.iso8601().replace('T', " ").replace('Z', " UTC");
    match link.start {
        None => format!("{time}, over the first waypoint, heading for the second"),
        Some(start) => {
            let height = link
                .height_or_default(start.height)
                .map_or_else(String::new, |height| height.text(0));
            let heading = start.heading.map_or_else(
                || "heading for the first waypoint".to_owned(),
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
