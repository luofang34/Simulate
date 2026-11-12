//! The map package of a link: what the package host is asked to plan, and
//! its plan, progress and result in plain language.

use navigate_data::{LayerKind, PlannedCamera};
use sim_link::{HeightDatum, Micro, Milli, SimulationLink};
use sim_navdata::{NavdataError, PackagePlan, PrefetchProgress, SealedPackage, planned_camera};

use crate::request::stated_height_m;
use crate::wording::line;
use crate::{HostError, HostedRequest, PlanLine, PlanSummary, flyable_route};

mod stage;

#[cfg(test)]
mod tests;

pub use stage::{PACKAGE_LINES, PackageStage, package_lines};

/// Bytes in one megabyte of a link's storage budget.
pub const BYTES_PER_MB: u64 = 1_000_000;

/// The storage budget of `link`, bytes.
pub const fn budget_bytes(link: &SimulationLink) -> u64 {
    link.budget_mb as u64 * BYTES_PER_MB
}

/// The points the package is planned along: the start when the link gives
/// one, then the fixes the route flies to, each `[lat, lon, height]` in
/// degrees and metres above mean sea level, the reference of the elevation
/// source.
///
/// # Errors
///
/// Refuses a leg the vehicle's software does not fly yet, and a point that
/// has no height when the link has no default.
pub fn package_route_msl(link: &SimulationLink) -> Result<Vec<[f64; 3]>, HostError> {
    let point = |what: String, lat: Micro, lon: Micro, own: Option<f64>| {
        let stated = stated_height_m(link, own).ok_or(HostError::MissingHeight { what })?;
        let (lat_deg, lon_deg) = (lat.as_f64(), lon.as_f64());
        let msl = match link.datum {
            HeightDatum::MeanSeaLevel => stated,
            HeightDatum::Ellipsoid => {
                stated - sim_geo::egm96_undulation_m(lat_deg.to_radians(), lon_deg.to_radians())
            }
        };
        Ok::<[f64; 3], HostError>([lat_deg, lon_deg, msl])
    };
    let route = flyable_route(link)?;
    let mut points = Vec::with_capacity(route.fixes.len().saturating_add(1));
    if let Some(start) = link.start {
        points.push(point(
            "the start".to_owned(),
            start.lat,
            start.lon,
            start.height.map(Milli::as_f64),
        )?);
    }
    for fix in &route.fixes {
        let what = format!("the fix of leg {}", fix.leg);
        points.push(point(what, fix.lat, fix.lon, fix.height_m)?);
    }
    Ok(points)
}

/// The terrain camera of `request` as the resource model sees it; `None`
/// when no set is visual, and the flight needs no map package.
pub fn package_camera(request: &HostedRequest) -> Option<PlannedCamera> {
    request.camera.map(|camera| {
        planned_camera(
            camera.width_px,
            camera.height_px,
            camera.horizontal_fov_deg,
            camera.depression_deg,
        )
    })
}

/// Megabytes with one decimal below a hundred.
fn megabytes(bytes: u64) -> String {
    let megabytes = bytes as f64 / BYTES_PER_MB as f64;
    if megabytes < 100.0 {
        format!("{megabytes:.1} MB")
    } else {
        format!("{megabytes:.0} MB")
    }
}

const fn layer_words(layer: LayerKind) -> &'static str {
    match layer {
        LayerKind::Imagery => "imagery",
        LayerKind::Elevation => "elevation",
        LayerKind::Unsupported => "unsupported layer",
    }
}

/// The plan of a package: what each leg needs, the tiles of each layer and
/// zoom, and the size against the budget of `link`.
pub fn package_plan_lines(
    link: &SimulationLink,
    summary: &PlanSummary,
    plan: &PackagePlan,
) -> Vec<PlanLine> {
    let mut lines = Vec::new();
    for (index, resources) in plan.resources.legs.iter().enumerate() {
        let ends = summary
            .legs
            .get(index)
            .map_or_else(String::new, |leg| format!("{} to {}: ", leg.from, leg.to));
        let zooms: Vec<String> = resources
            .bands
            .iter()
            .map(|band| format!("{} zoom {}", layer_words(band.layer), band.zoom))
            .collect();
        lines.push(line(
            &format!("Leg {}", index.saturating_add(1)),
            format!(
                "{ends}{:.1} km, at least {:.0} m above the ground, strip {:.1} km wide; {}",
                resources.length_m / 1000.0,
                resources.agl_m,
                2.0 * resources.corridor_half_width_m / 1000.0,
                zooms.join(", ")
            ),
        ));
    }
    for size in &plan.size.layers {
        lines.push(line(
            "Tiles",
            format!(
                "{} {} tiles at zoom {}, about {}, at most {}",
                size.tiles,
                layer_words(size.layer),
                size.zoom,
                megabytes(size.estimate_bytes),
                megabytes(size.upper_bytes)
            ),
        ));
    }
    lines.push(line(
        "Package",
        format!(
            "{} tiles, about {}, at most {}; the budget in the link is {}",
            plan.tile_count(),
            megabytes(plan.size.estimate_bytes),
            megabytes(plan.size.upper_bytes),
            megabytes(budget_bytes(link))
        ),
    ));
    lines
}

/// A budget refusal for a reader: what the package needs, what the link
/// allows, and the choices that are the visitor's to make.
pub fn budget_refusal_lines(link: &SimulationLink, error: &NavdataError) -> Vec<PlanLine> {
    let mut lines = vec![line("Reason", error_words(error))];
    if let NavdataError::OverBudget { needed_bytes, .. } = error {
        let enough = needed_bytes.div_ceil(BYTES_PER_MB);
        lines.push(line(
            "Choices",
            format!(
                "Raise the budget: mb={enough} or more in the link (now mb={}). Or shorten \
                 the route. Or fly higher: a higher flight is matched on coarser tiles. This \
                 page does not lower the map detail or cut the route by itself.",
                link.budget_mb
            ),
        ));
    }
    lines
}

/// A package host's refusal in megabytes and plain words.
pub fn error_words(error: &NavdataError) -> String {
    match error {
        NavdataError::OverBudget {
            needed_bytes,
            budget_bytes,
            tiles,
        } => format!(
            "the map package needs up to {} for {tiles} tiles; the budget is {}",
            megabytes(*needed_bytes),
            megabytes(*budget_bytes)
        ),
        NavdataError::BudgetReached {
            budget_bytes,
            bytes,
            done,
            total,
        } => format!(
            "the tiles reached the budget of {} after {done} of {total} ({}); the package was \
             not sealed. The tiles fetched stay in this browser's storage.",
            megabytes(*budget_bytes),
            megabytes(*bytes)
        ),
        other => other.to_string(),
    }
}

/// A sealed package for a reader: its identity and what it holds.
pub fn sealed_lines(sealed: &SealedPackage, fetched: &PrefetchProgress) -> Vec<PlanLine> {
    let mut holds = format!(
        "{} tiles, {}",
        sealed.summary.tiles,
        megabytes(sealed.summary.bytes)
    );
    if sealed.summary.gaps > 0 {
        holds.push_str(&format!(
            "; {} gaps the source does not have",
            sealed.summary.gaps
        ));
    }
    if sealed.repeated_tiles > 0 {
        holds.push_str(&format!(
            "; {} tiles repeat another's bytes (open water, or a provider's \"no imagery\" \
             picture this page does not know)",
            sealed.repeated_tiles
        ));
    }
    vec![
        line("Identity", sealed.summary.identity.to_string()),
        line("Holds", holds),
        line(
            "Fetched",
            format!(
                "{} now, {} from this browser's storage",
                fetched.fetched, fetched.reused
            ),
        ),
        line(
            "In memory",
            format!(
                "{}, opened and every tile checked",
                megabytes(sealed.memory_bytes)
            ),
        ),
    ]
}
