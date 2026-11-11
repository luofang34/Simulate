//! The map package of a link, built in the page: the plan and its budget
//! verdict, the visitor's consent, the fetch with progress, and the sealed
//! package in memory.

use navigate_data::{ReadPort, WritePort};
use sim_link::SimulationLink;
use sim_link_host::{
    BYTES_PER_MB, HostError, HostedRequest, PlanLine, PlanSummary, budget_bytes,
    budget_refusal_lines, package_camera, package_plan_lines, package_route_msl, progress_words,
    sealed_lines, world_line, world_memory_refusal,
};
use sim_navdata::browser::{
    BrowserTransport, OpfsStore, StorageQuota, request_persistence, storage_quota,
};
use sim_navdata::{
    NavdataError, PackagePlan, SealedPackage, SourceSet, plan_package, prefetch, read_ground,
    seal_package,
};
use tracing::{info, warn};

use super::section::Section;

/// The directory of the origin private file system the store lives in.
const STORE_DIRECTORY: &str = "navdata";

/// The link and what was derived from it.
pub struct LinkPlan {
    /// The decoded link.
    pub link: SimulationLink,
    /// Its mapping onto the hosts' request.
    pub request: HostedRequest,
    /// Its legs and Sun.
    pub summary: PlanSummary,
}

/// Why the package was not built.
enum Stop {
    /// The link's route cannot be planned along.
    Link(HostError),
    /// The package host refused or failed.
    Package(NavdataError),
    /// The camera's world, decoded, does not fit the tab: the reason and
    /// the visitor's choices.
    World(Vec<PlanLine>),
    /// The browser's quota cannot hold the plan's upper bound.
    Storage {
        /// The plan's upper bound, bytes.
        needed_bytes: u64,
        /// What the browser still lets this site store, bytes.
        free_bytes: u64,
    },
}

impl From<NavdataError> for Stop {
    fn from(error: NavdataError) -> Self {
        Self::Package(error)
    }
}

impl From<HostError> for Stop {
    fn from(error: HostError) -> Self {
        Self::Link(error)
    }
}

fn text_line(label: &str, text: String) -> PlanLine {
    PlanLine {
        label: label.to_owned(),
        text,
    }
}

/// Builds the map package of `plan` in `section`. Returns the sealed
/// package, or `None` when the flight needs none or it was refused; the
/// section then says which.
pub async fn build_package(section: &Section, plan: &LinkPlan) -> Option<SealedPackage> {
    let Some(camera) = package_camera(&plan.request) else {
        section.settled(
            "No set of this link uses the terrain camera: the flight needs no map package.",
        );
        return None;
    };
    let quota = storage_quota().await;
    let outcome = match OpfsStore::open(STORE_DIRECTORY).await {
        Ok(mut store) => package(section, plan, camera, quota, &mut store).await,
        Err(refusal) => {
            warn!(%refusal, "no origin private file system; packing in memory");
            let note = format!(
                "This browser gives the page no persistent storage ({refusal}): the package is \
                 kept in memory and fetched again after a reload."
            );
            let mut store = navigate_data::MemoryStore::default();
            package(section, plan, camera, quota, &mut store)
                .await
                .inspect(|_| section.settled(&note))
        }
    };
    match outcome {
        Ok(sealed) => Some(sealed),
        Err(stop) => {
            let (heading, lines) = refusal(&plan.link, &stop);
            warn!(heading, "the map package was not built");
            section.refuse(heading, &lines);
            None
        }
    }
}

/// A stop for a reader: the heading and the reason with the choices left.
fn refusal(link: &SimulationLink, stop: &Stop) -> (&'static str, Vec<PlanLine>) {
    match stop {
        Stop::Package(error) => {
            let heading = match error {
                NavdataError::OverBudget { .. } | NavdataError::BudgetReached { .. } => {
                    "The map package does not fit its budget"
                }
                _ => "The map package could not be built",
            };
            (heading, budget_refusal_lines(link, error))
        }
        Stop::World(lines) => (
            "The map package does not fit this tab's memory",
            lines.clone(),
        ),
        Stop::Link(error) => (
            "The map package could not be planned",
            vec![text_line("Reason", error.to_string())],
        ),
        Stop::Storage {
            needed_bytes,
            free_bytes,
        } => (
            "This browser's storage cannot hold the map package",
            vec![text_line(
                "Reason",
                format!(
                    "the package needs up to {} MB and this site may store {} MB more. Free \
                     storage for this site in the browser, or shorten the route.",
                    megabytes(*needed_bytes).saturating_add(1),
                    megabytes(*free_bytes)
                ),
            )],
        ),
    }
}

/// Plans, asks, fetches and seals over one store.
async fn package<S: ReadPort + WritePort>(
    section: &Section,
    plan: &LinkPlan,
    camera: navigate_data::PlannedCamera,
    quota: Option<StorageQuota>,
    store: &mut S,
) -> Result<SealedPackage, Stop> {
    let sources = SourceSet::default();
    let transport = BrowserTransport;
    let route = package_route_msl(&plan.link)?;
    let track: Vec<[f64; 2]> = route.iter().map(|[lat, lon, _]| [*lat, *lon]).collect();
    section.working(
        "Reading the terrain height along the route: a few coarse elevation tiles, which the \
         plan needs before it can say what the package holds.",
    );
    let ground = read_ground(&track, &sources, &transport, store).await?;
    let package_plan = plan_package(&route, camera, &sources, &ground)?;
    let budget = budget_bytes(&plan.link);
    let mut lines = package_plan_lines(&plan.link, &plan.summary, &package_plan);
    lines.push(world_line(&package_plan, &sources));
    lines.extend(storage_line(quota));
    lines.push(text_line("Sources", sources_words(&sources)));
    section.lines(&lines);
    package_plan.within(budget)?;
    storage_verdict(&package_plan, quota)?;
    if let Some(refusal) = world_memory_refusal(&package_plan, &sources) {
        return Err(Stop::World(refusal));
    }
    if plan.link.auto_start {
        info!("the link asks to build the map package without asking");
    } else {
        section.settled("The plan is above. Nothing of the package has been fetched yet.");
        section.clicked("Build the map package").await;
    }
    let kept = request_persistence().await;
    info!(
        kept,
        tiles = package_plan.tile_count(),
        "fetching the map package"
    );
    let keys = package_plan.keys(&sources);
    let fetched = prefetch(
        &keys,
        &sources,
        budget,
        &transport,
        store,
        &mut |progress| {
            let fraction = progress.done as f64 / progress.total.max(1) as f64;
            section.progress(fraction, &progress_words(progress));
        },
    )
    .await?;
    section.working("Sealing the package: Navigate checks every tile and writes the manifest.");
    let release = plan.link.time.compact();
    let sealed = seal_package(&package_plan, &sources, &release, &fetched.absent, store).await?;
    info!(identity = %sealed.summary.identity, "the map package is sealed");
    lines.extend(sealed_lines(&sealed, &fetched.progress));
    section.lines(&lines);
    section.settled("The map package is sealed and loaded in memory for the navigator.");
    Ok(sealed)
}

fn megabytes(bytes: u64) -> u64 {
    bytes / BYTES_PER_MB
}

/// What the browser lets the page store, when it says.
fn storage_line(quota: Option<StorageQuota>) -> Option<PlanLine> {
    let quota = quota?;
    let kept = if quota.persistent {
        "the browser has promised to keep it"
    } else {
        "the browser may clear it when the disk is short"
    };
    Some(text_line(
        "Browser storage",
        format!(
            "{} MB allowed to this site, {} MB used; {kept}",
            megabytes(quota.quota_bytes?),
            megabytes(quota.usage_bytes?)
        ),
    ))
}

/// Refuses a plan whose upper bound the browser's quota cannot hold.
fn storage_verdict(plan: &PackagePlan, quota: Option<StorageQuota>) -> Result<(), Stop> {
    match quota.and_then(|quota| quota.free_bytes()) {
        Some(free_bytes) if plan.size.upper_bytes > free_bytes => Err(Stop::Storage {
            needed_bytes: plan.size.upper_bytes,
            free_bytes,
        }),
        _ => Ok(()),
    }
}

fn sources_words(sources: &SourceSet) -> String {
    format!(
        "imagery {}; elevation {}",
        sources.imagery.attribution, sources.elevation.attribution
    )
}
