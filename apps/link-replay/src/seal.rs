//! Seals a simulation link's navigation map package through [`DiskCachedTransport`].

use navigate_data::MemoryStore;
use sim_link::SimulationLink;
use sim_link_host::{HostedRequest, budget_bytes, package_camera, package_route_msl};
use sim_navdata::{SealedPackage, SourceSet, plan_package, prefetch, read_ground, seal_package};
use tracing::{debug, info};

use crate::ReplayError;
use crate::cache::DiskCachedTransport;

/// Plans, fetches (or reads from local disk cache), and seals the terrain +
/// orthoimagery map package for `link`, returning `Ok(None)` when `no_camera`
/// is set or no visual navigation set is present.
///
/// # Errors
///
/// Returns [`ReplayError`] if route ground reading, budget checking, tile
/// retrieval, or `Navigate` package sealing fails.
pub fn seal_blocking(
    link: &SimulationLink,
    hosted: &HostedRequest,
    transport: &DiskCachedTransport,
    no_camera: bool,
) -> Result<Option<SealedPackage>, ReplayError> {
    if no_camera {
        return Ok(None);
    }
    let Some(camera) = package_camera(hosted) else {
        return Ok(None);
    };
    pollster::block_on(async {
        let sources = SourceSet::default();
        let mut store = MemoryStore::default();
        let route = package_route_msl(link)?;
        let track: Vec<[f64; 2]> = route.iter().map(|[lat, lon, _]| [*lat, *lon]).collect();
        let ground = read_ground(&track, &sources, transport, &mut store).await?;
        let plan = plan_package(&route, camera, &sources, &ground)?;
        let budget = budget_bytes(link);
        plan.within(budget)?;
        info!(
            tiles = plan.tile_count(),
            cache_dir = %transport.root().display(),
            "preparing visual map package"
        );
        let keys = plan.keys(&sources);
        let fetched = prefetch(&keys, &sources, budget, transport, &mut store, &mut |p| {
            if p.done > 0 && (p.done % 250 == 0 || p.done == p.total) {
                info!(
                    done = p.done,
                    total = p.total,
                    gaps = p.gaps,
                    "tile prefetch progress"
                );
            } else {
                debug!(
                    done = p.done,
                    total = p.total,
                    gaps = p.gaps,
                    "tile prefetch"
                );
            }
        })
        .await?;
        let release = link.time.compact();
        let sealed = seal_package(&plan, &sources, &release, &fetched.absent, &mut store).await?;
        let stats = transport.stats();
        info!(
            identity = %sealed.summary.identity,
            gaps = fetched.absent.len(),
            cache_hits = stats.hits,
            cache_misses = stats.misses,
            network_fetches = stats.network_fetches,
            "map package sealed"
        );
        Ok(Some(sealed))
    })
}
