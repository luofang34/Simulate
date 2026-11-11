//! Pre-departure: the route corridor's tiles over HTTP into the onboard
//! package and the simulated world's imagery. This is the only part of the
//! trial that touches the network; the flight reads the written packages.

use std::path::{Path, PathBuf};

use navigate_locate::{CachedHttpSource, TileKind, TileSource};
use sim_navigate::package::{
    PackageError, PackageSummary, TileAddress, TileFetch, TileLayer, prepare_package_blocking,
};
use sim_navigate::trial::package_specs;
use tracing::info;

use crate::NavTrialError;
use crate::cli::read_request_blocking;

/// Retries of one tile after a failed fetch.
const RETRIES: u32 = 4;

/// Esri World Imagery and AWS Terrarium tiles through Navigate's cached HTTP
/// source, one request at a time.
pub struct HttpTiles {
    inner: CachedHttpSource,
    fetched: u64,
}

impl HttpTiles {
    /// Tiles cached under `cache`.
    pub fn new(cache: PathBuf) -> Self {
        Self {
            inner: CachedHttpSource::new(cache),
            fetched: 0,
        }
    }
}

impl TileFetch for HttpTiles {
    fn fetch(
        &mut self,
        layer: TileLayer,
        address: TileAddress,
    ) -> Result<Option<Vec<u8>>, PackageError> {
        let kind = match layer {
            TileLayer::Imagery => TileKind::Imagery,
            TileLayer::Dem => TileKind::Terrain,
        };
        self.fetched = self.fetched.wrapping_add(1);
        if self.fetched.is_multiple_of(200) {
            info!(tiles = self.fetched, "fetching");
        }
        // A tile server's transient refusals (a 502 under load) are retried
        // a few times with growing pauses, as its usage norms ask, before
        // the preparation gives up.
        let mut attempt = 0_u32;
        loop {
            let fetched = pollster::block_on(self.inner.fetch(
                kind,
                u32::from(address.zoom),
                address.x,
                address.y,
            ));
            match fetched {
                Ok(tile) => return Ok(tile),
                Err(detail) if attempt >= RETRIES => {
                    return Err(PackageError::Fetch {
                        layer,
                        address,
                        detail,
                    });
                }
                Err(detail) => {
                    tracing::warn!(?layer, ?address, detail, attempt, "retrying a tile");
                    std::thread::sleep(std::time::Duration::from_secs(1_u64 << attempt));
                    attempt = attempt.wrapping_add(1);
                }
            }
        }
    }

    fn attribution(&self, layer: TileLayer) -> String {
        match layer {
            TileLayer::Imagery => {
                "Esri, Maxar, Earthstar Geographics, and the GIS User Community".into()
            }
            TileLayer::Dem => "Mapzen / AWS terrain tiles".into(),
        }
    }

    fn identity(&self) -> String {
        self.inner.identity()
    }
}

/// What `prepare` wrote.
#[derive(Debug, Clone, PartialEq)]
pub struct Prepared {
    /// The onboard package.
    pub onboard: PackageSummary,
    /// The world imagery.
    pub world: PackageSummary,
    /// Wall time, seconds.
    pub seconds: f64,
}

/// Writes the onboard package and the world imagery of the request at
/// `request`, fetching through a tile cache at `cache` (beside the onboard
/// package when not given).
pub fn prepare_files_blocking(
    request: &Path,
    onboard: &Path,
    world: &Path,
    cache: Option<&Path>,
) -> Result<Prepared, NavTrialError> {
    let request = read_request_blocking(request)?;
    let (onboard_spec, world_spec) = package_specs(&request)?;
    let cache = cache.map_or_else(|| onboard.with_file_name("tile-cache"), Path::to_path_buf);
    let started = std::time::Instant::now();
    let mut tiles = HttpTiles::new(cache);
    let package = |spec, out: &Path, tiles: &mut HttpTiles| {
        prepare_package_blocking(spec, tiles, out)
            .map_err(|error| NavTrialError::Trial(error.into()))
    };
    let onboard = package(&onboard_spec, onboard, &mut tiles)?;
    info!(?onboard, "onboard package written");
    let world = package(&world_spec, world, &mut tiles)?;
    info!(?world, "world imagery written");
    Ok(Prepared {
        onboard,
        world,
        seconds: started.elapsed().as_secs_f64(),
    })
}
