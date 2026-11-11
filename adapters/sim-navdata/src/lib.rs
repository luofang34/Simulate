//! The host side of Navigate's navigation-data contract.
//!
//! Navigate plans, checks and seals a map package and never fetches; a host
//! supplies the ports. This crate is that host for Simulate, the same on
//! the desktop and in the browser:
//!
//! - [`SourceSet`] and [`TileSource`]: the registry of tile sources, each
//!   with its URL template, attribution, licence, encoding and limits;
//! - [`TileTransport`]: one HTTP GET, the only thing a platform supplies to
//!   reach a source (`browser::BrowserTransport` on wasm32);
//! - a persistent tile cache over any `navigate-data` store
//!   (`browser::OpfsStore` on wasm32): each tile once at `chunks/<sha256>`
//!   and an index entry by tile key, so a package is rebuilt without
//!   fetching what the store holds;
//! - [`TerrariumGround`] and [`plan_package`]: Navigate's resource model run
//!   for a route and a camera, with the size estimate a budget is judged on;
//! - [`prefetch`]: the planned tiles fetched side by side with progress,
//!   stopping at the storage budget;
//! - [`seal_package`]: Navigate's own `prepare` over the cache, giving the
//!   package identity and the whole package in memory for a navigator.
//!
//! The layer declarations ([`TileSource::layer_spec`]), the planning zoom
//! ([`GROUND_ZOOM`]) and the plan numbers ([`SEARCH_RADIUS_M`],
//! [`DEVIATION_M`], [`MAX_RANGE_M`], [`ELEVATION_OFFSET`]) mirror the host
//! values of Navigate's `visual-pipeline package` tool, because the package
//! identity depends on them. Tests pin them.

#[cfg(target_arch = "wasm32")]
pub mod browser;
mod cache;
mod error;
mod ground;
mod plan;
mod prefetch;
mod seal;
mod source;
mod transport;

#[cfg(test)]
mod fixtures;

pub use cache::{CachedTile, index_path, lookup, record};
pub use error::NavdataError;
pub use ground::{GROUND_CORRIDOR_M, GROUND_ZOOM, TerrariumGround, ground_tiles, read_ground};
pub use plan::{
    DEVIATION_M, ELEVATION_OFFSET, MAX_RANGE_M, PackagePlan, SEARCH_RADIUS_M, plan_package,
    planned_camera,
};
pub use prefetch::{FETCH_ATTEMPTS, FETCHES_AT_ONCE, PrefetchOutcome, PrefetchProgress, prefetch};
pub use seal::{SealedPackage, seal_package};
pub use source::{SourceSet, TileSource, tile_url};
pub use transport::{HttpAnswer, TileTransport, fetched};
