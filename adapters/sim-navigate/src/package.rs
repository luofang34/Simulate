//! Map packages for visual navigation: written before departure from a route,
//! read during the flight.
//!
//! [`prepare_package_blocking`] selects the imagery and elevation tiles of a
//! route corridor, takes them from a [`TileFetch`], and writes a
//! self-contained directory: the tiles and a `catalog.json` manifest in the
//! `photo-reference-catalog` format that Navigate and Simulate both read, with
//! a `package` section naming its role, zooms, ground sample distance,
//! coverage and creation inputs. The SHA-256 of the manifest bytes is the
//! package's identity, `navigate_visual::MapRevision::manifest_sha256`.
//!
//! [`MapPackage::open_blocking`] reads a package back, checks every tile
//! against the manifest, and assembles Navigate render scenes around a point.
//! It has no network path: a flight given a package can read nothing else.

mod reader;
mod source;
mod tiles;
mod writer;

#[cfg(test)]
mod tests;

use serde::{Deserialize, Serialize};

pub use reader::{MapPackage, PackageIdentity, SceneWindow, read_identity_blocking};
pub use source::{CatalogDirSource, MemoryTileSource, TileFetch};
pub use tiles::{
    TILE_PIXELS, TileAddress, corridor_tiles, ground_sample_distance_m, tile_lat_lon, tile_xy,
    tiles_around,
};
pub use writer::{PackageInfo, PackageSpec, PackageSummary, prepare_package_blocking};

/// File name of a package manifest.
pub const MANIFEST_NAME: &str = "catalog.json";
/// The `package.format` of a package manifest.
pub const PACKAGE_FORMAT: &str = "simulate.nav-package";

/// The two tile layers of a package.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TileLayer {
    /// Orthoimagery.
    Imagery,
    /// Terrarium elevation, metres above mean sea level.
    Dem,
}

impl TileLayer {
    /// The manifest's `kind` of the layer.
    pub const fn kind(self) -> &'static str {
        match self {
            Self::Imagery => "imagery",
            Self::Dem => "dem",
        }
    }

    /// The layer of a manifest `kind`.
    pub fn from_kind(kind: &str) -> Option<Self> {
        match kind {
            "imagery" => Some(Self::Imagery),
            "dem" | "terrain" => Some(Self::Dem),
            _ => None,
        }
    }
}

/// What a package is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PackageRole {
    /// The navigator's onboard map.
    Onboard,
    /// The simulated world the vehicle camera sees; never onboard.
    World,
}

/// Why a package could not be written or read.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum PackageError {
    /// A file could not be read or written.
    #[error("{path}")]
    Io {
        /// The file.
        path: String,
        /// The system's refusal.
        #[source]
        source: std::io::Error,
    },
    /// A manifest did not parse or serialise.
    #[error("manifest {path}")]
    Manifest {
        /// The manifest.
        path: String,
        /// The parser's refusal.
        #[source]
        source: serde_json::Error,
    },
    /// A tile could not be fetched.
    #[error("fetching {layer:?} {address:?}: {detail}")]
    Fetch {
        /// The layer.
        layer: TileLayer,
        /// The tile.
        address: TileAddress,
        /// The source's message.
        detail: String,
    },
    /// A tile did not decode.
    #[error("decoding {layer:?} {address:?}")]
    Decode {
        /// The layer.
        layer: TileLayer,
        /// The tile.
        address: TileAddress,
        /// The decoder's refusal.
        #[source]
        source: image::ImageError,
    },
    /// The package's contents do not match its manifest, or its manifest
    /// does not match the hash expected of it.
    #[error("package integrity: {detail}")]
    Integrity {
        /// What does not match.
        detail: String,
    },
    /// A request or manifest value is unusable.
    #[error("{detail}")]
    Invalid {
        /// What is wrong.
        detail: String,
    },
}
