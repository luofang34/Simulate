//! Where `prepare` takes tiles from. The flight never sees a source: it reads
//! only a written package.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use super::{PackageError, TileAddress, TileLayer};

/// A supplier of encoded tiles for package preparation.
pub trait TileFetch {
    /// The encoded tile at `address` of `layer`, or `None` where the source
    /// has none.
    fn fetch(
        &mut self,
        layer: TileLayer,
        address: TileAddress,
    ) -> Result<Option<Vec<u8>>, PackageError>;

    /// Who serves the tiles, for the manifest's attribution.
    fn attribution(&self, layer: TileLayer) -> String;

    /// Identity of the source, for the manifest's creation inputs.
    fn identity(&self) -> String;
}

/// Tiles of a `photo-reference-catalog` directory already on disk.
#[derive(Debug, Clone)]
pub struct CatalogDirSource {
    root: PathBuf,
    paths: BTreeMap<(TileLayer, TileAddress), String>,
    attribution: BTreeMap<String, String>,
}

#[derive(Debug, Deserialize)]
struct CatalogFile {
    #[serde(default)]
    attribution: BTreeMap<String, String>,
    tiles: Vec<CatalogTile>,
}

#[derive(Debug, Deserialize)]
struct CatalogTile {
    kind: String,
    xyz: [u32; 3],
    path: String,
}

impl CatalogDirSource {
    /// The catalog whose `catalog.json` is in `root`.
    pub fn open_blocking(root: &Path) -> Result<Self, PackageError> {
        let manifest = root.join(super::MANIFEST_NAME);
        let text = std::fs::read_to_string(&manifest).map_err(|source| PackageError::Io {
            path: manifest.display().to_string(),
            source,
        })?;
        let file: CatalogFile =
            serde_json::from_str(&text).map_err(|source| PackageError::Manifest {
                path: manifest.display().to_string(),
                source,
            })?;
        let mut paths = BTreeMap::new();
        for tile in file.tiles {
            let Some(layer) = TileLayer::from_kind(&tile.kind) else {
                continue;
            };
            let [zoom, x, y] = tile.xyz;
            let zoom = u8::try_from(zoom).map_err(|_| PackageError::Invalid {
                detail: format!("zoom {zoom} in {}", manifest.display()),
            })?;
            paths.insert((layer, TileAddress { zoom, x, y }), tile.path);
        }
        Ok(Self {
            root: root.to_path_buf(),
            paths,
            attribution: file.attribution,
        })
    }
}

impl TileFetch for CatalogDirSource {
    fn fetch(
        &mut self,
        layer: TileLayer,
        address: TileAddress,
    ) -> Result<Option<Vec<u8>>, PackageError> {
        let Some(path) = self.paths.get(&(layer, address)) else {
            return Ok(None);
        };
        let full = self.root.join(path);
        std::fs::read(&full)
            .map(Some)
            .map_err(|source| PackageError::Io {
                path: full.display().to_string(),
                source,
            })
    }

    fn attribution(&self, layer: TileLayer) -> String {
        self.attribution
            .get(layer.kind())
            .cloned()
            .unwrap_or_default()
    }

    fn identity(&self) -> String {
        format!("catalog {}", self.root.display())
    }
}

/// Tiles held in memory, for tests and for sources assembled elsewhere.
#[derive(Debug, Clone, Default)]
pub struct MemoryTileSource {
    /// The tiles, by layer and address.
    pub tiles: BTreeMap<(TileLayer, TileAddress), Vec<u8>>,
}

impl TileFetch for MemoryTileSource {
    fn fetch(
        &mut self,
        layer: TileLayer,
        address: TileAddress,
    ) -> Result<Option<Vec<u8>>, PackageError> {
        Ok(self.tiles.get(&(layer, address)).cloned())
    }

    fn attribution(&self, _layer: TileLayer) -> String {
        "in-memory tiles".into()
    }

    fn identity(&self) -> String {
        "memory".into()
    }
}
