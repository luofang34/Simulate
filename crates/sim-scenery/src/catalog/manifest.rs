use std::collections::BTreeSet;
use std::path::{Component, Path};

use image::ImageReader;
use serde::Deserialize;
use sha2::{Digest, Sha256};

use crate::{CatalogScenery, ImageryRaster, SceneryError, TerrariumTile, TileId};

#[cfg(test)]
mod tests;

const FORMAT: &str = "photo-reference-catalog";
const VERSION: u32 = 1;
const MAX_TILES: usize = 100_000;
const MAX_ZOOM: u32 = 22;
const MANIFEST_NAME: &str = "catalog.json";

/// The manifest of a `Navigate` reference catalog.
#[derive(Debug, Deserialize)]
struct Manifest {
    format: String,
    version: u32,
    complete: bool,
    tiles: Vec<ManifestTile>,
}

#[derive(Debug, Deserialize)]
struct ManifestTile {
    kind: String,
    xyz: [u32; 3],
    path: String,
    sha256: String,
    bytes: usize,
}

impl CatalogScenery {
    /// Loads a `Navigate` reference catalog from its manifest text.
    /// `read_tile` fetches a tile's file by its manifest path, so a browser
    /// can supply bytes from anywhere; every tile must match the size and
    /// SHA-256 digest the manifest records.
    pub fn from_manifest(
        manifest_json: &str,
        read_tile: &mut dyn FnMut(&str) -> Result<Vec<u8>, SceneryError>,
    ) -> Result<Self, SceneryError> {
        let manifest: Manifest =
            serde_json::from_str(manifest_json).map_err(|err| invalid(err.to_string()))?;
        if manifest.format != FORMAT || manifest.version != VERSION {
            return Err(invalid(format!(
                "format {} version {} is not {FORMAT} version {VERSION}",
                manifest.format, manifest.version
            )));
        }
        if !manifest.complete {
            return Err(invalid("the catalog is marked incomplete".to_owned()));
        }
        if manifest.tiles.is_empty() || manifest.tiles.len() > MAX_TILES {
            return Err(invalid(format!("{} tiles", manifest.tiles.len())));
        }
        let mut catalog = Self::new();
        let mut seen = BTreeSet::new();
        for entry in &manifest.tiles {
            let tile = entry.address()?;
            if !seen.insert((entry.kind.as_str(), tile)) {
                return Err(invalid(format!("duplicate {} tile {:?}", entry.kind, tile)));
            }
            let bytes = read_tile(&entry.path)?;
            entry.verify(&bytes)?;
            match entry.kind.as_str() {
                "imagery" => catalog.insert_imagery(tile, entry.decode_imagery(&bytes)?)?,
                "dem" => catalog.insert_elevation(tile, entry.decode_elevation(&bytes)?)?,
                other => return Err(invalid(format!("unknown tile kind {other}"))),
            }
        }
        Ok(catalog)
    }

    /// Loads the `catalog.json` catalog in `root` from the filesystem.
    pub fn from_catalog_dir_blocking(root: &Path) -> Result<Self, SceneryError> {
        let read = |path: &Path| {
            std::fs::read(path).map_err(|err| SceneryError::Read {
                path: path.display().to_string(),
                detail: err.to_string(),
            })
        };
        let manifest_path = root.join(MANIFEST_NAME);
        let manifest = read(&manifest_path)?;
        let manifest = std::str::from_utf8(&manifest).map_err(|err| invalid(err.to_string()))?;
        Self::from_manifest(manifest, &mut |path| read(&root.join(path)))
    }
}

fn invalid(detail: String) -> SceneryError {
    SceneryError::InvalidCatalog { detail }
}

impl ManifestTile {
    fn address(&self) -> Result<TileId, SceneryError> {
        let [zoom, x, y] = self.xyz;
        let inside_root = Path::new(&self.path)
            .components()
            .all(|part| matches!(part, Component::Normal(_) | Component::CurDir));
        if !inside_root {
            return Err(invalid(format!("{} leaves the catalog root", self.path)));
        }
        if zoom > MAX_ZOOM || x >= 1 << zoom || y >= 1 << zoom {
            return Err(invalid(format!("tile {zoom}/{x}/{y} is out of range")));
        }
        Ok(TileId {
            zoom: zoom as u8,
            x,
            y,
        })
    }

    fn corrupt(&self, detail: String) -> SceneryError {
        SceneryError::CorruptTile {
            path: self.path.clone(),
            detail,
        }
    }

    fn verify(&self, bytes: &[u8]) -> Result<(), SceneryError> {
        if bytes.len() != self.bytes {
            return Err(self.corrupt(format!(
                "{} bytes where the manifest records {}",
                bytes.len(),
                self.bytes
            )));
        }
        let digest = hex_digest(bytes);
        if !digest.eq_ignore_ascii_case(&self.sha256) {
            return Err(self.corrupt(format!(
                "SHA-256 {digest} where the manifest records {}",
                self.sha256
            )));
        }
        Ok(())
    }

    fn decode(&self, bytes: &[u8]) -> Result<image::DynamicImage, SceneryError> {
        ImageReader::new(std::io::Cursor::new(bytes))
            .with_guessed_format()
            .map_err(|err| self.corrupt(err.to_string()))?
            .decode()
            .map_err(|err| self.corrupt(err.to_string()))
    }

    fn decode_imagery(&self, bytes: &[u8]) -> Result<ImageryRaster, SceneryError> {
        let pixels = self.decode(bytes)?.into_rgba8();
        ImageryRaster::new(pixels.width(), pixels.height(), pixels.into_raw())
    }

    fn decode_elevation(&self, bytes: &[u8]) -> Result<TerrariumTile, SceneryError> {
        let pixels = self.decode(bytes)?.into_rgb8();
        TerrariumTile::from_rgb(pixels.width(), pixels.height(), pixels.as_raw())
    }
}

fn hex_digest(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}
