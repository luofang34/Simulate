use std::collections::BTreeMap;
use std::path::Path;

use navigate_visual::{LocalFrame, MapRevision};
use navigate_visual_render::{AtlasLayout, ImageAtlas, RenderScene, TerrainAtlas};
use sim_scenery::{CatalogVerticalDatum, decode_terrarium};

use super::tiles::{TILE_PIXELS, TileAddress, tile_lat_lon, tiles_around};
use super::writer::{CATALOG_FORMAT, Manifest, PackageInfo, sha256_hex as digest_hex};
use super::{MANIFEST_NAME, PACKAGE_FORMAT, PackageError, TileLayer};

/// Samples of one decoded tile.
const TILE_SAMPLES: usize = (TILE_PIXELS * TILE_PIXELS) as usize;

/// A package read back and verified: decoded imagery as luma and elevation
/// as ellipsoidal heights, ready to assemble render scenes.
#[derive(Debug, Clone)]
pub struct MapPackage {
    info: PackageInfo,
    manifest_sha256: String,
    imagery_zoom: u8,
    terrain_zoom: u8,
    imagery: BTreeMap<TileAddress, Vec<u8>>,
    terrain: BTreeMap<TileAddress, Vec<f32>>,
}

/// A render scene around a point, with the share of its imagery the package
/// covers.
pub struct SceneWindow {
    /// Navigate's scene, in a WGS84 east-north-up frame at the point.
    pub scene: RenderScene,
    /// Covered fraction of the window's imagery pixels, in `[0, 1]`.
    pub imagery_coverage: f64,
}

/// A package's identity, read from its manifest without loading tiles.
#[derive(Debug, Clone, PartialEq)]
pub struct PackageIdentity {
    /// SHA-256 of the manifest bytes.
    pub manifest_sha256: String,
    /// The manifest's package section.
    pub info: PackageInfo,
    /// Imagery zoom.
    pub imagery_zoom: u8,
    /// Elevation zoom.
    pub terrain_zoom: u8,
    /// Imagery tiles listed.
    pub imagery_tiles: usize,
    /// Elevation tiles listed.
    pub terrain_tiles: usize,
    /// West, south, east, north, degrees.
    pub bbox: [f64; 4],
}

/// Reads the identity of the package in `dir`.
pub fn read_identity_blocking(dir: &Path) -> Result<PackageIdentity, PackageError> {
    let manifest_path = dir.join(MANIFEST_NAME);
    let bytes = std::fs::read(&manifest_path).map_err(|source| PackageError::Io {
        path: manifest_path.display().to_string(),
        source,
    })?;
    let manifest: Manifest =
        serde_json::from_slice(&bytes).map_err(|source| PackageError::Manifest {
            path: manifest_path.display().to_string(),
            source,
        })?;
    let count = |kind: TileLayer| {
        manifest
            .tiles
            .iter()
            .filter(|t| TileLayer::from_kind(&t.kind) == Some(kind))
            .count()
    };
    Ok(PackageIdentity {
        manifest_sha256: digest_hex(&bytes),
        imagery_zoom: manifest.configuration.imagery_zoom,
        terrain_zoom: manifest.configuration.dem_zoom,
        imagery_tiles: count(TileLayer::Imagery),
        terrain_tiles: count(TileLayer::Dem),
        bbox: manifest.configuration.bbox,
        info: manifest.package,
    })
}

impl MapPackage {
    /// Reads the package in `dir`. Every tile must match the size and SHA-256
    /// the manifest records, and the manifest's own SHA-256 must equal
    /// `expected_sha256` when one is given.
    pub fn open_blocking(dir: &Path, expected_sha256: Option<&str>) -> Result<Self, PackageError> {
        let manifest_path = dir.join(MANIFEST_NAME);
        let io = |path: &Path| {
            let path = path.display().to_string();
            move |source| PackageError::Io { path, source }
        };
        let bytes = std::fs::read(&manifest_path).map_err(io(&manifest_path))?;
        let manifest_sha256 = digest_hex(&bytes);
        if let Some(expected) = expected_sha256
            && expected != manifest_sha256
        {
            return Err(PackageError::Integrity {
                detail: format!("manifest SHA-256 {manifest_sha256}, expected {expected}"),
            });
        }
        let manifest: Manifest =
            serde_json::from_slice(&bytes).map_err(|source| PackageError::Manifest {
                path: manifest_path.display().to_string(),
                source,
            })?;
        if manifest.format != CATALOG_FORMAT || manifest.package.format != PACKAGE_FORMAT {
            return Err(PackageError::Invalid {
                detail: format!("{} is not a navigation package", manifest_path.display()),
            });
        }
        let mut package = Self {
            imagery_zoom: manifest.configuration.imagery_zoom,
            terrain_zoom: manifest.configuration.dem_zoom,
            info: manifest.package.clone(),
            manifest_sha256,
            imagery: BTreeMap::new(),
            terrain: BTreeMap::new(),
        };
        for tile in &manifest.tiles {
            let path = dir.join(&tile.path);
            let bytes = std::fs::read(&path).map_err(io(&path))?;
            if bytes.len() != tile.bytes || digest_hex(&bytes) != tile.sha256 {
                return Err(PackageError::Integrity {
                    detail: format!("{} does not match the manifest", path.display()),
                });
            }
            package.insert(tile.kind.as_str(), tile.xyz, &bytes)?;
        }
        Ok(package)
    }

    fn insert(
        &mut self,
        kind: &str,
        [zoom, x, y]: [u32; 3],
        bytes: &[u8],
    ) -> Result<(), PackageError> {
        let Some(layer) = TileLayer::from_kind(kind) else {
            return Ok(());
        };
        let address = TileAddress {
            zoom: u8::try_from(zoom).map_err(|_| PackageError::Invalid {
                detail: format!("zoom {zoom}"),
            })?,
            x,
            y,
        };
        let decoded = image::load_from_memory(bytes).map_err(|source| PackageError::Decode {
            layer,
            address,
            source,
        })?;
        if decoded.width() != TILE_PIXELS || decoded.height() != TILE_PIXELS {
            return Err(PackageError::Invalid {
                detail: format!("{layer:?} tile {address:?} is not {TILE_PIXELS} px square"),
            });
        }
        match layer {
            TileLayer::Imagery => {
                self.imagery.insert(address, decoded.to_luma8().into_raw());
            }
            TileLayer::Dem => {
                // Terrarium heights are above mean sea level; the scene's
                // frame measures heights above the ellipsoid. The geoid
                // changes by centimetres across one elevation tile.
                let [lat, lon] = tile_lat_lon(f64::from(x) + 0.5, f64::from(y) + 0.5, address.zoom);
                let (ellipsoidal, orthometric) = CatalogVerticalDatum::OrthometricMslEgm96.heights(
                    lat.to_radians(),
                    lon.to_radians(),
                    0.0,
                );
                let undulation = ellipsoidal.0 - orthometric.0;
                let heights = decoded
                    .to_rgb8()
                    .pixels()
                    .map(|p| (decode_terrarium(p.0) + undulation) as f32)
                    .collect();
                self.terrain.insert(address, heights);
            }
        }
        Ok(())
    }

    /// The package's section of its manifest.
    pub const fn info(&self) -> &PackageInfo {
        &self.info
    }

    /// The package as Navigate identifies a map.
    pub fn map_revision(&self) -> MapRevision {
        MapRevision {
            release_id: self.info.release_id.clone(),
            manifest_sha256: self.manifest_sha256.clone(),
        }
    }

    /// Imagery and elevation tiles held.
    pub fn tile_counts(&self) -> (usize, usize) {
        (self.imagery.len(), self.terrain.len())
    }

    /// The render scene of the square of half side `radius_m` around a point,
    /// in a WGS84 east-north-up frame anchored there.
    pub fn scene_around(
        &self,
        lat_deg: f64,
        lon_deg: f64,
        radius_m: f64,
    ) -> Result<SceneWindow, PackageError> {
        let invalid = |detail: String| PackageError::Invalid { detail };
        let frame = LocalFrame::wgs84_enu(lat_deg, lon_deg)
            .map_err(|error| invalid(format!("frame at {lat_deg}, {lon_deg}: {error}")))?;
        let imagery_tiles = tiles_around(lat_deg, lon_deg, radius_m, self.imagery_zoom);
        let terrain_tiles = tiles_around(lat_deg, lon_deg, radius_m, self.terrain_zoom);
        let (imagery_layout, pixels, coverage) = assemble(
            &imagery_tiles,
            |a| self.imagery.get(a).map(|t| t.as_slice()),
            0_u8,
        )?;
        let (terrain_layout, heights_m, terrain_coverage) = assemble(
            &terrain_tiles,
            |a| self.terrain.get(a).map(|t| t.as_slice()),
            0.0_f32,
        )?;
        let covered = coverage.iter().filter(|c| **c == 255).count();
        let imagery_coverage = covered as f64 / coverage.len().max(1) as f64;
        Ok(SceneWindow {
            scene: RenderScene {
                map: self.map_revision(),
                frame,
                imagery: ImageAtlas {
                    layout: imagery_layout,
                    pixels,
                    coverage,
                },
                terrain: TerrainAtlas {
                    layout: terrain_layout,
                    heights_m,
                    coverage: terrain_coverage,
                },
            },
            imagery_coverage,
        })
    }
}

type Assembled<T> = (AtlasLayout, Vec<T>, Vec<u8>);

/// The rectangle of `tiles` as one row-major grid, with coverage 255 where a
/// tile was present.
fn assemble<'a, T: Copy + 'a>(
    tiles: &[TileAddress],
    lookup: impl Fn(&TileAddress) -> Option<&'a [T]>,
    blank: T,
) -> Result<Assembled<T>, PackageError> {
    let (Some(first), Some(last)) = (tiles.first(), tiles.last()) else {
        return Err(PackageError::Invalid {
            detail: "an empty scene window".into(),
        });
    };
    let columns = last.x.saturating_sub(first.x).wrapping_add(1);
    let rows = last.y.saturating_sub(first.y).wrapping_add(1);
    let width = (columns * TILE_PIXELS) as usize;
    let size = width * (rows * TILE_PIXELS) as usize;
    let mut values = vec![blank; size];
    let mut coverage = vec![0_u8; size];
    let side = TILE_PIXELS as usize;
    for address in tiles {
        let Some(tile) = lookup(address).filter(|t| t.len() == TILE_SAMPLES) else {
            continue;
        };
        let column = (address.x - first.x) as usize * side;
        let row = (address.y - first.y) as usize * side;
        for (line, chunk) in tile.chunks_exact(side).enumerate() {
            let start = (row + line) * width + column;
            if let (Some(target), Some(mask)) = (
                values.get_mut(start..start + side),
                coverage.get_mut(start..start + side),
            ) {
                target.copy_from_slice(chunk);
                mask.fill(255);
            }
        }
    }
    Ok((
        AtlasLayout {
            zoom: u32::from(first.zoom),
            origin_tile: [first.x, first.y],
            tile_pixels: TILE_PIXELS,
            size: [columns * TILE_PIXELS, rows * TILE_PIXELS],
        },
        values,
        coverage,
    ))
}
