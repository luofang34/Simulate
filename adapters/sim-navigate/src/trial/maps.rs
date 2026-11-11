//! The map packages of a trial: the onboard package the navigator flies
//! with and the world package the simulated camera sees.
//!
//! Both are `navigate-data` packages in content-addressed stores that many
//! packages share, named by the store's root directory and the package
//! identity, the digest of the canonical manifest. Opening one recomputes
//! that digest and checks the tile indexes, so a package that is missing,
//! not the one named, or of an unsupported version is refused before the
//! run. Simulate reads the onboard package's identity and nothing else:
//! the package itself goes to the navigator untouched. The world package is
//! read whole into a [`CatalogScenery`] for rendering.

use std::path::{Path, PathBuf};

use navigate_data::{
    DataError, DirectoryStore, Encoding, GeoidModel, LayerKind, Package, Sha256Digest, TileRead,
    VerticalDatum, tile::ground_sample_distance_m,
};
use serde::{Deserialize, Serialize};
use sim_scenery::{CatalogScenery, CatalogVerticalDatum, ImageryRaster, TerrariumTile, TileId};

use super::TrialError;

/// A package named in a request.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PackageRef {
    /// The root directory of the store holding the package.
    pub store: String,
    /// The package identity, `navigate-data`'s digest of the canonical
    /// manifest, 64 hex digits.
    pub package: String,
}

/// Why a named package was refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PackageRefusal {
    /// The identity is not 64 hex digits.
    MalformedIdentity,
    /// The store does not hold the manifest or a file it needs.
    Missing,
    /// A file does not have the digest or size it is named by: the
    /// manifest found is not the package named, or a blob is corrupt.
    IdentityMismatch,
    /// The package is of a format version or holds a value this reader
    /// does not support.
    Unsupported,
    /// The store failed or a document does not decode.
    Unreadable,
}

/// The accuracy the host assumes for a package that declares none.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AssumedAccuracy {
    /// Horizontal 1-sigma, metres.
    pub horizontal_m: f64,
    /// Vertical 1-sigma, metres.
    pub vertical_m: f64,
}

/// The navigator's map package.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OnboardMapSpec {
    /// The root directory of the store holding the package.
    pub store: String,
    /// The package identity, 64 hex digits.
    pub package: String,
    /// The accuracy to assume where the package declares it unknown; the
    /// navigator refuses such a package without it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub assumed_accuracy: Option<AssumedAccuracy>,
    /// The largest frame, in pixels, the navigator matches at; absent takes
    /// the navigator's own bound. Frames are resampled to it, so their
    /// ground pixels grow.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_match_pixels: Option<u32>,
}

impl OnboardMapSpec {
    /// The package it names.
    pub fn package_ref(&self) -> PackageRef {
        PackageRef {
            store: self.store.clone(),
            package: self.package.clone(),
        }
    }
}

/// The geoid model the navigator is given for orthometric map heights.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GeoidChoice {
    /// The world's own model, `sim_geo::egm96_undulation_m` (EGM96,
    /// bilinear over a 5° grid): the navigator and the world agree exactly.
    World,
}

/// The two packages of a visual run, and the navigator's geoid.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MapsSpec {
    /// The navigator's map, handed over untouched.
    pub onboard: OnboardMapSpec,
    /// The world the simulated camera renders: the same package or another
    /// source, season or zoom.
    pub world: PackageRef,
    /// The geoid the navigator is given; absent, it has none and bounds the
    /// undulation in its error instead.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub geoid: Option<GeoidChoice>,
}

impl MapsSpec {
    /// Whether the camera renders the very package the navigator matches
    /// against: the best case, the same pixels on both sides.
    pub fn world_is_onboard(&self) -> bool {
        self.world.package == self.onboard.package
    }
}

/// What a result says of one package.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PackageReport {
    /// `onboard` or `world`.
    pub role: String,
    /// The package identity, hex.
    pub manifest_sha256: String,
    /// The imagery layer's source, as the manifest names it.
    pub imagery_source: String,
    /// The deepest imagery zoom held.
    pub imagery_zoom: u32,
    /// Ground size of one imagery pixel at that zoom, at the latitude of
    /// the package's tiles, metres.
    pub imagery_ground_sample_m: f64,
    /// Imagery tiles held.
    pub imagery_tiles: u64,
    /// Elevation tiles held.
    pub elevation_tiles: u64,
}

/// The maps of a visual run as the result states them.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MapsReport {
    /// The navigator's package.
    pub onboard: PackageReport,
    /// The package the camera renders.
    pub world: PackageReport,
    /// Whether they are the same package: the camera and the navigator then
    /// see the same pixels, the best case for matching.
    pub world_is_onboard: bool,
    /// The geoid the world's orthometric heights are converted with.
    pub world_geoid: String,
    /// The geoid the navigator is given, or `none`.
    pub navigator_geoid: String,
}

/// The geoid model of the simulated world.
pub const WORLD_GEOID: &str =
    "EGM96 by sim_geo::egm96_undulation_m, bilinear over a 5 degree grid of the NGA model";

/// Opens both packages of `maps` under their identities and states them.
pub fn maps_report_blocking(maps: &MapsSpec) -> Result<MapsReport, TrialError> {
    let onboard = OpenedPackage::open_blocking(&maps.onboard.package_ref(), "maps.onboard")?;
    let world = OpenedPackage::open_blocking(&maps.world, "maps.world")?;
    Ok(MapsReport {
        onboard: onboard.report_blocking("onboard")?,
        world: world.report_blocking("world")?,
        world_is_onboard: maps.world_is_onboard(),
        world_geoid: WORLD_GEOID.into(),
        navigator_geoid: match maps.geoid {
            Some(GeoidChoice::World) => format!("the world's: {WORLD_GEOID}"),
            None => "none".into(),
        },
    })
}

/// An opened package.
pub struct OpenedPackage {
    store: DirectoryStore,
    package: Package<<DirectoryStore as navigate_data::ReadPort>::File>,
    root: PathBuf,
}

impl std::fmt::Debug for OpenedPackage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OpenedPackage")
            .field("root", &self.root)
            .field("identity", &self.package.identity())
            .finish_non_exhaustive()
    }
}

impl OpenedPackage {
    /// Opens the package `reference` names, refusing one that is missing,
    /// not the one named, or of an unsupported version.
    pub fn open_blocking(reference: &PackageRef, field: &'static str) -> Result<Self, TrialError> {
        let refuse = |reason: PackageRefusal, detail: String| TrialError::Package {
            field,
            reason,
            detail,
        };
        let identity = parse_digest(&reference.package).ok_or_else(|| {
            refuse(
                PackageRefusal::MalformedIdentity,
                format!("`{}`", reference.package),
            )
        })?;
        let root = PathBuf::from(&reference.store);
        let store = DirectoryStore::new(root.clone());
        let package = pollster::block_on(Package::open(&store, identity)).map_err(|error| {
            let reason = match &error {
                DataError::Missing { .. } => PackageRefusal::Missing,
                DataError::Integrity { .. } => PackageRefusal::IdentityMismatch,
                DataError::Version { .. } | DataError::Unsupported { .. } => {
                    PackageRefusal::Unsupported
                }
                _ => PackageRefusal::Unreadable,
            };
            let detail = format!(
                "package {} in {}: {error}",
                reference.package,
                root.display()
            );
            refuse(reason, detail)
        })?;
        Ok(Self {
            store,
            package,
            root,
        })
    }

    /// The store directory.
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// What the package holds.
    pub fn report_blocking(&self, role: &str) -> Result<PackageReport, TrialError> {
        let manifest = self.package.manifest();
        let imagery = manifest.layer_of_kind(LayerKind::Imagery);
        let elevation = manifest.layer_of_kind(LayerKind::Elevation);
        let (zoom, latitude) = match imagery {
            Some(layer) => self.deepest(layer.key)?,
            None => (0, 0.0),
        };
        Ok(PackageReport {
            role: role.to_owned(),
            manifest_sha256: self.package.identity().to_string(),
            imagery_source: imagery.map_or_else(String::new, |l| format!("{:?}", l.key.source)),
            imagery_zoom: zoom,
            imagery_ground_sample_m: imagery.map_or(0.0, |layer| {
                ground_sample_distance_m(latitude, zoom, layer.tile_pixels)
            }),
            imagery_tiles: imagery.map_or(0, |l| l.index.tiles),
            elevation_tiles: elevation.map_or(0, |l| l.index.tiles),
        })
    }

    /// The deepest zoom of a layer and the latitude of a tile there.
    fn deepest(&self, layer: navigate_data::LayerKey) -> Result<(u32, f64), TrialError> {
        let entries = pollster::block_on(self.package.layer_entries(layer)).map_err(data)?;
        let deepest = entries
            .iter()
            .map(|(address, _)| *address)
            .max_by_key(|a| a.zoom);
        Ok(deepest.map_or((0, 0.0), |address| (address.zoom, address.north_west()[0])))
    }

    /// Every imagery and elevation tile of the package, decoded into a
    /// scenery the renderer reads. Gaps stay absent, so the camera sees
    /// black there rather than invented ground.
    pub fn scenery_blocking(&self) -> Result<CatalogScenery, TrialError> {
        let mut scenery = CatalogScenery::new();
        for layer in &self.package.manifest().layers {
            let kind = layer.key.kind;
            if !matches!(kind, LayerKind::Imagery | LayerKind::Elevation) {
                continue;
            }
            let entries =
                pollster::block_on(self.package.layer_entries(layer.key)).map_err(data)?;
            for (address, _) in entries {
                let read = pollster::block_on(self.package.tile(&self.store, layer.key, address))
                    .map_err(data)?;
                let TileRead::Present(bytes) = read else {
                    continue;
                };
                let tile = tile_id(address)?;
                if kind == LayerKind::Imagery {
                    scenery
                        .insert_imagery(tile, decode_imagery(&bytes)?)
                        .map_err(scenery_error)?;
                } else if layer.encoding == Encoding::TerrariumPng {
                    scenery.set_vertical_datum(catalog_datum(layer.vertical_datum)?);
                    scenery
                        .insert_elevation(tile, decode_elevation(&bytes)?)
                        .map_err(scenery_error)?;
                }
            }
        }
        Ok(scenery)
    }
}

/// The renderer's datum for an elevation layer's. EGM2008 and EGM96 differ
/// by under a metre over most land, far below a camera's ground sample, so
/// both read as the renderer's EGM96; a datum nobody states is refused
/// rather than guessed.
fn catalog_datum(datum: Option<VerticalDatum>) -> Result<CatalogVerticalDatum, TrialError> {
    match datum {
        Some(VerticalDatum::Ellipsoidal) => Ok(CatalogVerticalDatum::EllipsoidWgs84),
        Some(VerticalDatum::Orthometric {
            geoid: GeoidModel::Egm96 | GeoidModel::Egm2008,
        }) => Ok(CatalogVerticalDatum::OrthometricMslEgm96),
        other => Err(TrialError::Maps {
            what: format!("elevation datum {other:?} cannot be rendered"),
        }),
    }
}

fn tile_id(address: navigate_data::TileAddress) -> Result<TileId, TrialError> {
    let zoom = u8::try_from(address.zoom).map_err(|_| TrialError::Maps {
        what: format!("zoom {} is beyond the renderer", address.zoom),
    })?;
    Ok(TileId {
        zoom,
        x: address.x,
        y: address.y,
    })
}

fn decode(bytes: &[u8]) -> Result<image::DynamicImage, TrialError> {
    image::ImageReader::new(std::io::Cursor::new(bytes))
        .with_guessed_format()
        .map_err(|error| TrialError::Maps {
            what: format!("tile format: {error}"),
        })?
        .decode()
        .map_err(|error| TrialError::Maps {
            what: format!("tile decode: {error}"),
        })
}

fn decode_imagery(bytes: &[u8]) -> Result<ImageryRaster, TrialError> {
    let pixels = decode(bytes)?.into_rgba8();
    ImageryRaster::new(pixels.width(), pixels.height(), pixels.into_raw()).map_err(scenery_error)
}

fn decode_elevation(bytes: &[u8]) -> Result<TerrariumTile, TrialError> {
    let pixels = decode(bytes)?.into_rgb8();
    TerrariumTile::from_rgb(pixels.width(), pixels.height(), pixels.as_raw()).map_err(scenery_error)
}

fn data(error: navigate_data::DataError) -> TrialError {
    TrialError::Maps {
        what: error.to_string(),
    }
}

fn scenery_error(error: sim_scenery::SceneryError) -> TrialError {
    TrialError::Maps {
        what: error.to_string(),
    }
}

/// A digest in its hex text form.
fn parse_digest(text: &str) -> Option<Sha256Digest> {
    text.parse().ok()
}
