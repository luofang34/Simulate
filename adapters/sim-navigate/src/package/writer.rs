use std::collections::BTreeMap;
use std::path::Path;

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use super::tiles::{TileAddress, corridor_tiles, ground_sample_distance_m};
use super::{MANIFEST_NAME, PACKAGE_FORMAT, PackageError, PackageRole, TileFetch, TileLayer};

/// The `format` of the catalog part of a manifest.
pub(super) const CATALOG_FORMAT: &str = "photo-reference-catalog";

/// What to put in a package.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PackageSpec {
    /// Release identifier, `navigate_visual::MapRevision::release_id`.
    pub release_id: String,
    /// Onboard map or simulated world.
    pub role: PackageRole,
    /// The route, latitude and longitude in degrees, in fly order.
    pub route: Vec<[f64; 2]>,
    /// Half width of the covered corridor, metres: the planned corridor plus
    /// the camera footprint and the widest search around the estimate.
    pub corridor_radius_m: f64,
    /// Imagery zoom.
    pub imagery_zoom: u8,
    /// Elevation zoom.
    pub terrain_zoom: u8,
    /// Most tiles of both layers together; more is refused before fetching.
    pub max_tiles: usize,
}

/// What a written package holds.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PackageSummary {
    /// SHA-256 of the manifest bytes, hex.
    pub manifest_sha256: String,
    /// Imagery tiles written.
    pub imagery_tiles: u64,
    /// Elevation tiles written.
    pub terrain_tiles: u64,
    /// Tiles the source did not have.
    pub missing_tiles: u64,
    /// Bytes of tiles written.
    pub tile_bytes: u64,
}

/// The manifest of a package: a `photo-reference-catalog` with a `package`
/// section.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub(super) struct Manifest {
    pub(super) format: String,
    pub(super) version: u32,
    pub(super) complete: bool,
    pub(super) attribution: BTreeMap<String, String>,
    pub(super) scope: String,
    pub(super) configuration: Configuration,
    pub(super) tiles: Vec<ManifestTile>,
    pub(super) package: PackageInfo,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub(super) struct Configuration {
    /// West, south, east, north, degrees.
    pub(super) bbox: [f64; 4],
    pub(super) dem_zoom: u8,
    pub(super) imagery_zoom: u8,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub(super) struct ManifestTile {
    pub(super) kind: String,
    pub(super) xyz: [u32; 3],
    pub(super) path: String,
    pub(super) sha256: String,
    pub(super) bytes: usize,
}

/// The package section of a manifest.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PackageInfo {
    /// [`PACKAGE_FORMAT`].
    pub format: String,
    /// Format version.
    pub version: u32,
    /// What the package is for.
    pub role: PackageRole,
    /// Release identifier.
    pub release_id: String,
    /// The first route point, latitude and longitude in degrees.
    pub anchor: [f64; 2],
    /// Imagery ground sample distance at the anchor, metres.
    pub imagery_gsd_m: f64,
    /// Elevation ground sample distance at the anchor, metres.
    pub terrain_gsd_m: f64,
    /// Half width of the covered corridor, metres.
    pub corridor_radius_m: f64,
    /// The route the corridor follows.
    pub route: Vec<[f64; 2]>,
    /// Heights of the elevation tiles: Terrarium, mean sea level.
    pub vertical_datum: String,
    /// The tile source.
    pub source: String,
    /// Tiles the source did not have; the coverage has holes there.
    pub missing_tiles: u64,
}

/// SHA-256 of `bytes`, lower-case hex.
pub(super) fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

fn extension(bytes: &[u8]) -> &'static str {
    if bytes.starts_with(&[0x89, b'P', b'N', b'G']) {
        "png"
    } else if bytes.starts_with(&[0xFF, 0xD8]) {
        "jpg"
    } else if bytes.starts_with(b"RIFF") {
        "webp"
    } else {
        "bin"
    }
}

/// Selects the corridor's tiles, fetches each once from `source`, and writes
/// the package into `out`, which must not hold a manifest already.
pub fn prepare_package_blocking(
    spec: &PackageSpec,
    source: &mut dyn TileFetch,
    out: &Path,
) -> Result<PackageSummary, PackageError> {
    let Some(&[anchor_lat, anchor_lon]) = spec.route.first() else {
        return Err(PackageError::Invalid {
            detail: "a package needs at least one route point".into(),
        });
    };
    let jobs: Vec<(TileLayer, TileAddress)> = [
        (TileLayer::Dem, spec.terrain_zoom),
        (TileLayer::Imagery, spec.imagery_zoom),
    ]
    .into_iter()
    .flat_map(|(layer, zoom)| {
        corridor_tiles(&spec.route, spec.corridor_radius_m, zoom)
            .into_iter()
            .map(move |address| (layer, address))
    })
    .collect();
    if jobs.len() > spec.max_tiles {
        return Err(PackageError::Invalid {
            detail: format!(
                "{} tiles exceed the limit of {}",
                jobs.len(),
                spec.max_tiles
            ),
        });
    }
    let manifest_path = out.join(MANIFEST_NAME);
    if manifest_path.exists() {
        return Err(PackageError::Invalid {
            detail: format!("{} already exists", manifest_path.display()),
        });
    }
    let (tiles, mut summary) = fetch_and_write(jobs, source, out)?;
    let manifest = Manifest {
        format: CATALOG_FORMAT.into(),
        version: 1,
        complete: true,
        attribution: [TileLayer::Imagery, TileLayer::Dem]
            .into_iter()
            .map(|layer| (layer.kind().to_owned(), source.attribution(layer)))
            .collect(),
        scope: "Route corridor prepared before departure; nothing outside it is onboard.".into(),
        configuration: Configuration {
            bbox: bbox(&spec.route, spec.corridor_radius_m),
            dem_zoom: spec.terrain_zoom,
            imagery_zoom: spec.imagery_zoom,
        },
        tiles,
        package: PackageInfo {
            format: PACKAGE_FORMAT.into(),
            version: 1,
            role: spec.role,
            release_id: spec.release_id.clone(),
            anchor: [anchor_lat, anchor_lon],
            imagery_gsd_m: ground_sample_distance_m(anchor_lat, spec.imagery_zoom),
            terrain_gsd_m: ground_sample_distance_m(anchor_lat, spec.terrain_zoom),
            corridor_radius_m: spec.corridor_radius_m,
            route: spec.route.clone(),
            vertical_datum: "orthometric_msl_egm96".into(),
            source: source.identity(),
            missing_tiles: summary.missing_tiles,
        },
    };
    let text =
        serde_json::to_string_pretty(&manifest).map_err(|source| PackageError::Manifest {
            path: manifest_path.display().to_string(),
            source,
        })?;
    write_file(&manifest_path, text.as_bytes())?;
    summary.manifest_sha256 = sha256_hex(text.as_bytes());
    Ok(summary)
}

/// Fetches each tile of `jobs` once and writes it under `out`.
fn fetch_and_write(
    jobs: Vec<(TileLayer, TileAddress)>,
    source: &mut dyn TileFetch,
    out: &Path,
) -> Result<(Vec<ManifestTile>, PackageSummary), PackageError> {
    let mut tiles = Vec::with_capacity(jobs.len());
    let mut summary = PackageSummary {
        manifest_sha256: String::new(),
        imagery_tiles: 0,
        terrain_tiles: 0,
        missing_tiles: 0,
        tile_bytes: 0,
    };
    for (layer, address) in jobs {
        let Some(bytes) = source.fetch(layer, address)? else {
            summary.missing_tiles = summary.missing_tiles.wrapping_add(1);
            continue;
        };
        let path = format!(
            "{}/{}/{}/{}.{}",
            layer.kind(),
            address.zoom,
            address.x,
            address.y,
            extension(&bytes)
        );
        write_file(&out.join(&path), &bytes)?;
        match layer {
            TileLayer::Imagery => summary.imagery_tiles = summary.imagery_tiles.wrapping_add(1),
            TileLayer::Dem => summary.terrain_tiles = summary.terrain_tiles.wrapping_add(1),
        }
        summary.tile_bytes = summary.tile_bytes.wrapping_add(bytes.len() as u64);
        tiles.push(ManifestTile {
            kind: layer.kind().into(),
            xyz: [u32::from(address.zoom), address.x, address.y],
            path,
            sha256: sha256_hex(&bytes),
            bytes: bytes.len(),
        });
    }
    Ok((tiles, summary))
}

fn write_file(path: &Path, bytes: &[u8]) -> Result<(), PackageError> {
    let io = |source| PackageError::Io {
        path: path.display().to_string(),
        source,
    };
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(io)?;
    }
    std::fs::write(path, bytes).map_err(io)
}

/// West, south, east, north of the route grown by `radius_m`, degrees.
fn bbox(route: &[[f64; 2]], radius_m: f64) -> [f64; 4] {
    let lat_pad = radius_m / 111_320.0;
    let mut out = [
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    ];
    for &[lat, lon] in route {
        let lon_pad = radius_m / (111_320.0 * lat.to_radians().cos().max(1e-6));
        out = [
            out[0].min(lon - lon_pad),
            out[1].min(lat - lat_pad),
            out[2].max(lon + lon_pad),
            out[3].max(lat + lat_pad),
        ];
    }
    out
}
