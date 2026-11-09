//! [`CatalogScenery`]-backed [`HttpClient`] tile transport and synchronous lockstep DEM
//! supply for `maplibre`.

use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};

use image::{ExtendedColorType, ImageEncoder, RgbaImage, codecs::png::PngEncoder};
use maplibre::{
    coords::{TileCoords, WorldTileCoords, ZoomLevel},
    headless::map::HeadlessMap,
    io::{
        resource_loader::SharedLoader,
        source_client::{HttpClient, SourceFetchError},
    },
    style::source::TileAddressingScheme,
};
use sim_scenery::{CatalogScenery, ImagerySource, TerrariumTile, TileId, encode_terrarium};

use crate::MapAdapterError;

/// Canonical virtual DEM URL template recognized by [`CatalogTileLoader`].
pub const CATALOG_DEM_URL_TEMPLATE: &str = "https://sim.catalog/dem/{z}/{x}/{y}.png";

/// Canonical virtual orthoimagery URL template recognized by [`CatalogTileLoader`].
pub const CATALOG_ORTHO_URL_TEMPLATE: &str = "https://sim.catalog/ortho/{z}/{x}/{y}.png";

#[derive(Debug, thiserror::Error)]
#[error("catalog PNG encoding failed: {0}")]
struct PngEncodeFailure(#[from] image::ImageError);

/// In-memory [`HttpClient`] serving SHA-256-verified [`CatalogScenery`] elevation and
/// orthoimagery tiles directly to `maplibre` workers without network sockets.
#[derive(Clone, Debug)]
pub struct CatalogTileLoader {
    scenery: Arc<CatalogScenery>,
    dem_fetches: Arc<AtomicUsize>,
    ortho_fetches: Arc<AtomicUsize>,
}

impl CatalogTileLoader {
    /// Wraps `scenery` as an in-memory tile transport.
    #[must_use]
    pub fn new(scenery: Arc<CatalogScenery>) -> Self {
        Self {
            scenery,
            dem_fetches: Arc::new(AtomicUsize::new(0)),
            ortho_fetches: Arc::new(AtomicUsize::new(0)),
        }
    }

    /// Wraps `self` in a coalescing [`SharedLoader`].
    #[must_use]
    pub fn into_shared_loader(self) -> SharedLoader {
        SharedLoader::new(self)
    }

    /// Total number of DEM tile fetches answered from `scenery`.
    #[must_use]
    pub fn dem_fetch_count(&self) -> usize {
        self.dem_fetches.load(Ordering::Relaxed)
    }

    /// Total number of orthoimagery tile fetches answered from `scenery`.
    #[must_use]
    pub fn ortho_fetch_count(&self) -> usize {
        self.ortho_fetches.load(Ordering::Relaxed)
    }

    /// Underlying [`CatalogScenery`] reference.
    #[must_use]
    pub fn scenery(&self) -> &Arc<CatalogScenery> {
        &self.scenery
    }
}

#[cfg_attr(target_arch = "wasm32", async_trait::async_trait(?Send))]
#[cfg_attr(not(target_arch = "wasm32"), async_trait::async_trait)]
impl HttpClient for CatalogTileLoader {
    async fn fetch(&self, url: &str) -> Result<Vec<u8>, SourceFetchError> {
        if let Some(tile) = parse_zxy_suffix(url, "https://sim.catalog/dem/") {
            let Some(dem) = self.scenery.elevation_tile(tile) else {
                return Err(SourceFetchError::not_found(url));
            };
            self.dem_fetches.fetch_add(1, Ordering::Relaxed);
            let rgba = terrarium_tile_to_rgba_image(dem);
            return encode_rgba_png(rgba.width(), rgba.height(), rgba.as_raw());
        }
        if let Some(tile) = parse_zxy_suffix(url, "https://sim.catalog/ortho/") {
            let Ok(Some(raster)) = self.scenery.imagery(tile) else {
                return Err(SourceFetchError::not_found(url));
            };
            self.ortho_fetches.fetch_add(1, Ordering::Relaxed);
            return encode_rgba_png(raster.width, raster.height, &raster.rgba8);
        }
        Err(SourceFetchError::not_found(url))
    }
}

/// Converts a [`TerrariumTile`] into an [`RgbaImage`] encoded in Mapzen Terrarium RGB.
#[must_use]
pub fn terrarium_tile_to_rgba_image(tile: &TerrariumTile) -> RgbaImage {
    let width = tile.width();
    let height = tile.height();
    let cap = (width as usize)
        .saturating_mul(height as usize)
        .saturating_mul(4);
    let mut raw = Vec::with_capacity(cap);
    for y in 0..height {
        for x in 0..width {
            let h_m = tile.texel(x, y).unwrap_or(0.0);
            let [r, g, b] = encode_terrarium(h_m);
            raw.extend_from_slice(&[r, g, b, 255]);
        }
    }
    RgbaImage::from_raw(width, height, raw).unwrap_or_else(|| RgbaImage::new(1, 1))
}

/// Seeds a `z = 0` (`0, 0, 0`) sea-level (`0.0 m`) Terrarium DEM tile into `map` so
/// `TerrainCoverageIndex` has `exaggeration = 1.0` and global ancestor coverage on Frame 0
/// (including polar-cap frustum bounds at `±85.0511°..±90°`) before regional DEM tiles arrive.
pub(crate) fn seed_sea_level_root_dem(map: &mut HeadlessMap) -> Result<(), MapAdapterError> {
    let [r, g, b] = encode_terrarium(0.0);
    let raw = [r, g, b, 255].repeat(16);
    if let Some(image) = RgbaImage::from_raw(4, 4, raw) {
        map.load_dem_tiles(vec![(WorldTileCoords::default(), image)])
            .map_err(|source| MapAdapterError::MapOperation { source })?;
    }
    Ok(())
}

/// Synchronously populates all resident DEM tiles from `scenery` into `map` so Frame 1
/// of deterministic `30 Hz` optical evaluation has zero async tile pop-in.
pub fn supply_catalog_dem_synchronously(
    map: &mut HeadlessMap,
    scenery: &CatalogScenery,
) -> Result<usize, MapAdapterError> {
    let mut batch: Vec<(WorldTileCoords, RgbaImage)> = Vec::new();
    for tile in scenery.elevation_tiles() {
        let Some(dem) = scenery.elevation_tile(tile) else {
            continue;
        };
        let Some(coords) = tile_id_to_world_coords(tile) else {
            continue;
        };
        batch.push((coords, terrarium_tile_to_rgba_image(dem)));
    }
    let count = batch.len();
    if count > 0 {
        map.load_dem_tiles(batch)
            .map_err(|source| MapAdapterError::MapOperation { source })?;
    }
    Ok(count)
}

/// Synchronously populates all resident orthoimagery tiles from `scenery` into `map` under
/// `source_name` without PNG round-trip encoding so Frame 1 drapes cleanly.
pub fn supply_catalog_ortho_synchronously(
    map: &mut HeadlessMap,
    scenery: &CatalogScenery,
    source_name: &str,
) -> Result<usize, MapAdapterError> {
    let mut batch = Vec::new();
    for tile in scenery.imagery_tiles() {
        let Ok(Some(raster)) = scenery.imagery(tile) else {
            continue;
        };
        let Some(coords) = tile_id_to_world_coords(tile) else {
            continue;
        };
        let Some(image) = RgbaImage::from_raw(raster.width, raster.height, raster.rgba8) else {
            continue;
        };
        batch.push(maplibre::raster::AvailableRasterLayerData {
            coords,
            source: maplibre::raster::RasterSourceId::new(Some(source_name.to_owned())),
            image,
        });
    }
    let count = batch.len();
    if count > 0 {
        map.render_sources(maplibre::headless::map::ProcessedLayers::default(), batch)
            .map_err(|source| MapAdapterError::MapOperation { source })?;
    }
    Ok(count)
}

/// Converts a [`TileId`] into validated [`WorldTileCoords`] (`XYZ` slippy scheme).
#[must_use]
pub fn tile_id_to_world_coords(tile: TileId) -> Option<WorldTileCoords> {
    TileCoords {
        x: tile.x,
        y: tile.y,
        z: ZoomLevel::from(tile.zoom),
    }
    .into_world_tile(TileAddressingScheme::XYZ)
}

fn encode_rgba_png(width: u32, height: u32, rgba: &[u8]) -> Result<Vec<u8>, SourceFetchError> {
    let mut bytes = Vec::new();
    let encoder = PngEncoder::new(&mut bytes);
    encoder
        .write_image(rgba, width, height, ExtendedColorType::Rgba8)
        .map_err(|err| SourceFetchError(Box::new(PngEncodeFailure(err))))?;
    Ok(bytes)
}

pub(crate) fn parse_zxy_suffix(url: &str, prefix: &str) -> Option<TileId> {
    let rest = url.strip_prefix(prefix)?.strip_suffix(".png")?;
    let mut parts = rest.split('/');
    let zoom = parts.next()?.parse::<u8>().ok()?;
    let x = parts.next()?.parse::<u32>().ok()?;
    let y = parts.next()?.parse::<u32>().ok()?;
    if parts.next().is_some() {
        return None;
    }
    let tile = TileId { zoom, x, y };
    tile.is_valid().then_some(tile)
}
