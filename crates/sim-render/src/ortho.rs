use std::num::NonZeroU32;

use sim_geo::Geodetic;
use sim_scenery::{ImagerySource, TileId, project_to_tile};

use crate::RenderError;

#[cfg(test)]
mod tests;

/// A rectangle of Web Mercator tiles at one zoom, the extent of an atlas.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OrthoFootprint {
    /// Zoom of every tile.
    pub zoom: u8,
    /// Westernmost column.
    pub west: u32,
    /// Northernmost row.
    pub north: u32,
    /// Columns covered.
    pub tiles_wide: NonZeroU32,
    /// Rows covered.
    pub tiles_high: NonZeroU32,
}

impl OrthoFootprint {
    /// The smallest footprint over `tiles`, at the zoom of the first. `None`
    /// if there are none. Tiles at another zoom are ignored.
    pub fn covering(tiles: impl IntoIterator<Item = TileId>) -> Option<Self> {
        let mut tiles = tiles.into_iter();
        let first = tiles.next()?;
        let (mut west, mut east) = (first.x, first.x);
        let (mut north, mut south) = (first.y, first.y);
        for tile in tiles.filter(|tile| tile.zoom == first.zoom) {
            west = west.min(tile.x);
            east = east.max(tile.x);
            north = north.min(tile.y);
            south = south.max(tile.y);
        }
        Some(Self {
            zoom: first.zoom,
            west,
            north,
            tiles_wide: NonZeroU32::new(east - west + 1)?,
            tiles_high: NonZeroU32::new(south - north + 1)?,
        })
    }

    /// Where `position` falls in the atlas: `0..=1` inside the footprint,
    /// outside that range elsewhere. Computed in `f64` from the tile address,
    /// so a coordinate is good to a fraction of a metre even across a
    /// many-kilometre atlas. `None` where Web Mercator has no tiles.
    pub fn uv_at(&self, position: &Geodetic) -> Option<[f32; 2]> {
        let coord = project_to_tile(position, self.zoom)?;
        let u = (f64::from(coord.tile.x) - f64::from(self.west) + coord.u)
            / f64::from(self.tiles_wide.get());
        let v = (f64::from(coord.tile.y) - f64::from(self.north) + coord.v)
            / f64::from(self.tiles_high.get());
        Some([u as f32, v as f32])
    }
}

/// Orthoimagery tiles composited into one texture. Pixels with no tile have
/// zero alpha, so the shader can tell a hole from black ground.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OrthoAtlas {
    /// The tiles the atlas spans.
    pub footprint: OrthoFootprint,
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// Row-major RGBA8 pixels in sRGB.
    pub rgba8: Vec<u8>,
}

impl OrthoAtlas {
    /// Composites the tiles of `footprint` from `source`, each `tile_px`
    /// square; tiles the source does not cover stay transparent.
    pub fn from_source(
        source: &dyn ImagerySource,
        footprint: OrthoFootprint,
        tile_px: u32,
    ) -> Result<Self, RenderError> {
        let too_large = || RenderError::TextureTooLarge {
            what: "ortho atlas",
            width: footprint.tiles_wide.get().saturating_mul(tile_px),
            height: footprint.tiles_high.get().saturating_mul(tile_px),
            max: u32::MAX,
        };
        let width = footprint
            .tiles_wide
            .get()
            .checked_mul(tile_px)
            .ok_or_else(too_large)?;
        let height = footprint
            .tiles_high
            .get()
            .checked_mul(tile_px)
            .ok_or_else(too_large)?;
        let bytes = (width as usize)
            .checked_mul(height as usize)
            .and_then(|pixels| pixels.checked_mul(4))
            .ok_or_else(too_large)?;
        let mut atlas = Self {
            footprint,
            width,
            height,
            rgba8: vec![0; bytes],
        };
        for row in 0..footprint.tiles_high.get() {
            for column in 0..footprint.tiles_wide.get() {
                let tile = TileId {
                    zoom: footprint.zoom,
                    x: footprint.west + column,
                    y: footprint.north + row,
                };
                if let Some(raster) = source.imagery(tile)? {
                    if raster.width != tile_px || raster.height != tile_px {
                        return Err(RenderError::AtlasTileSize {
                            width: raster.width,
                            height: raster.height,
                            tile_px,
                        });
                    }
                    atlas.blit(&raster.rgba8, column * tile_px, row * tile_px, tile_px);
                }
            }
        }
        Ok(atlas)
    }

    fn blit(&mut self, rgba8: &[u8], left: u32, top: u32, tile_px: u32) {
        let stride = self.width as usize * 4;
        let tile_stride = tile_px as usize * 4;
        let start = top as usize * stride;
        let rows = self
            .rgba8
            .get_mut(start..)
            .into_iter()
            .flat_map(|rest| rest.chunks_exact_mut(stride));
        for (target, source) in rows.zip(rgba8.chunks_exact(tile_stride)) {
            let from = left as usize * 4;
            if let Some(slot) = target.get_mut(from..from + tile_stride) {
                slot.copy_from_slice(source);
            }
        }
    }
}
