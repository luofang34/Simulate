//! Ground heights for the resource model, from coarse Terrarium tiles read
//! before the plan: the plan cannot say how high above the ground a leg
//! flies until it knows the ground.

use std::collections::{BTreeMap, BTreeSet};

use navigate_data::corridor::corridor_tiles;
use navigate_data::tile::{decode_terrarium_m, tile_xy};
use navigate_data::{Fetched, GroundHeight, LayerKind, TileAddress};

use crate::prefetch::obtain;
use crate::{NavdataError, RepairPort, SourceSet, TileTransport};

#[cfg(test)]
mod tests;

/// The zoom of the planning elevation, about 120 m a pixel: the value of
/// Navigate's package tool.
pub const GROUND_ZOOM: u32 = 10;
/// Half width of the strip of planning tiles about the route, metres: the
/// value of Navigate's package tool.
pub const GROUND_CORRIDOR_M: f64 = 2_000.0;

/// Decoded Terrarium tiles at [`GROUND_ZOOM`]: heights above mean sea
/// level.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TerrariumGround {
    /// By column and row: the side of the tile in pixels and its heights,
    /// row by row.
    tiles: BTreeMap<(u32, u32), (u32, Vec<f32>)>,
}

impl TerrariumGround {
    /// Decodes one Terrarium PNG at [`GROUND_ZOOM`].
    ///
    /// # Errors
    ///
    /// Returns [`NavdataError::Decode`] for bytes that are not a PNG.
    pub fn insert(&mut self, address: TileAddress, png: &[u8]) -> Result<(), NavdataError> {
        let image = image::load_from_memory(png)
            .map_err(|error| NavdataError::Decode {
                tile: format!("elevation {}/{}/{}", address.zoom, address.x, address.y),
                detail: error.to_string(),
            })?
            .to_rgb8();
        let heights = image
            .pixels()
            .map(|pixel| decode_terrarium_m(pixel.0) as f32)
            .collect();
        self.tiles
            .insert((address.x, address.y), (image.width(), heights));
        Ok(())
    }

    /// Tiles decoded.
    pub fn len(&self) -> usize {
        self.tiles.len()
    }

    /// Whether no tile is decoded.
    pub fn is_empty(&self) -> bool {
        self.tiles.is_empty()
    }
}

impl GroundHeight for TerrariumGround {
    /// The height of the pixel holding `point`, as Navigate's package tool
    /// samples it.
    fn ground_m(&self, point: [f64; 2]) -> Option<f64> {
        let [x, y] = tile_xy(point, GROUND_ZOOM);
        let (side, heights) = self.tiles.get(&(x.floor() as u32, y.floor() as u32))?;
        let last = side.saturating_sub(1) as usize;
        let column = (((x - x.floor()) * f64::from(*side)) as usize).min(last);
        let row = (((y - y.floor()) * f64::from(*side)) as usize).min(last);
        let index = row.saturating_mul(*side as usize).saturating_add(column);
        heights.get(index).map(|height| f64::from(*height))
    }
}

/// The planning tiles about the route through `points`, `[lat, lon]` in
/// degrees.
///
/// # Errors
///
/// Returns Navigate's refusal of the route.
pub fn ground_tiles(points: &[[f64; 2]]) -> Result<BTreeSet<TileAddress>, NavdataError> {
    let mut tiles = BTreeSet::new();
    corridor_tiles(points, GROUND_CORRIDOR_M, GROUND_ZOOM, &mut tiles)
        .map_err(NavdataError::data("choosing the planning elevation tiles"))?;
    Ok(tiles)
}

/// Reads the planning tiles about `points` from the cache in `store`,
/// fetching through `transport` those it lacks. A tile the source does not
/// have leaves its ground unknown, which the plan then refuses.
///
/// # Errors
///
/// Returns a refused route, a failed fetch or store, and a tile that does
/// not decode.
pub async fn read_ground<T, S>(
    points: &[[f64; 2]],
    sources: &SourceSet,
    transport: &T,
    store: &mut S,
) -> Result<TerrariumGround, NavdataError>
where
    T: TileTransport,
    S: RepairPort,
{
    let mut ground = TerrariumGround::default();
    for address in ground_tiles(points)? {
        let key = sources.tile_key(LayerKind::Elevation, address);
        if let (Fetched::Tile(png), _) = obtain(&key, sources, transport, store).await? {
            ground.insert(address, &png)?;
        }
    }
    Ok(ground)
}
