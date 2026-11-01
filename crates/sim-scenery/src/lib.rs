//! Scenery sources the renderer and the flight model draw on.
//!
//! Each kind of scenery is a trait so the delivery mechanism stays outside
//! rendering and physics. [`CatalogScenery`] serves Web Mercator Terrarium
//! elevation and orthoimagery tiles from memory, from a `Navigate` reference
//! catalog, or from a deterministic synthetic corridor;
//! [`LunarTranquilityScenery`] is the procedural Apollo 11 landing site.
//! Absent data is `Ok(None)` everywhere, never a made-up sea level.
//! Photogrammetry is deferred: its source is a [`Deferred`] stub that fails
//! with a typed error.

mod aerodrome;
mod catalog;
mod deferred;
mod lunar;
mod mercator;
mod noise;
mod sources;
mod terrarium;

pub use aerodrome::{AerodromeId, AirspaceControl, OsmAerodromeDiagram};
pub use catalog::{CatalogScenery, SyntheticCorridorSpec};
pub use deferred::Deferred;
pub use lunar::{
    APOLLO_11_SITE_LAT_DEG, APOLLO_11_SITE_LON_DEG, LITTLE_WEST_CRATER, LunarCrater,
    LunarTranquilityScenery, WEST_CRATER,
};
pub use mercator::{
    MAX_MERCATOR_LAT_DEG, MAX_TILE_ZOOM, TileCoord, project_to_tile, tile_point_to_geodetic,
};
pub use sources::{
    ElevationSource, EllipsoidSurface, ImageryRaster, ImagerySource, MeshNode,
    PhotogrammetrySource, RadiometricSource, SceneryError, SurfaceRadiometry, TileId,
};
pub use terrarium::{TerrariumTile, decode_terrarium, encode_terrarium};
