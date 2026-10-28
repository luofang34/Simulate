//! Scenery sources the renderer and the flight model draw on.
//!
//! Each kind of scenery is a trait so the delivery mechanism (HTTP, verified
//! content-addressed archives, offline packs) stays outside rendering and
//! physics. Global imagery and photogrammetry are deliberately deferred: their
//! sources exist as [`Deferred`] stubs that fail with a typed error, so callers
//! already handle "not available" the way they will for a missing tile.

mod deferred;
mod sources;

pub use deferred::Deferred;
pub use sources::{
    ElevationSource, EllipsoidSurface, ImageryRaster, ImagerySource, MeshNode,
    PhotogrammetrySource, SceneryError, TileId,
};
