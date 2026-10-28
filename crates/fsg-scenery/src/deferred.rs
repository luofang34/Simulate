use crate::{ImageryRaster, ImagerySource, MeshNode, PhotogrammetrySource, SceneryError, TileId};

#[cfg(test)]
mod tests;

/// Stand-in for a scenery capability that is planned but not built. Every
/// request fails with [`SceneryError::Deferred`] naming the capability.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Deferred {
    capability: &'static str,
}

impl Deferred {
    /// Global orthoimagery.
    pub const IMAGERY: Self = Self {
        capability: "global imagery",
    };
    /// Photogrammetry meshes.
    pub const PHOTOGRAMMETRY: Self = Self {
        capability: "photogrammetry",
    };

    fn error(&self) -> SceneryError {
        SceneryError::Deferred {
            capability: self.capability,
        }
    }
}

impl ImagerySource for Deferred {
    fn imagery(&self, _tile: TileId) -> Result<ImageryRaster, SceneryError> {
        Err(self.error())
    }
}

impl PhotogrammetrySource for Deferred {
    fn node(&self, _path: &[u8]) -> Result<MeshNode, SceneryError> {
        Err(self.error())
    }
}
