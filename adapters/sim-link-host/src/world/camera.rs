//! The camera's world opened from the sealed package, and its frames: the
//! trial's own CPU renderer, so a frame is the same bytes in every run of
//! the same build, and a reopened link flies the same flight.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use navigate_data::{MemoryStore, Sha256Digest};
use sim_geo::SensorIntrinsics;
use sim_navigate::trial::{OpenedPackage, PackageRef};
use sim_scenery::{CatalogScenery, LumaImage, render_luma_view};

use crate::HostError;
use crate::chain::{PAGE_STORE, page_stores};

/// The world the terrain camera sees.
#[derive(Debug, Clone)]
pub struct CameraWorld {
    scenery: Arc<CatalogScenery>,
}

impl CameraWorld {
    /// Opens the package `identity` in `store` the way the trial opens its
    /// world, and decodes every tile. It blocks for as long as decoding
    /// takes, which the page measured at about a second for 532 tiles.
    ///
    /// # Errors
    ///
    /// [`HostError::Trial`] for a package the trial refuses or cannot
    /// decode.
    pub fn open_blocking(store: MemoryStore, identity: Sha256Digest) -> Result<Self, HostError> {
        let stores = page_stores(store);
        let reference = PackageRef {
            store: PAGE_STORE.to_owned(),
            package: identity.to_string(),
        };
        let scenery =
            OpenedPackage::open_blocking(&reference, &stores, "maps")?.scenery_blocking()?;
        Ok(Self {
            scenery: Arc::new(scenery),
        })
    }

    /// The decoded world, shared.
    pub fn scenery(&self) -> Arc<CatalogScenery> {
        Arc::clone(&self.scenery)
    }

    /// The scene luma a pinhole camera at `eye_ecef` sees, `camera_to_ecef`
    /// rotating camera axes (`+X` right, `+Y` up, looking along `-Z`) into
    /// WGS84 body-fixed ones: row-major, top row first.
    ///
    /// # Errors
    ///
    /// [`HostError::Render`] for a pose or intrinsics the renderer cannot
    /// use, or a world it cannot read there.
    pub fn render(
        &self,
        eye_ecef: DVec3,
        camera_to_ecef: DQuat,
        intrinsics: &SensorIntrinsics,
    ) -> Result<LumaImage, HostError> {
        render_luma_view(&self.scenery, eye_ecef, camera_to_ecef, intrinsics).map_err(|error| {
            HostError::Render {
                detail: error.to_string(),
            }
        })
    }
}
