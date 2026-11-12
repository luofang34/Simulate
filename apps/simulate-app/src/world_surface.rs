//! The one Earth surface physics, spawning, and the maps agree on, matched to
//! the active [`ScenerySource`].

use std::collections::BTreeSet;
use std::sync::{Arc, PoisonError, RwLock, RwLockReadGuard, RwLockWriteGuard};

use glam::DVec3;
use sim_geo::Geodetic;
use sim_render::Camera;
use sim_scenery::{
    AuthoritativeWorldSurface, CatalogScenery, CatalogVerticalDatum, LiveDemBridge, RunwayDatum,
    RunwayStrip, SceneryError, TileId, WorldSurfaceQuery,
};

use crate::map_rig::ScenerySource;

#[cfg(test)]
mod tests;

/// A shared handle on one [`AuthoritativeWorldSurface`], rebuilt when the scenery
/// source changes and fed live Terrarium tiles while real-world tiles stream.
///
/// Clones share the surface: the flight, its plants, the cockpit, and the map rig of one
/// app hold clones of the same handle, while each test builds its own.
///
/// Every surface carries the [`RunwayDatum`] of every charted runway, the same datum the
/// maps lay into the terrain they draw ([`ScenerySource`]), so the ground under a runway
/// is the runway that is drawn.
#[derive(Debug, Clone)]
pub struct WorldSurfaceHandle {
    shared: Arc<RwLock<AuthoritativeWorldSurface>>,
    /// One datum for the life of the handle, so runway profiles resolved while the maps
    /// stamp their tiles are the ones the ground reads, across scenery changes too.
    runway_datum: RunwayDatum,
}

impl Default for WorldSurfaceHandle {
    fn default() -> Self {
        Self::new()
    }
}

impl WorldSurfaceHandle {
    /// The offline catalog surface, the state before any map rig picks a source. It is
    /// pure in-memory data (no I/O).
    pub fn new() -> Self {
        let runway_datum = RunwayDatum::builtin();
        let surface = catalog_surface(&runway_datum).unwrap_or_else(|error| {
            tracing::error!(%error, "regional catalog unavailable; runways over bare Earth");
            with_runways(
                AuthoritativeWorldSurface::earth(CatalogVerticalDatum::OrthometricMslEgm96),
                &runway_datum,
            )
        });
        Self {
            shared: Arc::new(RwLock::new(surface)),
            runway_datum,
        }
    }

    /// The runway datum this surface and the maps drawing it share.
    pub fn runway_datum(&self) -> &RunwayDatum {
        &self.runway_datum
    }

    /// Replaces the surface with the one `source` draws: the synthetic catalog
    /// offline, or bare Earth waiting for live Terrarium tiles online. Both
    /// carry every charted runway.
    pub fn configure(&self, source: &ScenerySource) -> Result<(), SceneryError> {
        let surface = match source {
            ScenerySource::Catalog => catalog_surface(&self.runway_datum)?,
            ScenerySource::RealWorld { .. } => with_runways(
                AuthoritativeWorldSurface::earth(CatalogVerticalDatum::OrthometricMslEgm96),
                &self.runway_datum,
            ),
        };
        *self.write() = surface;
        Ok(())
    }

    /// Makes every tile queued on `bridge` resident; returns how many.
    pub fn drain(&self, bridge: &LiveDemBridge) -> usize {
        if bridge.pending_len_may_be_stale() == 0 {
            return 0;
        }
        bridge.drain_into(&mut self.write())
    }

    /// Keeps the live tiles in `drawn` (the DEM tiles the physical scene draws) and evicts
    /// far from `ownship` first, letting `bridge` take evicted tiles again; returns the
    /// drawn tiles the surface does not hold.
    pub fn keep_live_tiles(
        &self,
        ownship: Geodetic,
        drawn: BTreeSet<TileId>,
        bridge: &LiveDemBridge,
    ) -> BTreeSet<TileId> {
        let mut surface = self.write();
        let missing = drawn
            .iter()
            .copied()
            .filter(|&tile| !surface.has_live_tile(tile))
            .collect();
        surface.set_live_dem_focus(ownship, drawn);
        bridge.forget_evicted(&mut surface);
        missing
    }

    /// Ellipsoidal ground height under `position`, when the surface has ground there.
    pub fn ellipsoidal_ground_m(&self, position: &Geodetic) -> Option<f64> {
        let sample = self.read().sample_surface(position).ok()??;
        Some(sample.ellipsoidal_height_m.0)
    }

    /// Ellipsoidal ground height and [`AuthoritativeWorldSurface::world_version`] under
    /// `position`, so grounded plants distinguish physical terrain slope from asynchronous
    /// DEM tile arrivals.
    pub fn ellipsoidal_ground_and_version(&self, position: &Geodetic) -> Option<(f64, u64)> {
        let sample = self.read().sample_surface(position).ok()??;
        Some((sample.ellipsoidal_height_m.0, sample.world_version))
    }

    /// Replaces the ad-hoc runway strips stamped into [`Self::runway_datum`] and the
    /// live/extra world surface, restamping any already-resident live Terrarium tiles so
    /// newly delineated runways and bridge decks are immediately flat and landable.
    pub fn set_ad_hoc_strips(&self, strips: Vec<RunwayStrip>) {
        self.runway_datum.set_ad_hoc_strips(strips.clone());
        let mut surface = self.write();
        surface.set_extra_runways(strips);
        let _restamped = surface.restamp_live_tiles(&self.runway_datum);
    }

    /// Unprojects physical window cursor `cursor_px` through 3D perspective `camera` onto
    /// the authoritative terrain surface (`<= 150 km` slant range), returning the hit's
    /// [`Geodetic`] coordinates and orthometric height in metres MSL.
    pub fn raycast_cursor_geodetic(
        &self,
        camera: &Camera,
        cursor_px: [f64; 2],
        surface_px: [u32; 2],
    ) -> Option<(Geodetic, f64)> {
        let [cx, cy] = cursor_px;
        let [sw, sh] = surface_px.map(|v| f64::from(v.max(1)));
        if !cx.is_finite() || !cy.is_finite() || !camera.fov_y_rad.is_finite() {
            return None;
        }
        let ndc_x = 2.0 * (cx / sw) - 1.0;
        let ndc_y = 1.0 - 2.0 * (cy / sh);
        let tan_half_y = (camera.fov_y_rad * 0.5).tan();
        if !tan_half_y.is_finite() || tan_half_y <= 0.0 {
            return None;
        }
        let aspect = sw / sh;
        let dir_cam = DVec3::new(ndc_x * aspect * tan_half_y, ndc_y * tan_half_y, -1.0).normalize();
        let dir_ecef = (camera.pose.orientation * dir_cam).normalize_or_zero();
        if dir_ecef.length_squared() < 0.5 {
            return None;
        }
        let from_ecef = camera.pose.position_ecef;
        let to_ecef = from_ecef + dir_ecef * 150_000.0;
        let hit = self
            .read()
            .raycast_segment_ecef(from_ecef, to_ecef, 192)
            .ok()??;
        Some((hit.geodetic, hit.surface.orthometric_msl_m.0))
    }

    fn read(&self) -> RwLockReadGuard<'_, AuthoritativeWorldSurface> {
        self.shared.read().unwrap_or_else(PoisonError::into_inner)
    }

    fn write(&self) -> RwLockWriteGuard<'_, AuthoritativeWorldSurface> {
        self.shared.write().unwrap_or_else(PoisonError::into_inner)
    }
}

#[cfg_attr(
    not(test),
    allow(
        dead_code,
        reason = "airport-picker seam for the app, exercised by tests"
    )
)]
impl WorldSurfaceHandle {
    /// Bounds the live tiles to `tiles`, to force eviction.
    #[cfg(test)]
    pub fn set_live_dem_capacity(&self, tiles: usize) {
        let mut surface = self.write();
        *surface = surface.clone().with_live_dem_capacity(tiles);
    }

    /// The finest zoom of a resident live tile covering `position`.
    #[cfg(test)]
    pub fn finest_live_zoom_at(&self, position: &Geodetic) -> Option<u8> {
        let surface = self.read();
        (0..=sim_scenery::MAX_TILE_ZOOM).rev().find(|&zoom| {
            sim_scenery::project_to_tile(position, zoom)
                .is_some_and(|coord| surface.has_live_tile(coord.tile))
        })
    }

    /// How many live Terrarium tiles are resident.
    #[cfg(test)]
    pub fn live_dem_len(&self) -> usize {
        self.read().live_dem_len()
    }

    /// The datum the surface reads Earth heights in; ellipsoidal off Earth.
    pub fn vertical_datum(&self) -> CatalogVerticalDatum {
        self.read()
            .vertical_datum()
            .unwrap_or(CatalogVerticalDatum::EllipsoidWgs84)
    }

    /// Metres from the ellipsoid up to the surface's height reference.
    pub fn geoid_undulation_m(&self, lat_rad: f64, lon_rad: f64) -> f64 {
        self.read().geoid_undulation_m(lat_rad, lon_rad)
    }

    /// The ellipsoidal height of a charted height `height_msl_m` in the surface's datum.
    pub fn msl_to_ellipsoidal_m(&self, lat_rad: f64, lon_rad: f64, height_msl_m: f64) -> f64 {
        self.read()
            .msl_to_ellipsoidal_m(lat_rad, lon_rad, height_msl_m)
    }

    /// Ground height under `position` above EGM96 mean sea level, in either datum, when the
    /// surface has ground there.
    pub fn orthometric_ground_m(&self, position: &Geodetic) -> Option<f64> {
        let sample = self.read().sample_surface(position).ok()??;
        Some(sample.orthometric_msl_m.0)
    }
}

/// The synthetic regional catalog with `datum` laid into its terrain, as the offline maps
/// draw it.
pub fn stamped_regional_catalog(datum: &RunwayDatum) -> Result<CatalogScenery, SceneryError> {
    Ok(CatalogScenery::synthetic_regional_network()?.with_runway_datum(datum))
}

fn catalog_surface(datum: &RunwayDatum) -> Result<AuthoritativeWorldSurface, SceneryError> {
    Ok(with_runways(
        AuthoritativeWorldSurface::earth_with_catalog(stamped_regional_catalog(datum)?),
        datum,
    ))
}

fn with_runways(
    surface: AuthoritativeWorldSurface,
    datum: &RunwayDatum,
) -> AuthoritativeWorldSurface {
    surface
        .with_polar_elevations_m(0.0, 2_835.0)
        .with_runway_datum(datum.clone())
}
