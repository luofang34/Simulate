//! The one Earth surface physics, spawning, and the maps agree on, matched to
//! the active [`ScenerySource`].

use std::sync::{Arc, PoisonError, RwLock, RwLockReadGuard, RwLockWriteGuard};

use sim_geo::Geodetic;
use sim_scenery::{
    AerodromeId, AuthoritativeWorldSurface, CatalogScenery, CatalogVerticalDatum, LiveDemBridge,
    OsmAerodromeDiagram, RunwayStrip, SceneryError, WorldSurfaceQuery,
};

use crate::map_rig::ScenerySource;

#[cfg(test)]
mod tests;

/// A shared handle on one [`AuthoritativeWorldSurface`], rebuilt when the scenery
/// source changes and fed live Terrarium tiles while real-world tiles stream.
///
/// Clones share the surface: the flight, its plants, the cockpit, and the map rig of one
/// app hold clones of the same handle, while each test builds its own.
#[derive(Debug, Clone)]
pub struct WorldSurfaceHandle {
    shared: Arc<RwLock<Shared>>,
}

/// The surface and the runtime runway strips it must carry; one lock so a rebuild can
/// never miss strips registered concurrently.
#[derive(Debug)]
struct Shared {
    surface: AuthoritativeWorldSurface,
    /// Strips carry charted MSL heights, which every datum reads as its own height
    /// reference, so they re-apply unchanged to any rebuilt surface.
    extra_runways: Vec<RunwayStrip>,
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
        let surface = catalog_surface().unwrap_or_else(|error| {
            tracing::error!(%error, "regional catalog unavailable; runways over bare Earth");
            with_runways(AuthoritativeWorldSurface::earth(
                CatalogVerticalDatum::OrthometricMslEgm96,
            ))
        });
        Self {
            shared: Arc::new(RwLock::new(Shared {
                surface,
                extra_runways: Vec::new(),
            })),
        }
    }

    /// Replaces the surface with the one `source` draws: the synthetic catalog
    /// offline, or bare Earth waiting for live Terrarium tiles online. Both
    /// carry every built-in runway and the strips of [`Self::set_extra_runways`].
    pub fn configure(&self, source: &ScenerySource) -> Result<(), SceneryError> {
        let mut surface = match source {
            ScenerySource::Catalog => catalog_surface()?,
            ScenerySource::RealWorld { .. } => with_runways(AuthoritativeWorldSurface::earth(
                CatalogVerticalDatum::OrthometricMslEgm96,
            )),
        };
        let mut shared = self.write();
        surface.set_extra_runways(shared.extra_runways.clone());
        shared.surface = surface;
        Ok(())
    }

    /// Makes every tile queued on `bridge` resident; returns how many.
    pub fn drain(&self, bridge: &LiveDemBridge) -> usize {
        if bridge.pending_len_may_be_stale() == 0 {
            return 0;
        }
        bridge.drain_into(&mut self.write().surface)
    }

    /// Ellipsoidal ground height under `position`, when the surface has ground there.
    pub fn ellipsoidal_ground_m(&self, position: &Geodetic) -> Option<f64> {
        let sample = self.read().surface.sample_surface(position).ok()??;
        Some(sample.ellipsoidal_height_m.0)
    }

    /// Ellipsoidal ground height and [`AuthoritativeWorldSurface::world_version`] under
    /// `position`, so grounded plants distinguish physical terrain slope from asynchronous
    /// DEM tile arrivals.
    pub fn ellipsoidal_ground_and_version(&self, position: &Geodetic) -> Option<(f64, u64)> {
        let sample = self.read().surface.sample_surface(position).ok()??;
        Some((sample.ellipsoidal_height_m.0, sample.world_version))
    }

    fn read(&self) -> RwLockReadGuard<'_, Shared> {
        self.shared.read().unwrap_or_else(PoisonError::into_inner)
    }

    fn write(&self) -> RwLockWriteGuard<'_, Shared> {
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
    /// Registers the runway strips of an airport picked at run time (replacing earlier
    /// ones), kept across [`Self::configure`].
    pub fn set_extra_runways(&self, strips: Vec<RunwayStrip>) {
        let mut shared = self.write();
        shared.surface.set_extra_runways(strips.clone());
        shared.extra_runways = strips;
    }

    /// The datum the surface reads Earth heights in; ellipsoidal off Earth.
    pub fn vertical_datum(&self) -> CatalogVerticalDatum {
        self.read()
            .surface
            .vertical_datum()
            .unwrap_or(CatalogVerticalDatum::EllipsoidWgs84)
    }

    /// Metres from the ellipsoid up to the surface's height reference.
    pub fn geoid_undulation_m(&self, lat_rad: f64, lon_rad: f64) -> f64 {
        self.read().surface.geoid_undulation_m(lat_rad, lon_rad)
    }

    /// The ellipsoidal height of a charted height `height_msl_m` in the surface's datum.
    pub fn msl_to_ellipsoidal_m(&self, lat_rad: f64, lon_rad: f64, height_msl_m: f64) -> f64 {
        self.read()
            .surface
            .msl_to_ellipsoidal_m(lat_rad, lon_rad, height_msl_m)
    }

    /// Ground height under `position` above EGM96 mean sea level, in either datum, when the
    /// surface has ground there.
    pub fn orthometric_ground_m(&self, position: &Geodetic) -> Option<f64> {
        let sample = self.read().surface.sample_surface(position).ok()??;
        Some(sample.orthometric_msl_m.0)
    }
}

fn catalog_surface() -> Result<AuthoritativeWorldSurface, SceneryError> {
    let catalog = CatalogScenery::synthetic_regional_network()?;
    Ok(with_runways(AuthoritativeWorldSurface::earth_with_catalog(
        catalog,
    )))
}

fn with_runways(mut surface: AuthoritativeWorldSurface) -> AuthoritativeWorldSurface {
    surface = surface.with_polar_elevations_m(0.0, 2_835.0);
    for id in AerodromeId::ALL {
        match OsmAerodromeDiagram::builtin(id) {
            Ok(diagram) => surface = surface.with_aerodrome(&diagram),
            Err(error) => tracing::error!(?id, %error, "built-in aerodrome diagram unavailable"),
        }
    }
    surface
}
