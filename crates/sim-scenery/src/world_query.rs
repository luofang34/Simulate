//! One authoritative answer to "where is the ground?" for gear contact,
//! radar altimeters, terrain avoidance and line-of-sight.

use std::sync::{Arc, OnceLock};

use glam::DVec3;
use sim_geo::{
    AglHeightM, BodyEllipsoid, EllipsoidalHeightM, Geodetic, LocalFrame, OrthometricHeightM,
};

use crate::{
    AerodromeId, CatalogScenery, CatalogVerticalDatum, ElevationSource, LunarTranquilityScenery,
    OsmAerodromeDiagram, RunwayDatum, SceneryError, TerrariumTile, TileId,
};

mod compose;
mod datum;
mod live_dem;
mod raycast;
mod runway;

#[cfg(test)]
mod tests;

use live_dem::LiveDem;
pub use runway::RunwayStrip;
pub(crate) use runway::surface_ecef;

/// Half the baseline of the central differences behind a surface normal.
const NORMAL_HALF_BASELINE_M: f64 = 5.0;

/// Which data a [`SurfaceSample`] came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SurfaceSampleSource {
    /// A surveyed runway strip, or its skirt blending into the terrain.
    RunwaySurface,
    /// A resident elevation tile at `zoom`: a catalog tile, or a live
    /// Terrarium tile streamed in through
    /// [`AuthoritativeWorldSurface::insert_terrarium_tile`].
    ResidentDem {
        /// The zoom of the tile sampled.
        zoom: u8,
    },
    /// The procedural Apollo 11 landing site.
    LunarProcedural,
    /// The surface's uniform fallback elevation, where no tile is resident.
    FallbackElevation,
    /// The bare reference ellipsoid, which a ray reports hitting where no
    /// terrain is known; never returned by
    /// [`WorldSurfaceQuery::sample_surface`].
    ReferenceEllipsoid,
}

/// The ground at one point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceSample {
    /// Height above the body's reference ellipsoid.
    pub ellipsoidal_height_m: EllipsoidalHeightM,
    /// Height above EGM96 mean sea level. On the Moon, which has no geoid
    /// here, this equals the height above its reference sphere.
    pub orthometric_msl_m: OrthometricHeightM,
    /// Upward unit normal in the local east-north-up frame; `[0, 0, 1]` on
    /// level ground.
    pub normal_enu: DVec3,
    /// The same normal in body-fixed (ECEF) axes.
    pub normal_ecef: DVec3,
    /// Where the height came from.
    pub source: SurfaceSampleSource,
    /// [`WorldSurfaceQuery::world_version`] when sampled.
    pub world_version: u64,
}

/// Where a segment first meets the ground.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    /// Distance from the segment start, metres.
    pub distance_m: f64,
    /// The hit point, body-fixed metres.
    pub position_ecef: DVec3,
    /// The hit point on the body's ellipsoid.
    pub geodetic: Geodetic,
    /// The ground at the hit point.
    pub surface: SurfaceSample,
}

/// Ground queries shared by physics, sensors and the renderer, so every
/// consumer agrees on one surface.
pub trait WorldSurfaceQuery {
    /// A counter that changes whenever the surface does; equal versions mean
    /// equal answers.
    fn world_version(&self) -> u64;

    /// The ground under `position` (its height is ignored). `Ok(None)` means
    /// nothing covers the point, never sea level.
    fn sample_surface(&self, position: &Geodetic) -> Result<Option<SurfaceSample>, SceneryError>;

    /// How far `position`, an ellipsoidal height, is above the ground below
    /// it; negative underground.
    fn agl_clearance_m(&self, position: &Geodetic) -> Result<Option<AglHeightM>, SceneryError> {
        Ok(self
            .sample_surface(position)?
            .map(|surface| AglHeightM(position.height_m - surface.ellipsoidal_height_m.0)))
    }

    /// The first point where the straight segment `from_ecef -> to_ecef`
    /// meets the ground, or the reference ellipsoid where no ground is
    /// known. The segment is marched in `steps.clamp(4, 256)` chords, so a
    /// ridge narrower than one chord can be stepped over; a crossing found is
    /// refined by bisection to well under a metre.
    fn raycast_segment_ecef(
        &self,
        from_ecef: DVec3,
        to_ecef: DVec3,
        steps: u32,
    ) -> Result<Option<RayHit>, SceneryError>;
}

#[derive(Debug, Clone, PartialEq)]
enum Body {
    Earth(Box<Earth>),
    Moon(LunarTranquilityScenery),
}

#[derive(Debug, Clone, Default, PartialEq)]
struct Earth {
    catalog: Option<CatalogScenery>,
    live: LiveDem,
    runways: Vec<RunwayStrip>,
    /// Runways registered at run time, replaced as a whole.
    extra_runways: Vec<RunwayStrip>,
    /// Every charted runway, laid over the terrain as the drawn terrain carries it.
    runway_datum: Option<RunwayDatum>,
    polar_elevations_m: Option<[f64; 2]>,
    fallback_m: Option<f64>,
    datum: CatalogVerticalDatum,
    /// The renderer draws the live DEM as served, so where it is resident the
    /// ground is that DEM, runway datum and all.
    unflattened_visual_dem: bool,
}

/// The ground of one body: on Earth live Terrarium tiles over a catalog DEM,
/// surveyed runway strips and an optional uniform fallback; on the Moon the
/// Apollo 11 site.
///
/// The live overlay holds the Terrarium tiles a renderer or tile loader has
/// fetched, bounded and indexed by zoom. Where it covers a point its finest
/// zoom wins over the catalog; runway strips still blend over either, so a
/// paved runway stays at its surveyed height.
///
/// Every Earth height source is read in one [`CatalogVerticalDatum`], the
/// attached catalog's, so a runway and the DEM around it never differ by the
/// geoid undulation. A runway threshold's height is taken as charted in that
/// datum, whatever its field name says: the built-in aerodromes carry their
/// field elevation there.
#[derive(Debug, Clone, PartialEq)]
pub struct AuthoritativeWorldSurface {
    body: Body,
    version: u64,
}

impl AuthoritativeWorldSurface {
    /// Earth with no ground data yet, read in `datum`. An orthometric bare
    /// Earth is the live-tile world the renderer drapes over raw Terrarium
    /// relief, so it starts with
    /// [`Self::with_unflattened_visual_dem`] on; attaching a catalog turns it
    /// off.
    #[must_use]
    pub fn earth(datum: CatalogVerticalDatum) -> Self {
        Self {
            body: Body::Earth(Box::new(Earth {
                datum,
                unflattened_visual_dem: datum == CatalogVerticalDatum::OrthometricMslEgm96,
                ..Earth::default()
            })),
            version: 0,
        }
    }

    /// Earth over `catalog`, read in the catalog's datum.
    #[must_use]
    pub fn earth_with_catalog(catalog: CatalogScenery) -> Self {
        Self::earth(catalog.vertical_datum()).with_catalog(catalog)
    }

    /// Process-shared regional Earth surface combining [`CatalogScenery::synthetic_regional_network`]
    /// with all built-in [`OsmAerodromeDiagram`] runways (`47N`, `KEWR`, `KDOV`).
    pub fn regional_earth_shared() -> Result<&'static Self, SceneryError> {
        static SHARED: OnceLock<Result<AuthoritativeWorldSurface, SceneryError>> = OnceLock::new();
        SHARED
            .get_or_init(|| {
                let mut surface =
                    Self::earth_with_catalog(CatalogScenery::synthetic_regional_network()?);
                for id in AerodromeId::ALL {
                    surface = surface.with_aerodrome(&OsmAerodromeDiagram::builtin(id)?);
                }
                Ok(surface)
            })
            .as_ref()
            .map_err(Clone::clone)
    }

    /// The Apollo 11 landing site on the IAU lunar sphere.
    #[must_use]
    pub fn moon(scenery: LunarTranquilityScenery) -> Self {
        Self {
            body: Body::Moon(scenery),
            version: 0,
        }
    }

    /// Replaces the catalog, adopting its datum, and turns
    /// [`Self::with_unflattened_visual_dem`] off: a catalog's aerodromes are
    /// graded flat under their runways. Ignored on the Moon.
    #[must_use]
    pub fn with_catalog(mut self, catalog: CatalogScenery) -> Self {
        if let Body::Earth(earth) = &mut self.body {
            earth.datum = catalog.vertical_datum();
            earth.unflattened_visual_dem = false;
            earth.catalog = Some(catalog);
            self.bump();
        }
        self
    }

    /// Adds the runways of `aerodrome`. Ignored on the Moon.
    #[must_use]
    pub fn with_aerodrome(mut self, aerodrome: &OsmAerodromeDiagram) -> Self {
        if let Body::Earth(earth) = &mut self.body {
            earth
                .runways
                .extend(RunwayStrip::from_ends(&aerodrome.surface_graph.runway_ends));
            self.bump();
        }
        self
    }

    /// Uses `height_m`, in the surface's datum, wherever no catalog tile is
    /// resident. Ignored on the Moon.
    #[must_use]
    pub fn with_fallback_elevation_m(mut self, height_m: f64) -> Self {
        if let Body::Earth(earth) = &mut self.body {
            earth.fallback_m = height_m.is_finite().then_some(height_m);
            self.bump();
        }
        self
    }

    /// Makes a live Terrarium tile resident at `id`, replacing any live tile
    /// there, and changes [`WorldSurfaceQuery::world_version`]. Ignored on
    /// the Moon.
    pub fn insert_terrarium_tile(
        &mut self,
        id: TileId,
        tile: TerrariumTile,
    ) -> Result<(), SceneryError> {
        self.insert_shared_terrarium_tile(id, Arc::new(tile))
    }

    /// [`Self::insert_terrarium_tile`] for a tile already shared, as a
    /// [`crate::LiveDemBridge`] hands it over.
    pub fn insert_shared_terrarium_tile(
        &mut self,
        id: TileId,
        tile: Arc<TerrariumTile>,
    ) -> Result<(), SceneryError> {
        if let Body::Earth(earth) = &mut self.body {
            earth.live.insert(id, tile)?;
            self.bump();
        }
        Ok(())
    }

    /// Decodes Terrarium PNG bytes and makes the tile resident at `id`, as
    /// [`Self::insert_terrarium_tile`] does.
    pub fn insert_terrarium_png(
        &mut self,
        id: TileId,
        png_bytes: &[u8],
    ) -> Result<(), SceneryError> {
        self.insert_terrarium_tile(id, TerrariumTile::from_png_bytes(png_bytes)?)
    }

    /// Reads live Terrarium tiles in `datum` from now on; they are
    /// orthometric unless told otherwise, as published Terrarium tiles are.
    #[must_use]
    pub fn with_live_dem_datum(mut self, datum: CatalogVerticalDatum) -> Self {
        if let Body::Earth(earth) = &mut self.body {
            earth.live.set_datum(datum);
            self.bump();
        }
        self
    }

    /// Keeps at most `tiles` live tiles, evicting the longest resident first.
    #[must_use]
    pub fn with_live_dem_capacity(mut self, tiles: usize) -> Self {
        if let Body::Earth(earth) = &mut self.body {
            earth.live.set_capacity(tiles);
            self.bump();
        }
        self
    }

    /// The datum live Terrarium tiles are read in; `None` on the Moon.
    #[must_use]
    pub fn live_dem_datum(&self) -> Option<CatalogVerticalDatum> {
        match &self.body {
            Body::Earth(earth) => Some(earth.live.datum()),
            Body::Moon(_) => None,
        }
    }

    /// Resident live tile addresses, in order.
    pub fn live_dem_tiles(&self) -> Vec<TileId> {
        match &self.body {
            Body::Earth(earth) => earth.live.tile_ids().collect(),
            Body::Moon(_) => Vec::new(),
        }
    }

    /// How many live tiles are resident.
    #[must_use]
    pub fn live_dem_len(&self) -> usize {
        match &self.body {
            Body::Earth(earth) => earth.live.len(),
            Body::Moon(_) => 0,
        }
    }

    /// The datum Earth heights are read in; `None` on the Moon.
    #[must_use]
    pub fn vertical_datum(&self) -> Option<CatalogVerticalDatum> {
        match &self.body {
            Body::Earth(earth) => Some(earth.datum),
            Body::Moon(_) => None,
        }
    }

    /// The reference ellipsoid of the body.
    #[must_use]
    pub fn ellipsoid(&self) -> BodyEllipsoid {
        match self.body {
            Body::Earth(_) => BodyEllipsoid::WGS84,
            Body::Moon(_) => BodyEllipsoid::MOON_IAU,
        }
    }

    fn bump(&mut self) {
        self.version = self.version.wrapping_add(1);
    }

    /// Ellipsoidal and orthometric ground height at a latitude and longitude,
    /// without the normal.
    fn heights_at(
        &self,
        lat_rad: f64,
        lon_rad: f64,
    ) -> Result<Option<(EllipsoidalHeightM, OrthometricHeightM, SurfaceSampleSource)>, SceneryError>
    {
        let position = Geodetic {
            lat_rad,
            lon_rad,
            height_m: 0.0,
        };
        match &self.body {
            Body::Earth(earth) => Ok(earth.raw_height(&position).map(|(raw, source)| {
                let (ellipsoidal, orthometric) = earth.datum.heights(lat_rad, lon_rad, raw);
                (ellipsoidal, orthometric, source)
            })),
            Body::Moon(scenery) => Ok(scenery.ground_height_m(&position)?.map(|height| {
                (
                    EllipsoidalHeightM(height),
                    OrthometricHeightM(height),
                    SurfaceSampleSource::LunarProcedural,
                )
            })),
        }
    }

    /// Upward unit normal in east-north-up from central differences of the
    /// ellipsoidal height in the 3D local tangent frame, one-sided where a
    /// neighbour is uncovered and free of polar `1 / cos(lat)` singularities.
    fn normal_enu(&self, position: &Geodetic, centre_m: f64) -> Result<DVec3, SceneryError> {
        let ellipsoid = self.ellipsoid();
        let surface_pos = Geodetic {
            height_m: 0.0,
            ..*position
        };
        let origin = ellipsoid.geodetic_to_fixed(&surface_pos);
        let frame = LocalFrame::at(&surface_pos);
        let step_height = |offset: DVec3| -> Result<Option<f64>, SceneryError> {
            let stepped = ellipsoid.fixed_to_geodetic(origin + offset);
            Ok(self
                .heights_at(stepped.lat_rad, stepped.lon_rad)?
                .map(|(h, _, _)| h.0))
        };
        let slope = |plus: Option<f64>, minus: Option<f64>| match (plus, minus) {
            (Some(p), Some(m)) => (p - m) / (2.0 * NORMAL_HALF_BASELINE_M),
            (Some(p), None) => (p - centre_m) / NORMAL_HALF_BASELINE_M,
            (None, Some(m)) => (centre_m - m) / NORMAL_HALF_BASELINE_M,
            (None, None) => 0.0,
        };
        let east_step = frame.east() * NORMAL_HALF_BASELINE_M;
        let north_step = frame.north() * NORMAL_HALF_BASELINE_M;
        let east = slope(step_height(east_step)?, step_height(-east_step)?);
        let north = slope(step_height(north_step)?, step_height(-north_step)?);
        Ok(DVec3::new(-east, -north, 1.0).normalize())
    }
}

impl WorldSurfaceQuery for AuthoritativeWorldSurface {
    fn world_version(&self) -> u64 {
        match &self.body {
            Body::Earth(earth) => self
                .version
                .wrapping_add(earth.catalog.as_ref().map_or(0, CatalogScenery::version))
                .wrapping_add(
                    earth
                        .runway_datum
                        .as_ref()
                        .map_or(0, RunwayDatum::generation),
                ),
            _ => self.version,
        }
    }

    fn sample_surface(&self, position: &Geodetic) -> Result<Option<SurfaceSample>, SceneryError> {
        let Some((ellipsoidal, orthometric, source)) =
            self.heights_at(position.lat_rad, position.lon_rad)?
        else {
            return Ok(None);
        };
        let normal_enu = self.normal_enu(position, ellipsoidal.0)?;
        let normal_ecef = LocalFrame::at(position).enu_to_ecef() * normal_enu;
        Ok(Some(SurfaceSample {
            ellipsoidal_height_m: ellipsoidal,
            orthometric_msl_m: orthometric,
            normal_enu,
            normal_ecef,
            source,
            world_version: self.world_version(),
        }))
    }

    fn raycast_segment_ecef(
        &self,
        from_ecef: DVec3,
        to_ecef: DVec3,
        steps: u32,
    ) -> Result<Option<RayHit>, SceneryError> {
        raycast::first_hit(self, from_ecef, to_ecef, steps)
    }
}

/// Heights above the body's reference ellipsoid, so the surface can stand in
/// wherever an [`ElevationSource`] is read as ellipsoidal.
impl ElevationSource for AuthoritativeWorldSurface {
    fn ground_height_m(&self, position: &Geodetic) -> Result<Option<f64>, SceneryError> {
        Ok(self
            .heights_at(position.lat_rad, position.lon_rad)?
            .map(|(ellipsoidal, _, _)| ellipsoidal.0))
    }
}
