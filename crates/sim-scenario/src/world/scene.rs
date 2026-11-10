use sim_geo::{BodyEllipsoid, Geodetic};
use sim_render::{GlobeMesh, OrthoAtlas, OrthoFootprint, TerrainRegion, TerrainSources};
use sim_scenery::{
    AerodromeId, CatalogScenery, ElevationSource, LunarTranquilityScenery, RadiometricSource,
    SceneryError, SurfaceRadiometry, SyntheticCorridorSpec, TileId, tile_point_to_geodetic,
};

use super::ScenarioWorld;
use crate::ScenarioError;

#[cfg(test)]
mod tests;

/// The coarse globe under every world: 15° patches, as the free-flight view
/// has always drawn.
const GLOBE_LAT_PATCHES: u32 = 12;
const GLOBE_LON_PATCHES: u32 = 24;
const GLOBE_PATCH_GRID: u32 = 16;
/// How far the globe sits below a detailed region, metres: past the chord sag
/// of its long facets, so it never pokes through.
const EARTH_GLOBE_SINK_M: f64 = 300.0;
const MOON_GLOBE_SINK_M: f64 = 150.0;

/// Dark mare ground at a sunlit morning temperature.
const REGOLITH: SurfaceRadiometry = SurfaceRadiometry {
    albedo: 0.09,
    emissivity: 0.95,
    temperature_k: 300.0,
};

/// Shrinks a region off its data's edge, radians: about a metre.
const COVERAGE_MARGIN_RAD: f64 = 2e-7;
const DELAWARE_LAT_PATCHES: u32 = 6;
const DELAWARE_LON_PATCHES: u32 = 9;
const DELAWARE_GRID: u32 = 24;
/// Half-extents of the lunar mesh, metres. The corners stay inside the
/// scenery's 4 km radius.
const LUNAR_HALF_EAST_M: f64 = 3_500.0;
const LUNAR_HALF_NORTH_M: f64 = 1_500.0;
const LUNAR_LAT_PATCHES: u32 = 6;
const LUNAR_LON_PATCHES: u32 = 14;
const LUNAR_GRID: u32 = 24;

/// A level surface a fixed height from the ellipsoid.
#[derive(Debug, Clone, Copy, PartialEq)]
struct LevelSurface {
    height_m: f64,
}

impl ElevationSource for LevelSurface {
    fn ground_height_m(&self, _position: &Geodetic) -> Result<Option<f64>, SceneryError> {
        Ok(Some(self.height_m))
    }
}

/// Mare regolith, for the parts of the Moon with no detailed scenery: without
/// it the globe would draw in the shader's procedural Earth colours.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct BareRegolith;

impl RadiometricSource for BareRegolith {
    fn surface_radiometry(
        &self,
        _position: &Geodetic,
    ) -> Result<Option<SurfaceRadiometry>, SceneryError> {
        Ok(Some(REGOLITH))
    }
}

/// A world's terrain, ready to hand to a renderer.
#[derive(Debug, Clone)]
pub struct WorldScene {
    /// The terrain geometry.
    pub mesh: GlobeMesh,
    /// The drape, if the world has imagery.
    pub atlas: Option<OrthoAtlas>,
}

fn whole_body_at(
    body: BodyEllipsoid,
    height_m: f64,
    radiometry: Option<&dyn RadiometricSource>,
) -> Result<GlobeMesh, ScenarioError> {
    let region =
        TerrainRegion::whole_body(body, GLOBE_LAT_PATCHES, GLOBE_LON_PATCHES, GLOBE_PATCH_GRID);
    let surface = LevelSurface { height_m };
    let sources = TerrainSources {
        elevation: Some(&surface),
        radiometry,
        ortho: None,
    };
    Ok(GlobeMesh::build_region(&region, sources)?)
}

/// The corners of `footprint`: north-west, then south-east.
fn footprint_corners(footprint: &OrthoFootprint) -> Result<(Geodetic, Geodetic), ScenarioError> {
    let tile = |x, y| TileId {
        zoom: footprint.zoom,
        x,
        y,
    };
    let corner = |id: TileId, u: f64, v: f64| {
        tile_point_to_geodetic(id, u, v).ok_or(SceneryError::TileOutOfRange { tile: id })
    };
    let north_west = corner(tile(footprint.west, footprint.north), 0.0, 0.0)?;
    let south_east = corner(
        tile(
            footprint.west + footprint.tiles_wide.get() - 1,
            footprint.north + footprint.tiles_high.get() - 1,
        ),
        1.0,
        1.0,
    )?;
    Ok((north_west, south_east))
}

fn delaware_scene() -> Result<WorldScene, ScenarioError> {
    let spec = SyntheticCorridorSpec::delaware();
    let catalog = CatalogScenery::synthetic_corridor(&spec)?;
    catalog_region_scene(&catalog, spec.tile_px)
}

fn aerodrome_scene(aerodrome: AerodromeId) -> Result<WorldScene, ScenarioError> {
    let catalog = CatalogScenery::synthetic_aerodrome(aerodrome)?;
    let tile_px = match aerodrome {
        AerodromeId::DoverKdov => SyntheticCorridorSpec::dover_kdov().tile_px,
        AerodromeId::CentralJersey47N => SyntheticCorridorSpec::central_jersey_47n().tile_px,
        AerodromeId::NewarkKewr => SyntheticCorridorSpec::newark_kewr().tile_px,
    };
    catalog_region_scene(&catalog, tile_px)
}

fn catalog_region_scene(
    catalog: &CatalogScenery,
    tile_px: u32,
) -> Result<WorldScene, ScenarioError> {
    let footprint =
        OrthoFootprint::covering(catalog.imagery_tiles()).ok_or(SceneryError::InvalidCatalog {
            detail: "the corridor has no imagery".to_owned(),
        })?;
    let atlas = OrthoAtlas::from_source(catalog, footprint, tile_px)?;
    let (north_west, south_east) = footprint_corners(&footprint)?;
    let region = TerrainRegion {
        body: BodyEllipsoid::WGS84,
        south_rad: south_east.lat_rad + COVERAGE_MARGIN_RAD,
        west_rad: north_west.lon_rad + COVERAGE_MARGIN_RAD,
        lat_span_rad: north_west.lat_rad - south_east.lat_rad - 2.0 * COVERAGE_MARGIN_RAD,
        lon_span_rad: south_east.lon_rad - north_west.lon_rad - 2.0 * COVERAGE_MARGIN_RAD,
        lat_patches: DELAWARE_LAT_PATCHES,
        lon_patches: DELAWARE_LON_PATCHES,
        grid: DELAWARE_GRID,
    };
    let terrain = GlobeMesh::build_region(
        &region,
        TerrainSources {
            elevation: Some(catalog),
            ortho: Some(&footprint),
            ..TerrainSources::default()
        },
    )?;
    let mut mesh = whole_body_at(BodyEllipsoid::WGS84, -EARTH_GLOBE_SINK_M, None)?;
    mesh.append(&terrain)?;
    Ok(WorldScene {
        mesh,
        atlas: Some(atlas),
    })
}

fn lunar_scene() -> Result<WorldScene, ScenarioError> {
    let scenery = LunarTranquilityScenery::apollo_11_landing();
    let south_west = scenery.at_local(-LUNAR_HALF_EAST_M, -LUNAR_HALF_NORTH_M);
    let north_east = scenery.at_local(LUNAR_HALF_EAST_M, LUNAR_HALF_NORTH_M);
    let region = TerrainRegion {
        body: BodyEllipsoid::MOON_IAU,
        south_rad: south_west.lat_rad,
        west_rad: south_west.lon_rad,
        lat_span_rad: north_east.lat_rad - south_west.lat_rad,
        lon_span_rad: north_east.lon_rad - south_west.lon_rad,
        lat_patches: LUNAR_LAT_PATCHES,
        lon_patches: LUNAR_LON_PATCHES,
        grid: LUNAR_GRID,
    };
    let terrain = GlobeMesh::build_region(
        &region,
        TerrainSources {
            elevation: Some(&scenery),
            radiometry: Some(&scenery),
            ..TerrainSources::default()
        },
    )?;
    let mut mesh = whole_body_at(
        BodyEllipsoid::MOON_IAU,
        -MOON_GLOBE_SINK_M,
        Some(&BareRegolith),
    )?;
    mesh.append(&terrain)?;
    Ok(WorldScene { mesh, atlas: None })
}

impl ScenarioWorld {
    /// Builds the world's terrain: the detailed region, where there is one,
    /// on a coarse globe that carries the view out to the horizon. Scenery is
    /// generated here, so this takes noticeable time.
    pub fn scene(self) -> Result<WorldScene, ScenarioError> {
        match self {
            Self::EarthGlobe => Ok(WorldScene {
                mesh: whole_body_at(BodyEllipsoid::WGS84, 0.0, None)?,
                atlas: None,
            }),
            Self::DelawareCorridor => delaware_scene(),
            Self::CentralJersey47N => aerodrome_scene(AerodromeId::CentralJersey47N),
            Self::NewarkKewr => aerodrome_scene(AerodromeId::NewarkKewr),
            Self::LunarTranquility => lunar_scene(),
            Self::MarsGlobe => Ok(WorldScene {
                mesh: whole_body_at(BodyEllipsoid::MARS_IAU, 0.0, None)?,
                atlas: None,
            }),
        }
    }
}
