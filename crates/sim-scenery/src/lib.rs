//! Scenery sources the renderer and the flight model draw on.
//!
//! Each kind of scenery is a trait so the delivery mechanism stays outside
//! rendering and physics. [`CatalogScenery`] serves Web Mercator Terrarium
//! elevation and orthoimagery tiles from memory, from a `Navigate` reference
//! catalog, or from a deterministic synthetic corridor;
//! [`LunarTranquilityScenery`] is the procedural Apollo 11 landing site.
//! Absent data is `Ok(None)` everywhere, never a made-up sea level.
//! [`AuthoritativeWorldSurface`] combines a catalog, runway strips and the
//! geoid into the one ground every consumer queries through
//! [`WorldSurfaceQuery`]; a [`LiveDemBridge`] streams Terrarium tiles into
//! it from loader threads. [`AerodromeRegistry`] lists the aerodromes a
//! world can be entered at, and [`render_luma_view`] is a CPU pinhole camera
//! over a catalog for headless sensor simulation.
//! Photogrammetry is deferred: its source is a [`Deferred`] stub that fails
//! with a typed error.

mod aerodrome;
mod aerodrome_registry;
mod airport_db;
mod catalog;
mod deferred;
mod live_dem;
mod lunar;
mod mercator;
mod noise;
mod sensor_render;
mod sources;
mod terrarium;
mod world_query;

pub use aerodrome::{AerodromeId, AirspaceControl, OsmAerodromeDiagram};
pub use aerodrome_registry::{
    AerodromeRecord, AerodromeRegistry, AerodromeSummary, GLIDESLOPE_RAD, RunwaySummary,
    SHORT_FINAL_DISTANCE_M, SpawnAnchor, SpawnKind, THRESHOLD_CROSSING_HEIGHT_M,
};
pub use airport_db::{
    AerodromeTier, AirportDatabase, AirportFrequency, AirportIds, AirportIdx, AirportRecord,
    AirportRenewalSource, CompactCode, FaaNasrCsv, FrequencyRole, ICONIC_AIRPORTS,
    LandingRunwayGeometry, OurAirportsCsv, OurAirportsName, RenewalScope, RunwayEndRecord,
    RunwayRecord, RunwaySurfaceKind, globe_visible_radius_deg, magnetic_heading_deg,
    our_airports_us_names, parse_faa_nasr, parse_our_airports,
};
pub use catalog::{CatalogScenery, CatalogVerticalDatum, SyntheticCorridorSpec};
pub use deferred::Deferred;
pub use live_dem::LiveDemBridge;
pub use lunar::{
    APOLLO_11_SITE_LAT_DEG, APOLLO_11_SITE_LON_DEG, LITTLE_WEST_CRATER, LunarCrater,
    LunarTranquilityScenery, WEST_CRATER,
};
pub use mercator::{
    MAX_MERCATOR_LAT_DEG, MAX_TILE_ZOOM, TileCoord, project_to_tile, tile_point_to_geodetic,
};
pub use sensor_render::{LumaImage, render_luma_view};
pub use sources::{
    ElevationSource, EllipsoidSurface, ImageryRaster, ImagerySource, MeshNode,
    PhotogrammetrySource, RadiometricSource, SceneryError, SurfaceRadiometry, TileId,
};
pub use terrarium::{TerrariumTile, decode_terrarium, encode_terrarium};
pub use world_query::{
    AuthoritativeWorldSurface, RayHit, RunwayStrip, SurfaceSample, SurfaceSampleSource,
    WorldSurfaceQuery,
};
