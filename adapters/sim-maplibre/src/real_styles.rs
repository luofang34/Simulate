//! Real-world scenery endpoints and the styles drawn from them: photo scenery for the
//! simulator's physical scene, DEM relief for synthetic vision, and a dark aeronautical
//! moving-map / airport-surface chart with no photo imagery.

use maplibre::style::Style;
use serde_json::{Value, json};

use crate::{CATALOG_DEM_URL_TEMPLATE, CATALOG_ORTHO_URL_TEMPLATE, MapAdapterError};

/// AWS Terrain Tiles (Mapzen Terrarium encoding) DEM template.
pub const AWS_TERRARIUM_DEM_URL_TEMPLATE: &str =
    "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";
/// Esri World Imagery orthophoto template (`{z}/{y}/{x}` row-major order).
pub const ESRI_WORLD_IMAGERY_URL_TEMPLATE: &str = "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";
/// OpenFreeMap planet OpenMapTiles-schema vector TileJSON.
pub const OPENFREEMAP_PLANET_TILEJSON_URL: &str = "https://tiles.openfreemap.org/planet";

/// Where scenery tiles come from: the internet, or an offline [`crate::CatalogTileLoader`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SceneryEndpoints {
    /// Terrarium DEM tile template.
    pub dem_tiles: String,
    /// Deepest DEM zoom served.
    pub dem_maxzoom: u8,
    /// Orthoimagery tile template.
    pub ortho_tiles: String,
    /// Deepest orthoimagery zoom served.
    pub ortho_maxzoom: u8,
    /// OpenMapTiles-schema vector TileJSON, when vector basemap layers are available.
    pub vector_tilejson: Option<String>,
}

impl SceneryEndpoints {
    /// AWS Terrarium DEM, Esri World Imagery, and OpenFreeMap vector tiles.
    #[must_use]
    pub fn real_world() -> Self {
        Self {
            dem_tiles: AWS_TERRARIUM_DEM_URL_TEMPLATE.to_owned(),
            dem_maxzoom: 15,
            ortho_tiles: ESRI_WORLD_IMAGERY_URL_TEMPLATE.to_owned(),
            ortho_maxzoom: 18,
            vector_tilejson: Some(OPENFREEMAP_PLANET_TILEJSON_URL.to_owned()),
        }
    }

    /// The offline `https://sim.catalog/` endpoints served by a catalog loader.
    #[must_use]
    pub fn catalog(dem_maxzoom: u8, ortho_maxzoom: u8) -> Self {
        Self {
            dem_tiles: CATALOG_DEM_URL_TEMPLATE.to_owned(),
            dem_maxzoom,
            ortho_tiles: CATALOG_ORTHO_URL_TEMPLATE.to_owned(),
            ortho_maxzoom,
            vector_tilejson: None,
        }
    }

    fn dem_source(&self, maxzoom: u8) -> Value {
        json!({
            "type": "raster-dem",
            "tiles": [self.dem_tiles],
            "encoding": "terrarium",
            "tileSize": 256,
            "minzoom": 0,
            "maxzoom": maxzoom.min(self.dem_maxzoom)
        })
    }
}

/// Physical scene for out-the-window, optical sensors, and virtual cameras: photo
/// imagery draped on real terrain, sunlit snow/ice across the Antarctic polar cap
/// (`85°S..90°S`) where Web Mercator imagery ends, and nothing painted over airports.
pub fn truth_real_world_style(endpoints: &SceneryEndpoints) -> Result<Style, MapAdapterError> {
    parse(json!({
        "version": 8,
        "name": "Simulate physical scene",
        "projection": { "type": "vertical-perspective" },
        "sources": {
            "dem": endpoints.dem_source(15),
            "antarctic-ice": antarctic_polar_ice_source(),
            "ortho": {
                "type": "raster",
                "tiles": [endpoints.ortho_tiles],
                "tileSize": 256,
                "minzoom": 0,
                "maxzoom": endpoints.ortho_maxzoom
            }
        },
        "terrain": { "source": "dem", "exaggeration": 1.0 },
        "layers": [
            { "id": "sky-bg", "type": "background", "paint": { "background-color": "#0b1626" } },
            {
                "id": "antarctic-ice-fill",
                "type": "fill",
                "source": "antarctic-ice",
                "paint": { "fill-color": "#dce6f0" }
            },
            {
                "id": "ortho-surface",
                "type": "raster",
                "source": "ortho",
                "paint": { "raster-fade-duration": 0, "raster-resampling": "linear" }
            }
        ]
    }))
}

/// Synthetic-vision relief: DEM only, shaded by procedural relief, zero photo raster, under
/// an avionics sky with no atmospheric scattering (blue above, earth tone below the horizon).
pub fn belief_real_svs_style(endpoints: &SceneryEndpoints) -> Result<Style, MapAdapterError> {
    parse(json!({
        "version": 8,
        "name": "Indicate SVS relief",
        "projection": { "type": "vertical-perspective" },
        "sources": { "dem": endpoints.dem_source(14) },
        "terrain": { "source": "dem", "exaggeration": 1.0 },
        "sky": {
            "sky-color": "#0c3b75",
            "horizon-color": "#2c6db8",
            "sky-horizon-blend": 0.55
        },
        "layers": [
            { "id": "svs-bg", "type": "background", "paint": { "background-color": "#3b2d1f" } }
        ]
    }))
}

/// Dark aeronautical moving map and airport-surface diagram: hypsometric relief and
/// hillshade, muted vector basemap, and high-contrast OSM `aeroway=*` surfaces on top.
pub fn belief_aviation_chart_style(
    endpoints: &SceneryEndpoints,
    aerodrome_geojson: Value,
) -> Result<Style, MapAdapterError> {
    let mut sources = json!({
        "dem": endpoints.dem_source(14),
        "antarctic-ice": antarctic_polar_ice_source(),
        "aerodrome": { "type": "geojson", "data": aerodrome_geojson, "maxzoom": 16, "buffer": 128 }
    });
    let mut layers = vec![
        json!({ "id": "chart-bg", "type": "background", "paint": { "background-color": "#121923" } }),
        json!({
            "id": "antarctic-ice-fill",
            "type": "fill",
            "source": "antarctic-ice",
            "paint": { "fill-color": "#1d2733" }
        }),
        relief_layer(),
        json!({
            "id": "chart-hillshade",
            "type": "hillshade",
            "source": "dem",
            "paint": {
                "hillshade-exaggeration": 0.28,
                "hillshade-shadow-color": "#05080d",
                "hillshade-highlight-color": "#3b4a5a",
                "hillshade-accent-color": "#0b1018"
            }
        }),
    ];
    if let (Some(url), Some(map)) = (&endpoints.vector_tilejson, sources.as_object_mut()) {
        map.insert(
            "openmaptiles".to_owned(),
            json!({ "type": "vector", "url": url }),
        );
        layers.extend(super::chart_layers::basemap_layers());
    }
    layers.extend(super::chart_layers::aerodrome_layers());
    parse(json!({
        "version": 8,
        "name": "Indicate aeronautical chart",
        "projection": { "type": "vertical-perspective" },
        "sources": sources,
        "layers": layers,
    }))
}

fn relief_layer() -> Value {
    json!({
        "id": "chart-relief",
        "type": "color-relief",
        "source": "dem",
        "paint": {
            "color-relief-opacity": 0.92,
            "color-relief-color": [
                "interpolate", ["linear"], ["elevation"],
                -50, "#0e1620",
                0, "#141f1c",
                100, "#18251f",
                300, "#1e2a21",
                600, "#262f23",
                1000, "#2f3326",
                2000, "#3b392d",
                4000, "#4a463b"
            ]
        }
    })
}

/// Inline `maxzoom: 0` GeoJSON covering `85°S..90°S` so `maplibre` derives polar cap
/// ice albedo (`SOUTH_POLE_Y`) both online and offline without painting north of `85°S`.
fn antarctic_polar_ice_source() -> Value {
    json!({
        "type": "geojson",
        "maxzoom": 0,
        "data": {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "properties": {},
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[
                        [-180.0, -85.0],
                        [0.0, -85.0],
                        [180.0, -85.0],
                        [180.0, -90.0],
                        [0.0, -90.0],
                        [-180.0, -90.0],
                        [-180.0, -85.0]
                    ]]
                }
            }]
        }
    })
}

fn parse(spec: Value) -> Result<Style, MapAdapterError> {
    serde_json::from_value(spec).map_err(|source| MapAdapterError::StyleParse { source })
}
