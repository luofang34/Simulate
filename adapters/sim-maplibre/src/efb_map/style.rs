//! The EFB map's style: the dark aeronautical chart with an approach-plate image layer and
//! airspace boundaries between the basemap (relief, hillshade, vector features) and the
//! airport surface diagram. Ownship, route and traffic symbology draws above every layer
//! as the slot's overlay.

use std::sync::Arc;

use maplibre::{
    background::BackgroundPlugin,
    headless::environment::HeadlessEnvironment,
    hillshade::HillshadePlugin,
    hud::NavigationHudPlugin,
    overlay::OverlayPlugin,
    plugin::Plugin,
    raster::{DefaultRasterTransferables, RasterPlugin},
    render::RenderPlugin,
    style::{Style, source::Source},
    terrain::{TerrainPlugin, transferables::DefaultDemTransferables},
    vector::{DefaultVectorTransferables, VectorPlugin},
};
use serde_json::{Value, json};

use crate::{
    CockpitDeviceTier, MapAdapterError, SceneryEndpoints, belief_aviation_chart_style, chart_url,
    runtime::{DrapeBudgetPlugin, EARTH_POLAR_ELEVATIONS, PolarElevationsPlugin},
};

/// Image source carrying the approach plate.
pub(super) const PLATE_SOURCE: &str = "efb-plate-source";
/// Raster layer drawing the approach plate.
pub(super) const PLATE_LAYER: &str = "efb-plate-layer";
/// Inline GeoJSON source of airspace boundaries.
pub(super) const AIRSPACE_SOURCE: &str = "efb-airspace";
/// Line layer drawing airspace boundaries.
pub(super) const AIRSPACE_LAYER: &str = "efb-airspace-line";
/// Source of the airport surface diagram in the chart style.
pub(super) const AERODROME_SOURCE: &str = "aerodrome";
/// Chart id of the transparent pixel the plate source shows while no plate is set.
pub(super) const NO_PLATE_CHART_ID: &str = "efb-no-plate";
/// Default plate opacity, so the moving map stays readable beneath it.
pub(super) const DEFAULT_PLATE_OPACITY: f32 = 0.78;

/// The chart style with the EFB's plate and airspace layers, the plate hidden, and the
/// plan's layers (see [`super::plan_style`]).
pub(super) fn efb_style(
    endpoints: &SceneryEndpoints,
    aerodrome_geojson: Value,
) -> Result<Style, MapAdapterError> {
    let mut style = belief_aviation_chart_style(endpoints, aerodrome_geojson)?;
    let diagram = first_aerodrome_layer(&style);
    let plate_source: Source = serde_json::from_value(json!({
        "type": "image",
        "url": chart_url(NO_PLATE_CHART_ID),
        "coordinates": [[0.0, 0.001], [0.001, 0.001], [0.001, 0.0], [0.0, 0.0]]
    }))
    .map_err(|source| MapAdapterError::StyleParse { source })?;
    let airspace_source: Source = serde_json::from_value(json!({
        "type": "geojson",
        "data": { "type": "FeatureCollection", "features": [] }
    }))
    .map_err(|source| MapAdapterError::StyleParse { source })?;
    let mutation = |target: &str| {
        let target = target.to_owned();
        move |source| MapAdapterError::StyleMutation { target, source }
    };
    style
        .add_source(PLATE_SOURCE, plate_source)
        .map_err(mutation(PLATE_SOURCE))?;
    style
        .add_source(AIRSPACE_SOURCE, airspace_source)
        .map_err(mutation(AIRSPACE_SOURCE))?;
    style
        .add_layer(
            json!({
                "id": PLATE_LAYER, "type": "raster", "source": PLATE_SOURCE,
                "maxzoom": 13.8,
                "layout": { "visibility": "none" },
                "paint": { "raster-opacity": DEFAULT_PLATE_OPACITY, "raster-fade-duration": 0 }
            }),
            diagram.as_deref(),
        )
        .map_err(mutation(PLATE_LAYER))?;
    style
        .add_layer(
            json!({
                "id": AIRSPACE_LAYER, "type": "line", "source": AIRSPACE_SOURCE,
                "metadata": { "maplibre-rs:terrain-structure": "overlay" },
                "paint": { "line-color": "#60a5fa", "line-width": 1.6, "line-opacity": 0.85 }
            }),
            diagram.as_deref(),
        )
        .map_err(mutation(AIRSPACE_LAYER))?;
    super::plan_style::add_plan_layers(&mut style, endpoints, Some(PLATE_LAYER))?;
    // The relief stands up when a view tilts; looking straight down it is the same chart.
    style.terrain = Some(maplibre::style::terrain::TerrainSpecification {
        source: "dem".to_owned(),
        exaggeration: 1.0,
    });
    Ok(style)
}

/// Ids of the airport surface diagram's layers.
pub(super) fn aerodrome_layer_ids(style: &Style) -> Vec<String> {
    style
        .layers
        .iter()
        .filter(|layer| layer.source.as_deref() == Some(AERODROME_SOURCE))
        .map(|layer| layer.id.clone())
        .collect()
}

fn first_aerodrome_layer(style: &Style) -> Option<String> {
    aerodrome_layer_ids(style).into_iter().next()
}

/// The plugins the EFB map draws with.
pub(super) fn plugins(tier: CockpitDeviceTier) -> Vec<Box<dyn Plugin<HeadlessEnvironment>>> {
    vec![
        Box::new(RenderPlugin),
        Box::new(BackgroundPlugin),
        Box::new(RasterPlugin::<DefaultRasterTransferables>::default()),
        Box::new(VectorPlugin::<DefaultVectorTransferables>::default()),
        Box::new(HillshadePlugin),
        Box::new(TerrainPlugin::<DefaultDemTransferables>::default()),
        Box::new(DrapeBudgetPlugin(tier.otw_drape_budget())),
        Box::new(PolarElevationsPlugin(EARTH_POLAR_ELEVATIONS)),
        Box::new(OverlayPlugin),
        Box::new(NavigationHudPlugin),
    ]
}

/// A 1 × 1 transparent PNG for the plate source while no plate is shown.
pub(super) fn transparent_pixel_png() -> Result<Arc<[u8]>, MapAdapterError> {
    let mut cursor = std::io::Cursor::new(Vec::new());
    image::RgbaImage::new(1, 1)
        .write_to(&mut cursor, image::ImageFormat::Png)
        .map_err(|error| MapAdapterError::ChartRaster {
            chart_id: NO_PLATE_CHART_ID.to_owned(),
            detail: error.to_string(),
        })?;
    Ok(Arc::from(cursor.into_inner()))
}
