//! Eager (`Conformal3d`) and on-demand (`Chart2d`) [`HeadlessMap`] materialization for
//! [`super::InstrumentMapSurface`].

use maplibre::{
    celestial::EcefDirection,
    headless::{TerrainDrapeMode, ViewSlotConfig, ViewSlotId, map::HeadlessMap},
    projection::ProjectionType,
    render::view_state::NavigationMode,
    style::Style,
    window::PhysicalSize,
};
use sim_scenery::CatalogScenery;

use super::{InstrumentMapMode, SharedMapRuntime};
use crate::{
    MapAdapterError,
    runtime::{instrument_plugins_for, tile_json::resolve_or_strip_tile_json, validate_size},
    supply_catalog_dem_synchronously, tile_id_to_world_coords,
};

pub(super) struct DeferredChart {
    pub(super) style: Style,
    pub(super) catalog_dem: Option<CatalogScenery>,
    pub(super) sun_dir: Option<EcefDirection>,
}

pub(super) enum SurfaceBackend {
    Active {
        map: Box<HeadlessMap>,
        slot: ViewSlotId,
    },
    DeferredChart(Box<DeferredChart>),
}

impl SurfaceBackend {
    pub(super) fn new(
        runtime: &SharedMapRuntime,
        mode: InstrumentMapMode,
        style: Style,
        phys: PhysicalSize,
    ) -> Result<Self, MapAdapterError> {
        match mode {
            InstrumentMapMode::Conformal3d { .. } => {
                let (map, slot) = materialize(runtime, mode, style, phys, None, None, None)?;
                Ok(Self::Active {
                    map: Box::new(map),
                    slot,
                })
            }
            InstrumentMapMode::Chart2d { .. } => Ok(Self::DeferredChart(Box::new(DeferredChart {
                style,
                catalog_dem: None,
                sun_dir: None,
            }))),
        }
    }

    pub(super) fn ensure_active(
        &mut self,
        runtime: &SharedMapRuntime,
        mode: InstrumentMapMode,
        size: [u32; 2],
        pixel_ratio: Option<f64>,
    ) -> Result<(&mut HeadlessMap, ViewSlotId), MapAdapterError> {
        if let Self::DeferredChart(deferred) = self {
            let [width, height] = size;
            let phys = validate_size(width, height)?;
            let (map, slot) = materialize(
                runtime,
                mode,
                deferred.style.clone(),
                phys,
                pixel_ratio,
                deferred.sun_dir,
                deferred.catalog_dem.as_ref(),
            )?;
            *self = Self::Active {
                map: Box::new(map),
                slot,
            };
        }
        match self {
            Self::Active { map, slot } => Ok((map.as_mut(), *slot)),
            Self::DeferredChart(_) => Err(MapAdapterError::InstrumentModeMismatch {
                id: String::new(),
                requested: "Chart2d",
            }),
        }
    }

    pub(super) fn supply_dem(
        &mut self,
        runtime: &SharedMapRuntime,
        scenery: &CatalogScenery,
    ) -> Result<usize, MapAdapterError> {
        match self {
            Self::Active { map, .. } => {
                runtime.with_runtime_context(|| supply_catalog_dem_synchronously(map, scenery))
            }
            Self::DeferredChart(deferred) => {
                let count = scenery
                    .elevation_tiles()
                    .filter(|&t| {
                        scenery.elevation_tile(t).is_some() && tile_id_to_world_coords(t).is_some()
                    })
                    .count();
                deferred.catalog_dem = Some(scenery.clone());
                Ok(count)
            }
        }
    }

    pub(super) fn terrain_drape_mode(&self, runtime: &SharedMapRuntime) -> TerrainDrapeMode {
        match self {
            Self::Active { map, .. } => map.terrain_drape_mode(),
            Self::DeferredChart(_) => {
                runtime
                    .device_tier()
                    .belief_chart_settings(runtime.color_format())
                    .terrain_drape_mode
            }
        }
    }
}

fn materialize(
    runtime: &SharedMapRuntime,
    mode: InstrumentMapMode,
    mut style: Style,
    phys: PhysicalSize,
    pixel_ratio: Option<f64>,
    sun_dir: Option<EcefDirection>,
    catalog_dem: Option<&CatalogScenery>,
) -> Result<(HeadlessMap, ViewSlotId), MapAdapterError> {
    let tier = runtime.device_tier();
    let settings = match mode {
        InstrumentMapMode::Conformal3d {
            initial_reference_altitude_m,
        } => tier.belief_svs_settings(runtime.color_format(), initial_reference_altitude_m),
        InstrumentMapMode::Chart2d { .. } => tier.belief_chart_settings(runtime.color_format()),
    };
    let (kernel, renderer) = runtime.create_renderer(16, 16, settings)?;
    resolve_or_strip_tile_json(runtime, &mut style, kernel.source_client());
    let free_globe = matches!(mode, InstrumentMapMode::Chart2d { .. })
        && style
            .projection
            .as_ref()
            .is_some_and(|p| p.projection_type == ProjectionType::VerticalPerspective);
    let plugins = instrument_plugins_for(mode, tier);
    let mut map = runtime
        .with_runtime_context(|| HeadlessMap::new(style, renderer, kernel, plugins))
        .map_err(|source| MapAdapterError::Assemble { source })?;
    map.set_available_memory(tier.available_memory_bytes());
    if free_globe {
        map.set_navigation_mode(NavigationMode::FreeGlobe)
            .map_err(|source| MapAdapterError::Navigation { source })?;
    }
    if let Some(ratio) = pixel_ratio {
        map.set_pixel_ratio(ratio);
    }
    if let Some(dir) = sun_dir {
        map.set_sun_direction(Some(dir));
    }
    if matches!(mode, InstrumentMapMode::Conformal3d { .. }) {
        runtime
            .with_runtime_context(|| crate::scenery_loader::seed_sea_level_root_dem(&mut map))?;
    }
    if let Some(scenery) = catalog_dem {
        let _supplied =
            runtime.with_runtime_context(|| supply_catalog_dem_synchronously(&mut map, scenery))?;
    }
    let slot = map.register_view_slot(ViewSlotConfig {
        size: phys,
        depth_readout: false,
    });
    Ok((map, slot))
}
