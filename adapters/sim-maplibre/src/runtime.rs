//! Shared GPU + worker runtime, the simulator's physical scene, and instrument maps.
//!
//! [`SharedMapRuntime`] owns one [`HostGpu`], one [`SharedMapAssets`] cache
//! (`128 MiB` decoded `Arc<DemTile>` / raster / glyph cache + [`SharedPipelines`]),
//! and (on native targets) one bounded Tokio tile worker pool sized to the host's cores.
//!
//! [`TruthSceneMap`] and every [`InstrumentMapSurface`] of an [`AvionicsMapSet`] share
//! immutable static tiles and compiled pipelines through [`SharedMapRuntime`] while
//! keeping their ECS `World`s, overlays, and track sets strictly isolated.

use std::rc::Rc;
#[cfg(not(target_arch = "wasm32"))]
use std::sync::Arc;

use maplibre::{
    headless::{
        HeadlessRendererError, SharedMapAssets,
        create_headless_renderer_on_host_gpu_with_shared_assets, environment::HeadlessEnvironment,
    },
    io::resource_loader::SharedLoader,
    kernel::Kernel,
    plugin::Plugin,
    render::{Renderer, host_gpu::HostGpu, resource::SharedPipelines, settings::RendererSettings},
    schedule::Schedule,
    tcs::world::World,
    terrain::{DrapeBudget, PolarPoleElevations, TerrainCoverageIndex},
    window::PhysicalSize,
};

use crate::{CockpitDeviceTier, MapAdapterError};

mod instrument_map;
mod map_set;
mod pacing;
mod pixel_ratio;
#[cfg(not(target_arch = "wasm32"))]
mod readback;
mod svs_dem;
mod tile_json;
mod truth;

pub use instrument_map::{
    ChartCameraSpec, ChartFrameOutcome, ConformalMapFrame, InstrumentMapMode, InstrumentMapSpec,
    InstrumentMapSurface,
};
pub use map_set::AvionicsMapSet;
pub(crate) use pixel_ratio::pixel_ratio_update;
#[cfg(not(target_arch = "wasm32"))]
pub(crate) use readback::read_texture_rgba_blocking;
pub(crate) use tile_json::resolve_or_strip_tile_json;
pub use truth::{
    AircraftInstance, CelestialDisc, CelestialEarthShadow, CelestialSkyInput, CelestialStar,
    CelestialTwilight, TruthSceneConfig, TruthSceneFrame, TruthSceneMap, TruthSlotMask,
};

/// Shared GPU, pipeline, decoded-asset, and async tile-worker context for all
/// `maplibre` scenes in a process.
#[derive(Clone, Debug)]
pub struct SharedMapRuntime {
    gpu: HostGpu,
    shared_assets: SharedMapAssets,
    tier: CockpitDeviceTier,
    color_format: wgpu::TextureFormat,
    #[cfg(not(target_arch = "wasm32"))]
    tokio_runtime: Arc<tokio::runtime::Runtime>,
}

impl SharedMapRuntime {
    /// Initializes a shared runtime over `gpu` with an optional custom [`SharedLoader`]
    /// (for example [`crate::CatalogTileLoader::into_shared_loader`]).
    pub fn new(
        gpu: HostGpu,
        tier: CockpitDeviceTier,
        color_format: wgpu::TextureFormat,
        loader: Option<SharedLoader>,
    ) -> Result<Self, MapAdapterError> {
        let shared_assets = match loader {
            Some(l) => SharedMapAssets::with_loader(l),
            None => SharedMapAssets::new(),
        };
        #[cfg(not(target_arch = "wasm32"))]
        let tokio_runtime = Arc::new(
            tokio::runtime::Builder::new_multi_thread()
                .worker_threads(tile_worker_threads())
                .thread_name("sim-map-tiles")
                .enable_all()
                .build()
                .map_err(|source| MapAdapterError::RuntimeInit { source })?,
        );
        Ok(Self {
            gpu,
            shared_assets,
            tier,
            color_format,
            #[cfg(not(target_arch = "wasm32"))]
            tokio_runtime,
        })
    }

    /// Host `wgpu` handles shared by all map worlds.
    #[must_use]
    pub fn host_gpu(&self) -> &HostGpu {
        &self.gpu
    }

    /// Shared decoded-tile cache and compiled render-pipeline registry.
    #[must_use]
    pub fn shared_assets(&self) -> &SharedMapAssets {
        &self.shared_assets
    }

    /// Shared compiled WGSL shader and pipeline cache.
    #[must_use]
    pub fn shared_pipelines(&self) -> &SharedPipelines {
        self.shared_assets.pipelines()
    }

    /// Configured hardware memory/bandwidth tier.
    #[must_use]
    pub const fn device_tier(&self) -> CockpitDeviceTier {
        self.tier
    }

    /// Color attachment format used by offscreen viewports.
    #[must_use]
    pub const fn color_format(&self) -> wgpu::TextureFormat {
        self.color_format
    }

    pub(crate) fn create_renderer(
        &self,
        width: u32,
        height: u32,
        settings: RendererSettings,
    ) -> Result<(Kernel<HeadlessEnvironment>, Renderer), MapAdapterError> {
        self.with_runtime_context(|| {
            create_headless_renderer_on_host_gpu_with_shared_assets(
                width,
                height,
                self.gpu.clone(),
                settings,
                &self.shared_assets,
            )
            .map_err(|source: HeadlessRendererError| MapAdapterError::RendererInit { source })
        })
    }

    /// Runs `future` to completion on the tile worker pool.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn block_on<F: std::future::Future>(&self, future: F) -> F::Output {
        self.tokio_runtime.block_on(future)
    }

    /// Runs `future` on the tile worker pool without waiting for it.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn spawn_detached<F>(&self, future: F)
    where
        F: std::future::Future<Output = ()> + Send + 'static,
    {
        let _task = self.tokio_runtime.spawn(future);
    }

    pub(crate) fn with_runtime_context<R>(&self, f: impl FnOnce() -> R) -> R {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let _guard = self.tokio_runtime.enter();
            f()
        }
        #[cfg(target_arch = "wasm32")]
        {
            f()
        }
    }
}

/// Decode workers for DEM and imagery tiles: enough that a burst of arrivals while
/// taxiing does not queue behind two threads, few enough to leave cores for the frame.
#[cfg(not(target_arch = "wasm32"))]
fn tile_worker_threads() -> usize {
    std::thread::available_parallelism().map_or(4, |n| n.get().clamp(4, 8))
}

pub(crate) struct DrapeBudgetPlugin(pub(crate) DrapeBudget);

impl Plugin<HeadlessEnvironment> for DrapeBudgetPlugin {
    fn build(
        &self,
        _schedule: &mut Schedule,
        _kernel: Rc<Kernel<HeadlessEnvironment>>,
        world: &mut World,
        _renderer: &mut maplibre::render::Renderer,
    ) -> Result<(), maplibre::plugin::PluginError> {
        world.resources.insert(self.0);
        Ok(())
    }
}

/// Arctic sea-level (`0 m`) and Amundsen-Scott South Pole plateau (`2,835 m` MSL)
/// datum elevations for Web Mercator's `±85.0511°..±90°` terrain polar caps.
pub(crate) const EARTH_POLAR_ELEVATIONS: PolarPoleElevations = PolarPoleElevations {
    north_meters: 0.0,
    south_meters: 2_835.0,
};

pub(crate) struct PolarElevationsPlugin(pub(crate) PolarPoleElevations);

impl Plugin<HeadlessEnvironment> for PolarElevationsPlugin {
    fn build(
        &self,
        _schedule: &mut Schedule,
        _kernel: Rc<Kernel<HeadlessEnvironment>>,
        world: &mut World,
        _renderer: &mut maplibre::render::Renderer,
    ) -> Result<(), maplibre::plugin::PluginError> {
        world.resources.insert(self.0);
        world
            .resources
            .insert(TerrainCoverageIndex::default().with_poles(self.0));
        Ok(())
    }
}

pub(crate) fn validate_size(width: u32, height: u32) -> Result<PhysicalSize, MapAdapterError> {
    PhysicalSize::new(width, height).ok_or(MapAdapterError::InvalidViewportSize { width, height })
}

pub(super) fn instrument_plugins_for(
    mode: InstrumentMapMode,
    tier: CockpitDeviceTier,
) -> Vec<Box<dyn Plugin<HeadlessEnvironment>>> {
    use maplibre::{
        background::BackgroundPlugin,
        hillshade::HillshadePlugin,
        overlay::OverlayPlugin,
        raster::{DefaultRasterTransferables, RasterPlugin},
        render::RenderPlugin,
        terrain::{TerrainPlugin, transferables::DefaultDemTransferables},
        vector::{DefaultVectorTransferables, VectorPlugin},
    };
    match mode {
        InstrumentMapMode::Conformal3d { .. } => vec![
            Box::new(RenderPlugin),
            Box::new(BackgroundPlugin),
            Box::new(TerrainPlugin::<DefaultDemTransferables>::default()),
            Box::new(PolarElevationsPlugin(EARTH_POLAR_ELEVATIONS)),
            Box::new(OverlayPlugin),
        ],
        InstrumentMapMode::Chart2d { .. } => vec![
            Box::new(RenderPlugin),
            Box::new(BackgroundPlugin),
            Box::new(RasterPlugin::<DefaultRasterTransferables>::default()),
            Box::new(VectorPlugin::<DefaultVectorTransferables>::default()),
            Box::new(HillshadePlugin),
            Box::new(TerrainPlugin::<DefaultDemTransferables>::default()),
            Box::new(DrapeBudgetPlugin(tier.otw_drape_budget())),
            Box::new(PolarElevationsPlugin(EARTH_POLAR_ELEVATIONS)),
            Box::new(OverlayPlugin),
        ],
    }
}
