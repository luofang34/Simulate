//! Shared GPU + worker runtime and isolated `TruthScene` / `BeliefAvionics` maps.
//!
//! [`SharedMapRuntime`] owns one [`HostGpu`], one [`SharedMapAssets`] cache
//! (`128 MiB` decoded `Arc<DemTile>` / raster / glyph cache + [`SharedPipelines`]),
//! and (on native targets) one bounded 2-thread Tokio tile worker pool.
//!
//! [`TruthSceneMap`] and [`BeliefAvionicsMap`] share immutable static tiles and
//! compiled pipelines through [`SharedMapRuntime`] while keeping their ECS `World`s,
//! dynamic overlays, and track sets strictly isolated.

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
    render::{
        Renderer, graph::RenderGraph, host_gpu::HostGpu, resource::SharedPipelines,
        settings::RendererSettings,
    },
    schedule::Schedule,
    tcs::world::World,
    terrain::DrapeBudget,
    window::PhysicalSize,
};

use crate::{CockpitDeviceTier, MapAdapterError};

mod belief;
#[cfg(not(target_arch = "wasm32"))]
mod readback;
mod truth;

pub use belief::{BeliefAvionicsConfig, BeliefAvionicsMap, ChartCameraSpec, ChartFrameOutcome};
pub use truth::{TruthSceneConfig, TruthSceneFrame, TruthSceneMap};

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
                .worker_threads(2)
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

pub(crate) struct DrapeBudgetPlugin(pub(crate) DrapeBudget);

impl Plugin<HeadlessEnvironment> for DrapeBudgetPlugin {
    fn build(
        &self,
        _schedule: &mut Schedule,
        _kernel: Rc<Kernel<HeadlessEnvironment>>,
        world: &mut World,
        _graph: &mut RenderGraph,
    ) {
        world.resources.insert(self.0);
    }
}

pub(crate) fn validate_size(width: u32, height: u32) -> Result<PhysicalSize, MapAdapterError> {
    PhysicalSize::new(width, height).ok_or(MapAdapterError::InvalidViewportSize { width, height })
}
