//! Simulator-only `TruthSceneMap`: OTW cockpit camera, optional stereo companion,
//! and 30 Hz optical/EVS sensor slots over a single `HeadlessMap` world.

use std::time::Duration;

use flightbag::RunwayEndSpec;
use glam::DVec3;
use maplibre::{
    background::BackgroundPlugin,
    celestial::{EcefDirection, TwilightBand},
    headless::{
        CoveringPolicy, MultiViewFrame, SlotView, ViewSlotConfig, ViewSlotEntry, ViewSlotId,
        environment::HeadlessEnvironment, map::HeadlessMap,
    },
    overlay::{OverlayPlugin, ProbeResults},
    plugin::Plugin,
    raster::{DefaultRasterTransferables, RasterPlugin},
    render::RenderPlugin,
    starfield::StarfieldPlugin,
    style::Style,
    terrain::{TerrainPlugin, transferables::DefaultDemTransferables},
    vector::{DefaultVectorTransferables, VectorPlugin},
};
use sim_scenery::CatalogScenery;

use super::{DrapeBudgetPlugin, SharedMapRuntime, validate_size};
use crate::{
    ConformalFrameStamp, DynamicEntity3D, MapAdapterError, build_conformal_overlay,
    supply_catalog_dem_synchronously,
};

/// Viewport and overscan settings for [`TruthSceneMap`].
#[derive(Clone, Copy, Debug)]
pub struct TruthSceneConfig {
    /// Cockpit Out-the-Window (`OTW`) physical pixel dimensions `[width, height]`.
    pub otw_size: [u32; 2],
    /// Whether to register a companion stereo right-eye OTW slot (`CoveringPolicy::StereoReuse`).
    pub stereo_otw: bool,
    /// Whether the OTW slot resolves a 1× `Depth32Float` readout texture.
    pub otw_depth_readout: bool,
    /// Offscreen optical / EVS sensor physical pixel dimensions `[width, height]`.
    pub sensor_size: [u32; 2],
    /// Frustum tangent overscan (`>= 1.0`) for external-eye tile prefetch.
    pub request_overscan: f64,
}

impl Default for TruthSceneConfig {
    fn default() -> Self {
        Self {
            otw_size: [1280, 720],
            stereo_otw: false,
            otw_depth_readout: false,
            sensor_size: [800, 600],
            request_overscan: 1.15,
        }
    }
}

/// Per-frame inputs driving [`TruthSceneMap::render_frame`].
#[derive(Clone, Copy, Debug)]
pub struct TruthSceneFrame<'a> {
    /// Simulation frame timestamp for tile motion prefetch and animated paint properties.
    pub timestamp: Duration,
    /// Primary cockpit OTW camera stamp.
    pub otw_stamp: ConformalFrameStamp,
    /// Optional right-eye stereo OTW camera stamp (when `stereo_otw` is enabled).
    pub stereo_right_stamp: Option<ConformalFrameStamp>,
    /// Optional 30 Hz optical / EVS sensor camera stamp; when `None`, the sensor slot is
    /// skipped this frame (`0.0 ms` GPU) without evicting its tiles or drapes.
    pub sensor_stamp: Option<ConformalFrameStamp>,
    /// Optional runway outline drawn into depth-tested overlays.
    pub runway: Option<&'a RunwayEndSpec>,
    /// Truth-world dynamic 3D entities (per-slot visibility controlled by `ViewSlotMask`).
    pub entities: &'a [DynamicEntity3D],
}

/// Simulator-only `HeadlessMap` world serving OTW and synthetic optical sensors.
pub struct TruthSceneMap {
    runtime: SharedMapRuntime,
    map: HeadlessMap,
    otw_slot: ViewSlotId,
    otw_right_slot: Option<ViewSlotId>,
    sensor_slot: ViewSlotId,
    otw_size: [u32; 2],
    sensor_size: [u32; 2],
    request_overscan: f64,
    last_otw_stamp: Option<ConformalFrameStamp>,
    last_sensor_stamp: Option<ConformalFrameStamp>,
}

impl TruthSceneMap {
    /// Assembles a `TruthSceneMap` sharing `runtime`'s GPU, pipelines, and static tile cache.
    pub fn new(
        runtime: &SharedMapRuntime,
        style: Style,
        config: TruthSceneConfig,
    ) -> Result<Self, MapAdapterError> {
        let [otw_w, otw_h] = config.otw_size;
        let [sen_w, sen_h] = config.sensor_size;
        let otw_phys = validate_size(otw_w, otw_h)?;
        let sen_phys = validate_size(sen_w, sen_h)?;

        let settings = runtime
            .device_tier()
            .truth_scene_settings(runtime.color_format());
        // Minimal 16x16 default head; actual rendering targets the registered ViewSlots.
        let (kernel, renderer) = runtime.create_renderer(16, 16, settings)?;
        let plugins: Vec<Box<dyn Plugin<HeadlessEnvironment>>> = vec![
            Box::new(RenderPlugin),
            Box::new(StarfieldPlugin),
            Box::new(BackgroundPlugin),
            Box::new(RasterPlugin::<DefaultRasterTransferables>::default()),
            Box::new(VectorPlugin::<DefaultVectorTransferables>::default()),
            Box::new(TerrainPlugin::<DefaultDemTransferables>::default()),
            Box::new(DrapeBudgetPlugin(runtime.device_tier().otw_drape_budget())),
            Box::new(OverlayPlugin),
        ];
        let mut map = runtime
            .with_runtime_context(|| HeadlessMap::new(style, renderer, kernel, plugins))
            .map_err(|source| MapAdapterError::Assemble { source })?;
        map.set_available_memory(runtime.device_tier().available_memory_bytes());
        map.set_atmosphere(true);

        let otw_slot = map.register_view_slot(ViewSlotConfig {
            size: otw_phys,
            depth_readout: config.otw_depth_readout,
        });
        let otw_right_slot = config.stereo_otw.then(|| {
            map.register_view_slot(ViewSlotConfig {
                size: otw_phys,
                depth_readout: false,
            })
        });
        let sensor_slot = map.register_view_slot(ViewSlotConfig {
            size: sen_phys,
            depth_readout: true,
        });

        Ok(Self {
            runtime: runtime.clone(),
            map,
            otw_slot,
            otw_right_slot,
            sensor_slot,
            otw_size: config.otw_size,
            sensor_size: config.sensor_size,
            request_overscan: config.request_overscan.max(1.0),
            last_otw_stamp: None,
            last_sensor_stamp: None,
        })
    }

    /// Synchronously populates resident DEM tiles from `scenery` for Frame-1 determinism.
    pub fn supply_dem_synchronously(
        &mut self,
        scenery: &CatalogScenery,
    ) -> Result<usize, MapAdapterError> {
        self.runtime
            .with_runtime_context(|| supply_catalog_dem_synchronously(&mut self.map, scenery))
    }

    /// Updates the ECEF solar direction driving globe daylight, twilight, and atmosphere.
    pub fn set_sun_direction_ecef(&mut self, sun_ecef: DVec3) -> Result<(), MapAdapterError> {
        let dir = EcefDirection::new(sun_ecef.x, sun_ecef.y, sun_ecef.z)
            .map_err(|source| MapAdapterError::Celestial { source })?;
        self.map.set_sun_direction(Some(dir));
        Ok(())
    }

    /// Updates the solar twilight elevation band and night-side floor brightness.
    pub fn set_twilight_band(&mut self, band: TwilightBand) {
        self.map.set_twilight_band(band);
    }

    /// Enables or disables atmospheric limb scattering behind the globe.
    pub fn set_atmosphere(&mut self, enabled: bool) {
        self.map.set_atmosphere(enabled);
    }

    /// Renders active truth viewports in one [`MultiViewFrame`].
    pub fn render_frame(&mut self, frame: TruthSceneFrame<'_>) -> Result<(), MapAdapterError> {
        self.apply_slot_overlay(self.otw_slot, self.otw_size, &frame.otw_stamp, &frame, true)?;

        let mut slots = Vec::with_capacity(3);
        slots.push(ViewSlotEntry {
            slot: self.otw_slot,
            view: SlotView::External(frame.otw_stamp.external_view),
            policy: CoveringPolicy::Independent,
        });
        if let (Some(right_slot), Some(right_stamp)) =
            (self.otw_right_slot, frame.stereo_right_stamp)
        {
            self.apply_slot_overlay(right_slot, self.otw_size, &right_stamp, &frame, false)?;
            slots.push(ViewSlotEntry {
                slot: right_slot,
                view: SlotView::External(right_stamp.external_view),
                policy: CoveringPolicy::StereoReuse(self.otw_slot),
            });
        }
        if let Some(sensor_stamp) = frame.sensor_stamp {
            self.apply_slot_overlay(
                self.sensor_slot,
                self.sensor_size,
                &sensor_stamp,
                &frame,
                true,
            )?;
            slots.push(ViewSlotEntry {
                slot: self.sensor_slot,
                view: SlotView::External(sensor_stamp.external_view),
                policy: CoveringPolicy::Independent,
            });
            self.last_sensor_stamp = Some(sensor_stamp);
        }

        let mv = MultiViewFrame {
            timestamp: frame.timestamp,
            opaque_environment: true,
            request_overscan: self.request_overscan,
            slots,
        };
        self.runtime
            .with_runtime_context(|| self.map.run_multi_view_frame(mv))
            .map_err(|source| MapAdapterError::MultiView { source })?;
        self.last_otw_stamp = Some(frame.otw_stamp);
        Ok(())
    }

    fn apply_slot_overlay(
        &mut self,
        slot: ViewSlotId,
        viewport_px: [u32; 2],
        stamp: &ConformalFrameStamp,
        frame: &TruthSceneFrame<'_>,
        submit_probes: bool,
    ) -> Result<(), MapAdapterError> {
        let (mesh, probes) =
            build_conformal_overlay(stamp, viewport_px, frame.runway, frame.entities, Some(slot));
        self.map
            .set_view_slot_overlay(slot, mesh)
            .map_err(|source| MapAdapterError::MultiView { source })?;
        if submit_probes && !probes.is_empty() {
            let _ = self
                .map
                .set_view_slot_depth_probes(slot, probes)
                .map_err(|source| MapAdapterError::MultiView { source })?;
        }
        Ok(())
    }

    /// Primary cockpit OTW [`ViewSlotId`].
    #[must_use]
    pub const fn otw_slot(&self) -> ViewSlotId {
        self.otw_slot
    }

    /// Optical / EVS sensor [`ViewSlotId`].
    #[must_use]
    pub const fn sensor_slot(&self) -> ViewSlotId {
        self.sensor_slot
    }

    /// Zero-copy [`wgpu::TextureView`] of the cockpit OTW color attachment.
    #[must_use]
    pub fn otw_color_view(&self) -> Option<&wgpu::TextureView> {
        self.map.view_slot_color_view(self.otw_slot)
    }

    /// Zero-copy [`wgpu::Texture`] of the cockpit OTW color attachment.
    #[must_use]
    pub fn otw_color_texture(&self) -> Option<&wgpu::Texture> {
        self.map.view_slot_color_texture(self.otw_slot)
    }

    /// Zero-copy [`wgpu::TextureView`] of the cockpit OTW reversed-Z `Depth32Float` attachment.
    #[must_use]
    pub fn otw_depth_view(&self) -> Option<&wgpu::TextureView> {
        self.map.view_slot_depth_view(self.otw_slot)
    }

    /// Zero-copy [`wgpu::TextureView`] of the optical sensor color attachment.
    #[must_use]
    pub fn sensor_color_view(&self) -> Option<&wgpu::TextureView> {
        self.map.view_slot_color_view(self.sensor_slot)
    }

    /// Zero-copy [`wgpu::Texture`] of the optical sensor color attachment.
    #[must_use]
    pub fn sensor_color_texture(&self) -> Option<&wgpu::Texture> {
        self.map.view_slot_color_texture(self.sensor_slot)
    }

    /// Zero-copy [`wgpu::TextureView`] of the sensor reversed-Z `Depth32Float` attachment.
    #[must_use]
    pub fn sensor_depth_view(&self) -> Option<&wgpu::TextureView> {
        self.map.view_slot_depth_view(self.sensor_slot)
    }

    /// Zero-copy [`wgpu::Texture`] of the sensor reversed-Z `Depth32Float` attachment.
    #[must_use]
    pub fn sensor_depth_texture(&self) -> Option<&wgpu::Texture> {
        self.map.view_slot_depth_texture(self.sensor_slot)
    }

    /// Whether the OTW slot has allocated a CPU `MAP_READ` staging buffer (`false` in zero-copy operation).
    #[must_use]
    pub fn otw_has_readback_buffer(&self) -> Option<bool> {
        self.map.view_slot_has_readback_buffer(self.otw_slot)
    }

    /// Whether the sensor slot has allocated a CPU `MAP_READ` staging buffer (`false` unless CPU readback is requested).
    #[must_use]
    pub fn sensor_has_readback_buffer(&self) -> Option<bool> {
        self.map.view_slot_has_readback_buffer(self.sensor_slot)
    }

    /// Latest [`ConformalFrameStamp`] rendered on the OTW slot.
    #[must_use]
    pub const fn last_otw_stamp(&self) -> Option<ConformalFrameStamp> {
        self.last_otw_stamp
    }

    /// Latest [`ConformalFrameStamp`] rendered on the sensor slot.
    #[must_use]
    pub const fn last_sensor_stamp(&self) -> Option<ConformalFrameStamp> {
        self.last_sensor_stamp
    }

    /// Takes the latest completed [`ProbeResults`] for the OTW slot, if ready.
    pub fn take_otw_probe_results(&mut self) -> Result<Option<ProbeResults>, MapAdapterError> {
        self.map
            .take_view_slot_depth_probe_results(self.otw_slot)
            .map_err(|source| MapAdapterError::MultiView { source })
    }

    /// Takes the latest completed [`ProbeResults`] for the sensor slot, if ready.
    pub fn take_sensor_probe_results(&mut self) -> Result<Option<ProbeResults>, MapAdapterError> {
        self.map
            .take_view_slot_depth_probe_results(self.sensor_slot)
            .map_err(|source| MapAdapterError::MultiView { source })
    }
}
