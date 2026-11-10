//! Simulator-only `TruthSceneMap`: OTW cockpit camera, optional stereo companion, 30 Hz
//! optical/EVS sensor slot, and N dynamically registered virtual camera slots.
//!
//! Every slot shows only the physical scene: imagery on terrain and OpenUSD aircraft.
//! Symbology belongs to instrument maps, never to what a camera sees.

use std::{cell::RefCell, collections::BTreeMap, rc::Rc, sync::Arc, time::Duration};

use glam::DVec3;
use maplibre::{
    background::BackgroundPlugin,
    celestial::{EcefDirection, TwilightBand},
    headless::{
        CoveringPolicy, MultiViewFrame, SlotView, ViewSlotConfig, ViewSlotEntry, ViewSlotId,
        environment::HeadlessEnvironment, map::HeadlessMap,
    },
    overlay::OverlayPlugin,
    plugin::Plugin,
    raster::{DefaultRasterTransferables, RasterPlugin},
    render::RenderPlugin,
    starfield::StarfieldPlugin,
    style::Style,
    terrain::{TerrainPlugin, transferables::DefaultDemTransferables},
    vector::{DefaultVectorTransferables, VectorPlugin},
};
use sim_scenery::CatalogScenery;

use super::{
    DrapeBudgetPlugin, EARTH_POLAR_ELEVATIONS, PolarElevationsPlugin, SharedMapRuntime,
    pixel_ratio::pixel_ratio_update, validate_size,
};
pub use crate::usd_pass::AircraftInstance;
use crate::{
    ConformalFrameStamp, DynamicEntity3D, MapAdapterError, SlotCameraRole, UsdAircraftAsset,
    entity_depth_probes, supply_catalog_dem_synchronously,
    usd_pass::{
        AircraftLibrary, AircraftSlotView, PolarSkiwayKey, SharedAircraftLibrary, SkyLight,
        SlotDraws, SlotScene, UsdScene3dPlugin, ownship_shadow, pack_slot_draws, sun_in_anchor_enu,
    },
};

mod celestial;
mod ground;
mod ownship;
mod slots;
#[cfg(test)]
mod tests;
mod views;

pub use celestial::{
    CelestialDisc, CelestialEarthShadow, CelestialSkyInput, CelestialStar, CelestialTwilight,
};
pub use slots::TruthSlotMask;

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
            request_overscan: 1.22,
        }
    }
}

/// Per-frame inputs driving [`TruthSceneMap::render_frame`].
#[derive(Clone, Copy, Debug)]
pub struct TruthSceneFrame<'a> {
    /// Simulation frame timestamp for tile motion prefetch and animated paint properties.
    pub timestamp: Duration,
    /// Primary cockpit OTW camera stamp; when `None`, the OTW and stereo slots are skipped.
    pub otw_stamp: Option<ConformalFrameStamp>,
    /// Optional right-eye stereo OTW camera stamp (when `stereo_otw` is enabled).
    pub stereo_right_stamp: Option<ConformalFrameStamp>,
    /// Optional 30 Hz optical / EVS sensor camera stamp; when `None`, the sensor slot is
    /// skipped this frame (`0.0 ms` GPU) without evicting its tiles or drapes.
    pub sensor_stamp: Option<ConformalFrameStamp>,
    /// Additional virtual camera slots (`TowerPtz`, `ThresholdFinal`, wing/chase, multi-camera
    /// array) active on this frame. Registered virtual camera slots omitted here incur `0.0 ms`
    /// GPU cost without evicting resident tiles.
    pub extra_camera_slots: &'a [(ViewSlotId, ConformalFrameStamp)],
    /// Truth-world dynamic entities, answered by line-of-sight depth probes only.
    pub entities: &'a [DynamicEntity3D],
    /// OpenUSD aircraft drawn in 3D (ownship and traffic).
    pub aircraft: &'a [AircraftInstance],
    /// Which ownship parts each slot sees; slots not listed see the whole airframe.
    pub ownship_camera_roles: &'a [(ViewSlotId, SlotCameraRole)],
    /// Optional polar runway (`|lat| > 85.05°`) rendered as a 3D groomed snow skiway.
    pub polar_runway: Option<&'a sim_scenery::LandingRunwayGeometry>,
}

/// Simulator-only `HeadlessMap` world serving OTW, synthetic optical sensors, and virtual cameras.
pub struct TruthSceneMap {
    runtime: SharedMapRuntime,
    map: HeadlessMap,
    otw_slot: ViewSlotId,
    otw_right_slot: Option<ViewSlotId>,
    sensor_slot: ViewSlotId,
    otw_size: [u32; 2],
    sensor_size: [u32; 2],
    extra_slots: BTreeMap<ViewSlotId, [u32; 2]>,
    request_overscan: f64,
    last_otw_stamp: Option<ConformalFrameStamp>,
    last_sensor_stamp: Option<ConformalFrameStamp>,
    aircraft_library: SharedAircraftLibrary,
    aircraft_draws: SlotDraws,
    sun_ecef: DVec3,
    pixel_ratio: Option<f64>,
}

impl TruthSceneMap {
    /// Assembles a `TruthSceneMap` sharing `runtime`'s GPU, pipelines, and static tile cache.
    /// `aircraft_asset` draws every traffic aircraft and ownship while
    /// [`Self::ownship_visual`] is [`crate::OwnshipVisualKind::C172`].
    pub fn new(
        runtime: &SharedMapRuntime,
        style: Style,
        config: TruthSceneConfig,
        aircraft_asset: Arc<UsdAircraftAsset>,
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
        let aircraft_draws: SlotDraws = Rc::new(RefCell::new(BTreeMap::new()));
        let aircraft_library = Rc::new(RefCell::new(AircraftLibrary::new(aircraft_asset)));
        let plugins: Vec<Box<dyn Plugin<HeadlessEnvironment>>> = vec![
            Box::new(RenderPlugin),
            Box::new(StarfieldPlugin),
            Box::new(BackgroundPlugin),
            Box::new(RasterPlugin::<DefaultRasterTransferables>::default()),
            Box::new(VectorPlugin::<DefaultVectorTransferables>::default()),
            Box::new(TerrainPlugin::<DefaultDemTransferables>::default()),
            Box::new(PolarElevationsPlugin(EARTH_POLAR_ELEVATIONS)),
            Box::new(DrapeBudgetPlugin(runtime.device_tier().otw_drape_budget())),
            Box::new(OverlayPlugin),
            Box::new(UsdScene3dPlugin {
                library: Rc::clone(&aircraft_library),
                draws: Rc::clone(&aircraft_draws),
            }),
        ];
        let mut map = runtime
            .with_runtime_context(|| HeadlessMap::new(style, renderer, kernel, plugins))
            .map_err(|source| MapAdapterError::Assemble { source })?;
        map.set_available_memory(runtime.device_tier().available_memory_bytes());
        map.set_atmosphere(true);
        runtime
            .with_runtime_context(|| crate::scenery_loader::seed_sea_level_root_dem(&mut map))?;

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
            extra_slots: BTreeMap::new(),
            request_overscan: config.request_overscan.max(1.0),
            last_otw_stamp: None,
            last_sensor_stamp: None,
            aircraft_library,
            aircraft_draws,
            sun_ecef: DVec3::Z,
            pixel_ratio: None,
        })
    }

    /// Registers an additional virtual camera [`ViewSlotId`] sharing the `TruthScene` tile world.
    pub fn register_virtual_camera(
        &mut self,
        width: u32,
        height: u32,
        depth_readout: bool,
    ) -> Result<ViewSlotId, MapAdapterError> {
        let phys = validate_size(width, height)?;
        let slot = self.map.register_view_slot(ViewSlotConfig {
            size: phys,
            depth_readout,
        });
        self.extra_slots.insert(slot, [width, height]);
        Ok(slot)
    }

    /// Resizes a registered virtual camera [`ViewSlotId`]; zero dimensions are ignored.
    pub fn resize_virtual_camera(
        &mut self,
        slot: ViewSlotId,
        width: u32,
        height: u32,
    ) -> Result<(), MapAdapterError> {
        if width == 0
            || height == 0
            || self.extra_slots.get(&slot).copied() == Some([width, height])
        {
            return Ok(());
        }
        let phys = validate_size(width, height)?;
        self.map
            .resize_view_slot(slot, phys)
            .map_err(|source| MapAdapterError::MultiView { source })?;
        if let Some(dim) = self.extra_slots.get_mut(&slot) {
            *dim = [width, height];
        }
        Ok(())
    }

    /// Unregisters a virtual camera [`ViewSlotId`] and releases its GPU attachments.
    pub fn remove_virtual_camera(&mut self, slot: ViewSlotId) -> bool {
        self.extra_slots.remove(&slot);
        self.map.remove_view_slot(slot)
    }

    /// Physical `[width, height]` pixel dimensions of a registered virtual camera slot.
    #[must_use]
    pub fn virtual_camera_size(&self, slot: ViewSlotId) -> Option<[u32; 2]> {
        self.extra_slots.get(&slot).copied()
    }

    /// Synchronously populates resident DEM tiles from `scenery` for Frame-1 determinism.
    pub fn supply_dem_synchronously(
        &mut self,
        scenery: &CatalogScenery,
    ) -> Result<usize, MapAdapterError> {
        self.runtime
            .with_runtime_context(|| supply_catalog_dem_synchronously(&mut self.map, scenery))
    }

    /// Synchronously populates resident orthoimagery tiles from `scenery` for Frame-1 determinism.
    pub fn supply_ortho_synchronously(
        &mut self,
        scenery: &CatalogScenery,
    ) -> Result<usize, MapAdapterError> {
        self.runtime.with_runtime_context(|| {
            crate::supply_catalog_ortho_synchronously(&mut self.map, scenery, "ortho")
        })
    }

    /// Updates the ECEF solar direction driving globe daylight, twilight, and atmosphere.
    pub fn set_sun_direction_ecef(&mut self, sun_ecef: DVec3) -> Result<(), MapAdapterError> {
        let dir = EcefDirection::new(sun_ecef.x, sun_ecef.y, sun_ecef.z)
            .map_err(|source| MapAdapterError::Celestial { source })?;
        self.map.set_sun_direction(Some(dir));
        self.sun_ecef = sun_ecef;
        Ok(())
    }

    /// Updates the solar twilight elevation band and night-side floor brightness.
    pub fn set_twilight_band(&mut self, band: TwilightBand) {
        self.map.set_twilight_band(band);
    }

    /// Draws every slot at `ratio` physical pixels per logical pixel (Retina `2.0`).
    /// Non-finite ratios are ignored, others clamped to `0.5..=4.0`, and unchanged ratios
    /// not reapplied.
    pub fn set_pixel_ratio(&mut self, ratio: f64) {
        if let Some(applied) = pixel_ratio_update(self.pixel_ratio, ratio) {
            self.map.set_pixel_ratio(applied);
            self.pixel_ratio = Some(applied);
        }
    }

    /// Enables or disables atmospheric limb scattering behind the globe.
    pub fn set_atmosphere(&mut self, enabled: bool) {
        self.map.set_atmosphere(enabled);
    }

    /// Physical `[width, height]` pixel dimensions of the primary OTW slot.
    #[must_use]
    pub const fn otw_size(&self) -> [u32; 2] {
        self.otw_size
    }

    /// Physical `[width, height]` pixel dimensions of the offscreen sensor slot.
    #[must_use]
    pub const fn sensor_size(&self) -> [u32; 2] {
        self.sensor_size
    }

    /// Resizes the primary OTW slot (and stereo companion slot when enabled).
    pub fn resize_otw(&mut self, width: u32, height: u32) -> Result<(), MapAdapterError> {
        if width == 0 || height == 0 || self.otw_size == [width, height] {
            return Ok(());
        }
        let phys = validate_size(width, height)?;
        self.map
            .resize_view_slot(self.otw_slot, phys)
            .map_err(|source| MapAdapterError::MultiView { source })?;
        if let Some(right_slot) = self.otw_right_slot {
            self.map
                .resize_view_slot(right_slot, phys)
                .map_err(|source| MapAdapterError::MultiView { source })?;
        }
        self.otw_size = [width, height];
        Ok(())
    }

    /// Renders every active truth viewport ([`TruthSlotMask::BOTH`]) in one [`MultiViewFrame`].
    pub fn render_frame(&mut self, frame: TruthSceneFrame<'_>) -> Result<(), MapAdapterError> {
        self.render_frame_slots(frame, TruthSlotMask::BOTH)
    }

    /// Renders the built-in slots `mask` allows (and every listed extra camera slot) in one
    /// [`MultiViewFrame`]. Masked-out slots are not stepped, so their tiles stay resident.
    pub fn render_frame_slots(
        &mut self,
        frame: TruthSceneFrame<'_>,
        mask: TruthSlotMask,
    ) -> Result<(), MapAdapterError> {
        let plan = slots::plan_slots(
            mask,
            frame.otw_stamp.is_some(),
            self.otw_right_slot.is_some() && frame.stereo_right_stamp.is_some(),
            frame.sensor_stamp.is_some(),
        );
        let views = self.planned_views(&frame, plan);
        if views.is_empty() {
            return Ok(());
        }
        let mut slots = Vec::with_capacity(views.len());
        let mut draws = BTreeMap::new();
        for (slot, size, stamp, policy) in views {
            self.submit_probes(slot, size, &stamp, &frame)?;
            draws.insert(slot, self.slot_aircraft_draws(slot, &stamp, &frame));
            slots.push(ViewSlotEntry {
                slot,
                view: SlotView::External(stamp.external_view),
                policy,
            });
        }
        if let Ok(mut shared) = self.aircraft_draws.try_borrow_mut() {
            *shared = draws;
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
        if plan.otw {
            self.last_otw_stamp = frame.otw_stamp;
        }
        if plan.sensor {
            self.last_sensor_stamp = frame.sensor_stamp;
        }
        Ok(())
    }

    fn planned_views(
        &self,
        frame: &TruthSceneFrame<'_>,
        plan: slots::SlotPlan,
    ) -> Vec<(ViewSlotId, [u32; 2], ConformalFrameStamp, CoveringPolicy)> {
        let mut views = Vec::with_capacity(3_usize.saturating_add(frame.extra_camera_slots.len()));
        if let Some(otw_stamp) = frame.otw_stamp.filter(|_| plan.otw) {
            views.push((
                self.otw_slot,
                self.otw_size,
                otw_stamp,
                CoveringPolicy::Independent,
            ));
        }
        if let (Some(right_slot), Some(right_stamp), true) =
            (self.otw_right_slot, frame.stereo_right_stamp, plan.stereo)
        {
            views.push((
                right_slot,
                self.otw_size,
                right_stamp,
                CoveringPolicy::StereoReuse(self.otw_slot),
            ));
        }
        if let Some(sensor_stamp) = frame.sensor_stamp.filter(|_| plan.sensor) {
            views.push((
                self.sensor_slot,
                self.sensor_size,
                sensor_stamp,
                CoveringPolicy::Independent,
            ));
        }
        for &(slot, stamp) in frame.extra_camera_slots {
            let size = self
                .extra_slots
                .get(&slot)
                .copied()
                .unwrap_or(self.sensor_size);
            views.push((slot, size, stamp, CoveringPolicy::Independent));
        }
        views
    }

    fn slot_aircraft_draws(
        &self,
        slot: ViewSlotId,
        stamp: &ConformalFrameStamp,
        frame: &TruthSceneFrame<'_>,
    ) -> SlotScene {
        let ownship_role = frame
            .ownship_camera_roles
            .iter()
            .find(|(role_slot, _)| *role_slot == slot)
            .map_or(SlotCameraRole::External, |(_, role)| *role);
        let (sun_enu, daylight) = sun_in_anchor_enu(stamp, self.sun_ecef);
        let Ok(mut library) = self.aircraft_library.try_borrow_mut() else {
            return SlotScene {
                draws: Vec::new(),
                shadow: None,
                sky: SkyLight::new(sun_enu, daylight),
            };
        };
        let polar_skiway = frame
            .polar_runway
            .filter(|r| r.threshold.lat_deg().abs() > 85.05)
            .map(|r| {
                (
                    library.ensure_polar_skiway(PolarSkiwayKey::from_runway(r, stamp.render_body)),
                    r,
                )
            });
        let view = AircraftSlotView {
            stamp,
            sun_enu,
            daylight,
            ownship_role,
            slot,
        };
        SlotScene {
            draws: pack_slot_draws(&library, frame.aircraft, polar_skiway, &view),
            shadow: ownship_shadow(&library, frame.aircraft, &view),
            sky: SkyLight::new(sun_enu, daylight),
        }
    }

    fn submit_probes(
        &mut self,
        slot: ViewSlotId,
        viewport_px: [u32; 2],
        stamp: &ConformalFrameStamp,
        frame: &TruthSceneFrame<'_>,
    ) -> Result<(), MapAdapterError> {
        let probes = entity_depth_probes(stamp, viewport_px, frame.entities, Some(slot));
        if !probes.is_empty() {
            let _request = self
                .map
                .set_view_slot_depth_probes(slot, probes)
                .map_err(|source| MapAdapterError::MultiView { source })?;
        }
        Ok(())
    }
}
