//! `maplibre-rs` maps of the composite cockpit: the physical scene every camera sees and
//! the zero or more instrument maps of the selected avionics preset, all on one
//! [`SharedMapRuntime`].

use std::sync::Arc;
use std::time::Duration;

use serde_json::Value;
use sim_core::SimInstant;
use sim_geo::{BodyEllipsoid, ecef_to_geodetic};
use sim_maplibre::{
    AvionicsMapSet, ChartAssetStore, ConformalFrameStamp, ConformalMapFrame, HostGpu,
    InstrumentMapSurface, OwnshipVisualKind, SceneryEndpoints, SharedMapRuntime,
    TerrainPatchSnapshot, TruthSceneConfig, TruthSceneFrame, TruthSceneMap, TruthSlotMask,
    UsdAircraftAsset, ViewSlotId, ViewSlotMask, tile_id_to_world_coords, truth_real_world_style,
};
use sim_scenario::OperatorViewportMode;
use sim_scenery::{CatalogScenery, LandingRunwayGeometry, OsmAerodromeDiagram, TileId};

use crate::environment::TruthSky;
use crate::error::AppSimError;
use crate::world_surface::WorldSurfaceHandle;

pub(crate) mod aircraft;
mod charts;
mod device_tier;
mod efb;
mod frame_input;
mod globe;
mod live_ground;
mod preset;
mod scenery;
mod stamps;
#[cfg(test)]
mod tests;

pub use device_tier::TierChoice;
pub use efb::{EfbAirport, EfbCommand, EfbStatus};
pub use frame_input::MapFrameInput;
pub use globe::GlobeSelection;
pub use preset::{AIRPORT_SURFACE_MAP, AvionicsPreset, MFD_CHART_MAP, PFD_SVS_MAP};
pub use scenery::ScenerySource;

const SENSOR_WIDTH_PX: u32 = 800;
const SENSOR_HEIGHT_PX: u32 = 600;

/// The physical scene and the selected preset's instrument maps on one shared runtime.
pub struct MapLibreRig {
    runtime: SharedMapRuntime,
    endpoints: SceneryEndpoints,
    aerodrome_geojson: Value,
    catalog: Option<CatalogScenery>,
    live_ground: Option<live_ground::LiveGround>,
    truth: TruthSceneMap,
    avionics: AvionicsMapSet,
    preset: AvionicsPreset,
    window_px: [u32; 2],
    aux_virtual_camera_slot: Option<ViewSlotId>,
    last_ownship_tile: Option<TileId>,
    truth_shown: bool,
    pixel_ratio: f64,
    sensor_frames: u64,
    last_sensor_stamp: Option<ConformalFrameStamp>,
    surface: WorldSurfaceHandle,
    ownship_visual: OwnshipVisualKind,
    custom_livery_key: Option<(String, String, u32)>,
    charts: ChartAssetStore,
    efb: Option<efb::EfbRig>,
    efb_airport: Option<EfbAirport>,
    globe: Option<globe::GlobeRig>,
    tier: TierChoice,
}

impl MapLibreRig {
    /// Builds the physical scene and `preset`'s maps over `gpu`, with tiles from `source`,
    /// configuring `surface` to the same source and feeding it every streamed DEM tile.
    pub fn new(
        gpu: HostGpu,
        initial_size: [u32; 2],
        source: &ScenerySource,
        preset: AvionicsPreset,
        surface: WorldSurfaceHandle,
    ) -> Result<Self, AppSimError> {
        Self::new_with_aux_sensor(gpu, initial_size, source, preset, surface, true)
    }

    /// Lean constructor for a link's page view: skips the secondary `800×600` depth-readout
    /// virtual camera slot (the page renders its navigation camera on `CameraWorld`).
    pub fn new_for_page(
        gpu: HostGpu,
        initial_size: [u32; 2],
        source: &ScenerySource,
        preset: AvionicsPreset,
        surface: WorldSurfaceHandle,
    ) -> Result<Self, AppSimError> {
        Self::new_with_aux_sensor(gpu, initial_size, source, preset, surface, false)
    }

    fn new_with_aux_sensor(
        gpu: HostGpu,
        initial_size: [u32; 2],
        source: &ScenerySource,
        preset: AvionicsPreset,
        surface: WorldSurfaceHandle,
        register_aux_sensor: bool,
    ) -> Result<Self, AppSimError> {
        let aerodrome_geojson = OsmAerodromeDiagram::builtin_all_merged_geojson()
            .map_err(sim_scenario::ScenarioError::from)?;
        let charts = ChartAssetStore::new();
        let scenery::SceneryLoader {
            loader,
            endpoints,
            catalog,
            live_ground,
        } = source.loader(charts.clone(), surface.runway_datum())?;
        let tier = device_tier::choose_for(&gpu.adapter);
        let runtime = SharedMapRuntime::new(
            gpu,
            tier.tier,
            wgpu::TextureFormat::Rgba8Unorm,
            Some(loader),
        )?;
        let window_px = clamp_window(initial_size);
        // Traffic draws the generic C172; ownship selects its tail once the rig exists.
        let asset = Arc::new(UsdAircraftAsset::embedded_c172()?);
        let mut truth = TruthSceneMap::new(
            &runtime,
            truth_real_world_style(&endpoints)?,
            TruthSceneConfig {
                otw_size: window_px,
                stereo_otw: false,
                otw_depth_readout: false,
                sensor_size: [SENSOR_WIDTH_PX, SENSOR_HEIGHT_PX],
                request_overscan: 1.22,
            },
            asset,
        )?;
        let aux_virtual_camera_slot = register_aux_sensor
            .then(|| truth.register_virtual_camera(SENSOR_WIDTH_PX, SENSOR_HEIGHT_PX, true))
            .transpose()?;
        if let Some(scenery) = &catalog {
            let _supplied = truth.supply_dem_synchronously(scenery)?;
            let _supplied_ortho = truth.supply_ortho_synchronously(scenery)?;
        }
        let mut rig = Self {
            avionics: AvionicsMapSet::default(),
            runtime,
            endpoints,
            aerodrome_geojson,
            catalog,
            live_ground,
            truth,
            preset,
            window_px,
            aux_virtual_camera_slot,
            last_ownship_tile: None,
            truth_shown: true,
            pixel_ratio: 1.0,
            sensor_frames: 0,
            last_sensor_stamp: None,
            surface,
            ownship_visual: OwnshipVisualKind::default(),
            custom_livery_key: None,
            charts,
            efb: None,
            efb_airport: None,
            globe: None,
            tier,
        };
        rig.set_preset(preset)?;
        rig.truth.set_ownship_visual(rig.ownship_visual)?;
        rig.surface
            .configure(source)
            .map_err(sim_scenario::ScenarioError::from)?;
        Ok(rig)
    }

    /// Replaces the instrument maps with `preset`'s.
    pub fn set_preset(&mut self, preset: AvionicsPreset) -> Result<(), AppSimError> {
        let specs = preset.map_specs(&self.endpoints, &self.aerodrome_geojson, self.window_px)?;
        let mut avionics = AvionicsMapSet::new(&self.runtime, specs)?;
        if let Some(scenery) = &self.catalog {
            let _supplied = avionics.supply_dem_synchronously(scenery)?;
        }
        avionics.set_pixel_ratio(self.pixel_ratio);
        self.avionics = avionics;
        self.preset = preset;
        self.ensure_efb();
        Ok(())
    }

    /// Draws every map at `ratio` physical pixels per logical pixel (the window's scale
    /// factor, `2.0` on Retina), including maps a later preset switch builds.
    pub fn set_pixel_ratio(&mut self, ratio: f64) {
        if !(ratio.is_finite() && ratio > 0.0) {
            return;
        }
        self.pixel_ratio = ratio;
        self.truth.set_pixel_ratio(ratio);
        self.avionics.set_pixel_ratio(ratio);
        if let Some(efb) = self.efb.as_mut() {
            efb.set_pixel_ratio(ratio);
        }
    }

    /// Selected avionics preset.
    #[must_use]
    pub const fn preset(&self) -> AvionicsPreset {
        self.preset
    }

    /// The simulator's physical scene.
    #[must_use]
    pub const fn truth(&self) -> &TruthSceneMap {
        &self.truth
    }

    /// Sets the exposure time of the camera the outside scene is drawn for.
    #[cfg(test)]
    pub fn set_camera_shutter_s(&mut self, shutter_s: f64) {
        self.truth.set_shutter_exposure_s(shutter_s);
    }

    /// The preset's instrument maps.
    #[cfg(test)]
    #[must_use]
    pub const fn avionics(&self) -> &AvionicsMapSet {
        &self.avionics
    }

    /// Colour view of the instrument map `id`, when the preset has it.
    #[must_use]
    pub fn instrument_view(&self, id: &str) -> Option<&wgpu::TextureView> {
        self.avionics
            .get(id)
            .and_then(InstrumentMapSurface::color_view)
    }

    /// Marks every chart dirty so the next frame redraws immediately.
    pub fn invalidate_charts(&mut self) {
        self.avionics.invalidate_charts();
    }

    /// Whether any shown map has tile arrivals or camera animations waiting to be drawn; the
    /// physical scene is left out while the window shows a chart, since nothing would draw its
    /// arrivals.
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn needs_redraw(&self) -> bool {
        (self.truth_shown && self.truth.needs_redraw())
            || self.avionics.needs_redraw()
            || self.efb_needs_redraw()
            || self.globe_needs_redraw()
    }

    /// Sensor-slot frames rendered, one per frame that carried at least one exposure.
    #[must_use]
    pub const fn sensor_frame_count(&self) -> u64 {
        self.sensor_frames
    }

    /// Stamp of the latest sensor-slot frame: the newest exposure's exact instant and
    /// truth-state camera pose.
    #[must_use]
    pub const fn last_sensor_stamp(&self) -> Option<ConformalFrameStamp> {
        self.last_sensor_stamp
    }

    /// Secondary registered virtual camera slot, ready for multi-camera sensor feeds.
    #[cfg(test)]
    #[must_use]
    pub const fn aux_virtual_camera_slot(&self) -> Option<ViewSlotId> {
        self.aux_virtual_camera_slot
    }

    /// Stitched `129×129` resident DEM patch under ownship from the first conformal map.
    #[must_use]
    pub fn sample_terrain_patch(&self) -> Option<TerrainPatchSnapshot> {
        let tile = self
            .catalog
            .as_ref()
            .and_then(|c| c.elevation_tiles().next())
            .or(self.last_ownship_tile)?;
        let coords = tile_id_to_world_coords(tile)?;
        self.avionics
            .iter()
            .filter(|s| s.is_conformal())
            .find_map(|s| s.resident_terrain_patch(coords))
    }

    /// Steps the physical scene (window + 30 Hz sensor) and every instrument map, returning
    /// the frame's DEM-refined active landing runway so onboard HUD/guidance share one query.
    pub fn render_earth_frame(
        &mut self,
        input: &MapFrameInput<'_>,
    ) -> Result<Option<LandingRunwayGeometry>, AppSimError> {
        let _ = self.aux_virtual_camera_slot;
        if let Some(ground) = self.live_ground.as_mut() {
            let ownship = ecef_to_geodetic(input.vehicle.position_ecef);
            ground.sync(&self.surface, &self.truth, &self.runtime, ownship);
        }
        self.window_px = clamp_window(input.surface_size);
        let [w, h] = self.window_px;
        self.truth.resize_otw(w, h)?;
        if let Some(surface) = self.avionics.get_mut(AIRPORT_SURFACE_MAP) {
            surface.resize(w, h)?;
        }
        TruthSky::of(input.sky).apply(&mut self.truth)?;
        self.avionics
            .set_sun_direction_ecef(input.sky.environment.sun.direction_fixed)?;
        let sim_dur = Duration::from_secs_f64(
            input
                .timestamp
                .duration_since(SimInstant::START)
                .as_seconds()
                .max(0.0),
        );
        self.step_truth_world(input, sim_dur)?;
        let landing_runway = self.active_landing_runway(input);
        self.step_instrument_maps(input, sim_dur, landing_runway.as_ref())?;
        self.step_efb(input, sim_dur, landing_runway.as_ref())?;
        Ok(landing_runway)
    }

    /// Geoid undulation at `position_ecef` in the datum of this rig's scenery.
    fn undulation_at(&self, position_ecef: glam::DVec3) -> f64 {
        let geo = ecef_to_geodetic(position_ecef);
        stamps::geoid_undulation_m(
            self.catalog.as_ref().map(CatalogScenery::vertical_datum),
            geo.lat_rad,
            geo.lon_rad,
        )
    }

    /// Shows the ownship airframe and per-tail livery that match what is flying.
    fn sync_ownship_visual(&mut self, input: &MapFrameInput<'_>) -> Result<(), AppSimError> {
        let kind = aircraft::ownship_visual(input.profile, input.active_vehicle);
        if kind != self.ownship_visual {
            self.truth.set_ownship_visual(kind)?;
            self.ownship_visual = kind;
            self.custom_livery_key = None;
        }
        match input.ownship_livery_override {
            Some(((aircraft_id, livery_id, generation), asset)) => {
                let matches_cached = self.custom_livery_key.as_ref().is_some_and(|(a, l, g)| {
                    a == aircraft_id && l == livery_id && *g == generation
                });
                if !matches_cached {
                    self.truth.set_ownship_livery_asset(Arc::clone(asset))?;
                    self.custom_livery_key =
                        Some((aircraft_id.to_owned(), livery_id.to_owned(), generation));
                }
            }
            None => {
                if self.custom_livery_key.take().is_some() {
                    self.truth.set_ownship_visual(kind)?;
                }
            }
        }
        Ok(())
    }

    fn step_truth_world(
        &mut self,
        input: &MapFrameInput<'_>,
        sim_dur: Duration,
    ) -> Result<(), AppSimError> {
        // Every window view (cockpit, chase, volumetric, and the sensor aperture's virtual
        // camera) draws on the OTW slot at window resolution. The sensor slot steps only on
        // frames that carry an exposure; maplibre's slot residency floor keeps whichever
        // slot sits a frame out from losing its tiles and drapes.
        let mode = input.supervisor.viewport_mode();
        let exposure = input.sensor_exposures.last();
        let mask = TruthSlotMask {
            otw: mode != OperatorViewportMode::ChartMap,
            sensor: exposure.is_some(),
        };
        self.truth_shown = mask.otw;
        if !mask.otw && !mask.sensor {
            return Ok(());
        }
        self.sync_ownship_visual(input)?;
        let undulation_m = self.undulation_at(input.vehicle.position_ecef);
        let (cam_body, cam_spec) = input
            .supervisor
            .active_virtual_camera_on_body(input.profile);
        let otw_stamp = mask
            .otw
            .then(|| {
                stamps::build_camera_stamp(
                    input.timestamp,
                    &input.otw_camera,
                    self.window_px,
                    undulation_m,
                )
            })
            .transpose()?;
        let sensor_stamp = exposure
            .map(|exp| stamps::sensor_exposure_stamp(cam_body, &cam_spec, exp, undulation_m))
            .transpose()?;
        let truth_entities = input.supervisor.truth_entities(ViewSlotMask::ALL);
        let mut aircraft = aircraft::traffic_aircraft(&truth_entities, sim_dur.as_secs_f64());
        aircraft.push(aircraft::ownship_of(input, undulation_m));
        let sensor_role = aircraft::virtual_camera_role(&cam_spec);
        let window_role = if mode == OperatorViewportMode::SensorAperture {
            sensor_role
        } else {
            aircraft::window_camera_role(mode, self.ownship_visual)
        };
        let polar_runway = input.supervisor.active_landing_runway(input.profile);
        self.truth.render_frame_slots(
            TruthSceneFrame {
                timestamp: sim_dur,
                otw_stamp,
                stereo_right_stamp: None,
                sensor_stamp,
                extra_camera_slots: &[],
                entities: &truth_entities,
                aircraft: &aircraft,
                ownship_camera_roles: &[
                    (self.truth.otw_slot(), window_role),
                    (self.truth.sensor_slot(), sensor_role),
                ],
                polar_runway: polar_runway.as_ref(),
            },
            mask,
        )?;
        if sensor_stamp.is_some() {
            self.sensor_frames = self.sensor_frames.wrapping_add(1);
            self.last_sensor_stamp = sensor_stamp;
        }
        Ok(())
    }

    fn step_instrument_maps(
        &mut self,
        input: &MapFrameInput<'_>,
        sim_dur: Duration,
        landing_runway: Option<&LandingRunwayGeometry>,
    ) -> Result<(), AppSimError> {
        let mode = input.supervisor.viewport_mode();
        let body = input
            .profile
            .map_or(BodyEllipsoid::WGS84, |p| p.world().body());
        let own_geo = body.fixed_to_geodetic(input.estimated_vehicle.position_ecef);
        self.last_ownship_tile = Some(scenery::tile_at(&own_geo, 11));
        // The EFB replaces the dock's moving map; the map still draws as the full-window
        // chart of a preset without an airport-surface map.
        let chart_fallback = mode == OperatorViewportMode::ChartMap
            && self.avionics.get(AIRPORT_SURFACE_MAP).is_none();
        let skip_dock_chart = self.efb_shown() && !chart_fallback;
        let any_due = self.avionics.iter().any(|s| {
            s.is_due(sim_dur)
                && !(skip_dock_chart && s.id() == MFD_CHART_MAP)
                && !(s.id() == AIRPORT_SURFACE_MAP && mode != OperatorViewportMode::ChartMap)
        });
        if !any_due {
            return Ok(());
        }
        let undulation_m = if body == BodyEllipsoid::WGS84 {
            self.undulation_at(input.estimated_vehicle.position_ecef)
        } else {
            0.0
        };
        let belief_entities = input.supervisor.belief_entities(ViewSlotMask::ALL);
        let own = charts::map_ownship(&input.estimated_vehicle, input.steered);
        for surface in self.avionics.iter_mut() {
            if !surface.is_due(sim_dur) || (skip_dock_chart && surface.id() == MFD_CHART_MAP) {
                continue;
            }
            if surface.is_conformal() {
                let stamp = stamps::svs_stamp(
                    input.timestamp,
                    body,
                    input.active_vehicle,
                    &input.estimated_vehicle,
                    surface.size(),
                    undulation_m,
                )?;
                surface.render_conformal(ConformalMapFrame {
                    timestamp: sim_dur,
                    stamp,
                    ownship_msl_m: (own_geo.height_m - undulation_m) as f32,
                    runway: landing_runway,
                    entities: &belief_entities,
                })?;
                continue;
            }
            charts::render_chart_surface(
                surface,
                input,
                sim_dur,
                own,
                landing_runway,
                &belief_entities,
            )?;
        }
        Ok(())
    }
}

fn clamp_window([w, h]: [u32; 2]) -> [u32; 2] {
    [w.max(16), h.max(16)]
}

impl MapLibreRig {
    /// The tier the maps run at, and why.
    #[must_use]
    pub const fn tier(&self) -> &TierChoice {
        &self.tier
    }
}
