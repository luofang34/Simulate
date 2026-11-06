use sim_core::{SensorExposureEvent, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_maplibre::{DynamicEntity3D, HostGpu, ViewSlotMask, default_render_body};
use sim_render::{Camera, OverlayQuad, Renderer, VectorScenePass, ViewportRect};
use sim_scenario::{OperatorViewportMode, ScenarioWorld};
use tracing::warn;

use crate::avionics::{AvionicsFrameInput, AvionicsRig, ConformalSurface, HudView};
use crate::error::AppSimError;
use crate::flight::Flight;
use crate::host_guidance::StandInPilotageHost;
use crate::map_rig::{
    AIRPORT_SURFACE_MAP, AvionicsPreset, EfbAirport, EfbCommand, MFD_CHART_MAP, MapFrameInput,
    MapLibreRig, PFD_SVS_MAP, ScenerySource,
};
use crate::sky_rig::SkyFrame;
use crate::supervisor_rig::SupervisorRig;
use crate::world_surface::WorldSurfaceHandle;
use efb_chrome::{EfbChrome, efb_map_rect, rect_px};
use sim_maplibre::EfbDisplayMode;

pub mod efb_chrome;
mod layout;
#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests;
#[cfg(all(test, not(target_arch = "wasm32")))]
mod verify_shots;

/// Composite split-viewport cockpit orchestrating [`SupervisorRig`], [`MapLibreRig`],
/// [`AvionicsRig`], and the stand-in guidance host on one shared `wgpu::Device`.
pub struct CockpitRig {
    supervisor: SupervisorRig,
    host: StandInPilotageHost,
    maps: Option<MapLibreRig>,
    avionics: AvionicsRig,
    device: wgpu::Device,
    surface_size: [u32; 2],
    pixel_ratio: f64,
    show_instruments: bool,
    show_conformal_hud: bool,
    surface: WorldSurfaceHandle,
}

impl CockpitRig {
    /// Builds the composite cockpit sharing `renderer`'s GPU device, queue, adapter, and
    /// instance, streaming terrain into the flight's `surface`.
    ///
    /// Enables the `maplibre-rs` scene and instrument maps (real-world tiles) on `Metal`,
    /// `Vulkan`, `Dx12`, and `BrowserWebGpu`, falling back cleanly to `GlobeScene` + 2D
    /// `Indicate` panels on `WebGL2` (`wgpu::Backend::Gl`).
    pub fn from_renderer(
        renderer: &Renderer,
        hud_pass: VectorScenePass,
        surface: WorldSurfaceHandle,
    ) -> Result<Self, AppSimError> {
        let enable_maps = renderer.backend() != wgpu::Backend::Gl;
        let host_gpu = HostGpu {
            instance: renderer.instance().clone(),
            adapter: renderer.adapter().clone(),
            device: renderer.device().clone(),
            queue: renderer.queue().clone(),
        };
        Self::new_with_gpu(
            host_gpu,
            renderer.surface_size(),
            enable_maps.then(ScenerySource::from_environment),
            hud_pass,
            surface,
        )
    }

    /// Builds a composite cockpit directly from `host_gpu` (used by both the windowed app and
    /// headless GPU integration tests); `scenery` of `None` runs without maps, and the HUD
    /// draws through `hud_pass`. The map rig configures and feeds `surface`.
    pub fn new_with_gpu(
        host_gpu: HostGpu,
        initial_size: [u32; 2],
        scenery: Option<ScenerySource>,
        hud_pass: VectorScenePass,
        surface: WorldSurfaceHandle,
    ) -> Result<Self, AppSimError> {
        let avionics = AvionicsRig::new(&host_gpu.device, hud_pass)?;
        let device = host_gpu.device.clone();
        let maps = scenery.and_then(|source| {
            MapLibreRig::new(
                host_gpu,
                initial_size,
                &source,
                AvionicsPreset::default(),
                surface.clone(),
            )
            .inspect_err(|err| {
                warn!(error = %err, "MapLibre maps unavailable; using GlobeScene fallback");
            })
            .ok()
        });
        let mut supervisor = SupervisorRig::new();
        supervisor.set_vertical_datum(surface.vertical_datum());
        Ok(Self {
            supervisor,
            host: StandInPilotageHost::new(),
            maps,
            avionics,
            device,
            surface_size: initial_size,
            pixel_ratio: 1.0,
            show_instruments: true,
            show_conformal_hud: true,
            surface,
        })
    }

    /// Borrows [`SupervisorRig`].
    #[must_use]
    pub const fn supervisor(&self) -> &SupervisorRig {
        &self.supervisor
    }

    /// Mutably borrows [`SupervisorRig`].
    pub fn supervisor_mut(&mut self) -> &mut SupervisorRig {
        &mut self.supervisor
    }

    /// Borrows [`MapLibreRig`], when active on the current backend.
    #[must_use]
    pub const fn maps(&self) -> Option<&MapLibreRig> {
        self.maps.as_ref()
    }

    /// Mutably borrows [`MapLibreRig`], when active on the current backend.
    pub fn maps_mut(&mut self) -> Option<&mut MapLibreRig> {
        self.maps.as_mut()
    }

    /// The EFB's chrome as shown over `world` this frame, when the EFB is shown: on
    /// Earth, with the dock up, outside the full-window chart, on a map preset.
    #[must_use]
    pub fn efb_chrome(&self, world: ScenarioWorld) -> Option<EfbChrome> {
        let shown = world.body() == BodyEllipsoid::WGS84
            && self.show_instruments
            && self.supervisor.viewport_mode() != OperatorViewportMode::ChartMap;
        let maps = self.maps.as_ref().filter(|_| shown)?;
        if !maps.efb_ready() {
            return None;
        }
        Some(EfbChrome::new(self.surface_size, &maps.efb_status()?))
    }

    /// Applies an operator command to the EFB.
    pub fn efb_command(&mut self, command: EfbCommand) -> Result<(), AppSimError> {
        match self.maps.as_mut() {
            Some(maps) => maps.efb_command(command),
            None => Ok(()),
        }
    }

    /// Shows `airport` on the EFB (`None` follows the supervisor's aerodrome).
    pub fn set_efb_airport(&mut self, airport: Option<EfbAirport>) {
        if let Some(maps) = self.maps.as_mut() {
            maps.set_efb_airport(airport);
        }
    }

    /// Borrows [`AvionicsRig`].
    #[cfg(test)]
    #[must_use]
    pub const fn avionics(&self) -> &AvionicsRig {
        &self.avionics
    }

    /// The stand-in guidance host's rewind/reset generation.
    #[must_use]
    pub const fn guidance_epoch(&self) -> u32 {
        self.host.source_epoch()
    }

    /// Draws every map at the window's `scale_factor` physical pixels per logical pixel.
    pub fn set_pixel_ratio(&mut self, scale_factor: f64) {
        self.pixel_ratio = scale_factor;
        if let Some(maps) = self.maps.as_mut() {
            maps.set_pixel_ratio(scale_factor);
        }
    }

    /// Toggles the bottom glass-cockpit PFD/MFD/HSI dock.
    pub fn toggle_instruments(&mut self) -> bool {
        self.show_instruments = !self.show_instruments;
        self.show_instruments
    }

    /// Rebuilds the physical scene and instrument maps on `host_gpu` with tiles from
    /// `source` (`None` runs without maps), keeping the avionics preset, pixel ratio, and
    /// the EFB's chosen airport.
    pub fn rebuild_maps(&mut self, host_gpu: HostGpu, source: Option<&ScenerySource>) {
        let preset = self
            .maps
            .as_ref()
            .map_or_else(AvionicsPreset::default, MapLibreRig::preset);
        let efb_airport = self.maps.as_ref().and_then(MapLibreRig::efb_airport);
        self.maps = None;
        self.maps = source.and_then(|source| {
            MapLibreRig::new(
                host_gpu,
                self.surface_size,
                source,
                preset,
                self.surface.clone(),
            )
            .inspect_err(|err| {
                warn!(error = %err, "MapLibre maps unavailable; using GlobeScene fallback");
            })
            .ok()
        });
        // Without a rig nothing streams live tiles, so the ground falls back offline.
        if self.maps.is_none()
            && let Err(error) = self.surface.configure(&ScenerySource::Catalog)
        {
            warn!(%error, "offline ground unavailable; keeping the previous surface");
        }
        if let Some(maps) = self.maps.as_mut() {
            maps.set_efb_airport(efb_airport);
        }
        self.supervisor
            .set_vertical_datum(self.surface.vertical_datum());
        let ratio = self.pixel_ratio;
        self.set_pixel_ratio(ratio);
    }

    /// Whether the bottom glass-cockpit PFD/MFD/HSI dock is shown.
    #[must_use]
    pub const fn shows_instruments(&self) -> bool {
        self.show_instruments
    }

    /// Explicitly shows or hides the bottom glass-cockpit PFD/MFD/HSI dock.
    pub fn set_show_instruments(&mut self, visible: bool) {
        self.show_instruments = visible;
    }

    /// Switches to the next avionics preset, rebuilding its instrument maps.
    pub fn cycle_avionics_preset(&mut self) -> Result<Option<AvionicsPreset>, AppSimError> {
        let Some(maps) = self.maps.as_mut() else {
            return Ok(None);
        };
        let next = maps.preset().next();
        maps.set_preset(next)?;
        Ok(Some(next))
    }

    /// Selects `preset`, rebuilding its instrument maps.
    pub fn set_avionics_preset(&mut self, preset: AvionicsPreset) -> Result<(), AppSimError> {
        if let Some(maps) = self.maps.as_mut() {
            maps.set_preset(preset)?;
        }
        Ok(())
    }

    /// Toggles the full-window conformal HUD overlay in `OperatorViewportMode::Cockpit`.
    pub fn toggle_conformal_hud(&mut self) -> bool {
        self.show_conformal_hud = !self.show_conformal_hud;
        self.show_conformal_hud
    }

    /// Explicitly shows or hides the center conformal HUD overlay.
    pub fn set_show_conformal_hud(&mut self, visible: bool) {
        self.show_conformal_hud = visible;
    }

    /// Marks the MFD `Map2D` chart dirty for an immediate redraw on scenario/airport switch.
    pub fn invalidate_chart(&mut self) {
        if let Some(maps) = self.maps.as_mut() {
            maps.invalidate_charts();
        }
    }

    /// Whether background `maplibre` tile workers have new arrivals waiting to be drawn.
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn maps_need_redraw(&self) -> bool {
        self.maps.as_ref().is_some_and(MapLibreRig::needs_redraw)
    }

    /// Steps supervisor radio belief, renders the physical scene and instrument maps when on
    /// Earth, publishes the guidance host's report, updates `Indicate` PFD/HSI/HUD, and returns
    /// whether the caller should run the fallback 3D `GlobeScene` pass.
    ///
    /// The physical world (the window scene and `camera`) is drawn on truth,
    /// [`Flight::display_vehicle_state`], since light reaches the windshield from where the
    /// aircraft really is; the `800×600` sensor slot renders the newest of
    /// `sensor_exposures` (drained from [`Flight::drain_sensor_exposures`]) at its exact
    /// instant and truth state. Everything onboard (guidance host, PFD/HSI, SVS, charts, and
    /// the conformal HUD's symbology) reads [`Flight::onboard_sensors`] instead, so a
    /// navigation error shows as HUD and SVS symbology misregistered against the window.
    pub fn prepare_frame(
        &mut self,
        queue: &wgpu::Queue,
        surface_size: [u32; 2],
        flight: &Flight,
        camera: &Camera,
        sky: &SkyFrame<'_>,
        sensor_exposures: &[SensorExposureEvent],
    ) -> Result<bool, AppSimError> {
        let now = flight.now();
        let truth_vehicle = flight.display_vehicle_state();
        let sensed = flight.onboard_sensors();
        let estimated = sensed.estimated_vehicle;
        let profile = flight.profile();
        self.surface_size = surface_size;
        self.supervisor
            .sync(truth_vehicle.position_ecef, now, profile);

        let body = flight.world().body();
        let earth = body == BodyEllipsoid::WGS84;
        let viewport_mode = self.supervisor.viewport_mode();
        let mut terrain_patch = None;
        let mut svs_underlay_active = false;
        let mut map_sphere_active = false;
        if earth && let Some(maps) = self.maps.as_mut() {
            map_sphere_active = true;
            let expanded = efb_map_rect(surface_size, EfbDisplayMode::ExpandedEfb);
            maps.set_efb_expanded_px(rect_px(expanded, surface_size))?;
            maps.render_earth_frame(&MapFrameInput {
                surface_size,
                timestamp: now,
                profile,
                active_vehicle: flight.free_vehicle(),
                vehicle: truth_vehicle,
                estimated_vehicle: estimated,
                sensor_exposures,
                articulation: flight.display_articulation(),
                otw_camera: *camera,
                sky,
                supervisor: &self.supervisor,
            })?;
            svs_underlay_active = maps.instrument_view(PFD_SVS_MAP).is_some();
            if viewport_mode == OperatorViewportMode::VolumetricScene {
                terrain_patch = maps.sample_terrain_patch();
            }
        }

        let show_hud = self.show_conformal_hud && viewport_mode == OperatorViewportMode::Cockpit;
        let traffic = self.supervisor.belief_entities(ViewSlotMask::ALL);
        let runway = self.active_onboard_runway(earth, profile);
        let hud = show_hud
            .then(|| self.hud_view(body, map_sphere_active, camera, &truth_vehicle, &traffic));
        let guidance = self
            .host
            .publish(flight, runway.as_ref(), layout::heading_rad(&estimated));
        let _monitored = self.avionics.update(
            &self.device,
            queue,
            &AvionicsFrameInput {
                profile,
                timestamp: now,
                vehicle: estimated,
                cockpit: sensed.cockpit,
                air_data: sensed.air_data,
                specific_force_frd_m_s2: sensed.specific_force_frd_m_s2,
                svs_underlay_active,
                hud,
                runway: runway.as_ref(),
                guidance: &guidance,
                viewport_mode,
                terrain_patch: terrain_patch.as_ref(),
            },
        )?;

        let full_map_covered = earth && self.full_window_map_view().is_some();
        Ok(!full_map_covered)
    }

    fn active_onboard_runway(
        &self,
        earth: bool,
        profile: Option<sim_scenario::ScenarioProfile>,
    ) -> Option<sim_scenery::LandingRunwayGeometry> {
        let datum = self.supervisor.vertical_datum();
        earth
            .then(|| self.supervisor.active_landing_runway(profile))
            .flatten()
            .map(|rwy| match self.maps.as_ref() {
                Some(maps) => {
                    rwy.with_onboard_dem(datum, |lat, lon| maps.sample_svs_dem_msl(lat, lon))
                }
                None => rwy,
            })
    }

    fn hud_view<'a>(
        &self,
        body: BodyEllipsoid,
        map_sphere_active: bool,
        camera: &Camera,
        truth_vehicle: &VehicleState,
        traffic: &'a [DynamicEntity3D],
    ) -> HudView<'a> {
        let eye_body_lever_m = truth_vehicle.attitude.inverse()
            * (camera.pose.position_ecef - truth_vehicle.position_ecef);
        let conformal_surface = if map_sphere_active {
            ConformalSurface::GlobeSphere {
                physics: body,
                render_body: default_render_body(body),
            }
        } else {
            ConformalSurface::Ellipsoid { physics: body }
        };
        HudView {
            surface_size: self.surface_size,
            camera: *camera,
            traffic,
            airframe_attitude: truth_vehicle.attitude,
            eye_body_lever_m,
            conformal_surface,
            vertical_datum: (body == BodyEllipsoid::WGS84)
                .then(|| self.supervisor.vertical_datum()),
        }
    }

    /// Assembles the ordered zero-copy [`OverlayQuad`] stack for [`Renderer::render_composite`].
    #[must_use]
    pub fn overlay_quads(&self, world: ScenarioWorld) -> Vec<OverlayQuad<'_>> {
        let mut quads = Vec::with_capacity(6);
        let earth = world.body() == BodyEllipsoid::WGS84;
        let mode = self.supervisor.viewport_mode();
        if earth && let Some(full_view) = self.full_window_map_view() {
            quads.push(OverlayQuad::full_map(full_view));
        }
        if self.show_conformal_hud
            && mode == OperatorViewportMode::Cockpit
            && let Some(hud) = self.avionics.hud_view()
        {
            quads.push(OverlayQuad::vector_overlay(ViewportRect::FULL, hud));
        }
        if !self.show_instruments {
            return quads;
        }
        let maps = self.maps.as_ref().filter(|_| earth);
        let svs_view = maps.and_then(|m| m.instrument_view(PFD_SVS_MAP));
        let efb = self.efb_chrome(world).and_then(|chrome| {
            let view = maps.and_then(MapLibreRig::efb_view)?;
            Some(OverlayQuad::inset_map(chrome.map_viewport(), view, true))
        });
        // Without an EFB, a map preset keeps its moving map in the middle of the dock.
        let chart_view = maps
            .filter(|m| !m.efb_shown() && mode != OperatorViewportMode::ChartMap)
            .and_then(|m| m.instrument_view(MFD_CHART_MAP));
        let rects = layout::dock_rects(self.surface_size, if chart_view.is_some() { 3 } else { 2 });
        let (Some(&pfd_rect), Some(&hsi_rect)) = (rects.first(), rects.last()) else {
            return quads;
        };
        if let Some(svs) = svs_view {
            quads.push(OverlayQuad::inset_map(pfd_rect, svs, false));
        }
        quads.push(OverlayQuad::panel(
            pfd_rect,
            self.avionics.pfd_texture().view(),
            svs_view.is_none(),
            true,
        ));
        if let (Some(chart), Some(&mfd_rect)) = (chart_view, rects.get(1)) {
            quads.push(OverlayQuad::inset_map(mfd_rect, chart, true));
        }
        quads.push(OverlayQuad::panel(
            hsi_rect,
            self.avionics.hsi_texture().view(),
            true,
            true,
        ));
        // Last, so the expanded tablet lies over the dock.
        quads.extend(efb);
        quads
    }

    fn full_window_map_view(&self) -> Option<&wgpu::TextureView> {
        let maps = self.maps.as_ref()?;
        match self.supervisor.viewport_mode() {
            OperatorViewportMode::Cockpit
            | OperatorViewportMode::Chase
            | OperatorViewportMode::VolumetricScene
            | OperatorViewportMode::SensorAperture => maps.truth().otw_color_view(),
            OperatorViewportMode::ChartMap => maps
                .instrument_view(AIRPORT_SURFACE_MAP)
                .or_else(|| maps.instrument_view(MFD_CHART_MAP)),
        }
    }
}
