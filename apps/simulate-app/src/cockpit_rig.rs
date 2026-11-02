use glam::DVec3;
use sim_geo::BodyEllipsoid;
use sim_maplibre::HostGpu;
use sim_render::{Camera, OverlayQuad, Renderer, ViewportRect};
use sim_scenario::{OperatorViewportMode, ScenarioWorld};
use tracing::warn;

use crate::avionics::{AvionicsFrameInput, AvionicsRig};
use crate::error::AppSimError;
use crate::flight::Flight;
use crate::map_rig::{DualWorldMapRig, MapFrameInput};
use crate::supervisor_rig::SupervisorRig;

const THREE_PANEL_PFD_RECT: ViewportRect = ViewportRect {
    x: 0.02,
    y: 0.60,
    width: 0.30,
    height: 0.37,
};
const THREE_PANEL_MFD_RECT: ViewportRect = ViewportRect {
    x: 0.35,
    y: 0.60,
    width: 0.30,
    height: 0.37,
};
const THREE_PANEL_HSI_RECT: ViewportRect = ViewportRect {
    x: 0.68,
    y: 0.60,
    width: 0.30,
    height: 0.37,
};
const TWO_PANEL_PFD_RECT: ViewportRect = ViewportRect {
    x: 0.03,
    y: 0.60,
    width: 0.33,
    height: 0.37,
};
const TWO_PANEL_HSI_RECT: ViewportRect = ViewportRect {
    x: 0.64,
    y: 0.60,
    width: 0.33,
    height: 0.37,
};
const CONFORMAL_HUD_RECT: ViewportRect = ViewportRect {
    x: 0.30,
    y: 0.14,
    width: 0.40,
    height: 0.42,
};

/// Composite split-viewport cockpit orchestrating [`SupervisorRig`], [`DualWorldMapRig`],
/// and [`AvionicsRig`] on one shared `wgpu::Device`.
pub struct CockpitRig {
    supervisor: SupervisorRig,
    maps: Option<DualWorldMapRig>,
    avionics: AvionicsRig,
    show_instruments: bool,
    show_conformal_hud: bool,
}

impl CockpitRig {
    /// Builds the composite cockpit sharing `renderer`'s GPU device, queue, adapter, and instance.
    ///
    /// Automatically enables `SharedMapRuntime` (`TruthSceneMap` + `BeliefAvionicsMap`) on
    /// `Metal`, `Vulkan`, `Dx12`, and `BrowserWebGpu`, while falling back cleanly to
    /// `GlobeScene` + 2D `Indicate` panels on `WebGL2` (`wgpu::Backend::Gl`).
    pub fn from_renderer(renderer: &Renderer) -> Result<Self, AppSimError> {
        let enable_maps = renderer.backend() != wgpu::Backend::Gl;
        let host_gpu = HostGpu {
            instance: renderer.instance().clone(),
            adapter: renderer.adapter().clone(),
            device: renderer.device().clone(),
            queue: renderer.queue().clone(),
        };
        Self::new_with_gpu(host_gpu, renderer.surface_size(), enable_maps)
    }

    /// Builds a composite cockpit directly from `host_gpu` (used by both the windowed app and
    /// headless GPU integration tests).
    pub fn new_with_gpu(
        host_gpu: HostGpu,
        initial_size: [u32; 2],
        enable_maps: bool,
    ) -> Result<Self, AppSimError> {
        let avionics = AvionicsRig::new(&host_gpu.device)?;
        let maps = if enable_maps {
            match DualWorldMapRig::new(host_gpu, initial_size) {
                Ok(rig) => Some(rig),
                Err(err) => {
                    warn!(error = %err, "MapLibre dual-world underlay unavailable; using GlobeScene fallback");
                    None
                }
            }
        } else {
            None
        };
        Ok(Self {
            supervisor: SupervisorRig::new(),
            maps,
            avionics,
            show_instruments: true,
            show_conformal_hud: false,
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

    /// Borrows [`DualWorldMapRig`], when active on the current backend.
    #[cfg(test)]
    #[must_use]
    pub const fn maps(&self) -> Option<&DualWorldMapRig> {
        self.maps.as_ref()
    }

    /// Borrows [`AvionicsRig`].
    #[cfg(test)]
    #[must_use]
    pub const fn avionics(&self) -> &AvionicsRig {
        &self.avionics
    }

    /// Toggles the bottom glass-cockpit PFD/MFD/HSI dock.
    pub fn toggle_instruments(&mut self) -> bool {
        self.show_instruments = !self.show_instruments;
        self.show_instruments
    }

    /// Explicitly shows or hides the bottom glass-cockpit PFD/MFD/HSI dock.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn set_show_instruments(&mut self, visible: bool) {
        self.show_instruments = visible;
    }

    /// Toggles the center conformal HUD overlay in `OperatorViewportMode::Cockpit`.
    pub fn toggle_conformal_hud(&mut self) -> bool {
        self.show_conformal_hud = !self.show_conformal_hud;
        self.show_conformal_hud
    }

    /// Explicitly shows or hides the center conformal HUD overlay.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn set_show_conformal_hud(&mut self, visible: bool) {
        self.show_conformal_hud = visible;
    }

    /// Marks the MFD `Map2D` chart dirty for an immediate redraw on scenario/airport switch.
    pub fn invalidate_chart(&mut self) {
        if let Some(maps) = self.maps.as_mut() {
            maps.invalidate_chart();
        }
    }

    /// Whether background `maplibre` tile workers have new arrivals waiting to be drawn.
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn maps_need_redraw(&self) -> bool {
        self.maps
            .as_ref()
            .is_some_and(DualWorldMapRig::needs_redraw)
    }

    /// Steps supervisor radio belief, renders Earth `TruthSceneMap` + `BeliefAvionicsMap` viewports
    /// when applicable, updates `Indicate` PFD/HSI/HUD textures, and returns whether the caller
    /// should run the fallback 3D `GlobeScene` pass.
    pub fn prepare_frame(
        &mut self,
        queue: &wgpu::Queue,
        surface_size: [u32; 2],
        flight: &Flight,
        camera: &Camera,
        sun_direction_ecef: DVec3,
    ) -> Result<bool, AppSimError> {
        let now = flight.now();
        let vehicle = flight.vehicle_state();
        let profile = flight.profile();
        self.supervisor.sync(vehicle.position_ecef, now, profile);

        let earth = flight.world().body() == BodyEllipsoid::WGS84;
        let viewport_mode = self.supervisor.viewport_mode();
        let mut terrain_patch = None;
        let mut svs_underlay_active = false;

        if earth && let Some(maps) = self.maps.as_mut() {
            maps.render_earth_frame(&MapFrameInput {
                surface_size,
                timestamp: now,
                profile,
                vehicle,
                otw_camera: *camera,
                sun_direction_ecef,
                supervisor: &self.supervisor,
            })?;
            svs_underlay_active = maps.belief().svs_color_view().is_some();
            if viewport_mode == OperatorViewportMode::VolumetricScene {
                terrain_patch = maps.sample_belief_terrain_patch();
            }
        }

        let render_hud = self.show_conformal_hud && viewport_mode == OperatorViewportMode::Cockpit;
        let _monitored = self.avionics.update(
            queue,
            &AvionicsFrameInput {
                profile: flight.profile(),
                timestamp: now,
                vehicle,
                cockpit: flight.cockpit(),
                specific_force_frd_m_s2: flight.specific_force_frd_m_s2(),
                svs_underlay_active,
                render_hud,
                viewport_mode,
                terrain_patch: terrain_patch.as_ref(),
            },
        )?;

        let full_map_covered = earth && self.full_window_map_view().is_some();
        Ok(!full_map_covered)
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

        if self.show_conformal_hud && mode == OperatorViewportMode::Cockpit {
            let mut hud = OverlayQuad::panel(
                CONFORMAL_HUD_RECT,
                self.avionics.hud_texture().view(),
                false,
                false,
            );
            hud.opacity = 0.88;
            quads.push(hud);
        }

        if !self.show_instruments {
            return quads;
        }

        let svs_view = earth
            .then(|| self.maps.as_ref().and_then(|m| m.belief().svs_color_view()))
            .flatten();
        let mfd_chart_view = (earth && mode != OperatorViewportMode::ChartMap)
            .then(|| {
                self.maps
                    .as_ref()
                    .and_then(|m| m.belief().chart_color_view())
            })
            .flatten();

        if let Some(chart_view) = mfd_chart_view {
            if let Some(svs) = svs_view {
                quads.push(OverlayQuad::inset_map(THREE_PANEL_PFD_RECT, svs, false));
            }
            quads.push(OverlayQuad::panel(
                THREE_PANEL_PFD_RECT,
                self.avionics.pfd_texture().view(),
                svs_view.is_none(),
                true,
            ));
            quads.push(OverlayQuad::inset_map(
                THREE_PANEL_MFD_RECT,
                chart_view,
                true,
            ));
            quads.push(OverlayQuad::panel(
                THREE_PANEL_HSI_RECT,
                self.avionics.hsi_texture().view(),
                true,
                true,
            ));
        } else {
            if let Some(svs) = svs_view {
                quads.push(OverlayQuad::inset_map(TWO_PANEL_PFD_RECT, svs, false));
            }
            quads.push(OverlayQuad::panel(
                TWO_PANEL_PFD_RECT,
                self.avionics.pfd_texture().view(),
                svs_view.is_none(),
                true,
            ));
            quads.push(OverlayQuad::panel(
                TWO_PANEL_HSI_RECT,
                self.avionics.hsi_texture().view(),
                true,
                true,
            ));
        }
        quads
    }

    fn full_window_map_view(&self) -> Option<&wgpu::TextureView> {
        let maps = self.maps.as_ref()?;
        match self.supervisor.viewport_mode() {
            OperatorViewportMode::Cockpit
            | OperatorViewportMode::Chase
            | OperatorViewportMode::VolumetricScene => maps.truth().otw_color_view(),
            OperatorViewportMode::SensorAperture => maps.truth().sensor_color_view(),
            OperatorViewportMode::ChartMap => maps.belief().chart_color_view(),
        }
    }
}
