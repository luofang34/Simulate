//! Direct swapchain avionics layering and cockpit view modes.

use sim_geo::BodyEllipsoid;
use sim_render::{DirectVectorDraw, OverlayQuad, ViewportRect};
use sim_scenario::{OperatorViewportMode, ScenarioWorld};

use super::layout;
use super::safe_area::SafeAreaInsets;
use super::{CockpitRig, MFD_CHART_MAP, PFD_SVS_MAP};
use crate::map_rig::MapLibreRig;

/// Linear premultiplied panel backdrop matching `CompositorPass` offscreen vector panels.
const DIRECT_PANEL_BACKDROP: [f32; 4] = [0.025, 0.035, 0.05, 0.88];

/// Preset combinations of primary cockpit overlays (conformal HUD vs docked `PFD`+`HSI`).
#[cfg_attr(
    not(target_arch = "wasm32"),
    allow(dead_code, reason = "used by wasm32 touch controls and tests")
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CockpitViewMode {
    /// Both the conformal HUD and the `PFD`+`HSI` EFIS stack are visible.
    HudAndEfis,
    /// Clean windshield with only the `PFD`+`HSI` EFIS stack visible.
    EfisOnly,
    /// Full-window conformal HUD only (EFIS panels hidden).
    HudOnly,
    /// Bare outside/cabin view with neither HUD nor EFIS panels.
    Clean,
}

#[cfg_attr(
    not(target_arch = "wasm32"),
    allow(dead_code, reason = "used by wasm32 touch controls and tests")
)]
impl CockpitViewMode {
    /// All modes in cycling / menu order.
    #[cfg(any(target_arch = "wasm32", test))]
    pub const ALL: [Self; 4] = [Self::HudAndEfis, Self::EfisOnly, Self::HudOnly, Self::Clean];

    /// Stable identifier used in `localStorage` and touch button attributes.
    #[cfg(any(target_arch = "wasm32", test))]
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::HudAndEfis => "hud_efis",
            Self::EfisOnly => "efis",
            Self::HudOnly => "hud",
            Self::Clean => "clean",
        }
    }

    /// Parses a persisted or DOM view-mode string.
    #[cfg(any(target_arch = "wasm32", test))]
    #[must_use]
    pub fn from_code(raw: &str) -> Option<Self> {
        match raw.trim() {
            "hud_efis" => Some(Self::HudAndEfis),
            "efis" => Some(Self::EfisOnly),
            "hud" => Some(Self::HudOnly),
            "clean" => Some(Self::Clean),
            _ => None,
        }
    }

    /// Concise button label shown in compact touch/hub controls.
    #[cfg(target_arch = "wasm32")]
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::HudAndEfis => "HUD+EFIS",
            Self::EfisOnly => "EFIS",
            Self::HudOnly => "HUD",
            Self::Clean => "CLEAN",
        }
    }

    /// Whether this mode shows the `PFD`+`HSI` dock.
    #[must_use]
    pub const fn shows_instruments(self) -> bool {
        matches!(self, Self::HudAndEfis | Self::EfisOnly)
    }

    /// Whether this mode shows the conformal HUD overlay.
    #[must_use]
    pub const fn shows_hud(self) -> bool {
        matches!(self, Self::HudAndEfis | Self::HudOnly)
    }

    /// The mode a handheld's column tab asks for: the `PFD`+`HSI` column alone when
    /// `head_down`, the HUD alone otherwise. The two show the same flight data, and a
    /// phone has room for one of them beside the thumb controls.
    #[must_use]
    pub const fn for_head_down(head_down: bool) -> Self {
        if head_down {
            Self::EfisOnly
        } else {
            Self::HudOnly
        }
    }
}

/// Layered draw lists handed to [`sim_render::Renderer::render_composite_layered`].
#[derive(Default)]
pub struct CockpitOverlayLayers<'a> {
    /// Background textures drawn before direct vectors (OTW, ChartMap, SVS underlay, offscreen panels).
    pub under_quads: Vec<OverlayQuad<'a>>,
    /// Direct-to-swapchain crisp vector overlays (HUD, PFD, HSI) drawn without intermediate blur.
    pub direct_vectors: Vec<DirectVectorDraw<'a>>,
    /// Top-layer UI/EFB quads drawn over direct vectors.
    pub over_quads: Vec<OverlayQuad<'a>>,
}

impl CockpitRig {
    /// Display-ready summary of dialed heading bug, altitude preselect, and baro QNH.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "used by wasm32 touch controls and tests")
    )]
    #[must_use]
    pub fn dialed_readout(
        &self,
        flight: &crate::flight::Flight,
    ) -> crate::host_guidance::dials::DialedAvionicsReadout {
        let est_hdg_deg = layout::heading_rad(&flight.estimated_vehicle_state()).to_degrees();
        let est_alt_m = estimated_datum_altitude_m(flight);
        self.host.dials().readout(est_hdg_deg, est_alt_m)
    }

    /// Steps the HSI heading bug by `delta_deg`, initializing from current heading if unset.
    pub fn adjust_heading_bug(&mut self, flight: &crate::flight::Flight, delta_deg: f64) -> f64 {
        let est = layout::heading_rad(&flight.estimated_vehicle_state()).to_degrees();
        self.host.dials_mut().step_heading_deg(delta_deg, est)
    }

    /// Syncs the HSI heading bug to current estimated magnetic/local heading.
    pub fn sync_heading_bug(&mut self, flight: &crate::flight::Flight) -> f64 {
        let est = layout::heading_rad(&flight.estimated_vehicle_state()).to_degrees();
        self.host.dials_mut().sync_heading_deg(est)
    }

    /// Steps the PFD/HUD altitude preselect bug by `delta_ft`.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "used by wasm32 touch controls and tests")
    )]
    pub fn adjust_altitude_bug(&mut self, flight: &crate::flight::Flight, delta_ft: f64) -> f64 {
        let est_alt = estimated_datum_altitude_m(flight);
        self.host.dials_mut().step_altitude_ft(delta_ft, est_alt)
    }

    /// Syncs the altitude preselect bug to current estimated datum altitude rounded to 100 ft.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "used by wasm32 touch controls and tests")
    )]
    pub fn sync_altitude_bug(&mut self, flight: &crate::flight::Flight) -> f64 {
        let est_alt = estimated_datum_altitude_m(flight);
        self.host.dials_mut().sync_altitude_m(est_alt)
    }

    /// Steps baro QNH on both the guidance host and sensor bus (`PFD`/`HUD` + 3D Kollsman).
    pub fn adjust_baro_hpa(&mut self, flight: &mut crate::flight::Flight, delta_hpa: f64) -> f64 {
        let next = self.host.dials_mut().step_baro_hpa(delta_hpa);
        flight.set_altimeter_setting_hpa(next);
        next
    }

    /// Resets baro QNH to standard `1013.25 hPa` on both guidance host and sensor bus.
    pub fn set_baro_std(&mut self, flight: &mut crate::flight::Flight) -> f64 {
        let std = self.host.dials_mut().set_baro_std();
        flight.set_altimeter_setting_hpa(std);
        std
    }

    /// Whether this frame draws PFD, HSI, and HUD directly onto the swapchain.
    #[must_use]
    pub const fn uses_direct_avionics(&self) -> bool {
        self.direct_avionics && self.panels_apart.is_none()
    }

    /// Combined visibility mode of the HUD and EFIS overlays.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "used by wasm32 touch controls and tests")
    )]
    #[must_use]
    pub const fn view_mode(&self) -> CockpitViewMode {
        match (self.show_conformal_hud, self.show_instruments) {
            (true, true) => CockpitViewMode::HudAndEfis,
            (false, true) => CockpitViewMode::EfisOnly,
            (true, false) => CockpitViewMode::HudOnly,
            (false, false) => CockpitViewMode::Clean,
        }
    }

    /// Applies `mode` to the conformal HUD and docked EFIS panels atomically.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "used by wasm32 touch controls and tests")
    )]
    pub fn set_view_mode(&mut self, mode: CockpitViewMode) {
        let was_open_efb = self
            .maps
            .as_ref()
            .is_some_and(|m| m.efb_shown() && m.efb_status().is_some_and(|s| !s.collapsed));
        self.show_conformal_hud = mode.shows_hud();
        self.show_instruments = mode.shows_instruments();
        self.allow_efb_solo = !self.show_instruments && was_open_efb;
    }

    /// Keeps the docked instruments and the corner map clear of the window edges the
    /// system covers.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "only a browser reports insets")
    )]
    pub const fn set_safe_area(&mut self, insets: SafeAreaInsets) {
        self.safe_area = insets;
    }

    /// Shows or hides 3D cockpit control geometry (yoke/stick/column) in interior cameras.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "used by wasm32 touch controls and tests")
    )]
    pub fn set_hide_cockpit_controls(&mut self, hide: bool) {
        if let Some(maps) = self.maps.as_mut() {
            maps.truth_mut().set_hide_cockpit_controls(hide);
        }
    }

    /// Whether 3D cockpit controls are currently hidden in interior cameras.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "used by wasm32 touch controls and tests")
    )]
    #[must_use]
    pub fn hides_cockpit_controls(&self) -> bool {
        self.maps
            .as_ref()
            .is_some_and(|m| m.truth().hides_cockpit_controls())
    }

    /// Assembles layered background quads, direct vector instruments, and top-layer EFB quads
    /// for [`sim_render::Renderer::render_composite_layered`].
    #[must_use]
    pub fn overlay_layers(&self, world: ScenarioWorld) -> CockpitOverlayLayers<'_> {
        let mut layers = CockpitOverlayLayers::default();
        let earth = world.body() == BodyEllipsoid::WGS84;
        let mode = self.supervisor.viewport_mode();
        if earth && let Some(full_view) = self.full_window_map_view() {
            layers.under_quads.push(OverlayQuad::full_map(full_view));
        }
        if self.show_conformal_hud && mode == OperatorViewportMode::Cockpit {
            if self.uses_direct_avionics() {
                if let Some(vertices) = self.avionics.direct_hud_vertices() {
                    let [w, h] = self.surface_size.map(|v| v.max(1));
                    layers.direct_vectors.push(DirectVectorDraw {
                        rect_px: [0, 0, w, h],
                        vertices,
                        opaque_backdrop: None,
                        bezel: false,
                    });
                }
            } else if let Some(hud) = self.avionics.hud_view() {
                layers
                    .under_quads
                    .push(OverlayQuad::vector_overlay(ViewportRect::FULL, hud));
            }
        }
        let maps = self.maps.as_ref().filter(|_| earth);
        let efb = self.efb_chrome(world).and_then(|chrome| {
            let view = maps.and_then(|m| self.inset_view(m))?;
            Some(OverlayQuad::inset_map(chrome.map_viewport(), view, true))
        });
        if self.show_instruments && self.sources.avionics && self.panels_apart.is_none() {
            self.push_instrument_layers(maps, mode, &mut layers);
        }
        layers.over_quads.extend(efb);
        layers
    }

    /// Flattened [`OverlayQuad`] stack for desktop/gallery callers without direct-vector pass support.
    #[cfg(any(not(target_arch = "wasm32"), test))]
    #[must_use]
    pub fn overlay_quads(&self, world: ScenarioWorld) -> Vec<OverlayQuad<'_>> {
        let mut layers = self.overlay_layers(world);
        layers.under_quads.extend(layers.over_quads);
        layers.under_quads
    }

    fn push_instrument_layers<'a>(
        &'a self,
        maps: Option<&'a MapLibreRig>,
        mode: OperatorViewportMode,
        layers: &mut CockpitOverlayLayers<'a>,
    ) {
        let svs_view = maps.and_then(|m| m.instrument_view(PFD_SVS_MAP));
        let chart_view = maps
            .filter(|m| !m.efb_shown() && mode != OperatorViewportMode::ChartMap)
            .and_then(|m| m.instrument_view(MFD_CHART_MAP));
        let dock = layout::dock_layout(
            self.surface_size,
            self.dock_style,
            chart_view.is_some(),
            self.safe_area,
        );
        if self.uses_direct_avionics() {
            if let Some(svs) = svs_view {
                layers
                    .under_quads
                    .push(OverlayQuad::inset_map(dock.pfd, svs, false));
            }
            if let (Some(chart), Some(mfd_rect)) = (chart_view, dock.mfd) {
                layers
                    .under_quads
                    .push(OverlayQuad::inset_map(mfd_rect, chart, true));
            }
            if let Some(vertices) = self.avionics.direct_pfd_vertices() {
                layers.direct_vectors.push(DirectVectorDraw {
                    rect_px: layout::viewport_to_px_rect(dock.pfd, self.surface_size),
                    vertices,
                    opaque_backdrop: svs_view.is_none().then_some(DIRECT_PANEL_BACKDROP),
                    bezel: true,
                });
            }
            if let Some(vertices) = self.avionics.direct_hsi_vertices() {
                layers.direct_vectors.push(DirectVectorDraw {
                    rect_px: layout::viewport_to_px_rect(dock.hsi, self.surface_size),
                    vertices,
                    opaque_backdrop: Some(DIRECT_PANEL_BACKDROP),
                    bezel: true,
                });
            }
        } else {
            layout::push_dock_quads(
                &mut layers.under_quads,
                &dock,
                svs_view,
                self.avionics.pfd_view(),
                chart_view,
                self.avionics.hsi_view(),
            );
        }
    }
}

#[cfg_attr(
    not(target_arch = "wasm32"),
    allow(dead_code, reason = "used by wasm32 touch controls and tests")
)]
fn estimated_datum_altitude_m(flight: &crate::flight::Flight) -> f64 {
    let body = flight.world().body();
    let geo = body.fixed_to_geodetic(flight.estimated_vehicle_state().position_ecef);
    let undulation_m = if body == BodyEllipsoid::WGS84 {
        flight
            .world_surface()
            .geoid_undulation_m(geo.lat_rad, geo.lon_rad)
    } else {
        0.0
    };
    geo.height_m - undulation_m
}

#[cfg(test)]
mod tests {
    use super::CockpitViewMode;

    #[test]
    fn view_mode_round_trips_storage_codes() {
        for mode in CockpitViewMode::ALL {
            assert_eq!(CockpitViewMode::from_code(mode.code()), Some(mode));
        }
        assert_eq!(CockpitViewMode::from_code("unknown"), None);
    }

    #[test]
    fn the_column_tab_swaps_the_instruments_for_the_hud() {
        let down = CockpitViewMode::for_head_down(true);
        assert!(down.shows_instruments() && !down.shows_hud());
        let up = CockpitViewMode::for_head_down(false);
        assert!(up.shows_hud() && !up.shows_instruments());
    }
}
