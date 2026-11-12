//! The dock's panels drawn apart from the window view, each on a surface of
//! its own, so a page shows the whole HUD unobstructed beside the PFD and HSI.

use sim_geo::BodyEllipsoid;
use sim_render::{OverlayQuad, ViewportRect};
use sim_scenario::{OperatorViewportMode, ScenarioWorld};

use super::{CockpitRig, MFD_CHART_MAP, PFD_SVS_MAP, layout};

impl CockpitRig {
    /// Draws the PFD and the HSI apart from the window view, each `panel_px` in size, or
    /// back in the dock over it for `None`.
    pub fn set_panels_apart(&mut self, panel_px: Option<[u32; 2]>) {
        self.panels_apart = panel_px;
    }

    /// The quads of the PFD and of the HSI drawn apart, each filling its own surface; `None`
    /// while they are docked or hidden.
    #[must_use]
    pub fn apart_panel_quads(&self, world: ScenarioWorld) -> Option<[Vec<OverlayQuad<'_>>; 2]> {
        self.panels_apart?;
        if !self.show_instruments || !self.sources.avionics {
            return None;
        }
        let svs = self
            .maps
            .as_ref()
            .filter(|_| world.body() == BodyEllipsoid::WGS84)
            .and_then(|maps| maps.instrument_view(PFD_SVS_MAP));
        let mut pfd = Vec::with_capacity(2);
        if let Some(svs) = svs {
            pfd.push(OverlayQuad::inset_map(ViewportRect::FULL, svs, false));
        }
        pfd.push(OverlayQuad::vector_panel(
            ViewportRect::FULL,
            self.avionics.pfd_view(),
            svs.is_none(),
            true,
        ));
        let hsi = vec![OverlayQuad::vector_panel(
            ViewportRect::FULL,
            self.avionics.hsi_view(),
            true,
            true,
        )];
        Some([pfd, hsi])
    }

    /// The pixel size the PFD and HSI are drawn at: their own surfaces' when apart, else a
    /// docked panel's.
    pub(super) fn dock_panel_px(
        &self,
        earth: bool,
        mode: OperatorViewportMode,
    ) -> Option<[u32; 2]> {
        if !self.show_instruments || !self.sources.avionics {
            return None;
        }
        if let Some(apart) = self.panels_apart {
            return Some(apart);
        }
        let has_dock_chart = earth
            && mode != OperatorViewportMode::ChartMap
            && self
                .maps
                .as_ref()
                .is_some_and(|m| !m.efb_shown() && m.instrument_view(MFD_CHART_MAP).is_some());
        layout::dock_panel_px(self.surface_size, has_dock_chart)
    }
}
