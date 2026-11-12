//! The EFB on a page that has a map canvas of its own: the expanded view is the page's
//! map, the compact view stays the world view's inset, and both draw every frame from
//! the one map. The page hands over the plan and the tracks it lays over it, and reads
//! back where the plan's fixes are and which one was picked.

use sim_maplibre::{EfbDisplayMode, EfbRefreshPolicy, leg_state};

use super::super::MapLibreRig;
use super::EfbRig;
use crate::error::AppSimError;
use crate::page_map::{FixLabel, PageMapInput, PageMapView, PickedFix};

/// The page map's first zoom: a route of a few tens of kilometres in view, where the
/// tablet's own starts closer in.
const PAGE_ZOOM: f64 = 9.5;

/// How near a fix a tap picks it, CSS pixels: a fingertip's reach.
#[cfg(any(target_arch = "wasm32", test))]
const PICK_RADIUS_CSS_PX: f32 = 16.0;

impl EfbRig {
    /// Picks the fix nearest `at_px` on the shown view, or none.
    #[cfg(any(target_arch = "wasm32", test))]
    pub(super) fn pick(&mut self, at_px: [f32; 2]) {
        let radius = PICK_RADIUS_CSS_PX * self.surface.pixel_ratio() as f32;
        self.picked = self.surface.pick_fix(self.mode(), at_px, radius);
    }

    /// The shown view's fix labels and the picked fix.
    fn page_view(&self) -> PageMapView {
        let plan = self.surface.flight_plan();
        let progress = self.surface.route_progress();
        let labels = self
            .surface
            .fix_positions(self.mode())
            .into_iter()
            .zip(&plan.fixes)
            .enumerate()
            .filter_map(|(index, (at, fix))| {
                Some(FixLabel {
                    index,
                    ident: fix.ident.clone(),
                    at_px: at?,
                    state: leg_state(index, progress),
                })
            })
            .collect();
        let picked = self.picked.and_then(|pick| {
            let fix = plan.fixes.get(pick.index)?;
            Some(PickedFix {
                index: pick.index,
                ident: fix.ident.clone(),
                at_deg: [fix.lat_deg, fix.lon_deg],
                altitude_m: fix.altitude_m,
                state: leg_state(pick.index, progress),
            })
        });
        PageMapView {
            live: self.view.is_some(),
            status: Some(self.status()),
            labels,
            picked,
        }
    }
}

impl MapLibreRig {
    /// Applies what the page handed over: its commands address the page's map, the
    /// expanded view. Each view draws, at most 30 times a second, only while it can be
    /// seen: the page's map while the page shows it, the inset while `window_shown`.
    pub fn take_page_map(
        &mut self,
        input: PageMapInput,
        window_shown: bool,
    ) -> Result<(), AppSimError> {
        let Some(efb) = self.efb.as_mut() else {
            return Ok(());
        };
        if !efb.surface.all_views_shown() {
            efb.pacer.set_policy(EfbRefreshPolicy::Coalesced);
            efb.command(super::EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb))?;
            let zoom = PAGE_ZOOM - efb.surface.zoom();
            efb.command(super::EfbCommand::ZoomBy(zoom))?;
        }
        let shown = [window_shown, input.visible];
        if efb.views_shown != Some(shown) {
            efb.surface.set_views_shown(shown);
            efb.views_shown = Some(shown);
            efb.dirty = true;
        }
        for command in input.commands {
            efb.command(command)?;
        }
        if let Some((plan, progress)) = input.route {
            efb.surface.set_route(&plan, progress)?;
            efb.dirty = true;
        }
        if let Some(tracks) = input.tracks {
            efb.surface.set_tracks(tracks.as_ref().clone())?;
            efb.dirty = true;
        }
        Ok(())
    }

    /// What the page's map shows, once there is an EFB.
    #[must_use]
    pub fn page_map_view(&self) -> PageMapView {
        self.efb
            .as_ref()
            .map_or_else(PageMapView::default, EfbRig::page_view)
    }

    /// How many frames the EFB has drawn.
    #[must_use]
    pub fn efb_frames(&self) -> u64 {
        self.efb.as_ref().map_or(0, |efb| efb.frames)
    }

    /// `mode`'s last frame, when the preset shows the EFB.
    #[must_use]
    pub fn efb_view_of(&self, mode: EfbDisplayMode) -> Option<&wgpu::TextureView> {
        self.efb
            .as_ref()
            .filter(|_| self.efb_shown())
            .and_then(|efb| efb.surface.view_of(mode))
    }
}
