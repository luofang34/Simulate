//! The EFB map shown on a page's own canvas: the same map as the cockpit's moving-map
//! inset, its expanded view sized to the canvas instead of the window's tablet, while the
//! compact view stays the world view's inset.

use sim_core::SensorExposureEvent;
use sim_maplibre::EfbDisplayMode;
use sim_render::{Camera, CanvasPresenter, FrameOutcome, OverlayQuad, RenderError};
use tracing::warn;

use super::CockpitRig;
use super::efb_chrome::{efb_map_rect, rect_px};
use crate::flight::Flight;
use crate::map_rig::{EfbStatus, MapFrameInput, MapLibreRig};
use crate::page_map::PageMapLink;
use crate::sky_rig::SkyFrame;
use crate::supervisor_rig::SupervisorRig;

/// What every map draws `flight` from this frame, seen by `camera` under `sky`.
pub(super) fn frame_input<'a>(
    supervisor: &'a SupervisorRig,
    surface_size: [u32; 2],
    flight: &Flight,
    (camera, sky): (&Camera, &'a SkyFrame<'a>),
    sensor_exposures: &'a [SensorExposureEvent],
) -> MapFrameInput<'a> {
    MapFrameInput {
        surface_size,
        timestamp: flight.now(),
        profile: flight.profile(),
        active_vehicle: flight.free_vehicle(),
        vehicle: flight.display_vehicle_state(),
        model_origin_from_cg_frd_m: flight.model_origin_from_cg_frd_m(),
        ground_m: flight.display_ground_height_m(),
        estimated_vehicle: flight.onboard_sensors().estimated_vehicle,
        sensor_exposures,
        articulation: flight.display_articulation(),
        otw_camera: *camera,
        sky,
        supervisor,
    }
}

impl CockpitRig {
    /// Sizes the expanded EFB view to a page's map canvas of `size_px`, or to the
    /// window's tablet again with `None`.
    pub fn set_page_map_px(&mut self, size_px: Option<[u32; 2]>) {
        self.page_map_px = size_px;
    }

    /// The expanded EFB view's size: the page's map canvas when there is one, else the
    /// tablet's rectangle in a window of `surface_size`.
    pub(super) fn efb_expanded_px(&self, surface_size: [u32; 2]) -> [u32; 2] {
        self.page_map_px.unwrap_or_else(|| {
            rect_px(
                efb_map_rect(surface_size, EfbDisplayMode::ExpandedEfb),
                surface_size,
            )
        })
    }

    /// The EFB's status as its window chrome draws it: beside a page's map the window
    /// shows only the compact inset, whichever view the page's commands address.
    pub(super) fn inset_status(&self, mut status: EfbStatus) -> EfbStatus {
        if self.page_map_px.is_some() {
            status.mode = EfbDisplayMode::CompactCorner;
            status.collapsed = false;
        }
        status
    }

    /// The frame the window's EFB chrome frames: beside a page's map the compact inset,
    /// else the shown view.
    pub(super) fn inset_view<'m>(&self, maps: &'m MapLibreRig) -> Option<&'m wgpu::TextureView> {
        if self.page_map_px.is_some() {
            maps.efb_view_of(EfbDisplayMode::CompactCorner)
        } else {
            maps.efb_view()
        }
    }

    /// Draws only the EFB, for the page's map while the window that shows everything
    /// else is out of sight.
    pub fn prepare_page_map_only(
        &mut self,
        surface_size: [u32; 2],
        flight: &Flight,
        (camera, sky): (&Camera, &SkyFrame<'_>),
    ) {
        let expanded_px = self.efb_expanded_px(surface_size);
        let Some(maps) = self.maps.as_mut() else {
            return;
        };
        let input = frame_input(&self.supervisor, surface_size, flight, (camera, sky), &[]);
        if let Err(err) = maps
            .set_efb_expanded_px(expanded_px)
            .and_then(|()| maps.render_efb_frame(&input))
        {
            warn!(error = %err, "the page map's frame failed");
        }
    }

    /// Applies what the page handed over on `link` before the frame draws.
    pub fn take_page_map(&mut self, link: &PageMapLink) {
        let input = link.take();
        if let Some(maps) = self.maps.as_mut()
            && let Err(err) = maps.take_page_map(input)
        {
            warn!(error = %err, "the page map refused the page's input");
        }
    }

    /// Shows the expanded EFB view on `presenter`, and tells the page on `link` what it
    /// shows.
    pub fn present_page_map(
        &mut self,
        presenter: &mut CanvasPresenter,
        queue: &wgpu::Queue,
        link: &PageMapLink,
    ) -> Result<FrameOutcome, RenderError> {
        let Some(maps) = self.maps.as_ref() else {
            return Ok(FrameOutcome::Skipped);
        };
        let Some(view) = maps.efb_view_of(EfbDisplayMode::ExpandedEfb) else {
            return Ok(FrameOutcome::Skipped);
        };
        let outcome = presenter.present(&self.device, queue, OverlayQuad::full_map(view))?;
        link.show(maps.page_map_view());
        Ok(outcome)
    }
}
