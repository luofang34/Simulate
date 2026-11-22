//! Direct-to-swapchain vector tessellation storage and helpers for `AvionicsRig`.
//!
//! When direct presentation is active, `Indicate` PFD, HSI, and HUD triangles stay in
//! reusable host buffers so [`sim_render::DirectVectorPass`] draws them onto the final
//! `1×` swapchain surface without intermediate offscreen targets or GPU texture copies.

use sim_indicate::{MonitoredFrame, PfdConfig, VectorHudPanel};
use sim_render::{VectorTarget, VectorVertex};

use super::{AvionicsFrameInput, AvionicsRig, HudView, hud, pfd_config_for};
use crate::error::AppSimError;

/// Reusable triangle buffers filled when `AvionicsFrameInput::direct_mode` is `true`.
#[derive(Debug, Default)]
pub(super) struct DirectVectors {
    pub(super) pfd: Vec<VectorVertex>,
    pub(super) hsi: Vec<VectorVertex>,
    pub(super) hud: Vec<VectorVertex>,
    pub(super) pfd_ready: bool,
    pub(super) hsi_ready: bool,
    pub(super) hud_ready: bool,
}

impl DirectVectors {
    /// Marks every buffer empty while preserving its heap capacity across frames.
    pub(super) fn clear(&mut self) {
        self.pfd.clear();
        self.hsi.clear();
        self.hud.clear();
        self.pfd_ready = false;
        self.hsi_ready = false;
        self.hud_ready = false;
    }
}

impl AvionicsRig {
    /// Latest panel-local PFD triangles for direct swapchain rendering, when active this frame.
    #[must_use]
    pub fn direct_pfd_vertices(&self) -> Option<&[VectorVertex]> {
        self.direct.pfd_ready.then_some(self.direct.pfd.as_slice())
    }

    /// Latest panel-local HSI triangles for direct swapchain rendering, when active this frame.
    #[must_use]
    pub fn direct_hsi_vertices(&self) -> Option<&[VectorVertex]> {
        self.direct.hsi_ready.then_some(self.direct.hsi.as_slice())
    }

    /// Latest viewport-local conformal HUD triangles for direct swapchain rendering, when active.
    #[must_use]
    pub fn direct_hud_vertices(&self) -> Option<&[VectorVertex]> {
        self.direct.hud_ready.then_some(self.direct.hud.as_slice())
    }

    pub(super) fn encode_offscreen_vectors(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        monitored: &MonitoredFrame,
        input: &AvionicsFrameInput<'_>,
        hud_view: Option<HudView<'_>>,
    ) -> Result<(), AppSimError> {
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("indicate avionics vector"),
        });
        let mut recorded = false;
        if let Some([pw, ph]) = input.dock_panel_px {
            let pfd_cfg = pfd_config_for(
                self.domain,
                input.svs_underlay_active,
                input.airspeed_envelope.as_ref(),
            );
            self.encode_vector_panels(device, queue, &mut encoder, monitored, &pfd_cfg, [pw, ph])?;
            recorded = true;
        }
        if let Some(view) = hud_view {
            recorded |=
                self.encode_vector_hud(device, queue, &mut encoder, monitored, input, &view)?;
        }
        if recorded {
            queue.submit([encoder.finish()]);
        }
        Ok(())
    }

    pub(super) fn update_direct_vectors(
        &mut self,
        monitored: &MonitoredFrame,
        input: &AvionicsFrameInput<'_>,
        hud_view: Option<HudView<'_>>,
    ) -> Result<(), AppSimError> {
        if let Some([raw_w, raw_h]) = input.dock_panel_px {
            let (w, h) = (raw_w.max(64), raw_h.max(48));
            let pfd_cfg = pfd_config_for(
                self.domain,
                input.svs_underlay_active,
                input.airspeed_envelope.as_ref(),
            );
            self.tessellate_panels_direct(monitored, &pfd_cfg, w, h)?;
        }
        if let Some(view) = hud_view {
            self.tessellate_hud_direct(monitored, input, &view)?;
        }
        Ok(())
    }

    fn tessellate_panels_direct(
        &mut self,
        monitored: &MonitoredFrame,
        config: &PfdConfig,
        w: u32,
        h: u32,
    ) -> Result<(), AppSimError> {
        let pfd_verts = self.panels.tessellate_pfd_slice(
            &monitored.panel,
            config,
            Some(&monitored.alerts),
            w,
            h,
        )?;
        self.direct.pfd.clear();
        self.direct.pfd.extend_from_slice(pfd_verts);
        self.direct.pfd_ready = true;

        let hsi_verts =
            self.panels
                .tessellate_hsi_slice(&monitored.panel, Some(&monitored.alerts), w, h)?;
        self.direct.hsi.clear();
        self.direct.hsi.extend_from_slice(hsi_verts);
        self.direct.hsi_ready = true;
        Ok(())
    }

    fn tessellate_hud_direct(
        &mut self,
        monitored: &MonitoredFrame,
        input: &AvionicsFrameInput<'_>,
        view: &HudView<'_>,
    ) -> Result<(), AppSimError> {
        let [w, h] = view.surface_size.map(|v| v.max(1));
        if VectorHudPanel::design_frame(w, h).is_err() {
            return Ok(());
        }
        let context = hud::hud_context(
            &input.vehicle,
            view,
            input.runway,
            input.airspeed_envelope.as_ref(),
        );
        let vertices = self.hud_panel.tessellate_slice(
            &monitored.panel,
            &context,
            Some(&monitored.alerts),
            w,
            h,
        )?;
        self.direct.hud.clear();
        self.direct.hud.extend_from_slice(vertices);
        self.direct.hud_ready = true;
        Ok(())
    }
}

pub(super) fn ensure_target<'a>(
    slot: &'a mut Option<VectorTarget>,
    device: &wgpu::Device,
    label: &'static str,
    w: u32,
    h: u32,
    samples: u32,
) -> Result<&'a mut VectorTarget, AppSimError> {
    match slot {
        Some(target) => {
            target.ensure_size(device, w, h)?;
            Ok(target)
        }
        None => Ok(slot.insert(VectorTarget::with_sample_count(
            device, label, w, h, samples,
        )?)),
    }
}
