//! A page's PFD and HSI on canvases of their own beside the window view, so
//! the HUD over the world is never covered. The panels are the dock's own,
//! drawn onto other surfaces.

use sim_render::{OverlayQuad, PanelSurface, RenderError, Renderer};

/// One panel's surface and how big its canvas is laid out now.
pub struct PagePanel {
    surface: PanelSurface,
    laid_out_px: Box<dyn Fn() -> [u32; 2]>,
}

impl PagePanel {
    /// The panel drawn on `surface`, whose canvas's laid-out size `laid_out_px` reads in
    /// physical pixels.
    #[cfg(target_arch = "wasm32")]
    pub fn new(surface: PanelSurface, laid_out_px: Box<dyn Fn() -> [u32; 2]>) -> Self {
        Self {
            surface,
            laid_out_px,
        }
    }
}

/// The PFD's and the HSI's panels.
pub struct PagePanels {
    pfd: PagePanel,
    hsi: PagePanel,
}

impl PagePanels {
    /// The PFD drawn on `pfd`, the HSI on `hsi`.
    #[cfg(target_arch = "wasm32")]
    pub const fn new(pfd: PagePanel, hsi: PagePanel) -> Self {
        Self { pfd, hsi }
    }

    /// Resizes both surfaces to their canvases as laid out now, and returns the size the
    /// instruments are drawn at: the larger panel's, so neither is upscaled.
    pub fn follow_layout(&mut self, renderer: &Renderer) -> [u32; 2] {
        for panel in [&mut self.pfd, &mut self.hsi] {
            let [width, height] = (panel.laid_out_px)();
            renderer.resize_panel(&mut panel.surface, width, height);
        }
        let ([pw, ph], [hw, hh]) = (self.pfd.surface.size(), self.hsi.surface.size());
        [pw.max(hw), ph.max(hh)]
    }

    /// Presents the PFD's quads and the HSI's, each on its own surface.
    pub fn present(
        &mut self,
        renderer: &mut Renderer,
        [pfd, hsi]: &[Vec<OverlayQuad<'_>>; 2],
    ) -> Result<(), RenderError> {
        renderer.present_panel(&mut self.pfd.surface, pfd)?;
        renderer.present_panel(&mut self.hsi.surface, hsi)?;
        Ok(())
    }
}
