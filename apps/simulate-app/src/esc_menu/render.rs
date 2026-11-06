//! GPU side of the hub: a `4×` MSAA premultiplied vector target the size of the window.

use sim_render::{VectorScenePass, VectorTarget};

use super::flight_chrome::tessellate_flight_chrome;
use super::scene::{Highlights, tessellate_banner, tessellate_hub};
use super::view::HubView;
use crate::cockpit_rig::efb_chrome::EfbChrome;
use crate::error::AppSimError;

/// What the overlay shows this frame.
#[derive(Debug, Clone, Copy)]
pub enum OverlayContent<'a> {
    /// Nothing.
    Hidden,
    /// The open hub.
    Hub {
        /// The laid-out hub.
        view: &'a HubView,
        /// Focus and hover.
        highlights: Highlights,
    },
    /// A one-line banner over the flight.
    Banner(&'a str),
    /// In-flight chrome: the optional takeover banner and the EFB's bezel and buttons.
    FlightChrome {
        /// Takeover banner text, if shown.
        banner: Option<&'a str>,
        /// The EFB's chrome, if the EFB is shown.
        efb: Option<&'a EfbChrome>,
    },
}

/// The hub's vector pass and its window-sized target.
pub struct MenuOverlay {
    pass: VectorScenePass,
    target: Option<VectorTarget>,
    shown: bool,
}

impl MenuOverlay {
    /// Wraps a compiled vector pass; the target is allocated on the first shown frame.
    #[must_use]
    pub const fn new(pass: VectorScenePass) -> Self {
        Self {
            pass,
            target: None,
            shown: false,
        }
    }

    /// Redraws `content` at `surface_px`, or hides the overlay.
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        content: OverlayContent<'_>,
        surface_px: [u32; 2],
    ) -> Result<(), AppSimError> {
        self.shown = false;
        let [w, h] = surface_px.map(|v| v.max(1));
        let vertices = match content {
            OverlayContent::Hidden => return Ok(()),
            OverlayContent::Hub { view, highlights } => tessellate_hub(view, highlights, [w, h])?,
            OverlayContent::Banner(text) => tessellate_banner(text, [w, h])?,
            OverlayContent::FlightChrome {
                banner: None,
                efb: None,
            } => return Ok(()),
            OverlayContent::FlightChrome { banner, efb } => {
                tessellate_flight_chrome(banner, efb, [w, h])?
            }
        };
        let target = match self.target.as_mut() {
            Some(target) => {
                target.ensure_size(device, w, h)?;
                target
            }
            None => self
                .target
                .insert(VectorTarget::new(device, "flight hub", w, h)?),
        };
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("flight hub"),
        });
        self.pass
            .encode(device, queue, &mut encoder, target, &vertices)?;
        queue.submit([encoder.finish()]);
        self.shown = true;
        Ok(())
    }

    /// The resolved premultiplied overlay view, when it drew this frame.
    #[must_use]
    pub fn view(&self) -> Option<&wgpu::TextureView> {
        self.target
            .as_ref()
            .filter(|_| self.shown)
            .map(VectorTarget::view)
    }
}
