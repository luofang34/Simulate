//! The quad and vector layers of one window frame, encoded for one queue submission.

use crate::RenderError;
use crate::compositor::{CompositorPass, CompositorTarget, OverlayQuad};
use crate::vector_pass::{DirectVectorDraw, DirectVectorPass, DirectVectorTarget};

/// What a window frame draws over its optional globe pass, bottom layer first.
pub(super) struct FrameLayers<'a> {
    /// The colour the bottom layer clears the target to; `None` keeps what is there.
    pub clear: Option<wgpu::Color>,
    /// Textures under the vector scenes (the outside world, map underlays).
    pub under_quads: &'a [OverlayQuad<'a>],
    /// Vector scenes drawn straight onto the target.
    pub direct_vectors: &'a [DirectVectorDraw<'a>],
    /// Textures over the vector scenes (the corner map, menus).
    pub over_quads: &'a [OverlayQuad<'a>],
}

/// Encodes `layers` onto `target`. Both quad layers go through one
/// [`crate::CompositorLayers`]: they land in one submission, where a layer encoded on
/// its own would take the other's quad slots and redraw its quads.
pub(super) fn encode_frame_layers(
    compositor: &CompositorPass,
    direct_vector: &mut DirectVectorPass,
    (device, queue): (&wgpu::Device, &wgpu::Queue),
    encoder: &mut wgpu::CommandEncoder,
    target: DirectVectorTarget<'_>,
    layers: &FrameLayers<'_>,
) -> Result<(), RenderError> {
    let mut quad_layers = compositor.layers(device, queue);
    quad_layers.encode(
        encoder,
        CompositorTarget {
            view: target.color_view,
            clear: layers.clear,
            size_px: target.size_px,
        },
        layers.under_quads,
    );
    if !layers.direct_vectors.is_empty() {
        direct_vector.encode(device, queue, encoder, target, layers.direct_vectors)?;
    }
    quad_layers.encode(
        encoder,
        CompositorTarget {
            view: target.color_view,
            clear: None,
            size_px: target.size_px,
        },
        layers.over_quads,
    );
    Ok(())
}
