//! The world the terrain camera sees: the same sealed package the navigator
//! carries as its map, decoded whole in the page's memory, and what that
//! costs the tab.

use navigate_data::LayerKind;
use sim_navdata::{PackagePlan, SourceSet};

use crate::PlanLine;
use crate::wording::line;

mod camera;
mod overview;

#[cfg(test)]
mod tests;

pub use camera::CameraWorld;
pub use overview::{MapView, mercator, unmercator};

/// The most of a tab's memory the decoded world may take, bytes. A
/// browser grants a page's WebAssembly a few gigabytes at best and less on
/// a phone; the flight, the navigators and the page need the rest.
pub const WORLD_MEMORY_CEILING_BYTES: u64 = 1_000_000_000;

/// Bytes a decoded pixel of `layer` takes in the camera's world: imagery
/// as four 8-bit channels, elevation as one 64-bit height.
const fn decoded_bytes_per_pixel(layer: LayerKind) -> u64 {
    match layer {
        LayerKind::Imagery => 4,
        LayerKind::Elevation => 8,
        LayerKind::Unsupported => 0,
    }
}

/// The tab memory the camera's world of `plan` takes once decoded, bytes:
/// every planned tile, whole, at its source's tile size. It is many times
/// the package's own bytes, which hold the tiles compressed.
pub fn world_memory_bytes(plan: &PackagePlan, sources: &SourceSet) -> u64 {
    plan.size
        .layers
        .iter()
        .map(|size| {
            let side = u64::from(sources.of(size.layer).limits.tile_pixels);
            size.tiles
                .saturating_mul(side.saturating_mul(side))
                .saturating_mul(decoded_bytes_per_pixel(size.layer))
        })
        .fold(0, u64::saturating_add)
}

fn megabytes(bytes: u64) -> u64 {
    bytes.div_ceil(1_000_000)
}

/// What the camera's world is, said plainly beside the visual results:
/// the camera sees the imagery the onboard map was built from, and what
/// decoding it costs the tab.
pub fn world_line(plan: &PackagePlan, sources: &SourceSet) -> PlanLine {
    line(
        "Camera's world",
        format!(
            "the camera sees the same imagery the onboard map was built from, which favours \
             visual navigation. Decoded for the camera it takes about {} MB of this tab's \
             memory; the page allows {} MB.",
            megabytes(world_memory_bytes(plan, sources)),
            megabytes(WORLD_MEMORY_CEILING_BYTES)
        ),
    )
}

/// The refusal of a plan whose decoded world does not fit the tab: the
/// numbers and the visitor's choices. `None` when it fits.
pub fn world_memory_refusal(plan: &PackagePlan, sources: &SourceSet) -> Option<Vec<PlanLine>> {
    let needed = world_memory_bytes(plan, sources);
    (needed > WORLD_MEMORY_CEILING_BYTES).then(|| {
        vec![
            line(
                "Reason",
                format!(
                    "the camera's world needs about {} MB of this tab's memory once decoded, \
                     for {} tiles; the page allows {} MB.",
                    megabytes(needed),
                    plan.tile_count(),
                    megabytes(WORLD_MEMORY_CEILING_BYTES)
                ),
            ),
            line(
                "Choices",
                "Shorten the route, or fly higher: a higher flight is matched on coarser \
                 tiles. This page does not lower the map detail or cut the route by itself."
                    .to_owned(),
            ),
        ]
    })
}
