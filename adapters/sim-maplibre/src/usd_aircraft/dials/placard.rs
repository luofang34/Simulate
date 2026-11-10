//! Placards lettered onto their plates: each [`PartRole::DisplaySurface`]
//! named after a placard gets its wording baked once into the dial atlas, in
//! the panel's own typeface.

use glam::Vec3;
use indicate_instrument_scene::FaceSlot;
use sim_indicate::{FaceBake, FaceTable, bake_placard};
use tracing::warn;

use super::FACE_PX;
use super::atlas::AtlasBuilder;
use crate::usd_aircraft::{PartRole, PartSurface, UsdMeshPart};

/// An airframe's placards and the face slot they are lettered in.
#[derive(Debug, Clone, Default)]
pub(crate) struct Placards {
    /// `(prim path, wording)` of each placard plate.
    pub(crate) plates: Vec<(String, String)>,
    /// The slot of the face they are lettered in.
    pub(crate) slot: FaceSlot,
}

/// The plate's width over its height, from its quad.
fn aspect(part: &UsdMeshPart) -> f32 {
    let corner = |i: usize| {
        part.vertices
            .get(i)
            .map_or(Vec3::ZERO, |v| Vec3::from(v.position))
    };
    let width = corner(1).distance(corner(0));
    let height = corner(3).distance(corner(0));
    if height > 0.0 { width / height } else { 1.0 }
}

/// Each placard plate among `parts` with its baked wording.
pub(super) fn bake(
    parts: &[UsdMeshPart],
    placards: &Placards,
    faces: &FaceTable<'_>,
) -> Vec<(usize, FaceBake)> {
    parts
        .iter()
        .enumerate()
        .filter(|(_, part)| part.role == PartRole::DisplaySurface)
        .filter_map(|(index, part)| {
            let (_, text) = placards
                .plates
                .iter()
                .find(|(path, _)| *path == part.prim_path)?;
            let height = ((FACE_PX as f32 / aspect(part).max(1.0)).round() as u32).max(8);
            match bake_placard(text, FACE_PX, height, faces, placards.slot) {
                Ok(bake) => Some((index, bake)),
                Err(err) => {
                    warn!(placard = %part.prim_path, error = %err, "placard did not bake");
                    None
                }
            }
        })
        .collect()
}

/// Places each baked placard in `atlas` and prints its plate with it.
pub(super) fn apply(
    parts: &mut [UsdMeshPart],
    atlas: &mut AtlasBuilder,
    cell: &mut impl Iterator<Item = u32>,
    baked: Vec<(usize, FaceBake)>,
) {
    for (index, bake) in baked {
        let (Some(cell), Some(part)) = (cell.next(), parts.get_mut(index)) else {
            continue;
        };
        let uv_rect = atlas.place(cell, [bake.width, bake.height], &bake.rgba);
        part.surface = PartSurface::Display {
            uv_rect,
            glow_rgb: [0.0; 3],
        };
    }
}
