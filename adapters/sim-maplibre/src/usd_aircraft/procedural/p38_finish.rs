//! Finish of the P-38G's parts: its stressed skin is painted by the baked
//! [`P38Skin`]; the few painted parts outside it (wheel wells, coolant and
//! oil flaps) keep the shader's procedural weathering.

use sim_p38g::{P38Skin, SkinQuery};

use crate::UsdMeshPart;
use crate::usd_aircraft::finish::SurfaceDetail;
use crate::usd_aircraft::skin::{SkinLayout, SkinSample, SkinSource, SkinTexel};

/// The P-38G's bake: 4096 texels a side gives about 4 mm a texel over its
/// skin, so a panel seam and a rivet row resolve from a few metres; the
/// occlusion/roughness/metallic pack at half that. Albedo 64 MiB, normal
/// 32 MiB and the pack 16 MiB, with mips about 150 MiB in all.
pub(super) const SKIN_LAYOUT: SkinLayout = SkinLayout {
    side: 4096,
    gutter: 4,
    half_orm: true,
};

/// [`P38Skin`] asked for each texel of the bake.
pub(super) struct P38SkinSource {
    pub(super) skin: P38Skin,
    /// Prim path of each part, by index.
    pub(super) paths: Vec<String>,
}

impl SkinSource for P38SkinSource {
    fn sample(&self, texel: &SkinTexel) -> SkinSample {
        let part = self.paths.get(texel.part).map_or("", String::as_str);
        let paint = self.skin.sample(&SkinQuery {
            part,
            position: texel.position,
            normal: texel.normal,
            tangent: texel.tangent,
            bitangent: texel.bitangent,
            sky: texel.occlusion,
            footprint_m: texel.footprint_m,
        });
        SkinSample {
            albedo: paint.albedo,
            occlusion: paint.occlusion,
            roughness: paint.roughness,
            metallic: paint.metallic,
            normal: paint.normal,
        }
    }
}

/// Paints the assembled P-38G: recesses its wheel wells and bakes `skin`
/// onto its stressed skin.
pub(super) fn finish(asset: &mut crate::UsdAircraftAsset, skin: P38Skin) {
    recess_wheel_wells(&mut asset.parts);
    let source = P38SkinSource {
        skin,
        paths: asset.parts.iter().map(|p| p.prim_path.clone()).collect(),
    };
    asset.bake_skin(SKIN_LAYOUT, &source, skinned);
}

/// Open sky seen from inside a gear bay through its opening.
const BAY_SKY: f32 = 0.2;

/// Darkens the wheel wells: the airframe models each bay as a closed box
/// where the real one is an open, deep recess whose interior sees little
/// sky, which the vertex bake of a box's outer faces cannot find.
///
/// Temporary: this stands in for open gear-bay geometry. Once the
/// airframe models the bays as recesses, remove it and let the vertex bake
/// see the real bay.
fn recess_wheel_wells(parts: &mut [UsdMeshPart]) {
    for part in parts
        .iter_mut()
        .filter(|p| p.prim_path == "/P38g/Gear/WheelWells")
    {
        for vertex in &mut part.vertices {
            vertex.occlusion = vertex.occlusion.min(BAY_SKY);
        }
    }
}

/// Whether `part` is painted by the P-38G's skin.
pub(super) fn skinned(part: &UsdMeshPart) -> bool {
    P38Skin::paints(&part.prim_path)
}

/// The procedural finish of an unskinned part at `prim_path`.
pub(super) fn detail(prim_path: &str) -> SurfaceDetail {
    let path = prim_path.trim_start_matches("/P38g/");
    match path {
        p if p.starts_with("Gear/WheelWells") || p.starts_with("Cooling/") => SurfaceDetail {
            panel_pitch_m: [0.0; 3],
            weathering: 0.6,
            soot: None,
        },
        _ => SurfaceDetail::default(),
    }
}
