//! The P-38G's skin, asked point by point: the factory paint, markings,
//! panel seams, rivets and wear of `"Miss Virginia"` as a renderer's bake
//! needs them. Everything is placed in drawing units (fuselage station,
//! waterline, buttock line in inches) or against features measured from
//! the airframe's own meshes, so a re-shaped airframe re-bakes in place.

mod anchors;
mod glyphs;
mod livery;
mod markings;
mod noise;
mod panel_lines;
mod panels;
mod scheme;
mod seams;
#[cfg(test)]
mod tests;
mod victories;
mod weathering;

use glam::Vec3;

use crate::mesh::P38MeshPrim;
use anchors::{Anchors, M_PER_IN};
use seams::Seams;
use weathering::Exposure;

/// One point of the skin a bake asks about, in the aircraft frame at rest
/// (`+X` right, `+Y` forward, `+Z` up, metres).
#[derive(Clone, Copy, Debug)]
pub struct SkinQuery<'a> {
    /// USD prim path of the part the point is on.
    pub part: &'a str,
    /// Where the point is.
    pub position: Vec3,
    /// Unit outward normal.
    pub normal: Vec3,
    /// Unit directions of increasing texture `u` and `v` in the surface.
    pub tangent: Vec3,
    /// See [`Self::tangent`].
    pub bitangent: Vec3,
    /// Vertex-baked share of open sky, `1` fully open.
    pub sky: f32,
    /// Width of skin the bake's texel covers, metres.
    pub footprint_m: f32,
}

/// The skin at a point.
#[derive(Clone, Copy, Debug)]
pub struct SkinPaint {
    /// Linear base colour.
    pub albedo: Vec3,
    /// Perceptual roughness.
    pub roughness: f32,
    /// Metallic factor (bare aluminium where paint is worn through).
    pub metallic: f32,
    /// Occlusion of seams and rivet edges, `1` open.
    pub occlusion: f32,
    /// Shading normal, aircraft frame.
    pub normal: Vec3,
}

/// The P-38G's skin, measured against one build of its geometry.
#[derive(Clone, Debug)]
pub struct P38Skin {
    anchors: Anchors,
    seams: Seams,
}

impl P38Skin {
    /// The skin of the airframe built as `prims`.
    #[must_use]
    pub fn new(prims: &[P38MeshPrim]) -> Self {
        Self {
            anchors: Anchors::measure(prims),
            seams: Seams::new(),
        }
    }

    /// Whether the part at `path` is painted by the skin: the stressed skin
    /// of the gondola, booms, wing and tail, their moving surfaces, and the
    /// spinners (painted, not bare, on the 339th's G-13s: USMC
    /// 127-GR-8-72001).
    #[must_use]
    pub fn paints(path: &str) -> bool {
        let Some(path) = path.strip_prefix("/P38g/") else {
            return false;
        };
        matches!(
            path,
            "Gondola/UpperOlive" | "Gondola/LowerGray" | "Gondola/NoseUpperDeck"
        ) || path.starts_with("Booms/")
            || (path.starts_with("Wing/") && path != "Wing/PitotMast")
            || path.starts_with("Empennage/")
            || path.ends_with("/SpinnerBase")
            || path.ends_with("/SpinnerNose")
    }

    /// The skin at `query`.
    #[must_use]
    pub fn sample(&self, query: &SkinQuery<'_>) -> SkinPaint {
        let n = query.normal;
        let coat = scheme::coat(query.part, n);
        let (mut albedo, roughness) = scheme::base(coat);
        let footprint_in = query.footprint_m / M_PER_IN;
        for marking in livery::MARKINGS {
            if let Some(paint) =
                marking.paint(&self.anchors, query.part, query.position, n, footprint_in)
            {
                albedo = albedo.lerp(paint.colour, paint.cover);
            }
        }
        let relief = self
            .seams
            .relief(&self.anchors, query.part, query.position, n, footprint_in);
        let exposure = Exposure {
            position: query.position,
            normal: n,
            sky: query.sky,
            coat,
            seam_in: relief.distance_in,
            on_wing: matches!(
                query.part,
                "/P38g/Wing/UpperOlive" | "/P38g/Wing/FuelBayPanels" | "/P38g/Wing/KarmanFillets"
            ),
        };
        let mut worn = weathering::wear(&self.anchors, &exposure, albedo, roughness);
        if coat == scheme::Coat::Olive && relief.touch_up > 0.0 {
            // Fresh olive drab, darker and less chalky than the faded skin.
            let (fresh, gloss) = scheme::base(coat);
            worn.albedo = worn.albedo.lerp(fresh * 0.8, relief.touch_up * 0.95);
            worn.roughness += (gloss - worn.roughness) * relief.touch_up;
        }
        // Dirt in the gaps and around the rivet heads; the gap holds shadow.
        let dirt = 1.0 - 0.7 * relief.seam - 0.12 * relief.rivet;
        SkinPaint {
            albedo: worn.albedo * dirt,
            roughness: (worn.roughness + 0.15 * relief.seam).min(1.0),
            metallic: worn.metallic * (1.0 - relief.seam),
            occlusion: 1.0 - 0.6 * relief.seam - 0.2 * relief.rivet,
            normal: (n - relief.uphill * relief.slope).normalize_or(n),
        }
    }
}
