//! Runtime composition of a [`sim_pack::LiverySpec`] into OpenUSD `.usda` layers projected
//! `4 mm` proud onto the C-172 fuselage and vertical-stabilizer skin (`native` + `wasm32`).

use glam::Vec3;
use sim_pack::{LiveryPhotoSpec, LiverySpec, LiveryStripeSpec};

use super::{UsdAircraftAsset, UsdMeshPart};
use crate::MapAdapterError;

mod glyphs;
use glyphs::{
    sanitize_doc, sanitize_ident, write_material, write_mesh_prim, write_registration_pair,
};

/// Outward offset along the skin surface normal (`4 mm`) preventing depth fighting.
const STAND_OFF_M: f32 = 0.004;

const SKIN_SUFFIXES: [&str; 6] = [
    "/CabinWhite",
    "/CabinStripe",
    "/NoseCowling",
    "/NoseCowlingStripe",
    "/VerticalStabilizer",
    "/VertStabTip",
];

#[derive(Clone, Copy)]
struct SkinTri {
    a: Vec3,
    b: Vec3,
    c: Vec3,
    n: Vec3,
    ymin: f32,
    ymax: f32,
    zmin: f32,
    zmax: f32,
}

/// Composes the root `.usda` layer referencing `airframe_usda` and `livery_usda` and
/// overriding `/C172/Materials/MatWhitePaint` when `spec.base_coat_rgb` is set.
pub(super) fn compose_c172_root_usda(
    spec: &LiverySpec,
    airframe_usda: &str,
    livery_usda: &str,
) -> String {
    let mut out = String::from("#usda 1.0\n(\n");
    let title = spec.name.as_deref().unwrap_or(&spec.tail);
    out.push_str(&format!(
        "    doc = \"Runtime-imported C-172 tail {} ({})\"\n",
        sanitize_doc(&spec.tail),
        sanitize_doc(title)
    ));
    out.push_str("    defaultPrim = \"C172\"\n    metersPerUnit = 1\n    upAxis = \"Z\"\n");
    out.push_str(&format!(
        "    subLayers = [\n        @{livery_usda}@,\n        @{airframe_usda}@,\n        @c172.usdc@\n    ]\n)\n\nover \"C172\"\n{{\n    string sim:tail = \"{}\"\n",
        sanitize_doc(&spec.tail)
    ));
    if let Some([r, g, b]) = spec.base_coat_rgb {
        out.push_str(&format!(
            "    over \"Materials\"\n    {{\n        over \"MatWhitePaint\"\n        {{\n            over \"PreviewSurface\"\n            {{\n                color3f inputs:diffuseColor = ({r:.4}, {g:.4}, {b:.4})\n            }}\n        }}\n    }}\n"
        ));
    }
    if !spec.skin_overrides.is_empty() {
        out.push_str("    over \"Airframe\"\n    {\n");
        for (prim, mat) in &spec.skin_overrides {
            let prim_id = sanitize_ident(prim);
            let mat_id = sanitize_ident(mat);
            out.push_str(&format!(
                "        over \"{prim_id}\"\n        {{\n            rel material:binding = </C172/Materials/{mat_id}>\n        }}\n"
            ));
        }
        out.push_str("    }\n");
    }
    out.push_str("}\n");
    out
}

/// Projects `spec`'s cheatlines, fin flash, and registration marks onto the C-172 skin and
/// emits a self-contained `.usda` livery sublayer.
pub(super) fn compose_c172_livery_usda(
    base_c172: &UsdAircraftAsset,
    spec: &LiverySpec,
) -> Result<String, MapAdapterError> {
    let tris = collect_skin_triangles(base_c172);
    let photo = spec.photo.as_ref().and_then(PhotoTransform::fit);
    let mut out = String::from(
        "#usda 1.0\n(\n    defaultPrim = \"C172\"\n    metersPerUnit = 1\n    upAxis = \"Z\"\n)\n\n",
    );
    out.push_str("over \"C172\"\n{\n    over \"Materials\"\n    {\n");
    for (mat_name, mat) in &spec.materials {
        write_material(
            &mut out,
            mat_name,
            mat.diffuse.map(f64::from),
            f64::from(mat.metallic),
            f64::from(mat.roughness),
        );
    }
    out.push_str("    }\n\n    over \"Airframe\"\n    {\n");
    for stripe in spec.stripes.iter().chain(spec.fin_flash.iter()) {
        write_stripe_pair(&mut out, &tris, stripe, photo);
    }
    write_registration_pair(&mut out, &tris, &spec.registration, photo);
    out.push_str("    }\n}\n");
    Ok(out)
}

#[derive(Clone, Copy)]
struct PhotoTransform {
    a: f64,
    b: f64,
    ty: f64,
    tz: f64,
}

impl PhotoTransform {
    fn fit(photo: &LiveryPhotoSpec) -> Option<Self> {
        let n = photo.anchors_px_to_model_m.len();
        if n < 2 {
            return None;
        }
        let nf = n as f64;
        let (mut mx, mut my, mut mu, mut mv) = (0.0, 0.0, 0.0, 0.0);
        for item in &photo.anchors_px_to_model_m {
            mx += item.px[0] / nf;
            my += item.px[1] / nf;
            mu += item.model_yz[0] / nf;
            mv += item.model_yz[1] / nf;
        }
        let (mut sxx, mut num_a, mut num_b) = (0.0, 0.0, 0.0);
        for item in &photo.anchors_px_to_model_m {
            let (dx, dy) = (item.px[0] - mx, item.px[1] - my);
            let (du, dv) = (item.model_yz[0] - mu, item.model_yz[1] - mv);
            sxx += dx * dx + dy * dy;
            num_a += dx * du - dy * dv;
            num_b += dy * du + dx * dv;
        }
        if sxx < 1e-9 {
            return None;
        }
        let (a, b) = (num_a / sxx, num_b / sxx);
        Some(Self {
            a,
            b,
            ty: mu - (a * mx + b * my),
            tz: mv - (b * mx - a * my),
        })
    }

    fn map_px(self, u: f64, v: f64) -> [f64; 2] {
        [
            self.a * u + self.b * v + self.ty,
            self.b * u - self.a * v + self.tz,
        ]
    }
}

fn collect_skin_triangles(asset: &UsdAircraftAsset) -> Vec<SkinTri> {
    let mut tris = Vec::new();
    for part in asset.parts().iter().filter(|p| is_skin_part(p)) {
        append_part_triangles(part, &mut tris);
    }
    tris
}

fn is_skin_part(part: &UsdMeshPart) -> bool {
    SKIN_SUFFIXES
        .iter()
        .any(|suf| part.prim_path.ends_with(suf))
}

fn append_part_triangles(part: &UsdMeshPart, out: &mut Vec<SkinTri>) {
    let xf = part.rest_to_aircraft.as_mat4();
    for &[i0, i1, i2] in part.indices.as_chunks::<3>().0 {
        let (Some(v0), Some(v1), Some(v2)) = (
            part.vertices.get(i0 as usize),
            part.vertices.get(i1 as usize),
            part.vertices.get(i2 as usize),
        ) else {
            continue;
        };
        let a = xf.transform_point3(Vec3::from_array(v0.position));
        let b = xf.transform_point3(Vec3::from_array(v1.position));
        let c = xf.transform_point3(Vec3::from_array(v2.position));
        let n = (b - a).cross(c - a).normalize_or_zero();
        if n.length_squared() > 0.25 {
            out.push(SkinTri {
                a,
                b,
                c,
                n,
                ymin: a.y.min(b.y).min(c.y),
                ymax: a.y.max(b.y).max(c.y),
                zmin: a.z.min(b.z).min(c.z),
                zmax: a.z.max(b.z).max(c.z),
            });
        }
    }
}

fn project_onto_skin(tris: &[SkinTri], y: f32, z: f32, side: f32) -> Option<(Vec3, Vec3)> {
    let mut best: Option<(f32, Vec3, Vec3)> = None;
    for t in tris {
        if y < t.ymin - 1e-4 || y > t.ymax + 1e-4 || z < t.zmin - 1e-4 || z > t.zmax + 1e-4 {
            continue;
        }
        let Some((x, n)) = hit_yz(t, y, z) else {
            continue;
        };
        if x * side < -0.02 {
            continue;
        }
        let score = x * side;
        if best.is_none_or(|(prev, _, _)| score > prev) {
            best = Some((score, Vec3::new(x, y, z) + n * STAND_OFF_M, n));
        }
    }
    best.map(|(_, p, n)| (p, n))
}

fn hit_yz(t: &SkinTri, y: f32, z: f32) -> Option<(f32, Vec3)> {
    let (u1, v1) = (t.b.y - t.a.y, t.b.z - t.a.z);
    let (u2, v2) = (t.c.y - t.a.y, t.c.z - t.a.z);
    let det = u1 * v2 - u2 * v1;
    if det.abs() < 1e-8 {
        return None;
    }
    let (du, dv) = (y - t.a.y, z - t.a.z);
    let s = (du * v2 - u2 * dv) / det;
    let r = (u1 * dv - du * v1) / det;
    if s < -1e-4 || r < -1e-4 || s + r > 1.0 + 1e-4 {
        return None;
    }
    let x = t.a.x + s * (t.b.x - t.a.x) + r * (t.c.x - t.a.x);
    let mut n = t.n;
    if n.x * x < 0.0 {
        n = -n;
    }
    Some((x, n))
}

fn resolve_band_yz(
    stripe: &LiveryStripeSpec,
    photo: Option<PhotoTransform>,
) -> (Vec<[f64; 2]>, Vec<[f64; 2]>) {
    if stripe.upper_yz.len() >= 2 && stripe.lower_yz.len() >= 2 {
        return (stripe.upper_yz.clone(), stripe.lower_yz.clone());
    }
    let (Some([u0, v0, scale]), Some(xf)) = (stripe.crop_px, photo) else {
        return (Vec::new(), Vec::new());
    };
    let s = scale.max(1e-6);
    let conv = |pts: &[[f64; 2]]| -> Vec<[f64; 2]> {
        pts.iter()
            .map(|&[cu, cv]| xf.map_px(u0 + cu / s, v0 + cv / s))
            .collect()
    };
    (conv(&stripe.upper), conv(&stripe.lower))
}

fn write_stripe_pair(
    out: &mut String,
    tris: &[SkinTri],
    stripe: &LiveryStripeSpec,
    photo: Option<PhotoTransform>,
) {
    let (upper, lower) = resolve_band_yz(stripe, photo);
    let cols = sample_trace_columns(&upper, &lower, 0.05);
    let stem = sanitize_ident(&stripe.name);
    for (suffix, side) in [("Starboard", 1.0_f32), ("Port", -1.0_f32)] {
        if let Some((pts, nrms, counts, indices)) = build_ribbon(tris, &cols, side) {
            let prim = format!("Livery{stem}{suffix}");
            write_mesh_prim(out, &prim, &stripe.material, &pts, &nrms, &counts, &indices);
        }
    }
}

fn sample_trace_columns(
    upper: &[[f64; 2]],
    lower: &[[f64; 2]],
    step_m: f64,
) -> Vec<([f32; 2], [f32; 2])> {
    let (Some(&[u0y, _]), Some(&[u1y, _]), Some(&[l0y, _]), Some(&[l1y, _])) =
        (upper.first(), upper.last(), lower.first(), lower.last())
    else {
        return Vec::new();
    };
    let span = (u1y - u0y).abs().max((l1y - l0y).abs());
    let steps = ((span / step_m).ceil() as usize).clamp(2, 160);
    (0..=steps)
        .map(|k| {
            let t = k as f64 / steps as f64;
            (lerp_polyline(upper, t), lerp_polyline(lower, t))
        })
        .collect()
}

fn lerp_polyline(pts: &[[f64; 2]], t: f64) -> [f32; 2] {
    if pts.len() <= 1 {
        return pts
            .first()
            .map_or([0.0, 0.0], |&[y, z]| [y as f32, z as f32]);
    }
    let scaled = t.clamp(0.0, 1.0) * (pts.len().saturating_sub(1)) as f64;
    let idx = (scaled.floor() as usize).min(pts.len().saturating_sub(2));
    let frac = scaled - idx as f64;
    let [y0, z0] = pts.get(idx).copied().unwrap_or([0.0, 0.0]);
    let [y1, z1] = pts.get(idx.saturating_add(1)).copied().unwrap_or([y0, z0]);
    [
        (y0 + (y1 - y0) * frac) as f32,
        (z0 + (z1 - z0) * frac) as f32,
    ]
}

type RibbonBuffers = (Vec<Vec3>, Vec<Vec3>, Vec<usize>, Vec<usize>);

fn build_ribbon(
    tris: &[SkinTri],
    cols: &[([f32; 2], [f32; 2])],
    side: f32,
) -> Option<RibbonBuffers> {
    let mut pts = Vec::new();
    let mut nrms = Vec::new();
    for &([uy, uz], [ly, lz]) in cols {
        if let (Some((up, un)), Some((lp, ln))) = (
            project_onto_skin(tris, uy, uz, side),
            project_onto_skin(tris, ly, lz, side),
        ) {
            pts.extend_from_slice(&[lp, up]);
            nrms.extend_from_slice(&[ln, un]);
        }
    }
    let ncols = pts.len() / 2;
    if ncols < 2 {
        return None;
    }
    let mut counts = Vec::with_capacity(ncols.saturating_sub(1));
    let mut indices = Vec::with_capacity(ncols.saturating_sub(1).saturating_mul(4));
    for i in 0..ncols.saturating_sub(1) {
        let (l0, u0) = (i.saturating_mul(2), i.saturating_mul(2).saturating_add(1));
        let (l1, u1) = (
            i.saturating_add(1).saturating_mul(2),
            i.saturating_add(1).saturating_mul(2).saturating_add(1),
        );
        counts.push(4);
        if side > 0.0 {
            indices.extend_from_slice(&[l0, u0, u1, l1]);
        } else {
            indices.extend_from_slice(&[l0, l1, u1, u0]);
        }
    }
    Some((pts, nrms, counts, indices))
}
