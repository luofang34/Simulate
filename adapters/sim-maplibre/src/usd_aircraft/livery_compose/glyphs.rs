//! Block-letter registration glyph generation and `.usda` material/mesh serialization
//! for runtime-projected C-172 liveries.

use glam::Vec3;
use sim_pack::LiveryRegistrationSpec;

use super::{PhotoTransform, RibbonBuffers, SkinTri, project_onto_skin};

pub(super) fn write_registration_pair(
    out: &mut String,
    tris: &[SkinTri],
    reg: &LiveryRegistrationSpec,
    photo: Option<PhotoTransform>,
) {
    let Some(center) = resolve_reg_center(reg, photo) else {
        return;
    };
    let chars: Vec<char> = reg
        .text
        .chars()
        .filter(|c| c.is_ascii_alphanumeric() || *c == '-')
        .collect();
    if chars.is_empty() {
        return;
    }
    for (prim, side, forward_dir) in [
        ("LiveryRegistrationStarboard", 1.0_f32, -1.0_f32),
        ("LiveryRegistrationPort", -1.0_f32, 1.0_f32),
    ] {
        if let Some((pts, nrms, counts, indices)) =
            build_registration(tris, reg, center, &chars, side, forward_dir)
        {
            write_mesh_prim(out, prim, &reg.material, &pts, &nrms, &counts, &indices);
        }
    }
}

fn resolve_reg_center(
    reg: &LiveryRegistrationSpec,
    photo: Option<PhotoTransform>,
) -> Option<[f32; 2]> {
    if let Some([cy, cz]) = reg.center_yz {
        return Some([cy as f32, cz as f32]);
    }
    let ([u0, v0, u1, v1], xf) = (reg.box_px?, photo?);
    let [cy, cz] = xf.map_px(0.5 * (u0 + u1), 0.5 * (v0 + v1));
    Some([cy as f32, cz as f32])
}

fn build_registration(
    tris: &[SkinTri],
    reg: &LiveryRegistrationSpec,
    [cy, cz]: [f32; 2],
    chars: &[char],
    side: f32,
    dir: f32,
) -> Option<RibbonBuffers> {
    let h = reg.cap_height_m as f32;
    let w = reg.glyph_width_h as f32 * h;
    let gap = reg.gap_h as f32 * h;
    let stroke = (reg.stroke_h as f32).clamp(0.10, 0.30);
    let slant = reg.slant as f32;
    let total_w: f32 = chars
        .iter()
        .map(|&ch| if ch == '-' { 0.45 * w } else { w })
        .sum::<f32>()
        + gap * (chars.len().saturating_sub(1) as f32);
    let mut cursor = -0.5 * total_w;
    let (mut pts, mut nrms, mut counts, mut indices) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());

    for &ch in chars {
        let gw = if ch == '-' { 0.45 * w } else { w };
        for [u0, v0, u1, v1] in glyph_bars(ch, stroke) {
            let corners = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)];
            let mut quad_pts = Vec::with_capacity(4);
            let mut quad_nrms = Vec::with_capacity(4);
            for (u, v) in corners {
                let along = cursor + u * gw + (v - 0.5) * h * slant;
                if let Some((p, n)) =
                    project_onto_skin(tris, cy + dir * along, cz + (v - 0.5) * h, side)
                {
                    quad_pts.push(p);
                    quad_nrms.push(n);
                }
            }
            if quad_pts.len() == 4 {
                let base = pts.len();
                pts.extend(quad_pts);
                nrms.extend(quad_nrms);
                counts.push(4);
                let [b0, b1, b2, b3] = [
                    base,
                    base.saturating_add(1),
                    base.saturating_add(2),
                    base.saturating_add(3),
                ];
                if side * dir < 0.0 {
                    indices.extend_from_slice(&[b0, b3, b2, b1]);
                } else {
                    indices.extend_from_slice(&[b0, b1, b2, b3]);
                }
            }
        }
        cursor += gw + gap;
    }
    (!counts.is_empty()).then_some((pts, nrms, counts, indices))
}

fn glyph_bars(ch: char, s: f32) -> Vec<[f32; 4]> {
    let m = 0.5 - 0.5 * s;
    let (top, mid, bot) = (
        [0.0, 1.0 - s, 1.0, 1.0],
        [0.0, m, 1.0, m + s],
        [0.0, 0.0, 1.0, s],
    );
    let (ul, ll, ur, lr) = (
        [0.0, 0.5, s, 1.0],
        [0.0, 0.0, s, 0.5],
        [1.0 - s, 0.5, 1.0, 1.0],
        [1.0 - s, 0.0, 1.0, 0.5],
    );
    let ctr = [0.5 - 0.5 * s, 0.0, 0.5 + 0.5 * s, 1.0];
    match ch.to_ascii_uppercase() {
        '-' => vec![[0.05, m, 0.95, m + s]],
        '0' | 'O' | 'D' => vec![top, bot, ul, ll, ur, lr],
        '1' | 'I' => vec![ctr],
        '2' | 'Z' => vec![top, mid, bot, ur, ll],
        '3' => vec![top, mid, bot, ur, lr],
        '4' => vec![mid, ul, ur, lr],
        '5' | 'S' => vec![top, mid, bot, ul, lr],
        '6' => vec![top, mid, bot, ul, ll, lr],
        '7' => vec![top, ur, lr],
        '8' | 'B' => vec![top, mid, bot, ul, ll, ur, lr],
        '9' => vec![top, mid, bot, ul, ur, lr],
        'A' | 'R' => vec![top, mid, ul, ll, ur, lr],
        'C' => vec![top, bot, ul, ll],
        'E' => vec![top, mid, bot, ul, ll],
        'F' => vec![top, mid, ul, ll],
        'G' => vec![top, bot, ul, ll, lr, [0.5, m, 1.0, m + s]],
        'H' | 'K' | 'X' => vec![mid, ul, ll, ur, lr],
        'J' => vec![bot, ur, lr, [0.0, 0.0, s, 0.35]],
        'L' => vec![bot, ul, ll],
        'M' | 'N' => vec![
            top,
            ul,
            ll,
            ur,
            lr,
            [0.5 - 0.4 * s, 0.4, 0.5 + 0.4 * s, 1.0],
        ],
        'P' => vec![top, mid, ul, ll, ur],
        'Q' => vec![top, bot, ul, ll, ur, lr, [0.55, 0.0, 1.0, 0.35]],
        'T' => vec![top, ctr],
        'U' | 'V' => vec![bot, ul, ll, ur, lr],
        'W' => vec![
            bot,
            ul,
            ll,
            ur,
            lr,
            [0.5 - 0.4 * s, 0.0, 0.5 + 0.4 * s, 0.6],
        ],
        'Y' => vec![mid, ul, ur, [0.5 - 0.5 * s, 0.0, 0.5 + 0.5 * s, 0.55]],
        _ => vec![top, bot, ul, ll, ur, lr],
    }
}

pub(super) fn write_material(
    out: &mut String,
    name: &str,
    rgb: [f64; 3],
    metallic: f64,
    roughness: f64,
) {
    let ident = sanitize_ident(name);
    let [r, g, b] = rgb;
    out.push_str(&format!(
        "        def Material \"{ident}\"\n        {{\n            token outputs:surface.connect = </C172/Materials/{ident}/PreviewSurface.outputs:surface>\n            def Shader \"PreviewSurface\"\n            {{\n                uniform token info:id = \"UsdPreviewSurface\"\n                color3f inputs:diffuseColor = ({r:.4}, {g:.4}, {b:.4})\n                float inputs:metallic = {metallic:.3}\n                float inputs:roughness = {roughness:.3}\n                token outputs:surface\n            }}\n        }}\n"
    ));
}

pub(super) fn write_mesh_prim(
    out: &mut String,
    name: &str,
    material: &str,
    pts: &[Vec3],
    nrms: &[Vec3],
    counts: &[usize],
    indices: &[usize],
) {
    let mat = sanitize_ident(material);
    out.push_str(&format!(
        "        def Mesh \"{name}\" (\n            prepend apiSchemas = [\"MaterialBindingAPI\"]\n        )\n        {{\n            rel material:binding = </C172/Materials/{mat}>\n            uniform bool doubleSided = 0\n            int[] faceVertexCounts = ["
    ));
    for (i, c) in counts.iter().enumerate() {
        out.push_str(&format!("{}{c}", if i == 0 { "" } else { ", " }));
    }
    out.push_str("]\n            int[] faceVertexIndices = [");
    for (i, idx) in indices.iter().enumerate() {
        out.push_str(&format!("{}{idx}", if i == 0 { "" } else { ", " }));
    }
    out.push_str("]\n            point3f[] points = [");
    for (i, p) in pts.iter().enumerate() {
        out.push_str(&format!(
            "{}({:.4}, {:.4}, {:.4})",
            if i == 0 { "" } else { ", " },
            p.x,
            p.y,
            p.z
        ));
    }
    out.push_str("]\n            normal3f[] normals = [");
    for (i, n) in nrms.iter().enumerate() {
        out.push_str(&format!(
            "{}({:.4}, {:.4}, {:.4})",
            if i == 0 { "" } else { ", " },
            n.x,
            n.y,
            n.z
        ));
    }
    out.push_str("] (\n                interpolation = \"vertex\"\n            )\n            custom token sim:partRole = \"cabin_exterior\"\n        }\n");
}

pub(super) fn sanitize_ident(raw: &str) -> String {
    let mut s: String = raw
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect();
    if s.is_empty() || s.starts_with(|c: char| c.is_ascii_digit()) {
        s.insert(0, 'L');
    }
    s
}

pub(super) fn sanitize_doc(raw: &str) -> String {
    raw.chars()
        .filter(|c| *c != '"' && *c != '\\' && *c != '\n')
        .collect()
}
