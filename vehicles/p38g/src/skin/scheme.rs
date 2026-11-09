//! The factory finish: Olive Drab 41 on upper and side surfaces over
//! Neutral Gray 43 below.

use glam::Vec3;

use crate::mesh::P38Material;
use crate::stations::{NEUTRAL_GRAY, OLIVE_DRAB};

/// Which coat a point of the skin wears.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Coat {
    Olive,
    Gray,
}

fn rgb(material: P38Material) -> Vec3 {
    Vec3::from(material.diffuse_rgb)
}

/// The base colour and roughness of `coat`, linear.
pub(crate) fn base(coat: Coat) -> (Vec3, f32) {
    match coat {
        Coat::Olive => (rgb(OLIVE_DRAB), OLIVE_DRAB.roughness),
        Coat::Gray => (rgb(NEUTRAL_GRAY), NEUTRAL_GRAY.roughness),
    }
}

/// Below this outward normal `z` a surface that is not split into upper
/// and lower parts (control surfaces, fillets, tail) faces down and wears
/// gray: the demarcation follows each surface's own leading and trailing
/// edges, as on the wing.
const UNDERSIDE_NZ: f32 = -0.25;

/// The coat at a point of part `path` with outward normal `normal`.
pub(crate) fn coat(path: &str, normal: Vec3) -> Coat {
    if path.ends_with("LowerGray") {
        Coat::Gray
    } else if path.ends_with("UpperOlive")
        || path.ends_with("NoseUpperDeck")
        || path.ends_with("VerticalFins")
        || path.contains("Rudder")
        || path.contains("/Spinner")
    {
        Coat::Olive
    } else if normal.z < UNDERSIDE_NZ {
        Coat::Gray
    } else {
        Coat::Olive
    }
}
