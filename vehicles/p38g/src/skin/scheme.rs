//! The factory finish: Olive Drab 41 on upper and side surfaces over
//! Neutral Gray 43 below (Spec 24114, which carries the gray up the sides
//! only so far as it is not seen from more than 30° above them, as quoted
//! by secondary sources). The demarcation follows the split the
//! airframe's geometry already makes between its upper and lower skins and
//! is hard-edged; no source for the P-38's line or its edge was found, so
//! neither is claimed. The colours are the airframe palette's; olive drab
//! is kept a satin finish rather than flat, as sunlit photos show it.

use glam::Vec3;

use crate::mesh::P38Material;
use crate::stations::{NEUTRAL_GRAY, OLIVE_DRAB};

/// Olive drab's roughness before wear: satin, catching the sun's highlight
/// on upper surfaces in the photo matches (p38-views, 42-12842 and
/// 42-67079) where the palette's flatter value read dark.
const SATIN_OLIVE: f32 = 0.55;

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
        Coat::Olive => (rgb(OLIVE_DRAB), SATIN_OLIVE),
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
    if path.ends_with("LowerGray") || path.ends_with("Underside") || path.contains("ChinOilCooler")
    {
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
