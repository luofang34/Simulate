//! Wear of a 339th FS P-38G after months on Guadalcanal's fields in 1943,
//! as the period photos show it (USMC 127-GR-8-72001, 43-2239 at Fighter 2;
//! USAAF 342-FH-3A43494): olive drab sun-faded and chalky on top and
//! mottled by the season's touch-ups, turbo soot down the boom tops, oil
//! along the nacelle bellies, and paint worn through to aluminium where
//! crews walked and the slipstream eroded.
//!
//! Every mask comes from where a point is on the airframe and which way it
//! faces, measured from the meshes, so the wear follows the geometry.

use glam::Vec3;

use super::anchors::Anchors;
use super::noise;
use super::scheme::Coat;

/// Bare 24ST clad aluminium where the paint is gone.
const ALUMINIUM: Vec3 = Vec3::new(0.55, 0.56, 0.57);
/// Turbo exhaust soot.
const SOOT: Vec3 = Vec3::new(0.025, 0.024, 0.022);
/// Distance aft of a turbo over which its soot thins to a third: the stain on
/// 339th FS booms runs most of the way to the radiators (USAAF photos of
/// 43-2239 and 43-2242, Fighter Two, 1943).
const SOOT_REACH_M: f32 = 3.2;
/// Engine oil and the dust it gathers.
const OIL: Vec3 = Vec3::new(0.05, 0.04, 0.025);

/// The worn finish at a point.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Worn {
    pub(crate) albedo: Vec3,
    pub(crate) roughness: f32,
    pub(crate) metallic: f32,
}

/// What wear acts on: the point, its paint, and how exposed it is.
pub(crate) struct Exposure {
    pub(crate) position: Vec3,
    pub(crate) normal: Vec3,
    /// Vertex-baked open sky, `1` fully open.
    pub(crate) sky: f32,
    pub(crate) coat: Coat,
    /// Distance to the nearest panel seam, inches: edges chip first.
    pub(crate) seam_in: f32,
    /// Whether the point is on the wing, where crews walk.
    pub(crate) on_wing: bool,
}

/// Wears `albedo` and `roughness` at `at`.
pub(crate) fn wear(anchors: &Anchors, at: &Exposure, albedo: Vec3, roughness: f32) -> Worn {
    let p = at.position;
    let mut worn = Worn {
        albedo,
        roughness,
        metallic: 0.0,
    };
    fade(&mut worn, at);
    grime(&mut worn, at);
    soot(&mut worn, anchors, p, at.normal);
    oil(&mut worn, anchors, p, at.normal);
    cowl_streaks(&mut worn, anchors, p, at.normal);
    chips(&mut worn, anchors, at);
    worn
}

/// Sun fading: olive drab on top turns lighter, greyer and browner, in
/// blotches; neutral gray below barely changes.
fn fade(worn: &mut Worn, at: &Exposure) {
    let up = at.normal.z.clamp(0.0, 1.0);
    let sun = (0.25 + 0.75 * up) * at.sky.clamp(0.0, 1.0);
    // Broad, gentle unevenness and a fine chalky grain, low in contrast.
    let broad = noise::fractal(at.position, 1.2);
    let fine = noise::fractal(at.position, 45.0);
    let strength = match at.coat {
        Coat::Olive => 0.9,
        Coat::Gray => 0.25,
    };
    let fade = (strength * sun * (0.45 + 0.3 * broad + 0.25 * fine)).clamp(0.0, 1.0);
    let luma = worn.albedo.dot(Vec3::new(0.2126, 0.7152, 0.0722));
    let chalked = Vec3::splat(luma) * Vec3::new(1.12, 1.04, 0.86) * 1.75 + Vec3::splat(0.012);
    worn.albedo = worn.albedo.lerp(chalked, fade * 0.9);
    worn.roughness = (worn.roughness + 0.18 * fade).min(0.95);
}

/// Dust and dirt settle where the sky and the rain do not reach.
fn grime(worn: &mut Worn, at: &Exposure) {
    let hidden = (1.0 - at.sky).clamp(0.0, 1.0);
    let streak = noise::value(at.position * Vec3::new(9.0, 0.6, 9.0));
    worn.albedo *= 1.0 - 0.32 * hidden * (0.6 + 0.4 * streak);
}

/// Turbosupercharger exhaust: from each wheel's buckets the gas streams aft
/// over the boom top, spreading and thinning.
fn soot(worn: &mut Worn, anchors: &Anchors, p: Vec3, n: Vec3) {
    for turbo in anchors.turbo {
        if turbo == Vec3::ZERO || p.x * turbo.x <= 0.0 {
            continue;
        }
        let aft = turbo.y - p.y;
        if aft < -0.05 {
            continue;
        }
        let spread = 0.16 + 0.09 * aft.max(0.0);
        let lateral = (p.x - turbo.x).abs();
        let above = (p.z - (turbo.z - 0.25)).max(0.0);
        let streaks = 0.55 + 0.45 * noise::value(Vec3::new(p.x * 22.0, p.y * 0.8, 3.0));
        let deposit = (-aft.max(0.0) / SOOT_REACH_M).exp()
            * smooth(spread, spread * 0.4, lateral)
            * smooth(0.0, 0.12, above)
            * smooth(-0.3, 0.3, n.z)
            * streaks;
        let d = (deposit * 1.6).clamp(0.0, 0.97);
        worn.albedo = worn.albedo.lerp(SOOT, d);
        worn.roughness += (0.9 - worn.roughness) * d;
    }
}

/// Oil seeping from the cowl seams and the oil cooler runs aft along the
/// nacelle bellies in thin streaks.
fn oil(worn: &mut Worn, anchors: &Anchors, p: Vec3, n: Vec3) {
    let lateral = (p.x.abs() - anchors.boom_x).abs();
    if lateral > 0.45 || n.z > -0.2 {
        return;
    }
    let below = (-n.z).clamp(0.0, 1.0);
    let lines = noise::value(Vec3::new(p.x * 30.0, p.y * 0.35, 7.0));
    let runs = smooth(0.55, 0.85, lines) * below * smooth(0.45, 0.15, lateral);
    // Strongest just aft of the engine, fading toward the tail.
    let along = smooth(2.6, 1.6, p.y) * smooth(-4.5, -0.5, p.y);
    let d = (runs * along * 0.7).clamp(0.0, 1.0);
    worn.albedo = worn.albedo.lerp(OIL, d);
    worn.roughness += (0.25 - worn.roughness) * d * 0.8;
}

/// Oil, exhaust and hand grime down the nacelles' sides from the cowl
/// seams and exhaust shrouds, running aft and down in soft streaks: the
/// blotched cowls of 43-2239 (USAAF photo, Fighter Two, 1943).
fn cowl_streaks(worn: &mut Worn, anchors: &Anchors, p: Vec3, n: Vec3) {
    let lateral = (p.x.abs() - anchors.boom_x).abs();
    if lateral > 0.7 || n.z.abs() > 0.75 {
        return;
    }
    // From just aft of the spinner to the wing's trailing edge.
    let along = smooth(anchors.nose_y - 0.2, anchors.nose_y - 0.9, p.y) * smooth(-2.2, -0.6, p.y);
    // The streak pattern is sheared so each run falls as it trails aft.
    let lines = noise::value(Vec3::new((p.y + 1.4 * p.z) * 6.0, p.x.signum() * 3.0, 11.0));
    let blotch = noise::fractal(p, 2.2);
    let d = (along * (0.7 * smooth(0.4, 0.75, lines) + 0.5 * smooth(0.45, 0.7, blotch)))
        .clamp(0.0, 0.85);
    worn.albedo = worn.albedo.lerp(OIL, d);
    worn.roughness += (0.4 - worn.roughness) * d * 0.6;
}

/// Paint worn through to aluminium: the inner wing walkways either side of
/// the gondola, and the leading edges the slipstream sandblasts.
fn chips(worn: &mut Worn, anchors: &Anchors, at: &Exposure) {
    let p = at.position;
    let walkway = if at.on_wing
        && at.normal.z > 0.6
        && p.x.abs() < anchors.boom_x - 0.5
        && p.x.abs() > 0.55
    {
        anchors.wing_chord(p.x).map_or(0.0, |(lead, trail)| {
            let from_trail = (p.y - trail) / (lead - trail).max(0.1);
            smooth(0.75, 0.2, from_trail) * smooth(1.6, 0.8, p.x.abs())
        })
    } else {
        0.0
    };
    let leading = smooth(0.2, 0.75, at.normal.y) * smooth(-0.2, 0.3, at.normal.z + 0.3);
    let edge = smooth(1.2, 0.2, at.seam_in) * at.normal.z.clamp(0.0, 1.0);
    let grain = noise::fractal(p, 34.0);
    let exposure = (walkway * 0.8 + leading * 0.3 + edge * 0.35).clamp(0.0, 1.0);
    let chipped = smooth(1.0 - exposure * 0.45, 1.0 - exposure * 0.45 + 0.03, grain);
    if chipped > 0.0 {
        worn.albedo = worn.albedo.lerp(ALUMINIUM, chipped);
        worn.roughness += (0.38 - worn.roughness) * chipped;
        worn.metallic = chipped;
    }
    // Boots and knees scuff the walkway paint lighter before it wears through.
    worn.albedo *= 1.0 + 0.25 * walkway * noise::value(p * 7.0);
}

/// Hermite step from `edge0` to `edge1` (either order).
pub(crate) fn smooth(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
