//! Airframe landmarks a photograph can be matched on, found on the P-38G mesh by
//! query rather than stored as coordinates, so a reshaped or moved part carries its
//! landmark with it.

use glam::DVec3;
use sim_p38g::{P38MeshPrim, p38g_mesh_prims};

/// Which half of the airframe a landmark lies on (`x` is right in mesh axes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Side {
    Left,
    Right,
    Any,
}

/// The extreme point of one prim along a direction.
struct LandmarkQuery {
    id: &'static str,
    /// Suffix of the prim's USD path.
    prim: &'static str,
    side: Side,
    /// Direction, mesh axes (`x` right, `y` forward, `z` up), the landmark is furthest along.
    toward: [f64; 3],
}

/// Vertices within this distance of the extreme are averaged into the landmark, so a
/// flat or rounded tip lands at its middle.
const EXTREME_BAND_M: f64 = 0.015;

const LANDMARKS: &[LandmarkQuery] = &[
    q(
        "nose_tip",
        "/Gondola/UpperOlive",
        Side::Any,
        [0.0, 1.0, 0.0],
    ),
    q(
        "gun_muzzle",
        "/Gondola/NoseArmament",
        Side::Any,
        [0.0, 1.0, 0.0],
    ),
    q(
        "canopy_top",
        "/Canopy/Glazing/Hatch",
        Side::Any,
        [0.0, 0.0, 1.0],
    ),
    q(
        "gondola_tail",
        "/Gondola/UpperOlive",
        Side::Any,
        [0.0, -1.0, 0.0],
    ),
    q(
        "spinner_l",
        "/PropLeft/SpinnerNose",
        Side::Left,
        [0.0, 1.0, 0.0],
    ),
    q(
        "spinner_r",
        "/PropRight/SpinnerNose",
        Side::Right,
        [0.0, 1.0, 0.0],
    ),
    q(
        "wingtip_l",
        "/Wing/UpperOlive",
        Side::Left,
        [-1.0, 0.0, 0.0],
    ),
    q(
        "wingtip_r",
        "/Wing/UpperOlive",
        Side::Right,
        [1.0, 0.0, 0.0],
    ),
    q(
        "fin_top_l",
        "/Empennage/VerticalFins",
        Side::Left,
        [0.0, 0.0, 1.0],
    ),
    q(
        "fin_top_r",
        "/Empennage/VerticalFins",
        Side::Right,
        [0.0, 0.0, 1.0],
    ),
    q(
        "fin_bottom_l",
        "/Empennage/VerticalFins",
        Side::Left,
        [0.0, 0.0, -1.0],
    ),
    q(
        "fin_bottom_r",
        "/Empennage/VerticalFins",
        Side::Right,
        [0.0, 0.0, -1.0],
    ),
    q(
        "rudder_tail_l",
        "/Empennage/RudderLeft",
        Side::Left,
        [0.0, -1.0, 0.0],
    ),
    q(
        "rudder_tail_r",
        "/Empennage/RudderRight",
        Side::Right,
        [0.0, -1.0, 0.0],
    ),
    q("wheel_l", "/Gear/WheelLeft", Side::Left, [0.0, 0.0, -1.0]),
    q("wheel_r", "/Gear/WheelRight", Side::Right, [0.0, 0.0, -1.0]),
    q("nose_wheel", "/Gear/NoseWheel", Side::Any, [0.0, 0.0, -1.0]),
];

const fn q(id: &'static str, prim: &'static str, side: Side, toward: [f64; 3]) -> LandmarkQuery {
    LandmarkQuery {
        id,
        prim,
        side,
        toward,
    }
}

/// Every landmark the current P-38G mesh yields, `(id, mesh-axis position in metres)`.
pub(super) fn p38g_landmarks() -> Vec<(&'static str, DVec3)> {
    let prims = p38g_mesh_prims();
    LANDMARKS
        .iter()
        .filter_map(|query| Some((query.id, find(&prims, query)?)))
        .collect()
}

fn find(prims: &[P38MeshPrim], query: &LandmarkQuery) -> Option<DVec3> {
    let points: Vec<DVec3> = prim_points(prims, query.prim)
        .into_iter()
        .filter(|p| match query.side {
            Side::Left => p.x < 0.0,
            Side::Right => p.x > 0.0,
            Side::Any => true,
        })
        .collect();
    extreme(&points, DVec3::from(query.toward))
}

/// Every vertex, mesh axes, of the prims whose USD path ends with `suffix`.
pub(super) fn prim_points(prims: &[P38MeshPrim], suffix: &str) -> Vec<DVec3> {
    prims
        .iter()
        .filter(|prim| prim.prim_path.ends_with(suffix))
        .flat_map(|prim| {
            prim.vertices.iter().map(|v| {
                prim.rest_to_aircraft
                    .transform_point3(DVec3::from(v.position.map(f64::from)))
            })
        })
        .collect()
}

/// The middle of the points furthest along `toward`.
pub(super) fn extreme(points: &[DVec3], toward: DVec3) -> Option<DVec3> {
    let reach = points
        .iter()
        .map(|p| p.dot(toward))
        .fold(f64::NEG_INFINITY, f64::max);
    let tip: Vec<&DVec3> = points
        .iter()
        .filter(|p| p.dot(toward) >= reach - EXTREME_BAND_M)
        .collect();
    let count = u32::try_from(tip.len()).ok().filter(|&n| n > 0)?;
    Some(tip.into_iter().sum::<DVec3>() / f64::from(count))
}
