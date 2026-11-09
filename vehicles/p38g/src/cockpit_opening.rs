//! Upper gondola skin with the cockpit cut out under the canopy, and the canopy
//! sill rails that close the gap between the skin edge and the glazing base.
//!
//! Without the opening the dorsal skin runs straight across the cockpit at sill
//! height and the seated pilot looks down onto olive drab instead of the panel.

use std::f32::consts::PI;

use glam::Vec3;

use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, fixed};
use crate::stations::{CANOPY_FRAME, GONDOLA_STATIONS, OLIVE_DRAB, gondola_point};

/// Windscreen base arch (`FS 74`): the skin ahead of it is closed.
pub(crate) const OPENING_FWD_Y: f32 = 1.68;
/// Canopy tail (`FS 175`): the skin aft of it is closed.
pub(crate) const OPENING_AFT_Y: f32 = -0.90;
/// Glazing base sits this far below the hull crown at every station.
pub(crate) const GLAZING_BASE_DROP_M: f32 = 0.05;

const ARC_STEPS: usize = 20;
const UPPER_HALF_ARC: f32 = 0.54 * PI;

/// The skin arc angle where the hull is as wide as the canopy, so the opening's
/// edge runs straight under the glazing base.
fn opening_edge_theta(half_w: f32, w_canopy: f32) -> f32 {
    let ratio = (w_canopy / half_w.max(1e-3)).clamp(0.0, 0.999);
    ratio.powf(1.0 / 0.84).asin().min(UPPER_HALF_ARC)
}

fn arc(y: f32, from: f32, to: f32) -> Vec<Vec3> {
    (0..=ARC_STEPS)
        .map(|i| gondola_point(y, from + (to - from) * i as f32 / ARC_STEPS as f32))
        .collect()
}

/// Upper olive skin: closed ahead of the windscreen and behind the canopy, two
/// side strips beside the cockpit, and the sill rails up to the glazing base.
pub(crate) fn upper_gondola_skin(prims: &mut Vec<P38MeshPrim>) {
    let full = |y: f32| arc(y, -UPPER_HALF_ARC, UPPER_HALF_ARC);
    let fwd: Vec<Vec<Vec3>> = GONDOLA_STATIONS
        .iter()
        .filter(|s| s.0 >= OPENING_FWD_Y)
        .map(|s| full(s.0))
        .collect();
    let aft: Vec<Vec<Vec3>> = GONDOLA_STATIONS
        .iter()
        .filter(|s| s.0 <= OPENING_AFT_Y)
        .map(|s| full(s.0))
        .collect();
    let cockpit: Vec<_> = GONDOLA_STATIONS
        .iter()
        .filter(|s| (OPENING_AFT_Y..=OPENING_FWD_Y).contains(&s.0))
        .collect();
    let mut skin = MeshBuilder::new();
    skin.loft_open(&fwd, Inside::Axis);
    skin.loft_open(&aft, Inside::Axis);
    let mut sills = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        let mut strip = Vec::with_capacity(cockpit.len());
        let mut wall = Vec::with_capacity(cockpit.len());
        for &&(y, _, half_w, z_top, _, _, w_canopy) in &cockpit {
            let edge = opening_edge_theta(half_w, w_canopy);
            let row = if side < 0.0 {
                arc(y, -UPPER_HALF_ARC, -edge)
            } else {
                arc(y, edge, UPPER_HALF_ARC)
            };
            let lip = gondola_point(y, side * edge);
            let base = Vec3::new(side * w_canopy, y, z_top - GLAZING_BASE_DROP_M);
            wall.push(vec![lip, base]);
            strip.push(row);
        }
        skin.loft_open(&strip, Inside::Point(Vec3::new(0.0, 0.4, 0.0)));
        sills.loft_open(&wall, Inside::Point(Vec3::new(0.0, 0.4, 0.2)));
    }
    prims.push(fixed(
        "/P38g/Gondola/UpperOlive",
        P38PartRole::CabinExterior,
        OLIVE_DRAB,
        skin,
    ));
    let mut rails = fixed(
        "/P38g/Gondola/CockpitSillRails",
        P38PartRole::CabinExterior,
        CANOPY_FRAME,
        sills,
    );
    rails.double_sided = true;
    prims.push(rails);
}

#[cfg(test)]
mod tests;
