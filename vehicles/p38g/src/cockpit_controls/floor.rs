//! On the floor: the rudder pedals sliding fore and aft with the rudder command, and
//! the fuel selector valves on their pedestal ahead of the seat.

use glam::Vec3;

use super::{CADMIUM, ENAMEL_BLACK, GRIP_WORN, control};
use crate::mesh::{MeshBuilder, P38ControlJoint, P38MeshPrim, P38PartRole, fixed};

/// Lateral position of each pedal's centre, m.
const PEDAL_X: f32 = 0.15;
/// Each pedal's centre at rest.
const PEDAL_Y: f32 = 1.10;
const PEDAL_Z: f32 = -0.10;
/// Pedal travel per unit of the rudder command, m, the right pedal forward for
/// right rudder. Assumed: no source gives the travel.
const PEDAL_PER_UNIT: f32 = 0.075;
/// Height of the pedal rails' centrelines, just above the floor.
const RAIL_Z: f32 = -0.205;
/// Backward lean of the pedal plates from vertical, rad.
const PEDAL_LEAN: f32 = 0.45;

/// The fuel selector valves' centres, on the floor ahead of the seat and left of the
/// column (P-38G photographs; which valve selects which tanks is not known).
const SELECTORS: [Vec3; 2] = [
    Vec3::new(-0.17, 0.80, PEDESTAL_TOP_Z),
    Vec3::new(-0.07, 0.83, PEDESTAL_TOP_Z),
];
/// Rubber fuel hose.
const HOSE_BLACK: crate::mesh::P38Material = crate::mesh::mat([0.04, 0.04, 0.04], 0.0, 0.7);
/// Top of the pedestal the valves stand on, as Z-4972 shows them raised off
/// the floor ahead of the seat and left of the column, with the fuel hoses
/// curving away beneath.
const PEDESTAL_TOP_Z: f32 = -0.08;
const SELECTOR_RADIUS: f32 = 0.048;

pub(super) fn prims() -> Vec<P38MeshPrim> {
    let mut prims = Vec::new();
    for side in [-1.0_f32, 1.0] {
        prims.extend(pedal(side));
    }
    prims.extend(fuel_selectors());
    prims
}

fn pedal(side: f32) -> Vec<P38MeshPrim> {
    let joint = [P38ControlJoint {
        channel: "yaw",
        origin_m: [0.0; 3],
        axis: [0.0, 1.0, 0.0],
        per_unit: side * PEDAL_PER_UNIT,
        rest: 0.0,
        slides: true,
    }];
    let x = side * PEDAL_X;
    let centre = Vec3::new(x, PEDAL_Y, PEDAL_Z);
    let up = Vec3::new(0.0, -PEDAL_LEAN.sin(), PEDAL_LEAN.cos());
    let face = Vec3::new(0.0, -PEDAL_LEAN.cos(), -PEDAL_LEAN.sin());
    // The carriage on its two floor rails, the uprights, the pedal's heel
    // plate and, hinged above it, the toe-brake plate.
    let mut frame = MeshBuilder::new();
    let foot = Vec3::new(x, centre.y + 0.07, RAIL_Z + 0.02);
    frame.cuboid(
        foot - Vec3::new(0.05, 0.035, 0.012),
        foot + Vec3::new(0.05, 0.035, 0.012),
    );
    for dx in [-0.045_f32, 0.045] {
        frame.tube(
            foot + Vec3::new(dx, 0.0, 0.0),
            centre - up * 0.05 + Vec3::new(dx, 0.0, 0.0),
            0.007,
            0.007,
            8,
        );
    }
    frame.slab(centre - up * 0.035, Vec3::X * 0.05, up * 0.05, face * 0.006);
    let toe = centre + up * 0.065 + face * 0.004;
    let toe_up = (up - face * 0.25).normalize();
    frame.slab(toe, Vec3::X * 0.05, toe_up * 0.04, face * 0.006);
    // The fore-and-aft adjusting lever on the pedal's outboard corner.
    let corner = centre - up * 0.07 + Vec3::X * side * 0.05;
    frame.tube(
        corner,
        corner + Vec3::new(side * 0.025, -0.02, 0.03),
        0.004,
        0.004,
        6,
    );
    // Rubber treads across both plates, scuffed lighter where the boot rests.
    let mut treads = MeshBuilder::new();
    for k in -1..=1 {
        let at = centre - up * (0.035 + k as f32 * 0.025) - face * 0.007;
        treads.slab(at, Vec3::X * 0.046, up * 0.005, face * 0.002);
    }
    for k in 0..2 {
        let at = toe + toe_up * (k as f32 * 0.03 - 0.015) - face * 0.007;
        treads.slab(at, Vec3::X * 0.046, toe_up * 0.005, face * 0.002);
    }
    let name = if side < 0.0 { "Left" } else { "Right" };
    let path = format!("/P38g/CockpitInterior/RudderPedal{name}");
    let mut rails = MeshBuilder::new();
    for dx in [-0.03_f32, 0.03] {
        rails.tube(
            Vec3::new(x + dx, PEDAL_Y - 0.06, RAIL_Z),
            Vec3::new(x + dx, PEDAL_Y + 0.26, RAIL_Z),
            0.008,
            0.008,
            8,
        );
    }
    vec![
        control(&path, ENAMEL_BLACK, frame, &joint),
        control(&format!("{path}/Treads"), GRIP_WORN, treads, &joint),
        fixed(
            &format!("{path}/Rails"),
            P38PartRole::CabinInterior,
            ENAMEL_BLACK,
            rails,
        ),
    ]
}

/// The selectors have no tank-selection state in the simulation to follow, so they
/// stand fixed, each pointer fore and aft.
fn fuel_selectors() -> Vec<P38MeshPrim> {
    let mut bodies = MeshBuilder::new();
    let mut pointers = MeshBuilder::new();
    let floor = -0.22;
    bodies.cuboid(
        Vec3::new(-0.23, 0.74, floor),
        Vec3::new(-0.01, 0.89, PEDESTAL_TOP_Z - 0.02),
    );
    let mut hoses = MeshBuilder::new();
    for c in SELECTORS {
        let out = Vec3::new(c.x, 0.89, -0.13);
        super::polyline(
            &mut hoses,
            &[
                out,
                out + Vec3::new(0.0, 0.06, -0.03),
                out + Vec3::new(0.02, 0.14, -0.08),
            ],
            0.012,
            10,
        );
    }
    for c in SELECTORS {
        bodies.tube(c - Vec3::Z * 0.02, c, SELECTOR_RADIUS, SELECTOR_RADIUS, 20);
        pointers.tube(
            c + Vec3::new(0.0, -0.038, 0.012),
            c + Vec3::new(0.0, 0.038, 0.012),
            0.008,
            0.004,
            8,
        );
        pointers.ellipsoid(c + Vec3::Z * 0.012, Vec3::new(0.014, 0.014, 0.012), 10, 6);
    }
    vec![
        fixed(
            "/P38g/CockpitInterior/FuelSelectors",
            P38PartRole::CabinInterior,
            ENAMEL_BLACK,
            bodies,
        ),
        fixed(
            "/P38g/CockpitInterior/FuelSelectors/Hoses",
            P38PartRole::CabinInterior,
            HOSE_BLACK,
            hoses,
        ),
        fixed(
            "/P38g/CockpitInterior/FuelSelectors/Pointers",
            P38PartRole::CabinInterior,
            CADMIUM,
            pointers,
        ),
    ]
}
