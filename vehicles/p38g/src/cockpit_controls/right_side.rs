//! The right side: the flap lever in its four-detent gate on a wall bracket, the aileron
//! tab control below the panel by the column, and the rudder tab control low and
//! forward on the floor.

use glam::Vec3;

use super::left_console::CONSOLE_TOP_Z;
use super::{CADMIUM, ENAMEL_BLACK, GRIP_BLACK, WORN_STEEL, control, turn};
use crate::mesh::{MeshBuilder, P38MeshPrim, P38PartRole, fixed};

/// The flap lever's pivot on the right console (01-75FF-1 figure 7 item 2).
const FLAP_PIVOT: Vec3 = Vec3::new(0.355, 0.80, CONSOLE_TOP_Z + 0.02);
/// Swing between neighbouring detents, `UP`, `CLOSED`, `MANEUVER`, `DOWN` from aft
/// to forward, rad. The order is the placard's; the direction and spacing are
/// assumed (no source found).
const FLAP_DETENT: f32 = -0.30;

/// The aileron tab control, on the right wall below the panel (figure 7 item 1).
const AILERON_TAB: Vec3 = Vec3::new(0.432, 1.02, 0.10);
/// The rudder tab control, low and forward on the right of the floor (figure 7
/// item 17).
const RUDDER_TAB: Vec3 = Vec3::new(0.30, 1.14, -0.207);
/// Tab control turn per unit of trim, rad. Assumed: three-eighths of a turn either way.
const TAB_KNOB_PER_UNIT: f32 = 2.36;

pub(super) fn prims() -> Vec<P38MeshPrim> {
    let mut prims = flap_lever();
    prims.extend(tab_knob("trim_aileron", AILERON_TAB, Vec3::X, "AileronTab"));
    prims.extend(tab_knob(
        "trim_rudder",
        RUDDER_TAB,
        Vec3::NEG_Z,
        "RudderTab",
    ));
    prims
}

fn flap_lever() -> Vec<P38MeshPrim> {
    let joint = [turn("flap_selector", FLAP_PIVOT, Vec3::X, FLAP_DETENT, 1.0)];
    let tip = FLAP_PIVOT + Vec3::new(0.0, 0.0, 0.12);
    let mut arm = MeshBuilder::new();
    arm.tube(FLAP_PIVOT, tip, 0.006, 0.005, 8);
    let mut knob = MeshBuilder::new();
    knob.ellipsoid(
        tip + Vec3::new(0.0, 0.0, 0.01),
        Vec3::new(0.014, 0.014, 0.02),
        12,
        8,
    );
    // The trigger under the knob that lifts through the notch to DOWN.
    knob.cuboid(
        tip - Vec3::new(0.004, 0.018, 0.03),
        tip + Vec3::new(0.004, -0.006, -0.012),
    );
    let mut wear = MeshBuilder::new();
    wear.tube(tip - Vec3::new(0.0, 0.0, 0.03), tip, 0.0062, 0.0055, 8);
    // The gate: a slotted plate the lever stands in, its detents a step apart.
    let mut gate = MeshBuilder::new();
    let (y0, y1) = (FLAP_PIVOT.y - 0.07, FLAP_PIVOT.y + 0.09);
    for x in [FLAP_PIVOT.x - 0.022, FLAP_PIVOT.x + 0.008] {
        gate.cuboid(
            Vec3::new(x, y0, CONSOLE_TOP_Z),
            Vec3::new(x + 0.014, y1, CONSOLE_TOP_Z + 0.04),
        );
    }
    // The gate's bracket off the wall: the right side has no console box.
    gate.cuboid(
        Vec3::new(FLAP_PIVOT.x - 0.03, y0, CONSOLE_TOP_Z - 0.05),
        Vec3::new(0.44, y1, CONSOLE_TOP_Z),
    );
    vec![
        control("/P38g/CockpitInterior/FlapLever", ENAMEL_BLACK, arm, &joint),
        control(
            "/P38g/CockpitInterior/FlapLever/Knob",
            GRIP_BLACK,
            knob,
            &joint,
        ),
        control(
            "/P38g/CockpitInterior/FlapLever/Wear",
            WORN_STEEL,
            wear,
            &joint,
        ),
        fixed(
            "/P38g/CockpitInterior/FlapGate",
            P38PartRole::CabinInterior,
            ENAMEL_BLACK,
            gate,
        ),
    ]
}

/// A tab control: a knurled knob with a pointer bar, turning about `axis`, which
/// points into the surface it is mounted on, in its cadmium bezel.
fn tab_knob(channel: &'static str, centre: Vec3, axis: Vec3, name: &str) -> Vec<P38MeshPrim> {
    let joint = [turn(channel, centre, axis, TAB_KNOB_PER_UNIT, 0.0)];
    let mut knob = MeshBuilder::new();
    knob.tube(centre, centre - axis * 0.022, 0.024, 0.022, 16);
    let across = if axis.z.abs() > 0.5 { Vec3::Y } else { Vec3::Z };
    let mut pointer = MeshBuilder::new();
    pointer.tube(
        centre - axis * 0.024 - across * 0.03,
        centre - axis * 0.024 + across * 0.03,
        0.006,
        0.006,
        8,
    );
    let mut bezel = MeshBuilder::new();
    bezel.tube(centre + axis * 0.012, centre, 0.034, 0.034, 18);
    let path = format!("/P38g/CockpitInterior/{name}");
    vec![
        control(&path, GRIP_BLACK, knob, &joint),
        control(&format!("{path}/Pointer"), ENAMEL_BLACK, pointer, &joint),
        fixed(
            &format!("{path}/Bezel"),
            P38PartRole::CabinInterior,
            CADMIUM,
            bezel,
        ),
    ]
}
