//! Glycol radiator scoops on both sides of each tail boom, with their exit flaps, from
//! the AN 01-75F-2 side view (a box from `FS 265` to `FS 314`, `0.5..21.5 in` above the
//! fuselage reference line, exit flaps from `FS 298`) and plan view (proud of the boom
//! to `19.6 in` off its centreline at `FS 280`).

use std::f32::consts::FRAC_PI_2;

use glam::{DMat4, DVec3, Vec3};

use crate::drawing::{INCH_M, frl_to_z, fs_to_y};
use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, fixed, part};
use crate::stations::{BOOM_X, NEUTRAL_GRAY, OLIVE_DRAB, PANEL_BLACK, interpolate_boom};

/// Outer face of a scoop: `(FS, inches off the boom centreline)`, from the plan view.
const SCOOP_PLAN: [(f32, f32); 7] = [
    (265.0, 17.8),
    (270.0, 19.0),
    (280.0, 19.6),
    (290.0, 19.2),
    (300.0, 17.0),
    (308.0, 14.6),
    (314.0, 11.8),
];
/// Scoop top and bottom above the fuselage reference line (side view).
const SCOOP_TOP_IN: f32 = 21.5;
const SCOOP_BOTTOM_IN: f32 = 0.5;
/// Radius of the scoop's outer corners.
const CORNER_IN: f32 = 3.0;
/// Exit flap hinge station and length (the inner rectangle on the side view).
const FLAP_HINGE_FS: f32 = 298.0;
const FLAP_LENGTH_IN: f32 = 16.0;

pub(crate) fn radiator_prims() -> Vec<P38MeshPrim> {
    let mut olive = MeshBuilder::new();
    let mut gray = MeshBuilder::new();
    let mut cores = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        for face in [-1.0_f32, 1.0] {
            build_scoop(side * BOOM_X, face, [&mut olive, &mut gray, &mut cores]);
        }
    }
    let mut prims = vec![
        fixed(
            "/P38g/Powerplant/RadiatorScoops",
            P38PartRole::CabinExterior,
            OLIVE_DRAB,
            olive,
        ),
        fixed(
            "/P38g/Powerplant/RadiatorScoopsUnderside",
            P38PartRole::CabinExterior,
            NEUTRAL_GRAY,
            gray,
        ),
        fixed(
            "/P38g/Powerplant/RadiatorCores",
            P38PartRole::CabinExterior,
            PANEL_BLACK,
            cores,
        ),
    ];
    build_exit_flaps(&mut prims);
    prims
}

/// Outer-face offset at `fs`, inches off the boom centreline.
fn outer_in(fs: f32) -> f32 {
    let Some(&first) = SCOOP_PLAN.first() else {
        return 0.0;
    };
    let mut prev = first;
    for &(f, o) in &SCOOP_PLAN {
        if fs <= f {
            let t = ((fs - prev.0) / (f - prev.0).max(1e-3)).clamp(0.0, 1.0);
            return prev.1 + (o - prev.1) * t;
        }
        prev = (f, o);
    }
    prev.1
}

/// One scoop section at `fs` on face `face` (`-1` towards `-x`, `+1` towards `+x`) of the
/// boom at `bx`: the upper half (olive) and lower half (gray), each from the boom to the
/// outer face's mid-height. The inboard edge is buried in the boom skin.
fn section(bx: f32, face: f32, fs: f32) -> (Vec<Vec3>, Vec<Vec3>) {
    let y = fs_to_y(fs);
    let core = interpolate_boom(y).1 / INCH_M;
    let outer = outer_in(fs).max(core + 0.5);
    let r = CORNER_IN.min(0.5 * (outer - 0.7 * core));
    let at = |off: f32, h: f32| Vec3::new(bx + face * off * INCH_M, y, frl_to_z(h));
    let mid = 0.5 * (SCOOP_TOP_IN + SCOOP_BOTTOM_IN);
    let arc = |h_centre: f32, from: f32| -> Vec<Vec3> {
        (0..=4)
            .map(|i| {
                let a = from + FRAC_PI_2 * (i as f32 / 4.0);
                at(outer - r + r * a.cos(), h_centre + r * a.sin())
            })
            .collect()
    };
    let mut upper = vec![at(0.7 * core, SCOOP_TOP_IN)];
    upper.extend(arc(SCOOP_TOP_IN - r, 0.0).into_iter().rev());
    upper.push(at(outer, mid));
    let mut lower = vec![at(outer, mid)];
    lower.extend(arc(SCOOP_BOTTOM_IN + r, -FRAC_PI_2).into_iter().rev());
    lower.push(at(0.7 * core, SCOOP_BOTTOM_IN));
    (upper, lower)
}

fn build_scoop(bx: f32, face: f32, [olive, gray, cores]: [&mut MeshBuilder; 3]) {
    let stations: Vec<f32> = SCOOP_PLAN.iter().map(|&(fs, _)| fs).collect();
    let (upper, lower): (Vec<Vec<Vec3>>, Vec<Vec<Vec3>>) =
        stations.iter().map(|&fs| section(bx, face, fs)).unzip();
    let axis = Vec3::new(bx, fs_to_y(290.0), frl_to_z(10.0));
    olive.loft_open(&upper, Inside::Point(axis));
    gray.loft_open(&lower, Inside::Point(axis));
    // The radiator matrix seen through the open intake, just inside the lip.
    let (up, lo) = section(bx, face, 266.5);
    let face_ring: Vec<Vec3> = up.into_iter().chain(lo.into_iter().skip(1)).collect();
    cores.polygon(&face_ring, Vec3::Y);
}

fn build_exit_flaps(prims: &mut Vec<P38MeshPrim>) {
    let mid = 0.5 * (SCOOP_TOP_IN + SCOOP_BOTTOM_IN);
    let half_height = 0.5 * (SCOOP_TOP_IN - SCOOP_BOTTOM_IN - 2.0 * CORNER_IN) * INCH_M;
    let hinge_out = outer_in(FLAP_HINGE_FS);
    // Closed, a flap lies along the outer face as it converges on the boom.
    let toe = (hinge_out - outer_in(FLAP_HINGE_FS + FLAP_LENGTH_IN)).atan2(FLAP_LENGTH_IN);
    for (label, side) in [("Left", -1.0_f32), ("Right", 1.0_f32)] {
        let bx = side * BOOM_X;
        for (sub, face) in [("In", -1.0_f32), ("Out", 1.0_f32)] {
            let mut flap = MeshBuilder::new();
            flap.cuboid(
                Vec3::new(-0.010, -FLAP_LENGTH_IN * INCH_M, -half_height),
                Vec3::new(0.010, 0.0, half_height),
            );
            let rest = DMat4::from_translation(DVec3::new(
                f64::from(bx + face * hinge_out * INCH_M),
                f64::from(fs_to_y(FLAP_HINGE_FS)),
                f64::from(frl_to_z(mid)),
            )) * DMat4::from_rotation_z(f64::from(-face * toe));
            let mut prim = part(
                &format!("/P38g/Cooling/CoolantFlap{label}{sub}"),
                P38PartRole::CabinExterior,
                OLIVE_DRAB,
                rest,
                flap,
            );
            // Opening swings the trailing edge away from the boom.
            prim.custom_hinge_axis = Some(DVec3::new(0.0, 0.0, f64::from(face)));
            prims.push(prim);
        }
    }
}
