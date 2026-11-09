//! Articulated P-38G-13-LO flight controls, Curtiss Electric full-feathering
//! propellers, flaps, ailerons, elevator and rudders (`T.O. 01-75F-1`); the landing gear
//! is built in [`crate::gear`].

use std::f32::consts::TAU;

use glam::{DMat4, DVec3, Vec3};

use crate::gear::{build_main_gear, build_nose_gear};
use crate::mesh::{Inside, MeshBuilder, P38Material, P38MeshPrim, P38PartRole, aerofoil, part};
use crate::stations::{
    AILERON_END_X, BOOM_X, ELEVATOR_HINGE_Y, FIN_PROFILE, HTAIL_Z, INSIGNIA_RED, OLIVE_DRAB,
    PROP_BLACK, PROP_BLADE_OFFSET_M, PROP_HUB_Y, PROP_HUB_Z, PROP_RADIUS_M, PROP_YELLOW,
    RUDDER_HINGE_Y, SPINNER_ALUMINIUM, wing_station,
};

pub(crate) fn articulated_prims() -> Vec<P38MeshPrim> {
    let mut prims = Vec::new();
    build_curtiss_propellers(&mut prims);
    build_propeller_discs(&mut prims);
    build_fowler_flaps(&mut prims);
    build_ailerons_and_trims(&mut prims);
    build_elevator_and_tabs(&mut prims);
    build_rudders_and_tabs(&mut prims);
    build_nose_gear(&mut prims);
    build_main_gear(&mut prims);
    prims
}

fn build_curtiss_propellers(prims: &mut Vec<P38MeshPrim>) {
    for (label, side) in [("PropLeft", -1.0_f32), ("PropRight", 1.0_f32)] {
        let hub_rest = DMat4::from_translation(DVec3::new(
            f64::from(side * BOOM_X),
            f64::from(PROP_HUB_Y),
            f64::from(PROP_HUB_Z),
        ));
        for (sub, mat, rings_spec) in [
            (
                "SpinnerBase",
                OLIVE_DRAB,
                &[(0.0_f32, 0.33_f32), (0.14, 0.31), (0.26, 0.27)][..],
            ),
            (
                "SpinnerNose",
                SPINNER_ALUMINIUM,
                &[
                    (0.26, 0.27),
                    (0.40, 0.22),
                    (0.53, 0.155),
                    (0.64, 0.08),
                    (0.71, 0.014),
                ][..],
            ),
        ] {
            let mut m = MeshBuilder::new();
            let rings: Vec<Vec<Vec3>> = rings_spec
                .iter()
                .map(|&(y, r)| circle_xz(y, r, 24))
                .collect();
            m.loft(&rings, true, Inside::Axis);
            prims.push(part(
                &format!("/P38g/{label}/{sub}"),
                P38PartRole::PropellerSpinner,
                mat,
                hub_rest,
                m,
            ));
        }
        for b in 0..3_u32 {
            let blade_rest =
                hub_rest * DMat4::from_rotation_y(f64::from(b) * (std::f64::consts::TAU / 3.0));
            let (body, tip) = build_curtiss_paddle_blade(-side);
            for (sub, mat, mesh) in [("Blade", PROP_BLACK, body), ("BladeTip", PROP_YELLOW, tip)] {
                let mut p = part(
                    &format!("/P38g/{label}/{sub}{b}"),
                    P38PartRole::PropellerBlades,
                    mat,
                    blade_rest,
                    mesh,
                );
                p.custom_hinge_axis = Some(DVec3::X);
                prims.push(p);
            }
        }
    }
}

fn build_propeller_discs(prims: &mut Vec<P38MeshPrim>) {
    let disc_mat = P38Material {
        diffuse_rgb: [0.16, 0.17, 0.18],
        emissive_rgb: [0.02, 0.02, 0.02],
        metallic: 0.15,
        roughness: 0.35,
        opacity: 0.22,
    };
    for (name, side) in [("PropDiscLeft", -1.0_f32), ("PropDiscRight", 1.0_f32)] {
        let mut disc = MeshBuilder::new();
        disc.disc_xz(PROP_RADIUS_M, 40);
        let rest = DMat4::from_translation(DVec3::new(
            f64::from(side * BOOM_X),
            f64::from(PROP_HUB_Y + PROP_BLADE_OFFSET_M),
            f64::from(PROP_HUB_Z),
        ));
        let mut p = part(
            &format!("/P38g/{name}"),
            P38PartRole::PropellerDisc,
            disc_mat,
            rest,
            disc,
        );
        p.double_sided = true;
        prims.push(p);
    }
}

fn circle_xz(y: f32, r: f32, segs: u32) -> Vec<Vec3> {
    (0..segs)
        .map(|i| {
            let (s, c) = (TAU * i as f32 / segs as f32).sin_cos();
            Vec3::new(c * r, y, s * r)
        })
        .collect()
}

fn build_curtiss_paddle_blade(pitch_hand: f32) -> (MeshBuilder, MeshBuilder) {
    let mut body = MeshBuilder::new();
    body.tube(
        Vec3::new(0.18, PROP_BLADE_OFFSET_M, 0.0),
        Vec3::new(0.38, PROP_BLADE_OFFSET_M, 0.0),
        0.048,
        0.056,
        12,
    );
    let body_rings: Vec<Vec<Vec3>> = [
        (0.36_f32, 0.13_f32, 0.030_f32, 34.0_f32),
        (0.62, 0.21, 0.024, 28.0),
        (0.95, 0.25, 0.018, 22.0),
        (1.28, 0.23, 0.013, 17.0),
        (1.56, 0.18, 0.009, 13.0),
    ]
    .into_iter()
    .map(|(r, c, t, deg)| blade_ring(r, c, t, deg.to_radians() * pitch_hand))
    .collect();
    body.loft(&body_rings, true, Inside::Axis);

    let mut tip = MeshBuilder::new();
    let tip_rings: Vec<Vec<Vec3>> = [
        (1.56_f32, 0.18_f32, 0.009_f32, 13.0_f32),
        (1.68, 0.13, 0.006, 11.0),
        (PROP_RADIUS_M, 0.06, 0.004, 9.5),
    ]
    .into_iter()
    .map(|(r, c, t, deg)| blade_ring(r, c, t, deg.to_radians() * pitch_hand))
    .collect();
    tip.loft(&tip_rings, true, Inside::Axis);
    (body, tip)
}

fn blade_ring(x_r: f32, chord: f32, half_t: f32, beta: f32) -> Vec<Vec3> {
    let (sb, cb) = beta.sin_cos();
    let chord_dir = Vec3::new(0.0, sb, cb);
    let thick_dir = Vec3::new(0.0, cb, -sb);
    let lead = Vec3::new(x_r, PROP_BLADE_OFFSET_M, 0.0) - chord_dir * (0.42 * chord);
    aerofoil(lead, chord_dir * chord, thick_dir * half_t)
}

fn build_fowler_flaps(prims: &mut Vec<P38MeshPrim>) {
    // AN 01-75F-2 plan view: flaps from the gondola to `WS 69.25` and from `WS 118.75` to
    // `WS 157.25`, `23.5 in` chord at the flap leading edge.
    const FLAP_CHORD_M: f32 = 23.5 * 0.0254;
    let (fl, fr) = (P38PartRole::FlapLeft, P38PartRole::FlapRight);
    let (inner, outer) = (
        (0.50_f32, 69.25 * 0.0254),
        (118.75 * 0.0254, 157.25 * 0.0254),
    );
    for (name, role, x0, x1) in [
        ("FlapInboardLeft", fl, -inner.1, -inner.0),
        ("FlapOutboardLeft", fl, -outer.1, -outer.0),
        ("FlapInboardRight", fr, inner.0, inner.1),
        ("FlapOutboardRight", fr, outer.0, outer.1),
    ] {
        let xm = 0.5 * (x0 + x1);
        let (_, ty_m, zm_m, ..) = wing_station(xm);
        let (hy, hz) = (ty_m + FLAP_CHORD_M, zm_m - 0.025);
        let mut mesh = MeshBuilder::new();
        let sections: Vec<Vec<Vec3>> = [x0, xm, x1]
            .into_iter()
            .map(|x| {
                let (_, ty, zm, ..) = wing_station(x);
                aerofoil(
                    Vec3::new(x - xm, 0.0, zm - hz),
                    Vec3::new(0.0, ty - hy, -0.015),
                    Vec3::new(0.0, 0.0, 0.038),
                )
            })
            .collect();
        mesh.loft(&sections, true, Inside::Axis);
        let rest = DMat4::from_translation(DVec3::new(f64::from(xm), f64::from(hy), f64::from(hz)));
        prims.push(part(
            &format!("/P38g/Wing/{name}"),
            role,
            OLIVE_DRAB,
            rest,
            mesh,
        ));
    }
}

/// Aileron hinge line: `70 %` chord (AN 01-75F-2 `HINGE ℄ AT 70% C.`).
fn aileron_hinge(x: f32) -> (f32, f32, f32) {
    let (lead, trail, zm, ..) = wing_station(x);
    let aft_chord = 0.30 * (lead - trail);
    (trail + aft_chord, zm, aft_chord)
}

fn build_ailerons_and_trims(prims: &mut Vec<P38MeshPrim>) {
    // AN 01-75F-2 plan view: `107 in` of aileron outboard of `WS 182`.
    const AILERON_START_X: f32 = 182.2 * 0.0254;
    for (name, trim_name, role, x0, x1) in [
        (
            "AileronLeft",
            "AileronTrimTabLeft",
            P38PartRole::AileronLeft,
            -AILERON_END_X,
            -AILERON_START_X,
        ),
        (
            "AileronRight",
            "AileronTrimTabRight",
            P38PartRole::AileronRight,
            AILERON_START_X,
            AILERON_END_X,
        ),
    ] {
        let xm = 0.5 * (x0 + x1);
        let (hy, hz, mid_chord) = aileron_hinge(xm);
        let (hy0, hz0, _) = aileron_hinge(x0);
        let (hy1, hz1, _) = aileron_hinge(x1);
        let axis = DVec3::new(
            f64::from(x1 - x0),
            f64::from(hy1 - hy0),
            f64::from(hz1 - hz0),
        )
        .normalize_or(DVec3::X);

        let mut mesh = MeshBuilder::new();
        let sections: Vec<Vec<Vec3>> = [x0, xm, x1]
            .into_iter()
            .map(|x| {
                let (y, z, chord) = aileron_hinge(x);
                aerofoil(
                    Vec3::new(x - xm, y - hy, z - hz),
                    Vec3::new(0.0, -chord, 0.0),
                    Vec3::new(0.0, 0.0, 0.032),
                )
            })
            .collect();
        mesh.loft(&sections, true, Inside::Axis);

        let rest = DMat4::from_translation(DVec3::new(f64::from(xm), f64::from(hy), f64::from(hz)));
        let mut prim = part(&format!("/P38g/Wing/{name}"), role, OLIVE_DRAB, rest, mesh);
        prim.custom_hinge_axis = Some(axis);
        prims.push(prim);

        let mut tab = MeshBuilder::new();
        tab.cuboid(
            Vec3::new(-0.24, -mid_chord, -0.012),
            Vec3::new(0.24, 0.10 - mid_chord, 0.012),
        );
        let mut tab_prim = part(
            &format!("/P38g/Wing/{trim_name}"),
            P38PartRole::CabinExterior,
            INSIGNIA_RED,
            rest,
            tab,
        );
        tab_prim.custom_hinge_axis = Some(axis);
        prims.push(tab_prim);
    }
}

fn build_elevator_and_tabs(prims: &mut Vec<P38MeshPrim>) {
    let elev_rest = DMat4::from_translation(DVec3::new(
        0.0,
        f64::from(ELEVATOR_HINGE_Y),
        f64::from(HTAIL_Z),
    ));
    let mut elev = MeshBuilder::new();
    let e_sections: Vec<Vec<Vec3>> = [-BOOM_X + 0.14, -1.10, 0.0, 1.10, BOOM_X - 0.14]
        .into_iter()
        .map(|x| {
            aerofoil(
                Vec3::new(x, 0.0, 0.0),
                Vec3::new(0.0, -0.50, 0.0),
                Vec3::new(0.0, 0.0, 0.044),
            )
        })
        .collect();
    elev.loft(&e_sections, true, Inside::Axis);
    prims.push(part(
        "/P38g/Empennage/Elevator",
        P38PartRole::Elevator,
        OLIVE_DRAB,
        elev_rest,
        elev,
    ));

    let mut elev_tabs = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        elev_tabs.cuboid(
            Vec3::new(side * 0.85 - 0.34, -0.53, -0.014),
            Vec3::new(side * 0.85 + 0.34, -0.44, 0.014),
        );
    }
    prims.push(part(
        "/P38g/Empennage/ElevatorTrimTab",
        P38PartRole::CabinExterior,
        INSIGNIA_RED,
        elev_rest,
        elev_tabs,
    ));
}

fn build_rudders_and_tabs(prims: &mut Vec<P38MeshPrim>) {
    for (name, side) in [("RudderLeft", -1.0_f32), ("RudderRight", 1.0_f32)] {
        let r_rest = DMat4::from_translation(DVec3::new(
            f64::from(side * BOOM_X),
            f64::from(RUDDER_HINGE_Y),
            0.40,
        ));
        let mut r_mesh = MeshBuilder::new();
        let r_sections: Vec<Vec<Vec3>> = FIN_PROFILE
            .iter()
            .map(|&(z, _ly, ty, half_t)| {
                let aft_c = (ty - RUDDER_HINGE_Y).min(-0.14);
                aerofoil(
                    Vec3::new(0.0, 0.0, z - 0.40),
                    Vec3::new(0.0, aft_c, 0.0),
                    Vec3::new(half_t * 0.85, 0.0, 0.0),
                )
            })
            .collect();
        r_mesh.loft(&r_sections, true, Inside::Axis);
        prims.push(part(
            &format!("/P38g/Empennage/{name}"),
            P38PartRole::Rudder,
            OLIVE_DRAB,
            r_rest,
            r_mesh,
        ));

        let mut r_tab = MeshBuilder::new();
        r_tab.cuboid(
            Vec3::new(-0.014, -1.06, -0.18),
            Vec3::new(0.014, -0.92, 0.22),
        );
        prims.push(part(
            &format!("/P38g/Empennage/{name}TrimTab"),
            P38PartRole::CabinExterior,
            INSIGNIA_RED,
            r_rest,
            r_tab,
        ));
    }
}
