//! Articulated P-38G-13-LO flight controls, Curtiss Electric full-feathering
//! propellers (`BS 0..26`), cooling shutters, and tricycle landing gear (`T.O. 01-75F-1`).

use std::f32::consts::TAU;

use glam::{DMat4, DVec3, Vec3};

use crate::mesh::{
    Inside, MeshBuilder, P38Material, P38MeshPrim, P38PartRole, aerofoil, fixed, part,
};
use crate::stations::{
    AILERON_END_X, BOOM_X, CHROME_OLEO, ELEVATOR_HINGE_Y, FIN_PROFILE, HTAIL_Z, INSIGNIA_RED,
    NEUTRAL_GRAY, OLIVE_DRAB, PROP_BLACK, PROP_HUB_Y, PROP_HUB_Z, PROP_RADIUS_M, PROP_YELLOW,
    RUDDER_HINGE_Y, SPINNER_WHITE, TIRE_RUBBER, WHEEL_WELL_ZINC, wing_station,
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
                &[(0.0_f32, 0.33_f32), (0.12, 0.31), (0.22, 0.28)][..],
            ),
            (
                "SpinnerNose",
                SPINNER_WHITE,
                &[
                    (0.22, 0.28),
                    (0.36, 0.23),
                    (0.49, 0.16),
                    (0.59, 0.08),
                    (0.66, 0.014),
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
            f64::from(PROP_HUB_Y + 0.11),
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
        Vec3::new(0.18, 0.11, 0.0),
        Vec3::new(0.38, 0.11, 0.0),
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
    let lead = Vec3::new(x_r, 0.11, 0.0) - chord_dir * (0.42 * chord);
    aerofoil(lead, chord_dir * chord, thick_dir * half_t)
}

fn build_fowler_flaps(prims: &mut Vec<P38MeshPrim>) {
    let (fl, fr) = (P38PartRole::FlapLeft, P38PartRole::FlapRight);
    for (name, role, x0, x1) in [
        ("FlapInboardLeft", fl, -1.86_f32, -0.62_f32),
        ("FlapOutboardLeft", fl, -3.76, -2.84),
        ("FlapInboardRight", fr, 0.62, 1.86),
        ("FlapOutboardRight", fr, 2.84, 3.76),
    ] {
        let xm = 0.5 * (x0 + x1);
        let (_, ty_m, zm_m, ..) = wing_station(xm);
        let (hy, hz) = (ty_m + 0.42, zm_m - 0.025);
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

fn build_ailerons_and_trims(prims: &mut Vec<P38MeshPrim>) {
    for (name, trim_name, role, x0, x1) in [
        (
            "AileronLeft",
            "AileronTrimTabLeft",
            P38PartRole::AileronLeft,
            -AILERON_END_X,
            -3.84_f32,
        ),
        (
            "AileronRight",
            "AileronTrimTabRight",
            P38PartRole::AileronRight,
            3.84,
            AILERON_END_X,
        ),
    ] {
        let xm = 0.5 * (x0 + x1);
        let (_, ty_m, zm_m, ..) = wing_station(xm);
        let (hy, hz) = (ty_m + 0.32, zm_m);
        let (_, ty0, zm0, ..) = wing_station(x0);
        let (_, ty1, zm1, ..) = wing_station(x1);
        let axis = DVec3::new(
            f64::from(x1 - x0),
            f64::from(ty1 - ty0),
            f64::from(zm1 - zm0),
        )
        .normalize_or(DVec3::X);

        let mut mesh = MeshBuilder::new();
        let sections: Vec<Vec<Vec3>> = [x0, xm, x1]
            .into_iter()
            .map(|x| {
                let (_, ty, zm, ..) = wing_station(x);
                aerofoil(
                    Vec3::new(x - xm, ty + 0.32 - hy, zm - hz),
                    Vec3::new(0.0, -0.32, 0.0),
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
            Vec3::new(-0.24, -0.34, -0.012),
            Vec3::new(0.24, -0.24, 0.012),
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

fn build_nose_gear(prims: &mut Vec<P38MeshPrim>) {
    let mut wells = MeshBuilder::new();
    wells.cuboid(Vec3::new(-0.20, 1.54, -0.44), Vec3::new(0.20, 2.30, -0.34));
    for side in [-1.0_f32, 1.0] {
        let bx = side * BOOM_X;
        wells.cuboid(
            Vec3::new(bx - 0.24, -1.52, -0.42),
            Vec3::new(bx + 0.24, -0.36, -0.32),
        );
    }
    prims.push(fixed(
        "/P38g/Gear/WheelWells",
        P38PartRole::CabinExterior,
        WHEEL_WELL_ZINC,
        wells,
    ));

    let nose_rest = DMat4::from_translation(DVec3::new(0.0, 2.25, -0.36));
    let mut n_strut = MeshBuilder::new();
    n_strut.tube(Vec3::ZERO, Vec3::new(0.0, 0.06, -0.82), 0.044, 0.036, 12);
    n_strut.tube(
        Vec3::new(0.0, 0.06, -0.82),
        Vec3::new(0.06, 0.08, -1.36),
        0.032,
        0.028,
        12,
    );
    prims.push(part(
        "/P38g/Gear/NoseStrut",
        P38PartRole::GearStruts,
        CHROME_OLEO,
        nose_rest,
        n_strut,
    ));

    let mut n_door = MeshBuilder::new();
    n_door.cuboid(Vec3::new(0.20, -0.68, -0.26), Vec3::new(0.23, 0.08, -0.02));
    prims.push(part(
        "/P38g/Gear/NoseDoor",
        P38PartRole::GearStruts,
        NEUTRAL_GRAY,
        nose_rest,
        n_door,
    ));
    append_wheel_and_hub(
        prims,
        "/P38g/Gear/NoseWheel",
        P38PartRole::NoseWheel,
        DVec3::new(0.0, 2.33, -1.72),
        0.34,
        0.10,
    );
}

fn build_main_gear(prims: &mut Vec<P38MeshPrim>) {
    for (s_name, w_name, w_role, side) in [
        (
            "MainStrutLeft",
            "WheelLeft",
            P38PartRole::WheelLeft,
            -1.0_f32,
        ),
        (
            "MainStrutRight",
            "WheelRight",
            P38PartRole::WheelRight,
            1.0_f32,
        ),
    ] {
        let bx = side * BOOM_X;
        let m_rest = DMat4::from_translation(DVec3::new(f64::from(bx), -0.45, -0.36));
        let mut m_strut = MeshBuilder::new();
        m_strut.tube(Vec3::ZERO, Vec3::new(0.0, -0.04, -1.24), 0.056, 0.044, 14);
        m_strut.tube(
            Vec3::new(0.0, -0.58, -0.06),
            Vec3::new(0.0, -0.02, -0.78),
            0.030,
            0.024,
            10,
        );
        m_strut.cuboid(
            Vec3::new(side * 0.36, -0.92, -0.28),
            Vec3::new(side * 0.39, 0.12, -0.02),
        );
        prims.push(part(
            &format!("/P38g/Gear/{s_name}"),
            P38PartRole::GearStruts,
            CHROME_OLEO,
            m_rest,
            m_strut,
        ));
        append_wheel_and_hub(
            prims,
            &format!("/P38g/Gear/{w_name}"),
            w_role,
            DVec3::new(f64::from(bx), -0.49, -1.60),
            0.46,
            0.15,
        );
    }
}

fn append_wheel_and_hub(
    prims: &mut Vec<P38MeshPrim>,
    path: &str,
    role: P38PartRole,
    center: DVec3,
    radius: f32,
    half_width: f32,
) {
    let mut tire = MeshBuilder::new();
    let rings: Vec<Vec<Vec3>> = [
        (-half_width, radius * 0.42),
        (-half_width * 0.92, radius * 0.86),
        (-half_width * 0.48, radius),
        (half_width * 0.48, radius),
        (half_width * 0.92, radius * 0.86),
        (half_width, radius * 0.42),
    ]
    .into_iter()
    .map(|(x, r)| {
        (0..24)
            .map(|i| {
                let (s, c) = (TAU * i as f32 / 24.0).sin_cos();
                Vec3::new(x, c * r, s * r)
            })
            .collect()
    })
    .collect();
    tire.loft(&rings, true, Inside::Axis);
    let rest = DMat4::from_translation(center);
    prims.push(part(path, role, TIRE_RUBBER, rest, tire));

    let mut hub = MeshBuilder::new();
    hub.tube(
        Vec3::new(-half_width * 1.04, 0.0, 0.0),
        Vec3::new(half_width * 1.04, 0.0, 0.0),
        radius * 0.44,
        radius * 0.44,
        16,
    );
    prims.push(part(&format!("{path}Hub"), role, CHROME_OLEO, rest, hub));
}
