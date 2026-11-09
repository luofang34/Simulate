//! Markings, paint and surface blends of the built mesh.

use crate::{P38PartRole, p38g_mesh_prims, wing_station, wing_surface_point};

/// Flaps, ailerons, elevator and stabiliser carry the factory scheme like the wing: olive
/// drab on top, neutral gray underneath. Trim tabs are painted with their surface (no
/// source shows red tabs on a P-38G-13).
#[test]
fn control_surfaces_and_spinners_carry_the_factory_scheme() {
    let prims = p38g_mesh_prims();
    let colour = |path: &str| {
        prims
            .iter()
            .find(|p| p.prim_path == path)
            .map(|p| p.material.diffuse_rgb)
            .unwrap_or_else(|| panic!("no prim {path}"))
    };
    let (olive, gray) = (
        colour("/P38g/Wing/UpperOlive"),
        colour("/P38g/Wing/LowerGray"),
    );
    for surface in [
        "/P38g/Wing/FlapInboardLeft",
        "/P38g/Wing/FlapOutboardRight",
        "/P38g/Wing/AileronLeft",
        "/P38g/Wing/AileronRight",
        "/P38g/Empennage/Elevator",
        "/P38g/Empennage/HorizontalTail",
    ] {
        assert_eq!(colour(surface), olive, "{surface}");
        assert_eq!(colour(&format!("{surface}Underside")), gray, "{surface}");
    }
    for tab in prims.iter().filter(|p| p.prim_path.contains("TrimTab")) {
        assert_eq!(tab.material.diffuse_rgb, olive, "{}", tab.prim_path);
    }
    // The 339th FS painted its spinners with the airframe.
    for spinner in prims
        .iter()
        .filter(|p| p.role == P38PartRole::PropellerSpinner)
    {
        assert_eq!(spinner.material.diffuse_rgb, olive, "{}", spinner.prim_path);
    }
}

/// The wing-root fillets sit on the wing, never sunk through its upper skin (an
/// ellipsoid pushed through the wing showed as a saw-tooth along the root).
#[test]
fn wing_root_fillets_stay_on_the_wing_skin() {
    let prims = p38g_mesh_prims();
    let fillets = prims
        .iter()
        .find(|p| p.prim_path == "/P38g/Wing/KarmanFillets")
        .expect("fillets");
    for v in &fillets.vertices {
        let [x, y, z] = v.position;
        let (lead, trail, ..) = wing_station(x);
        let u = ((lead - y) / (lead - trail)).clamp(0.0, 1.0);
        let skin = wing_surface_point(x, u, true).z;
        assert!(
            z >= skin - 0.004,
            "fillet vertex {:?} is {:.3} m under the wing",
            v.position,
            skin - z
        );
    }
}

/// The rudders swing about their own vertical hinge line.
#[test]
fn rudders_hinge_on_their_vertical_hinge_line() {
    let prims = p38g_mesh_prims();
    let rudders: Vec<_> = prims
        .iter()
        .filter(|p| p.role == P38PartRole::Rudder)
        .collect();
    assert_eq!(rudders.len(), 2);
    for rudder in rudders {
        assert_eq!(
            rudder.custom_hinge_axis,
            Some(glam::DVec3::Z),
            "{}",
            rudder.prim_path
        );
    }
}

/// The drawn blades are the blades the propeller performance model flies: the planform
/// constants repeat `design/curtiss_electric_3b.json`.
#[test]
fn drawn_blades_follow_the_propeller_design_file() {
    use crate::articulated::{BLADE_CHORD_OVER_D, BLADE_THICKNESS_OVER_CHORD};
    let design: serde_json::Value =
        serde_json::from_str(include_str!("../../design/curtiss_electric_3b.json")).unwrap();
    let f = |a: &str, b: &str| design[a][b].as_f64().unwrap() as f32;
    assert_eq!(
        BLADE_CHORD_OVER_D,
        (
            f("chord_over_diameter", "base"),
            f("chord_over_diameter", "bulge"),
            f("chord_over_diameter", "tip_taper")
        )
    );
    assert_eq!(
        BLADE_THICKNESS_OVER_CHORD,
        (
            f("thickness_over_chord", "root"),
            f("thickness_over_chord", "tip")
        )
    );
    assert!(
        (design["diameter_in"].as_f64().unwrap() as f32 * 0.0254 - 2.0 * crate::PROP_RADIUS_M)
            .abs()
            < 0.002
    );
}
