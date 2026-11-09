//! Markings, paint and surface blends of the built mesh.

use crate::{P38PartRole, p38g_mesh_prims, wing_station, wing_surface_point};

/// Seen from starboard (nose to the right) the nose number must read like the port one
/// (nose to the left): its strokes start and end at the same distances from the left
/// edge of the number, which a mirror image does not (the layout is not symmetric).
#[test]
fn the_nose_number_reads_the_same_from_either_side() {
    let prims = p38g_mesh_prims();
    let number = prims
        .iter()
        .find(|p| p.prim_path == "/P38g/Decals/White147Numbers")
        .expect("nose number");
    // Distance from the viewer's left edge of the number: aft is right from port,
    // forward is right from starboard.
    let seen = |port: bool| -> Vec<f32> {
        let xs: Vec<f32> = number
            .vertices
            .iter()
            .filter(|v| (v.position[0] < 0.0) == port)
            .map(|v| if port { -v.position[1] } else { v.position[1] })
            .collect();
        let left = xs.iter().copied().fold(f32::INFINITY, f32::min);
        xs.iter().map(|x| x - left).collect()
    };
    let (port, starboard) = (seen(true), seen(false));
    for (from, to, name) in [
        (&port, &starboard, "port"),
        (&starboard, &port, "starboard"),
    ] {
        for x in from {
            assert!(
                to.iter().any(|y| (x - y).abs() < 3e-3),
                "{name} stroke edge {x:.4} m from the left has no counterpart"
            );
        }
    }
}

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
