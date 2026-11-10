use glam::{DVec3, Vec3};
use sim_core::AirframeArticulation;

use crate::{SlotCameraRole, UsdAircraftAsset};

/// The canopy opens as the P-38G's did: the top hatch swings up about its aft
/// frame, lifting its forward edge clear, and each side window winds down past
/// the sill, the other panes staying put.
#[test]
fn p38g_canopy_hatch_swings_up_and_side_windows_wind_down() {
    let p38 = UsdAircraftAsset::p38g().unwrap();
    let mut open = AirframeArticulation::default();
    open.cockpit.canopy = sim_core::CanopyOpening {
        hatch: 1.0,
        side_windows: [1.0, 0.0],
    };
    // Where the vertex of `path` furthest along `toward` goes as the canopy opens.
    let moved = |path: &str, toward: DVec3| {
        let index = p38
            .parts()
            .iter()
            .position(|p| p.prim_path == path)
            .unwrap();
        let vertex = p38.parts()[index]
            .vertices
            .iter()
            .map(|v| Vec3::from_array(v.position).as_dvec3())
            .max_by(|a, b| a.dot(toward).total_cmp(&b.dot(toward)))
            .unwrap();
        let at = |a: &AirframeArticulation| {
            let draws = p38.evaluate_draws(a, SlotCameraRole::External);
            let draw = draws.iter().find(|d| d.part_index == index).unwrap();
            draw.part_to_aircraft.transform_point3(vertex)
        };
        (at(&AirframeArticulation::default()), at(&open))
    };
    let (shut, raised) = moved("/P38g/Canopy/Glazing/Hatch", DVec3::Y);
    assert!(
        raised.z > shut.z + 0.4 && raised.y < shut.y,
        "{shut:?} -> {raised:?}"
    );
    // The hatch's frame goes up with its pane.
    let (frame_shut, frame_raised) = moved("/P38g/Canopy/HatchFrames", DVec3::Y);
    assert!(
        frame_raised.z > frame_shut.z + 0.4,
        "{frame_shut:?} -> {frame_raised:?}"
    );
    let sill = f64::from(sim_p38g::canopy_sill_z(shut.y as f32));
    let (top, wound) = moved("/P38g/Canopy/Glazing/SideWindowLeft", DVec3::Z);
    assert!(
        top.z > sill + 0.1 && wound.z < sill + 0.01,
        "{top:?} -> {wound:?}"
    );
    // Its top edge ends out on the sill, not standing inside the cabin.
    assert!(wound.x < top.x - 0.05 && (wound.y - top.y).abs() < 1e-9);
    for fixed in [
        "Glazing/SideWindowRight",
        "Glazing/Windscreen",
        "Glazing/Rear",
        "InteriorFrames",
    ] {
        let (a, b) = moved(&format!("/P38g/Canopy/{fixed}"), DVec3::Z);
        assert!(a.abs_diff_eq(b, 1e-9), "{fixed}");
    }
}
