use glam::{DMat4, DVec3};

use super::dial::{DialFace, face_angle_for_test};
use super::{instrument_pack, instrument_panel_prims, panel_frame};
use crate::{NEEDLE_HANDS_PER_SLOT, P38PartRole};

fn face(id: &str) -> DialFace {
    let pack = instrument_pack().expect("the pinned pack parses");
    DialFace::from_spec(pack.instrument(id).expect(id)).expect("a dial")
}

#[test]
fn every_layout_slot_gets_needles_on_its_own_channels() {
    let pack = instrument_pack().unwrap();
    let prims = instrument_panel_prims();
    let needles: Vec<_> = prims
        .iter()
        .filter(|p| p.role == P38PartRole::InstrumentNeedle)
        .collect();
    // Six single-needle dials and the altimeter's three hands.
    assert_eq!(pack.layout.len(), 7);
    assert_eq!(needles.len(), 9);
    for (slot, place) in pack.layout.iter().enumerate() {
        let channel = slot * NEEDLE_HANDS_PER_SLOT;
        let needle = needles
            .iter()
            .find(|p| p.needle_channel == Some(channel))
            .unwrap_or_else(|| panic!("no needle on channel {channel}"));
        assert!(needle.prim_path.contains(&place.instrument));
    }
}

#[test]
fn dials_put_their_zero_where_the_period_instruments_did() {
    let deg = |f: &DialFace, v: f64| f64::from(face_angle_for_test(f, v)).to_degrees();
    let vsi = face("rate_of_climb");
    assert!(
        (deg(&vsi, 0.0) + 90.0).abs() < 1e-3,
        "VSI zero at nine o'clock"
    );
    assert!(
        deg(&vsi, 1000.0) < deg(&vsi, 2000.0),
        "climb turns clockwise"
    );
    let alt = face("altimeter");
    assert!(deg(&alt, 0.0).abs() < 1e-3, "altimeter zero at twelve");
    let asi = face("airspeed");
    let gap_centre = 0.5 * (deg(&asi, 0.0) + deg(&asi, 700.0) - 360.0);
    assert!(
        (gap_centre - 180.0).abs() < 1e-3,
        "ASI gap at six: {gap_centre}"
    );
}

/// The needle turned by the mechanism's angle for a value lies on that value's
/// graduation, so a needle reads what its face says.
#[test]
fn a_needle_turned_by_the_scale_angle_points_at_that_graduation() {
    let asi = face("airspeed");
    let prims = instrument_panel_prims();
    let needle = prims
        .iter()
        .find(|p| p.prim_path.ends_with("/airspeed_0"))
        .unwrap();
    let scale_200_rad = 120.0_f64.to_radians();
    let tip =
        needle.rest_to_aircraft * DMat4::from_rotation_y(scale_200_rad) * DVec3::Z.extend(0.0);
    let a = f64::from(face_angle_for_test(&asi, 200.0));
    let expected = panel_frame() * DVec3::new(a.sin(), 0.0, a.cos()).extend(0.0);
    assert!((tip - expected).length() < 1e-6, "{tip} vs {expected}");
}
