use super::{MeshFeature, StructuralLandmarks};
use crate::{builtin_c172_package, builtin_p38g_package};

#[test]
fn builtin_packages_ship_their_landmarks() {
    let c172 = builtin_c172_package().unwrap();
    let landmarks = c172.structural_landmarks().unwrap();
    assert_eq!(landmarks.landmarks.len(), 4);
    let propeller = &landmarks.landmarks[0];
    assert_eq!(propeller.feature, MeshFeature::PropellerDiscCentre);
    assert!((landmarks.structural_station_in(propeller) - 40.4787).abs() < 1e-9);
    let p38g = builtin_p38g_package().unwrap();
    assert!(!p38g.structural_landmarks().unwrap().landmarks.is_empty());
}

#[test]
fn landmarks_without_stations_or_spans_are_refused() {
    let base = r#"{"source_station_offset_in":0,"source_frame":"f",
        "height":{"feature":{"rule":"propeller_disc_centre"},"structural_up_in":0,"source":"s"},
        "landmarks":LANDMARKS}"#;
    let parse = |landmarks: &str| {
        StructuralLandmarks::from_json_bytes(base.replace("LANDMARKS", landmarks).as_bytes())
    };
    assert!(parse("[]").is_err());
    assert!(
        parse(r#"[{"name":"le","station_in":1,"source":"s","feature":{"rule":"wing_leading_edge","span_from_m":2,"span_to_m":1}}]"#)
            .is_err()
    );
    assert!(parse(r#"[{"name":"p","station_in":1,"source":"s","feature":{"rule":"prim_box_centre","prim_path":"/A"}}]"#).is_ok());
}
