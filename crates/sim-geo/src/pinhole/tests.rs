use super::*;

const VGA: SensorIntrinsics = SensorIntrinsics {
    width: 800,
    height: 600,
    fx: 700.0,
    fy: 700.0,
    cx: 399.5,
    cy: 299.5,
};

#[test]
fn accepts_a_real_camera() {
    assert!(VGA.is_usable());
}

#[test]
fn rejects_optics_no_projection_can_be_built_from() {
    let unusable = [
        SensorIntrinsics { width: 0, ..VGA },
        SensorIntrinsics { height: 0, ..VGA },
        SensorIntrinsics { fx: 0.0, ..VGA },
        SensorIntrinsics { fy: -1.0, ..VGA },
        SensorIntrinsics {
            fx: f64::INFINITY,
            ..VGA
        },
        SensorIntrinsics {
            cx: f64::NAN,
            ..VGA
        },
        SensorIntrinsics {
            cy: f64::INFINITY,
            ..VGA
        },
    ];
    for intrinsics in unusable {
        assert!(!intrinsics.is_usable(), "{intrinsics:?}");
    }
}
