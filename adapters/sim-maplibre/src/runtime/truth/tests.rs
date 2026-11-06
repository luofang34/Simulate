use super::slots::{SlotPlan, TruthSlotMask, plan_slots};

#[test]
fn otw_only_never_plans_the_sensor() {
    let plan = plan_slots(TruthSlotMask::OTW_ONLY, true, true, true);
    assert_eq!(
        plan,
        SlotPlan {
            otw: true,
            stereo: true,
            sensor: false
        }
    );
}

#[test]
fn sensor_only_skips_otw_and_stereo() {
    let plan = plan_slots(TruthSlotMask::SENSOR_ONLY, true, true, true);
    assert_eq!(
        plan,
        SlotPlan {
            otw: false,
            stereo: false,
            sensor: true
        }
    );
}

#[test]
fn both_requires_stamps() {
    let plan = plan_slots(TruthSlotMask::BOTH, false, true, false);
    assert_eq!(
        plan,
        SlotPlan {
            otw: false,
            stereo: false,
            sensor: false
        }
    );
    let plan = plan_slots(TruthSlotMask::BOTH, true, false, true);
    assert_eq!(
        plan,
        SlotPlan {
            otw: true,
            stereo: false,
            sensor: true
        }
    );
}

#[test]
fn invalid_stars_are_dropped() {
    use glam::DVec3;

    use super::celestial::{CelestialStar, to_maplibre_stars};

    let good = CelestialStar {
        direction_ecef: DVec3::X,
        v_mag: 1.0,
        srgb: [1.0, 0.9, 0.8],
    };
    let stars = [
        good,
        CelestialStar {
            direction_ecef: DVec3::ZERO,
            ..good
        },
        CelestialStar {
            srgb: [2.0, 0.0, 0.0],
            ..good
        },
        CelestialStar {
            v_mag: f32::NAN,
            ..good
        },
    ];
    let converted = to_maplibre_stars(&stars);
    assert_eq!(converted.len(), 1);
    assert_eq!(converted.first().map(|s| s.magnitude()), Some(1.0));
}
