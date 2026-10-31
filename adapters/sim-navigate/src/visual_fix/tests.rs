use glam::DMat3;
use navigate_contract::{ClockDomainId, SourceEpoch, SourceId};
use navigate_fusion::ObservationValue;
use navigate_visual_fusion::VerticalDatum;
use navigate_visual_session::{RegionChange, SurfaceModel};
use sim_core::{MapCell, VisualEstimate};

use super::*;

const SIGMA_M: f64 = 2.0;
const ANCHOR: Geodetic = Geodetic {
    lat_rad: 0.68,
    lon_rad: -1.32,
    height_m: 0.0,
};
/// A long boom, so a dropped or sign-flipped lever arm cannot hide in noise.
const LEVER_ARM_BODY_M: DVec3 = DVec3::new(40.0, 0.0, 12.0);

fn config() -> VisualFixConfig {
    VisualFixConfig {
        anchor: ANCHOR,
        mount: CameraMount::forward_nose(LEVER_ARM_BODY_M),
        cg_body_m: DVec3::ZERO,
        intrinsics: SensorIntrinsics {
            width: 800,
            height: 600,
            fx: 600.0,
            fy: 600.0,
            cx: 399.5,
            cy: 299.5,
        },
        map: MapRevision {
            release_id: "synthetic-delaware".into(),
            manifest_sha256: "a".repeat(64),
        },
        budget: VisualErrorBudget {
            map_horizontal_m: 3.0,
            map_vertical_m: 3.0,
            calibration_m: 1.0,
            geometry_model_m: 1.0,
            vertical_datum: VerticalDatum::Wgs84Ellipsoid,
            vertical_datum_m: 0.5,
        },
        reliability: MapReliability {
            horizontal_m: 3.0,
            vertical_m: 3.0,
            imagery_age_years: Some(0.0),
            age_growth_m_per_year: 0.0,
            unknown_age_years: 0.0,
            surface: SurfaceModel::Surface,
            change: RegionChange::Unchanged,
        },
        identity: VisualSourceIdentity {
            source: SourceId::new(40),
            epoch: SourceEpoch::new(0),
            clock: ClockDomainId::new(9),
        },
        host_independence: EvidenceIndependence::ValidatedIndependent,
        attitude_rms_rad: 0.002,
        session: SessionConfig::standard(),
    }
}

fn adapter() -> VisualFixAdapter {
    VisualFixAdapter::new(config(), MasterClockBridge::new(ClockDomainId::new(9))).unwrap()
}

fn estimate_at(position_enu_m: DVec3) -> VisualEstimate {
    VisualEstimate {
        position_enu_m,
        covariance_enu_m2: DMat3::from_diagonal(DVec3::splat(SIGMA_M * SIGMA_M)),
    }
}

fn instant(seconds: u64) -> SimInstant {
    SimInstant::from_nanos(seconds * 1_000_000_000)
}

fn fix(seconds: u64, position_enu_m: DVec3, eligibility: FusionEligibility) -> VisualFix {
    VisualFix {
        captured_at: instant(seconds),
        delivered_at: instant(seconds),
        eligibility,
        map_cell: Some(MapCell::containing(position_enu_m, 500.0)),
        estimate: estimate_at(position_enu_m),
        true_eye_enu_m: position_enu_m,
        position_nees: 0.0,
    }
}

fn admitted(disposition: FixDisposition) -> navigate_visual_fusion::VisualFix {
    match disposition {
        FixDisposition::Admitted(fix) => *fix,
        other => panic!("{other:?}"),
    }
}

fn fix_position(observation: &navigate_fusion::Observation) -> DVec3 {
    let ObservationValue::PositionFix { position, .. } = observation.value else {
        panic!("{observation:?}");
    };
    sim_geo::geodetic_to_ecef(&Geodetic {
        lat_rad: position.latitude_rad,
        lon_rad: position.longitude_rad,
        height_m: position.altitude_m,
    })
}

/// A level vehicle whose attitude is a quarter turn off the frame, so an arm
/// applied in the wrong axes lands in the wrong place.
fn attitude() -> DQuat {
    let up = sim_geo::LocalFrame::at(&ANCHOR).up();
    DQuat::from_axis_angle(up, 1.1)
        * DQuat::from_mat3(&sim_geo::LocalFrame::at(&ANCHOR).ned_to_ecef())
}

/// The fix must describe the vehicle reference point the filter tracks, not
/// the camera 41.8 m away on its boom.
#[test]
fn the_lever_arm_comes_off_before_the_fix_reaches_the_filter() {
    let mut adapter = adapter();
    let eye = DVec3::new(100.0, 100.0, 900.0);
    let eye_ecef = adapter.frame.enu_to_ecef(eye);
    let fix = admitted(
        adapter
            .offer(
                &fix(1, eye, FusionEligibility::IndependentAnchor),
                attitude(),
            )
            .unwrap(),
    );

    let expected = eye_ecef - attitude() * LEVER_ARM_BODY_M;
    let position = fix_position(&fix.observation);
    assert!((position - expected).length() < 1e-3, "{position}");
    assert!((position - eye_ecef).length() > 41.0);
}

/// The arm is measured from the centre of gravity as it moves, not the origin.
#[test]
fn the_lever_arm_is_measured_from_the_centre_of_gravity() {
    let cg_body_m = DVec3::new(3.0, 0.0, 1.0);
    let mut adapter = VisualFixAdapter::new(
        VisualFixConfig {
            cg_body_m,
            ..config()
        },
        MasterClockBridge::new(ClockDomainId::new(9)),
    )
    .unwrap();
    let eye = DVec3::new(100.0, 100.0, 900.0);
    let eye_ecef = adapter.frame.enu_to_ecef(eye);
    let fix = admitted(
        adapter
            .offer(
                &fix(1, eye, FusionEligibility::IndependentAnchor),
                attitude(),
            )
            .unwrap(),
    );
    let expected = eye_ecef - attitude() * (LEVER_ARM_BODY_M - cg_body_m);
    assert!((fix_position(&fix.observation) - expected).length() < 1e-3);
}

/// The ledger and the camera pose the session sees stay at the camera.
#[test]
fn the_session_is_keyed_on_the_camera_not_the_shifted_vehicle() {
    let adapter = adapter();
    let eye = DVec3::new(100.0, 100.0, 900.0);
    let at_vehicle = adapter
        .vehicle_estimate(
            &fix(1, eye, FusionEligibility::IndependentAnchor),
            attitude(),
            0,
        )
        .unwrap();
    let at_camera = adapter
        .camera_estimate(
            &fix(1, eye, FusionEligibility::IndependentAnchor),
            attitude(),
            0,
        )
        .unwrap();
    assert!((at_camera.pose.position - to_vector(eye)).norm() < 1e-3);
    assert!((at_vehicle.pose.position - at_camera.pose.position).norm() > 41.0);
    assert_eq!(at_vehicle.observation_sha256, at_camera.observation_sha256);
}

#[test]
fn an_uncovered_lever_arm_error_is_refused_at_setup() {
    let refused = VisualFixAdapter::new(
        VisualFixConfig {
            attitude_rms_rad: 0.1,
            ..config()
        },
        MasterClockBridge::default(),
    );
    assert!(matches!(
        refused,
        Err(NavigateBridgeError::LeverArmUncovered { .. })
    ));
}

#[test]
fn a_zero_budget_term_is_refused_at_setup() {
    let mut zero = config();
    zero.budget.map_horizontal_m = 0.0;
    assert!(matches!(
        VisualFixAdapter::new(zero, MasterClockBridge::default()),
        Err(NavigateBridgeError::VisualFusion(
            VisualFusionError::InvalidBudget { .. }
        ))
    ));
}

/// Warm tracks are correlated with the anchors behind them: Navigate takes
/// only map anchors as position fixes.
#[test]
fn a_warm_track_never_becomes_a_position_fix() {
    let mut adapter = adapter();
    let eye = DVec3::new(100.0, 100.0, 900.0);
    let disposition = adapter
        .offer(&fix(1, eye, FusionEligibility::CorrelatedTrack), attitude())
        .unwrap();
    assert_eq!(disposition, FixDisposition::Tracked);
    assert_eq!(adapter.session().usage().frames, 0);
}

/// A revisit the harness calls shared is refused even if the session has not
/// seen that cell, because either ledger alone can withhold independence.
#[test]
fn either_ledger_alone_withholds_independence() {
    let mut adapter = adapter();
    let eye = DVec3::new(100.0, 100.0, 900.0);
    let disposition = adapter
        .offer(&fix(1, eye, FusionEligibility::SharedMapCell), attitude())
        .unwrap();
    let FixDisposition::RefusedCorrelated {
        session_label,
        harness_label,
    } = disposition
    else {
        panic!("{disposition:?}");
    };
    assert_eq!(session_label, SessionEligibility::Independent);
    assert_eq!(harness_label, FusionEligibility::SharedMapCell);
}

#[test]
fn a_host_that_has_not_validated_independence_gets_no_fixes() {
    let mut adapter = VisualFixAdapter::new(
        VisualFixConfig {
            host_independence: EvidenceIndependence::Unknown,
            ..config()
        },
        MasterClockBridge::new(ClockDomainId::new(9)),
    )
    .unwrap();
    let disposition = adapter
        .offer(
            &fix(
                1,
                DVec3::new(100.0, 100.0, 900.0),
                FusionEligibility::IndependentAnchor,
            ),
            attitude(),
        )
        .unwrap();
    assert!(matches!(
        disposition,
        FixDisposition::RefusedCorrelated { .. }
    ));
}

/// The real ledger, not the adapter, names the earlier frame of a revisit.
#[test]
fn a_second_anchor_in_a_cell_carries_the_ledgers_shared_map_error() {
    let mut adapter = adapter();
    let (first, second) = (
        DVec3::new(100.0, 100.0, 900.0),
        DVec3::new(200.0, 100.0, 900.0),
    );
    admitted(
        adapter
            .offer(
                &fix(1, first, FusionEligibility::IndependentAnchor),
                attitude(),
            )
            .unwrap(),
    );
    let disposition = adapter
        .offer(
            &fix(2, second, FusionEligibility::SharedMapCell),
            attitude(),
        )
        .unwrap();
    let FixDisposition::RefusedCorrelated {
        session_label: SessionEligibility::SharedMapError { cell, earlier },
        ..
    } = disposition
    else {
        panic!("{disposition:?}");
    };
    assert_eq!((cell.east, cell.north), (0, 0));
    assert_eq!(earlier.index, 0);
}

#[test]
fn a_new_cell_is_admitted_with_map_identity_and_stamped_time() {
    let mut adapter = adapter();
    let cells = [
        DVec3::new(100.0, 100.0, 900.0),
        DVec3::new(700.0, 100.0, 900.0),
    ];
    for (seconds, eye) in (1u64..).zip(cells) {
        let fix = admitted(
            adapter
                .offer(
                    &fix(seconds, eye, FusionEligibility::IndependentAnchor),
                    attitude(),
                )
                .unwrap(),
        );
        assert_eq!(fix.map.release_id, "synthetic-delaware");
        assert_eq!(
            fix.observation.stamp.acquired_at.as_nanos(),
            seconds * 1_000_000_000
        );
        assert_eq!(fix.observation.stamp.source, SourceId::new(40));
    }
}

/// A budget of a few metres must widen the fix well past the image residual.
#[test]
fn the_error_budget_inflates_the_fix_beyond_image_geometry() {
    let mut adapter = adapter();
    let fix = admitted(
        adapter
            .offer(
                &fix(
                    1,
                    DVec3::new(100.0, 100.0, 900.0),
                    FusionEligibility::IndependentAnchor,
                ),
                attitude(),
            )
            .unwrap(),
    );
    let ObservationValue::PositionFix { covariance, .. } = fix.observation.value else {
        panic!();
    };
    let (north, east, down) = covariance.diagonal();
    assert!(north > 2.0 * SIGMA_M * SIGMA_M + 25.0, "{north}");
    assert!(east > 2.0 * SIGMA_M * SIGMA_M + 25.0, "{east}");
    assert!(down > 2.0 * SIGMA_M * SIGMA_M + 25.0, "{down}");
}

/// Warm-track differences reach the session as odometry, so a second anchor
/// joins the first's segment rather than starting after a gap.
#[test]
fn warm_tracks_join_anchors_into_one_odometry_segment() {
    let mut adapter = adapter();
    let (first, second) = (
        DVec3::new(100.0, 100.0, 900.0),
        DVec3::new(130.0, 100.0, 900.0),
    );
    for (seconds, eye, tier) in [
        (1, first, FusionEligibility::IndependentAnchor),
        (2, second, FusionEligibility::SharedMapCell),
    ] {
        adapter
            .offer(
                &fix(seconds, eye, FusionEligibility::CorrelatedTrack),
                attitude(),
            )
            .unwrap();
        adapter.offer(&fix(seconds, eye, tier), attitude()).unwrap();
    }
    assert_eq!(adapter.session().usage().segments, 1);
}
