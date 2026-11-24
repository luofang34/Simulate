use core::f64::consts::{FRAC_PI_2, PI, TAU};

use glam::DVec3;
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_spacetime::{NaifId, TICKS_PER_SECOND, TdbEpoch};

use super::solver::{enu_to_az_el, shift_epoch_seconds, wrap_azimuth_rad};
use super::{
    GroundTracker, GroundTrackerError, GroundTrackerStationSpec, TrackingErrorSpec,
    TrackingLinkMode,
};

fn sample_station() -> GroundTrackerStationSpec {
    GroundTrackerStationSpec::new(
        14,
        NaifId::EARTH,
        BodyEllipsoid::WGS84,
        Geodetic::from_degrees(35.426, -116.89, 1000.0),
        10.0_f64.to_radians(),
        8.4e9,
    )
}

#[test]
fn station_and_error_spec_validation_rejects_invalid_inputs() {
    let mut bad_semi = sample_station();
    bad_semi.ellipsoid.semi_major_m = 0.0;
    assert!(matches!(
        GroundTracker::new(bad_semi, TrackingErrorSpec::IDEAL),
        Err(GroundTrackerError::InvalidSpec(_))
    ));

    let mut bad_flat = sample_station();
    bad_flat.ellipsoid.flattening = 1.0;
    assert!(matches!(
        GroundTracker::new(bad_flat, TrackingErrorSpec::IDEAL),
        Err(GroundTrackerError::InvalidSpec(_))
    ));

    let mut bad_lat = sample_station();
    bad_lat.geodetic.lat_rad = 2.0;
    assert!(matches!(
        GroundTracker::new(bad_lat, TrackingErrorSpec::IDEAL),
        Err(GroundTrackerError::InvalidSpec(_))
    ));

    let mut bad_mask = sample_station();
    bad_mask.min_elevation_rad = FRAC_PI_2 + 0.01;
    assert!(matches!(
        GroundTracker::new(bad_mask, TrackingErrorSpec::IDEAL),
        Err(GroundTrackerError::InvalidSpec(_))
    ));

    let mut bad_freq = sample_station();
    bad_freq.carrier_frequency_hz = -1.0;
    assert!(matches!(
        GroundTracker::new(bad_freq, TrackingErrorSpec::IDEAL),
        Err(GroundTrackerError::InvalidSpec(_))
    ));

    let bad_occ = sample_station().with_occulting_body(NaifId::MOON, -10.0);
    assert!(matches!(
        GroundTracker::new(bad_occ, TrackingErrorSpec::IDEAL),
        Err(GroundTrackerError::InvalidSpec(_))
    ));

    let bad_sigma = TrackingErrorSpec {
        range_sigma_m: -0.5,
        ..TrackingErrorSpec::IDEAL
    };
    assert!(matches!(
        GroundTracker::new(sample_station(), bad_sigma),
        Err(GroundTrackerError::InvalidSpec(_))
    ));

    let nan_bias = TrackingErrorSpec {
        azimuth_bias_rad: f64::NAN,
        ..TrackingErrorSpec::IDEAL
    };
    assert!(matches!(
        GroundTracker::new(sample_station(), nan_bias),
        Err(GroundTrackerError::InvalidSpec(_))
    ));

    let inf_sigma = TrackingErrorSpec {
        elevation_sigma_rad: f64::INFINITY,
        ..TrackingErrorSpec::IDEAL
    };
    assert!(matches!(
        GroundTracker::new(sample_station(), inf_sigma),
        Err(GroundTrackerError::InvalidSpec(_))
    ));
}

#[test]
fn shift_epoch_seconds_preserves_picosecond_precision_at_2026_epochs() {
    let base = TdbEpoch::JAN_1_2026_00Z_TDB;
    for delta in [-123.456_789_012_3, -0.005, -1.0e-11, 0.0, 42.125] {
        let (tick_epoch, sub_tick) = shift_epoch_seconds(base, delta).unwrap();
        assert!(
            sub_tick.abs() <= 0.5 / (TICKS_PER_SECOND as f64) + 1.0e-15,
            "sub_tick={sub_tick} exceeded half-tick for delta={delta}"
        );
        let reconstructed = tick_epoch.signed_offset_seconds_from(base) + sub_tick;
        assert!(
            (reconstructed - delta).abs() < 1.0e-14,
            "delta={delta}, reconstructed={reconstructed}"
        );
    }
}

#[test]
fn wrap_azimuth_maps_into_half_open_zero_to_tau() {
    for raw in [-10.0 * TAU, -1.0e-15, -0.0, 0.0, 1.23, TAU, TAU + 0.5] {
        let w = wrap_azimuth_rad(raw);
        assert!(
            (0.0..TAU).contains(&w),
            "wrapped {raw} -> {w} out of [0, TAU)"
        );
        assert!(!w.is_sign_negative() || w == 0.0);
    }
    assert_eq!(wrap_azimuth_rad(TAU), 0.0);
    assert_eq!(wrap_azimuth_rad(-0.0), 0.0);
}

#[test]
fn enu_to_az_el_matches_cardinal_directions_and_zenith() {
    let (az_n, el_n) = enu_to_az_el(DVec3::new(0.0, 100.0, 0.0));
    assert!(az_n.abs() < 1e-12);
    assert!(el_n.abs() < 1e-12);

    let (az_e, el_e) = enu_to_az_el(DVec3::new(100.0, 0.0, 0.0));
    assert!((az_e - FRAC_PI_2).abs() < 1e-12);
    assert!(el_e.abs() < 1e-12);

    let (az_s, el_s) = enu_to_az_el(DVec3::new(0.0, -100.0, 0.0));
    assert!((az_s - PI).abs() < 1e-12);
    assert!(el_s.abs() < 1e-12);

    let (az_w, el_w) = enu_to_az_el(DVec3::new(-100.0, 0.0, 0.0));
    assert!((az_w - 3.0 * FRAC_PI_2).abs() < 1e-12);
    assert!(el_w.abs() < 1e-12);

    let (_, el_zen) = enu_to_az_el(DVec3::new(0.0, 0.0, 500_000.0));
    assert!((el_zen - FRAC_PI_2).abs() < 1e-12);

    let v = DVec3::new(3.0, 4.0, 5.0);
    let (az, el) = enu_to_az_el(v);
    let unit_reconstructed = DVec3::new(az.sin() * el.cos(), az.cos() * el.cos(), el.sin());
    assert!((unit_reconstructed - v.normalize()).length() < 1e-12);
}

use glam::DMat3;
use sim_spacetime::{
    BodyOrientationCatalog, BodyRotationState, ContentHash, CoverageSet, DataProvenance,
    DerivativeOrder, DynamicalModelIdentity, EphemerisCatalog, EphemerisError, FrameAxesSpec,
    FrameCode, FramedState, KinematicStateIcrf, OrientationError, SPEED_OF_LIGHT_M_S,
    StateFrameSpec, SurfaceNavGrade, TdbSpanNs, TrajectoryQueryError,
    TranslationalTrajectorySource, UnitSemantics, ValidEpochInterval,
};

struct ZeroCatalog;
impl EphemerisCatalog for ZeroCatalog {
    fn content_hash(&self) -> ContentHash {
        ContentHash::of_bytes(b"zero")
    }
    fn coverage(&self, _: NaifId, _: NaifId, _: DerivativeOrder) -> Vec<TdbSpanNs> {
        CoverageSet::unbounded().spans().to_vec()
    }
    fn geometric_state_icrf(
        &self,
        _: NaifId,
        _: NaifId,
        _: TdbEpoch,
        _: DerivativeOrder,
    ) -> Result<KinematicStateIcrf, EphemerisError> {
        Ok(KinematicStateIcrf::ZERO)
    }
    fn segment_knots_in(&self, _: NaifId, _: NaifId, _: TdbEpoch, _: TdbEpoch) -> Vec<TdbEpoch> {
        Vec::new()
    }
}
impl BodyOrientationCatalog for ZeroCatalog {
    fn content_hash(&self) -> ContentHash {
        ContentHash::of_bytes(b"zero-rot")
    }
    fn coverage(&self, _: NaifId, _: SurfaceNavGrade) -> Vec<TdbSpanNs> {
        CoverageSet::unbounded().spans().to_vec()
    }
    fn body_rotation(&self, _: NaifId, _: TdbEpoch) -> Result<BodyRotationState, OrientationError> {
        Ok(BodyRotationState {
            r_icrf_to_fixed: DMat3::IDENTITY,
            omega_body_rad_s: DVec3::ZERO,
            alpha_body_rad_s2: DVec3::ZERO,
            frame_code: FrameCode::ITRF93,
            grade: SurfaceNavGrade::PrecisionGeodetic,
        })
    }
}

struct SuperluminalTarget {
    prov: DataProvenance,
}
impl TranslationalTrajectorySource for SuperluminalTarget {
    fn trajectory_id(&self) -> u64 {
        1
    }
    fn model_identity(&self) -> DynamicalModelIdentity {
        DynamicalModelIdentity::PlanetaryKinematicOrFlight {
            body: NaifId::EARTH,
        }
    }
    fn provenance(&self) -> &DataProvenance {
        &self.prov
    }
    fn sample_translational_icrf(
        &self,
        epoch: TdbEpoch,
    ) -> Result<FramedState, TrajectoryQueryError> {
        let dt = epoch.signed_offset_seconds_from(TdbEpoch::J2000);
        let speed = 1.5 * SPEED_OF_LIGHT_M_S;
        Ok(FramedState {
            spec: StateFrameSpec {
                epoch,
                origin: NaifId::EARTH,
                axes: FrameAxesSpec::IcrfInertial,
                units: UnitSemantics::SiMetric,
            },
            pos: DVec3::new(0.0, 0.0, 1.0e8 + speed * dt),
            vel: DVec3::new(0.0, 0.0, speed),
        })
    }
}

#[test]
fn superluminal_target_rejects_with_non_convergence() {
    let station = GroundTrackerStationSpec::new(
        1,
        NaifId::EARTH,
        BodyEllipsoid::WGS84,
        Geodetic::from_degrees(90.0, 0.0, 0.0),
        -FRAC_PI_2,
        1.0e9,
    );
    let tracker = GroundTracker::new(station, TrackingErrorSpec::IDEAL).unwrap();
    let prov = DataProvenance::new(
        "test",
        "1",
        ValidEpochInterval::new(TdbEpoch::new(-1000, 0), TdbEpoch::new(1000, 0)),
        ContentHash::of_bytes(b"super"),
    );
    let target = SuperluminalTarget { prov };
    let res = tracker.measure_truth(
        &target,
        TdbEpoch::J2000,
        TrackingLinkMode::OneWayDownlink,
        &ZeroCatalog,
        &ZeroCatalog,
    );
    assert!(matches!(
        res,
        Err(GroundTrackerError::LightTimeDidNotConverge)
    ));
}
