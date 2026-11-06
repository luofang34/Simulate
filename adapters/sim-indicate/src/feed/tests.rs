use indicate_instrument_state::abi::v8::{AbiError, CAPACITY};
use indicate_instrument_state::{
    AirframeDisplayProfile, FreshnessPolicy, HeadingReference, IdentStr, PanelData, SignalStatus,
    UnusualAttitudeState, resolve_stateful,
};
use sim_apollo::CockpitInstrumentPanel;
use sim_core::{TickSpan, VehicleState};
use sim_geo::UtcTime;

use super::*;
use crate::{AirDataSample, DisplayAxes};

const AT: SimInstant = SimInstant::START;
const KT_PER_M_S: f32 = 1.943_844_5;
const FT_PER_M: f32 = 3.280_84;

fn origin() -> Geodetic {
    Geodetic::from_degrees(39.1, -75.6, 100.0)
}

fn config(altitude: AltitudeSource) -> FeedConfig {
    FeedConfig {
        body: BodyEllipsoid::WGS84,
        axes: DisplayAxes::AEROSPACE,
        origin: origin(),
        origin_id: 7,
        altitude,
        heading_mode: HeadingMode::default(),
    }
}

fn feed() -> StateFeed {
    StateFeed::new(config(AltitudeSource::OriginRelative))
}

fn cruising(heading_rad: f64) -> VehicleState {
    VehicleState::level_flight(
        &Geodetic::from_degrees(39.2, -75.6, 1_100.0),
        heading_rad,
        50.0,
    )
}

fn resolved(state: &AircraftState) -> PanelData {
    resolve_stateful(
        state,
        &FreshnessPolicy::default(),
        &AirframeDisplayProfile::simulator(),
        &mut UnusualAttitudeState::default(),
    )
}

fn air(static_pressure_pa: Option<f64>, setting: Option<f64>) -> AirDataSample {
    AirDataSample {
        true_airspeed_m_s: 50.0,
        density_kg_m3: 1.225 * 0.81,
        static_pressure_pa,
        altimeter_setting_hpa: setting,
    }
}

fn with_air(air: AirDataSample, validity: SourceValidity) -> FlightSample {
    FlightSample {
        air_data: Some(StampedReading::new(air, AT, validity)),
        ..FlightSample::truth(AT, cruising(0.0))
    }
}

/// A snapshot of an Earth vehicle, with the flown attitude's health replaced.
fn cockpit(validity: SourceValidity) -> FlightSample {
    let vehicle = cruising(1.0);
    let mut snapshot =
        CockpitInstrumentPanel::new(AT).observe_earth(&sim_apollo::EarthVehicleSources {
            timestamp: AT,
            vehicle,
            height_m: 1_100.0,
        });
    snapshot.attitude.validity = validity;
    FlightSample {
        cockpit: Some(snapshot),
        ..FlightSample::truth(AT, vehicle)
    }
}

#[test]
fn level_flight_resolves_to_what_the_vehicle_is_doing() {
    let state = feed()
        .state(&FlightSample::truth(AT, cruising(1.2)))
        .unwrap();
    let panel = resolved(&state);
    assert_eq!(panel.roll_rad.status, SignalStatus::Valid);
    assert!(panel.roll_rad.value.abs() < 1e-5 && panel.pitch_rad.value.abs() < 1e-5);
    assert!((panel.heading.value_rad.value - 1.2).abs() < 1e-5);
    assert_eq!(panel.heading.reference, HeadingReference::SimLocalTrue);
    assert!((panel.gs_kt.value - 50.0 * KT_PER_M_S).abs() < 1e-2);
    assert!((panel.track_rad.value - 1.2).abs() < 1e-4);
    assert!(panel.vsi_fpm.value.abs() < 1e-2);
    assert_eq!(panel.gs_kt.status, SignalStatus::Valid);
}

#[test]
fn origin_relative_altitude_is_labelled_as_such_and_ignores_the_earths_curve() {
    let far = VehicleState::level_flight(&Geodetic::from_degrees(41.0, -73.0, 1_100.0), 0.0, 50.0);
    let panel = resolved(&feed().state(&FlightSample::truth(AT, far)).unwrap());
    assert_eq!(panel.altitude.class, AltitudeClass::LocalRelative);
    assert_eq!(panel.altitude.origin, OriginId(7));
    assert!((panel.altitude.value_ft.value - 1_000.0 * FT_PER_M).abs() < 1.0);
}

#[test]
fn position_is_north_and_east_of_the_origin() {
    let north = VehicleState::level_flight(&Geodetic::from_degrees(39.11, -75.6, 100.0), 0.0, 0.0);
    let state = feed().state(&FlightSample::truth(AT, north)).unwrap();
    let [n, e, d] = state.kinematics.data.unwrap().pos_ned_m;
    assert!(
        (n - 1_111.0).abs() < 5.0 && e.abs() < 1.0 && d.abs() < 0.1,
        "{n} {e} {d}"
    );
}

#[test]
fn a_barometer_set_to_field_pressure_reads_height_above_the_field() {
    let mut feed = StateFeed::new(config(AltitudeSource::Barometric));
    let sample = with_air(air(Some(101_000.0), Some(1_010.0)), SourceValidity::Valid);
    let panel = resolved(&feed.state(&sample).unwrap());
    assert_eq!(panel.altitude.class, AltitudeClass::BaroIndicated);
    assert_eq!(panel.altitude.value_ft.status, SignalStatus::Valid);
    assert!(panel.altitude.value_ft.value.abs() < 1.0);
    assert!((panel.baro_hpa.value - 1_010.0).abs() < 1e-3);
}

#[test]
fn a_barometer_with_no_setting_declares_pressure_altitude() {
    let mut feed = StateFeed::new(config(AltitudeSource::Barometric));
    let sample = with_air(air(Some(89_875.0), None), SourceValidity::Valid);
    let panel = resolved(&feed.state(&sample).unwrap());
    assert_eq!(panel.altitude.class, AltitudeClass::Pressure);
    assert!((panel.altitude.value_ft.value - 3_281.0).abs() < 5.0);
}

#[test]
fn a_barometric_tape_fails_rather_than_falling_back_to_position() {
    let mut feed = StateFeed::new(config(AltitudeSource::Barometric));
    for sample in [
        FlightSample::truth(AT, cruising(0.0)),
        with_air(air(None, Some(1_013.25)), SourceValidity::Valid),
        with_air(air(Some(89_875.0), Some(1_013.25)), SourceValidity::Failed),
    ] {
        let panel = resolved(&feed.state(&sample).unwrap());
        assert_eq!(panel.altitude.value_ft.status, SignalStatus::Failed);
    }
}

#[test]
fn airspeed_is_fed_indicated_and_true_from_the_same_sample() {
    let sample = with_air(air(None, None), SourceValidity::Valid);
    let panel = resolved(&feed().state(&sample).unwrap());
    assert!((panel.ias_kt.value - 45.0 * KT_PER_M_S).abs() < 1e-2);
    assert!((panel.tas_kt.value - 50.0 * KT_PER_M_S).abs() < 1e-2);
}

#[test]
fn a_failed_air_data_source_offers_no_value() {
    let sample = with_air(air(None, None), SourceValidity::Failed);
    let state = feed().state(&sample).unwrap();
    assert_eq!(state.air.data, None);
    assert_eq!(resolved(&state).ias_kt.status, SignalStatus::Missing);
}

#[test]
fn no_air_data_is_missing_on_every_tape_it_feeds() {
    let panel = resolved(
        &feed()
            .state(&FlightSample::truth(AT, cruising(0.0)))
            .unwrap(),
    );
    assert_eq!(panel.ias_kt.status, SignalStatus::Missing);
    assert_eq!(panel.tas_kt.status, SignalStatus::Missing);
}

#[test]
fn source_health_decides_how_attitude_shows() {
    for (validity, status) in [
        (SourceValidity::Valid, SignalStatus::Valid),
        (SourceValidity::Degraded, SignalStatus::Degraded),
        (SourceValidity::Failed, SignalStatus::Failed),
        (SourceValidity::Missing, SignalStatus::Missing),
    ] {
        let state = feed().state(&cockpit(validity)).unwrap();
        assert_eq!(resolved(&state).roll_rad.status, status, "{validity:?}");
    }
}

#[test]
fn a_backup_source_lowers_the_quality_of_the_whole_frame() {
    assert_eq!(
        feed()
            .state(&cockpit(SourceValidity::Degraded))
            .unwrap()
            .quality,
        EstimateQuality::Degraded
    );
    assert_eq!(
        feed()
            .state(&cockpit(SourceValidity::Valid))
            .unwrap()
            .quality,
        EstimateQuality::Good
    );
}

#[test]
fn a_reading_that_stopped_updating_goes_stale_by_its_age() {
    let mut sample = cockpit(SourceValidity::Valid);
    sample.timestamp = AT.advanced(TickSpan::from_seconds_and_subsec_ticks(1, 0));
    let state = feed().state(&sample).unwrap();
    let age = state.attitude.age_ms.unwrap();
    assert!((age - 1_000.0).abs() < 1e-3, "{age}");
    assert_eq!(resolved(&state).roll_rad.status, SignalStatus::Stale);
}

#[test]
fn groups_acquired_too_far_apart_are_not_called_coherent() {
    let mut late = with_air(air(None, None), SourceValidity::Valid);
    late.timestamp = AT.advanced(TickSpan::from_ticks(sim_core::TICKS_PER_SECOND / 5));
    let state = feed().state(&late).unwrap();
    assert_eq!(state.snapshot.coherence, SnapshotCoherence::ExcessiveSkew);
    let fresh = with_air(air(None, None), SourceValidity::Valid);
    let state = feed().state(&fresh).unwrap();
    assert_eq!(state.snapshot.coherence, SnapshotCoherence::Coherent);
}

#[test]
fn a_nose_at_the_vertical_withholds_heading_and_turn_rather_than_inventing_them() {
    let mut vehicle = cruising(0.0);
    vehicle.attitude *= DQuat::from_rotation_y(90f64.to_radians());
    vehicle.angular_rate_body = DVec3::Z;
    let state = feed().state(&FlightSample::truth(AT, vehicle)).unwrap();
    assert_eq!(state.heading.data, None);
    assert_eq!(state.dynamics.data, None);
    assert_eq!(
        resolved(&state).heading.value_rad.status,
        SignalStatus::Missing
    );
}

#[test]
fn turn_rate_comes_from_heading_change_not_body_yaw() {
    let bank = 30f64.to_radians();
    let turn_rate = 0.1;
    let mut vehicle = cruising(0.0);
    vehicle.attitude *= DQuat::from_rotation_x(bank);
    vehicle.angular_rate_body = DQuat::from_rotation_x(bank).inverse() * (DVec3::Z * turn_rate);
    let state = feed().state(&FlightSample::truth(AT, vehicle)).unwrap();
    let turn = state.dynamics.data.unwrap().turn.unwrap();
    assert_eq!(turn.basis, TurnBasis::HeadingRate);
    assert!(
        (f64::from(turn.rate_rps) - turn_rate).abs() < 1e-6,
        "{turn:?}"
    );
    assert!(state.valid.turn);
}

#[test]
fn slip_reads_lateral_force_in_display_axes() {
    let sample = FlightSample {
        specific_force_body_m_s2: Some(DVec3::new(0.0, 1.5, -9.8)),
        ..FlightSample::truth(AT, cruising(0.0))
    };
    let panel = resolved(&feed().state(&sample).unwrap());
    assert_eq!(panel.slip_lat_mps2.status, SignalStatus::Valid);
    assert!((panel.slip_lat_mps2.value - 1.5).abs() < 1e-6);
}

#[test]
fn non_finite_motion_is_refused_by_name_and_instant() {
    let mut vehicle = cruising(0.0);
    vehicle.velocity_ecef = DVec3::NAN;
    let later = AT.advanced(TickSpan::from_ticks(5));
    assert_eq!(
        feed().state(&FlightSample::truth(later, vehicle)),
        Err(FeedError::NonFinite {
            field: "vehicle.velocity_ecef",
            tick: 5
        }),
    );
    let mut sample = FlightSample::truth(AT, cruising(0.0));
    sample.specific_force_body_m_s2 = Some(DVec3::INFINITY);
    assert!(matches!(
        feed().state(&sample),
        Err(FeedError::NonFinite {
            field: "specific_force_body_m_s2",
            ..
        })
    ));
}

#[test]
fn a_j2000_attitude_without_the_bodys_orientation_is_refused() {
    let mut sample = cockpit(SourceValidity::Valid);
    if let Some(snapshot) = sample.cockpit.as_mut() {
        snapshot.guidance_source = Some(sim_apollo::GuidanceSource::Pgns);
    }
    assert_eq!(
        feed().state(&sample),
        Err(FeedError::MissingBodyFrame { tick: 0 })
    );
    let vehicle_to_fixed = sample.vehicle.attitude;
    let frame = InertialTransform::earth_at(UtcTime::from_unix_seconds(1_700_000_000.0));
    if let Some(snapshot) = sample.cockpit.as_mut() {
        snapshot.attitude.value = frame.fixed_to_j2000_rotation() * vehicle_to_fixed;
    }
    sample.central_body = Some(frame);
    let inertial = feed().state(&sample).unwrap();
    let fixed = feed()
        .state(&FlightSample::truth(AT, sample.vehicle))
        .unwrap();
    let (a, b) = (inertial.heading.data.unwrap(), fixed.heading.data.unwrap());
    assert!((a.heading_rad - b.heading_rad).abs() < 1e-5);
}

#[test]
fn generations_advance_per_frame() {
    let mut feed = feed();
    let sample = FlightSample::truth(AT, cruising(0.0));
    let first = feed.state(&sample).unwrap().snapshot.generation;
    let second = feed.state(&sample).unwrap().snapshot.generation;
    assert_eq!(second, first.wrapping_add(1));
}

#[test]
fn a_frame_survives_the_binary_abi_unchanged() {
    let sample = FlightSample {
        specific_force_body_m_s2: Some(DVec3::new(0.0, 0.2, -9.8)),
        ..with_air(air(Some(89_875.0), Some(1_013.25)), SourceValidity::Valid)
    };
    let mut feed = StateFeed::new(config(AltitudeSource::Barometric));
    let state = feed.state(&sample).unwrap();
    let mut buffer = [0u8; CAPACITY];
    let length = encode_frame(&state, &mut buffer).unwrap();
    let report = decode_frame(&buffer[..length]).unwrap();
    assert_eq!(report.state, state);
    assert_eq!(report.unknown_groups, 0);
    assert_eq!(report.trailing_bytes, 0);
}

#[test]
fn a_frame_that_does_not_fit_or_does_not_parse_says_so() {
    let state = feed()
        .state(&FlightSample::truth(AT, cruising(0.0)))
        .unwrap();
    let mut small = [0u8; 16];
    assert!(matches!(
        encode_frame(&state, &mut small),
        Err(FeedError::Encode { capacity: 16, .. })
    ));
    assert_eq!(
        decode_frame(&[9, 0]).unwrap_err(),
        FeedError::Decode {
            length: 2,
            source: AbiError::BadVersion { found: 9 }
        },
    );
}

#[test]
fn earth_magnetic_mode_resolves_newark_rwy_22l_in_magnetic_degrees() {
    let threshold = Geodetic::from_degrees(40.69, -74.17, 5.0);
    let course_rad = 206.0_f64.to_radians();
    let mut mag_feed = StateFeed::new(FeedConfig {
        origin: threshold,
        heading_mode: HeadingMode::EarthMagnetic,
        ..config(AltitudeSource::OriginRelative)
    });
    let vehicle = VehicleState::level_flight(&threshold, course_rad, 50.0);
    let sample = FlightSample {
        guidance: Some(HostFlightGuidance {
            selections: PilotSelections {
                heading_rad: course_rad,
                altitude_m: None,
                baro_setting_hpa: None,
            },
            nav: HostNavGuidance::RunwayReceiver(RunwayNavGuidance {
                threshold,
                course_rad,
                length_m: 3_353.0,
                glide_path_rad: 3.0_f64.to_radians(),
                ident: IdentStr::new("22L").unwrap(),
            }),
            autopilot: AutopilotModeSummary::default(),
            source_epoch: 0,
        }),
        ..FlightSample::truth(AT, vehicle)
    };
    let state = mag_feed.state(&sample).unwrap();
    let panel = resolved(&state);
    assert_eq!(panel.heading.reference, HeadingReference::Magnetic);
    let hdg_deg = panel.heading.value_rad.value.to_degrees();
    let trk_deg = panel.track_rad.value.to_degrees();
    let bug_deg = panel.heading_bug_rose_rad.value.to_degrees();
    let crs_deg = panel.nav.course_rose_rad.value.to_degrees();
    assert!((hdg_deg - 219.0).abs() < 1.0, "hdg={hdg_deg}");
    assert!((trk_deg - hdg_deg).abs() < 0.05, "trk={trk_deg}");
    assert!((bug_deg - hdg_deg).abs() < 0.05, "bug={bug_deg}");
    assert!((crs_deg - hdg_deg).abs() < 0.05, "crs={crs_deg}");
    assert!(panel.nav.data.cdi_dots.abs() < 1e-4);
}

#[test]
fn earth_magnetic_mode_reverts_to_true_north_in_polar_keyhole() {
    for polar in [
        Geodetic::from_degrees(86.0, 140.0, 1_000.0),
        Geodetic::from_degrees(-89.9975, 139.2729, 2_835.0),
    ] {
        let mut mag_feed = StateFeed::new(FeedConfig {
            origin: polar,
            heading_mode: HeadingMode::EarthMagnetic,
            ..config(AltitudeSource::OriginRelative)
        });
        let vehicle = VehicleState::level_flight(&polar, 1.2, 50.0);
        let state = mag_feed.state(&FlightSample::truth(AT, vehicle)).unwrap();
        assert_eq!(state.variation.data, None);
        let panel = resolved(&state);
        assert_eq!(panel.heading.reference, HeadingReference::True);
        assert!((panel.heading.value_rad.value - 1.2).abs() < 1e-5);
        assert!((panel.track_rad.value - 1.2).abs() < 1e-4);
    }
}
