//! Round-trip integration tests for the `GroundTracker` <-> CCSDS `TDM` interchange adapter.

use std::error::Error;

use ccsds_ndm::{
    ParseMode, Tdm, TdmAngleType, TdmMode, TdmRangeMode, TdmRangeUnits, TimeSystem, WriteOptions,
};
use sim_devices::{
    GroundTrackerStationSpec, GroundTrackerTdm, GroundTrackerTdmError, GroundTrackerTdmMeta,
    TdmImportedObservation, TdmRangeExportUnit, TrackingLinkMode, TrackingMeasurement,
    TrackingTruthMeasurement,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_spacetime::{NaifId, SPEED_OF_LIGHT_M_S, TdbEpoch, TimeScaleCatalog};

type TestResult = Result<(), Box<dyn Error>>;

fn dss14_station() -> GroundTrackerStationSpec {
    GroundTrackerStationSpec::new(
        14,
        NaifId::EARTH,
        BodyEllipsoid::WGS84,
        Geodetic::from_degrees(35.426, -116.89, 1000.0),
        10.0_f64.to_radians(),
        8.4e9,
    )
}

fn make_truth(
    rx_epoch: TdbEpoch,
    range_m: f64,
    range_rate_m_s: f64,
    azimuth_rad: f64,
    elevation_rad: f64,
    two_way: bool,
) -> TrackingTruthMeasurement {
    let lt = if two_way {
        2.0 * range_m / SPEED_OF_LIGHT_M_S
    } else {
        range_m / SPEED_OF_LIGHT_M_S
    };
    let factor = if two_way { 2.0 } else { 1.0 };
    TrackingTruthMeasurement {
        rx_epoch,
        tx_epoch: rx_epoch.advanced_seconds(-lt),
        bounce_epoch: two_way.then(|| rx_epoch.advanced_seconds(-0.5 * lt)),
        light_time_s: lt,
        range_m,
        range_rate_m_s,
        doppler_hz: -factor * 8.4e9 * range_rate_m_s / SPEED_OF_LIGHT_M_S,
        azimuth_rad,
        elevation_rad,
    }
}

fn noisy_sample(truth: TrackingTruthMeasurement, below_mask: bool) -> TrackingMeasurement {
    TrackingMeasurement {
        range_m: truth.range_m + 1.25,
        range_rate_m_s: truth.range_rate_m_s - 0.004,
        doppler_hz: truth.doppler_hz + 0.22,
        azimuth_rad: truth.azimuth_rad + 1.5e-4,
        elevation_rad: truth.elevation_rad - 1.0e-4,
        below_mask,
        truth,
    }
}

fn assert_obs_matches_truth(
    got: &TdmImportedObservation,
    expected: &TrackingTruthMeasurement,
    range_tol_m: f64,
    epoch_tol_s: f64,
) -> TestResult {
    assert!(
        got.epoch
            .signed_offset_seconds_from(expected.rx_epoch)
            .abs()
            < epoch_tol_s
    );
    assert!((got.range_m.ok_or("missing range_m")? - expected.range_m).abs() < range_tol_m);
    assert!(
        (got.range_rate_m_s.ok_or("missing range_rate_m_s")? - expected.range_rate_m_s).abs()
            < 1.0e-9
    );
    assert!((got.doppler_hz.ok_or("missing doppler_hz")? - expected.doppler_hz).abs() < 1.0e-9);
    assert!((got.azimuth_rad.ok_or("missing azimuth_rad")? - expected.azimuth_rad).abs() < 1.0e-12);
    assert!(
        (got.elevation_rad.ok_or("missing elevation_rad")? - expected.elevation_rad).abs()
            < 1.0e-12
    );
    Ok(())
}

#[test]
fn one_way_downlink_km_round_trips_kvn_and_xml() -> TestResult {
    let scales = TimeScaleCatalog::builtin()?;
    let station = dss14_station();
    let mut meta = GroundTrackerTdmMeta::for_station(
        "SIMULATE",
        "DSS-14",
        "AQUA",
        &station,
        TrackingLinkMode::OneWayDownlink,
    );
    meta.track_id = Some("TRK-001".to_owned());

    let t0 = TdbEpoch::JAN_1_2026_00Z_TDB;
    let m0 = noisy_sample(
        make_truth(t0, 1_450_123.45, -3_210.5, 1.15, 0.42, false),
        false,
    );
    let m_masked = noisy_sample(
        make_truth(
            t0.advanced_seconds(10.0),
            1_418_000.0,
            -3_100.0,
            1.20,
            0.05,
            false,
        ),
        true,
    );
    let m1 = noisy_sample(
        make_truth(
            t0.advanced_seconds(20.25),
            1_386_543.21,
            -2_980.25,
            1.25,
            0.48,
            false,
        ),
        false,
    );

    let tdm = GroundTrackerTdm::measurements_to_tdm(&meta, &[m0, m_masked, m1], &scales)?;
    let seg_meta = &tdm.segments.first().ok_or("missing segment")?.metadata;
    assert_eq!(seg_meta.mode, Some(TdmMode::Sequential));
    assert_eq!(seg_meta.path.as_deref(), Some("2,1"));
    assert_eq!(seg_meta.range_mode, Some(TdmRangeMode::OneWay));
    assert_eq!(seg_meta.range_units, Some(TdmRangeUnits::Km));

    for wire in [
        Tdm::from_kvn(&tdm.to_kvn(WriteOptions::default())?, ParseMode::Strict)?,
        Tdm::from_xml(&tdm.to_xml(WriteOptions::default())?, ParseMode::Strict)?,
    ] {
        let imported = GroundTrackerTdm::tdm_to_observations(
            &wire,
            Some(station.carrier_frequency_hz),
            &scales,
        )?;
        assert_eq!(imported.len(), 2);
        for (got, expected) in imported.iter().zip([&m0, &m1]) {
            assert!(
                got.epoch
                    .signed_offset_seconds_from(expected.truth.rx_epoch)
                    .abs()
                    < 1.0e-6
            );
            assert!((got.range_m.ok_or("missing range")? - expected.range_m).abs() < 1.0e-6);
            assert!(
                (got.range_rate_m_s.ok_or("missing rate")? - expected.range_rate_m_s).abs()
                    < 1.0e-9
            );
            assert!(
                (got.doppler_hz.ok_or("missing doppler")? - expected.doppler_hz).abs() < 1.0e-9
            );
            assert!((got.azimuth_rad.ok_or("missing az")? - expected.azimuth_rad).abs() < 1.0e-12);
            assert!(
                (got.elevation_rad.ok_or("missing el")? - expected.elevation_rad).abs() < 1.0e-12
            );
        }
    }
    Ok(())
}

#[test]
fn two_way_coherent_seconds_round_trips_truth_and_noisy_kvn_xml() -> TestResult {
    let scales = TimeScaleCatalog::builtin()?;
    let station = dss14_station();
    let mut meta = GroundTrackerTdmMeta::for_station(
        "SIMULATE",
        "DSS-14",
        "ORION",
        &station,
        TrackingLinkMode::TwoWayCoherent,
    );
    meta.time_system = TimeSystem::Tdb;
    meta.range_unit = TdmRangeExportUnit::Seconds;
    meta.include_below_mask = true;

    let t0 = TdbEpoch::JAN_1_2026_00Z_TDB;
    let truth0 = make_truth(t0, 384_400_123.5, 412.75, 3.85, 0.67, true);
    let truth1 = make_truth(
        t0.advanced_seconds(60.5),
        384_424_890.0,
        415.10,
        3.86,
        0.66,
        true,
    );

    let tdm_truth = GroundTrackerTdm::truth_measurements_to_tdm(&meta, &[truth0, truth1], &scales)?;
    let seg_meta = &tdm_truth
        .segments
        .first()
        .ok_or("missing segment")?
        .metadata;
    assert_eq!(seg_meta.path.as_deref(), Some("1,2,1"));
    assert_eq!(seg_meta.range_mode, Some(TdmRangeMode::Coherent));
    assert_eq!(seg_meta.range_units, Some(TdmRangeUnits::Seconds));
    assert_eq!(seg_meta.turnaround_numerator, Some(1));
    assert_eq!(seg_meta.turnaround_denominator, Some(1));

    for wire in [
        Tdm::from_kvn(
            &tdm_truth.to_kvn(WriteOptions::default())?,
            ParseMode::Strict,
        )?,
        Tdm::from_xml(
            &tdm_truth.to_xml(WriteOptions::default())?,
            ParseMode::Strict,
        )?,
    ] {
        let imported = GroundTrackerTdm::tdm_to_observations(
            &wire,
            Some(station.carrier_frequency_hz),
            &scales,
        )?;
        assert_eq!(imported.len(), 2);
        assert_obs_matches_truth(
            imported.first().ok_or("missing obs 0")?,
            &truth0,
            1.0e-4,
            1.0e-9,
        )?;
        assert_obs_matches_truth(
            imported.get(1).ok_or("missing obs 1")?,
            &truth1,
            1.0e-4,
            1.0e-9,
        )?;
    }

    let noisy = [noisy_sample(truth0, true)];
    let tdm_noisy = GroundTrackerTdm::measurements_to_tdm(&meta, &noisy, &scales)?;
    let back_noisy = GroundTrackerTdm::tdm_to_observations(&tdm_noisy, None, &scales)?;
    let b0 = back_noisy.first().ok_or("missing noisy obs 0")?;
    assert_eq!(back_noisy.len(), 1);
    assert!((b0.range_m.ok_or("missing noisy range")? - noisy[0].range_m).abs() < 1.0e-4);
    assert!((b0.doppler_hz.ok_or("missing noisy doppler")? - noisy[0].doppler_hz).abs() < 1.0e-9);
    Ok(())
}

#[test]
fn tdm_epoch_conversions_and_error_cases() -> TestResult {
    let scales = TimeScaleCatalog::builtin()?;
    let t0 = TdbEpoch::from_j2000_nanos(820_497_600_i128 * 1_000_000_000 + 123_456_789);
    for ts in [
        TimeSystem::Utc,
        TimeSystem::Tdb,
        TimeSystem::Tai,
        TimeSystem::Tt,
    ] {
        let ccsds = GroundTrackerTdm::tdb_to_epoch(t0, ts, &scales)?;
        let back = GroundTrackerTdm::epoch_to_tdb(&ccsds, ts, &scales)?;
        let tol = if ts == TimeSystem::Tdb {
            1.0e-9
        } else {
            1.0e-6
        };
        assert!(
            back.signed_offset_seconds_from(t0).abs() < tol,
            "scale={ts:?}"
        );
    }

    let station = dss14_station();
    let meta = GroundTrackerTdmMeta::for_station(
        "SIMULATE",
        "DSS-14",
        "TARGET",
        &station,
        TrackingLinkMode::OneWayDownlink,
    );
    assert!(matches!(
        GroundTrackerTdm::measurements_to_tdm(&meta, &[], &scales),
        Err(GroundTrackerTdmError::EmptyMeasurements)
    ));

    let truth = make_truth(t0, 500_000.0, 0.0, 0.5, 0.5, false);
    let mut bad_angle =
        GroundTrackerTdm::truth_measurements_to_tdm(&meta, std::slice::from_ref(&truth), &scales)?;
    bad_angle
        .segments
        .first_mut()
        .ok_or("missing seg")?
        .metadata
        .angle_type = Some(TdmAngleType::Radec);
    assert!(matches!(
        GroundTrackerTdm::tdm_to_observations(&bad_angle, None, &scales),
        Err(GroundTrackerTdmError::UnsupportedAngleType(_))
    ));

    let mut bad_ru =
        GroundTrackerTdm::truth_measurements_to_tdm(&meta, std::slice::from_ref(&truth), &scales)?;
    bad_ru
        .segments
        .first_mut()
        .ok_or("missing seg")?
        .metadata
        .range_units = Some(TdmRangeUnits::Ru);
    assert!(matches!(
        GroundTrackerTdm::tdm_to_observations(&bad_ru, None, &scales),
        Err(GroundTrackerTdmError::UnsupportedRangeUnits(_))
    ));
    Ok(())
}
