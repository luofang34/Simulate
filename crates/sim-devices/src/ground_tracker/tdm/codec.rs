//! Export/import observation mapping between `GroundTracker` rows and `ccsds_ndm::Tdm`.

use ccsds_ndm::{
    Epoch, Header, Tdm, TdmAngleType, TdmData, TdmMetadata, TdmMode, TdmObservable, TdmObservation,
    TdmRangeMode, TdmRangeUnits, TdmSegment, TdmTimetagRef, TimeSystem, Version,
};
use sim_spacetime::{SPEED_OF_LIGHT_M_S, TdbEpoch, TimeScaleCatalog};

use super::super::TrackingLinkMode;
use super::epoch::{epoch_to_tdb_impl, tdb_to_epoch_impl};
use super::{
    GroundTrackerTdmError, GroundTrackerTdmMeta, TdmImportedObservation, TdmRangeExportUnit,
};

const M_TO_KM: f64 = 1.0e-3;
const KM_TO_M: f64 = 1_000.0;
const RAD_TO_DEG: f64 = 180.0 / core::f64::consts::PI;
const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;

pub(super) struct ExportRow {
    pub(super) rx_epoch: TdbEpoch,
    pub(super) range_m: f64,
    pub(super) range_rate_m_s: f64,
    pub(super) doppler_hz: f64,
    pub(super) azimuth_rad: f64,
    pub(super) elevation_rad: f64,
}

struct EpochBucket {
    raw_epoch: Epoch,
    entry: TdmImportedObservation,
}

pub(super) fn build_tdm_from_rows(
    meta: &GroundTrackerTdmMeta,
    rows: &[ExportRow],
    scales: &TimeScaleCatalog,
) -> Result<Tdm, GroundTrackerTdmError> {
    if rows.is_empty() {
        return Err(GroundTrackerTdmError::EmptyMeasurements);
    }
    let has_freq = meta
        .carrier_frequency_hz
        .is_some_and(|f| f.is_finite() && f > 0.0);
    let mut observations = Vec::with_capacity(rows.len().saturating_mul(5));
    for row in rows {
        push_row_observations(meta, row, has_freq, scales, &mut observations)?;
    }
    let start_time = observations.first().map(|o| o.epoch.clone());
    let stop_time = observations.last().map(|o| o.epoch.clone());
    let creation_date = meta.creation_date.clone().or_else(|| start_time.clone());
    let metadata = build_tdm_metadata(meta, has_freq, start_time, stop_time);
    Ok(Tdm {
        version: Version::V2,
        header: Header {
            comments: Vec::new(),
            classification: None,
            creation_date,
            originator: Some(meta.originator.clone()),
            message_id: None,
        },
        segments: vec![TdmSegment {
            metadata,
            data: TdmData {
                comments: Vec::new(),
                observations,
            },
        }],
        extensions: Vec::new(),
    })
}

fn build_tdm_metadata(
    meta: &GroundTrackerTdmMeta,
    has_freq: bool,
    start_time: Option<Epoch>,
    stop_time: Option<Epoch>,
) -> TdmMetadata {
    let (path, range_mode, turnaround) = match meta.link_mode {
        TrackingLinkMode::OneWayDownlink => ("2,1".to_owned(), TdmRangeMode::OneWay, None),
        TrackingLinkMode::TwoWayCoherent => (
            "1,2,1".to_owned(),
            TdmRangeMode::Coherent,
            has_freq.then_some(1),
        ),
    };
    let mut data_types = vec![
        "RANGE".to_owned(),
        "DOPPLER_INSTANTANEOUS".to_owned(),
        "ANGLE_1".to_owned(),
        "ANGLE_2".to_owned(),
    ];
    if has_freq {
        data_types.push("RECEIVE_FREQ_1".to_owned());
    }
    let range_units = match meta.range_unit {
        TdmRangeExportUnit::Km => TdmRangeUnits::Km,
        TdmRangeExportUnit::Seconds => TdmRangeUnits::Seconds,
    };
    let mut out = empty_tdm_metadata(meta.time_system);
    out.track_id = meta.track_id.clone();
    out.data_types = Some(data_types);
    out.start_time = start_time;
    out.stop_time = stop_time;
    out.participants = [
        Some(meta.station_name.clone()),
        Some(meta.target_name.clone()),
        None,
        None,
        None,
    ];
    out.mode = Some(TdmMode::Sequential);
    out.path = Some(path);
    out.turnaround_numerator = turnaround;
    out.turnaround_denominator = turnaround;
    out.timetag_ref = Some(TdmTimetagRef::Receive);
    out.freq_offset = meta
        .carrier_frequency_hz
        .filter(|f| f.is_finite() && *f > 0.0);
    out.range_mode = Some(range_mode);
    out.range_units = Some(range_units);
    out.angle_type = Some(TdmAngleType::Azel);
    out.reference_frame = meta.reference_frame.clone();
    out.data_quality = meta.data_quality.clone();
    out
}

fn empty_tdm_metadata(time_system: TimeSystem) -> TdmMetadata {
    TdmMetadata {
        comments: Vec::new(),
        track_id: None,
        data_types: None,
        time_system,
        start_time: None,
        stop_time: None,
        participants: [None, None, None, None, None],
        mode: None,
        path: None,
        path_1: None,
        path_2: None,
        ephemeris_names: [None, None, None, None, None],
        transmit_band: None,
        receive_band: None,
        turnaround_numerator: None,
        turnaround_denominator: None,
        timetag_ref: None,
        integration_interval: None,
        integration_ref: None,
        freq_offset: None,
        range_mode: None,
        range_modulus: None,
        range_units: None,
        angle_type: None,
        reference_frame: None,
        interpolation: None,
        interpolation_degree: None,
        doppler_count_bias: None,
        doppler_count_scale: None,
        doppler_count_rollover: None,
        transmit_delays: [None, None, None, None, None],
        receive_delays: [None, None, None, None, None],
        data_quality: None,
        correction_angle_1: None,
        correction_angle_2: None,
        correction_doppler: None,
        correction_mag: None,
        correction_range: None,
        correction_rcs: None,
        correction_receive: None,
        correction_transmit: None,
        correction_aberration_yearly: None,
        correction_aberration_diurnal: None,
        corrections_applied: None,
    }
}

fn push_row_observations(
    meta: &GroundTrackerTdmMeta,
    row: &ExportRow,
    has_freq: bool,
    scales: &TimeScaleCatalog,
    out: &mut Vec<TdmObservation>,
) -> Result<(), GroundTrackerTdmError> {
    let epoch = tdb_to_epoch_impl(row.rx_epoch, meta.time_system, scales)?;
    let range_val = match meta.range_unit {
        TdmRangeExportUnit::Km => row.range_m * M_TO_KM,
        TdmRangeExportUnit::Seconds => match meta.link_mode {
            TrackingLinkMode::OneWayDownlink => row.range_m / SPEED_OF_LIGHT_M_S,
            TrackingLinkMode::TwoWayCoherent => 2.0 * row.range_m / SPEED_OF_LIGHT_M_S,
        },
    };
    push_finite_obs(out, &epoch, TdmObservable::Range, range_val)?;
    push_finite_obs(
        out,
        &epoch,
        TdmObservable::DopplerInstantaneous,
        row.range_rate_m_s * M_TO_KM,
    )?;
    if has_freq {
        push_finite_obs(
            out,
            &epoch,
            TdmObservable::ReceiveFreq(Some(1)),
            row.doppler_hz,
        )?;
    }
    push_finite_obs(
        out,
        &epoch,
        TdmObservable::Angle1,
        row.azimuth_rad * RAD_TO_DEG,
    )?;
    push_finite_obs(
        out,
        &epoch,
        TdmObservable::Angle2,
        row.elevation_rad * RAD_TO_DEG,
    )?;
    Ok(())
}

fn push_finite_obs(
    out: &mut Vec<TdmObservation>,
    epoch: &Epoch,
    kind: TdmObservable,
    value: f64,
) -> Result<(), GroundTrackerTdmError> {
    if !value.is_finite() {
        return Err(GroundTrackerTdmError::NonFinite);
    }
    out.push(TdmObservation {
        epoch: epoch.clone(),
        kind,
        value,
    });
    Ok(())
}

pub(super) fn import_segment_observations(
    seg: &TdmSegment,
    nominal_carrier_hz: Option<f64>,
    scales: &TimeScaleCatalog,
    out: &mut Vec<TdmImportedObservation>,
) -> Result<(), GroundTrackerTdmError> {
    let meta = &seg.metadata;
    if let Some(angle_type) = &meta.angle_type
        && *angle_type != TdmAngleType::Azel
    {
        return Err(GroundTrackerTdmError::UnsupportedAngleType(
            angle_type.clone(),
        ));
    }
    if let Some(ru) = &meta.range_units
        && !matches!(ru, TdmRangeUnits::Km | TdmRangeUnits::Seconds)
    {
        return Err(GroundTrackerTdmError::UnsupportedRangeUnits(ru.clone()));
    }
    let mut buckets: Vec<EpochBucket> = Vec::new();
    for obs in &seg.data.observations {
        let bucket = find_or_insert_bucket(&mut buckets, &obs.epoch, meta.time_system, scales)?;
        apply_tdm_observable(meta, obs, nominal_carrier_hz, bucket)?;
    }
    out.extend(buckets.into_iter().map(|b| b.entry));
    Ok(())
}

fn find_or_insert_bucket<'a>(
    buckets: &'a mut Vec<EpochBucket>,
    raw_epoch: &Epoch,
    time_system: TimeSystem,
    scales: &TimeScaleCatalog,
) -> Result<&'a mut TdmImportedObservation, GroundTrackerTdmError> {
    let idx = if let Some(found) = buckets.iter().rposition(|b| &b.raw_epoch == raw_epoch) {
        found
    } else {
        let epoch = epoch_to_tdb_impl(raw_epoch, time_system, scales)?;
        buckets.push(EpochBucket {
            raw_epoch: raw_epoch.clone(),
            entry: TdmImportedObservation {
                epoch,
                range_m: None,
                range_rate_m_s: None,
                doppler_hz: None,
                azimuth_rad: None,
                elevation_rad: None,
            },
        });
        buckets.len().saturating_sub(1)
    };
    buckets
        .get_mut(idx)
        .map(|b| &mut b.entry)
        .ok_or(GroundTrackerTdmError::EmptyMeasurements)
}

fn apply_tdm_observable(
    meta: &TdmMetadata,
    obs: &TdmObservation,
    nominal_carrier_hz: Option<f64>,
    entry: &mut TdmImportedObservation,
) -> Result<(), GroundTrackerTdmError> {
    if !obs.value.is_finite() {
        return Err(GroundTrackerTdmError::NonFinite);
    }
    match &obs.kind {
        TdmObservable::Range => {
            let range_m = match meta.range_units {
                Some(TdmRangeUnits::Seconds) => {
                    let light_m = obs.value * SPEED_OF_LIGHT_M_S;
                    match meta.range_mode {
                        Some(TdmRangeMode::Coherent) => 0.5 * light_m,
                        _ => light_m,
                    }
                }
                _ => obs.value * KM_TO_M,
            };
            entry.range_m = Some(range_m);
        }
        TdmObservable::DopplerInstantaneous | TdmObservable::DopplerIntegrated => {
            entry.range_rate_m_s = Some(obs.value * KM_TO_M);
        }
        TdmObservable::ReceiveFreq(Some(1) | None) => {
            let shift = match (meta.freq_offset, nominal_carrier_hz) {
                (Some(off), Some(nom)) if (off - nom).abs() == 0.0 => obs.value,
                (off, Some(nom)) => obs.value + off.unwrap_or(0.0) - nom,
                _ => obs.value,
            };
            entry.doppler_hz = Some(shift);
        }
        TdmObservable::Angle1 => entry.azimuth_rad = Some(obs.value * DEG_TO_RAD),
        TdmObservable::Angle2 => entry.elevation_rad = Some(obs.value * DEG_TO_RAD),
        _ => {}
    }
    Ok(())
}
