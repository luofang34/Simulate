//! Shared TDM metadata and version validation helpers for KVN and XML.

use super::{
    TdmAngleType, TdmDataQuality, TdmIntegrationRef, TdmMetadata, TdmMode, TdmObservable,
    TdmRangeMode, TdmRangeUnits, TdmTimetagRef,
};
use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::error::{Error, ErrorKind, Location, Result};
use crate::extension::Extension;
use crate::header::Version;
use crate::{ParseMode, WriteOptions, number};

pub(super) const VERSION_KEY: &str = "CCSDS_TDM_VERS";

const PARTICIPANT_KEYS: [&str; 5] = [
    "PARTICIPANT_1",
    "PARTICIPANT_2",
    "PARTICIPANT_3",
    "PARTICIPANT_4",
    "PARTICIPANT_5",
];

const EPHEMERIS_KEYS: [&str; 5] = [
    "EPHEMERIS_NAME_1",
    "EPHEMERIS_NAME_2",
    "EPHEMERIS_NAME_3",
    "EPHEMERIS_NAME_4",
    "EPHEMERIS_NAME_5",
];

const TX_DELAY_KEYS: [&str; 5] = [
    "TRANSMIT_DELAY_1",
    "TRANSMIT_DELAY_2",
    "TRANSMIT_DELAY_3",
    "TRANSMIT_DELAY_4",
    "TRANSMIT_DELAY_5",
];

const RX_DELAY_KEYS: [&str; 5] = [
    "RECEIVE_DELAY_1",
    "RECEIVE_DELAY_2",
    "RECEIVE_DELAY_3",
    "RECEIVE_DELAY_4",
    "RECEIVE_DELAY_5",
];

pub(super) fn check_tdm_version(entry: &Entry, mode: ParseMode) -> Result<Version> {
    let version = Version::parse(entry, mode)?;
    if version == Version::V3 {
        return Err(entry.error(ErrorKind::UnsupportedVersion {
            text: entry.value.clone(),
        }));
    }
    Ok(version)
}

pub(super) fn read_metadata(
    block: Block,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<TdmMetadata> {
    let at = block.at;
    let (comments, mut fields) = Fields::new(block, mode)?;
    let meta = TdmMetadata {
        comments,
        track_id: fields.optional("TRACK_ID", Entry::string)?,
        data_types: fields.optional("DATA_TYPES", parse_data_types)?,
        time_system: fields.required("TIME_SYSTEM", Entry::time_system)?,
        start_time: fields.optional("START_TIME", Entry::epoch)?,
        stop_time: fields.optional("STOP_TIME", Entry::epoch)?,
        participants: read_opt_strings(&mut fields, PARTICIPANT_KEYS)?,
        mode: fields.optional("MODE", parse_mode)?,
        path: fields.optional("PATH", Entry::string)?,
        path_1: fields.optional("PATH_1", Entry::string)?,
        path_2: fields.optional("PATH_2", Entry::string)?,
        ephemeris_names: read_opt_strings(&mut fields, EPHEMERIS_KEYS)?,
        transmit_band: fields.optional("TRANSMIT_BAND", Entry::string)?,
        receive_band: fields.optional("RECEIVE_BAND", Entry::string)?,
        turnaround_numerator: fields.optional("TURNAROUND_NUMERATOR", parse_pos_u32)?,
        turnaround_denominator: fields.optional("TURNAROUND_DENOMINATOR", parse_pos_u32)?,
        timetag_ref: fields.optional("TIMETAG_REF", parse_timetag_ref)?,
        integration_interval: fields.optional("INTEGRATION_INTERVAL", Entry::real)?,
        integration_ref: fields.optional("INTEGRATION_REF", parse_integration_ref)?,
        freq_offset: fields.optional("FREQ_OFFSET", Entry::real)?,
        range_mode: fields.optional("RANGE_MODE", parse_range_mode)?,
        range_modulus: fields.optional("RANGE_MODULUS", Entry::real)?,
        range_units: fields.optional("RANGE_UNITS", parse_range_units)?,
        angle_type: fields.optional("ANGLE_TYPE", parse_angle_type)?,
        reference_frame: fields.optional("REFERENCE_FRAME", Entry::frame)?,
        interpolation: fields.optional("INTERPOLATION", |e, _| {
            Ok(crate::Interpolation::from_name(&e.value))
        })?,
        interpolation_degree: fields.optional("INTERPOLATION_DEGREE", parse_pos_u32)?,
        doppler_count_bias: fields.optional("DOPPLER_COUNT_BIAS", Entry::real)?,
        doppler_count_scale: fields.optional("DOPPLER_COUNT_SCALE", Entry::real)?,
        doppler_count_rollover: fields.optional("DOPPLER_COUNT_ROLLOVER", Entry::yes_no)?,
        transmit_delays: read_opt_reals(&mut fields, TX_DELAY_KEYS)?,
        receive_delays: read_opt_reals(&mut fields, RX_DELAY_KEYS)?,
        data_quality: fields.optional("DATA_QUALITY", parse_data_quality)?,
        correction_angle_1: fields.optional("CORRECTION_ANGLE_1", Entry::real)?,
        correction_angle_2: fields.optional("CORRECTION_ANGLE_2", Entry::real)?,
        correction_doppler: fields.optional("CORRECTION_DOPPLER", Entry::real)?,
        correction_mag: fields.optional("CORRECTION_MAG", Entry::real)?,
        correction_range: fields.optional("CORRECTION_RANGE", Entry::real)?,
        correction_rcs: fields.optional("CORRECTION_RCS", Entry::real)?,
        correction_receive: fields.optional("CORRECTION_RECEIVE", Entry::real)?,
        correction_transmit: fields.optional("CORRECTION_TRANSMIT", Entry::real)?,
        correction_aberration_yearly: fields
            .optional("CORRECTION_ABERRATION_YEARLY", Entry::yes_no)?,
        correction_aberration_diurnal: fields
            .optional("CORRECTION_ABERRATION_DIURNAL", Entry::yes_no)?,
        corrections_applied: fields.optional("CORRECTIONS_APPLIED", Entry::yes_no)?,
    };
    fields.finish(extensions)?;
    if mode == ParseMode::Strict {
        let [p1, ..] = &meta.participants;
        if p1.is_none() {
            let err = Error::new(ErrorKind::MissingKeyword).with_keyword("PARTICIPANT_1");
            return Err(match at {
                Some(loc) => err.at(loc),
                None => err,
            });
        }
    }
    Ok(meta)
}

pub(super) fn write_metadata(meta: &TdmMetadata, options: WriteOptions) -> Result<Block> {
    let mut out = BlockWriter::new(&meta.comments, options);
    out.text_opt("TRACK_ID", meta.track_id.as_ref());
    if let Some(types) = &meta.data_types {
        out.text("DATA_TYPES", types.join(","));
    }
    out.text("TIME_SYSTEM", meta.time_system);
    out.text_opt("START_TIME", meta.start_time.as_ref());
    out.text_opt("STOP_TIME", meta.stop_time.as_ref());
    write_opt_strings(&mut out, PARTICIPANT_KEYS, &meta.participants);
    out.text_opt("MODE", meta.mode.as_ref());
    out.text_opt("PATH", meta.path.as_ref());
    out.text_opt("PATH_1", meta.path_1.as_ref());
    out.text_opt("PATH_2", meta.path_2.as_ref());
    write_opt_strings(&mut out, EPHEMERIS_KEYS, &meta.ephemeris_names);
    write_link_fields(&mut out, meta)?;
    write_delay_and_corr_fields(&mut out, meta)?;
    Ok(out.finish())
}

fn write_link_fields(out: &mut BlockWriter, meta: &TdmMetadata) -> Result<()> {
    out.text_opt("TRANSMIT_BAND", meta.transmit_band.as_ref());
    out.text_opt("RECEIVE_BAND", meta.receive_band.as_ref());
    out.text_opt("TURNAROUND_NUMERATOR", meta.turnaround_numerator);
    out.text_opt("TURNAROUND_DENOMINATOR", meta.turnaround_denominator);
    out.text_opt("TIMETAG_REF", meta.timetag_ref.as_ref());
    out.real_opt("INTEGRATION_INTERVAL", meta.integration_interval)?;
    out.text_opt("INTEGRATION_REF", meta.integration_ref.as_ref());
    out.real_opt("FREQ_OFFSET", meta.freq_offset)?;
    out.text_opt("RANGE_MODE", meta.range_mode.as_ref());
    out.real_opt("RANGE_MODULUS", meta.range_modulus)?;
    out.text_opt("RANGE_UNITS", meta.range_units.as_ref());
    out.text_opt("ANGLE_TYPE", meta.angle_type.as_ref());
    out.text_opt("REFERENCE_FRAME", meta.reference_frame.as_ref());
    out.text_opt("INTERPOLATION", meta.interpolation.as_ref());
    out.text_opt("INTERPOLATION_DEGREE", meta.interpolation_degree);
    out.real_opt("DOPPLER_COUNT_BIAS", meta.doppler_count_bias)?;
    out.real_opt("DOPPLER_COUNT_SCALE", meta.doppler_count_scale)?;
    out.text_opt("DOPPLER_COUNT_ROLLOVER", meta.doppler_count_rollover);
    Ok(())
}

fn write_delay_and_corr_fields(out: &mut BlockWriter, meta: &TdmMetadata) -> Result<()> {
    write_opt_reals(out, TX_DELAY_KEYS, &meta.transmit_delays)?;
    write_opt_reals(out, RX_DELAY_KEYS, &meta.receive_delays)?;
    out.text_opt("DATA_QUALITY", meta.data_quality.as_ref());
    out.real_opt("CORRECTION_ANGLE_1", meta.correction_angle_1)?;
    out.real_opt("CORRECTION_ANGLE_2", meta.correction_angle_2)?;
    out.real_opt("CORRECTION_DOPPLER", meta.correction_doppler)?;
    out.real_opt("CORRECTION_MAG", meta.correction_mag)?;
    out.real_opt("CORRECTION_RANGE", meta.correction_range)?;
    out.real_opt("CORRECTION_RCS", meta.correction_rcs)?;
    out.real_opt("CORRECTION_RECEIVE", meta.correction_receive)?;
    out.real_opt("CORRECTION_TRANSMIT", meta.correction_transmit)?;
    out.text_opt(
        "CORRECTION_ABERRATION_YEARLY",
        meta.correction_aberration_yearly,
    );
    out.text_opt(
        "CORRECTION_ABERRATION_DIURNAL",
        meta.correction_aberration_diurnal,
    );
    out.text_opt("CORRECTIONS_APPLIED", meta.corrections_applied);
    Ok(())
}

fn read_opt_strings(fields: &mut Fields, keys: [&str; 5]) -> Result<[Option<String>; 5]> {
    let [k1, k2, k3, k4, k5] = keys;
    Ok([
        fields.optional(k1, Entry::string)?,
        fields.optional(k2, Entry::string)?,
        fields.optional(k3, Entry::string)?,
        fields.optional(k4, Entry::string)?,
        fields.optional(k5, Entry::string)?,
    ])
}

fn read_opt_reals(fields: &mut Fields, keys: [&str; 5]) -> Result<[Option<f64>; 5]> {
    let [k1, k2, k3, k4, k5] = keys;
    Ok([
        fields.optional(k1, Entry::real)?,
        fields.optional(k2, Entry::real)?,
        fields.optional(k3, Entry::real)?,
        fields.optional(k4, Entry::real)?,
        fields.optional(k5, Entry::real)?,
    ])
}

fn write_opt_strings(out: &mut BlockWriter, keys: [&str; 5], vals: &[Option<String>; 5]) {
    let [k1, k2, k3, k4, k5] = keys;
    let [v1, v2, v3, v4, v5] = vals;
    out.text_opt(k1, v1.as_ref());
    out.text_opt(k2, v2.as_ref());
    out.text_opt(k3, v3.as_ref());
    out.text_opt(k4, v4.as_ref());
    out.text_opt(k5, v5.as_ref());
}

fn write_opt_reals(out: &mut BlockWriter, keys: [&str; 5], vals: &[Option<f64>; 5]) -> Result<()> {
    let [k1, k2, k3, k4, k5] = keys;
    let [v1, v2, v3, v4, v5] = *vals;
    out.real_opt(k1, v1)?;
    out.real_opt(k2, v2)?;
    out.real_opt(k3, v3)?;
    out.real_opt(k4, v4)?;
    out.real_opt(k5, v5)?;
    Ok(())
}

fn parse_data_types(entry: &Entry, _: ParseMode) -> Result<Vec<String>> {
    let items: Vec<String> = entry
        .value
        .split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(str::to_owned)
        .collect();
    if items.is_empty() {
        return Err(entry.error(ErrorKind::EmptyValue));
    }
    Ok(items)
}

fn parse_pos_u32(entry: &Entry, _: ParseMode) -> Result<u32> {
    let val: u32 = entry.value.trim().parse().map_err(|_| {
        entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "a positive integer",
        })
    })?;
    if val == 0 {
        return Err(entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "an integer >= 1",
        }));
    }
    Ok(val)
}

fn parse_mode(entry: &Entry, mode: ParseMode) -> Result<TdmMode> {
    let val = TdmMode::from_name(&entry.value);
    if mode == ParseMode::Strict && matches!(val, TdmMode::Other(_)) {
        return Err(entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "SEQUENTIAL or SINGLE_DIFF",
        }));
    }
    Ok(val)
}

fn parse_timetag_ref(entry: &Entry, mode: ParseMode) -> Result<TdmTimetagRef> {
    let val = TdmTimetagRef::from_name(&entry.value);
    if mode == ParseMode::Strict && matches!(val, TdmTimetagRef::Other(_)) {
        return Err(entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "TRANSMIT or RECEIVE",
        }));
    }
    Ok(val)
}

fn parse_integration_ref(entry: &Entry, mode: ParseMode) -> Result<TdmIntegrationRef> {
    let val = TdmIntegrationRef::from_name(&entry.value);
    if mode == ParseMode::Strict && matches!(val, TdmIntegrationRef::Other(_)) {
        return Err(entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "START, MIDDLE, or END",
        }));
    }
    Ok(val)
}

fn parse_range_mode(entry: &Entry, mode: ParseMode) -> Result<TdmRangeMode> {
    let val = TdmRangeMode::from_name(&entry.value);
    if mode == ParseMode::Strict && matches!(val, TdmRangeMode::Other(_)) {
        return Err(entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "COHERENT, CONSTANT, or ONE_WAY",
        }));
    }
    Ok(val)
}

fn parse_range_units(entry: &Entry, mode: ParseMode) -> Result<TdmRangeUnits> {
    let val = TdmRangeUnits::from_name(&entry.value);
    if mode == ParseMode::Strict && matches!(val, TdmRangeUnits::Other(_)) {
        return Err(entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "km, s, or RU",
        }));
    }
    Ok(val)
}

fn parse_angle_type(entry: &Entry, mode: ParseMode) -> Result<TdmAngleType> {
    let val = TdmAngleType::from_name(&entry.value);
    if mode == ParseMode::Strict && matches!(val, TdmAngleType::Other(_)) {
        return Err(entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "AZEL, RADEC, XEYN, or XSYE",
        }));
    }
    Ok(val)
}

fn parse_data_quality(entry: &Entry, mode: ParseMode) -> Result<TdmDataQuality> {
    let val = TdmDataQuality::from_name(&entry.value);
    if mode == ParseMode::Strict && matches!(val, TdmDataQuality::Other(_)) {
        return Err(entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "RAW, VALIDATED, or DEGRADED",
        }));
    }
    Ok(val)
}

pub(super) fn parse_observable_keyword(entry: &Entry, mode: ParseMode) -> Result<TdmObservable> {
    let kind = TdmObservable::from_keyword(&entry.keyword);
    if mode == ParseMode::Strict && !kind.is_standard() {
        return Err(entry.error(ErrorKind::UnknownKeyword));
    }
    Ok(kind)
}

pub(super) fn parse_scalar_value(
    token: &str,
    keyword: &str,
    at: Option<Location>,
    mode: ParseMode,
) -> Result<f64> {
    number::parse_f64(token, mode).ok_or_else(|| {
        let err = Error::new(ErrorKind::InvalidNumber {
            text: token.to_owned(),
        })
        .with_keyword(keyword.to_owned());
        match at {
            Some(loc) => err.at(loc),
            None => err,
        }
    })
}
