//! Tracking Data Message (TDM 503.0-B-2 / 503.0-B-1): tracking observables
//! (range, Doppler, angles, signal power, media/clock calibrations) in KVN and XML.

mod enums;
mod kvn;
mod layout;
mod xml;

use crate::error::Result;
use crate::extension::Extension;
use crate::frame::{Interpolation, RefFrame, YesNo};
use crate::header::{Header, Version};
use crate::time_system::TimeSystem;
use crate::{Epoch, ParseMode, WriteOptions, xml as xml_codec};

pub use enums::{
    TdmAngleType, TdmDataQuality, TdmIntegrationRef, TdmMode, TdmObservable, TdmRangeMode,
    TdmRangeUnits, TdmTimetagRef,
};

/// A Tracking Data Message.
#[derive(Debug, Clone, PartialEq)]
pub struct Tdm {
    /// `CCSDS_TDM_VERS` (`1.0` or `2.0`).
    pub version: Version,
    /// The header.
    pub header: Header,
    /// Metadata + tracking data segments.
    pub segments: Vec<TdmSegment>,
    /// Undefined keywords kept in lenient mode.
    pub extensions: Vec<Extension>,
}

/// One metadata + observation segment of a [`Tdm`].
#[derive(Debug, Clone, PartialEq)]
pub struct TdmSegment {
    /// Segment metadata (`META_START` .. `META_STOP`).
    pub metadata: TdmMetadata,
    /// Interleaved tracking observations (`DATA_START` .. `DATA_STOP`).
    pub data: TdmData,
}

/// Metadata for one TDM segment.
#[derive(Debug, Clone, PartialEq)]
pub struct TdmMetadata {
    /// Comments inside the metadata block.
    pub comments: Vec<String>,
    /// Optional track identifier (`TRACK_ID`, TDM v2).
    pub track_id: Option<String>,
    /// Optional comma-separated list of included observables (`DATA_TYPES`, TDM v2).
    pub data_types: Option<Vec<String>>,
    /// `TIME_SYSTEM`.
    pub time_system: TimeSystem,
    /// `START_TIME`.
    pub start_time: Option<Epoch>,
    /// `STOP_TIME`.
    pub stop_time: Option<Epoch>,
    /// `PARTICIPANT_1` through `PARTICIPANT_5` (`PARTICIPANT_1` is required).
    pub participants: [Option<String>; 5],
    /// `MODE` (`SEQUENTIAL` or `SINGLE_DIFF`).
    pub mode: Option<TdmMode>,
    /// `PATH` (e.g. `"1,2,1"`).
    pub path: Option<String>,
    /// `PATH_1` for `SINGLE_DIFF`.
    pub path_1: Option<String>,
    /// `PATH_2` for `SINGLE_DIFF`.
    pub path_2: Option<String>,
    /// `EPHEMERIS_NAME_1` through `EPHEMERIS_NAME_5`.
    pub ephemeris_names: [Option<String>; 5],
    /// `TRANSMIT_BAND`.
    pub transmit_band: Option<String>,
    /// `RECEIVE_BAND`.
    pub receive_band: Option<String>,
    /// `TURNAROUND_NUMERATOR`.
    pub turnaround_numerator: Option<u32>,
    /// `TURNAROUND_DENOMINATOR`.
    pub turnaround_denominator: Option<u32>,
    /// `TIMETAG_REF` (`TRANSMIT` or `RECEIVE`).
    pub timetag_ref: Option<TdmTimetagRef>,
    /// `INTEGRATION_INTERVAL` (`s`).
    pub integration_interval: Option<f64>,
    /// `INTEGRATION_REF` (`START`, `MIDDLE`, or `END`).
    pub integration_ref: Option<TdmIntegrationRef>,
    /// `FREQ_OFFSET` (`Hz`).
    pub freq_offset: Option<f64>,
    /// `RANGE_MODE` (`COHERENT`, `CONSTANT`, or `ONE_WAY`).
    pub range_mode: Option<TdmRangeMode>,
    /// `RANGE_MODULUS` (in `RANGE_UNITS`).
    pub range_modulus: Option<f64>,
    /// `RANGE_UNITS` (`km`, `s`, or `RU`).
    pub range_units: Option<TdmRangeUnits>,
    /// `ANGLE_TYPE` (`AZEL`, `RADEC`, `XEYN`, `XSYE`).
    pub angle_type: Option<TdmAngleType>,
    /// `REFERENCE_FRAME`.
    pub reference_frame: Option<RefFrame>,
    /// `INTERPOLATION`.
    pub interpolation: Option<Interpolation>,
    /// `INTERPOLATION_DEGREE`.
    pub interpolation_degree: Option<u32>,
    /// `DOPPLER_COUNT_BIAS` (`Hz`).
    pub doppler_count_bias: Option<f64>,
    /// `DOPPLER_COUNT_SCALE`.
    pub doppler_count_scale: Option<f64>,
    /// `DOPPLER_COUNT_ROLLOVER` (`YES`/`NO`).
    pub doppler_count_rollover: Option<YesNo>,
    /// `TRANSMIT_DELAY_1` through `TRANSMIT_DELAY_5` (`s`).
    pub transmit_delays: [Option<f64>; 5],
    /// `RECEIVE_DELAY_1` through `RECEIVE_DELAY_5` (`s`).
    pub receive_delays: [Option<f64>; 5],
    /// `DATA_QUALITY` (`RAW`, `VALIDATED`, `DEGRADED`).
    pub data_quality: Option<TdmDataQuality>,
    /// `CORRECTION_ANGLE_1` (`deg`).
    pub correction_angle_1: Option<f64>,
    /// `CORRECTION_ANGLE_2` (`deg`).
    pub correction_angle_2: Option<f64>,
    /// `CORRECTION_DOPPLER` (`km/s`).
    pub correction_doppler: Option<f64>,
    /// `CORRECTION_MAG` (magnitude).
    pub correction_mag: Option<f64>,
    /// `CORRECTION_RANGE` (in `RANGE_UNITS`).
    pub correction_range: Option<f64>,
    /// `CORRECTION_RCS` (`m**2`).
    pub correction_rcs: Option<f64>,
    /// `CORRECTION_RECEIVE` (`Hz` or `s`).
    pub correction_receive: Option<f64>,
    /// `CORRECTION_TRANSMIT` (`Hz` or `s`).
    pub correction_transmit: Option<f64>,
    /// `CORRECTION_ABERRATION_YEARLY` (`YES`/`NO`).
    pub correction_aberration_yearly: Option<YesNo>,
    /// `CORRECTION_ABERRATION_DIURNAL` (`YES`/`NO`).
    pub correction_aberration_diurnal: Option<YesNo>,
    /// `CORRECTIONS_APPLIED` (`YES`/`NO`).
    pub corrections_applied: Option<YesNo>,
}

/// Tracking data section of a [`TdmSegment`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TdmData {
    /// Comments attached to the data block.
    pub comments: Vec<String>,
    /// Ordered tracking observations.
    pub observations: Vec<TdmObservation>,
}

/// One epoch-stamped tracking observable in a [`TdmData`].
#[derive(Debug, Clone, PartialEq)]
pub struct TdmObservation {
    /// Measurement time tag (in segment `TIME_SYSTEM`).
    pub epoch: Epoch,
    /// Observable type (`RANGE`, `DOPPLER_INTEGRATED`, `ANGLE_1`, ...).
    pub kind: TdmObservable,
    /// Measured scalar value.
    pub value: f64,
}

impl Tdm {
    /// Parses a TDM from KVN text.
    pub fn from_kvn(text: &str, mode: ParseMode) -> Result<Self> {
        kvn::read(text, mode)
    }

    /// Parses a TDM from XML text.
    pub fn from_xml(text: &str, mode: ParseMode) -> Result<Self> {
        let root = xml_codec::tree::parse(text)?;
        Self::from_element(&root, mode)
    }

    pub(crate) fn from_element(root: &xml_codec::tree::Element, mode: ParseMode) -> Result<Self> {
        xml::read(root, mode)
    }

    /// Formats the TDM as KVN text.
    pub fn to_kvn(&self, options: WriteOptions) -> Result<String> {
        kvn::write(self, options)
    }

    /// Formats the TDM as a standalone XML document.
    pub fn to_xml(&self, options: WriteOptions) -> Result<String> {
        let mut out = xml_codec::writer::XmlWriter::document();
        self.write_element(&mut out, options, true)?;
        Ok(out.finish())
    }

    pub(crate) fn write_element(
        &self,
        out: &mut xml_codec::writer::XmlWriter,
        options: WriteOptions,
        standalone: bool,
    ) -> Result<()> {
        xml::write(self, out, options, standalone)
    }
}
