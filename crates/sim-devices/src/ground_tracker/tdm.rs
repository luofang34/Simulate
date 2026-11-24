//! Bidirectional CCSDS TDM 503.0-B-2 (`Tdm`) interchange adapter for [`super::GroundTracker`].

mod codec;
mod epoch;

use ccsds_ndm::{Epoch, RefFrame, Tdm, TdmAngleType, TdmDataQuality, TdmRangeUnits, TimeSystem};
use sim_spacetime::{TdbEpoch, TimeError, TimeScaleCatalog};
use thiserror::Error;

use super::{
    GroundTrackerStationSpec, TrackingLinkMode, TrackingMeasurement, TrackingTruthMeasurement,
};

use codec::{ExportRow, build_tdm_from_rows, import_segment_observations};
use epoch::{epoch_to_tdb_impl, tdb_to_epoch_impl};

/// Errors returned when converting between `GroundTracker` measurements and CCSDS `TDM`.
#[derive(Debug, Error)]
pub enum GroundTrackerTdmError {
    /// Underlying CCSDS NDM serialization/validation failure.
    #[error("ccsds error: {0}")]
    Ccsds(#[from] ccsds_ndm::Error),
    /// `sim-spacetime` time-scale conversion failure.
    #[error("time scale error: {0}")]
    Time(#[from] TimeError),
    /// Invalid or out-of-range CCSDS calendar epoch.
    #[error("invalid epoch: {0}")]
    Epoch(String),
    /// CCSDS `TIME_SYSTEM` is not supported by the local time-scale catalog.
    #[error("unsupported CCSDS time system `{0}`")]
    UnsupportedTimeSystem(TimeSystem),
    /// CCSDS `ANGLE_TYPE` cannot be mapped to topocentric `(azimuth, elevation)`.
    #[error("unsupported TDM angle type `{0}` (expected `AZEL`)")]
    UnsupportedAngleType(TdmAngleType),
    /// CCSDS `RANGE_UNITS` cannot be converted unambiguously into metres.
    #[error("unsupported TDM range units `{0}` (expected `km` or `s`)")]
    UnsupportedRangeUnits(TdmRangeUnits),
    /// Encountered a non-finite floating-point value.
    #[error("non-finite value in CCSDS TDM conversion")]
    NonFinite,
    /// No exportable measurements were supplied.
    #[error("cannot export an empty measurement series to CCSDS TDM")]
    EmptyMeasurements,
}

/// Range unit convention selected when exporting `GroundTracker` data to CCSDS `TDM`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TdmRangeExportUnit {
    /// Export `RANGE` in kilometres (`km`: slant range for one-way, half round-trip for two-way).
    Km,
    /// Export `RANGE` in seconds of light time (`s`: one-way or full two-way round-trip).
    Seconds,
}

/// Metadata configuration used to build a CCSDS `TDM` from `GroundTracker` measurements.
#[derive(Clone, Debug, PartialEq)]
pub struct GroundTrackerTdmMeta {
    /// `ORIGINATOR` header string.
    pub originator: String,
    /// Station identifier written as `PARTICIPANT_1`.
    pub station_name: String,
    /// Target spacecraft identifier written as `PARTICIPANT_2`.
    pub target_name: String,
    /// Optional `TRACK_ID` (TDM v2.0).
    pub track_id: Option<String>,
    /// CCSDS `TIME_SYSTEM` used for `START_TIME`, `STOP_TIME`, and observation time tags.
    pub time_system: TimeSystem,
    /// Optional header `CREATION_DATE` (defaults to first measurement epoch when `None`).
    pub creation_date: Option<Epoch>,
    /// Physical tracking link mode (`OneWayDownlink` or `TwoWayCoherent`).
    pub link_mode: TrackingLinkMode,
    /// Optional nominal carrier frequency in Hz (written as `FREQ_OFFSET` with `RECEIVE_FREQ_1`).
    pub carrier_frequency_hz: Option<f64>,
    /// Output unit convention for `RANGE`.
    pub range_unit: TdmRangeExportUnit,
    /// Whether to include observations marked `below_mask == true`.
    pub include_below_mask: bool,
    /// Optional `DATA_QUALITY` metadata flag.
    pub data_quality: Option<TdmDataQuality>,
    /// Optional `REFERENCE_FRAME` metadata entry.
    pub reference_frame: Option<RefFrame>,
}

impl GroundTrackerTdmMeta {
    /// Builds default TDM metadata for a [`GroundTrackerStationSpec`].
    #[must_use]
    pub fn for_station(
        originator: impl Into<String>,
        station_name: impl Into<String>,
        target_name: impl Into<String>,
        station: &GroundTrackerStationSpec,
        link_mode: TrackingLinkMode,
    ) -> Self {
        let carrier = (station.carrier_frequency_hz.is_finite()
            && station.carrier_frequency_hz > 0.0)
            .then_some(station.carrier_frequency_hz);
        Self {
            originator: originator.into(),
            station_name: station_name.into(),
            target_name: target_name.into(),
            track_id: None,
            time_system: TimeSystem::Utc,
            creation_date: None,
            link_mode,
            carrier_frequency_hz: carrier,
            range_unit: TdmRangeExportUnit::Km,
            include_below_mask: false,
            data_quality: Some(TdmDataQuality::Validated),
            reference_frame: None,
        }
    }
}

/// Imported observables at a single epoch extracted from a CCSDS `TDM`.
#[derive(Clone, Debug, PartialEq)]
pub struct TdmImportedObservation {
    /// Observation receive epoch resolved into [`TdbEpoch`].
    pub epoch: TdbEpoch,
    /// Slant or two-way equivalent one-way range in metres (`RANGE`), if present.
    pub range_m: Option<f64>,
    /// Instantaneous or integrated range rate in m/s (`DOPPLER_*`), if present.
    pub range_rate_m_s: Option<f64>,
    /// Sky Doppler frequency shift in Hz (`RECEIVE_FREQ*` relative to nominal carrier), if present.
    pub doppler_hz: Option<f64>,
    /// Topocentric azimuth angle in radians (`ANGLE_1` for `AZEL`), if present.
    pub azimuth_rad: Option<f64>,
    /// Topocentric elevation angle in radians (`ANGLE_2` for `AZEL`), if present.
    pub elevation_rad: Option<f64>,
}

/// Bidirectional converters between [`super::GroundTracker`] measurements and [`ccsds_ndm::Tdm`].
pub struct GroundTrackerTdm;

impl GroundTrackerTdm {
    /// Exports simulated [`TrackingMeasurement`] samples into a CCSDS [`Tdm`].
    pub fn measurements_to_tdm(
        meta: &GroundTrackerTdmMeta,
        measurements: &[TrackingMeasurement],
        scales: &TimeScaleCatalog,
    ) -> Result<Tdm, GroundTrackerTdmError> {
        let rows: Vec<ExportRow> = measurements
            .iter()
            .filter(|m| meta.include_below_mask || !m.below_mask)
            .map(|m| ExportRow {
                rx_epoch: m.truth.rx_epoch,
                range_m: m.range_m,
                range_rate_m_s: m.range_rate_m_s,
                doppler_hz: m.doppler_hz,
                azimuth_rad: m.azimuth_rad,
                elevation_rad: m.elevation_rad,
            })
            .collect();
        build_tdm_from_rows(meta, &rows, scales)
    }

    /// Exports noiseless [`TrackingTruthMeasurement`] samples into a CCSDS [`Tdm`].
    pub fn truth_measurements_to_tdm(
        meta: &GroundTrackerTdmMeta,
        measurements: &[TrackingTruthMeasurement],
        scales: &TimeScaleCatalog,
    ) -> Result<Tdm, GroundTrackerTdmError> {
        let rows: Vec<ExportRow> = measurements
            .iter()
            .map(|m| ExportRow {
                rx_epoch: m.rx_epoch,
                range_m: m.range_m,
                range_rate_m_s: m.range_rate_m_s,
                doppler_hz: m.doppler_hz,
                azimuth_rad: m.azimuth_rad,
                elevation_rad: m.elevation_rad,
            })
            .collect();
        build_tdm_from_rows(meta, &rows, scales)
    }

    /// Imports tracking observables from every segment of a CCSDS [`Tdm`].
    pub fn tdm_to_observations(
        tdm: &Tdm,
        nominal_carrier_hz: Option<f64>,
        scales: &TimeScaleCatalog,
    ) -> Result<Vec<TdmImportedObservation>, GroundTrackerTdmError> {
        let mut out = Vec::new();
        for seg in &tdm.segments {
            import_segment_observations(seg, nominal_carrier_hz, scales, &mut out)?;
        }
        Ok(out)
    }

    /// Converts a CCSDS [`Epoch`] in `time_system` into a [`TdbEpoch`].
    pub fn epoch_to_tdb(
        epoch: &Epoch,
        time_system: TimeSystem,
        scales: &TimeScaleCatalog,
    ) -> Result<TdbEpoch, GroundTrackerTdmError> {
        epoch_to_tdb_impl(epoch, time_system, scales)
    }

    /// Converts a [`TdbEpoch`] into a CCSDS [`Epoch`] in `time_system`.
    pub fn tdb_to_epoch(
        tdb: TdbEpoch,
        time_system: TimeSystem,
        scales: &TimeScaleCatalog,
    ) -> Result<Epoch, GroundTrackerTdmError> {
        tdb_to_epoch_impl(tdb, time_system, scales)
    }
}
