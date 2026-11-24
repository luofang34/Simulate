//! TDM enumerations and observable types (CCSDS 503.0-B-2 / 503.0-B-1).

use std::fmt;

use crate::open_enum::open_enum;

open_enum! {
    /// Tracking link configuration (`MODE`).
    TdmMode {
        /// Sequential link along `PATH`.
        Sequential => "SEQUENTIAL",
        /// Single-differenced link along `PATH_1` and `PATH_2`.
        SingleDiff => "SINGLE_DIFF",
    }
}

open_enum! {
    /// Time-tag reference point (`TIMETAG_REF`).
    TdmTimetagRef {
        /// Time tag refers to transmission epoch.
        Transmit => "TRANSMIT",
        /// Time tag refers to reception epoch.
        Receive => "RECEIVE",
    }
}

open_enum! {
    /// Integration interval reference point (`INTEGRATION_REF`).
    TdmIntegrationRef {
        /// Integration interval start.
        Start => "START",
        /// Integration interval midpoint.
        Middle => "MIDDLE",
        /// Integration interval end.
        End => "END",
    }
}

open_enum! {
    /// Ranging mode (`RANGE_MODE`).
    TdmRangeMode {
        /// Coherent two-way/multi-way ranging.
        Coherent => "COHERENT",
        /// Constant-frequency ranging.
        Constant => "CONSTANT",
        /// One-way ranging.
        OneWay => "ONE_WAY",
    }
}

open_enum! {
    /// Range observable units (`RANGE_UNITS`).
    TdmRangeUnits {
        /// Kilometers (or round-trip half-light-time equivalent km).
        Km => "km",
        /// Seconds of round-trip or one-way light time.
        Seconds => "s",
        /// Range Units (`RU`).
        Ru => "RU",
    }
}

open_enum! {
    /// Tracking angle coordinate pair (`ANGLE_TYPE`).
    TdmAngleType {
        /// Azimuth and elevation (`AZEL`).
        Azel => "AZEL",
        /// Right ascension and declination (`RADEC`).
        Radec => "RADEC",
        /// X-East / Y-North (`XEYN`).
        Xeyn => "XEYN",
        /// X-South / Y-East (`XSYE`).
        Xsye => "XSYE",
    }
}

open_enum! {
    /// Tracking data quality flag (`DATA_QUALITY`).
    TdmDataQuality {
        /// Uncalibrated/unvalidated raw measurements.
        Raw => "RAW",
        /// Validated measurements.
        Validated => "VALIDATED",
        /// Degraded measurements.
        Degraded => "DEGRADED",
    }
}

/// Tracking observable keyword inside a TDM data block.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum TdmObservable {
    /// `RANGE` (in `RANGE_UNITS`).
    Range,
    /// `DOPPLER_INSTANTANEOUS` (`km/s`).
    DopplerInstantaneous,
    /// `DOPPLER_INTEGRATED` (`km/s`).
    DopplerIntegrated,
    /// `DOPPLER_COUNT` (dimensionless cycles).
    DopplerCount,
    /// `ANGLE_1` (`deg`).
    Angle1,
    /// `ANGLE_2` (`deg`).
    Angle2,
    /// `PC_N0` (`dB-Hz`).
    PcN0,
    /// `PR_N0` (`dB-Hz`).
    PrN0,
    /// `CARRIER_POWER` (`dBW` / `dBm`).
    CarrierPower,
    /// `CLOCK_BIAS` (`s`).
    ClockBias,
    /// `CLOCK_DRIFT` (`s/s`).
    ClockDrift,
    /// `DOR` (`s`).
    Dor,
    /// `VLBI_DELAY` (`s`).
    VlbiDelay,
    /// `STEC` (`TECU`).
    Stec,
    /// `TROPO_DRY` (`m`).
    TropoDry,
    /// `TROPO_WET` (`m`).
    TropoWet,
    /// `PRESSURE` (`hPa`).
    Pressure,
    /// `RHUMIDITY` (`%`).
    Rhumidity,
    /// `TEMPERATURE` (`K`).
    Temperature,
    /// `MAG` (visual magnitude).
    Mag,
    /// `RCS` (`m**2`).
    Rcs,
    /// `TRANSMIT_FREQ_n` (`Hz`, `n` in `1..=5`).
    TransmitFreq(u8),
    /// `TRANSMIT_FREQ_RATE_n` (`Hz/s`, `n` in `1..=5`).
    TransmitFreqRate(u8),
    /// `TRANSMIT_PHASE_CT_n` (cycles, `n` in `1..=5`).
    TransmitPhaseCt(u8),
    /// `RECEIVE_FREQ` (`None`) or `RECEIVE_FREQ_n` (`Some(n)` for `n` in `1..=5`, `Hz`).
    ReceiveFreq(Option<u8>),
    /// `RECEIVE_PHASE_CT_n` (cycles, `n` in `1..=5`).
    ReceivePhaseCt(u8),
    /// Extension or unrecognized observable keyword (allowed in lenient mode).
    Other(String),
}

impl TdmObservable {
    /// Parses a TDM data keyword, returning [`Self::Other`] if non-standard.
    pub fn from_keyword(keyword: &str) -> Self {
        match keyword {
            "RANGE" => Self::Range,
            "DOPPLER_INSTANTANEOUS" => Self::DopplerInstantaneous,
            "DOPPLER_INTEGRATED" => Self::DopplerIntegrated,
            "DOPPLER_COUNT" => Self::DopplerCount,
            "ANGLE_1" => Self::Angle1,
            "ANGLE_2" => Self::Angle2,
            "PC_N0" => Self::PcN0,
            "PR_N0" => Self::PrN0,
            "CARRIER_POWER" => Self::CarrierPower,
            "CLOCK_BIAS" => Self::ClockBias,
            "CLOCK_DRIFT" => Self::ClockDrift,
            "DOR" => Self::Dor,
            "VLBI_DELAY" => Self::VlbiDelay,
            "STEC" => Self::Stec,
            "TROPO_DRY" => Self::TropoDry,
            "TROPO_WET" => Self::TropoWet,
            "PRESSURE" => Self::Pressure,
            "RHUMIDITY" => Self::Rhumidity,
            "TEMPERATURE" => Self::Temperature,
            "MAG" => Self::Mag,
            "RCS" => Self::Rcs,
            "RECEIVE_FREQ" => Self::ReceiveFreq(None),
            _ => {
                parse_indexed_observable(keyword).unwrap_or_else(|| Self::Other(keyword.to_owned()))
            }
        }
    }

    /// Keyword spelling for KVN and XML output.
    pub fn as_str(&self) -> String {
        match self {
            Self::Range => "RANGE".to_owned(),
            Self::DopplerInstantaneous => "DOPPLER_INSTANTANEOUS".to_owned(),
            Self::DopplerIntegrated => "DOPPLER_INTEGRATED".to_owned(),
            Self::DopplerCount => "DOPPLER_COUNT".to_owned(),
            Self::Angle1 => "ANGLE_1".to_owned(),
            Self::Angle2 => "ANGLE_2".to_owned(),
            Self::PcN0 => "PC_N0".to_owned(),
            Self::PrN0 => "PR_N0".to_owned(),
            Self::CarrierPower => "CARRIER_POWER".to_owned(),
            Self::ClockBias => "CLOCK_BIAS".to_owned(),
            Self::ClockDrift => "CLOCK_DRIFT".to_owned(),
            Self::Dor => "DOR".to_owned(),
            Self::VlbiDelay => "VLBI_DELAY".to_owned(),
            Self::Stec => "STEC".to_owned(),
            Self::TropoDry => "TROPO_DRY".to_owned(),
            Self::TropoWet => "TROPO_WET".to_owned(),
            Self::Pressure => "PRESSURE".to_owned(),
            Self::Rhumidity => "RHUMIDITY".to_owned(),
            Self::Temperature => "TEMPERATURE".to_owned(),
            Self::Mag => "MAG".to_owned(),
            Self::Rcs => "RCS".to_owned(),
            Self::TransmitFreq(n) => format!("TRANSMIT_FREQ_{n}"),
            Self::TransmitFreqRate(n) => format!("TRANSMIT_FREQ_RATE_{n}"),
            Self::TransmitPhaseCt(n) => format!("TRANSMIT_PHASE_CT_{n}"),
            Self::ReceiveFreq(None) => "RECEIVE_FREQ".to_owned(),
            Self::ReceiveFreq(Some(n)) => format!("RECEIVE_FREQ_{n}"),
            Self::ReceivePhaseCt(n) => format!("RECEIVE_PHASE_CT_{n}"),
            Self::Other(s) => s.clone(),
        }
    }

    /// Whether this observable keyword is part of CCSDS 503.0-B-2.
    pub const fn is_standard(&self) -> bool {
        !matches!(self, Self::Other(_))
    }
}

impl fmt::Display for TdmObservable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.as_str())
    }
}

fn parse_indexed_observable(keyword: &str) -> Option<TdmObservable> {
    if let Some(n) = parse_suffix_1_to_5(keyword, "TRANSMIT_FREQ_") {
        return Some(TdmObservable::TransmitFreq(n));
    }
    if let Some(n) = parse_suffix_1_to_5(keyword, "TRANSMIT_FREQ_RATE_") {
        return Some(TdmObservable::TransmitFreqRate(n));
    }
    if let Some(n) = parse_suffix_1_to_5(keyword, "TRANSMIT_PHASE_CT_") {
        return Some(TdmObservable::TransmitPhaseCt(n));
    }
    if let Some(n) = parse_suffix_1_to_5(keyword, "RECEIVE_FREQ_") {
        return Some(TdmObservable::ReceiveFreq(Some(n)));
    }
    if let Some(n) = parse_suffix_1_to_5(keyword, "RECEIVE_PHASE_CT_") {
        return Some(TdmObservable::ReceivePhaseCt(n));
    }
    None
}

fn parse_suffix_1_to_5(keyword: &str, prefix: &str) -> Option<u8> {
    let rest = keyword.strip_prefix(prefix)?;
    let n: u8 = rest.parse().ok()?;
    (1..=5).contains(&n).then_some(n)
}
