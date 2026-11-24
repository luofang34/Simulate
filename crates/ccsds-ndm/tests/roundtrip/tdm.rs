//! TDM KVN and XML round-trip tests (CCSDS 503.0-B-2 and 503.0-B-1).

use ccsds_ndm::{
    ParseMode, Tdm, TdmAngleType, TdmIntegrationRef, TdmMode, TdmObservable, TdmRangeMode,
    TdmRangeUnits, TdmTimetagRef, WriteOptions,
};

const TDM_KVN: &str = "\
CCSDS_TDM_VERS = 2.0
COMMENT Two-way coherent range, Doppler, angles, and calibrations
CREATION_DATE = 2026-09-01T00:00:00.000
ORIGINATOR = GOLDSTONE_DSN

META_START
TRACK_ID = PASS-2026-0901-A
DATA_TYPES = RANGE,DOPPLER_INTEGRATED,ANGLE_1,ANGLE_2,PC_N0,TROPO_DRY,CLOCK_BIAS
TIME_SYSTEM = UTC
START_TIME = 2026-09-01T01:00:00.000
STOP_TIME = 2026-09-01T01:05:00.000
PARTICIPANT_1 = DSS-14
PARTICIPANT_2 = LUNAR-RELAY-1
MODE = SEQUENTIAL
PATH = 1,2,1
TRANSMIT_BAND = X
RECEIVE_BAND = X
TURNAROUND_NUMERATOR = 880
TURNAROUND_DENOMINATOR = 749
TIMETAG_REF = RECEIVE
INTEGRATION_INTERVAL = 1.0
INTEGRATION_REF = MIDDLE
RANGE_MODE = COHERENT
RANGE_UNITS = km
ANGLE_TYPE = AZEL
REFERENCE_FRAME = ITRF2000
TRANSMIT_DELAY_1 = 0.00000125
RECEIVE_DELAY_1 = 0.00000150
DATA_QUALITY = VALIDATED
CORRECTIONS_APPLIED = YES
META_STOP

DATA_START
TRANSMIT_FREQ_1 = 2026-09-01T01:00:00.000 7180000000.0
TRANSMIT_FREQ_RATE_1 = 2026-09-01T01:00:00.000 -12.5
RECEIVE_FREQ_1 = 2026-09-01T01:00:02.500 8435123456.789
RANGE = 2026-09-01T01:00:02.500 384402.12575
DOPPLER_INTEGRATED = 2026-09-01T01:00:02.500 -0.4521875
DOPPLER_INSTANTANEOUS = 2026-09-01T01:00:02.500 -0.4521900
ANGLE_1 = 2026-09-01T01:00:02.500 142.375
ANGLE_2 = 2026-09-01T01:00:02.500 38.625
PC_N0 = 2026-09-01T01:00:02.500 44.5
PR_N0 = 2026-09-01T01:00:02.500 31.25
CARRIER_POWER = 2026-09-01T01:00:02.500 -132.5
STEC = 2026-09-01T01:00:02.500 14.2
TROPO_DRY = 2026-09-01T01:00:02.500 2.31
TROPO_WET = 2026-09-01T01:00:02.500 0.14
CLOCK_BIAS = 2026-09-01T01:00:02.500 0.00000012
CLOCK_DRIFT = 2026-09-01T01:00:02.500 -1.2e-12
DATA_STOP
";

#[test]
fn tdm_kvn_and_xml_round_trip() {
    let tdm = Tdm::from_kvn(TDM_KVN, ParseMode::Strict).unwrap();
    assert_eq!(tdm.segments.len(), 1);
    let meta = &tdm.segments[0].metadata;
    assert_eq!(meta.mode, Some(TdmMode::Sequential));
    assert_eq!(meta.timetag_ref, Some(TdmTimetagRef::Receive));
    assert_eq!(meta.integration_ref, Some(TdmIntegrationRef::Middle));
    assert_eq!(meta.range_mode, Some(TdmRangeMode::Coherent));
    assert_eq!(meta.range_units, Some(TdmRangeUnits::Km));
    assert_eq!(meta.angle_type, Some(TdmAngleType::Azel));
    assert_eq!(meta.turnaround_numerator, Some(880));
    assert_eq!(tdm.segments[0].data.observations.len(), 16);
    assert_eq!(
        tdm.segments[0].data.observations[3].kind,
        TdmObservable::Range
    );
    assert_eq!(
        tdm.segments[0].data.observations[3].value.to_bits(),
        384402.12575_f64.to_bits()
    );

    let kvn = tdm.to_kvn(WriteOptions::default()).unwrap();
    assert_eq!(Tdm::from_kvn(&kvn, ParseMode::Strict).unwrap(), tdm);
    let xml = tdm.to_xml(WriteOptions::default()).unwrap();
    assert_eq!(Tdm::from_xml(&xml, ParseMode::Strict).unwrap(), tdm);
}

#[test]
fn tdm_multi_observable_xml_block_and_strict_checks() {
    let xml = "\
<tdm id=\"CCSDS_TDM_VERS\" version=\"1.0\">
  <header>
    <CREATION_DATE>2026-09-01T00:00:00</CREATION_DATE>
    <ORIGINATOR>TEST</ORIGINATOR>
  </header>
  <body>
    <segment>
      <metadata>
        <TIME_SYSTEM>UTC</TIME_SYSTEM>
        <PARTICIPANT_1>STA-1</PARTICIPANT_1>
        <PARTICIPANT_2>SAT-1</PARTICIPANT_2>
        <ANGLE_TYPE>AZEL</ANGLE_TYPE>
      </metadata>
      <data>
        <observation>
          <EPOCH>2026-09-01T00:01:00.000</EPOCH>
          <ANGLE_1>120.5</ANGLE_1>
          <ANGLE_2>25.25</ANGLE_2>
        </observation>
      </data>
    </segment>
  </body>
</tdm>";
    let tdm = Tdm::from_xml(xml, ParseMode::Strict).unwrap();
    assert_eq!(tdm.segments[0].data.observations.len(), 2);
    assert_eq!(
        tdm.segments[0].data.observations[0].kind,
        TdmObservable::Angle1
    );
    assert_eq!(
        tdm.segments[0].data.observations[1].kind,
        TdmObservable::Angle2
    );

    let unknown_obs = TDM_KVN.replace("CLOCK_DRIFT =", "CUSTOM_OBS =");
    assert!(Tdm::from_kvn(&unknown_obs, ParseMode::Strict).is_err());
    let lenient = Tdm::from_kvn(&unknown_obs, ParseMode::Lenient).unwrap();
    assert!(matches!(
        lenient.segments[0].data.observations.last().unwrap().kind,
        TdmObservable::Other(ref s) if s == "CUSTOM_OBS"
    ));
}
