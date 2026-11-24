//! APM KVN and XML round-trip tests (ADM 504.0-B-2 and 504.0-B-1).

use ccsds_ndm::{Apm, CenterName, EulerRotSeq, ParseMode, RefFrame, TimeSystem, WriteOptions};

const APM_V2_KVN: &str = "\
CCSDS_APM_VERS = 2.0
COMMENT Full ADM v2 APM round-trip fixture
CREATION_DATE = 2026-06-15T12:00:00.000Z
ORIGINATOR = SIMULATE_GNC
MESSAGE_ID = APM-2026-001
OBJECT_NAME = SENTINEL-6A
OBJECT_ID = 2020-086A
CENTER_NAME = EARTH
TIME_SYSTEM = UTC
EPOCH = 2026-06-15T12:30:00.123456
REF_FRAME_A = GCRF
REF_FRAME_B = SC_BODY_1
Q1 = 0.1234567890123456
Q2 = -0.2345678901234567
Q3 = 0.3456789012345678
QC = 0.9001234567890123
Q1_DOT = 0.00012 [1/s]
Q2_DOT = -0.00034 [1/s]
Q3_DOT = 0.00056 [1/s]
QC_DOT = -0.00004 [1/s]
REF_FRAME_A = GCRF
REF_FRAME_B = SC_BODY_1
EULER_ROT_SEQ = ZYX
RATE_FRAME = SC_BODY_1
ANGLE_1 = 15.25 [deg]
ANGLE_2 = -4.5 [deg]
ANGLE_3 = 179.75 [deg]
ANGLE_1_DOT = 0.01 [deg/s]
ANGLE_2_DOT = -0.02 [deg/s]
ANGLE_3_DOT = 0.03 [deg/s]
ANGVEL_FRAME = SC_BODY_1
ANGVEL_X = 0.11 [deg/s]
ANGVEL_Y = -0.22 [deg/s]
ANGVEL_Z = 0.33 [deg/s]
INERTIA_REF_FRAME = SC_BODY_1
IXX = 1250.5 [kg*m**2]
IYY = 2310.25 [kg*m**2]
IZZ = 2890.75 [kg*m**2]
IXY = -12.125 [kg*m**2]
IXZ = 4.5 [kg*m**2]
IYZ = -8.25 [kg*m**2]
MAN_EPOCH_START = 2026-06-15T12:31:00.000
MAN_DURATION = 42.5 [s]
MAN_REF_FRAME = SC_BODY_1
MAN_TOR_1 = 0.05 [N*m]
MAN_TOR_2 = -0.025 [N*m]
MAN_TOR_3 = 0.0125 [N*m]
MAN_DELTA_MASS = -0.15 [kg]
USER_DEFINED_MODE = SLEW
";

const APM_V1_KVN: &str = "\
CCSDS_APM_VERS = 1.0
CREATION_DATE = 2008-09-01T10:00:00
ORIGINATOR = CNES
OBJECT_NAME = JASON-2
OBJECT_ID = 2008-032A
CENTER_NAME = EARTH
TIME_SYSTEM = UTC
EPOCH = 2008-09-01T10:05:00
Q_FRAME_A = EME2000
Q_FRAME_B = SC_BODY_1
Q_DIR = A2B
Q1 = 0.0
Q2 = 0.5
Q3 = 0.0
QC = 0.8660254037844386
SPIN_FRAME_A = EME2000
SPIN_FRAME_B = SC_BODY_1
SPIN_DIR = A2B
SPIN_ALPHA = 120.0 [deg]
SPIN_DELTA = -15.5 [deg]
SPIN_ANGLE = 45.0 [deg]
SPIN_ANGLE_VEL = 18.0 [deg/s]
NUTATION = 1.25 [deg]
NUTATION_PER = 32.0 [s]
NUTATION_PHASE = 90.0 [deg]
I11 = 420.0 [kg*m**2]
I22 = 510.0 [kg*m**2]
I33 = 610.0 [kg*m**2]
I12 = 1.5 [kg*m**2]
I13 = -2.5 [kg*m**2]
I23 = 0.75 [kg*m**2]
";

#[test]
fn apm_v2_kvn_and_xml_round_trip() {
    let apm = Apm::from_kvn(APM_V2_KVN, ParseMode::Strict).unwrap();
    assert_eq!(apm.metadata.center_name, Some(CenterName::Earth));
    assert_eq!(apm.metadata.time_system, TimeSystem::Utc);
    let q = apm.data.quaternion.as_ref().unwrap();
    assert_eq!(q.ref_frame_a, RefFrame::Gcrf);
    assert_eq!(q.quaternion.qc.to_bits(), 0.9001234567890123_f64.to_bits());
    assert_eq!(apm.data.euler.as_ref().unwrap().rot_seq, EulerRotSeq::Zyx);
    assert_eq!(apm.data.maneuvers.len(), 1);
    assert_eq!(apm.data.user_defined.as_ref().unwrap().parameters.len(), 1);

    for opts in [
        WriteOptions::default(),
        WriteOptions::default().with_units(true),
    ] {
        let kvn = apm.to_kvn(opts).unwrap();
        let from_kvn = Apm::from_kvn(&kvn, ParseMode::Strict).unwrap();
        assert_eq!(from_kvn, apm);
        let xml = apm.to_xml(opts).unwrap();
        let from_xml = Apm::from_xml(&xml, ParseMode::Strict).unwrap();
        assert_eq!(from_xml, apm);
    }
}

#[test]
fn apm_v1_spin_and_inertia_round_trip() {
    let apm = Apm::from_kvn(APM_V1_KVN, ParseMode::Strict).unwrap();
    let spin = apm.data.spin.as_ref().unwrap();
    assert_eq!(spin.spin_alpha.value(), 120.0);
    assert_eq!(spin.nutation.unwrap().value(), 1.25);
    let inertia = apm.data.inertia.as_ref().unwrap();
    assert_eq!(inertia.ixx.value(), 420.0);
    let xml = apm
        .to_xml(WriteOptions::default().with_units(true))
        .unwrap();
    assert!(xml.contains("eulerElementsSpin"));
    assert!(xml.contains("spacecraftParameters"));
    let from_xml = Apm::from_xml(&xml, ParseMode::Strict).unwrap();
    assert_eq!(from_xml, apm);
    let back_kvn = from_xml.to_kvn(WriteOptions::default()).unwrap();
    assert_eq!(Apm::from_kvn(&back_kvn, ParseMode::Strict).unwrap(), apm);
}

#[test]
fn apm_validation_and_lenient_extensions() {
    let missing_state = "\
CCSDS_APM_VERS = 2.0
CREATION_DATE = 2026-06-15T12:00:00
ORIGINATOR = TEST
OBJECT_NAME = SAT
OBJECT_ID = 2026-001A
TIME_SYSTEM = UTC
EPOCH = 2026-06-15T12:30:00
";
    assert!(Apm::from_kvn(missing_state, ParseMode::Strict).is_err());

    let extra = format!("{APM_V2_KVN}AGENCY_FLAG = NOMINAL\n");
    assert!(Apm::from_kvn(&extra, ParseMode::Strict).is_err());
    let lenient = Apm::from_kvn(&extra, ParseMode::Lenient).unwrap();
    assert_eq!(lenient.extensions.len(), 1);
    assert_eq!(lenient.extensions[0].keyword, "AGENCY_FLAG");
}
