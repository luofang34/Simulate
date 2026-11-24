//! AEM KVN and XML round-trip tests (ADM 504.0-B-2 and 504.0-B-1).

use ccsds_ndm::{
    Aem, AemAttitudeState, AttitudeType, EulerRotSeq, ParseMode, QuaternionType, WriteOptions,
};

const AEM_V2_MULTI: &str = "\
CCSDS_AEM_VERS = 2.0
COMMENT Multi-segment ADM v2 Attitude Ephemeris Message
CREATION_DATE = 2026-07-01T00:00:00
ORIGINATOR = DLR_GSOC

META_START
OBJECT_NAME = ENMAP
OBJECT_ID = 2022-033A
CENTER_NAME = EARTH
REF_FRAME_A = GCRF
REF_FRAME_B = SC_BODY_1
TIME_SYSTEM = UTC
START_TIME = 2026-07-01T00:00:00.000
USEABLE_START_TIME = 2026-07-01T00:01:00.000
USEABLE_STOP_TIME = 2026-07-01T00:09:00.000
STOP_TIME = 2026-07-01T00:10:00.000
ATTITUDE_TYPE = QUATERNION/ANGVEL
ANGVEL_FRAME = SC_BODY_1
INTERPOLATION_METHOD = HERMITE
INTERPOLATION_DEGREE = 5
META_STOP

DATA_START
COMMENT Nominal nadir-pointing pass
2026-07-01T00:00:00.000 0.1 0.2 0.3 0.9273618495495703 0.001 -0.002 0.059
2026-07-01T00:05:00.000 -0.15 0.25 0.35 0.8902246907382428 -0.001 0.003 0.058
DATA_STOP

META_START
OBJECT_NAME = ENMAP
OBJECT_ID = 2022-033A
REF_FRAME_A = EME2000
REF_FRAME_B = SC_BODY_1
TIME_SYSTEM = UTC
START_TIME = 2026-07-01T00:10:00.000
STOP_TIME = 2026-07-01T00:12:00.000
ATTITUDE_TYPE = EULER_ANGLE/ANGVEL
EULER_ROT_SEQ = ZYX
ANGVEL_FRAME = SC_BODY_1
INTERPOLATION_METHOD = LAGRANGE
INTERPOLATION_DEGREE = 3
META_STOP

DATA_START
2026-07-01T00:10:00.000 12.5 -3.25 45.0 0.01 -0.02 0.05
2026-07-01T00:12:00.000 15.5 -2.25 51.0 0.012 -0.018 0.049
DATA_STOP
";

const AEM_V1_QUAT_FIRST: &str = "\
CCSDS_AEM_VERS = 1.0
CREATION_DATE = 2009-05-01T12:00:00
ORIGINATOR = JPL
META_START
OBJECT_NAME = MARS_RECON
OBJECT_ID = 2005-029A
REF_FRAME_A = EME2000
REF_FRAME_B = SC_BODY_1
ATTITUDE_DIR = A2B
TIME_SYSTEM = UTC
START_TIME = 2009-05-01T12:00:00.000
STOP_TIME = 2009-05-01T12:01:00.000
ATTITUDE_TYPE = QUATERNION/DERIVATIVE
QUATERNION_TYPE = FIRST
INTERPOLATION_METHOD = HERMITE
INTERPOLATION_DEGREE = 3
META_STOP
DATA_START
2009-05-01T12:00:00.000 0.9 0.1 0.2 0.3741657386773941 -0.001 0.002 0.003 -0.004
DATA_STOP
";

const AEM_SPIN_KVN: &str = "\
CCSDS_AEM_VERS = 2.0
CREATION_DATE = 2026-07-01T00:00:00
ORIGINATOR = ESA
META_START
OBJECT_NAME = CLUSTER
OBJECT_ID = 2000-041A
REF_FRAME_A = GCRF
REF_FRAME_B = SC_BODY_1
TIME_SYSTEM = UTC
START_TIME = 2026-07-01T01:00:00
STOP_TIME = 2026-07-01T01:05:00
ATTITUDE_TYPE = SPIN/NUTATION
META_STOP
DATA_START
2026-07-01T01:00:00 180.0 23.5 10.0 90.0 0.5 4.0 15.0
DATA_STOP
";

#[test]
fn aem_multi_segment_kvn_and_xml_round_trip() {
    let aem = Aem::from_kvn(AEM_V2_MULTI, ParseMode::Strict).unwrap();
    assert_eq!(aem.segments.len(), 2);
    assert_eq!(
        aem.segments[0].metadata.attitude_type,
        AttitudeType::QuaternionAngVel
    );
    assert_eq!(aem.segments[0].metadata.interpolation_degree, Some(5));
    assert_eq!(
        aem.segments[1].metadata.euler_rot_seq,
        Some(EulerRotSeq::Zyx)
    );
    if let AemAttitudeState::QuaternionAngVel { quaternion, .. } =
        &aem.segments[0].data.states[0].state
    {
        assert_eq!(quaternion.qc.to_bits(), 0.9273618495495703_f64.to_bits());
    } else {
        panic!("expected QuaternionAngVel");
    }
    let kvn = aem.to_kvn(WriteOptions::default()).unwrap();
    assert_eq!(Aem::from_kvn(&kvn, ParseMode::Strict).unwrap(), aem);
    let xml = aem.to_xml(WriteOptions::default()).unwrap();
    let from_xml = Aem::from_xml(&xml, ParseMode::Strict).unwrap();
    assert_eq!(from_xml, aem);
}

#[test]
fn aem_v1_quaternion_first_and_spin_nutation_round_trip() {
    let aem = Aem::from_kvn(AEM_V1_QUAT_FIRST, ParseMode::Strict).unwrap();
    assert_eq!(
        aem.segments[0].metadata.quaternion_type,
        Some(QuaternionType::First)
    );
    let AemAttitudeState::QuaternionDerivative { quaternion, q_dot } =
        &aem.segments[0].data.states[0].state
    else {
        panic!("expected QuaternionDerivative");
    };
    assert_eq!(quaternion.qc, 0.9);
    assert_eq!(quaternion.q1, 0.1);
    assert_eq!(q_dot[3], -0.001);
    assert_eq!(q_dot[0], 0.002);
    let xml = aem.to_xml(WriteOptions::default()).unwrap();
    assert_eq!(Aem::from_xml(&xml, ParseMode::Strict).unwrap(), aem);

    let spin = Aem::from_kvn(AEM_SPIN_KVN, ParseMode::Strict).unwrap();
    let spin_xml = spin.to_xml(WriteOptions::default()).unwrap();
    assert_eq!(Aem::from_xml(&spin_xml, ParseMode::Strict).unwrap(), spin);
}
