//! CDM KVN and XML round-trip tests (CCSDS 508.0-B-1).

use ccsds_ndm::{Cdm, CdmObject, ParseMode, RefFrame, WriteOptions, YesNo};

const CDM_KVN: &str = "\
CCSDS_CDM_VERS = 1.0
COMMENT Conjunction Data Message Blue Book style test
CREATION_DATE = 2026-08-10T18:00:00.000
ORIGINATOR = 18_SDS
MESSAGE_FOR = SENTINEL-6A
MESSAGE_ID = CDM-20260810-0042

TCA = 2026-08-12T04:15:32.125
MISS_DISTANCE = 142.5 [m]
RELATIVE_SPEED = 14512.75 [m/s]
RELATIVE_POSITION_R = 24.5 [m]
RELATIVE_POSITION_T = -112.25 [m]
RELATIVE_POSITION_N = 84.0 [m]
RELATIVE_VELOCITY_R = -12.5 [m/s]
RELATIVE_VELOCITY_T = -14400.0 [m/s]
RELATIVE_VELOCITY_N = 1805.25 [m/s]
START_SCREEN_PERIOD = 2026-08-11T00:00:00.000
STOP_SCREEN_PERIOD = 2026-08-14T00:00:00.000
SCREEN_VOLUME_FRAME = RTN
SCREEN_VOLUME_SHAPE = ELLIPSOID
SCREEN_VOLUME_X = 500.0 [m]
SCREEN_VOLUME_Y = 2000.0 [m]
SCREEN_VOLUME_Z = 500.0 [m]
COLLISION_PROBABILITY = 0.0004125
COLLISION_PROBABILITY_METHOD = ALFANO-2005

OBJECT = OBJECT1
OBJECT_DESIGNATOR = 46984
CATALOG_NAME = SATCAT
OBJECT_NAME = SENTINEL-6A
INTERNATIONAL_DESIGNATOR = 2020-086A
OBJECT_TYPE = PAYLOAD
EPHEMERIS_NAME = EPHEM_46984
COVARIANCE_METHOD = CALCULATED
MANEUVERABLE = YES
ORBIT_CENTER = EARTH
REF_FRAME = EME2000
GRAVITY_MODEL = EGM-96: 36D 36O
SOLAR_RAD_PRESSURE = YES
EARTH_TIDES = YES
INTRACK_THRUST = NO
TIME_LASTOB_START = 2026-08-10T00:00:00.000
TIME_LASTOB_END = 2026-08-10T17:30:00.000
ACTUAL_OD_SPAN = 3.5 [d]
OBS_AVAILABLE = 210
OBS_USED = 204
RESIDUALS_ACCEPTED = 97.14 [%]
WEIGHTED_RMS = 0.85
AREA_PC = 12.4 [m**2]
MASS = 1192.0 [kg]
CD_AREA_OVER_MASS = 0.022 [m**2/kg]
SEDR = 0.000125 [W/kg]
X = 6878.137 [km]
Y = 125.5 [km]
Z = -2100.25 [km]
X_DOT = 1.25 [km/s]
Y_DOT = 7.125 [km/s]
Z_DOT = 2.0125 [km/s]
CR_R = 100.0 [m**2]
CT_R = 5.0 [m**2]
CT_T = 400.0 [m**2]
CN_R = -2.0 [m**2]
CN_T = 1.0 [m**2]
CN_N = 90.0 [m**2]
CRDOT_R = 0.01 [m**2/s]
CRDOT_T = -0.02 [m**2/s]
CRDOT_N = 0.003 [m**2/s]
CRDOT_RDOT = 0.0001 [m**2/s**2]
CTDOT_R = 0.004 [m**2/s]
CTDOT_T = -0.015 [m**2/s]
CTDOT_N = 0.002 [m**2/s]
CTDOT_RDOT = 0.00001 [m**2/s**2]
CTDOT_TDOT = 0.0002 [m**2/s**2]
CNDOT_R = -0.001 [m**2/s]
CNDOT_T = 0.002 [m**2/s]
CNDOT_N = 0.005 [m**2/s]
CNDOT_RDOT = -0.00002 [m**2/s**2]
CNDOT_TDOT = 0.00003 [m**2/s**2]
CNDOT_NDOT = 0.00008 [m**2/s**2]

OBJECT = OBJECT2
OBJECT_DESIGNATOR = 99001
CATALOG_NAME = SATCAT
OBJECT_NAME = COSMOS-DEB
INTERNATIONAL_DESIGNATOR = 1998-067XY
OBJECT_TYPE = DEBRIS
EPHEMERIS_NAME = NONE
COVARIANCE_METHOD = CALCULATED
MANEUVERABLE = NO
ORBIT_CENTER = EARTH
REF_FRAME = EME2000
X = 6878.1125 [km]
Y = 125.61225 [km]
Z = -2100.334 [km]
X_DOT = -1.25 [km/s]
Y_DOT = -7.125 [km/s]
Z_DOT = 2.0125 [km/s]
CR_R = 250.0 [m**2]
CT_R = 0.0 [m**2]
CT_T = 900.0 [m**2]
CN_R = 0.0 [m**2]
CN_T = 0.0 [m**2]
CN_N = 180.0 [m**2]
CRDOT_R = 0.0 [m**2/s]
CRDOT_T = 0.0 [m**2/s]
CRDOT_N = 0.0 [m**2/s]
CRDOT_RDOT = 0.001 [m**2/s**2]
CTDOT_R = 0.0 [m**2/s]
CTDOT_T = 0.0 [m**2/s]
CTDOT_N = 0.0 [m**2/s]
CTDOT_RDOT = 0.0 [m**2/s**2]
CTDOT_TDOT = 0.002 [m**2/s**2]
CNDOT_R = 0.0 [m**2/s]
CNDOT_T = 0.0 [m**2/s]
CNDOT_N = 0.0 [m**2/s]
CNDOT_RDOT = 0.0 [m**2/s**2]
CNDOT_TDOT = 0.0 [m**2/s**2]
CNDOT_NDOT = 0.0005 [m**2/s**2]
USER_DEFINED_SCREENING_RUN = PROD-7
";

#[test]
fn cdm_kvn_and_xml_round_trip() {
    let cdm = Cdm::from_kvn(CDM_KVN, ParseMode::Strict).unwrap();
    assert_eq!(cdm.header.message_for.as_deref(), Some("SENTINEL-6A"));
    assert_eq!(cdm.relative_metadata.miss_distance.value(), 142.5);
    assert_eq!(cdm.relative_metadata.collision_probability, Some(0.0004125));
    assert_eq!(cdm.object1.metadata.object, CdmObject::Object1);
    assert_eq!(cdm.object1.metadata.solar_rad_pressure, Some(YesNo::Yes));
    assert_eq!(cdm.object1.metadata.ref_frame, RefFrame::Eme2000);
    assert_eq!(cdm.object2.metadata.object, CdmObject::Object2);
    assert_eq!(
        cdm.object1.data.od_parameters.as_ref().unwrap().obs_used,
        Some(204)
    );
    assert_eq!(cdm.object1.data.covariance.matrix.get(2, 2), Some(90.0));
    assert_eq!(cdm.user_defined.as_ref().unwrap().parameters.len(), 1);

    for opts in [
        WriteOptions::default(),
        WriteOptions::default().with_units(true),
    ] {
        let kvn = cdm.to_kvn(opts).unwrap();
        assert_eq!(Cdm::from_kvn(&kvn, ParseMode::Strict).unwrap(), cdm);
        let xml = cdm.to_xml(opts).unwrap();
        let from_xml = Cdm::from_xml(&xml, ParseMode::Strict).unwrap();
        assert_eq!(from_xml, cdm);
        let back_kvn = from_xml.to_kvn(opts).unwrap();
        assert_eq!(Cdm::from_kvn(&back_kvn, ParseMode::Strict).unwrap(), cdm);
    }
}

#[test]
fn cdm_9x9_solve_for_covariance_round_trips() {
    let mut cdm = Cdm::from_kvn(CDM_KVN, ParseMode::Strict).unwrap();
    cdm.object1.data.covariance.drag_row = Some([1e-3, 2e-3, 3e-3, 4e-6, 5e-6, 6e-6, 7e-4]);
    cdm.object1.data.covariance.srp_row = Some([1e-4, 2e-4, 3e-4, 4e-7, 5e-7, 6e-7, 7e-8, 8e-5]);
    cdm.object1.data.covariance.thrust_row =
        Some([1e-5, 2e-5, 3e-5, 4e-8, 5e-8, 6e-8, 7e-9, 8e-9, 9e-6]);
    let opts = WriteOptions::default().with_units(true);
    let kvn = cdm.to_kvn(opts).unwrap();
    assert!(kvn.contains("CDRG_DRG"));
    assert!(kvn.contains("CTHR_THR"));
    let from_kvn = Cdm::from_kvn(&kvn, ParseMode::Strict).unwrap();
    assert_eq!(from_kvn, cdm);
    let xml = from_kvn.to_xml(opts).unwrap();
    assert_eq!(Cdm::from_xml(&xml, ParseMode::Strict).unwrap(), cdm);
}

#[test]
fn cdm_rejects_bad_probability_or_wrong_unit_in_strict() {
    let bad_prob = CDM_KVN.replace(
        "COLLISION_PROBABILITY = 0.0004125",
        "COLLISION_PROBABILITY = 1.5",
    );
    assert!(Cdm::from_kvn(&bad_prob, ParseMode::Strict).is_err());

    let bad_unit = CDM_KVN.replace("CR_R = 100.0 [m**2]", "CR_R = 100.0 [km**2]");
    assert!(Cdm::from_kvn(&bad_unit, ParseMode::Strict).is_err());
    assert!(Cdm::from_kvn(&bad_unit, ParseMode::Lenient).is_ok());

    let swapped = CDM_KVN
        .replace("OBJECT = OBJECT1", "OBJECT = TMP_OBJ")
        .replace("OBJECT = OBJECT2", "OBJECT = OBJECT1")
        .replace("OBJECT = TMP_OBJ", "OBJECT = OBJECT2");
    assert!(Cdm::from_kvn(&swapped, ParseMode::Strict).is_err());
}
