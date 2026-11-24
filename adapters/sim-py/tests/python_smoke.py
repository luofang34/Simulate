#!/usr/bin/env python3
"""End-to-end smoke verification for the `sim_py` PyO3 extension module."""

import math
import sim_py


MU_EARTH_M3_S2 = 3.986_004_418e14
R_WGS84_M = 6_378_137.0


OPM_KVN = """\
CCSDS_OPM_VERS = 3.0
CREATION_DATE = 2026-06-01T00:00:00
ORIGINATOR = SIMULATE
OBJECT_NAME = SENTINEL-6A
OBJECT_ID = 2020-086A
CENTER_NAME = EARTH
REF_FRAME = ICRF
TIME_SYSTEM = UTC
EPOCH = 2026-06-01T12:00:00Z
X = 6878.137 [km]
Y = 125.5 [km]
Z = -2100.25 [km]
X_DOT = 1.25 [km/s]
Y_DOT = 7.125 [km/s]
Z_DOT = 2.0125 [km/s]
"""

OMM_KVN = """\
CCSDS_OMM_VERS = 3.0
CREATION_DATE = 2026-06-01T00:00:00
ORIGINATOR = SIMULATE
OBJECT_NAME = ISS (ZARYA)
OBJECT_ID = 1998-067A
CENTER_NAME = EARTH
REF_FRAME = TEME
TIME_SYSTEM = UTC
MEAN_ELEMENT_THEORY = SGP4
EPOCH = 2026-06-01T00:00:00.000000
MEAN_MOTION = 15.49560532
ECCENTRICITY = 0.0001771
INCLINATION = 51.6435
RA_OF_ASC_NODE = 225.4004
ARG_OF_PERICENTER = 44.9625
MEAN_ANOMALY = 5.1087
EPHEMERIS_TYPE = 0
CLASSIFICATION_TYPE = U
NORAD_CAT_ID = 25544
ELEMENT_SET_NO = 999
REV_AT_EPOCH = 55000
BSTAR = 0.0001027
MEAN_MOTION_DOT = 0.00016717
MEAN_MOTION_DDOT = 0.0
"""

OMM_JSON = """\
{
  "OBJECT_NAME": "ISS (ZARYA)",
  "OBJECT_ID": "1998-067A",
  "EPOCH": "2026-06-01T00:00:00.000000",
  "MEAN_MOTION": 15.49560532,
  "ECCENTRICITY": 0.0001771,
  "INCLINATION": 51.6435,
  "RA_OF_ASC_NODE": 225.4004,
  "ARG_OF_PERICENTER": 44.9625,
  "MEAN_ANOMALY": 5.1087,
  "EPHEMERIS_TYPE": 0,
  "CLASSIFICATION_TYPE": "U",
  "NORAD_CAT_ID": 25544,
  "ELEMENT_SET_NO": 999,
  "REV_AT_EPOCH": 55000,
  "BSTAR": 0.0001027,
  "MEAN_MOTION_DOT": 0.00016717,
  "MEAN_MOTION_DDOT": 0.0
}
"""

OEM_KVN = """\
CCSDS_OEM_VERS = 3.0
CREATION_DATE = 2026-06-01T00:00:00
ORIGINATOR = SIMULATE
META_START
OBJECT_NAME = DEMO-SAT
OBJECT_ID = 2026-001A
CENTER_NAME = EARTH
REF_FRAME = GCRF
TIME_SYSTEM = TDB
START_TIME = 2026-06-01T00:00:00
STOP_TIME = 2026-06-01T00:01:00
INTERPOLATION = LAGRANGE
INTERPOLATION_DEGREE = 1
META_STOP
2026-06-01T00:00:00 7000.0 0.0 0.0 0.0 7.546 0.0
2026-06-01T00:01:00 6985.0 452.0 0.0 -0.487 7.530 0.0
"""

APM_KVN = """\
CCSDS_APM_VERS = 2.0
CREATION_DATE = 2026-06-01T00:00:00
ORIGINATOR = SIMULATE
OBJECT_NAME = DEMO-SAT
OBJECT_ID = 2026-001A
CENTER_NAME = EARTH
TIME_SYSTEM = UTC
EPOCH = 2026-06-01T00:00:00
REF_FRAME_A = GCRF
REF_FRAME_B = SC_BODY_1
Q1 = 0.0
Q2 = 0.0
Q3 = 0.0
QC = 1.0
ANGVEL_FRAME = SC_BODY_1
ANGVEL_X = 0.01
ANGVEL_Y = -0.02
ANGVEL_Z = 0.03
INERTIA_REF_FRAME = SC_BODY_1
IXX = 1200.0
IYY = 2100.0
IZZ = 2800.0
IXY = 0.0
IXZ = 0.0
IYZ = 0.0
"""

AEM_KVN = """\
CCSDS_AEM_VERS = 2.0
CREATION_DATE = 2026-06-01T00:00:00
ORIGINATOR = SIMULATE
META_START
OBJECT_NAME = DEMO-SAT
OBJECT_ID = 2026-001A
CENTER_NAME = EARTH
REF_FRAME_A = GCRF
REF_FRAME_B = SC_BODY_1
TIME_SYSTEM = UTC
START_TIME = 2026-06-01T00:00:00
STOP_TIME = 2026-06-01T00:01:00
ATTITUDE_TYPE = QUATERNION/ANGVEL
RATE_FRAME = SC_BODY_1
INTERPOLATION_METHOD = HERMITE
INTERPOLATION_DEGREE = 1
META_STOP
DATA_START
2026-06-01T00:00:00 0.0 0.0 0.0 1.0 0.01 -0.02 0.03
2026-06-01T00:01:00 0.0 0.0 0.0 1.0 0.01 -0.02 0.03
DATA_STOP
"""

CDM_KVN = """\
CCSDS_CDM_VERS = 1.0
CREATION_DATE = 2026-06-01T00:00:00
ORIGINATOR = SIMULATE
MESSAGE_FOR = SENTINEL-6A
MESSAGE_ID = CDM-SMOKE-001
TCA = 2026-06-01T12:00:00
MISS_DISTANCE = 25.0 [m]
RELATIVE_SPEED = 14000.0 [m/s]
COLLISION_PROBABILITY = 0.00125
OBJECT = OBJECT1
OBJECT_DESIGNATOR = 46984
CATALOG_NAME = SATCAT
OBJECT_NAME = SENTINEL-6A
INTERNATIONAL_DESIGNATOR = 2020-086A
EPHEMERIS_NAME = SIMULATE
COVARIANCE_METHOD = CALCULATED
MANEUVERABLE = YES
ORBIT_CENTER = EARTH
REF_FRAME = EME2000
X = 6878.137 [km]
Y = 0.0 [km]
Z = 0.0 [km]
X_DOT = 0.0 [km/s]
Y_DOT = 7.612 [km/s]
Z_DOT = 0.0 [km/s]
CR_R = 100.0 [m**2]
CT_R = 0.0 [m**2]
CT_T = 200.0 [m**2]
CN_R = 0.0 [m**2]
CN_T = 0.0 [m**2]
CN_N = 80.0 [m**2]
CRDOT_R = 0.0 [m**2/s]
CRDOT_T = 0.0 [m**2/s]
CRDOT_N = 0.0 [m**2/s]
CRDOT_RDOT = 0.01 [m**2/s**2]
CTDOT_R = 0.0 [m**2/s]
CTDOT_T = 0.0 [m**2/s]
CTDOT_N = 0.0 [m**2/s]
CTDOT_RDOT = 0.0 [m**2/s**2]
CTDOT_TDOT = 0.02 [m**2/s**2]
CNDOT_R = 0.0 [m**2/s]
CNDOT_T = 0.0 [m**2/s]
CNDOT_N = 0.0 [m**2/s]
CNDOT_RDOT = 0.0 [m**2/s**2]
CNDOT_TDOT = 0.0 [m**2/s**2]
CNDOT_NDOT = 0.01 [m**2/s**2]
OBJECT = OBJECT2
OBJECT_DESIGNATOR = 99001
CATALOG_NAME = SATCAT
OBJECT_NAME = DEB
INTERNATIONAL_DESIGNATOR = 1999-001B
EPHEMERIS_NAME = SIMULATE
COVARIANCE_METHOD = CALCULATED
MANEUVERABLE = NO
ORBIT_CENTER = EARTH
REF_FRAME = EME2000
X = 6878.152 [km]
Y = 0.0 [km]
Z = 0.02 [km]
X_DOT = 0.0 [km/s]
Y_DOT = -7.612 [km/s]
Z_DOT = 0.5 [km/s]
CR_R = 120.0 [m**2]
CT_R = 0.0 [m**2]
CT_T = 240.0 [m**2]
CN_R = 0.0 [m**2]
CN_T = 0.0 [m**2]
CN_N = 90.0 [m**2]
CRDOT_R = 0.0 [m**2/s]
CRDOT_T = 0.0 [m**2/s]
CRDOT_N = 0.0 [m**2/s]
CRDOT_RDOT = 0.01 [m**2/s**2]
CTDOT_R = 0.0 [m**2/s]
CTDOT_T = 0.0 [m**2/s]
CTDOT_N = 0.0 [m**2/s]
CTDOT_RDOT = 0.0 [m**2/s**2]
CTDOT_TDOT = 0.02 [m**2/s**2]
CNDOT_R = 0.0 [m**2/s]
CNDOT_T = 0.0 [m**2/s]
CNDOT_N = 0.0 [m**2/s]
CNDOT_RDOT = 0.0 [m**2/s**2]
CNDOT_TDOT = 0.0 [m**2/s**2]
CNDOT_NDOT = 0.01 [m**2/s**2]
"""

TDM_KVN = """\
CCSDS_TDM_VERS = 2.0
CREATION_DATE = 2026-09-01T00:00:00.000
ORIGINATOR = GOLDSTONE_DSN
META_START
TRACK_ID = PASS-2026-0901-A
TIME_SYSTEM = UTC
START_TIME = 2026-09-01T01:00:00.000
STOP_TIME = 2026-09-01T01:05:00.000
PARTICIPANT_1 = DSS-14
PARTICIPANT_2 = LUNAR-RELAY-1
MODE = SEQUENTIAL
RANGE_MODE = COHERENT
RANGE_UNITS = km
ANGLE_TYPE = AZEL
META_STOP
DATA_START
RANGE = 2026-09-01T01:00:02.500 384402.12575
ANGLE_1 = 2026-09-01T01:00:02.500 142.375
ANGLE_2 = 2026-09-01T01:00:02.500 38.625
DATA_STOP
"""


def assert_raises_value_error(fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except ValueError:
        return
    raise AssertionError(f"expected ValueError for {fn.__name__}")


def check_time_scales():
    utc_in = "2026-06-01T12:00:00.125Z"
    tdb_s = sim_py.utc_to_tdb_s(utc_in)
    utc_out = sim_py.tdb_to_utc(tdb_s)
    assert utc_out == utc_in, f"roundtrip mismatch: {utc_out} != {utc_in}"
    tdb_again = sim_py.utc_to_tdb_s(utc_out)
    assert abs(tdb_again - tdb_s) < 1e-6

    jd = sim_py.tdb_s_to_jd(0.0)
    assert abs(jd - 2_451_545.0) < 1e-12
    assert abs(sim_py.jd_to_tdb_s(jd)) < 1e-9
    tdb_tai = sim_py.tdb_minus_tai_s(tdb_s)
    assert abs(tdb_tai - 32.184) < 0.003

    assert_raises_value_error(sim_py.utc_to_tdb_s, "not-a-timestamp")
    assert_raises_value_error(sim_py.tdb_to_utc, float("nan"))


def check_kepler_and_bplane():
    r0 = [7_000_000.0, 0.0, 0.0]
    v_circ = math.sqrt(MU_EARTH_M3_S2 / r0[0])
    v0 = [0.0, v_circ, 0.0]
    period = 2.0 * math.pi * math.sqrt(r0[0] ** 3 / MU_EARTH_M3_S2)

    r1, v1 = sim_py.propagate_kepler(r0, v0, MU_EARTH_M3_S2, period)
    r_mag0 = math.sqrt(sum(c * c for c in r0))
    r_mag1 = math.sqrt(sum(c * c for c in r1))
    v_mag0 = math.sqrt(sum(c * c for c in v0))
    v_mag1 = math.sqrt(sum(c * c for c in v1))
    e0 = 0.5 * v_mag0 * v_mag0 - MU_EARTH_M3_S2 / r_mag0
    e1 = 0.5 * v_mag1 * v_mag1 - MU_EARTH_M3_S2 / r_mag1
    assert abs(r_mag1 - r_mag0) < 1e-5
    assert math.sqrt(sum((a - b) ** 2 for a, b in zip(r1, r0))) < 1e-4
    assert abs((e1 - e0) / e0) < 1e-13

    cov = [[0.0] * 6 for _ in range(6)]
    for i in range(3):
        cov[i][i] = 100.0
        cov[i + 3][i + 3] = 0.01
    res = sim_py.bplane_collision_probability(
        [7_000_000.0, 0.0, 0.0],
        [0.0, 7_500.0, 0.0],
        cov,
        [7_000_005.0, 0.0, 0.0],
        [0.0, -7_500.0, 0.0],
        cov,
        15.0,
    )
    assert 0.0 < res["probability"] <= 1.0
    assert abs(res["miss_distance_m"] - 5.0) < 1e-9
    assert len(res["b_plane_sigmas_m"]) == 2
    assert len(res["miss_b_plane_m"]) == 2
    assert res["mahalanobis_distance_sq"] > 0.0
    assert_raises_value_error(
        sim_py.bplane_collision_probability,
        [7_000_000.0, 0.0, 0.0],
        [0.0, 7_500.0, 0.0],
        cov,
        [7_000_005.0, 0.0, 0.0],
        [0.0, -7_500.0, 0.0],
        cov,
        -1.0,
    )


def check_sgp4():
    states_json = sim_py.sgp4_propagate(OMM_JSON, [0.0, 900.0, 5400.0])
    states_kvn = sim_py.sgp4_propagate(OMM_KVN, [0.0, 900.0, 5400.0])
    assert len(states_json) == 3
    for (p1, v1), (p2, v2) in zip(states_json, states_kvn):
        r_km = math.sqrt(sum(c * c for c in p1)) / 1000.0
        v_km_s = math.sqrt(sum(c * c for c in v1)) / 1000.0
        assert 6500.0 < r_km < 7100.0, f"unexpected LEO radius {r_km} km"
        assert 7.2 < v_km_s < 8.0, f"unexpected LEO speed {v_km_s} km/s"
        assert math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2))) < 1e-6
        assert math.sqrt(sum((a - b) ** 2 for a, b in zip(v1, v2))) < 1e-9

    sat = sim_py.Sgp4Satellite(OMM_KVN)
    assert sat.norad_id == 25544
    assert len(sat.elements_hash) == 64
    assert len(sat.propagate([0.0, 60.0])) == 2


def check_ccsds_all_seven():
    opm = sim_py.parse_ccsds_opm(OPM_KVN)
    assert opm["message_type"] == "OPM"
    assert abs(opm["position_km"][0] - 6878.137) < 1e-9
    opm_xml = sim_py.parse_ccsds_opm(opm["xml"])
    assert opm_xml["position_km"] == opm["position_km"]
    opm_state = sim_py.ccsds_opm_to_state(opm["kvn"])
    assert abs(opm_state["pos_m"][0] - 6_878_137.0) < 1e-6

    omm = sim_py.parse_ccsds_omm(OMM_KVN)
    assert omm["norad_cat_id"] == 25544
    omm_xml = sim_py.parse_ccsds_omm(omm["xml"])
    assert abs(omm_xml["inclination_deg"] - 51.6435) < 1e-9
    omm_json = sim_py.parse_ccsds_omm(OMM_JSON, strict=False)
    assert omm_json["norad_cat_id"] == 25544

    oem = sim_py.parse_ccsds_oem(OEM_KVN)
    assert oem["segment_count"] == 1
    assert len(oem["segments"][0]["states"]) == 2
    oem_xml = sim_py.parse_ccsds_oem(oem["xml"])
    assert oem_xml["segments"][0]["states"][1]["position_km"][0] == 6985.0

    apm = sim_py.parse_ccsds_apm(APM_KVN)
    assert apm["quaternion"]["qc"] == 1.0
    assert apm["inertia_kg_m2"]["ixx"] == 1200.0
    apm_xml = sim_py.parse_ccsds_apm(apm["xml"])
    assert apm_xml["ang_vel_deg_s"] == apm["ang_vel_deg_s"]

    aem = sim_py.parse_ccsds_aem(AEM_KVN)
    assert aem["segment_count"] == 1
    assert aem["segments"][0]["state_count"] == 2
    aem_xml = sim_py.parse_ccsds_aem(aem["xml"])
    assert aem_xml["segments"][0]["states"][0]["quaternion_xyzw"] == [0.0, 0.0, 0.0, 1.0]

    cdm = sim_py.parse_ccsds_cdm(CDM_KVN)
    assert abs(cdm["miss_distance_m"] - 25.0) < 1e-9
    cdm_xml = sim_py.parse_ccsds_cdm(cdm["xml"])
    assert abs(cdm_xml["collision_probability"] - 0.00125) < 1e-12

    tdm = sim_py.parse_ccsds_tdm(TDM_KVN)
    assert tdm["segment_count"] == 1
    assert len(tdm["segments"][0]["observations"]) == 3
    tdm_xml = sim_py.parse_ccsds_tdm(tdm["xml"])
    assert abs(tdm_xml["segments"][0]["observations"][0]["value"] - 384402.12575) < 1e-9


def check_spice_spk_and_ck():
    spk_bytes = sim_py.build_hermite_spk(
        "SMOKE-SPK",
        "PROBE-TRAJ",
        -77,
        399,
        1,
        [0.0, 60.0, 120.0, 180.0],
        [
            [7000.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [7060.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [7120.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [7180.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        ],
        window=2,
        comments=["generated in python_smoke.py"],
    )
    info = sim_py.inspect_spice_spk(spk_bytes)
    assert info["internal_name"] == "SMOKE-SPK"
    assert info["segment_count"] == 1
    assert info["segments"][0]["target"] == -77
    assert info["segments"][0]["data_type"] == 13
    pos, vel = sim_py.evaluate_spice_spk(spk_bytes, -77, 399, 90.0)
    assert abs(pos[0] - 7090.0) < 1e-9
    assert abs(vel[0] - 1.0) < 1e-9

    half = math.pi / 4.0
    ck_bytes = sim_py.build_type3_ck(
        "SMOKE-CK",
        "PROBE-ATT",
        -77_000,
        1,
        [0.0, 100.0],
        [
            [1.0, 0.0, 0.0, 0.0],
            [math.cos(half), 0.0, 0.0, -math.sin(half)],
        ],
        angular_velocities=[[0.0, 0.0, 0.015707963267948967]] * 2,
    )
    ck_info = sim_py.inspect_spice_ck(ck_bytes)
    assert ck_info["internal_name"] == "SMOKE-CK"
    assert ck_info["segments"][0]["instrument"] == -77_000
    assert ck_info["segments"][0]["has_angular_velocity"] is True
    pointing = sim_py.evaluate_spice_ck(ck_bytes, -77_000, 50.0, tolerance=0.0)
    assert abs(pointing["sclk"] - 50.0) < 1e-12
    assert len(pointing["c_matrix"]) == 3


def check_h3_coverage_and_monte_carlo():
    r_sat = R_WGS84_M + 1_500_000.0
    v_sat = math.sqrt(MU_EARTH_M3_S2 / r_sat)
    sat1 = ([r_sat, 0.0, 0.0], [0.0, v_sat, 0.0])
    sat2 = ([-r_sat, 0.0, 0.0], [0.0, -v_sat, 0.0])

    rep_a = sim_py.evaluate_h3_coverage(
        [sat1, sat2],
        "2026-06-01T00:00:00Z",
        300.0,
        60.0,
        resolution=1,
        min_elevation_deg=10.0,
        min_fold=1,
        max_cells=2_000,
    )
    rep_b = sim_py.evaluate_h3_coverage(
        [sat1, sat2],
        "2026-06-01T00:00:00Z",
        300.0,
        60.0,
        resolution=1,
        min_elevation_deg=10.0,
        min_fold=1,
        max_cells=2_000,
    )
    assert rep_a["total_cells"] == 842
    assert rep_a == rep_b
    assert 0.0 < rep_a["n_fold_fraction"] < 1.0
    indices = [c["h3_index"] for c in rep_a["cells"]]
    assert indices == sorted(indices)

    rep_box = sim_py.evaluate_h3_coverage(
        [OMM_KVN],
        "2026-06-01T00:00:00Z",
        600.0,
        60.0,
        resolution=2,
        min_elevation_deg=5.0,
        lat_min_deg=-15.0,
        lat_max_deg=15.0,
        lon_min_deg=-20.0,
        lon_max_deg=20.0,
        max_cells=2_000,
    )
    assert rep_box["total_cells"] > 0

    mc1 = sim_py.monte_carlo_sample(12345, "isp_bias", stream_index=1, draw_count=12)
    mc2 = sim_py.monte_carlo_sample(12345, "isp_bias", stream_index=1, draw_count=12)
    mc_diff_seed = sim_py.monte_carlo_sample(12346, "isp_bias", stream_index=1, draw_count=12)
    mc_diff_stream = sim_py.monte_carlo_sample(12345, "isp_bias", stream_index=2, draw_count=12)

    assert mc1["samples"] == mc2["samples"]
    assert mc1["manifest_sha256"] == mc2["manifest_sha256"]
    assert len(mc1["manifest_sha256"]) == 64
    assert mc1["samples"] != mc_diff_seed["samples"]
    assert mc1["manifest_sha256"] != mc_diff_seed["manifest_sha256"]
    assert mc1["samples"] != mc_diff_stream["samples"]
    assert mc1["manifest_sha256"] != mc_diff_stream["manifest_sha256"]

    stream_a = sim_py.MonteCarloStream(77, "nav_cov", stream_index=0)
    stream_b = sim_py.MonteCarloStream(77, "nav_cov", stream_index=0)
    vec_a = stream_a.correlated_normal([1.0, -2.0], [4.0, 1.0, 1.0, 9.0])
    vec_b = stream_b.correlated_normal([1.0, -2.0], [4.0, 1.0, 1.0, 9.0])
    assert vec_a == vec_b
    assert stream_a.manifest_sha256 == stream_b.manifest_sha256


def main():
    check_time_scales()
    check_kepler_and_bplane()
    check_sgp4()
    check_ccsds_all_seven()
    check_spice_spk_and_ck()
    check_h3_coverage_and_monte_carlo()
    print("sim_py python_smoke.py: ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
