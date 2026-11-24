use mbd_core::{ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use sim_atmosphere::VacuumAtmosphere;
use sim_bodies::{BodyDescriptor, HarmonicGravity};
use sim_core::{RigidBody6DofBlock, RigidBodyConfig, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, UtcTime};

use super::*;
use crate::{
    GnssErrorSpec, ImuErrorSpec, MagneticEnvironment, MagnetometerSpec, PressureErrorSpec,
    SensorSuite, SensorSuiteSpec,
};

const MGAL: f64 = 1.0e-5;
const EOTVOS: f64 = 1.0e-9;

fn site(lat_deg: f64, lon_deg: f64) -> DVec3 {
    BodyEllipsoid::WGS84.geodetic_to_fixed(&Geodetic::from_degrees(lat_deg, lon_deg, 0.0))
}

/// On the reference field a gravimeter reads normal gravity: about 9.780 m/s²
/// on the equator and 9.832 m/s² at the poles.
#[test]
fn the_reference_field_reads_normal_gravity() {
    let reference = HarmonicGravity::reference(&BodyEllipsoid::WGS84).unwrap();
    let read = |lat| gravimeter_truth(&reference, &BodyEllipsoid::WGS84, site(lat, 10.0));
    assert!((read(0.0).magnitude_m_s2() - 9.780).abs() < 2e-3);
    assert!((read(90.0).magnitude_m_s2() - 9.832).abs() < 2e-3);
    // The plumb line is the ellipsoid normal to within the J2 model's error.
    let down = -LocalFrame::at(&Geodetic::from_degrees(45.0, 10.0, 0.0)).up();
    let tilt = read(45.0).gravity_fixed_m_s2.normalize().dot(down).acos();
    assert!(tilt < 1e-4, "{tilt}");
    // The vertical gradient of gravity is the free-air gradient, 3086 E.
    let gradient = read(45.0).gradient_per_s2.unwrap();
    let vertical = down.dot(gradient * down) / EOTVOS;
    assert!((vertical - 3_086.0).abs() < 10.0, "{vertical}");
    // The closed-form field states no gradient, and the sample says so.
    let closed = gravimeter_truth(
        &BodyEllipsoid::WGS84,
        &BodyEllipsoid::WGS84,
        site(45.0, 10.0),
    );
    assert!(closed.gradient_per_s2.is_none());
    assert!((closed.magnitude_m_s2() - read(45.0).magnitude_m_s2()).abs() < 1e-12);
}

/// The hook gravity-map navigation reads: a detailed truth field against the
/// navigator's `J2` reference differs by the gravity anomaly, tens of
/// milligal in magnitude and a few eötvös in gradient at degree 36.
#[test]
fn a_detailed_world_differs_from_the_navigators_reference_by_the_anomaly() {
    let reference = HarmonicGravity::reference(&BodyEllipsoid::WGS84).unwrap();
    let detailed = HarmonicGravity::egm96_degree_36().unwrap();
    let (mut largest_mgal, mut largest_eotvos) = (0.0_f64, 0.0_f64);
    let mut signs = [false; 2];
    for (lat, lon) in [
        (64.0, -20.0),
        (5.0, 78.0),
        (-25.0, -70.0),
        (39.0, -75.5),
        (-2.0, 128.0),
    ] {
        let at = site(lat, lon);
        let truth = gravimeter_truth(&detailed, &BodyEllipsoid::WGS84, at);
        let model = gravimeter_truth(&reference, &BodyEllipsoid::WGS84, at);
        let anomaly = (truth.magnitude_m_s2() - model.magnitude_m_s2()) / MGAL;
        signs[usize::from(anomaly > 0.0)] = true;
        largest_mgal = largest_mgal.max(anomaly.abs());
        assert!(anomaly.abs() < 150.0, "{lat} {lon}: {anomaly} mGal");
        let difference = truth.gradient_per_s2.unwrap() - model.gradient_per_s2.unwrap();
        let eotvos = difference
            .to_cols_array()
            .iter()
            .fold(0.0_f64, |m, v| m.max(v.abs()));
        largest_eotvos = largest_eotvos.max(eotvos / EOTVOS);
    }
    assert!(largest_mgal > 20.0, "{largest_mgal}");
    assert!(signs[0] && signs[1], "anomalies of one sign only");
    assert!((0.05..30.0).contains(&largest_eotvos), "{largest_eotvos}");
}

fn point_mass() -> ComponentTree {
    ComponentTree::new([(
        ComponentSpec {
            name: "probe".to_owned(),
            placement: Placement::ROOT,
            dry_mass_kg: 10.0,
            dry_inertia_body: glam::DMat3::IDENTITY,
            thermal_capacity_j_k: 1.0,
            nominal_power_w: 0.0,
            joint: JointSpec::FIXED,
        },
        ComponentState::at_temperature(290.0),
    )])
    .unwrap()
}

/// One world, one field of each kind: the dynamics, the gravimeter and the
/// magnetometer hold the very instances the body's catalog entry does.
#[test]
fn dynamics_and_sensors_share_the_bodys_field_instances() {
    let earth = BodyDescriptor::earth()
        .unwrap()
        .with_gravity(HarmonicGravity::egm96_degree_36().unwrap());
    let fields = earth.fields();
    let at = Geodetic::from_degrees(39.0, -75.5, 3_000.0);
    let state = VehicleState::level_flight(&at, 0.3, 60.0);
    let block = RigidBody6DofBlock::new(
        RigidBodyConfig {
            body: earth.ellipsoid,
            period: TickSpan::from_ticks(TICKS_PER_SECOND / 100),
            epoch: UtcTime::from_unix_seconds(1_800_000_000.0),
            initial: state,
        },
        point_mass(),
        Vec::new(),
        Box::new(VacuumAtmosphere),
        fields.gravity.clone(),
    )
    .unwrap();
    let spec = SensorSuiteSpec {
        imu: ImuErrorSpec::IDEAL,
        static_port: PressureErrorSpec::IDEAL,
        pitot: None,
        magnetometer: MagnetometerSpec::IDEAL,
        gnss: GnssErrorSpec::OPEN_SKY,
        magnetic: MagneticEnvironment::main_field(2026.5),
        air_data_every: 1,
        magnetometer_every: 1,
        gnss_every: 1,
        seed: 1,
    };
    let suite = SensorSuite::new(&spec, fields.magnetic.clone()).unwrap();
    assert!(fields.are_instances(block.gravity(), suite.magnetic_field()));
    assert!(
        earth
            .fields()
            .are_instances(block.gravity(), suite.magnetic_field())
    );
    let elsewhere = BodyDescriptor::earth().unwrap().fields();
    assert!(
        !elsewhere.are_instances(block.gravity(), suite.magnetic_field()),
        "a second descriptor's equal fields are other instances"
    );

    // The gravimeter reads what the dynamics integrate, not the reference.
    let fixed = state.position_ecef;
    let sensed = gravimeter_truth(block.gravity().as_ref(), &earth.ellipsoid, fixed);
    assert_eq!(
        sensed.attraction_fixed_m_s2,
        earth.gravity().attraction_fixed(fixed)
    );
    let reference = gravimeter_truth(&earth.ellipsoid, &earth.ellipsoid, fixed);
    let anomaly_mgal = (sensed.magnitude_m_s2() - reference.magnitude_m_s2()) / MGAL;
    assert!(anomaly_mgal.abs() > 1.0, "{anomaly_mgal}");
}
