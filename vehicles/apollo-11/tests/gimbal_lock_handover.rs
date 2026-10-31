//! A vehicle turned into gimbal lock loses its platform while the strapdown
//! abort sensors carry on, and guidance fails over to them.

use glam::{DQuat, DVec3};
use mbd_core::VacuumAero;
use sim_apollo::{
    GIMBAL_LOCK_CAPTURE_DEG, GIMBAL_LOCK_LAMP, GIMBAL_LOCK_WARNING_DEG, GuidanceHandover,
    GuidanceSource, InertialSiteSample, LM_ASA, LM_IMU, StrapdownAsaBlock, StrapdownAsaConfig,
    ThreeGimbalImuBlock, ThreeGimbalImuConfig, build_apollo_lm_tree,
};
use sim_atmosphere::VacuumAtmosphere;
use sim_core::{
    Block, CrewObservation, CrewStationPort, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput,
    SimInstant, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, UtcTime};

const EPOCH_UNIX_S: f64 = -14_182_940.0;
const HZ: u64 = 50;

#[cfg(test)]
fn frame_at(vehicle: &RigidBody6DofBlock) -> InertialTransform {
    let elapsed = vehicle.now().duration_since(SimInstant::START).as_seconds();
    InertialTransform::moon_at(UtcTime::from_unix_seconds(EPOCH_UNIX_S).advanced_by(elapsed))
}

/// The LM in free flight, turning about `rate_body`.
#[cfg(test)]
fn lm_turning(rate_body: DVec3) -> RigidBody6DofBlock {
    let body = BodyEllipsoid::MOON_IAU;
    let position = body.geodetic_to_fixed(&Geodetic::from_degrees(0.0, 23.0, 110_000.0));
    let initial = VehicleState {
        position_ecef: position,
        velocity_ecef: DVec3::new(0.0, 0.0, 1_600.0),
        attitude: DQuat::IDENTITY,
        angular_rate_body: rate_body,
    };
    RigidBody6DofBlock::new(
        RigidBodyConfig {
            body,
            period: TickSpan::period_of_hz(HZ).unwrap(),
            epoch: UtcTime::from_unix_seconds(EPOCH_UNIX_S),
            initial,
        },
        build_apollo_lm_tree().unwrap(),
        Box::new(VacuumAero),
        Box::new(VacuumAtmosphere),
    )
    .unwrap()
}

#[cfg(test)]
struct Sensors {
    imu: ThreeGimbalImuBlock,
    asa: StrapdownAsaBlock,
}

#[cfg(test)]
impl Sensors {
    fn aligned_to(vehicle: &RigidBody6DofBlock) -> Self {
        let period = vehicle.period();
        let frame = frame_at(vehicle);
        let at_imu = InertialSiteSample::of(vehicle, LM_IMU, &frame).unwrap();
        let at_asa = InertialSiteSample::of(vehicle, LM_ASA, &frame).unwrap();
        Self {
            imu: ThreeGimbalImuBlock::new(ThreeGimbalImuConfig::nominal(
                period,
                at_imu.site_to_inertial,
            ))
            .unwrap(),
            asa: StrapdownAsaBlock::new(StrapdownAsaConfig::nominal(
                period,
                at_asa.site_to_inertial,
            ))
            .unwrap(),
        }
    }

    /// Steps everything once and returns the true NavBase attitude.
    fn tick(&mut self, vehicle: &mut RigidBody6DofBlock) -> DQuat {
        vehicle.step(&RigidBodyInput::default()).unwrap();
        let frame = frame_at(vehicle);
        let at_imu = InertialSiteSample::of(vehicle, LM_IMU, &frame).unwrap();
        let at_asa = InertialSiteSample::of(vehicle, LM_ASA, &frame).unwrap();
        self.imu.step(&at_imu).unwrap();
        self.asa.step(&at_asa).unwrap();
        at_asa.site_to_inertial
    }
}

#[test]
fn turning_past_the_middle_gimbal_limits_warns_then_captures_while_the_asa_holds() {
    let mut vehicle = lm_turning(DVec3::new(0.0, 0.0, 4f64.to_radians()));
    let mut sensors = Sensors::aligned_to(&vehicle);
    let mut guidance = GuidanceHandover::default();

    let mut truth = DQuat::IDENTITY;
    while sensors.imu.output().mga_rad.abs() < GIMBAL_LOCK_WARNING_DEG.to_radians() {
        assert!(!sensors.imu.output().gimbal_lock_warning);
        truth = sensors.tick(&mut vehicle);
    }
    let warned = sensors.imu.output();
    assert!(warned.gimbal_lock_warning);
    assert!(!warned.gimbal_locked);
    assert!(warned.platform_valid);
    assert!(warned.indicated_navbase_to_inertial.angle_between(truth) < 1e-5);
    let held = guidance.attitude(&warned, &sensors.asa.output());
    assert_eq!(guidance.active(), GuidanceSource::Pgns);
    assert_eq!(held, warned.indicated_navbase_to_inertial);

    let mut port = CrewStationPort::new();
    sensors.imu.publish_lamp(vehicle.now(), &mut port).unwrap();
    assert!(port.latest_observations().iter().any(|seen| matches!(
        seen.observation,
        CrewObservation::AnnunciatorLamp {
            lamp_id: GIMBAL_LOCK_LAMP,
            lit: true
        }
    )));

    while sensors.imu.output().mga_rad.abs() < GIMBAL_LOCK_CAPTURE_DEG.to_radians() {
        assert!(!sensors.imu.is_gimbal_locked());
        sensors.tick(&mut vehicle);
    }
    assert!(sensors.imu.is_gimbal_locked());
    assert!(!sensors.imu.output().platform_valid);

    for _ in 0..10 * HZ {
        truth = sensors.tick(&mut vehicle);
        let asa_error = sensors.asa.output().site_to_inertial.angle_between(truth);
        assert!(asa_error < 2e-4, "{asa_error}");
        assert!(sensors.imu.is_gimbal_locked(), "capture latches");
    }
    let tumbled = sensors.imu.output();
    let imu_error = tumbled.indicated_navbase_to_inertial.angle_between(truth);
    assert!(imu_error > 0.5, "reference lost: {imu_error}");

    let flown = guidance.attitude(&tumbled, &sensors.asa.output());
    assert_eq!(guidance.active(), GuidanceSource::Ags);
    assert!(flown.angle_between(truth) < 2e-4);

    sensors.imu.coarse_align(truth);
    let realigned = sensors.imu.output();
    assert!(!realigned.gimbal_locked && realigned.platform_valid);
    assert!(realigned.indicated_navbase_to_inertial.angle_between(truth) < 1e-9);
    guidance.attitude(&realigned, &sensors.asa.output());
    assert_eq!(
        guidance.active(),
        GuidanceSource::Ags,
        "only the crew returns it"
    );
    guidance.crew_select(GuidanceSource::Pgns);
    assert_eq!(guidance.active(), GuidanceSource::Pgns);
}

#[test]
fn a_roll_about_the_outer_axis_never_nears_lock() {
    let mut vehicle = lm_turning(DVec3::new(20f64.to_radians(), 0.0, 0.0));
    let mut sensors = Sensors::aligned_to(&vehicle);
    let mut truth = DQuat::IDENTITY;
    for _ in 0..30 * HZ {
        truth = sensors.tick(&mut vehicle);
        let imu = sensors.imu.output();
        assert!(!imu.gimbal_lock_warning, "{}", imu.mga_rad);
    }
    let imu = sensors.imu.output();
    assert!(imu.indicated_navbase_to_inertial.angle_between(truth) < 1e-5);
}
