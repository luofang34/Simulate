use glam::DVec3;
use sim_core::{Block, TICKS_PER_SECOND, TickSpan};

use super::*;
use crate::{InertialSiteSample, ThreeGimbalImuBlock, ThreeGimbalImuConfig};

const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 50);

fn imu_at(yaw_rad: f64) -> ImuOutput {
    let mut imu =
        ThreeGimbalImuBlock::new(ThreeGimbalImuConfig::nominal(PERIOD, DQuat::IDENTITY)).unwrap();
    imu.step(&InertialSiteSample {
        site_to_inertial: DQuat::from_rotation_z(yaw_rad),
        specific_force_site_m_s2: DVec3::ZERO,
        angular_rate_site_rad_s: DVec3::ZERO,
    })
    .unwrap();
    imu.output()
}

fn asa() -> AsaOutput {
    AsaOutput {
        site_to_inertial: DQuat::from_rotation_x(0.3),
        delta_velocity_inertial_m_s: DVec3::ZERO,
    }
}

#[test]
fn a_healthy_platform_stays_in_the_loop() {
    let mut handover = GuidanceHandover::default();
    let imu = imu_at(0.2);
    assert_eq!(
        handover.attitude(&imu, &asa()),
        imu.indicated_navbase_to_inertial
    );
    assert_eq!(handover.active(), GuidanceSource::Pgns);
}

#[test]
fn a_captured_platform_hands_over_and_stays_handed_over() {
    let mut handover = GuidanceHandover::default();
    assert_eq!(
        handover.attitude(&imu_at(1.53), &asa()),
        asa().site_to_inertial
    );
    assert_eq!(handover.active(), GuidanceSource::Ags);
    assert_eq!(
        handover.attitude(&imu_at(0.2), &asa()),
        asa().site_to_inertial
    );
    assert_eq!(handover.active(), GuidanceSource::Ags);
}

#[test]
fn the_crew_can_switch_either_way_but_not_back_onto_a_dead_platform() {
    let mut handover = GuidanceHandover::default();
    handover.crew_select(GuidanceSource::Ags);
    assert_eq!(
        handover.attitude(&imu_at(0.2), &asa()),
        asa().site_to_inertial
    );
    handover.crew_select(GuidanceSource::Pgns);
    let healthy = imu_at(0.2);
    assert_eq!(
        handover.attitude(&healthy, &asa()),
        healthy.indicated_navbase_to_inertial
    );
    handover.crew_select(GuidanceSource::Pgns);
    handover.attitude(&imu_at(1.53), &asa());
    assert_eq!(handover.active(), GuidanceSource::Ags);
}
