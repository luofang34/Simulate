//! The integrator a vehicle is configured with changes how a period is
//! crossed, not where the vehicle ends up.

use mbd_multibody::Tolerances;

use super::*;

fn capsule_after_a_second(integrator: VehicleIntegrator) -> VehicleState {
    let mut block = capsule_block(integrator);
    for _ in 0..100 {
        block.step(&MultibodyVehicleInput::default()).unwrap();
    }
    block.output()
}

#[test]
fn error_controlled_schemes_fly_the_capsule_arc_of_the_fixed_step() {
    let fixed = capsule_after_a_second(VehicleIntegrator::REAL_TIME);
    let tolerances = Tolerances::uniform(1e-10, 1e-10);
    for integrator in [
        VehicleIntegrator::Adaptive(tolerances),
        VehicleIntegrator::Stiff(Tolerances::uniform(1e-7, 1e-7)),
        VehicleIntegrator::FixedRk4 { substeps: 4 },
    ] {
        let state = capsule_after_a_second(integrator);
        let apart = (state.position_ecef - fixed.position_ecef).length();
        let turned = state.attitude.angle_between(fixed.attitude);
        assert!(apart < 1e-5, "{integrator:?}: {apart} m");
        assert!(turned < 1e-6, "{integrator:?}: {turned} rad");
    }
}

#[test]
fn a_stiff_scheme_settles_the_lander_where_the_fixed_step_does() {
    let body = BodyEllipsoid::MOON_IAU;
    let pos = Geodetic::from_degrees(0.0, 0.0, 1.3);
    let rest_height = |integrator| {
        let mut block = footpad_lander_with(body, pos, footpad_set(), integrator);
        for _ in 0..300 {
            block.step(&MultibodyVehicleInput::default()).unwrap();
        }
        assert!(block.telemetry().in_ground_contact, "{integrator:?}");
        let state = block.output();
        (
            body.fixed_to_geodetic(state.position_ecef).height_m,
            state.velocity_ecef.length(),
        )
    };
    let (fixed, _) = rest_height(VehicleIntegrator::REAL_TIME);
    for integrator in [
        VehicleIntegrator::Stiff(Tolerances::uniform(1e-6, 1e-6)),
        VehicleIntegrator::Adaptive(Tolerances::uniform(1e-8, 1e-8)),
    ] {
        let (height, speed) = rest_height(integrator);
        assert!((height - fixed).abs() < 1e-4, "{integrator:?}: {height}");
        assert!(speed < 1e-4, "{integrator:?}: {speed} m/s");
    }
}

#[test]
fn a_body_without_gravity_needs_its_time_scale_stated() {
    let body = BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        ..BodyEllipsoid::WGS84
    };
    let build = |time_scale| {
        MultibodyVehicleBlock::new(
            MultibodyVehicleConfig {
                body,
                period: TickSpan::from_ticks(TICKS_PER_SECOND / 100),
                epoch: UtcTime::from_unix_seconds(0.0),
                initial: VehicleState {
                    position_ecef: DVec3::new(7_000_000.0, 0.0, 0.0),
                    velocity_ecef: DVec3::ZERO,
                    attitude: DQuat::IDENTITY,
                    angular_rate_body: DVec3::ZERO,
                },
                integrator: VehicleIntegrator::REAL_TIME,
                time_scale,
            },
            revolute_arm_tree(),
            Vec::new(),
            MultibodySubsystems::default(),
            Box::new(VacuumAtmosphere),
        )
    };
    assert!(matches!(
        build(MotionTimeScale::SurfaceGravity),
        Err(crate::BlockError::Multibody {
            source: mbd_multibody::MultibodyError::InvalidScale { .. },
            ..
        })
    ));
    assert!(build(MotionTimeScale::Seconds(1.0)).is_ok());
}
