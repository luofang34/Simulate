use mbd_core::{BluntBodyCapsuleAero, BluntBodyCapsuleParams, SUTTON_GRAVES_K_EARTH};
use sim_atmosphere::{AtmosphereError, AtmosphereSample, ReentryAtmosphere, StandardAtmosphere};

use super::*;

/// An air mass of fixed density, so a trim test is not racing the descent.
struct UniformAir {
    density_kg_m3: f64,
    wind_ned_m_s: DVec3,
}

impl AtmosphereField for UniformAir {
    fn sample(&self, _: &Geodetic, _: UtcTime) -> Result<AtmosphereSample, AtmosphereError> {
        Ok(AtmosphereSample {
            temperature_k: 230.0,
            pressure_pa: 30.0,
            density_kg_m3: self.density_kg_m3,
            speed_of_sound_m_s: 300.0,
            wind_ned_m_s: self.wind_ned_m_s,
        })
    }
}

const NOSE_RADIUS_M: f64 = 4.694;
const NORMAL_FORCE_SLOPE: f64 = 0.113;
const AXIAL_DRAG_COEFF: f64 = 1.3;

pub(super) fn capsule_aero() -> Box<BluntBodyCapsuleAero> {
    let params = BluntBodyCapsuleParams {
        base_radius_m: 1.956,
        nose_radius_m: NOSE_RADIUS_M,
        reference_point_body_m: DVec3::new(-NOSE_RADIUS_M, 0.0, 0.0),
        normal_force_slope: NORMAL_FORCE_SLOPE,
        axial_drag_coeff: AXIAL_DRAG_COEFF,
        pitch_damping_coeff: -10.0,
        sutton_graves_k: SUTTON_GRAVES_K_EARTH,
    };
    Box::new(BluntBodyCapsuleAero::new(params).unwrap())
}

pub(super) fn capsule_tree(cg: DVec3) -> ComponentTree {
    ComponentTree::new([(
        spec(
            "capsule",
            Placement {
                translation_m: cg,
                ..Placement::ROOT
            },
            5_500.0,
            DVec3::new(5_000.0, 4_500.0, 4_000.0),
        ),
        component(290.0),
    )])
    .unwrap()
}

/// The body origin is the heat-shield apex, `+x` the leading axis. Far from
/// any body and with no spin, only the air acts.
fn capsule_block(cg: DVec3, density_kg_m3: f64) -> RigidBody6DofBlock {
    let body = BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        ..BodyEllipsoid::WGS84
    };
    let start = VehicleState {
        velocity_ecef: DVec3::new(7_500.0, 0.0, 0.0),
        ..rest_at(&body, 80_000.0)
    };
    RigidBody6DofBlock::new(
        config(body, TEN_MS, start),
        capsule_tree(cg),
        capsule_aero(),
        Box::new(UniformAir {
            density_kg_m3,
            wind_ned_m_s: DVec3::ZERO,
        }),
    )
    .unwrap()
}

fn incidence_rad(block: &RigidBody6DofBlock) -> f64 {
    let state = block.output();
    let air = state.attitude.inverse() * state.velocity_ecef;
    air.z.atan2(air.x)
}

#[test]
fn an_offset_cg_capsule_settles_at_the_analytic_trim_incidence() {
    let offset_cg = DVec3::new(-1.15, 0.0, -0.112);
    let mut block = capsule_block(offset_cg, 2e-4);
    assert_eq!(incidence_rad(&block), 0.0);
    assert!(block.alpha_body().y.abs() > 0.01, "released out of trim");
    run(&mut block, 12_000, &RigidBodyInput::default());

    // The force passes through the moment reference, so trim puts its line of
    // action through the centre of gravity.
    let arm = DVec3::new(-NOSE_RADIUS_M, 0.0, 0.0) - offset_cg;
    let trim = (AXIAL_DRAG_COEFF / NORMAL_FORCE_SLOPE * arm.z / arm.x).atan();
    let incidence = incidence_rad(&block);
    assert!(trim.abs() > 0.2, "{trim}");
    assert!((incidence - trim).abs() < 2e-3, "{incidence} vs {trim}");
    assert!(block.omega_inertial_body().length() < 1e-2);
}

#[test]
fn an_on_axis_cg_capsule_flies_at_zero_incidence() {
    let mut block = capsule_block(DVec3::new(-1.15, 0.0, 0.0), 2e-4);
    run(&mut block, 2_000, &RigidBodyInput::default());
    assert!(incidence_rad(&block).abs() < 1e-12);
    assert!(block.aero_wrench().force_body_n.x < 0.0);
}

#[test]
fn wind_reaches_the_aerodynamics_model_in_body_axes() {
    let body = no_spin(BodyEllipsoid::WGS84);
    let start = rest_at(&body, 10_000.0);
    let block = RigidBody6DofBlock::new(
        config(body, TEN_MS, start),
        capsule_tree(DVec3::ZERO),
        capsule_aero(),
        Box::new(UniformAir {
            density_kg_m3: 0.4,
            wind_ned_m_s: DVec3::new(0.0, 0.0, -50.0),
        }),
    )
    .unwrap();
    // At 0°N 0°E, ECEF +x is up and body axes match ECEF here, so rising air
    // pushes the vehicle along body +x and nowhere else.
    let force = block.aero_wrench().force_body_n;
    assert!(force.x > 0.0, "{force}");
    assert!(force.y.hypot(force.z) < 1e-9 * force.x, "{force}");
}

#[test]
fn the_block_accepts_every_atmosphere_on_its_own_body() {
    let earth = BodyEllipsoid::WGS84;
    let moon = BodyEllipsoid::MOON_IAU;
    let atmospheres: [(BodyEllipsoid, Box<dyn AtmosphereField>); 3] = [
        (earth, Box::new(StandardAtmosphere)),
        (earth, Box::new(ReentryAtmosphere)),
        (moon, Box::new(VacuumAtmosphere)),
    ];
    for (body, atmosphere) in atmospheres {
        let start = rest_at(&body, 5_000.0);
        let mut block = RigidBody6DofBlock::new(
            config(body, TEN_MS, start),
            capsule_tree(DVec3::ZERO),
            capsule_aero(),
            atmosphere,
        )
        .unwrap();
        run(&mut block, 10, &RigidBodyInput::default());
        assert!(block.output().position_ecef.is_finite());
    }
}
