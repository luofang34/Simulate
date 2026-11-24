//! The rigid vehicle on a rotating body against closed-form motion: a
//! circular orbit in a point-mass field with a steady spin about a principal
//! axis, both of which are known exactly in inertial axes.

use sim_geo::GravityField;

use super::*;

/// The attraction of a point mass at the body's centre.
#[derive(Debug)]
struct PointMass {
    mu_m3_s2: f64,
}

impl GravityField for PointMass {
    fn attraction_fixed(&self, fixed: DVec3) -> DVec3 {
        -fixed * (self.mu_m3_s2 / fixed.length().powi(3))
    }
}

const RADIUS_M: f64 = 7_000_000.0;
const INCLINATION_RAD: f64 = 0.9;
const SPIN_RAD_S: f64 = 0.3;
const STEPS: u32 = 5_000;
const STEPS_PER_SECOND: u64 = 50;

/// Inertial-from-body attitude at the start.
fn initial_attitude() -> DQuat {
    DQuat::from_euler(glam::EulerRot::ZYX, 0.7, -0.4, 0.2)
}

/// The vehicle `elapsed_s` after the start as the body's fixed axes see it,
/// from the closed-form inertial motion.
fn closed_form(body: &BodyEllipsoid, elapsed_s: f64) -> VehicleState {
    let speed = (body.mu_m3_s2 / RADIUS_M).sqrt();
    let along = DVec3::new(0.0, INCLINATION_RAD.cos(), INCLINATION_RAD.sin());
    let (sin, cos) = (speed / RADIUS_M * elapsed_s).sin_cos();
    let position = (DVec3::X * cos + along * sin) * RADIUS_M;
    let velocity = (along * cos - DVec3::X * sin) * speed;
    let attitude = initial_attitude() * DQuat::from_rotation_x(SPIN_RAD_S * elapsed_s);

    let spin = DVec3::Z * body.omega_rad_s;
    let to_fixed = DQuat::from_rotation_z(-body.omega_rad_s * elapsed_s);
    let attitude_fixed = (to_fixed * attitude).normalize();
    VehicleState {
        position_ecef: to_fixed * position,
        velocity_ecef: to_fixed * (velocity - spin.cross(position)),
        attitude: attitude_fixed,
        angular_rate_body: DVec3::X * SPIN_RAD_S - attitude_fixed.conjugate() * spin,
    }
}

#[test]
fn a_spinning_rigid_vehicle_in_circular_orbit_follows_the_closed_form_on_a_rotating_body() {
    let body = BodyEllipsoid::WGS84;
    let tree = ComponentTree::new([
        rigid_part(
            "bus",
            Placement::ROOT,
            900.0,
            DVec3::new(500.0, 800.0, 800.0),
        ),
        // On the spin axis, so the pair keeps an axis of symmetry and the
        // centre of gravity is not the root origin.
        rigid_part(
            "mast",
            Placement::child_of(ComponentId(0), DVec3::new(3.0, 0.0, 0.0)),
            100.0,
            DVec3::new(20.0, 60.0, 60.0),
        ),
    ])
    .unwrap();
    let mut block = MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body,
            period: TickSpan::from_ticks(TICKS_PER_SECOND / STEPS_PER_SECOND),
            epoch: UtcTime::from_unix_seconds(0.0),
            initial: closed_form(&body, 0.0),
            integrator: VehicleIntegrator::REAL_TIME,
            time_scale: MotionTimeScale::SurfaceGravity,
        },
        tree,
        Vec::new(),
        MultibodySubsystems::default(),
        Box::new(VacuumAtmosphere),
        std::sync::Arc::new(PointMass {
            mu_m3_s2: body.mu_m3_s2,
        }),
    )
    .unwrap();
    assert!(block.telemetry().used_rigid_fast_path);

    for _ in 0..STEPS {
        block.step(&MultibodyVehicleInput::default()).unwrap();
    }
    assert!(block.telemetry().used_rigid_fast_path);
    let flown = block.output();
    let exact = closed_form(&body, f64::from(STEPS) / STEPS_PER_SECOND as f64);
    let position = (flown.position_ecef - exact.position_ecef).length();
    let velocity = (flown.velocity_ecef - exact.velocity_ecef).length();
    // Two body axes, compared as vectors: the angle between two attitudes
    // comes from an arc cosine that cannot tell 1e-9 rad from none.
    let attitude = [DVec3::X, DVec3::Y]
        .map(|axis| (flown.attitude * axis - exact.attitude * axis).length())
        .into_iter()
        .fold(0.0, f64::max);
    let rate = (flown.angular_rate_body - exact.angular_rate_body).length();
    assert!(position < 1e-5, "{position} m from the closed form");
    assert!(velocity < 1e-8, "{velocity} m/s from the closed form");
    assert!(attitude < 1e-10, "{attitude} rad from the closed form");
    assert!(rate < 1e-12, "{rate} rad/s from the closed form");
}
