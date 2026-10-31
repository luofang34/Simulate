//! The LM flies the last 12 km to the surface on the descent engine, reading
//! its radar and shifting its mass, until a probe calls for cutoff.

use glam::{DQuat, DVec3};
use mbd_core::VacuumAero;
use sim_apollo::{
    LM_DPS_EXHAUST_VELOCITY_M_S, LM_DPS_TANK, LM_DPS_THRUST_N, LM_DPS_THRUST_POINT_M,
    LM_LANDING_RADAR, LM_LEG_MINUS_Z, LM_PROBE_TIPS, LandingRadarAntenna, LandingRadarBlock,
    LandingRadarConfig, LandingRadarInput, LunarContactInput, LunarContactProbeBlock,
    PROBE_LENGTH_M, RadarValidity, SurfacePlane, build_apollo_lm_tree, site_in_fixed_frame,
};
use sim_atmosphere::VacuumAtmosphere;
use sim_core::{
    Block, PropellantBurn, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput, ThrusterWrench,
    TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime};

const HZ: u64 = 50;
const START_HEIGHT_M: f64 = 13_000.0;
const TERRAIN_HEIGHT_M: f64 = -1_900.0;
const START_DESCENT_M_S: f64 = 120.0;
const FINAL_DESCENT_M_S: f64 = 1.0;
const CROSSRANGE_DRIFT_M_S: f64 = 8.0;
const MIN_THROTTLE: f64 = 0.1;
/// Probe tip distance below the centre of gravity, roughly: where the
/// descent profile has to be down to its touchdown rate.
const CG_HEIGHT_AT_CONTACT_M: f64 = 6.0;
/// Combined lateral thrust of the RCS jets, N.
const RCS_TRANSLATION_N: f64 = 1_800.0;

#[cfg(test)]
fn lander(body: BodyEllipsoid) -> RigidBody6DofBlock {
    let geodetic = Geodetic::from_degrees(0.67, 23.47, TERRAIN_HEIGHT_M + START_HEIGHT_M);
    let position = body.geodetic_to_fixed(&geodetic);
    let up = position.normalize();
    let across = up.cross(DVec3::Z).normalize();
    let initial = VehicleState {
        position_ecef: position,
        velocity_ecef: up * -START_DESCENT_M_S + across * CROSSRANGE_DRIFT_M_S,
        attitude: DQuat::from_rotation_arc(DVec3::X, up),
        angular_rate_body: DVec3::ZERO,
    };
    RigidBody6DofBlock::new(
        RigidBodyConfig {
            body,
            period: TickSpan::period_of_hz(HZ).unwrap(),
            epoch: UtcTime::from_unix_seconds(-14_182_940.0),
            initial,
        },
        build_apollo_lm_tree().unwrap(),
        Box::new(VacuumAero),
        Box::new(VacuumAtmosphere),
    )
    .unwrap()
}

/// Descent rate to hold at `height_m`: a constant-deceleration profile well
/// inside the engine's authority, tapering to the touchdown rate.
#[cfg(test)]
fn commanded_descent_m_s(height_m: f64) -> f64 {
    let to_contact_m = (height_m - CG_HEIGHT_AT_CONTACT_M).max(0.0);
    (0.9 * to_contact_m.sqrt()).clamp(FINAL_DESCENT_M_S, START_DESCENT_M_S)
}

/// DPS thrust along `+x`, trimmed through the live centre of gravity by the
/// engine gimbal, holding the descent profile, with RCS translation nulling
/// the drift across the surface.
#[cfg(test)]
fn descent_command(vehicle: &RigidBody6DofBlock, body: &BodyEllipsoid) -> RigidBodyInput {
    let state = vehicle.output();
    let up = state.position_ecef.normalize();
    let height_m = body.fixed_to_geodetic(state.position_ecef).height_m - TERRAIN_HEIGHT_M;
    let mass = vehicle.mass_properties();
    let gravity = body.mu_m3_s2 / state.position_ecef.length_squared();
    let descent = -state.velocity_ecef.dot(up);
    let wanted = gravity + (descent - commanded_descent_m_s(height_m)) * 0.8;
    let thrust_n =
        (wanted * mass.total_mass_kg).clamp(MIN_THROTTLE * LM_DPS_THRUST_N, LM_DPS_THRUST_N);
    let from_gimbal = (mass.cg_body_m - LM_DPS_THRUST_POINT_M).normalize();
    let drift = state.velocity_ecef - up * state.velocity_ecef.dot(up);
    let translation = (state.attitude.inverse() * drift * -0.3 * mass.total_mass_kg)
        .clamp_length_max(RCS_TRANSLATION_N);
    RigidBodyInput {
        thrusters: vec![
            ThrusterWrench {
                application_point_body_m: LM_DPS_THRUST_POINT_M,
                force_body_n: from_gimbal * thrust_n,
                moment_body_nm: DVec3::ZERO,
            },
            ThrusterWrench {
                application_point_body_m: mass.cg_body_m,
                force_body_n: translation,
                moment_body_nm: DVec3::ZERO,
            },
        ],
        burns: vec![PropellantBurn {
            tank: LM_DPS_TANK,
            dm_dt_kg_s: -thrust_n / LM_DPS_EXHAUST_VELOCITY_M_S,
        }],
        ..Default::default()
    }
}

/// What the radar showed on the way down.
#[cfg(test)]
#[derive(Default)]
struct RadarLog {
    out_of_limits_samples: u32,
    valid_samples: u32,
}

#[cfg(test)]
impl RadarLog {
    /// Reads the radar at the vehicle's current state and checks it against
    /// the true geometry.
    fn read(
        &mut self,
        radar: &mut LandingRadarBlock,
        vehicle: &RigidBody6DofBlock,
    ) -> SurfacePlane {
        let body = BodyEllipsoid::MOON_IAU;
        let site = site_in_fixed_frame(vehicle, LM_LANDING_RADAR).unwrap();
        let surface = SurfacePlane::level_beneath(&body, site.position_m, TERRAIN_HEIGHT_M);
        radar
            .step(&LandingRadarInput {
                site,
                surface,
                antenna: LandingRadarAntenna::Hover,
            })
            .unwrap();
        let reading = radar.output();
        if reading.validity == RadarValidity::Valid {
            self.valid_samples += 1;
            let vertical = surface.clearance_m(site.position_m);
            assert!(reading.slant_range_m > vertical);
            assert!(reading.slant_range_m < 1.02 * vertical);
            let truth = site.attitude.inverse() * site.velocity_m_s;
            let solved = reading.velocity_site_m_s;
            assert!((solved - truth).length() < 1e-6, "{solved} vs {truth}");
        } else {
            self.out_of_limits_samples += 1;
            assert_eq!(self.valid_samples, 0, "lock holds once acquired");
            assert!(
                reading.slant_range_m > 11_990.0,
                "{}",
                reading.slant_range_m
            );
        }
        surface
    }
}

/// Flies the descent profile until a probe touches, checking every sample.
#[cfg(test)]
fn fly_to_contact(
    vehicle: &mut RigidBody6DofBlock,
    probes: &mut LunarContactProbeBlock,
) -> RadarLog {
    let body = BodyEllipsoid::MOON_IAU;
    let mut radar = LandingRadarBlock::new(LandingRadarConfig::nominal(vehicle.period())).unwrap();
    let mut log = RadarLog::default();
    let mut last_cg_x = vehicle.mass_properties().cg_body_m.x;
    for _ in 0..600 * HZ {
        let command = descent_command(vehicle, &body);
        vehicle.step(&command).unwrap();

        let cg_x = vehicle.mass_properties().cg_body_m.x;
        assert!(cg_x > last_cg_x, "burning low propellant raises the cg");
        last_cg_x = cg_x;

        let surface = log.read(&mut radar, vehicle);
        let tips = LM_PROBE_TIPS.map(|tip| site_in_fixed_frame(vehicle, tip).unwrap().position_m);
        probes
            .step(&LunarContactInput {
                tips_fixed_m: tips,
                surface,
            })
            .unwrap();
        if probes.output().lunar_contact {
            break;
        }
    }
    log
}

#[test]
fn powered_descent_shifts_the_cg_reads_the_radar_and_latches_contact() {
    let body = BodyEllipsoid::MOON_IAU;
    let mut vehicle = lander(body);
    let start_mass = *vehicle.mass_properties();
    let start_fuel = vehicle.tree().state(LM_DPS_TANK).unwrap().fluid_mass_kg;
    let mut probes = LunarContactProbeBlock::new(vehicle.period()).unwrap();

    let radar_log = fly_to_contact(&mut vehicle, &mut probes);

    assert!(probes.output().lunar_contact, "never reached the surface");
    assert!(
        radar_log.out_of_limits_samples > 0,
        "started above the limit"
    );
    assert!(
        radar_log.valid_samples > 1_000,
        "{}",
        radar_log.valid_samples
    );

    let state = vehicle.output();
    let up = state.position_ecef.normalize();
    let descent = -state.velocity_ecef.dot(up);
    assert!((0.5..1.5).contains(&descent), "{descent}");
    let drift = (state.velocity_ecef + up * descent).length();
    assert!(drift < 0.5, "{drift}");
    let footpad = site_in_fixed_frame(&vehicle, LM_LEG_MINUS_Z)
        .unwrap()
        .position_m;
    let footpad_height = body.fixed_to_geodetic(footpad).height_m - TERRAIN_HEIGHT_M;
    assert!(
        (footpad_height - PROBE_LENGTH_M).abs() < 0.1,
        "the probe touches first: {footpad_height}"
    );

    let end_mass = vehicle.mass_properties();
    let end_fuel = vehicle.tree().state(LM_DPS_TANK).unwrap().fluid_mass_kg;
    assert!(end_fuel > 0.0);
    let burned = start_fuel - end_fuel;
    assert!(burned > 1_000.0, "{burned}");
    assert!((start_mass.total_mass_kg - end_mass.total_mass_kg - burned).abs() < 1e-6);
    let cg_rise = end_mass.cg_body_m.x - start_mass.cg_body_m.x;
    assert!(cg_rise > 0.1, "{cg_rise}");
    let inertia_drop = start_mass.inertia_cg_body.y_axis.y - end_mass.inertia_cg_body.y_axis.y;
    assert!(inertia_drop > 0.0, "{inertia_drop}");

    vehicle.step(&RigidBodyInput::default()).unwrap();
    assert_eq!(vehicle.specific_force_cg_body(), DVec3::ZERO);
    probes
        .step(&LunarContactInput {
            tips_fixed_m: [DVec3::splat(1e7); 3],
            surface: SurfacePlane::level_beneath(&body, state.position_ecef, TERRAIN_HEIGHT_M),
        })
        .unwrap();
    assert!(probes.output().lunar_contact, "the light stays latched");
}
