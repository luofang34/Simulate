//! `Raketa` as a vessel of a `Universe`: off the pad under closed-loop ascent guidance, into
//! orbital propagation by the tier policy once it coasts above the atmosphere, and back into
//! flight dynamics after a retrograde burn, with no state re-initialization anywhere.

use std::sync::Arc;

use glam::DVec3;
use sim_core::{SimInstant, TickSpan};
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime};
use sim_orbital::{
    AscentGuidanceConfig, BuiltinSolarSystem2026_2028, ContinuousPlantState, Dop853Tolerances,
    ForceModel, LatchedBurnLaw, NaifId, PlayerCommandPayload, VesselId,
    evaluate_gravity_turn_ascent,
};
use sim_raketa::{RAKETA_MASS_KG, RaketaCommand, RaketaEnvironment, RaketaPlant};
use sim_spacetime::SimEpoch;
use sim_universe::{OrbitalContext, Tier, TierPolicy, Universe};

const RAKETA: VesselId = VesselId(1);
/// `2027-03-01T00:00:00Z`.
const START_UTC_UNIX_S: f64 = 1_803_859_200.0;
const SECOND: TickSpan = TickSpan::from_seconds_and_subsec_ticks(1, 0);

#[cfg(test)]
fn seconds(s: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(s, 0))
}

#[cfg(test)]
fn universe_with_raketa_on_the_pad() -> Universe {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let force_model = ForceModel::standard_solar_system(solar.constants.clone()).expect("gravity");
    let environment = RaketaEnvironment {
        ephemeris: Arc::new(solar.ephemeris),
        orientation: Arc::new(solar.orientation),
        force_model: force_model.clone(),
    };
    let epoch =
        SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(START_UTC_UNIX_S)).expect("epoch");
    let context = OrbitalContext {
        ephemeris: Arc::clone(&environment.ephemeris),
        orientation: Arc::clone(&environment.orientation),
        force_model,
        quantum: TickSpan::from_seconds_and_subsec_ticks(10, 0),
        tolerances: Dop853Tolerances::default(),
    };
    let mut universe =
        Universe::new(epoch.clone(), context).with_tier_policy(TierPolicy::standard());
    let pad = Geodetic {
        lat_rad: 28.572_9_f64.to_radians(),
        lon_rad: (-80.649_0_f64).to_radians(),
        height_m: 0.0,
    };
    let plant = RaketaPlant::on_pad(
        environment.clone(),
        epoch,
        SimInstant::START,
        BodyEllipsoid::WGS84,
        &pad,
    )
    .expect("pad");
    universe
        .insert_local(RAKETA, Box::new(plant), RaketaPlant::factory(environment))
        .expect("insert");
    universe
}

#[cfg(test)]
/// Perigee and apogee heights above Earth's equatorial radius, m.
fn apsides_m(state: &ContinuousPlantState) -> (f64, f64) {
    let mu = BodyEllipsoid::WGS84.mu_m3_s2;
    let (r, v) = (state.pos_rel_icrf_m, state.vel_rel_icrf_m_s);
    let energy = 0.5 * v.length_squared() - mu / r.length();
    let semi_major = -mu / (2.0 * energy);
    let h = r.cross(v).length();
    let eccentricity = (1.0 - h * h / (mu * semi_major)).max(0.0).sqrt();
    let radius = BodyEllipsoid::WGS84.semi_major_m;
    (
        semi_major * (1.0 - eccentricity) - radius,
        semi_major * (1.0 + eccentricity) - radius,
    )
}

#[cfg(test)]
/// Flies the ascent one guidance update a second, cuts the engine for good once guidance
/// reports the parking orbit, and returns when the tier policy has taken the vessel.
fn fly_to_orbit(universe: &mut Universe) -> SimInstant {
    let guidance = AscentGuidanceConfig::earth_leo_185km();
    let mut engine_cut = false;
    let mut t = universe.now();
    while universe.tier(RAKETA).expect("tier") == Tier::Local {
        assert!(t < seconds(1_500), "no orbit after 1500 s of ascent");
        let plant = universe.local_mut::<RaketaPlant>(RAKETA).expect("Raketa");
        let state = *plant.inertial_state();
        let east = DVec3::Z.cross(state.pos_rel_icrf_m).normalize();
        let steer = evaluate_gravity_turn_ascent(
            state.pos_rel_icrf_m,
            state.vel_rel_icrf_m_s,
            east,
            &guidance,
        );
        engine_cut |= steer.orbit_achieved;
        plant.set_command(if engine_cut {
            RaketaCommand::default()
        } else {
            RaketaCommand {
                main_accel_m_s2: steer.commanded_accel_m_s2,
                nose_icrf: steer.thrust_dir_icrf,
                ..RaketaCommand::default()
            }
        });
        t = t.advanced(SECOND);
        universe.advance_to(t).expect("ascent step");
    }
    t
}

/// Coasts one revolution in the orbital tier, burns retrograde for a minute so perigee
/// drops into the atmosphere, and returns when the tier policy has handed the vessel back.
#[cfg(test)]
fn coast_a_revolution_then_deorbit(universe: &mut Universe, meco: SimInstant) -> SimInstant {
    let burn_start = meco.advanced(TickSpan::from_seconds_and_subsec_ticks(5_400, 0));
    universe.advance_to(burn_start).expect("coast");
    assert_eq!(universe.tier(RAKETA).expect("tier"), Tier::Orbital);
    let coasting = universe.inertial_state(RAKETA).expect("orbit");
    let retrograde = -coasting.vel_rel_icrf_m_s.normalize();
    let burn = PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::InertialThrust {
        dir_icrf: retrograde.to_array(),
        thrust_n: 2.0 * RAKETA_MASS_KG,
        exhaust_vel_m_s: 4_500.0,
        rw_torque_cmd_body_nm: [0.0; 3],
    });
    let burn_end = burn_start.advanced(TickSpan::from_seconds_and_subsec_ticks(60, 0));
    universe.command(RAKETA, burn_start, burn).expect("ignite");
    universe
        .command(
            RAKETA,
            burn_end,
            PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::Coast),
        )
        .expect("cut off");

    let mut t = burn_end;
    while universe.tier(RAKETA).expect("tier") == Tier::Orbital {
        assert!(
            t < burn_end.advanced(TickSpan::from_seconds_and_subsec_ticks(3_000, 0)),
            "the vessel never came back down"
        );
        t = t.advanced(TickSpan::from_seconds_and_subsec_ticks(10, 0));
        universe.advance_to(t).expect("descent");
    }
    t
}

#[test]
fn raketa_flies_from_the_pad_to_orbit_and_back_with_no_state_reset() {
    let mut universe = universe_with_raketa_on_the_pad();
    universe.advance_to(seconds(3)).expect("pad rest");
    let plant = universe.local::<RaketaPlant>(RAKETA).expect("Raketa");
    assert!(plant.is_on_pad(), "it sits on the pad until it has thrust");
    let rest_speed = universe
        .vehicle_state(RAKETA)
        .expect("state")
        .velocity_ecef
        .length();
    assert!(rest_speed < 1e-6, "pad rest moves at {rest_speed} m/s");

    let meco = fly_to_orbit(&mut universe);
    let changes = universe.take_tier_changes();
    assert_eq!(changes.len(), 1, "one handoff on the way up: {changes:?}");
    assert_eq!(changes[0].to, Tier::Orbital);
    assert_eq!(universe.central_body(RAKETA).expect("body"), NaifId::EARTH);
    let (perigee, apogee) = apsides_m(&universe.inertial_state(RAKETA).expect("orbit"));
    assert!(
        perigee > 170_000.0 && apogee < 200_000.0,
        "parking orbit is {perigee:.0} m by {apogee:.0} m"
    );

    let t = coast_a_revolution_then_deorbit(&mut universe, meco);
    let changes = universe.take_tier_changes();
    assert_eq!(changes.len(), 1, "one handoff on the way down: {changes:?}");
    assert_eq!(changes[0].to, Tier::Local);

    let before = universe.vehicle_state(RAKETA).expect("state");
    universe
        .advance_to(t.advanced(SECOND))
        .expect("entry flight");
    let after = universe.vehicle_state(RAKETA).expect("state");
    let plant = universe
        .local::<RaketaPlant>(RAKETA)
        .expect("Raketa is local again");
    assert!(!plant.is_on_pad());
    let height = BodyEllipsoid::WGS84
        .fixed_to_geodetic(after.position_ecef)
        .height_m;
    assert!(
        (100_000.0..140_000.0).contains(&height),
        "back in flight dynamics at {height:.0} m"
    );
    let moved = (after.position_ecef - before.position_ecef).length();
    assert!(
        (6_000.0..9_000.0).contains(&moved),
        "it flew {moved:.0} m in its first second back, not a jump"
    );
}
