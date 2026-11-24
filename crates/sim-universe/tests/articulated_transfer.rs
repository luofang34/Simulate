//! One articulated spacecraft, one plant, from a low Earth orbit to Mars:
//! local tier, Earth orbital, heliocentric, Mars orbital. Its gimbal angle,
//! its tank and every other bit of its internals come through each change
//! of tier and of origin.

use sim_core::TickSpan;
use sim_orbital::NaifId;
use sim_universe::{LocalPlant, Tier};

#[path = "articulated_transfer/carried_slew.rs"]
mod carried_slew;
#[path = "articulated_transfer/mars_arrival.rs"]
mod mars_arrival;
#[path = "articulated_transfer/spacecraft.rs"]
mod spacecraft;
#[path = "articulated_transfer/voyage.rs"]
mod voyage;

use spacecraft::{BLOCK_PERIOD, CARRY_PERIOD, XENON_KG, gimbal_effort_nm, input};
use voyage::{DAY_S, Internals, PROBE, Voyage, at_s};

/// Where the antenna is told to point in the local tier, rad.
const GIMBAL_COMMAND_RAD: f64 = 0.6;
/// Where it is told to point next, carried about the Earth, rad.
const ORBITAL_COMMAND_RAD: f64 = 0.25;
/// And then, carried about the Sun, rad.
const CRUISE_COMMAND_RAD: f64 = -0.3;
/// The first slew, flown in the local tier.
const LOCAL_S: u64 = 5_400;
/// The electric thruster runs this long, the second half of it carried.
const THRUST_UNTIL_S: u64 = 9_000;
/// How long the gimbal is given to reach a new command about the Sun, s.
const CRUISE_SLEW_S: u64 = 3_600;
/// How near its command the gimbal is held through a cruise, rad.
const HELD_RAD: f64 = 1e-4;
/// Prograde burn out of the parking orbit, m/s: a hyperbolic excess near
/// 3.6 km/s.
const ESCAPE_DELTA_V_M_S: f64 = 3_750.0;
const MARS_SOI_M: f64 = 577_200_000.0;
/// How far from plan the arrival may be, m. The last trim is nine days out
/// and aims a Sun-centred conic at Mars's centre; Mars's own pull over
/// those days, which that conic leaves out, bends the path by a few
/// thousand kilometres.
const ARRIVAL_MISS_M: f64 = 2.0e7;

/// Slews the gimbal to its command with the plant flying itself, a
/// position loop closed once per plant step, and returns the largest
/// tracking error over the last ten minutes.
#[cfg(test)]
fn slew_in_the_local_tier(voyage: &mut Voyage) -> f64 {
    let mut settled_error = 0.0_f64;
    let step_s = BLOCK_PERIOD.as_seconds() as u64;
    for step in 1..=LOCAL_S / step_s {
        let now = voyage.internals();
        let effort = gimbal_effort_nm(
            GIMBAL_COMMAND_RAD,
            now.gimbal_rad,
            now.gimbal_rate_rad_s,
            step_s as f64,
        );
        let commanded = input(voyage.ship().input(), 1.0, effort);
        voyage.ship_mut().set_input(commanded);
        voyage.universe.advance_to(at_s(step * step_s)).unwrap();
        if step * step_s > LOCAL_S - 600 {
            let error = (voyage.internals().gimbal_rad - GIMBAL_COMMAND_RAD).abs();
            settled_error = settled_error.max(error);
        }
    }
    settled_error
}

/// Flies the carried plant one exchange of `sample` on, with the gimbal
/// loop closed through that exchange: the motor holds, until the next one,
/// the torque the loop asks for from the angle and rate the plant has now.
#[cfg(test)]
fn carry_one_exchange(voyage: &mut Voyage, command_rad: f64, throttle: f64, sample: TickSpan) {
    let now = voyage.internals();
    let effort = gimbal_effort_nm(
        command_rad,
        now.gimbal_rad,
        now.gimbal_rate_rad_s,
        sample.as_seconds(),
    );
    let commanded = input(voyage.ship().input(), throttle, effort);
    let ship = voyage.ship_mut();
    ship.set_carry_period(sample);
    ship.set_input(commanded);
    let next = voyage.ship().now().advanced(sample);
    voyage.universe.advance_to(next).unwrap();
    assert_eq!(voyage.universe.tier(PROBE).unwrap(), Tier::Orbital);
}

/// Slews the gimbal to `command_rad` while the orbital tier carries the
/// plant, exchanging once per plant step until `until_s`, and returns the
/// largest tracking error over the last ten minutes.
#[cfg(test)]
fn slew_while_carried(voyage: &mut Voyage, command_rad: f64, throttle: f64, until_s: u64) -> f64 {
    let mut settled_error = 0.0_f64;
    while voyage.universe.now() < at_s(until_s) {
        carry_one_exchange(voyage, command_rad, throttle, BLOCK_PERIOD);
        if voyage.universe.now() > at_s(until_s - 600) {
            let error = (voyage.internals().gimbal_rad - command_rad).abs();
            settled_error = settled_error.max(error);
        }
    }
    settled_error
}

/// Coasts to `until` with the gimbal held at `command_rad` by the same loop
/// closed once per cruise exchange.
#[cfg(test)]
fn hold_until(voyage: &mut Voyage, command_rad: f64, until: sim_core::SimInstant) {
    while voyage.ship().now().advanced(CARRY_PERIOD) <= until {
        carry_one_exchange(voyage, command_rad, 0.0, CARRY_PERIOD);
    }
    voyage.universe.advance_to(until).unwrap();
}

/// What must hold of the plant across a change of origin that fell between
/// two exchanges an hour apart.
#[cfg(test)]
fn assert_carried_across(
    voyage: &Voyage,
    before: &Internals,
    after: &Internals,
    command_rad: f64,
    what: &str,
) {
    assert_eq!(
        after.address, before.address,
        "{what}: the plant is the same object"
    );
    assert_eq!(voyage.rebuilds.get(), 0, "{what}: the factory never ran");
    assert_eq!(voyage.universe.tier(PROBE).unwrap(), Tier::Orbital);
    assert_eq!(voyage.universe.state_reinitializations(), 0);
    for at in [before, after] {
        assert!(
            (at.gimbal_rad - command_rad).abs() < HELD_RAD,
            "{what}: the gimbal holds {command_rad} rad: {} rad",
            at.gimbal_rad
        );
    }
    assert_eq!(
        after.xenon_kg.to_bits(),
        before.xenon_kg.to_bits(),
        "{what}: the shut thruster draws nothing"
    );
    assert!(
        (after.temperature_k - before.temperature_k).abs() < 1.0,
        "{what}: the thermal node has no step: {} K to {} K",
        before.temperature_k,
        after.temperature_k
    );
    assert!(
        (after.battery_wh - before.battery_wh).abs() < 1.0,
        "{what}: the battery has no step"
    );
}

/// Coasts an exchange at a time, the gimbal held at `command_rad`, until
/// the scheduler integrates the vessel about `origin`, and returns the
/// internals one exchange before and one after the change.
#[cfg(test)]
fn fly_until_about(
    voyage: &mut Voyage,
    origin: NaifId,
    give_up_s: u64,
    command_rad: f64,
) -> (Internals, Internals) {
    loop {
        let before = voyage.internals();
        carry_one_exchange(voyage, command_rad, 0.0, CARRY_PERIOD);
        if voyage.universe.central_body(PROBE).unwrap() == origin {
            return (before, voyage.internals());
        }
        assert!(
            voyage.universe.now() < at_s(give_up_s),
            "never came to be about {origin:?}"
        );
    }
}

/// Promotes the slewed spacecraft, slews its gimbal again while the orbital
/// tier carries it about the Earth, and flies it out of the Earth's sphere
/// of influence; returns its internals as the Sun's region takes it.
#[cfg(test)]
fn leave_the_earth(voyage: &mut Voyage) -> Internals {
    let local = voyage.internals();
    voyage.universe.promote_to_orbital(PROBE).unwrap();
    let promoted = voyage.internals();
    assert_eq!(promoted, local, "promotion changes no bit of the plant");
    assert_eq!(voyage.universe.central_body(PROBE).unwrap(), NaifId::EARTH);

    let settled_error = slew_while_carried(voyage, ORBITAL_COMMAND_RAD, 1.0, THRUST_UNTIL_S);
    tracing::info!(settled_error, "gimbal slewed while carried about the Earth");
    assert!(
        settled_error < 1e-3,
        "the carried gimbal reached its command about the Earth: {settled_error} rad"
    );
    assert_eq!(voyage.universe.central_body(PROBE).unwrap(), NaifId::EARTH);
    let thrusting = voyage.internals();
    assert!(
        thrusting.xenon_kg < promoted.xenon_kg && promoted.xenon_kg < XENON_KG,
        "the thruster drew xenon in both tiers: {} then {} kg",
        promoted.xenon_kg,
        thrusting.xenon_kg
    );
    let prograde = voyage.vessel_about(NaifId::EARTH).1.normalize();
    voyage.burn(prograde * ESCAPE_DELTA_V_M_S);

    let (before, after) = fly_until_about(voyage, NaifId::SSB, 12 * DAY_S, ORBITAL_COMMAND_RAD);
    assert_carried_across(
        voyage,
        &before,
        &after,
        ORBITAL_COMMAND_RAD,
        "leaving the Earth",
    );
    tracing::info!(
        "heliocentric from {:.2} d; xenon {:.6} kg, gimbal {:.9} rad",
        voyage.universe.now().duration_since(at_s(0)).as_seconds() / 86_400.0,
        after.xenon_kg,
        after.gimbal_rad
    );
    after
}

/// Slews the gimbal to its cruise command while the orbital tier carries
/// the plant about the Sun.
#[cfg(test)]
fn slew_about_the_sun(voyage: &mut Voyage) -> Internals {
    let from_s = voyage.universe.now().duration_since(at_s(0)).as_seconds() as u64;
    let settled_error = slew_while_carried(voyage, CRUISE_COMMAND_RAD, 0.0, from_s + CRUISE_SLEW_S);
    tracing::info!(settled_error, "gimbal slewed while carried about the Sun");
    assert!(
        settled_error < 1e-3,
        "the carried gimbal reached its command about the Sun: {settled_error} rad"
    );
    assert_eq!(voyage.universe.central_body(PROBE).unwrap(), NaifId::SSB);
    voyage.internals()
}

/// The flight time, in days from now, whose transfer costs the least
/// velocity change.
#[cfg(test)]
fn cheapest_flight_days(voyage: &Voyage) -> u64 {
    let cost = |days: u64| {
        let arrival = voyage.tdb().advanced_seconds((days * DAY_S) as f64);
        voyage
            .correction_to_mars(arrival)
            .map_or(f64::INFINITY, |dv| dv.length())
    };
    (30..=64)
        .map(|k| k * 5)
        .min_by(|a, b| cost(*a).total_cmp(&cost(*b)))
        .unwrap()
}

/// Flies the Sun-centred leg with three trims, the gimbal held at its
/// cruise command. Returns the miss between the conic planned at the last
/// trim and the path flown, at the instant Mars's region takes the vessel,
/// and how long before the planned arrival that instant is, s.
#[cfg(test)]
fn cross_to_mars(voyage: &mut Voyage, cruise: &Internals) -> (f64, f64) {
    let departed_s = voyage.universe.now().duration_since(at_s(0)).as_seconds() as u64;
    let flight_days = cheapest_flight_days(voyage);
    let arrival_s = departed_s + flight_days * DAY_S;
    let arrival = voyage.tdb().advanced_seconds((flight_days * DAY_S) as f64);
    let injection = voyage.correction_to_mars(arrival).unwrap();
    tracing::info!(
        "{flight_days} d transfer, injection {:.1} m/s",
        injection.length()
    );
    voyage.burn(injection);
    let mut planned = (voyage.tdb(), voyage.vessel_about(NaifId::SUN));
    for fraction in [0.5, 0.85, 0.97] {
        let trim_s = departed_s + (fraction * (flight_days * DAY_S) as f64) as u64;
        hold_until(voyage, CRUISE_COMMAND_RAD, at_s(trim_s - trim_s % 3_600));
        let trim = voyage.correction_to_mars(arrival).unwrap();
        tracing::info!("trim at {fraction} of the way: {:.2} m/s", trim.length());
        let done = voyage.burn(trim);
        voyage.universe.advance_to(done).unwrap();
        planned = (voyage.tdb(), voyage.vessel_about(NaifId::SUN));
        assert_eq!(voyage.universe.central_body(PROBE).unwrap(), NaifId::SSB);
    }
    let (before, after) = fly_until_about(
        voyage,
        NaifId::MARS,
        arrival_s + 5 * DAY_S,
        CRUISE_COMMAND_RAD,
    );
    assert_carried_across(
        voyage,
        &before,
        &after,
        CRUISE_COMMAND_RAD,
        "entering Mars's region",
    );
    assert_eq!(after.xenon_kg.to_bits(), cruise.xenon_kg.to_bits());
    let (epoch, (position, velocity)) = planned;
    let since_s = voyage.tdb().signed_offset_seconds_from(epoch);
    let flown = voyage.vessel_about(NaifId::SUN).0;
    let entry_s = voyage.universe.now().duration_since(at_s(0)).as_seconds();
    let miss_m = (flown - voyage.conic(position, velocity, since_s)).length();
    (miss_m, arrival_s as f64 - entry_s)
}

/// Slews the gimbal in the local tier, again while carried about the Earth
/// and a third time while carried about the Sun, flying out of the Earth's
/// sphere of influence in between.
#[cfg(test)]
fn depart(voyage: &mut Voyage) -> Internals {
    let start = voyage.internals();
    let settled_error = slew_in_the_local_tier(voyage);
    tracing::info!(settled_error, "gimbal slewed to its command");
    assert!(
        settled_error < 1e-3,
        "the gimbal reached its command: {settled_error} rad"
    );
    assert_eq!(voyage.universe.tier(PROBE).unwrap(), Tier::Local);
    assert_eq!(voyage.internals().address, start.address);
    leave_the_earth(voyage);
    let cruise = slew_about_the_sun(voyage);
    assert_eq!(cruise.address, start.address);
    assert_eq!(voyage.rebuilds.get(), 0);
    cruise
}

/// Local tier, Earth orbital, heliocentric, and the injection towards Mars:
/// the first three regimes of the voyage, in the time a test suite has. The
/// gimbal follows a new command in each of them.
#[test]
fn one_plant_keeps_its_internals_from_the_local_tier_into_heliocentric_flight() {
    let mut voyage = Voyage::begin();
    let cruise = depart(&mut voyage);
    let flight_days = cheapest_flight_days(&voyage);
    let arrival = voyage.tdb().advanced_seconds((flight_days * DAY_S) as f64);
    let injection = voyage.correction_to_mars(arrival).unwrap();
    let done = voyage.burn(injection);
    voyage.universe.advance_to(done).unwrap();
    let residual = voyage.correction_to_mars(arrival).unwrap().length();
    tracing::info!(flight_days, injection_m_s = injection.length(), residual);
    assert!(
        residual < 0.02 * injection.length(),
        "the finite burn put the vessel on the planned conic to within 2 %: {residual} m/s left \
         of {} m/s",
        injection.length()
    );
    for _ in 0..3 {
        carry_one_exchange(&mut voyage, CRUISE_COMMAND_RAD, 0.0, CARRY_PERIOD);
    }
    assert_eq!(voyage.universe.central_body(PROBE).unwrap(), NaifId::SSB);
    let after = voyage.internals();
    assert!(
        (after.gimbal_rad - CRUISE_COMMAND_RAD).abs() < HELD_RAD,
        "the gimbal holds its cruise command: {} rad",
        after.gimbal_rad
    );
    assert_eq!(after.xenon_kg.to_bits(), cruise.xenon_kg.to_bits());
    assert_eq!(after.address, cruise.address);
    assert_eq!(voyage.rebuilds.get(), 0);
}

/// The whole voyage. The orbital integrator ends its steps on the knots of
/// the ephemeris it reads, so ten months of Sun-centred cruise take many
/// more steps than their smoothness needs: a quarter of an hour unoptimised.
#[test]
#[ignore = "flies 306 days of heliocentric cruise: about 17 minutes unoptimised"]
fn one_plant_flies_from_earth_orbit_to_mars_with_its_internals() {
    let mut voyage = Voyage::begin();
    let cruise = depart(&mut voyage);
    let (miss_m, to_centre_s) = cross_to_mars(&mut voyage, &cruise);

    assert_eq!(voyage.universe.central_body(PROBE).unwrap(), NaifId::MARS);
    let (position, velocity) = voyage.vessel_about(NaifId::MARS);
    let energy = 0.5 * velocity.length_squared() - voyage.mu_mars / position.length();
    let momentum = position.cross(velocity).length();
    let eccentricity = (1.0 + 2.0 * energy * momentum.powi(2) / voyage.mu_mars.powi(2)).sqrt();
    let periapsis_m = momentum.powi(2) / (voyage.mu_mars * (1.0 + eccentricity));
    let coast_s = position.length() / velocity.length();
    tracing::info!(
        range_m = position.length(),
        speed_m_s = velocity.length(),
        miss_m,
        periapsis_m,
        to_centre_s,
        "about Mars"
    );
    assert!(position.length() < MARS_SOI_M);
    assert!(energy > 0.0, "it arrives on a hyperbola");
    assert!(
        miss_m < ARRIVAL_MISS_M,
        "the vessel is {miss_m} m off the conic planned at the last trim"
    );
    assert!(
        periapsis_m < ARRIVAL_MISS_M,
        "the approach aimed at Mars's centre passes {periapsis_m} m from it"
    );
    assert!(
        (to_centre_s - coast_s).abs() < 0.25 * DAY_S as f64,
        "it is {coast_s} s of straight flight from the centre, {to_centre_s} s before the \
         planned arrival"
    );
    assert_eq!(voyage.internals().address, cruise.address);
    assert_eq!(voyage.rebuilds.get(), 0);
}
