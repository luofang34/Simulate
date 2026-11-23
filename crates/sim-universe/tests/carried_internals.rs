//! A thrusting spacecraft keeps its internals through the orbital tier.
//!
//! The vessel starts in low Earth orbit in the local tier with its electric
//! thruster running, is promoted by the tier policy, flies about five
//! revolutions in the orbital tier, each with an eclipse its battery cannot
//! bridge, and is demoted. Its tank, battery and temperature are the same
//! objects throughout, and the orbit it ends on is the one its own thrust
//! raised.

use std::cell::Cell;
use std::rc::Rc;

use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan};
use sim_geo::BodyEllipsoid;
use sim_orbital::{LatchedBurnLaw, PlayerCommandPayload, VesselId};
use sim_universe::{MultibodyLocalPlant, Tier, TierChange, TierPolicy, Universe};

#[path = "carried_internals/observe.rs"]
mod observe;
#[path = "carried_internals/spacecraft.rs"]
mod spacecraft;

use observe::{
    Internals, Phase, fingerprint, held_mass, internals, latched_thrust_n, relative, ship,
};
use spacecraft::{
    BATTERY_CAPACITY_WH, BATTERY_MIN_SOC, CARRY_PERIOD, DRY_MASS_KG, EXHAUST_VELOCITY_M_S,
    FULL_THRUST_N, Launch, ORBIT_RADIUS_M, XENON_KG, context, counting_factory, launch,
    orbital_energy_j_kg, plant, seconds,
};

const SHIP: VesselId = VesselId(1);
/// The policy's first check, where it promotes the vessel.
const PROMOTED_S: u64 = 10;
/// Exchanges flown in the orbital tier: 27,700 s, five revolutions of 92.6 min.
const EXCHANGES: u64 = 2_770;
const DEMOTED_S: u64 = PROMOTED_S + EXCHANGES * 10;

#[cfg(test)]
fn universe_with(launch: &Launch, calls: &Rc<Cell<u32>>, carried: bool, policy: bool) -> Universe {
    let mut universe = Universe::new(launch.epoch.clone(), context());
    if policy {
        universe = universe.with_tier_policy(TierPolicy {
            check_period: CARRY_PERIOD,
            ..TierPolicy::standard()
        });
    }
    let ship = plant(
        BodyEllipsoid::WGS84,
        &launch.state,
        SimInstant::START,
        &launch.epoch,
        launch.sun_fixed,
        carried,
    )
    .unwrap();
    let factory = counting_factory(Rc::clone(calls), launch.sun_fixed, carried);
    universe.insert_local(SHIP, ship, factory).unwrap();
    universe
}

/// Advances to `until` in slices of `slice`, or in one call.
#[cfg(test)]
fn advance(universe: &mut Universe, until: SimInstant, slice: Option<TickSpan>) {
    if let Some(slice) = slice {
        while universe.now().advanced(slice) < until {
            let next = universe.now().advanced(slice);
            universe.advance_to(next).unwrap();
        }
    }
    universe.advance_to(until).unwrap();
}

/// What the vessel did over its exchanges in the orbital tier.
struct Aloft {
    /// Thrust the orbital tier held, summed over the time it held it, N·s.
    orbital_impulse_n_s: f64,
    eclipses: u32,
    /// Exchanges that found the thruster shed in shadow.
    shed_samples: u32,
    min_battery_wh: f64,
    after_ten_minutes: Internals,
    last: Internals,
}

/// One exchange's worth of the bus model's behaviour: `now` against the
/// exchange before it. `true` when the thruster stayed shed between them.
#[cfg(test)]
fn check_exchange(universe: &Universe, previous: &Internals, now: &Internals) -> bool {
    let floor_wh = BATTERY_CAPACITY_WH * BATTERY_MIN_SOC;
    if previous.lit == 1.0 && now.lit < 1.0 {
        assert!(
            relative(previous.battery_wh, BATTERY_CAPACITY_WH) < 1e-9,
            "the array refills the battery before each eclipse: {} Wh",
            previous.battery_wh
        );
    }
    if now.lit == 0.0 {
        assert!(
            now.battery_wh < previous.battery_wh || now.battery_wh <= floor_wh * (1.0 + 1e-12),
            "with no sunlight the thruster runs the battery down"
        );
    }
    if now.lit == 1.0 && previous.lit == 1.0 {
        assert!(
            relative(latched_thrust_n(universe), FULL_THRUST_N) < 1e-12,
            "in sunlight the bus grants the thruster full power"
        );
    }
    let shed = now.lit == 0.0 && previous.lit == 0.0 && previous.thrust_n == 0.0;
    if shed {
        assert_eq!(
            now.tank_kg.to_bits(),
            previous.tank_kg.to_bits(),
            "a shed thruster draws no xenon"
        );
        assert_eq!(latched_thrust_n(universe), 0.0, "and pushes on no orbit");
    }
    shed
}

/// Flies the promoted vessel through every exchange, one at a time.
#[cfg(test)]
fn fly_exchanges(universe: &mut Universe, promoted: Internals) -> Aloft {
    let mut aloft = Aloft {
        orbital_impulse_n_s: 0.0,
        eclipses: 0,
        shed_samples: 0,
        min_battery_wh: f64::MAX,
        after_ten_minutes: promoted,
        last: promoted,
    };
    for exchange in 1..=EXCHANGES {
        aloft.orbital_impulse_n_s += latched_thrust_n(universe) * 10.0;
        universe
            .advance_to(seconds(PROMOTED_S + exchange * 10))
            .unwrap();
        let now = internals(universe);
        if exchange == 60 {
            aloft.after_ten_minutes = now;
        }
        if aloft.last.lit == 1.0 && now.lit < 1.0 {
            aloft.eclipses += 1;
        }
        let held_kg = held_mass(universe).mass_kg;
        assert_eq!(
            held_kg.to_bits(),
            ship(universe)
                .block()
                .mass_properties()
                .total_mass_kg
                .to_bits(),
            "the orbital tier holds the mass the plant reported at this exchange"
        );
        let one_step_kg = FULL_THRUST_N / EXHAUST_VELOCITY_M_S;
        assert!(
            (held_kg - (DRY_MASS_KG + now.tank_kg)).abs() <= one_step_kg * (1.0 + 1e-6),
            "which is the tank's, to the one block step by which the block's mass \
             properties trail its propulsion: {held_kg} kg"
        );
        if check_exchange(universe, &aloft.last, &now) {
            aloft.shed_samples += 1;
        }
        assert!(
            universe.orbital().command_log().entries().is_empty(),
            "the exchange holds the loads in the vessel's checkpoint, not as commands"
        );
        aloft.min_battery_wh = aloft.min_battery_wh.min(now.battery_wh);
        aloft.last = now;
    }
    aloft
}

/// The vessel at the policy's first check, beside one flown locally to the
/// same instant with no policy.
#[cfg(test)]
fn promote_by_policy(launch: &Launch, calls: &Rc<Cell<u32>>) -> (Universe, Internals) {
    let mut universe = universe_with(launch, calls, true, true);
    let mut local_only = universe_with(launch, &Rc::new(Cell::new(0)), true, false);
    local_only.advance_to(seconds(PROMOTED_S)).unwrap();
    universe.advance_to(seconds(PROMOTED_S)).unwrap();
    assert_eq!(
        universe.take_tier_changes(),
        vec![TierChange {
            vessel: SHIP,
            at: seconds(PROMOTED_S),
            to: Tier::Orbital
        }],
        "the policy promotes a vessel under small steady thrust at its first check"
    );
    let promoted = internals(&universe);
    assert_eq!(
        promoted,
        Internals {
            lit: promoted.lit,
            ..internals(&local_only)
        },
        "promotion moves the plant, so its internals are bit for bit those of local flight"
    );
    assert_eq!(
        latched_thrust_n(&universe).to_bits(),
        promoted.thrust_n.to_bits(),
        "the thrust the plant was making is latched from the promotion instant"
    );
    (universe, promoted)
}

#[test]
fn internals_run_in_orbit_as_the_bus_and_tank_models_dictate() {
    let launch = launch();
    let (mut universe, promoted) = promote_by_policy(&launch, &Rc::new(Cell::new(0)));
    let aloft = fly_exchanges(&mut universe, promoted);

    assert!(
        aloft.eclipses >= 4,
        "five revolutions cross the shadow: {} eclipses",
        aloft.eclipses
    );
    assert!(
        relative(aloft.min_battery_wh, BATTERY_CAPACITY_WH * BATTERY_MIN_SOC) < 1e-9,
        "each eclipse drains the battery to its protected floor: {} Wh",
        aloft.min_battery_wh
    );
    assert!(
        aloft.shed_samples > 100,
        "then the bus sheds the thruster: {} exchanges",
        aloft.shed_samples
    );
    let steady_kg = promoted.tank_kg - aloft.after_ten_minutes.tank_kg;
    assert!(
        relative(steady_kg, FULL_THRUST_N * 600.0 / EXHAUST_VELOCITY_M_S) < 1e-9,
        "at full power the tank drains by thrust x time / exhaust velocity: {steady_kg} kg"
    );
    let used_kg = promoted.tank_kg - aloft.last.tank_kg;
    assert!(
        relative(used_kg * EXHAUST_VELOCITY_M_S, aloft.orbital_impulse_n_s) < 1e-6,
        "the impulse the orbit received is the one the tank paid for, the propellant \
         integrated once: {used_kg} kg against {} N s",
        aloft.orbital_impulse_n_s
    );
    assert!(
        (aloft.last.temperature_k - promoted.temperature_k).abs() > 1.0
            && (150.0..400.0).contains(&aloft.last.temperature_k),
        "the thermal node keeps integrating in orbit: {} K",
        aloft.last.temperature_k
    );
}

#[test]
fn internals_are_continuous_across_promotion_and_demotion() {
    let launch = launch();
    let calls = Rc::new(Cell::new(0));
    let (mut universe, promoted) = promote_by_policy(&launch, &calls);
    universe.advance_to(seconds(DEMOTED_S)).unwrap();
    let aloft = internals(&universe);
    assert!(
        aloft.tank_kg < promoted.tank_kg - 0.1 && aloft.temperature_k != promoted.temperature_k,
        "the internals moved on while the orbital tier flew the vessel"
    );

    let before = universe.vehicle_state(SHIP).unwrap();
    universe.demote_to_local(SHIP).unwrap();
    assert_eq!(
        calls.get(),
        0,
        "the carried plant comes back; the factory is never asked"
    );
    assert_eq!(
        internals(&universe),
        aloft,
        "demotion hands the base state to the same plant and touches nothing inside it"
    );
    let after = universe.vehicle_state(SHIP).unwrap();
    assert!(
        (after.position_ecef - before.position_ecef).length() < 1e-3
            && (after.velocity_ecef - before.velocity_ecef).length() < 1e-6,
        "the plant resumes from the state the orbital tier reached"
    );
    universe.advance_to(seconds(DEMOTED_S + 5)).unwrap();
    let landed = internals(&universe);
    assert!(
        relative(
            aloft.tank_kg - landed.tank_kg,
            5.0 * landed.thrust_n / EXHAUST_VELOCITY_M_S
        ) < 1e-9
            && landed.thrust_n > 0.0,
        "back in the local tier the same thruster keeps drawing the same tank"
    );
}

#[test]
fn carried_thrust_raises_the_orbit_as_low_thrust_theory_predicts() {
    let launch = launch();
    let carried_calls = Rc::new(Cell::new(0));
    let dropped_calls = Rc::new(Cell::new(0));
    let mut carried = universe_with(&launch, &carried_calls, true, true);
    let mut dropped = universe_with(&launch, &dropped_calls, false, false);
    dropped.advance_to(seconds(PROMOTED_S)).unwrap();
    let tank_at_promotion_kg = internals(&dropped).tank_kg;
    dropped.promote_to_orbital(SHIP).unwrap();
    assert!(
        dropped.carried::<MultibodyLocalPlant>(SHIP).is_none(),
        "a plant that cannot be carried is dropped on promotion"
    );
    carried.advance_to(seconds(DEMOTED_S)).unwrap();
    dropped.advance_to(seconds(DEMOTED_S)).unwrap();

    let earth = BodyEllipsoid::WGS84;
    let semi_major = |universe: &Universe| {
        -earth.mu_m3_s2 / (2.0 * orbital_energy_j_kg(&universe.vehicle_state(SHIP).unwrap()))
    };
    let raised_m = semi_major(&carried) - semi_major(&dropped);
    let used_kg = tank_at_promotion_kg - internals(&carried).tank_kg;
    let mean_mass_kg = DRY_MASS_KG + tank_at_promotion_kg - 0.5 * used_kg;
    let mean_motion = (earth.mu_m3_s2 / ORBIT_RADIUS_M.powi(3)).sqrt();
    let delta_v = used_kg * EXHAUST_VELOCITY_M_S / mean_mass_kg;
    let predicted_m = 2.0 * delta_v / mean_motion;
    assert!(
        relative(raised_m, predicted_m) < 0.005,
        "tangential thrust raises a circular orbit by 2 dv / n; the body-fixed thruster \
         drifts a few degrees off the velocity over five revolutions, a cosine loss of \
         parts in a thousand: {raised_m} m against {predicted_m} m for {delta_v} m/s"
    );

    dropped.demote_to_local(SHIP).unwrap();
    carried.demote_to_local(SHIP).unwrap();
    assert_eq!(
        (dropped_calls.get(), carried_calls.get()),
        (1, 0),
        "only the plant that was dropped is rebuilt by its factory"
    );
    assert!(
        internals(&dropped).tank_kg == XENON_KG && tank_at_promotion_kg < XENON_KG,
        "the rebuilt plant forgot what it had burnt: its tank is full again"
    );
    assert!(
        internals(&carried).tank_kg < tank_at_promotion_kg - 0.1,
        "the carried plant kept its tank and went on drawing from it"
    );
}

#[test]
fn a_planned_burn_on_a_carried_vessel_flies_beside_the_plant_thrust() {
    let launch = launch();
    let burn = LatchedBurnLaw::InertialThrust {
        dir_icrf: [0.0, 0.0, 1.0],
        thrust_n: 0.05,
        exhaust_vel_m_s: 3_000.0,
        rw_torque_cmd_body_nm: [0.0; 3],
    };
    let (mut planned, _) = promote_by_policy(&launch, &Rc::new(Cell::new(0)));
    let (mut unplanned, _) = promote_by_policy(&launch, &Rc::new(Cell::new(0)));
    planned
        .command(
            SHIP,
            seconds(PROMOTED_S),
            PlayerCommandPayload::SetBurnLaw(burn),
        )
        .unwrap();
    for universe in [&mut planned, &mut unplanned] {
        universe.advance_to(seconds(PROMOTED_S + 60)).unwrap();
    }

    let checkpoint = planned.orbital().committed_checkpoint(SHIP).unwrap();
    assert_eq!(
        checkpoint.gnc.latched_law, burn,
        "six exchanges later the plan's burn is still the one latched"
    );
    assert!(
        relative(latched_thrust_n(&planned), FULL_THRUST_N) < 1e-12,
        "beside the plant's own thrust"
    );
    assert_eq!(
        planned.orbital().command_log().entries().len(),
        1,
        "and the plan's command is the only one logged"
    );
    let velocity = |u: &Universe| u.inertial_state(SHIP).unwrap().vel_rel_icrf_m_s;
    let gained = (velocity(&planned) - velocity(&unplanned)).z;
    let expected = 0.05 * 60.0 / held_mass(&planned).mass_kg;
    assert!(
        relative(gained, expected) < 0.01,
        "the burn adds its own impulse: {gained} m/s against {expected} m/s"
    );
}

#[test]
fn the_outcome_does_not_depend_on_how_the_advance_is_sliced() {
    let launch = launch();
    let fly = |slice: Option<TickSpan>| {
        let mut universe = universe_with(&launch, &Rc::new(Cell::new(0)), true, true);
        advance(&mut universe, seconds(DEMOTED_S), slice);
        let aloft = fingerprint(&universe);
        universe.demote_to_local(SHIP).unwrap();
        advance(&mut universe, seconds(DEMOTED_S + 5), slice);
        (
            aloft,
            fingerprint(&universe),
            universe.vehicle_state(SHIP).unwrap(),
        )
    };
    let whole = fly(None);
    // Seven seconds and a third never line up with the ten-second exchanges.
    let sevens = fly(Some(TickSpan::from_ticks(TICKS_PER_SECOND * 22 / 3)));
    let minutes = fly(Some(TickSpan::from_seconds_and_subsec_ticks(60, 0)));
    // With no policy to stop at its checks, one call spans every exchange.
    let unchecked = {
        let mut universe = universe_with(&launch, &Rc::new(Cell::new(0)), true, false);
        universe.advance_to(seconds(PROMOTED_S)).unwrap();
        universe.promote_to_orbital(SHIP).unwrap();
        universe.advance_to(seconds(DEMOTED_S)).unwrap();
        fingerprint(&universe)
    };
    assert_eq!(
        whole.0, unchecked,
        "one advance across 2,770 exchanges stops at each of them by itself"
    );
    assert_eq!(
        whole, sevens,
        "exchanges happen at their own instants, not where a caller's slices end"
    );
    assert_eq!(
        whole, minutes,
        "and a slice spanning several exchanges runs each of them"
    );
}

/// Checkpoints at `at`, flies on to `until`, restores, and flies to `until`
/// again: both arrivals must be the same run.
#[cfg(test)]
fn replay_from(universe: &mut Universe, at: SimInstant, until: SimInstant) {
    let saved = universe.checkpoint();
    let at_checkpoint = fingerprint(universe);
    universe.advance_to(until).unwrap();
    let continued = fingerprint(universe);
    assert_ne!(continued, at_checkpoint);
    universe.restore(&saved).unwrap();
    assert_eq!(
        fingerprint(universe),
        at_checkpoint,
        "restoring at {at:?} puts the carried plant back with the orbital tier"
    );
    universe.advance_to(until).unwrap();
    assert_eq!(
        fingerprint(universe),
        continued,
        "a checkpoint at {at:?} holds the plant's internals and the orbital tier's \
         buffered steps, so the replay is the same run"
    );
}

#[test]
fn checkpoints_mid_orbit_replay_bit_for_bit_with_steps_buffered_in_every_phase() {
    let launch = launch();
    let mut universe = universe_with(&launch, &Rc::new(Cell::new(0)), true, true);
    // Between two exchanges and between two block steps.
    let mid = TickSpan::from_ticks(TICKS_PER_SECOND * 11 / 2);
    let mut taken = Vec::new();
    let mut exchange_s = PROMOTED_S;
    while taken.len() < 6 && exchange_s < DEMOTED_S - 600 {
        let at = seconds(exchange_s).advanced(mid);
        universe.advance_to(at).unwrap();
        let phase = internals(&universe).phase();
        let buffered = universe.orbital().speculative_segment_count(SHIP).unwrap();
        let wanted = taken.iter().filter(|&&(p, _)| p == phase).count() < 2;
        if buffered > 1 && wanted {
            replay_from(&mut universe, at, seconds(exchange_s + 300));
            taken.push((phase, buffered));
            exchange_s += 300;
        } else {
            exchange_s += 10;
        }
    }
    let phases: Vec<Phase> = taken.iter().map(|&(phase, _)| phase).collect();
    assert!(
        [Phase::Sunlit, Phase::OnBattery, Phase::Shed]
            .iter()
            .all(|phase| phases.iter().filter(|&p| p == phase).count() == 2),
        "two checkpoints in sunlight, on battery and shed in the umbra, each with \
         several steps buffered: {taken:?}"
    );

    universe.advance_to(seconds(DEMOTED_S)).unwrap();
    let mut uninterrupted = universe_with(&launch, &Rc::new(Cell::new(0)), true, true);
    uninterrupted.advance_to(seconds(DEMOTED_S)).unwrap();
    assert_eq!(
        fingerprint(&universe),
        fingerprint(&uninterrupted),
        "a run restored six times is the run that was never interrupted"
    );
}
