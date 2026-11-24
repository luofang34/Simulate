//! A gimbal slewed by a fixed torque profile while the orbital tier carries
//! the spacecraft: the angular momentum of bus and antenna together stays
//! what it was, and the bus ends where flying the same slew in the local
//! tier puts it, the closer the more often the tiers exchange.

use glam::{DQuat, DVec3};
use sim_core::{TICKS_PER_SECOND, TickSpan, VehicleIntegrator};
use sim_orbital::Dop853Tolerances;
use sim_universe::{LocalPlant, Tier};

use super::spacecraft::{ANTENNA_INERTIA_KG_M2, BLOCK_PERIOD, input};
use super::voyage::{PROBE, Placed, Voyage, at_s};

/// The motor's torque for the first half of the slew, N·m, and its opposite
/// for the second.
const MOTOR_NM: f64 = 2.0e-6;
const HALF_SLEW_S: u64 = 1_200;
const END_S: u64 = 3_600;
/// Slow tumble so the antenna's momentum couples gently into every axis.
const TUMBLE_RAD_S: DVec3 = DVec3::new(2.0e-4, -1.0e-4, 1.5e-4);
/// Slow multi-axis spin (~0.05 rad/s), where RK4 with h = 10 s begins to need
/// adaptive stepping in carried mode.
const SLOW_SPIN_RAD_S: DVec3 = DVec3::new(0.004, 0.050, 0.0);
/// Fast multi-axis spin (~0.50 rad/s), far beyond RK4 stability at h = 10 s.
const FAST_SPIN_RAD_S: DVec3 = DVec3::new(0.020, 0.500, 0.0);

#[cfg(test)]
fn motor_nm(at: u64) -> f64 {
    match at {
        t if t < HALF_SLEW_S => MOTOR_NM,
        t if t < 2 * HALF_SLEW_S => -MOTOR_NM,
        _ => 0.0,
    }
}

/// The spinning spacecraft in its parking orbit, integrated with `integrator`.
#[cfg(test)]
fn tumbling(rate_body_rad_s: DVec3, integrator: VehicleIntegrator) -> Voyage {
    let tolerances = Dop853Tolerances {
        rtol: 1.0e-12,
        atol_att_rad: 1.0e-13,
        atol_omega_rad_s: 1.0e-15,
        ..Dop853Tolerances::default()
    };
    Voyage::placed_with_integrator(tolerances, integrator, |_, _, mu_earth, _| {
        let radius_m = 6_778_137.0;
        Placed {
            position_from_earth_m: DVec3::X * radius_m,
            velocity_from_earth_m_s: DVec3::Y * (mu_earth / radius_m).sqrt(),
            rate_body_rad_s,
        }
    })
}

/// Angular momentum of bus and antenna about their centre of gravity in
/// inertial axes, N·m·s: the bus's rate through the inertia of both, plus
/// the antenna turning on its gimbal, whose axis is the bus's Z.
#[cfg(test)]
fn angular_momentum_icrf(voyage: &Voyage) -> DVec3 {
    let orbital = voyage.universe.inertial_state(PROBE).unwrap();
    let inertia = voyage.ship().block().mass_properties().inertia_cg_body;
    let gimbal = DVec3::Z * ANTENNA_INERTIA_KG_M2 * voyage.internals().gimbal_rate_rad_s;
    orbital.q_body_to_icrf * (inertia * orbital.omega_body_rad_s + gimbal)
}

/// What a slew ends in.
#[cfg(test)]
struct Slewed {
    attitude: DQuat,
    gimbal_rad: f64,
    /// Largest departure of the angular momentum from its first value over
    /// the exchanges, as a fraction of it.
    momentum_drift: f64,
}

/// Flies the slew with the plant carried (`FixedRk4(6)`), exchanging every `exchange_s`.
#[cfg(test)]
fn carried(rate_body_rad_s: DVec3, exchange_s: u64) -> Slewed {
    let mut voyage = tumbling(rate_body_rad_s, VehicleIntegrator::FixedRk4 { substeps: 6 });
    voyage.universe.promote_to_orbital(PROBE).unwrap();
    let exchange = TickSpan::from_ticks(exchange_s * TICKS_PER_SECOND);
    let first = angular_momentum_icrf(&voyage);
    let mut momentum_drift = 0.0_f64;
    for at in (0..END_S).step_by(exchange_s as usize) {
        let commanded = input(voyage.ship().input(), 0.0, motor_nm(at));
        let ship = voyage.ship_mut();
        ship.set_carry_period(exchange);
        ship.set_input(commanded);
        voyage.universe.advance_to(at_s(at + exchange_s)).unwrap();
        assert_eq!(voyage.ship().now(), at_s(at + exchange_s));
        let drift = (angular_momentum_icrf(&voyage) - first).length() / first.length();
        momentum_drift = momentum_drift.max(drift);
    }
    Slewed {
        attitude: voyage.universe.vehicle_state(PROBE).unwrap().attitude,
        gimbal_rad: voyage.internals().gimbal_rad,
        momentum_drift,
    }
}

/// Flies the slew with the plant flying itself: bus and antenna integrated
/// together at every stage.
#[cfg(test)]
fn flown_locally(rate_body_rad_s: DVec3, integrator: VehicleIntegrator) -> Slewed {
    let mut voyage = tumbling(rate_body_rad_s, integrator);
    let step_s = BLOCK_PERIOD.as_seconds() as u64;
    for at in (0..END_S).step_by(step_s as usize) {
        let commanded = input(voyage.ship().input(), 0.0, motor_nm(at));
        voyage.ship_mut().set_input(commanded);
        voyage.universe.advance_to(at_s(at + step_s)).unwrap();
    }
    Slewed {
        attitude: voyage.universe.vehicle_state(PROBE).unwrap().attitude,
        gimbal_rad: voyage.internals().gimbal_rad,
        momentum_drift: 0.0,
    }
}

#[cfg(test)]
fn verify_slew_conserves_and_converges(
    rate_body_rad_s: DVec3,
    local_integrator: VehicleIntegrator,
    max_att_err: f64,
    max_gimbal_err: f64,
) {
    let local = flown_locally(rate_body_rad_s, local_integrator);
    assert!(
        local.gimbal_rad.abs() > 0.5,
        "the antenna slews through {} rad",
        local.gimbal_rad
    );
    let mut errors = Vec::new();
    for exchange_s in [1_200, 600, 300] {
        let slewed = carried(rate_body_rad_s, exchange_s);
        let attitude_error = slewed.attitude.angle_between(local.attitude);
        let gimbal_error = (slewed.gimbal_rad - local.gimbal_rad).abs();
        tracing::info!(
            exchange_s,
            momentum_drift = slewed.momentum_drift,
            attitude_error,
            gimbal_error,
            "carried slew"
        );
        assert!(
            slewed.momentum_drift < 1e-11,
            "exchanging every {exchange_s} s the angular momentum drifts by {:e} of itself",
            slewed.momentum_drift
        );
        errors.push((attitude_error, gimbal_error));
    }
    for pair in errors.windows(2) {
        let ((coarse, coarse_gimbal), (fine, fine_gimbal)) = (pair[0], pair[1]);
        assert!(
            fine <= 0.6 * coarse || fine < 1e-4,
            "halving the exchange period left attitude error at {coarse} -> {fine} rad"
        );
        assert!(
            fine_gimbal <= 0.6 * coarse_gimbal || fine_gimbal < 1e-5,
            "halving the exchange period left gimbal error at {coarse_gimbal} -> {fine_gimbal} rad"
        );
    }
    let (finest, finest_gimbal) = errors[2];
    assert!(finest < max_att_err, "the bus ends {finest} rad off");
    assert!(
        finest_gimbal < max_gimbal_err,
        "the gimbal ends {finest_gimbal} rad off"
    );
}

#[test]
fn a_carried_slew_conserves_angular_momentum_and_converges_on_the_local_tier() {
    verify_slew_conserves_and_converges(
        TUMBLE_RAD_S,
        VehicleIntegrator::FixedRk4 { substeps: 6 },
        2e-4,
        1e-3,
    );
}

#[test]
fn a_carried_slew_on_slowly_spinning_spacecraft_conserves_momentum_and_converges() {
    verify_slew_conserves_and_converges(
        SLOW_SPIN_RAD_S,
        VehicleIntegrator::adaptive(1e-11, 1e-11),
        2e-4,
        2.5e-3,
    );
}

#[test]
fn a_carried_slew_on_fast_spinning_spacecraft_conserves_momentum_and_converges() {
    verify_slew_conserves_and_converges(
        FAST_SPIN_RAD_S,
        VehicleIntegrator::adaptive(1e-11, 1e-11),
        2e-4,
        2.5e-2,
    );
}

/// Angular momentum of bus and antenna about their centre of gravity in the
/// Earth's fixed axes of this instant, whichever tier flies the vessel.
#[cfg(test)]
fn angular_momentum_fixed(voyage: &Voyage) -> DVec3 {
    let state = voyage.universe.vehicle_state(PROBE).unwrap();
    let spin_fixed = DVec3::Z * voyage.ship().body().omega_rad_s;
    let rate_body = state.angular_rate_body + state.attitude.conjugate() * spin_fixed;
    let m_bb = voyage.ship().block().mass_properties().inertia_cg_body;
    let (inertia, internal) = match voyage.universe.tier(PROBE).unwrap() {
        Tier::Orbital => {
            let (i_orb, h_held) = voyage
                .universe
                .orbital_held_rotational_state_body(PROBE)
                .unwrap();
            let w_orb = voyage
                .universe
                .inertial_state(PROBE)
                .unwrap()
                .omega_body_rad_s;
            (i_orb, h_held - (m_bb - i_orb) * (w_orb - rate_body))
        }
        Tier::Local => (
            m_bb,
            DVec3::Z * ANTENNA_INERTIA_KG_M2 * voyage.internals().gimbal_rate_rad_s,
        ),
    };
    state.attitude * (inertia * rate_body + internal)
}

#[test]
fn a_demotion_between_exchanges_hands_the_bus_the_reaction_of_the_slew() {
    let mut voyage = tumbling(TUMBLE_RAD_S, VehicleIntegrator::FixedRk4 { substeps: 6 });
    voyage.universe.promote_to_orbital(PROBE).unwrap();
    let commanded = input(voyage.ship().input(), 0.0, MOTOR_NM);
    let ship = voyage.ship_mut();
    ship.set_carry_period(TickSpan::from_ticks(600 * TICKS_PER_SECOND));
    ship.set_input(commanded);
    voyage.universe.advance_to(at_s(900)).unwrap();
    assert_eq!(voyage.ship().now(), at_s(600));
    let aloft = angular_momentum_fixed(&voyage);
    let rate_aloft = voyage.internals().gimbal_rate_rad_s;

    voyage.universe.demote_to_local(PROBE).unwrap();
    assert_eq!(voyage.universe.tier(PROBE).unwrap(), Tier::Local);
    assert_eq!(voyage.rebuilds.get(), 0);
    assert_eq!(voyage.ship().now(), at_s(900));
    let rate_down = voyage.internals().gimbal_rate_rad_s;
    assert!(
        rate_down > 1.4 * rate_aloft,
        "the antenna went on slewing on the way down: {rate_aloft} to {rate_down} rad/s"
    );
    let down = angular_momentum_fixed(&voyage);
    assert!(
        (down - aloft).length() < 1e-12 * aloft.length(),
        "the demotion took the angular momentum from {aloft} to {down}"
    );
}
