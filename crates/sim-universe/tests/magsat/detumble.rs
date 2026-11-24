//! A tumbling magsat detumbles on IGRF through the orbital tier, converges
//! with its sample period, and keeps its invariants with the torquers off.

use glam::{DMat3, DVec3};
use sim_devices::MagnetometerSpec;
use sim_orbital::VesselId;
use sim_universe::Universe;

use super::orbit::{
    FLIGHT_MAGNETOMETER, World, at_millis, launch, magsat, millis, orbit_period_s, world,
};
use super::satellite::{GAIN_A_M2_S_PER_T, INERTIA_KG_M2, MAX_DIPOLE_A_M2, MagsatSpec};

const SAT: VesselId = VesselId(7);
/// 5°/s about an axis off every principal one.
const TUMBLE_RAD_S: DVec3 = DVec3::new(0.05, -0.06, 0.04);

#[cfg(test)]
fn spec(sample_ms: u64, magnetometer: MagnetometerSpec, torquers_on: bool) -> MagsatSpec {
    MagsatSpec {
        sample_period: millis(sample_ms),
        magnetometer,
        magnetometer_seed: 42,
        torquers_on,
        gravity_gradient: torquers_on,
    }
}

/// The satellite's inertial rate in body axes, rad/s.
#[cfg(test)]
fn rate(universe: &Universe) -> DVec3 {
    universe.inertial_state(SAT).unwrap().omega_body_rad_s
}

/// The field at the satellite, T, as its exchange context would give it.
#[cfg(test)]
fn field_t(world: &World) -> f64 {
    let universe = &world.universe;
    let state = universe.vehicle_state(SAT).unwrap();
    let year = universe
        .epoch()
        .tdb_at(universe.now())
        .approximate_decimal_year();
    magsat(universe, SAT)
        .fields()
        .magnetic
        .field_fixed_nt(state.position_ecef, year)
        .unwrap()
        .length()
        * 1e-9
}

/// B-dot takes the tumble out.
///
/// With the tumble much faster than the orbit the field turns in body axes
/// at `dB/dt = -ω × B`, so `m = -k dB/dt = k ω × B` and the torque is
/// `m × B = -k B² ω⊥`, with `ω⊥` the part of the rate across the field: a
/// drag on that part alone, of rate `k B² / I`. The field direction sweeps
/// through the body as the satellite tumbles and moves along its orbit, so
/// on average two thirds of the rate lies across it and the whole rate
/// decays with the time constant `τ = 3 I / (2 k ⟨B²⟩)`. With the mean
/// principal inertia 0.13 kg·m², `k = 1.5e5` and `⟨B²⟩ ≈ (38 µT)²` that is
/// about 900 s. The estimate ignores the spread of the inertias, the
/// sampling and the part of the rate that lingers along the field, so the
/// measured constant is held to a factor of two of it.
#[test]
fn bdot_detumbles_a_five_degree_per_second_tumble_within_an_orbit() {
    let mut world = world(millis(1_000), 1);
    launch(
        &mut world,
        SAT,
        spec(1_000, FLIGHT_MAGNETOMETER, true),
        0.0,
        TUMBLE_RAD_S,
    );
    let initial = rate(&world.universe).length();
    assert!((initial.to_degrees() - 5.03).abs() < 0.01);
    let orbit_s = orbit_period_s();
    let (mut b2_sum, mut samples, mut peak_dipole) = (0.0, 0_u32, 0.0_f64);
    let (mut at_300, mut at_1500) = (0.0, 0.0);
    let mut second = 0_u64;
    while (second as f64) < orbit_s {
        second += 20;
        world
            .universe
            .advance_to(at_millis(second * 1_000))
            .unwrap();
        peak_dipole = peak_dipole.max(
            magsat(&world.universe, SAT)
                .dipole_a_m2()
                .abs()
                .max_element(),
        );
        if second <= 1_500 {
            b2_sum += field_t(&world).powi(2);
            samples += 1;
        }
        match second {
            300 => at_300 = rate(&world.universe).length(),
            1_500 => at_1500 = rate(&world.universe).length(),
            _ => {}
        }
    }
    let after_one_orbit = rate(&world.universe).length();
    let measured_tau_s = 1_200.0 / (at_300 / at_1500).ln();
    let mean_inertia = INERTIA_KG_M2.element_sum() / 3.0;
    let mean_b2 = b2_sum / f64::from(samples);
    let analytic_tau_s = 3.0 * mean_inertia / (2.0 * GAIN_A_M2_S_PER_T * mean_b2);
    tracing::info!(
        "initial {:.3} deg/s, after one orbit {:.4} deg/s ({:.2} %), tau measured {:.0} s, \
         analytic {:.0} s, mean |B| {:.1} uT, peak dipole {:.3} A m2",
        initial.to_degrees(),
        after_one_orbit.to_degrees(),
        100.0 * after_one_orbit / initial,
        measured_tau_s,
        analytic_tau_s,
        mean_b2.sqrt() * 1e6,
        peak_dipole,
    );
    assert!(
        after_one_orbit < 0.05 * initial,
        "the rate is under 5 % of the tumble after one orbit: {after_one_orbit} rad/s"
    );
    assert!(
        (0.5..2.0).contains(&(measured_tau_s / analytic_tau_s)),
        "decay constant {measured_tau_s} s against the estimate {analytic_tau_s} s"
    );
    assert!(
        peak_dipole < MAX_DIPOLE_A_M2,
        "the torquers never saturated, so the linear estimate applies: {peak_dipole} A·m²"
    );
}

/// The end state of a noiseless detumble sampled every `sample_ms`.
#[cfg(test)]
fn end_rate(sample_ms: u64) -> DVec3 {
    let mut world = world(millis(sample_ms), 1);
    let ideal = spec(sample_ms, MagnetometerSpec::IDEAL, true);
    launch(&mut world, SAT, ideal, 0.4, TUMBLE_RAD_S * 0.2);
    world.universe.advance_to(at_millis(400_000)).unwrap();
    rate(&world.universe)
}

/// The controller differences two readings one period apart and holds its
/// command for the next: both lag the continuous law by half a period, a
/// first-order error. Halving the period therefore halves the distance
/// between successive end states.
#[test]
fn halving_the_sample_period_converges_at_first_order() {
    let [coarse, medium, fine] = [4_000, 2_000, 1_000].map(end_rate);
    let wide = (coarse - medium).length();
    let narrow = (medium - fine).length();
    tracing::info!(
        "end rates {coarse:?} {medium:?} {fine:?} rad/s; differences {wide:.3e}, {narrow:.3e}, \
         ratio {:.3}",
        wide / narrow
    );
    assert!(narrow < wide, "the end state converges monotonically");
    assert!(
        (1.6..2.6).contains(&(wide / narrow)),
        "a first-order sample-and-hold halves the error: ratio {}",
        wide / narrow
    );
    assert!(
        narrow < 0.05 * fine.length(),
        "and at one second the end state is settled to a few percent"
    );
}

/// With the torquers off and no gravity gradient nothing torques the
/// satellite: its rotational energy and its angular momentum in inertial
/// axes hold to the integrator's tolerance.
#[test]
fn an_untorqued_tumble_keeps_its_energy_and_angular_momentum() {
    let mut world = world(millis(10_000), 1);
    let quiet = spec(10_000, FLIGHT_MAGNETOMETER, false);
    launch(&mut world, SAT, quiet, 0.0, TUMBLE_RAD_S);
    let inertia = DMat3::from_diagonal(INERTIA_KG_M2);
    let invariants = |universe: &Universe| {
        let state = universe.inertial_state(SAT).unwrap();
        let momentum_body = inertia * state.omega_body_rad_s;
        (
            0.5 * state.omega_body_rad_s.dot(momentum_body),
            state.q_body_to_icrf * momentum_body,
        )
    };
    let (energy, momentum) = invariants(&world.universe);
    let (mut worst_energy, mut worst_momentum) = (0.0_f64, 0.0_f64);
    let mut second = 0_u64;
    while (second as f64) < orbit_period_s() {
        second += 100;
        world
            .universe
            .advance_to(at_millis(second * 1_000))
            .unwrap();
        let (e, h) = invariants(&world.universe);
        worst_energy = worst_energy.max((e / energy - 1.0).abs());
        worst_momentum = worst_momentum.max((h - momentum).length() / momentum.length());
        assert_eq!(magsat(&world.universe, SAT).dipole_a_m2(), DVec3::ZERO);
    }
    tracing::info!(
        "relative drift: energy {worst_energy:.3e}, angular momentum {worst_momentum:.3e}"
    );
    // The integrator holds each step's attitude and rate to 1e-9 absolute;
    // over the revolution's steps that leaves parts in 1e-9 of drift.
    assert!(
        worst_energy < 1e-8,
        "rotational energy drifted {worst_energy}"
    );
    assert!(
        worst_momentum < 1e-8,
        "angular momentum drifted {worst_momentum}"
    );
}
