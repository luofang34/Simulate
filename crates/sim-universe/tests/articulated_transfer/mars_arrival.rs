//! A plant built about the Earth and carried by the orbital tier comes down
//! about Mars as the same plant: it makes Mars its body, and its tank, bus,
//! temperatures and gimbal are bit for bit what they were aloft.

use std::sync::Arc;

use glam::DVec3;
use sim_core::Block;
use sim_orbital::{DerivativeOrder, Dop853Tolerances, NaifId};
use sim_universe::{LocalPlant, MultibodyLocalPlant, Tier, UniverseError};

use super::carry_one_exchange;
use super::spacecraft::{BLOCK_PERIOD, XENON_KG};
use super::voyage::{PROBE, Placed, Voyage};

const MARS_RADIUS_M: f64 = 3_396_200.0;
const ALTITUDE_M: f64 = 500_000.0;

/// The spacecraft in a circular orbit 500 km above Mars, in a plant built
/// about the Earth: the state a voyage ends in, placed there directly.
#[cfg(test)]
fn above_mars() -> Voyage {
    Voyage::placed(
        Dop853Tolerances::default(),
        |ephemeris, start, _, mu_mars| {
            let mars = ephemeris
                .geometric_state_icrf(
                    NaifId::MARS,
                    NaifId::EARTH,
                    start,
                    DerivativeOrder::Velocity,
                )
                .unwrap();
            let radius_m = MARS_RADIUS_M + ALTITUDE_M;
            Placed {
                position_from_earth_m: mars.pos_m + DVec3::X * radius_m,
                velocity_from_earth_m_s: mars.vel_m_s + DVec3::Y * (mu_mars / radius_m).sqrt(),
                rate_body_rad_s: DVec3::ZERO,
            }
        },
    )
}

/// Everything of the plant that is not where its base is, printed to the
/// last bit.
#[cfg(test)]
fn internals_in_full(plant: &MultibodyLocalPlant) -> String {
    let block = plant.block();
    let state = block.model_state();
    let subsystems = block.subsystems();
    format!(
        "{:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?}",
        block.tree().states(),
        state.positions().get(7..),
        state.velocities().get(6..),
        state.auxiliary(),
        state.modes(),
        block.mass_properties(),
        subsystems.fluids,
        subsystems.electrical,
        subsystems.thermal,
    )
}

#[test]
fn a_plant_from_the_earth_demoted_about_mars_keeps_its_internals() {
    let mut voyage = above_mars();
    let earth = voyage.universe.bodies().get(NaifId::EARTH).unwrap().clone();
    let mars = voyage.universe.bodies().get(NaifId::MARS).unwrap().clone();
    assert_eq!(voyage.ship().body(), earth.ellipsoid);
    voyage.universe.promote_to_orbital(PROBE).unwrap();
    // Ten minutes aloft with the thruster running and the gimbal slewing, so
    // the internals handed down are not the ones the plant was built with.
    for _ in 0..10 {
        carry_one_exchange(&mut voyage, 0.4, 1.0, BLOCK_PERIOD);
    }
    assert_eq!(voyage.universe.central_body(PROBE).unwrap(), NaifId::MARS);
    let aloft = voyage.internals();
    assert!(aloft.gimbal_rad > 1e-3 && aloft.gimbal_rate_rad_s > 0.0);
    assert!(aloft.xenon_kg < XENON_KG);
    let aloft_in_full = internals_in_full(voyage.ship());
    let about_the_earth = voyage.universe.checkpoint();

    voyage.universe.demote_to_local(PROBE).unwrap();
    assert_eq!(voyage.universe.tier(PROBE).unwrap(), Tier::Local);
    assert_eq!(voyage.rebuilds.get(), 0, "the factory never ran");
    let landed = voyage.internals();
    assert_eq!(
        landed.address, aloft.address,
        "the plant is the same object"
    );
    let ship = voyage.ship();
    assert_eq!(ship.body(), mars.ellipsoid, "the plant flies about Mars");
    assert!(Arc::ptr_eq(ship.block().gravity(), mars.gravity_field()));
    assert_eq!(internals_in_full(ship), aloft_in_full);
    assert_eq!(landed.gimbal_rad.to_bits(), aloft.gimbal_rad.to_bits());
    assert_eq!(landed.xenon_kg.to_bits(), aloft.xenon_kg.to_bits());
    assert_eq!(landed.battery_wh.to_bits(), aloft.battery_wh.to_bits());
    assert_eq!(
        landed.temperature_k.to_bits(),
        aloft.temperature_k.to_bits()
    );

    // It flies on in Mars's field: a tenth of an orbit later it is still on
    // its circle, which the Earth's field at this distance would not hold.
    let radius_m = MARS_RADIUS_M + ALTITUDE_M;
    let orbit_s = std::f64::consts::TAU * (radius_m.powi(3) / voyage.mu_mars).sqrt();
    let steps = (0.1 * orbit_s / BLOCK_PERIOD.as_seconds()) as u32;
    let mut until = voyage.universe.now();
    for _ in 0..steps {
        until = until.advanced(BLOCK_PERIOD);
    }
    voyage.universe.advance_to(until).unwrap();
    assert_eq!(voyage.universe.tier(PROBE).unwrap(), Tier::Local);
    let flown = voyage.ship().block().output();
    let height_m = mars
        .ellipsoid
        .fixed_to_geodetic(flown.position_ecef)
        .height_m;
    tracing::info!(height_m, steps, "a tenth of an orbit about Mars");
    assert!(
        (flown.position_ecef.length() - radius_m).abs() < 5_000.0,
        "{} m from Mars's centre after {steps} steps",
        flown.position_ecef.length()
    );
    assert_eq!(voyage.rebuilds.get(), 0);
    // The plant's state is fixed to Mars from here on, so it refuses the
    // one saved while it was still the Earth's.
    voyage.universe.promote_to_orbital(PROBE).unwrap();
    assert!(matches!(
        voyage.universe.restore(&about_the_earth),
        Err(UniverseError::PlantSnapshotMismatch { vessel: PROBE })
    ));
}
