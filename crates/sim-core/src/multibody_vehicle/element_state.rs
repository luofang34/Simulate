//! Where the states of a vehicle's own elements sit in its model state, and
//! their typed copies outside it.
//!
//! The model state is what the integrator advances. Footpad and wheel
//! states are also kept typed beside their specifications, and canopy
//! states beside the aerodynamic elements, so that a vehicle split or
//! joined to another carries them into a model state of a new layout.

use mbd_aero::AeroElement;
use mbd_multibody::{ContactPointState, ForceElement, ModelState, WheelState};

use super::subsystems::MultibodySubsystems;
use crate::flight_model::coupled_mechanism::CoupledMechanismState;
use crate::flight_model::elements::{VehicleElements, aero_initial_state, aero_lens};
use crate::flight_model::environment::FlightEnvironment;

/// Inflation fraction and deployment latch of a canopy; unused for an
/// element with no state.
pub(crate) type CanopyState = [f64; 2];

/// The state every element of `aero` starts with.
pub(super) fn initial_canopies(aero: &[AeroElement]) -> Vec<CanopyState> {
    aero.iter()
        .map(|element| {
            let (states, modes) = aero_lens(element);
            let mut canopy = [0.0; 2];
            let (state, mode) = canopy.split_at_mut(1);
            aero_initial_state(
                element,
                state.get_mut(..states).unwrap_or(&mut []),
                mode.get_mut(..modes).unwrap_or(&mut []),
            );
            canopy
        })
        .collect()
}

fn split<'a>(cursor: &mut &'a mut [f64], n: usize) -> &'a mut [f64] {
    let whole = core::mem::take(cursor);
    let at = n.min(whole.len());
    let (head, tail) = whole.split_at_mut(at);
    *cursor = tail;
    head
}

/// Writes the typed states of the vehicle's own elements into `state`.
pub(super) fn write_own(
    elements: &VehicleElements,
    canopies: &[CanopyState],
    subsystems: &MultibodySubsystems,
    model_len: usize,
    state: &mut ModelState,
) {
    let (x, modes) = state.parts_mut();
    let mut aux = x.get_mut(model_len..).unwrap_or(&mut []);
    let mut modes = modes;
    for (element, canopy) in elements.aero.iter().zip(canopies) {
        let (states, mode_count) = aero_lens(element);
        for (slot, value) in split(&mut aux, states).iter_mut().zip(canopy.first()) {
            *slot = *value;
        }
        for (slot, value) in split(&mut modes, mode_count).iter_mut().zip(canopy.get(1)) {
            *slot = *value;
        }
    }
    for (_, typed) in &subsystems.contact_points {
        typed.write_blocks(split(&mut aux, 3), split(&mut modes, 1));
    }
    for (_, typed) in &subsystems.wheels {
        for slot in split(&mut aux, 1) {
            *slot = typed.spin_rate_rad_s;
        }
    }
}

/// Reads the states of the vehicle's own elements out of `state` into their
/// typed copies.
pub(super) fn read_own(
    elements: &VehicleElements,
    state: &ModelState,
    canopies: &mut [CanopyState],
    subsystems: &mut MultibodySubsystems,
) {
    let mut aux = state.auxiliary();
    let mut modes = state.modes();
    let take = |cursor: &mut &[f64], n: usize| {
        let (head, tail) = cursor.split_at_checked(n).unwrap_or((&[], &[]));
        *cursor = tail;
        head.to_vec()
    };
    for (element, canopy) in elements.aero.iter().zip(canopies) {
        let (states, mode_count) = aero_lens(element);
        let inflation = take(&mut aux, states);
        let deployed = take(&mut modes, mode_count);
        *canopy = [
            inflation.first().copied().unwrap_or(0.0),
            deployed.first().copied().unwrap_or(0.0),
        ];
    }
    for (_, typed) in &mut subsystems.contact_points {
        *typed = ContactPointState::from_blocks(&take(&mut aux, 3), &take(&mut modes, 1));
    }
    for (_, typed) in &mut subsystems.wheels {
        *typed = WheelState {
            spin_rate_rad_s: take(&mut aux, 1).first().copied().unwrap_or(0.0),
        };
    }
}

/// Copies the leading continuous states and modes of `from` over those of
/// `to`: the vehicle's own part of a state that a step's mechanisms extend.
pub(super) fn copy_prefix(from: &ModelState, to: &mut ModelState) {
    let (x, modes) = to.parts_mut();
    for (slot, value) in x.iter_mut().zip(from.as_slice()) {
        *slot = *value;
    }
    for (slot, value) in modes.iter_mut().zip(from.modes()) {
        *slot = *value;
    }
}

/// The states this step's mechanisms ended in, in their order.
pub(super) fn mechanism_states(
    elements: &VehicleElements,
    state: &ModelState,
) -> Vec<CoupledMechanismState> {
    let (own_states, own_modes) = elements.own_lens();
    let mut aux = state.auxiliary().get(own_states..).unwrap_or(&[]);
    let mut modes = state.modes().get(own_modes..).unwrap_or(&[]);
    elements
        .mechanisms
        .iter()
        .map(|mechanism| {
            let states = ForceElement::<FlightEnvironment<'_>>::state_len(mechanism);
            let mode_count = ForceElement::<FlightEnvironment<'_>>::mode_len(mechanism);
            let (state_block, rest) = aux.split_at_checked(states).unwrap_or((&[], &[]));
            let (mode_block, rest_modes) = modes.split_at_checked(mode_count).unwrap_or((&[], &[]));
            aux = rest;
            modes = rest_modes;
            mechanism.read(state_block, mode_block)
        })
        .collect()
}
