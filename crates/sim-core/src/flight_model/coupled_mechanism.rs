//! Mechanisms that couple a vehicle to something whose state lives outside
//! it for the span of a step: rails and hinges fired by a coordinator, and
//! one end of a tether or docking ring between two vehicles.

use glam::DQuat;
use mbd_multibody::{
    DockingCapture, DockingState, ElementContext, ForceElement, HingeRelease, HingeSwingState,
    Loads, ModelScale, MultibodyError, PortAnchor, PortKinematics, RailSlide, RailSlideState,
    Tether, TetherAnchor, WinchTetherState,
};

use crate::flight_model::environment::FlightEnvironment;

type Env<'e> = FlightEnvironment<'e>;
type Ctx<'c, 'e> = ElementContext<'c, Env<'e>>;

/// A mechanism that acts on a vehicle for the span of one step with its
/// state supplied from outside: a rail or a hinge on the vehicle itself, or
/// this vehicle's end of a tether or docking ring.
///
/// A far end that is not on the vehicle (a `World` anchor or port) is given
/// as it is at the start of the step, in the inertial axes that coincide
/// with the central body's fixed axes at that instant. Over the step it is
/// carried on at its velocity and turn rate, with the acceleration this
/// vehicle itself has at the start of the step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CoupledMechanism {
    /// A guide-rail ejection on one of the vehicle's prismatic joints.
    Rail(RailSlide, RailSlideState),
    /// A sprung panel on one of the vehicle's revolute joints.
    Hinge(HingeRelease, HingeSwingState),
    /// A tether with at least one anchor on the vehicle.
    Tether(Tether, WinchTetherState),
    /// A soft-capture ring with at least one port on the vehicle.
    Docking(DockingCapture, DockingState),
}

/// The state a [`CoupledMechanism`] reached at the end of a step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CoupledMechanismState {
    /// Latches of a rail.
    Rail(RailSlideState),
    /// Latches of a hinge.
    Hinge(HingeSwingState),
    /// Line state of a tether.
    Tether(WinchTetherState),
    /// Phase, stand-off and dwell of a docking ring.
    Docking(DockingState),
}

fn anchor_at(anchor: TetherAnchor, world: &Env<'_>, elapsed_s: f64) -> TetherAnchor {
    match anchor {
        TetherAnchor::World {
            position_world_m,
            velocity_world_m_s,
        } => {
            let (position_world_m, velocity_world_m_s) =
                world.far_end(position_world_m, velocity_world_m_s, elapsed_s);
            TetherAnchor::World {
                position_world_m,
                velocity_world_m_s,
            }
        }
        on_vehicle @ TetherAnchor::Link { .. } => on_vehicle,
    }
}

fn port_at(anchor: PortAnchor, world: &Env<'_>, elapsed_s: f64) -> PortAnchor {
    match anchor {
        PortAnchor::World(port) => {
            let (position_world_m, velocity_world_m_s) =
                world.far_end(port.position_world_m, port.velocity_world_m_s, elapsed_s);
            let turned = DQuat::from_scaled_axis(port.angular_vel_world_rad_s * elapsed_s);
            PortAnchor::World(PortKinematics {
                position_world_m,
                velocity_world_m_s,
                rotation_world: (turned * port.rotation_world).normalize(),
                ..port
            })
        }
        on_vehicle @ PortAnchor::Link { .. } => on_vehicle,
    }
}

impl CoupledMechanism {
    /// The mechanism with its far ends where `world` carries them
    /// `elapsed_s` into the step.
    pub(crate) fn carried_on(&self, world: &Env<'_>, elapsed_s: f64) -> Self {
        match *self {
            Self::Tether(tether, state) => Self::Tether(
                Tether {
                    anchor_a: anchor_at(tether.anchor_a, world, elapsed_s),
                    anchor_b: anchor_at(tether.anchor_b, world, elapsed_s),
                    ..tether
                },
                state,
            ),
            Self::Docking(ring, state) => Self::Docking(
                DockingCapture {
                    port_a: port_at(ring.port_a, world, elapsed_s),
                    port_b: port_at(ring.port_b, world, elapsed_s),
                    ..ring
                },
                state,
            ),
            on_vehicle @ (Self::Rail(..) | Self::Hinge(..)) => on_vehicle,
        }
    }

    /// Reads the mechanism's state from its blocks of a model state.
    pub(crate) fn read(&self, state: &[f64], modes: &[f64]) -> CoupledMechanismState {
        match self {
            Self::Rail(..) => CoupledMechanismState::Rail(RailSlideState::from_modes(modes)),
            Self::Hinge(..) => CoupledMechanismState::Hinge(HingeSwingState::from_modes(modes)),
            Self::Tether(..) => {
                CoupledMechanismState::Tether(WinchTetherState::from_blocks(state, modes))
            }
            Self::Docking(..) => {
                CoupledMechanismState::Docking(DockingState::from_blocks(state, modes))
            }
        }
    }
}

macro_rules! each_mechanism {
    ($self:expr, $element:ident => $call:expr) => {
        match $self {
            CoupledMechanism::Rail($element, _) => $call,
            CoupledMechanism::Hinge($element, _) => $call,
            CoupledMechanism::Tether($element, _) => $call,
            CoupledMechanism::Docking($element, _) => $call,
        }
    };
}

impl ForceElement<Env<'_>> for CoupledMechanism {
    fn state_len(&self) -> usize {
        each_mechanism!(self, e => ForceElement::<Env<'_>>::state_len(e))
    }

    fn mode_len(&self) -> usize {
        each_mechanism!(self, e => ForceElement::<Env<'_>>::mode_len(e))
    }

    fn has_transitions(&self) -> bool {
        true
    }

    /// The state the mechanism was handed, in place of a fresh one.
    fn initial_state(&self, state: &mut [f64], modes: &mut [f64]) {
        match self {
            Self::Rail(_, s) => s.write_modes(modes),
            Self::Hinge(_, s) => s.write_modes(modes),
            Self::Tether(_, s) => s.write_blocks(state, modes),
            Self::Docking(_, s) => s.write_blocks(state, modes),
        }
    }

    fn state_scale(&self, model: &ModelScale, scale: &mut [f64]) {
        each_mechanism!(self, e => ForceElement::<Env<'_>>::state_scale(e, model, scale));
    }

    fn evaluate(
        &self,
        ctx: &Ctx<'_, '_>,
        state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let seen = self.carried_on(ctx.inputs, ctx.time_s);
        each_mechanism!(&seen, e => e.evaluate(ctx, state, modes, loads, state_dot))
    }

    fn transition(
        &self,
        ctx: &Ctx<'_, '_>,
        state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        let seen = self.carried_on(ctx.inputs, ctx.time_s);
        each_mechanism!(&seen, e => e.transition(ctx, state, modes))
    }
}
