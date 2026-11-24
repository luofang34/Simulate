//! A board at one instant: its airframe, its devices and its flight controller, kept
//! whole so a run returns to the instant and flies on from it exactly as it did before.

use aviate_core::control::VehicleController;
use aviate_core::mixer::Mixer;
use sim_aviate::{AviateDirectRunner, DirectActuatorCmd};
use sim_core::SimInstant;
use sim_devices::{DeviceSamples, SensorSuite};

use super::{DeviceOutages, HostedBoard};
use crate::{Airframe, ChainError};

/// A [`HostedBoard`] at one instant.
pub struct BoardSnapshot<A, C, M>
where
    A: Airframe,
    C: VehicleController,
    M: Mixer,
{
    airframe: A::Snapshot,
    suite: SensorSuite,
    controller: Box<AviateDirectRunner<C, M>>,
    now: SimInstant,
    samples: DeviceSamples,
    outages: DeviceOutages,
    command: DirectActuatorCmd,
}

impl<A, C, M> HostedBoard<A, C, M>
where
    A: Airframe,
    C: VehicleController + Clone,
    M: Mixer + Clone,
{
    /// The board as it stands: the airframe, the devices with the noise they have yet
    /// to draw, and the flight controller with its filter, its laws and its link rings.
    pub fn snapshot(&self) -> BoardSnapshot<A, C, M> {
        BoardSnapshot {
            airframe: self.airframe.snapshot(),
            suite: self.suite,
            controller: self.controller.clone(),
            now: self.now,
            samples: self.samples,
            outages: self.outages,
            command: self.command,
        }
    }

    /// Returns to `snapshot`, which this board took. The snapshot stays whole, so a run
    /// returns to it as often as it likes.
    pub fn restore(&mut self, snapshot: &BoardSnapshot<A, C, M>) -> Result<(), ChainError> {
        self.airframe.restore(&snapshot.airframe)?;
        self.suite = snapshot.suite;
        self.controller.clone_from(&snapshot.controller);
        self.now = snapshot.now;
        self.samples = snapshot.samples;
        self.outages = snapshot.outages;
        self.command = snapshot.command;
        Ok(())
    }
}
