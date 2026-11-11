//! A flight controller's board on an airframe: the vehicle's sensor suite
//! turns truth into samples, the Aviate kernel steps on those samples, and
//! its actuator command flies the airframe. Truth reaches the kernel through
//! the suite and through nothing else.

#[cfg(test)]
mod tests;

use aviate_core::control::{ControlMode, VehicleController};
use aviate_core::kernel::DefaultAviateKernel;
use aviate_core::mixer::Mixer;
use aviate_hal_io::SystemCommand;
use sim_aviate::{AviateDirectConfig, AviateDirectRunner, DirectActuatorCmd};
use sim_core::{SimInstant, VehicleState};
use sim_devices::{DeviceSamples, SensorSuite, SensorSuiteSpec};
use sim_geo::{Geodetic, egm96_undulation_m};

use crate::{Airframe, ChainError};

/// Devices of the suite that are out of service: they deliver nothing to
/// any consumer, as a failed or switched-off device would.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DeviceOutages {
    /// The GNSS receiver.
    pub gnss: bool,
    /// The air-data unit: the static port and the pitot.
    pub air_data: bool,
}

/// An airframe, its sensor suite and the Aviate kernel that flies it.
pub struct HostedBoard<A, C, M>
where
    C: VehicleController,
    M: Mixer,
{
    airframe: A,
    suite: SensorSuite,
    /// On the heap: the board carries its rings and filter by value.
    controller: Box<AviateDirectRunner<C, M>>,
    origin: Geodetic,
    now: SimInstant,
    samples: DeviceSamples,
    outages: DeviceOutages,
    command: DirectActuatorCmd,
}

impl<A, C, M> HostedBoard<A, C, M>
where
    A: Airframe,
    C: VehicleController,
    M: Mixer,
{
    /// `kernel` on `airframe` with the devices of `suite`. `origin` is the
    /// origin of the flight controller's local frame and `decimal_year` the
    /// date it evaluates its geomagnetic reference for.
    pub fn new(
        airframe: A,
        suite: &SensorSuiteSpec,
        kernel: DefaultAviateKernel<C, M>,
        origin: Geodetic,
        decimal_year: f64,
    ) -> Result<Self, ChainError> {
        let config = AviateDirectConfig {
            decimal_year,
            ..AviateDirectConfig::ideal(origin)
        };
        let mut controller = AviateDirectRunner::new(kernel, config)
            .map_err(|source| ChainError::FlightController { at_s: 0.0, source })?;
        // The installation states the geoid's height at the origin, as a real
        // one would, so the controller reports heights above mean sea level.
        controller
            .board_mut()
            .set_geoid_separation(egm96_undulation_m(origin.lat_rad, origin.lon_rad));
        Ok(Self {
            airframe,
            suite: SensorSuite::new(suite).map_err(ChainError::Devices)?,
            controller: Box::new(controller),
            origin,
            now: SimInstant::START,
            samples: DeviceSamples::default(),
            outages: DeviceOutages::default(),
            command: DirectActuatorCmd::default(),
        })
    }

    /// One cycle: the devices sense, the kernel steps on their samples, and
    /// the airframe flies its command.
    pub fn step(&mut self) -> Result<(), ChainError> {
        let period = self.airframe.period();
        let truth = self.airframe.sensed()?;
        self.samples = self
            .suite
            .sense(&truth, period.as_seconds())
            .map_err(ChainError::Devices)?;
        if self.outages.gnss {
            self.samples.gnss = None;
        }
        if self.outages.air_data {
            self.samples.air_data = None;
        }
        let at_s = self.now.duration_since(SimInstant::START).as_seconds();
        self.command = self
            .controller
            .step_sensors(self.now, &self.samples)
            .map_err(|source| ChainError::FlightController { at_s, source })?;
        self.airframe.fly(&self.command)?;
        self.now = self.now.advanced(period);
        Ok(())
    }

    /// The instant the vehicle state describes.
    pub const fn now(&self) -> SimInstant {
        self.now
    }

    /// The origin of the flight controller's local frame.
    pub const fn origin(&self) -> Geodetic {
        self.origin
    }

    /// The true vehicle state, for scoring and drawing.
    pub fn true_state(&self) -> VehicleState {
        self.airframe.state()
    }

    /// The airframe.
    pub const fn airframe(&self) -> &A {
        &self.airframe
    }

    /// The sensor suite, to take the sky from its receiver or displace it.
    pub const fn suite_mut(&mut self) -> &mut SensorSuite {
        &mut self.suite
    }

    /// The flight controller.
    pub fn controller(&self) -> &AviateDirectRunner<C, M> {
        &self.controller
    }

    /// Hands the flight controller a typed command.
    pub fn send_command(&mut self, command: SystemCommand) {
        self.controller.send_command(command);
    }

    /// Pushes MAVLink bytes into the flight controller's receive ring and
    /// returns how many it took.
    pub fn uplink(&mut self, bytes: &[u8]) -> usize {
        self.controller.push_mavlink_rx(bytes)
    }

    /// Pops MAVLink bytes from the flight controller's transmit ring.
    pub fn downlink(&mut self, buffer: &mut [u8]) -> usize {
        self.controller.pop_mavlink_tx(buffer)
    }

    /// Takes devices out of service, or returns them, from the next cycle.
    pub const fn set_outages(&mut self, outages: DeviceOutages) {
        self.outages = outages;
    }

    /// The devices out of service.
    pub const fn outages(&self) -> DeviceOutages {
        self.outages
    }

    /// What the devices delivered in the last cycle.
    pub const fn last_samples(&self) -> &DeviceSamples {
        &self.samples
    }

    /// The actuator command of the last cycle.
    pub const fn last_command(&self) -> &DirectActuatorCmd {
        &self.command
    }

    /// The control mode the flight controller flew on its last cycle.
    pub fn flight_mode(&self) -> ControlMode {
        self.controller.board().last_effective_command().mode
    }

    /// The flight controller's estimate of the vehicle state, when it has a
    /// usable one.
    pub fn estimated_state(&self) -> Option<VehicleState> {
        self.controller.estimated_state()
    }
}
