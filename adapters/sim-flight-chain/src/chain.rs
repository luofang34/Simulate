//! The chain as one steppable object: Pilotage's mission engine commanding
//! a vehicle's Aviate flight controller at the engine's own frame rate.

#[cfg(test)]
mod tests;

use aviate_core::control::{ControlMode, VehicleController};
use aviate_core::mixer::Mixer;
use navigate_contract::{MonotonicNanos, NavigationSolution};
use pilotage_adapter_api::{StepBudget, VehicleAdapter};
use pilotage_mission::MissionState;
use sim_aviate::DirectNoiseTier;
use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};

use crate::{
    Airframe, AviateVehicleConfig, AviateX500Vehicle, ChainError, FLIGHT_RATE_HZ, FrameReport,
    HostedVehicle, MissionHost, MissionSetup,
};

/// Rate of the mission engine's frames, hertz: its documented interval.
pub const MISSION_FRAME_HZ: u64 = 20;
const FRAME_SPAN_TICKS: u64 = TICKS_PER_SECOND / MISSION_FRAME_HZ;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(MISSION_FRAME_HZ));
const NANOS_PER_SECOND: f64 = 1.0e9;

/// What the chain asks of the vehicle a mission flies, beyond Pilotage's
/// adapter API: its clock and, for scoring and drawing, its truth.
pub trait ChainVehicle: VehicleAdapter {
    /// The instant the vehicle state describes.
    fn now(&self) -> SimInstant;

    /// The flight controller's cycle.
    fn period(&self) -> TickSpan;

    /// The true vehicle state.
    fn true_state(&self) -> VehicleState;

    /// The control mode the flight controller flew on its last cycle.
    fn flight_mode(&self) -> ControlMode;

    /// Takes the reason stepping stopped, leaving the vehicle stopped.
    fn take_fault(&mut self) -> Option<ChainError>;
}

impl ChainVehicle for AviateX500Vehicle {
    fn now(&self) -> SimInstant {
        Self::now(self)
    }

    fn period(&self) -> TickSpan {
        TickSpan::from_ticks(TICKS_PER_SECOND / FLIGHT_RATE_HZ)
    }

    fn true_state(&self) -> VehicleState {
        Self::true_state(self)
    }

    fn flight_mode(&self) -> ControlMode {
        Self::flight_mode(self)
    }

    fn take_fault(&mut self) -> Option<ChainError> {
        Self::take_fault(self)
    }
}

impl<A, C, M> ChainVehicle for HostedVehicle<A, C, M>
where
    A: Airframe,
    C: VehicleController,
    M: Mixer,
{
    fn now(&self) -> SimInstant {
        Self::now(self)
    }

    fn period(&self) -> TickSpan {
        Self::period(self)
    }

    fn true_state(&self) -> VehicleState {
        Self::true_state(self)
    }

    fn flight_mode(&self) -> ControlMode {
        Self::flight_mode(self)
    }

    fn take_fault(&mut self) -> Option<ChainError> {
        Self::take_fault(self)
    }
}

/// A mission and the vehicle that flies it.
#[derive(Debug, Clone, PartialEq)]
pub struct ChainSetup {
    /// The route and how the mission flies it.
    pub mission: MissionSetup,
    /// Heading on the pad from true north, radians.
    pub heading_rad: f64,
    /// Noise of the sensors the flight controller synthesises.
    pub noise: DirectNoiseTier,
    /// Seed of that noise.
    pub noise_seed: u64,
}

/// Pilotage's mission engine, an Aviate flight controller and its vehicle.
pub struct FlightChain<V = AviateX500Vehicle> {
    vehicle: V,
    host: MissionHost,
    /// Simulation ticks flown since the last mission frame fell due.
    since_frame_ticks: u64,
}

impl FlightChain {
    /// The X500 on its pad and the mission built, nothing commanded yet.
    pub fn new(setup: &ChainSetup) -> Result<Self, ChainError> {
        let vehicle = AviateX500Vehicle::new(AviateVehicleConfig {
            vehicle: setup.mission.vehicle,
            pad: setup.mission.anchor,
            heading_rad: setup.heading_rad,
            noise: setup.noise,
            noise_seed: setup.noise_seed,
        })?;
        Self::over(vehicle, &setup.mission)
    }

    /// Silences the flight controller's satellite receiver from now on.
    pub fn silence_gnss(&mut self) {
        self.vehicle.silence_gnss();
    }
}

impl<V: ChainVehicle> FlightChain<V> {
    /// `mission` built for `vehicle`, nothing commanded yet.
    pub fn over(vehicle: V, mission: &MissionSetup) -> Result<Self, ChainError> {
        let host = MissionHost::new(mission, &vehicle.capabilities())?;
        Ok(Self {
            vehicle,
            host,
            since_frame_ticks: FRAME_SPAN_TICKS,
        })
    }

    /// The instant the vehicle state describes.
    pub fn now(&self) -> SimInstant {
        self.vehicle.now()
    }

    /// The true vehicle state, for scoring and drawing.
    pub fn true_state(&self) -> VehicleState {
        self.vehicle.true_state()
    }

    /// The vehicle.
    pub const fn vehicle(&self) -> &V {
        &self.vehicle
    }

    /// The vehicle, to reach its devices.
    pub const fn vehicle_mut(&mut self) -> &mut V {
        &mut self.vehicle
    }

    /// The mission host.
    pub const fn host(&self) -> &MissionHost {
        &self.host
    }

    /// The control mode the flight controller flew on its last cycle.
    pub fn flight_mode(&self) -> ControlMode {
        self.vehicle.flight_mode()
    }

    /// Hands a navigator's solution to the mission as its ownship; see
    /// [`MissionHost::offer_navigation`].
    pub fn offer_navigation(&mut self, solution: &NavigationSolution) {
        let now = self.mission_now();
        self.host.offer_navigation(solution, now);
    }

    /// The present instant on the mission's clock.
    fn mission_now(&self) -> MonotonicNanos {
        let elapsed_s = self.now().duration_since(SimInstant::START).as_seconds();
        MonotonicNanos::from_nanos((elapsed_s * NANOS_PER_SECOND).round() as u64)
    }

    /// The mission phase after the last frame.
    pub fn mission_state(&self) -> MissionState {
        self.host.engine().state()
    }

    /// One cycle of the flight controller and the airframe, after a mission
    /// frame when one is due. Returns that frame's report.
    pub fn advance_tick(&mut self) -> Result<Option<FrameReport>, ChainError> {
        let now = self.mission_now();
        // A flight controller whose cycle does not divide the frame interval
        // gets its frames on the cycle after each one falls due.
        let due = self.since_frame_ticks >= FRAME_SPAN_TICKS;
        let report = due.then(|| {
            self.since_frame_ticks -= FRAME_SPAN_TICKS;
            self.host.frame(&mut self.vehicle, now)
        });
        if self.vehicle.step(StepBudget { ticks: 1 }).advanced == 0 {
            return Err(self.vehicle.take_fault().unwrap_or(ChainError::Stopped));
        }
        self.since_frame_ticks = self
            .since_frame_ticks
            .saturating_add(self.vehicle.period().ticks());
        Ok(report)
    }

    /// One mission frame's worth of flight, and the report of the frame
    /// that fell in it.
    pub fn advance_frame(&mut self) -> Result<FrameReport, ChainError> {
        let mut report = None;
        loop {
            if let Some(frame) = self.advance_tick()? {
                report = Some(frame);
            }
            if self.since_frame_ticks >= FRAME_SPAN_TICKS {
                return report.ok_or(ChainError::Stopped);
            }
        }
    }
}
