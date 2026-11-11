//! The chain as one steppable object: Pilotage's mission engine commanding
//! the Aviate X500 at the engine's own frame rate.

#[cfg(test)]
mod tests;

use aviate_core::control::ControlMode;
use navigate_contract::{MonotonicNanos, NavigationSolution};
use pilotage_adapter_api::{StepBudget, VehicleAdapter};
use pilotage_mission::MissionState;
use sim_aviate::DirectNoiseTier;
use sim_core::{SimInstant, VehicleState};

use crate::{
    AviateVehicleConfig, AviateX500Vehicle, ChainError, FLIGHT_RATE_HZ, FrameReport, MissionHost,
    MissionSetup,
};

/// Rate of the mission engine's frames, hertz: its documented interval.
pub const MISSION_FRAME_HZ: u64 = 20;
const FRAME_TICKS: u32 = (FLIGHT_RATE_HZ / MISSION_FRAME_HZ) as u32;
const _: () = assert!(FLIGHT_RATE_HZ.is_multiple_of(MISSION_FRAME_HZ));
const NANOS_PER_SECOND: f64 = 1.0e9;

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

/// Pilotage's mission engine, the Aviate flight controller and the X500.
pub struct FlightChain {
    vehicle: AviateX500Vehicle,
    host: MissionHost,
    ticks_into_frame: u32,
}

impl FlightChain {
    /// The vehicle on its pad and the mission built, nothing commanded yet.
    pub fn new(setup: &ChainSetup) -> Result<Self, ChainError> {
        let vehicle = AviateX500Vehicle::new(AviateVehicleConfig {
            vehicle: setup.mission.vehicle,
            pad: setup.mission.anchor,
            heading_rad: setup.heading_rad,
            noise: setup.noise,
            noise_seed: setup.noise_seed,
        })?;
        let host = MissionHost::new(&setup.mission, &vehicle.capabilities())?;
        Ok(Self {
            vehicle,
            host,
            ticks_into_frame: 0,
        })
    }

    /// The instant the vehicle state describes.
    pub const fn now(&self) -> SimInstant {
        self.vehicle.now()
    }

    /// The true vehicle state, for scoring and drawing.
    pub fn true_state(&self) -> VehicleState {
        self.vehicle.true_state()
    }

    /// The vehicle.
    pub const fn vehicle(&self) -> &AviateX500Vehicle {
        &self.vehicle
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

    /// Silences the flight controller's satellite receiver from now on.
    pub fn silence_gnss(&mut self) {
        self.vehicle.silence_gnss();
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
        let report = (self.ticks_into_frame == 0).then(|| self.host.frame(&mut self.vehicle, now));
        if self.vehicle.step(StepBudget { ticks: 1 }).advanced == 0 {
            return Err(self.vehicle.take_fault().unwrap_or(ChainError::Stopped));
        }
        self.ticks_into_frame = self.ticks_into_frame.wrapping_add(1) % FRAME_TICKS;
        Ok(report)
    }

    /// One mission frame's worth of flight, and the report of the frame
    /// that fell in it.
    pub fn advance_frame(&mut self) -> Result<FrameReport, ChainError> {
        let mut report = None;
        for _ in 0..FRAME_TICKS {
            if let Some(frame) = self.advance_tick()? {
                report = Some(frame);
            }
        }
        report.ok_or(ChainError::Stopped)
    }
}
