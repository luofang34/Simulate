use glam::DVec3;
use sim_core::{
    Block, BlockError, CrewObservation, CrewPortError, CrewStationPort, SimInstant, TickSpan,
};

use crate::SurfacePlane;
use crate::site::require_period;

const BLOCK: &str = "lunar_contact_probe";

/// Lamp that [`LunarContactProbeBlock::publish_lamp`] drives.
pub const CONTACT_LIGHT_LAMP: u16 = 0o102;

/// What a [`LunarContactProbeBlock`] reads each step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LunarContactInput {
    /// Probe tips in the central body's frame, metres.
    pub tips_fixed_m: [DVec3; 3],
    /// The terrain beneath the vehicle.
    pub surface: SurfacePlane,
}

/// What a [`LunarContactProbeBlock`] publishes, and all of its state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LunarContactOutput {
    /// Latched: a probe has touched the surface, lighting the blue `CONTACT
    /// LIGHT` that calls for engine cutoff.
    pub lunar_contact: bool,
}

/// The contact probes trailing below three of the LM's footpads.
///
/// A probe tip at or below the surface latches the light, which then stays lit
/// whatever the vehicle does next.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LunarContactProbeBlock {
    period: TickSpan,
    output: LunarContactOutput,
}

impl LunarContactProbeBlock {
    /// Probes clear of the surface. A zero period is an error.
    pub fn new(period: TickSpan) -> Result<Self, BlockError> {
        require_period(BLOCK, period)?;
        Ok(Self {
            period,
            output: LunarContactOutput::default(),
        })
    }

    /// Publishes the `CONTACT LIGHT` lamp at `at`.
    pub fn publish_lamp(
        &self,
        at: SimInstant,
        port: &mut CrewStationPort,
    ) -> Result<(), CrewPortError> {
        port.publish(
            at,
            CrewObservation::AnnunciatorLamp {
                lamp_id: CONTACT_LIGHT_LAMP,
                lit: self.output.lunar_contact,
            },
        )
    }
}

impl Block for LunarContactProbeBlock {
    type Input = LunarContactInput;
    type Output = LunarContactOutput;
    type Snapshot = LunarContactOutput;

    fn period(&self) -> TickSpan {
        self.period
    }

    fn step(&mut self, input: &LunarContactInput) -> Result<(), BlockError> {
        input.surface.validate(BLOCK)?;
        if !input.tips_fixed_m.iter().all(|tip| tip.is_finite()) {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "tips_fixed_m",
                detail: format!("{:?} is not finite", input.tips_fixed_m),
            });
        }
        let touched = input
            .tips_fixed_m
            .iter()
            .any(|tip| input.surface.clearance_m(*tip) <= 0.0);
        self.output.lunar_contact |= touched;
        Ok(())
    }

    fn output(&self) -> LunarContactOutput {
        self.output
    }

    fn snapshot(&self) -> LunarContactOutput {
        self.output
    }

    fn restore(&mut self, snapshot: &LunarContactOutput) {
        self.output = *snapshot;
    }
}
