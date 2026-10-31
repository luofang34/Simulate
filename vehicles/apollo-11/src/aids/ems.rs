use glam::DVec3;
use sim_core::{Block, BlockError, TickSpan};

use crate::InertialSiteSample;
use crate::site::require_period;

const BLOCK: &str = "entry_monitor";

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;
/// Sensed deceleration at which the EMS starts its entry scroll, g.
const ENTRY_THRESHOLD_G: f64 = 0.05;
/// Time after the threshold at which the corridor is judged.
const CORRIDOR_CHECK_DELAY: TickSpan = TickSpan::from_seconds_and_subsec_ticks(10, 0);
/// Deceleration at the corridor check that marks a steep entry, g.
const CORRIDOR_CHECK_G: f64 = 0.262;

/// Which way the corridor check says to hold the lift vector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CorridorIndication {
    /// The check has not happened yet.
    Pending,
    /// The entry is steep: fly lift up.
    LiftUp,
    /// The entry is shallow: fly lift down so the capsule does not skip out.
    LiftDown,
}

/// Setup of an [`EntryMonitorBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EntryMonitorConfig {
    /// Sample period.
    pub period: TickSpan,
    /// Accelerometer sensing axis in site axes, unit; deceleration reads
    /// positive when specific force lies against it.
    pub sensing_axis_site: DVec3,
    /// Velocity loaded in the `ΔV` counter before entry, m/s.
    pub initial_delta_v_m_s: f64,
}

/// What an [`EntryMonitorBlock`] publishes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EntryMonitorOutput {
    /// Sensed deceleration, g.
    pub g_load: f64,
    /// Peak sensed deceleration, g.
    pub peak_g_load: f64,
    /// The `ΔV` counter, m/s, wound down by sensed deceleration.
    pub delta_v_counter_m_s: f64,
    /// Latched: the deceleration has reached 0.05 g.
    pub threshold_reached: bool,
    /// The corridor check's verdict.
    pub corridor: CorridorIndication,
}

/// Complete state of an [`EntryMonitorBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EntryMonitorSnapshot {
    output: EntryMonitorOutput,
    steps_since_threshold: u64,
}

/// The CM's Entry Monitor System: an independent axial accelerometer that lets
/// the crew check the guidance computer through entry.
///
/// Nothing happens until the 0.05 g threshold latches. Ten seconds later the
/// deceleration is compared with the corridor check level to pick the lift
/// direction, and the `ΔV` counter winds down all the way from the start.
#[derive(Debug, Clone, PartialEq)]
pub struct EntryMonitorBlock {
    config: EntryMonitorConfig,
    snapshot: EntryMonitorSnapshot,
}

impl EntryMonitorBlock {
    /// A monitor before entry. A zero period, or a sensing axis that is not a
    /// unit vector, is an error.
    pub fn new(config: EntryMonitorConfig) -> Result<Self, BlockError> {
        require_period(BLOCK, config.period)?;
        if !config.sensing_axis_site.is_normalized() {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "sensing_axis_site",
                detail: format!("{} is not a unit vector", config.sensing_axis_site),
            });
        }
        Ok(Self {
            snapshot: EntryMonitorSnapshot {
                output: EntryMonitorOutput {
                    g_load: 0.0,
                    peak_g_load: 0.0,
                    delta_v_counter_m_s: config.initial_delta_v_m_s,
                    threshold_reached: false,
                    corridor: CorridorIndication::Pending,
                },
                steps_since_threshold: 0,
            },
            config,
        })
    }
}

impl Block for EntryMonitorBlock {
    type Input = InertialSiteSample;
    type Output = EntryMonitorOutput;
    type Snapshot = EntryMonitorSnapshot;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &InertialSiteSample) -> Result<(), BlockError> {
        input.validate(BLOCK)?;
        let dt_s = self.config.period.as_seconds();
        let deceleration = -input
            .specific_force_site_m_s2
            .dot(self.config.sensing_axis_site);
        let g_load = deceleration / STANDARD_GRAVITY_M_S2;
        let snapshot = &mut self.snapshot;
        let output = &mut snapshot.output;
        output.g_load = g_load;
        output.peak_g_load = output.peak_g_load.max(g_load);
        output.delta_v_counter_m_s -= deceleration * dt_s;
        if output.threshold_reached {
            snapshot.steps_since_threshold = snapshot.steps_since_threshold.wrapping_add(1);
        }
        output.threshold_reached |= g_load >= ENTRY_THRESHOLD_G;
        let elapsed_ticks =
            u128::from(snapshot.steps_since_threshold) * self.config.period.ticks_u128();
        if output.corridor == CorridorIndication::Pending
            && elapsed_ticks >= CORRIDOR_CHECK_DELAY.ticks_u128()
        {
            output.corridor = if g_load >= CORRIDOR_CHECK_G {
                CorridorIndication::LiftUp
            } else {
                CorridorIndication::LiftDown
            };
        }
        Ok(())
    }

    fn output(&self) -> EntryMonitorOutput {
        self.snapshot.output
    }

    fn snapshot(&self) -> EntryMonitorSnapshot {
        self.snapshot
    }

    fn restore(&mut self, snapshot: &EntryMonitorSnapshot) {
        self.snapshot = *snapshot;
    }
}
