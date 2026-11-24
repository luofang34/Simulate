//! The whole chain at one instant: Pilotage's host core, the link between it and the
//! flight controller, and the board. A run that returns to a snapshot flies on from it
//! byte for byte as it did the first time, which is what rewinding a flight needs.

use aviate_core::control::VehicleController;
use aviate_core::mixer::Mixer;
use navigate_contract::MonotonicNanos;
use pilotage_host_core::{HostCore, HostEvent};
use pilotage_mission::{WaypointPlan, WaypointPlanRecord};
use sim_core::SimInstant;

use super::{HostChain, LinkCounters};
use crate::hosted_board::BoardSnapshot;
use crate::{Airframe, ChainError};

/// A [`HostChain`] at one instant.
pub struct ChainSnapshot<A, C, M>
where
    A: Airframe,
    C: VehicleController,
    M: Mixer,
{
    board: BoardSnapshot<A, C, M>,
    core: Box<HostCore>,
    record: Option<WaypointPlanRecord>,
    plan: Option<WaypointPlan>,
    load_at: MonotonicNanos,
    next_wake: MonotonicNanos,
    transmit: Vec<u8>,
    transmit_len: usize,
    sent: Vec<Vec<u8>>,
    events: Vec<(SimInstant, HostEvent)>,
    counters: LinkCounters,
}

impl<A, C, M> HostChain<A, C, M>
where
    A: Airframe,
    C: VehicleController + Clone,
    M: Mixer + Clone,
{
    /// The chain as it stands. The host core refuses a copy only when it talks to a
    /// flight controller outside the process, which a chain never does.
    pub fn snapshot(&self) -> Result<ChainSnapshot<A, C, M>, ChainError> {
        // Every field is named so that one added to the chain must be placed here.
        let Self {
            board,
            core,
            record,
            plan,
            load_at,
            next_wake,
            transmit,
            transmit_len,
            sent,
            events,
            counters,
        } = self;
        Ok(ChainSnapshot {
            board: board.snapshot(),
            core: Box::new(copy_of(core)?),
            record: record.clone(),
            plan: plan.clone(),
            load_at: *load_at,
            next_wake: *next_wake,
            transmit: transmit.clone(),
            transmit_len: *transmit_len,
            sent: sent.clone(),
            events: events.clone(),
            counters: *counters,
        })
    }

    /// Returns to `snapshot`, which this chain took. The snapshot stays whole, so a run
    /// returns to it as often as it likes.
    pub fn restore(&mut self, snapshot: &ChainSnapshot<A, C, M>) -> Result<(), ChainError> {
        // The copy that can fail is made before anything of the chain changes.
        let core = copy_of(&snapshot.core)?;
        self.board.restore(&snapshot.board)?;
        *self.core = core;
        self.record.clone_from(&snapshot.record);
        self.plan.clone_from(&snapshot.plan);
        self.load_at = snapshot.load_at;
        self.next_wake = snapshot.next_wake;
        self.transmit.clone_from(&snapshot.transmit);
        self.transmit_len = snapshot.transmit_len;
        self.sent.clone_from(&snapshot.sent);
        self.events.clone_from(&snapshot.events);
        self.counters = snapshot.counters;
        Ok(())
    }
}

fn copy_of(core: &HostCore) -> Result<HostCore, ChainError> {
    core.try_clone()
        .map_err(|source| ChainError::Host(Box::new(source)))
}
