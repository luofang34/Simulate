use sim_core::{Block, KinematicInput, SimInstant, VehicleState};

use super::SimulateVehicleAdapter;

/// Everything needed to put an adapter's vehicle back where it was: the
/// dynamics block's snapshot plus the adapter state the block does not hold.
/// The control law is deliberately absent, because a restored vehicle starts
/// from a neutral command rather than one issued in a timeline that no longer
/// exists.
#[derive(Debug, Clone)]
pub struct AdapterCheckpoint<S> {
    /// The dynamics block's own snapshot.
    pub block: S,
    /// Whether the vehicle was armed.
    pub armed: bool,
    /// The surface elevation ranging measured against, metres.
    pub surface_elevation_m: Option<f64>,
}

impl<B: Block<Input = KinematicInput, Output = VehicleState>> SimulateVehicleAdapter<B> {
    /// The vehicle's present state, to hand back to [`Self::restore_checkpoint`].
    pub fn checkpoint(&self) -> AdapterCheckpoint<B::Snapshot> {
        AdapterCheckpoint {
            block: self.block.snapshot(),
            armed: self.armed,
            surface_elevation_m: self.surface_elevation_m,
        }
    }

    /// Puts the vehicle back to `checkpoint` at simulated time `at`, with a
    /// neutral control law and any fault cleared. The avionics source
    /// restarts in a new epoch with its sequence reset, so no measurement
    /// from the abandoned timeline orders against what follows; the tick
    /// still advances so frame stamps stay monotonic.
    pub fn restore_checkpoint(
        &mut self,
        checkpoint: &AdapterCheckpoint<B::Snapshot>,
        at: SimInstant,
    ) {
        self.block.restore(&checkpoint.block);
        self.law.reset();
        self.armed = checkpoint.armed;
        self.surface_elevation_m = checkpoint.surface_elevation_m;
        self.fault = None;
        self.now = at;
        self.tick = self.tick.next();
        self.avionics.restart(self.config.seed);
        self.publish();
    }
}
