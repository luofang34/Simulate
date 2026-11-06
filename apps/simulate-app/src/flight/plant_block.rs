//! [`FreeFlightPlant`] as a [`sim_core::Block`], so free flight runs inside a
//! [`sim_core::SimSession`] with stamped inputs, fixed steps, and sensor exposures.

use std::cell::{Cell, RefCell};

use sim_core::{Block, BlockError, ControlInputs, SystemControls, TickSpan, VehicleState};
use tracing::error;

use crate::error::AppSimError;
use crate::vehicles::{FreeFlightPlant, PlantSnapshot};
use crate::world_surface::WorldSurfaceHandle;

const BLOCK_NAME: &str = "free-flight plant";

/// A free-flight plant stepped by the session.
///
/// The session hands its block out only by shared reference, so operator system switches
/// (flaps, magnetos, brakes) wait in a [`Cell`] and take effect at the start of the next
/// step, and a plant failure is kept in full in a [`RefCell`] so the flight can report the
/// typed [`AppSimError`] rather than the session's flattened [`BlockError`].
pub struct PlantBlock {
    plant: FreeFlightPlant,
    surface: WorldSurfaceHandle,
    pending_system: Cell<Option<SystemControls>>,
    fault: RefCell<Option<AppSimError>>,
}

impl PlantBlock {
    /// Wraps `plant`, flying it over `surface`'s ground.
    pub fn new(plant: FreeFlightPlant, surface: WorldSurfaceHandle) -> Self {
        Self {
            plant,
            surface,
            pending_system: Cell::new(None),
            fault: RefCell::new(None),
        }
    }

    /// The wrapped plant.
    pub const fn plant(&self) -> &FreeFlightPlant {
        &self.plant
    }

    /// System controls as the next step will fly them: a pending change, else the plant's.
    pub fn system_controls(&self) -> Option<SystemControls> {
        let current = self.plant.system_controls()?;
        Some(self.pending_system.get().unwrap_or(current))
    }

    /// Queues `controls` for the start of the next step (JSBSim-backed plants only).
    pub fn queue_system_controls(&self, controls: SystemControls) {
        if self.plant.system_controls().is_some() {
            self.pending_system.set(Some(controls));
        }
    }

    /// Hands over the typed failure behind the last refused step or restore, if any.
    pub fn take_fault(&self) -> Option<AppSimError> {
        self.fault.try_borrow_mut().ok().and_then(|mut f| f.take())
    }

    fn refuse(&mut self, err: AppSimError) -> BlockError {
        let detail = error_chain(&err);
        *self.fault.get_mut() = Some(err);
        BlockError::InputOutOfRange {
            block: BLOCK_NAME,
            input: "step",
            detail,
        }
    }
}

impl Block for PlantBlock {
    type Input = ControlInputs;
    type Output = VehicleState;
    type Snapshot = PlantSnapshot;

    fn period(&self) -> TickSpan {
        self.plant.period()
    }

    fn step(&mut self, input: &ControlInputs) -> Result<(), BlockError> {
        if let Some(detail) = self.fault.get_mut().as_ref().map(error_chain) {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK_NAME,
                input: "restore",
                detail,
            });
        }
        if let Some(system) = self.pending_system.take() {
            self.plant.set_system_controls(system);
        }
        match self.plant.step(*input, &self.surface) {
            Ok(()) => Ok(()),
            Err(err) => Err(self.refuse(err)),
        }
    }

    fn output(&self) -> VehicleState {
        self.plant.state()
    }

    fn snapshot(&self) -> PlantSnapshot {
        self.plant.snapshot()
    }

    fn restore(&mut self, snapshot: &PlantSnapshot) {
        self.pending_system.set(None);
        if let Err(err) = self.plant.restore(snapshot) {
            error!(error = %err, "free-flight plant refused a snapshot; it will not step");
            *self.fault.get_mut() = Some(err);
        }
    }
}

fn error_chain(err: &AppSimError) -> String {
    let mut text = err.to_string();
    let mut source = std::error::Error::source(err);
    while let Some(cause) = source {
        text.push_str(": ");
        text.push_str(&cause.to_string());
        source = cause.source();
    }
    text
}
