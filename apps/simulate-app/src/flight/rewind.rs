//! Deterministic rewind: a ring of checkpoints taken every [`CHECKPOINT_INTERVAL_S`] of
//! simulated time, covering about [`MAX_CHECKPOINTS`] × [`CHECKPOINT_INTERVAL_S`] seconds.
//!
//! Scrubbing is non-destructive: while the hub holds the simulation, the flight can move
//! back and forth along the history and return to the live frontier exactly, because the
//! live state is pinned as the newest checkpoint before the first scrub. Only flying on
//! from a scrubbed point forgets the checkpoints after it.

use std::collections::VecDeque;

use sim_apollo::CockpitInstrumentPanel;
use sim_core::{ControlInputs, SimInstant, TimelineCheckpoint};
use sim_scenario::{ScenarioError, ScenarioSnapshot};
use sim_universe::UniverseCheckpoint;

use tracing::info;

use super::Flight;
use super::mode::Mode;
use crate::error::AppSimError;
use crate::sensor_bus::OnboardSensorBus;
use crate::vehicles::PlantSnapshot;

/// Simulated seconds between checkpoints.
pub const CHECKPOINT_INTERVAL_S: f64 = 0.5;
/// Checkpoints kept (`66 s` of history at [`CHECKPOINT_INTERVAL_S`]).
pub const MAX_CHECKPOINTS: usize = 132;

/// Mode-specific state captured at one instant.
#[derive(Debug, Clone)]
enum CheckpointState {
    Free {
        plant: Box<TimelineCheckpoint<PlantSnapshot>>,
        latched: ControlInputs,
        universe: Box<UniverseCheckpoint>,
        panel: Box<CockpitInstrumentPanel>,
    },
    Scenario(Box<ScenarioSnapshot>),
}

/// The flight as it was at `at`.
#[derive(Debug, Clone)]
pub(super) struct FlightCheckpoint {
    at: SimInstant,
    state: CheckpointState,
    sensors: OnboardSensorBus,
}

impl FlightCheckpoint {
    /// Captures `mode` and its onboard `sensors` at the mode's current instant, or
    /// `None` while the free-flight ownship has no session to checkpoint.
    pub(super) fn capture(mode: &Mode, sensors: &OnboardSensorBus) -> Option<Self> {
        match mode {
            Mode::Free { world, panel, .. } => {
                let session = world.session()?;
                Some(Self {
                    at: session.physics_time(),
                    state: CheckpointState::Free {
                        plant: Box::new(session.checkpoint()),
                        latched: *session.latched_input(),
                        universe: Box::new(world.checkpoint()),
                        panel: panel.clone(),
                    },
                    sensors: *sensors,
                })
            }
            Mode::Scenario(run) => Some(Self {
                at: run.now(),
                state: CheckpointState::Scenario(Box::new(run.snapshot())),
                sensors: *sensors,
            }),
            // A spaceflight keeps no rewind history.
            Mode::Space(_) => None,
        }
    }

    /// The onboard sensor bus as it was at this checkpoint.
    pub(super) const fn sensors(&self) -> OnboardSensorBus {
        self.sensors
    }

    /// Puts `mode` back to this checkpoint, starting a new session timeline (sensors
    /// re-phase, the session's source epoch advances); a checkpoint of another kind of
    /// flight is refused.
    pub(super) fn restore_into(&self, mode: &mut Mode) -> Result<(), AppSimError> {
        match (mode, &self.state) {
            (
                Mode::Free { world, panel, .. },
                CheckpointState::Free {
                    plant,
                    latched,
                    universe,
                    panel: saved_panel,
                },
            ) => {
                world.restore(universe, |session| {
                    session.rewind_timeline(plant, *latched);
                    session.block().take_fault().map_or(Ok(()), Err)
                })?;
                panel.clone_from(saved_panel);
                Ok(())
            }
            (Mode::Scenario(run), CheckpointState::Scenario(saved)) => Ok(run.restore(saved)?),
            _ => Err(ScenarioError::SnapshotMismatch {
                detail: "the checkpoint was taken from another kind of flight",
            }
            .into()),
        }
    }
}

/// Seconds of `at` since the simulation start.
pub(super) fn seconds(at: SimInstant) -> f64 {
    at.duration_since(SimInstant::START).as_seconds()
}

/// Ring buffer of the most recent checkpoints, oldest first, and the scrub cursor.
#[derive(Debug, Clone, Default)]
pub(super) struct RewindHistory {
    entries: VecDeque<FlightCheckpoint>,
    /// `None` at the live frontier; `Some(i)` while the flight shows `entries[i]`.
    cursor: Option<usize>,
}

impl RewindHistory {
    /// Forgets every checkpoint (a new flight began).
    pub(super) fn clear(&mut self) {
        self.entries.clear();
        self.cursor = None;
    }

    /// Whether a checkpoint is due at `now`.
    pub(super) fn due(&self, now: SimInstant) -> bool {
        self.entries
            .back()
            .is_none_or(|last| seconds(now) - seconds(last.at) >= CHECKPOINT_INTERVAL_S - 1e-9)
    }

    /// Appends `checkpoint`, dropping the oldest beyond [`MAX_CHECKPOINTS`]. A checkpoint
    /// taken while scrubbed back means the flight flew on from there, so the branch is
    /// committed first.
    pub(super) fn push(&mut self, checkpoint: FlightCheckpoint) {
        self.commit_live_branch();
        self.entries.push_back(checkpoint);
        while self.entries.len() > MAX_CHECKPOINTS {
            self.entries.pop_front();
        }
    }

    /// Whether the flight is at the live frontier rather than scrubbed back.
    pub(super) const fn is_live(&self) -> bool {
        self.cursor.is_none()
    }

    /// Keeps `live`, the flight's current state, as the newest checkpoint so scrubbing can
    /// return to it exactly; ignored when scrubbed or when no newer than the newest.
    pub(super) fn pin_live_head(&mut self, live: FlightCheckpoint) {
        let newer = self.entries.back().is_none_or(|last| last.at < live.at);
        if self.is_live() && newer {
            self.push(live);
        }
    }

    /// Moves the playhead `delta_s` simulated seconds from `now_s` (the shown instant) to
    /// the nearest checkpoint; returns it when the playhead moved.
    pub(super) fn scrub_by_seconds(
        &mut self,
        delta_s: f64,
        now_s: f64,
    ) -> Option<&FlightCheckpoint> {
        let target_s = now_s + if delta_s.is_finite() { delta_s } else { 0.0 };
        let index = self.nearest(target_s)?;
        self.scrub_to_index(index)
    }

    /// Moves the playhead to `fraction` (`0` oldest, `1` newest) of the buffered history.
    pub(super) fn scrub_to_fraction(&mut self, fraction: f64) -> Option<&FlightCheckpoint> {
        let (first, last) = (self.entries.front()?, self.entries.back()?);
        let (first_s, last_s) = (seconds(first.at), seconds(last.at));
        let fraction = if fraction.is_finite() {
            fraction.clamp(0.0, 1.0)
        } else {
            1.0
        };
        let index = self.nearest(first_s + fraction * (last_s - first_s))?;
        self.scrub_to_index(index)
    }

    /// Returns the playhead to the live frontier; the pinned live checkpoint when it moved.
    pub(super) fn scrub_to_live(&mut self) -> Option<&FlightCheckpoint> {
        self.cursor.take()?;
        self.entries.back()
    }

    /// Forgets every checkpoint after the scrubbed one, so the flight branches from it.
    pub(super) fn commit_live_branch(&mut self) {
        if let Some(index) = self.cursor.take() {
            self.entries.truncate(index.saturating_add(1));
        }
    }

    /// Simulated seconds from the oldest checkpoint to the frontier (`now` when live).
    pub(super) fn history_span_s(&self, now: SimInstant) -> f64 {
        let (Some(first), Some(last)) = (self.entries.front(), self.entries.back()) else {
            return 0.0;
        };
        let frontier_s = if self.is_live() {
            seconds(now).max(seconds(last.at))
        } else {
            seconds(last.at)
        };
        (frontier_s - seconds(first.at)).max(0.0)
    }

    /// Simulated seconds the playhead is behind the frontier (`0` when live).
    pub(super) fn rewind_offset_s(&self) -> f64 {
        let shown = self.cursor.and_then(|i| self.entries.get(i));
        match (shown, self.entries.back()) {
            (Some(shown), Some(last)) => (seconds(last.at) - seconds(shown.at)).max(0.0),
            _ => 0.0,
        }
    }

    /// Index of the checkpoint closest to `target_s`.
    fn nearest(&self, target_s: f64) -> Option<usize> {
        self.entries
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                let da = (seconds(a.at) - target_s).abs();
                let db = (seconds(b.at) - target_s).abs();
                da.total_cmp(&db)
            })
            .map(|(index, _)| index)
    }

    /// Shows `entries[index]`; the newest entry is the live frontier. Returns the
    /// checkpoint to restore, or `None` when the playhead did not move.
    fn scrub_to_index(&mut self, index: usize) -> Option<&FlightCheckpoint> {
        let last = self.entries.len().checked_sub(1)?;
        let current = self.cursor.unwrap_or(last);
        if index == current {
            return None;
        }
        self.cursor = (index < last).then_some(index);
        self.entries.get(index)
    }

    /// Checkpoints held.
    #[cfg(test)]
    pub(super) fn len(&self) -> usize {
        self.entries.len()
    }
}

impl Flight {
    /// Scrubs back by about `seconds_back` (to the nearest checkpoint) and returns the
    /// simulated seconds actually rewound; newer history is kept until the flight flies on.
    pub fn rewind_by_seconds(&mut self, seconds_back: f64) -> Result<f64, AppSimError> {
        let before_s = seconds(self.now());
        if !self.scrub_by_seconds(-seconds_back.max(0.0))? {
            return Err(AppSimError::NoCheckpoint {
                requested_s: seconds_back,
            });
        }
        let rewound_s = before_s - seconds(self.now());
        info!(requested_s = seconds_back, rewound_s, "rewound flight");
        Ok(rewound_s)
    }

    /// Moves the flight `delta_s` simulated seconds along its history (negative is back)
    /// to the nearest checkpoint; returns whether it moved.
    pub fn scrub_by_seconds(&mut self, delta_s: f64) -> Result<bool, AppSimError> {
        if delta_s < 0.0 {
            self.pin_live_head();
        }
        let now_s = seconds(self.now());
        let shown = self.history.scrub_by_seconds(delta_s, now_s);
        let moved = restore(&mut self.mode, &mut self.sensors, shown)?;
        self.after_restore(moved)
    }

    /// Moves the flight to `fraction` (`0` oldest, `1` newest) of its history; returns
    /// whether it moved.
    pub fn scrub_to_fraction(&mut self, fraction: f64) -> Result<bool, AppSimError> {
        self.pin_live_head();
        let shown = self.history.scrub_to_fraction(fraction);
        let moved = restore(&mut self.mode, &mut self.sensors, shown)?;
        self.after_restore(moved)
    }

    /// Returns a scrubbed flight to exactly the state it left the live frontier in;
    /// returns whether it moved.
    pub fn scrub_to_live(&mut self) -> Result<bool, AppSimError> {
        let shown = self.history.scrub_to_live();
        let moved = restore(&mut self.mode, &mut self.sensors, shown)?;
        self.after_restore(moved)
    }

    /// Simulated seconds of rewind history, oldest checkpoint to the live frontier.
    #[must_use]
    pub fn history_span_s(&self) -> f64 {
        self.history.history_span_s(self.now())
    }

    /// Simulated seconds the shown flight is behind the live frontier (`0` when live).
    #[must_use]
    pub fn rewind_offset_s(&self) -> f64 {
        self.history.rewind_offset_s()
    }

    /// Flying on from a scrubbed point: the history after it is forgotten.
    pub(super) fn commit_live_branch(&mut self) {
        if !self.history.is_live() {
            self.history.commit_live_branch();
            info!(
                at_s = seconds(self.now()),
                "flying on from the rewound point"
            );
        }
    }

    fn pin_live_head(&mut self) {
        if self.history.is_live()
            && let Some(live) = FlightCheckpoint::capture(&self.mode, &self.sensors)
        {
            self.history.pin_live_head(live);
        }
    }

    fn after_restore(&mut self, moved: bool) -> Result<bool, AppSimError> {
        if moved {
            self.after_discontinuity();
        }
        Ok(moved)
    }
}

/// Puts `mode` and `sensors` back to `checkpoint`; returns whether there was one.
fn restore(
    mode: &mut Mode,
    sensors: &mut OnboardSensorBus,
    checkpoint: Option<&FlightCheckpoint>,
) -> Result<bool, AppSimError> {
    let Some(checkpoint) = checkpoint else {
        return Ok(false);
    };
    checkpoint.restore_into(mode)?;
    *sensors = checkpoint.sensors();
    Ok(true)
}
