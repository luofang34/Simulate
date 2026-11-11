//! A recorded progress stream flown again: each step hands the monitor the
//! records of the next instant, so a recorded run shows the way a live one
//! does.

use std::convert::Infallible;

use sim_navigate::trial::ProgressRecord;

use super::{FlightStepper, MonitorFeed, StepState};
use crate::HostError;

/// A recorded run, stepped one recorded instant at a time.
#[derive(Debug, Clone, PartialEq)]
pub struct Replay {
    records: Vec<ProgressRecord>,
    next: usize,
    sim_time_s: f64,
}

/// The route time of a record; the run's start and end have none.
const fn record_time(record: &ProgressRecord) -> Option<f64> {
    match record {
        ProgressRecord::Sample(sample) => Some(sample.t_s),
        ProgressRecord::Event(event) => Some(event.t_s),
        ProgressRecord::Started(_) | ProgressRecord::Finished(_) => None,
    }
}

impl Replay {
    /// The run recorded in `text`, one progress record a line; blank lines
    /// are skipped.
    ///
    /// # Errors
    ///
    /// [`HostError::Replay`] for a line that is not a progress record.
    pub fn from_lines(text: &str) -> Result<Self, HostError> {
        let records = text
            .lines()
            .enumerate()
            .filter(|(_, line)| !line.trim().is_empty())
            .map(|(index, line)| {
                ProgressRecord::from_json_line(line).map_err(|source| HostError::Replay {
                    line: index.saturating_add(1),
                    source,
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            records,
            next: 0,
            sim_time_s: 0.0,
        })
    }

    /// Records in the run.
    pub fn len(&self) -> usize {
        self.records.len()
    }

    /// Whether the run holds no record.
    pub fn is_empty(&self) -> bool {
        self.records.is_empty()
    }
}

impl FlightStepper for Replay {
    type Error = Infallible;

    fn sim_time_s(&self) -> f64 {
        self.sim_time_s
    }

    /// Hands over the records up to and including the next recorded
    /// instant, with the start before it and the end after the last.
    fn step(&mut self, out: &mut Vec<MonitorFeed>) -> Result<StepState, Infallible> {
        let mut instant = None;
        while let Some(record) = self.records.get(self.next) {
            match (record_time(record), instant) {
                (Some(t_s), Some(at)) if t_s > at => break,
                (Some(t_s), _) => instant = Some(t_s),
                (None, _) => {}
            }
            out.push(MonitorFeed::Progress(record.clone()));
            self.next = self.next.saturating_add(1);
        }
        if let Some(t_s) = instant {
            self.sim_time_s = self.sim_time_s.max(t_s);
        }
        if self.next >= self.records.len() {
            return Ok(StepState::Finished);
        }
        Ok(StepState::Running)
    }
}
