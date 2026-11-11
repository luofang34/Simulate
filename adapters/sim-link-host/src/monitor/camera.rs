//! The terrain camera as the monitor sees it: the latest frame the flight
//! delivered, what each visual set made of it, and where the camera was
//! for every frame a set used.
//!
//! The navigator reports no outcome per frame, only each set's running
//! tally. A set takes a frame at its readout, one navigation step after
//! delivery at the latest, and the next frame is a camera period later, so
//! whatever a set's tally gained since the latest delivery is that frame's
//! outcome.

use std::collections::BTreeMap;

#[cfg(test)]
mod tests;

/// One terrain-camera frame as delivered to the flight.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraShot {
    /// The exposure's index in the run, from 1.
    pub index: u64,
    /// Route time of the middle of the exposure, seconds.
    pub mid_s: f64,
    /// Width and height, pixels.
    pub size: (u32, u32),
    /// Grey levels, row-major, top row first.
    pub luma: Vec<u8>,
    /// TRUTH: the camera's latitude and longitude at mid-exposure, degrees.
    pub at_deg: (f64, f64),
}

/// Where the camera was when a set used one of its frames.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixMark {
    /// The set's index in the run's order.
    pub set: usize,
    /// TRUTH: the camera's latitude and longitude at mid-exposure, degrees.
    pub at_deg: (f64, f64),
    /// Whether the set's filter took the fix.
    pub accepted: bool,
}

/// The latest frame, each set's outcome of it, and the marks so far.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CameraLog {
    latest: Option<CameraShot>,
    outcomes: BTreeMap<String, Vec<String>>,
    marks: Vec<FixMark>,
}

impl CameraLog {
    /// A frame delivered: the outcomes of the one before it are done.
    pub(super) fn deliver(&mut self, shot: CameraShot) {
        self.latest = Some(shot);
        self.outcomes.clear();
    }

    /// `set`, at `index` in the run's order, went from tally `before` to
    /// `after`: each outcome key that grew is the latest frame's.
    pub(super) fn tallied(
        &mut self,
        (set, index): (&str, usize),
        before: Option<&BTreeMap<String, u64>>,
        after: &BTreeMap<String, u64>,
    ) {
        let Some(shot) = &self.latest else {
            return;
        };
        let at_deg = shot.at_deg;
        for (key, count) in after {
            if key.starts_with("context:") {
                continue;
            }
            let was = before
                .and_then(|tally| tally.get(key))
                .copied()
                .unwrap_or(0);
            for _ in was..*count {
                self.outcomes
                    .entry(set.to_owned())
                    .or_default()
                    .push(key.clone());
                if key == "accepted" || key.starts_with("rejected:") {
                    self.marks.push(FixMark {
                        set: index,
                        at_deg,
                        accepted: key == "accepted",
                    });
                }
            }
        }
    }

    /// The latest frame delivered.
    pub const fn latest(&self) -> Option<&CameraShot> {
        self.latest.as_ref()
    }

    /// What `set` made of the latest frame, as tally keys; `None` while it
    /// has not reported.
    pub fn outcomes(&self, set: &str) -> Option<&[String]> {
        self.outcomes.get(set).map(Vec::as_slice)
    }

    /// Where the camera was for each frame a set's filter judged, in order.
    pub fn marks(&self) -> &[FixMark] {
        &self.marks
    }
}

/// A tally key in plain words: `accepted`, `rejected by the filter (…)`, or
/// the navigator's reason for not using the frame.
pub fn outcome_words(key: &str) -> String {
    if let Some(reason) = key.strip_prefix("rejected:") {
        return format!("rejected by the filter ({reason})");
    }
    if let Some(rest) = key.strip_prefix("unused:") {
        return rest.split(':').next().unwrap_or(rest).to_owned();
    }
    key.to_owned()
}
