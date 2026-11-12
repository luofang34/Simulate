//! The terrain camera's part of a replay: each frame handed to the
//! navigator, and what it did with them.

use std::collections::BTreeMap;

use navigate_contract::MonotonicNanos;
use navigate_fusion::RejectionReason;
use navigate_onboard::{Outcome, SensorData};

use super::tally::Reading;
use super::{SetRun, verdicts};
use crate::NavMeans;
use crate::trial::TrialError;
use crate::trial::availability::SensorKind;
use crate::trial::onboard::TERRAIN_CAMERA_SOURCE;
use crate::trial::record::SensorLog;
use crate::trial::route::step_seconds;
use crate::trial::terrain_report::{MapLoadReport, TerrainSetReport, WorkingCameraReport};

impl SetRun {
    /// The terrain camera's frame read out at `step`, handed over unless a
    /// schedule covers any instant of its exposure middle to readout; each
    /// of the navigator's records is counted by outcome, reason and context.
    pub(super) fn terrain_at(
        &mut self,
        log: &SensorLog,
        step: usize,
        now: MonotonicNanos,
    ) -> Result<(), TrialError> {
        while let Some(frame) = log
            .terrain
            .get(self.next_terrain)
            .filter(|f| f.end_step <= step)
        {
            self.next_terrain = self.next_terrain.wrapping_add(1);
            if !self.selection.means.contains(NavMeans::Visual) {
                continue;
            }
            let window = (step_seconds(frame.mid_step), step_seconds(frame.end_step));
            if self
                .selection
                .availability
                .withheld(SensorKind::TerrainCamera, window.0, window.1)
            {
                self.ledger
                    .record(SensorKind::TerrainCamera, window, &Reading::Withheld);
                continue;
            }
            let mid = self.clock.to_monotonic(frame.mid)?;
            let (width, height) = frame.size;
            let data = SensorData::TerrainFrame {
                width,
                height,
                luma: frame.pixels.clone(),
            };
            let started = web_time::Instant::now();
            self.ingest(TERRAIN_CAMERA_SOURCE, (mid, now), data)?;
            let took = started.elapsed().as_secs_f64();
            self.terrain_wall = (
                self.terrain_wall.0 + took,
                self.terrain_wall.1.wrapping_add(1),
                self.terrain_wall.2.max(took),
            );
            for record in &self.records {
                let outcome = outcome_key(&record.outcome, record.context.is_some());
                let context = record.context.unwrap_or("none");
                for key in [outcome, format!("context:{context}")] {
                    let count = self.terrain_fixes.entry(key).or_default();
                    *count = count.wrapping_add(1);
                }
            }
            let verdicts = verdicts(&self.records);
            self.ledger.record(
                SensorKind::TerrainCamera,
                window,
                &Reading::Offered(&verdicts),
            );
        }
        Ok(())
    }

    /// The terrain camera's fixes so far, by outcome (`accepted`,
    /// `rejected:<filter reason>`, `unused:<navigator reason>`) and by
    /// context (`context:keyframe`, `context:render`).
    pub fn terrain_fixes(&self) -> &BTreeMap<String, u64> {
        &self.terrain_fixes
    }

    /// Wall-clock seconds the navigator spent on terrain frames, the
    /// frames handed over, and the slowest frame's seconds. Not
    /// deterministic: for logs, never a result.
    pub const fn terrain_wall(&self) -> (f64, u64, f64) {
        self.terrain_wall
    }

    /// The record of the set's latest terrain match, when its navigator
    /// keeps one ([`super::super::replay_set_inspecting`]) and has matched a
    /// frame since.
    pub fn last_terrain_match(&self) -> Option<&navigate_onboard::TerrainMatch> {
        self.navigator
            .as_any()
            .downcast_ref::<navigate_onboard::ReferenceNavigator>()?
            .last_terrain_match(TERRAIN_CAMERA_SOURCE)
    }

    /// The set's terrain-camera record, when it takes visual fixes: its
    /// fixes and, from the reference navigator, the map it loaded and the
    /// camera it matched at.
    pub fn terrain_report(&self) -> Option<TerrainSetReport> {
        if !self.selection.means.contains(NavMeans::Visual) {
            return None;
        }
        let reference = self
            .navigator
            .as_any()
            .downcast_ref::<navigate_onboard::ReferenceNavigator>();
        Some(TerrainSetReport {
            fixes: self.terrain_fixes.clone(),
            map: reference
                .and_then(|n| n.terrain_map())
                .map(MapLoadReport::of),
            working_camera: reference
                .and_then(|n| n.terrain_working_camera(TERRAIN_CAMERA_SOURCE))
                .map(|camera| WorkingCameraReport::of(&camera)),
        })
    }
}

/// A terrain frame's outcome as the tally keys it: `accepted`,
/// `rejected:<filter reason>`, or `unused:<navigator reason>:matched` or
/// `:unmatched`, by whether a reference view was `matched` against.
pub fn outcome_key(outcome: &Outcome, matched: bool) -> String {
    match outcome {
        Outcome::Accepted => "accepted".to_owned(),
        Outcome::Rejected(reason) => format!("rejected:{}", reason_name(reason)),
        Outcome::Unused(reason) => format!(
            "unused:{reason}:{}",
            if matched { "matched" } else { "unmatched" }
        ),
    }
}

/// A rejection reason's variant name, without its values.
fn reason_name(reason: &RejectionReason) -> String {
    let text = format!("{reason:?}");
    text.split([' ', '{', '('])
        .next()
        .unwrap_or_default()
        .to_owned()
}
