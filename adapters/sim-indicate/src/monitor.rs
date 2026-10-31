use glam::DQuat;
use indicate_alerts::{AlertContext, AlertEvent, AlertManager, AlertOutput, AlertProfile};
use indicate_instrument_state::{
    AircraftState, AirframeDisplayProfile, AttitudeMeasure, Candidate, FrameTag, FreshnessPolicy,
    HeadingMeasure, HeadingReference, IntegrityLevel, PanelData, SourceEpoch, SourceId,
    SourceInputs, SourceMonitorReport, SourceMonitors, SourcePolicies, SourceStep,
    UnusualAttitudeState, resolve_with_sources,
};
use sim_apollo::CockpitInstrumentSnapshot;
use sim_core::{SourceValidity, StampedReading};

use crate::frames::{indicate_quat, true_heading_rad};
use crate::{FeedError, FlightSample, LocalLevel, StateFeed, instant_ms};

#[cfg(test)]
mod tests;

/// The primary attitude source: first in
/// [`SourcePolicies::simulator`]'s priority.
pub const PGNS_SOURCE: SourceId = SourceId(1);
/// The backup attitude source: second in that priority.
pub const AGS_SOURCE: SourceId = SourceId(2);
/// The frame both sources are compared in: the local level, whichever body the
/// vehicle is over.
pub const LOCAL_NED_FRAME: FrameTag = FrameTag(1);

/// The one clock every Simulate source runs on, so no reading is ever on an
/// epoch of its own.
const MASTER_CLOCK_EPOCH: SourceEpoch = SourceEpoch(1);

/// What one monitored frame hands a display.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MonitoredFrame {
    /// The state fed to Indicate.
    pub state: AircraftState,
    /// What to draw, with the selected source folded in.
    pub panel: PanelData,
    /// Indicate's verdict on each display function this frame.
    pub report: SourceMonitorReport,
    /// The alert stack, sustained miscompares included.
    pub alerts: AlertOutput,
}

/// Offers each attitude source to Indicate's own comparator and keeps what it
/// needs across frames. Simulate never picks the displayed source: a reading's
/// health goes in as evidence and the selection comes back in the frame.
#[derive(Debug, Clone)]
pub struct SourceMonitor {
    monitors: SourceMonitors,
    policies: SourcePolicies,
    profile: AirframeDisplayProfile,
    freshness: FreshnessPolicy,
    unusual: UnusualAttitudeState,
    alerts: AlertManager,
    alert_profile: AlertProfile,
    sequence: u32,
}

impl Default for SourceMonitor {
    fn default() -> Self {
        Self::new()
    }
}

/// One source's reading in display terms.
struct Offered {
    source: SourceId,
    reading: StampedReading<DQuat>,
    display_to_ned: DQuat,
}

impl Offered {
    fn candidate<M>(&self, sequence: u32, usable: bool, measurement: M) -> Candidate<M> {
        let stamp = instant_ms(self.reading.timestamp);
        Candidate {
            source: self.source,
            epoch: MASTER_CLOCK_EPOCH,
            source_time_ms: stamp,
            // Sources share the master clock, so a reading is received when it
            // is taken, and one that stopped updating goes stale by this time.
            receive_time_ms: stamp,
            sequence,
            valid: usable,
            integrity: integrity_of(self.reading.validity),
            accuracy: 0.0,
            measurement,
        }
    }
}

fn integrity_of(validity: SourceValidity) -> IntegrityLevel {
    match validity {
        SourceValidity::Valid => IntegrityLevel::High,
        SourceValidity::Degraded => IntegrityLevel::Low,
        SourceValidity::Stale | SourceValidity::Missing | SourceValidity::Failed => {
            IntegrityLevel::None
        }
    }
}

/// Both of a vehicle's attitude sources, or nothing on a vehicle with one.
fn offered_sources(
    sample: &FlightSample,
    cockpit: &CockpitInstrumentSnapshot,
    feed: &StateFeed,
    local: &LocalLevel,
) -> Result<Vec<Offered>, FeedError> {
    if cockpit.guidance_source.is_none() {
        return Ok(Vec::new());
    }
    [
        (PGNS_SOURCE, cockpit.pgns_attitude),
        (AGS_SOURCE, cockpit.ags_attitude),
    ]
    .into_iter()
    .map(|(source, reading)| {
        Ok(Offered {
            source,
            reading,
            display_to_ned: sample.display_attitude(reading.value, local, &feed.config().axes)?,
        })
    })
    .collect()
}

impl SourceMonitor {
    /// A monitor with the simulator policies, on the primary, nothing heard.
    pub fn new() -> Self {
        Self {
            monitors: SourceMonitors::new(),
            policies: SourcePolicies::simulator(),
            profile: AirframeDisplayProfile::simulator(),
            freshness: FreshnessPolicy::default(),
            unusual: UnusualAttitudeState::default(),
            alerts: AlertManager::new(),
            alert_profile: AlertProfile::simulator(),
            sequence: 0,
        }
    }

    /// Runs one display frame: `sample` through `feed`, then both attitude
    /// sources in its cockpit snapshot through Indicate. A sample with no
    /// sources to compare resolves as a single-source display.
    pub fn step(
        &mut self,
        feed: &mut StateFeed,
        sample: &FlightSample,
    ) -> Result<MonitoredFrame, FeedError> {
        let state = feed.state(sample)?;
        let local = feed.local_level(sample);
        let offered = match &sample.cockpit {
            Some(cockpit) => offered_sources(sample, cockpit, feed, &local)?,
            None => Vec::new(),
        };
        self.sequence = self.sequence.wrapping_add(1);
        let sequence = self.sequence;
        let attitude: Vec<_> = offered
            .iter()
            .map(|source| {
                let measurement = AttitudeMeasure {
                    quat: indicate_quat(source.display_to_ned),
                    frame: LOCAL_NED_FRAME,
                };
                source.candidate(sequence, source.reading.validity.is_usable(), measurement)
            })
            .collect();
        let heading: Vec<_> = offered
            .iter()
            .map(|source| {
                let heading = true_heading_rad(source.display_to_ned);
                let measurement = HeadingMeasure {
                    heading_rad: heading.unwrap_or(f64::NAN) as f32,
                    reference: HeadingReference::SimLocalTrue,
                };
                let usable = heading.is_some() && source.reading.validity.is_usable();
                source.candidate(sequence, usable, measurement)
            })
            .collect();
        let now_ms = instant_ms(sample.timestamp);
        Ok(self.resolve(&state, &attitude, &heading, now_ms))
    }

    fn resolve(
        &mut self,
        state: &AircraftState,
        attitude: &[Candidate<AttitudeMeasure>],
        heading: &[Candidate<HeadingMeasure>],
        now_ms: u64,
    ) -> MonitoredFrame {
        let step = SourceStep {
            inputs: SourceInputs {
                attitude,
                heading,
                ..SourceInputs::default()
            },
            policies: &self.policies,
            now_ms,
        };
        let (panel, report) = resolve_with_sources(
            state,
            &self.freshness,
            &self.profile,
            &mut self.unusual,
            &mut self.monitors,
            &step,
        );
        let events: Vec<AlertEvent> = report.transitions().into_iter().flatten().collect();
        let context = AlertContext {
            declutter: panel.presentation.unusual,
            ..AlertContext::default()
        };
        let alerts = self
            .alerts
            .step(&self.alert_profile, &events, context, now_ms);
        MonitoredFrame {
            state: *state,
            panel,
            report,
            alerts,
        }
    }
}
