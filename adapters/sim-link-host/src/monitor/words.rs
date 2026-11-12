//! The monitor in plain language: times, events, sensor states and the
//! terrain camera's tallies, as a page shows them.

use sim_navigate::trial::{EventKind, EventRecord, SensorKind, SensorState};

use super::Monitor;

#[cfg(test)]
mod tests;

/// Route time as minutes and seconds, `m:ss`; hours as `h:mm:ss`.
pub fn clock_words(t_s: f64) -> String {
    let whole = if t_s.is_finite() && t_s > 0.0 {
        t_s.floor() as u64
    } else {
        0
    };
    let (hours, minutes, seconds) = (whole / 3600, whole / 60 % 60, whole % 60);
    if hours > 0 {
        format!("{hours}:{minutes:02}:{seconds:02}")
    } else {
        format!("{minutes}:{seconds:02}")
    }
}

/// A sensor's name for a reader.
pub const fn sensor_words(sensor: SensorKind) -> &'static str {
    match sensor {
        SensorKind::Gnss => "GNSS",
        SensorKind::Baro => "barometer",
        SensorKind::Stars => "star camera",
        SensorKind::TerrainCamera => "terrain camera",
    }
}

/// What a sensor is doing for a set; `None` when the set does not have it.
pub const fn state_words(state: SensorState) -> Option<&'static str> {
    match state {
        SensorState::Aiding => Some("aiding"),
        SensorState::ScheduledUnavailable => Some("withheld"),
        SensorState::PhysicallyUnavailable => Some("no reading"),
        SensorState::Rejected => Some("refused by the filter"),
        SensorState::NotInSet => None,
    }
}

/// What happened, for the event log.
pub fn event_words(event: &EventRecord) -> String {
    let what = match &event.kind {
        EventKind::WaypointSequenced {
            ident,
            plan_complete,
            ..
        } => {
            if *plan_complete {
                format!("reached {ident}, the last waypoint")
            } else {
                format!("sequenced {ident}")
            }
        }
        EventKind::AvailabilityStart { sensor, reason, .. } => format!(
            "{} withheld{}",
            sensor_words(*sensor),
            reason
                .as_deref()
                .map_or_else(String::new, |reason| format!(": {reason}"))
        ),
        EventKind::AvailabilityEnd { sensor, .. } => {
            format!("{} available again", sensor_words(*sensor))
        }
        EventKind::SpoofingStart {
            sensor,
            rate_m_s,
            bearing_deg,
        } => format!(
            "{} spoofed: its position dragged {rate_m_s} m/s toward {bearing_deg:03.0}°",
            sensor_words(*sensor)
        ),
        EventKind::SpoofingEnd { sensor } => {
            format!("{} spoofing ended", sensor_words(*sensor))
        }
        EventKind::GuidanceRefused { refusal } => {
            format!("guidance refused the solution ({refusal})")
        }
        EventKind::HoldingStarted { pattern } => super::pattern_words(pattern),
        EventKind::HoldingEntered { entry } => {
            format!("entered the hold: {}", super::entry_words(*entry))
        }
        EventKind::HoldingLegChanged { leg, circuits } => format!(
            "hold: {}, {circuits} circuits flown",
            super::leg_words(*leg)
        ),
        EventKind::Terminated { rule, detail } => format!("flight ended: {rule} ({detail})"),
    };
    match &event.set {
        Some(set) => format!("{set}: {what}"),
        None => what,
    }
}

/// What became of `set`'s terrain-camera frames.
pub fn terrain_words(monitor: &Monitor, set: &str) -> String {
    let Some((accepted, reasons)) = monitor.unused_reasons(set) else {
        return "no terrain-camera tally reported yet".to_owned();
    };
    let mut words = format!("{accepted} accepted");
    if !reasons.is_empty() {
        let unused: Vec<String> = reasons
            .iter()
            .map(|(reason, count)| format!("{reason} {count}"))
            .collect();
        words.push_str(&format!("; unused: {}", unused.join(", ")));
    }
    words
}
