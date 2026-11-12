//! The faults a visitor can give the simulated sensors, as a page lists
//! them: each control with what it does, whether it is on, and what every
//! consumer of the sensor makes of it now.
//!
//! A control acts on the device, so every set that has the sensor and the
//! flight controller meet the same fault. What each makes of it is its own
//! report: the page words it and judges nothing.

use sim_navigate::trial::{
    GnssCheck, GnssCheckState, GnssReference, SensorKind, SensorState, SensorStates,
};

use super::{Monitor, SetTrace, clock_words, state_words};

#[cfg(test)]
mod tests;

/// What a control does to its sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorFault {
    /// The device delivers nothing.
    Stop(SensorKind),
    /// A spoofer drags the GNSS receiver's reported position away.
    SpoofGnss,
}

/// One control and its line.
#[derive(Debug, Clone, PartialEq)]
pub struct SensorControl {
    /// What it does.
    pub fault: SensorFault,
    /// Its name on the control.
    pub label: &'static str,
    /// Whether the flight reports the fault in force.
    pub active: bool,
    /// What the fault is and what each consumer of the sensor reports.
    pub state: String,
}

/// The drag a spoofer applies: m/s, and degrees from true north.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpoofDrag {
    /// The rate the reported position leaves the true one, m/s.
    pub rate_m_s: f64,
    /// The direction of the drag, degrees from true north.
    pub bearing_deg: f64,
}

const fn stop_label(sensor: SensorKind) -> &'static str {
    match sensor {
        SensorKind::Gnss => "Stop GNSS",
        SensorKind::Baro => "Stop baro",
        SensorKind::Stars => "Stop star camera",
        SensorKind::TerrainCamera => "Stop camera",
    }
}

const fn state_of(sensors: &SensorStates, sensor: SensorKind) -> SensorState {
    match sensor {
        SensorKind::Gnss => sensors.gnss,
        SensorKind::Baro => sensors.baro,
        SensorKind::Stars => sensors.stars,
        SensorKind::TerrainCamera => sensors.terrain_camera,
    }
}

/// Each consumer of `sensor` with its state: the sets that have it and,
/// for a device it reads, the flight controller.
fn consumers(monitor: &Monitor, sensor: SensorKind) -> Vec<(&SetTrace, SensorState)> {
    monitor
        .traces()
        .iter()
        .filter_map(|trace| {
            let state = state_of(monitor.sensors(&trace.name)?, sensor);
            (state != SensorState::NotInSet).then_some((trace, state))
        })
        .collect()
}

/// What `sensor` is doing for the set of `trace`, in words. A receiver its
/// navigator's own check excluded is said to be excluded: the filter is
/// handed none of its fixes, which is no fault of the device's.
fn consumer_words(
    monitor: &Monitor,
    sensor: SensorKind,
    (trace, state): (&SetTrace, SensorState),
) -> Option<String> {
    let excluded = sensor == SensorKind::Gnss
        && state != SensorState::ScheduledUnavailable
        && monitor
            .gnss_check(&trace.name)
            .is_some_and(|check| check.checkable && check.state == GnssCheckState::Excluded);
    let words = if excluded {
        "excluded by its navigator"
    } else {
        state_words(state)?
    };
    Some(format!("{} {words}", trace.name))
}

fn stop(monitor: &Monitor, sensor: SensorKind) -> SensorControl {
    let consumers = consumers(monitor, sensor);
    let active = consumers
        .iter()
        .any(|(_, state)| *state == SensorState::ScheduledUnavailable);
    let each: Vec<String> = consumers
        .iter()
        .filter_map(|consumer| consumer_words(monitor, sensor, *consumer))
        .collect();
    let state = match (active, each.is_empty()) {
        (_, true) => "no consumer has reported yet".to_owned(),
        (true, false) => format!("STOPPED for every consumer · {}", each.join(" · ")),
        (false, false) => format!("delivering · {}", each.join(" · ")),
    };
    SensorControl {
        fault: SensorFault::Stop(sensor),
        label: stop_label(sensor),
        active,
        state,
    }
}

/// One GNSS consumer under the spoofer: its state, how far its estimate is
/// from the truth, and the uncertainty it states.
fn spoofed_words(monitor: &Monitor, (trace, state): (&SetTrace, SensorState)) -> String {
    let mut words = consumer_words(monitor, SensorKind::Gnss, (trace, state))
        .unwrap_or_else(|| format!("{} —", trace.name));
    if let Some(latest) = trace.latest() {
        words.push_str(&format!(", {:.0} m off", latest.horizontal_error_m));
        if let Some(sigma_m) = latest.sigma_horizontal_m {
            words.push_str(&format!(", σ {sigma_m:.1}"));
        }
    }
    words
}

fn spoof(monitor: &Monitor, by_hand: SpoofDrag) -> SensorControl {
    let (active, state) = match monitor.gnss_spoofing() {
        Some(spoofing) => {
            let dragged_m = spoofing.rate_m_s * (monitor.sim_time_s() - spoofing.since_s).max(0.0);
            let each: Vec<String> = consumers(monitor, SensorKind::Gnss)
                .into_iter()
                .map(|consumer| spoofed_words(monitor, consumer))
                .collect();
            let head = format!(
                "SPOOFING since {}, reported position {dragged_m:.0} m from true toward {:03.0}°",
                clock_words(spoofing.since_s),
                spoofing.bearing_deg
            );
            (true, [vec![head], each].concat().join(" · "))
        }
        None => (
            false,
            format!(
                "true sky · on, every consumer's GNSS position is dragged {} m/s toward \
                 {:03.0}°, its velocity with it",
                by_hand.rate_m_s, by_hand.bearing_deg
            ),
        ),
    };
    SensorControl {
        fault: SensorFault::SpoofGnss,
        label: "Spoof GNSS",
        active,
        state,
    }
}

/// The controls of `monitor`'s run: a stop for each sensor some set uses,
/// and the spoofer when a set uses GNSS. `by_hand` is the drag a spoofer
/// switched on now would apply.
pub fn sensor_controls(monitor: &Monitor, by_hand: SpoofDrag) -> Vec<SensorControl> {
    let mut controls: Vec<SensorControl> = monitor
        .switches()
        .into_iter()
        .map(|switch| stop(monitor, switch.sensor))
        .collect();
    if monitor
        .switches()
        .iter()
        .any(|switch| switch.sensor == SensorKind::Gnss)
    {
        controls.push(spoof(monitor, by_hand));
    }
    controls
}

/// One set's GNSS check in a line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GnssCheckLine {
    /// The set.
    pub set: String,
    /// What its navigator says of the receiver.
    pub words: String,
    /// Whether the navigator doubts or excludes the receiver.
    pub alert: bool,
}

/// The fix against the reference: how far, and the bounds it is held to.
fn separation_words(check: &GnssCheck) -> String {
    check.last.map_or_else(String::new, |last| {
        format!(
            ", separation {:.0} m (caution {:.0} m, exclusion {:.0} m)",
            last.separation_m, last.caution_bound_m, last.exclusion_bound_m
        )
    })
}

/// What the receiver is checked against, and what that reference cannot
/// find.
fn reference_words(reference: GnssReference) -> String {
    match reference {
        GnssReference::PositionFix => "against its camera-fixed solution".to_owned(),
        GnssReference::InertialOnly {
            detectable_rate_m_s: Some(rate_m_s),
        } => format!(
            "against inertial navigation only, which cannot check a drift below {rate_m_s:.1} m/s"
        ),
        GnssReference::InertialOnly {
            detectable_rate_m_s: None,
        } => "against inertial navigation only".to_owned(),
    }
}

fn check_words(check: &GnssCheck) -> (String, bool) {
    let since = check
        .since_s
        .map_or_else(String::new, |t_s| format!(" since {}", clock_words(t_s)));
    let reference = reference_words(check.reference);
    if !check.checkable {
        return (
            format!("GNSS unchecked: the reference is too loose to judge it ({reference})"),
            false,
        );
    }
    let separation = separation_words(check);
    match check.state {
        GnssCheckState::Consistent => (
            format!("GNSS consistent{separation}, checked {reference}"),
            false,
        ),
        GnssCheckState::Disagreeing => (
            format!("GNSS doubted{since}{separation}, still used, checked {reference}"),
            true,
        ),
        GnssCheckState::Excluded => (
            format!("GNSS unreliable — excluded{since}{separation}, checked {reference}"),
            true,
        ),
    }
}

/// Each GNSS set's check as its navigator reports it, in the run's order
/// of sets; a set whose navigator has reported none has no line.
pub fn gnss_check_lines(monitor: &Monitor) -> Vec<GnssCheckLine> {
    monitor
        .traces()
        .iter()
        .filter_map(|trace| {
            let (words, alert) = check_words(monitor.gnss_check(&trace.name)?);
            Some(GnssCheckLine {
                set: trace.name.clone(),
                words,
                alert,
            })
        })
        .collect()
}
