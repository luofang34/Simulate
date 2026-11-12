//! The faults a visitor can give the simulated sensors, as a page lists
//! them: each control with what it does, whether it is on, and what every
//! consumer of the sensor makes of it now.
//!
//! A control acts on the device, so every set that has the sensor and the
//! flight controller meet the same fault. What each makes of it is its own
//! report: the page words it and judges nothing.

use sim_navigate::trial::{SensorKind, SensorState, SensorStates};

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

fn stop(monitor: &Monitor, sensor: SensorKind) -> SensorControl {
    let consumers = consumers(monitor, sensor);
    let active = consumers
        .iter()
        .any(|(_, state)| *state == SensorState::ScheduledUnavailable);
    let each: Vec<String> = consumers
        .iter()
        .filter_map(|(trace, state)| Some(format!("{} {}", trace.name, state_words(*state)?)))
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
fn spoofed_words(trace: &SetTrace, state: SensorState) -> String {
    let mut words = format!("{} {}", trace.name, state_words(state).unwrap_or("—"));
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
                .map(|(trace, state)| spoofed_words(trace, state))
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
