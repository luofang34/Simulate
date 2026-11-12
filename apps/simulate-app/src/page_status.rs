//! The world view's status under its canvas on a link's page: a fixed
//! number of short lines, one group of facts each, so the block keeps its
//! height while the flight runs, pauses or warps, and nothing under it moves.
//! A fact the flight does not have keeps its place with a dash.

use sim_scenario::SimTimeScale;

#[cfg(test)]
mod tests;

/// Lines in the world view's status block, in every state.
pub const PAGE_STATUS_LINES: usize = 3;
/// What stands for a fact the flight does not have.
const ABSENT: &str = "—";

/// The vehicle's engine as the status shows it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EngineState {
    /// The vehicle has no engine the status reads.
    Absent,
    /// The engine is stopped.
    Off,
    /// The engine turns its propeller at this many revolutions a minute.
    Running(f64),
}

/// What the world view's status says, gathered from the flight.
#[derive(Debug, Clone, PartialEq)]
pub struct PageStatus<'a> {
    /// What the view shows: OTW, Chase, a sensor's name.
    pub view: &'a str,
    /// Height above the reference, metres.
    pub height_m: f64,
    /// Speed, metres per second.
    pub speed_m_s: f64,
    /// The engine.
    pub engine: EngineState,
    /// Whether the flight controller is armed, when there is one.
    pub armed: Option<bool>,
    /// The mission phase, when a mission host names one.
    pub phase: Option<&'a str>,
    /// The navigation realism, when the flight is on the Earth.
    pub navigation: Option<&'a str>,
    /// The time scale.
    pub time_scale: SimTimeScale,
    /// Simulated seconds per wall second, rolling.
    pub real_time_ratio: f64,
    /// Frames per second, rolling.
    pub fps: f64,
}

impl PageStatus<'_> {
    /// The block: the vehicle's state, its onboard state, the clock.
    #[must_use]
    pub fn lines(&self) -> [String; PAGE_STATUS_LINES] {
        let engine = match self.engine {
            EngineState::Absent => ABSENT.to_owned(),
            EngineState::Off => "engine off".to_owned(),
            EngineState::Running(rpm) => format!("{rpm:.0} rpm"),
        };
        let armed = match self.armed {
            Some(true) => "FC armed",
            Some(false) => "FC disarmed",
            None => "FC —",
        };
        let clock = match self.time_scale {
            SimTimeScale::Paused => "Paused".to_owned(),
            SimTimeScale::RealTime => "Real time".to_owned(),
            SimTimeScale::Warp(warp) => format!("Warp {warp:.0}×"),
        };
        [
            format!(
                "{} · {:.0} m · {:.0} m/s · {engine}",
                self.view, self.height_m, self.speed_m_s
            ),
            format!(
                "{armed} · {} · {}",
                self.phase.unwrap_or(ABSENT),
                self.navigation.unwrap_or(ABSENT)
            ),
            format!(
                "{clock} · RT {:.2}× · {:.0} fps",
                self.real_time_ratio, self.fps
            ),
        ]
    }
}
