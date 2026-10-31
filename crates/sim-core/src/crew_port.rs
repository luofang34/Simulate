use std::collections::BTreeMap;

use crate::{SimInstant, TickSpan};

#[cfg(test)]
mod tests;

/// Star slots in an optical sight report.
const STAR_SLOTS: u8 = 8;
/// The smallest step past an instant on the master clock.
const ONE_TICK: TickSpan = TickSpan::from_ticks(1);

/// Something a crew member does: a person at a panel, an autopilot in
/// `Pilotage`, or an external coding agent, all through the same port.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CrewAction {
    /// Moves a panel switch.
    SetSwitch {
        /// Panel.
        panel_id: u16,
        /// Switch on the panel.
        switch_id: u16,
        /// New position: `-1`, `0`, `1` for a three-position switch.
        state: i8,
    },
    /// Presses a key on an AGC DSKY or AGS DEDA.
    KeypadPress {
        /// Keypad unit.
        unit_id: u8,
        /// Key code.
        key_code: u8,
    },
    /// Slews an optical sight.
    OpticalSlew {
        /// Sight.
        sight_id: u8,
        /// Shaft rate, rad/s.
        shaft_rate_rad_s: f64,
        /// Trunnion rate, rad/s.
        trunnion_rate_rad_s: f64,
    },
    /// Marks the target at the sight's reticle.
    OpticalMark {
        /// Sight.
        sight_id: u8,
    },
    /// Holds the control stick. Axes follow [`crate::ControlInputs`].
    ControlStick {
        /// Roll axis.
        roll: f64,
        /// Pitch axis.
        pitch: f64,
        /// Yaw axis.
        yaw: f64,
        /// Throttle.
        throttle: f64,
    },
}

impl CrewAction {
    fn is_finite(&self) -> bool {
        match *self {
            Self::OpticalSlew {
                shaft_rate_rad_s,
                trunnion_rate_rad_s,
                ..
            } => shaft_rate_rad_s.is_finite() && trunnion_rate_rad_s.is_finite(),
            Self::ControlStick {
                roll,
                pitch,
                yaw,
                throttle,
            } => [roll, pitch, yaw, throttle].iter().all(|v| v.is_finite()),
            Self::SetSwitch { .. } | Self::KeypadPress { .. } | Self::OpticalMark { .. } => true,
        }
    }
}

/// Something a crew member can see.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CrewObservation {
    /// The view through an optical sight.
    OpticalSightView {
        /// Sight.
        sight_id: u8,
        /// Shaft angle, rad.
        shaft_rad: f64,
        /// Trunnion angle, rad.
        trunnion_rad: f64,
        /// Octal ids of the stars in view; only the first `visible_count` are
        /// meaningful.
        visible_star_octals: [u8; 8],
        /// Stars in view, at most 8.
        visible_count: u8,
        /// `[shaft, trunnion]` change that would put each star in
        /// `visible_star_octals` on the reticle, rad, as a person reads it off
        /// the sight's field of view.
        visible_star_offsets_rad: [[f64; 2]; 8],
        /// Whether the target sits on the reticle.
        target_centered: bool,
    },
    /// A display register's characters.
    DisplayRegister {
        /// Display unit.
        unit_id: u8,
        /// Register on the unit.
        reg_id: u8,
        /// Characters shown.
        characters: [u8; 8],
    },
    /// A caution or status lamp.
    AnnunciatorLamp {
        /// Lamp.
        lamp_id: u16,
        /// Whether it is lit.
        lit: bool,
    },
}

/// The indicator an observation updates, so each keeps only its latest state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Indicator {
    Sight(u8),
    Display(u8, u8),
    Lamp(u16),
}

impl CrewObservation {
    fn indicator(&self) -> Indicator {
        match *self {
            Self::OpticalSightView { sight_id, .. } => Indicator::Sight(sight_id),
            Self::DisplayRegister {
                unit_id, reg_id, ..
            } => Indicator::Display(unit_id, reg_id),
            Self::AnnunciatorLamp { lamp_id, .. } => Indicator::Lamp(lamp_id),
        }
    }

    fn validate(&self) -> Result<(), CrewPortError> {
        if let Self::OpticalSightView {
            shaft_rad,
            trunnion_rad,
            visible_count,
            visible_star_offsets_rad,
            ..
        } = *self
        {
            let offsets_finite = visible_star_offsets_rad
                .iter()
                .flatten()
                .all(|v| v.is_finite());
            if !shaft_rad.is_finite() || !trunnion_rad.is_finite() || !offsets_finite {
                return Err(CrewPortError::NonFinite {
                    detail: format!("{self:?}"),
                });
            }
            if visible_count > STAR_SLOTS {
                return Err(CrewPortError::TooManyStars { visible_count });
            }
        }
        Ok(())
    }
}

/// A [`CrewAction`] and the instant it takes effect.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StampedCrewAction {
    /// When the action takes effect.
    pub at: SimInstant,
    /// The action.
    pub action: CrewAction,
}

/// A [`CrewObservation`] and the instant it was true.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StampedCrewObservation {
    /// When the observation was made.
    pub at: SimInstant,
    /// The observation.
    pub observation: CrewObservation,
}

/// Why the port refused an action or observation.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum CrewPortError {
    /// A rate, stick axis or sight angle is not finite.
    #[error("crew value is not finite: {detail}")]
    NonFinite {
        /// The offending value.
        detail: String,
    },
    /// An action is stamped at or before an instant already delivered, so
    /// accepting it would make the order depend on when it arrived.
    #[error("crew action stamped {at:?} is at or before delivered instant {delivered_through:?}")]
    StaleAction {
        /// The action's stamp.
        at: SimInstant,
        /// The latest instant already delivered.
        delivered_through: SimInstant,
    },
    /// An observation is older than the one already held for its indicator.
    #[error("crew observation stamped {at:?} is older than the held {held:?}")]
    StaleObservation {
        /// The observation's stamp.
        at: SimInstant,
        /// The stamp of the held observation.
        held: SimInstant,
    },
    /// A sight reports more stars than its slots hold.
    #[error("sight reports {visible_count} stars, more than 8")]
    TooManyStars {
        /// The reported count.
        visible_count: u8,
    },
}

/// The crew's interface to the vehicle: a [`SimInstant`]-stamped action queue
/// and the latest observation of each indicator.
///
/// A person at a UI, `Pilotage`, and an external coding agent all operate a
/// sextant or DSKY here, so a run replays identically whoever flew it: actions
/// come out in stamp order, and in submission order within one instant. The
/// whole port is plain data; a clone is its snapshot.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CrewStationPort {
    queued: BTreeMap<(SimInstant, u64), CrewAction>,
    next_sequence: u64,
    delivered_through: Option<SimInstant>,
    observations: BTreeMap<Indicator, StampedCrewObservation>,
}

impl CrewStationPort {
    /// An empty port.
    pub fn new() -> Self {
        Self::default()
    }

    /// Queues `action` to take effect at `at`.
    pub fn submit(&mut self, at: SimInstant, action: CrewAction) -> Result<(), CrewPortError> {
        if !action.is_finite() {
            return Err(CrewPortError::NonFinite {
                detail: format!("{action:?}"),
            });
        }
        if let Some(delivered_through) = self.delivered_through
            && at <= delivered_through
        {
            return Err(CrewPortError::StaleAction {
                at,
                delivered_through,
            });
        }
        self.queued.insert((at, self.next_sequence), action);
        self.next_sequence = self.next_sequence.wrapping_add(1);
        Ok(())
    }

    /// Removes and returns every action due by `now`, in delivery order.
    pub fn take_due(&mut self, now: SimInstant) -> Vec<StampedCrewAction> {
        let later = self.queued.split_off(&(now.advanced(ONE_TICK), 0));
        let due = std::mem::replace(&mut self.queued, later);
        if self.delivered_through.is_none_or(|through| now > through) {
            self.delivered_through = Some(now);
        }
        due.into_iter()
            .map(|((at, _), action)| StampedCrewAction { at, action })
            .collect()
    }

    /// Number of actions waiting.
    pub fn queued_len(&self) -> usize {
        self.queued.len()
    }

    /// Records the latest state of an indicator.
    pub fn publish(
        &mut self,
        at: SimInstant,
        observation: CrewObservation,
    ) -> Result<(), CrewPortError> {
        observation.validate()?;
        let indicator = observation.indicator();
        if let Some(held) = self.observations.get(&indicator)
            && at < held.at
        {
            return Err(CrewPortError::StaleObservation { at, held: held.at });
        }
        self.observations
            .insert(indicator, StampedCrewObservation { at, observation });
        Ok(())
    }

    /// The latest observation of every indicator, in a stable order.
    pub fn latest_observations(&self) -> Vec<StampedCrewObservation> {
        self.observations.values().copied().collect()
    }
}
