//! Cockpit controls (wheel, levers, pedals, trim wheels) that turn or slide on joints
//! following where the pilot has set them, so the cockpit shows the inputs the flight
//! model is flying.

use glam::{DMat4, DVec3};
use sim_core::{CockpitControls, FlapSelector};

/// The pilot's control a [`ControlJoint`] follows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlChannel {
    /// Control column, −1 full forward to 1 full back.
    Pitch,
    /// Control wheel, −1 full left to 1 full right.
    Roll,
    /// Rudder pedals, −1 full left to 1 full right.
    Yaw,
    /// An engine's throttle, 0 closed to 1 open (`0` left, `1` right).
    Throttle(usize),
    /// An engine's propeller lever, 0 coarse to 1 fine.
    Propeller(usize),
    /// An engine's mixture lever, 0 idle cut-off to 1 full rich.
    Mixture(usize),
    /// A trim control, −1 to 1 (`0` aileron, `1` elevator, `2` rudder).
    Trim(usize),
    /// The flap selector's detent: 0 `UP`, 1 `CLOSED`, 2 `MANEUVER`, 3 `DOWN`.
    FlapSelector,
    /// The landing-gear lever: 0 up, 1 down.
    GearLever,
}

impl ControlChannel {
    /// The channel named by a `sim:control*:channel` token, if any.
    #[must_use]
    pub fn from_token(token: &str) -> Option<Self> {
        let engine = |side: &str| match side {
            "left" => Some(0),
            "right" => Some(1),
            _ => None,
        };
        Some(match token.split_once('_') {
            None => match token {
                "pitch" => Self::Pitch,
                "roll" => Self::Roll,
                "yaw" => Self::Yaw,
                _ => return None,
            },
            Some(("throttle", side)) => Self::Throttle(engine(side)?),
            Some(("propeller", side)) => Self::Propeller(engine(side)?),
            Some(("mixture", side)) => Self::Mixture(engine(side)?),
            Some(("trim", axis)) => Self::Trim(match axis {
                "aileron" => 0,
                "elevator" => 1,
                "rudder" => 2,
                _ => return None,
            }),
            Some(("flap", "selector")) => Self::FlapSelector,
            Some(("gear", "lever")) => Self::GearLever,
            Some(_) => return None,
        })
    }

    /// The channel's value in `cockpit`.
    #[must_use]
    pub fn value(self, cockpit: &CockpitControls) -> f64 {
        let pick = |values: &[f64], i: usize| values.get(i).copied().unwrap_or(0.0);
        match self {
            Self::Pitch => cockpit.pitch_norm,
            Self::Roll => cockpit.roll_norm,
            Self::Yaw => cockpit.yaw_norm,
            Self::Throttle(i) => pick(&cockpit.throttle_norm, i),
            Self::Propeller(i) => pick(&cockpit.prop_norm, i),
            Self::Mixture(i) => pick(&cockpit.mixture_norm, i),
            Self::Trim(i) => pick(&cockpit.trim_norm, i),
            Self::FlapSelector => match cockpit.flap_selector {
                FlapSelector::Up => 0.0,
                FlapSelector::Closed => 1.0,
                FlapSelector::Maneuver => 2.0,
                FlapSelector::Down => 3.0,
            },
            Self::GearLever => f64::from(u8::from(cockpit.gear_lever_down)),
        }
    }
}

/// One joint of a cockpit control, in its part's local frame: it turns about (or
/// slides along) `axis` through `origin` by `per_unit` for each unit its channel's
/// value is away from `rest`, the value the mesh is modelled at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlJoint {
    /// The control followed.
    pub channel: ControlChannel,
    /// A point on the joint axis, part-local metres.
    pub origin: DVec3,
    /// The joint axis, part-local unit vector.
    pub axis: DVec3,
    /// Radians (or metres, sliding) per unit of the channel's value.
    pub per_unit: f64,
    /// The channel value the mesh is modelled at.
    pub rest: f64,
    /// Whether the joint slides along its axis instead of turning about it.
    pub slides: bool,
}

impl ControlJoint {
    fn matrix(&self, cockpit: &CockpitControls) -> DMat4 {
        let value = self.channel.value(cockpit);
        let travel = if value.is_finite() {
            (value - self.rest) * self.per_unit
        } else {
            0.0
        };
        let axis = self.axis.normalize_or_zero();
        if self.slides {
            DMat4::from_translation(axis * travel)
        } else {
            DMat4::from_translation(self.origin)
                * DMat4::from_axis_angle(axis, travel)
                * DMat4::from_translation(-self.origin)
        }
    }
}

/// The part-local motion of a control on `joints`, the first joint outermost: a
/// wheel turning on a column that itself pitches lists the column's joint first.
pub(super) fn control_matrix(joints: &[ControlJoint], cockpit: &CockpitControls) -> DMat4 {
    joints
        .iter()
        .fold(DMat4::IDENTITY, |m, joint| m * joint.matrix(cockpit))
}

/// The renderer's joints for the P-38G's authored ones, dropping any whose channel
/// token is unknown.
pub(super) fn from_p38(authored: &[sim_p38g::P38ControlJoint]) -> Vec<ControlJoint> {
    authored
        .iter()
        .filter_map(|joint| {
            let Some(channel) = ControlChannel::from_token(joint.channel) else {
                tracing::warn!(
                    token = joint.channel,
                    "cockpit control joint follows an unknown channel"
                );
                return None;
            };
            Some(ControlJoint {
                channel,
                origin: DVec3::from_array(joint.origin_m.map(f64::from)),
                axis: DVec3::from_array(joint.axis.map(f64::from)),
                per_unit: f64::from(joint.per_unit),
                rest: f64::from(joint.rest),
                slides: joint.slides,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests;
