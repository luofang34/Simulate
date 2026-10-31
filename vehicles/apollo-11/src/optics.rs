use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, PI, TAU};

use glam::{DQuat, DVec3};
use sim_core::{
    Block, BlockError, CrewAction, CrewObservation, CrewPortError, CrewStationPort, SimInstant,
    StampedCrewAction, TickSpan,
};
use sim_geo::{NavStar, stars_in_cone};

use crate::site::require_period;

mod realign;

pub use realign::{AlignmentError, ImuRealignment, StarSighting, realign_imu_from_two_star_marks};

#[cfg(test)]
mod tests;

const BLOCK: &str = "optical_sight";

/// Crew panel whose switches pick an AOT detent: the switch id is the sight
/// id, the switch state the detent `0..=5`.
pub const AOT_DETENT_PANEL: u16 = 0o630;
/// Detents of the Alignment Optical Telescope.
pub const AOT_DETENTS: i8 = 6;
/// AOT shaft-angle spacing between detents, rad.
const AOT_DETENT_STEP_RAD: f64 = FRAC_PI_3;
/// AOT half field of view, rad: 30°.
const AOT_FIELD_HALF_ANGLE_RAD: f64 = PI / 6.0;
/// Sextant half field of view, rad: 0.9°.
const SEXTANT_FIELD_HALF_ANGLE_RAD: f64 = 0.9 * PI / 180.0;
/// Scanning telescope half field of view, rad: 30°.
const SCANNING_FIELD_HALF_ANGLE_RAD: f64 = PI / 6.0;
/// Separation at which a star reads as on the reticle, rad.
const RETICLE_HALF_ANGLE_RAD: f64 = 2e-5;
/// Sextant shaft travel either side of zero, rad: 270°.
const SEXTANT_SHAFT_HALF_RANGE_RAD: f64 = 1.5 * PI;
/// Sextant trunnion travel, rad.
const SEXTANT_TRUNNION_RANGE_RAD: (f64, f64) = (0.0, FRAC_PI_2);
/// Star slots in a [`CrewObservation::OpticalSightView`].
const STAR_SLOTS: usize = 8;

/// A sight's field: the telescope either looks where its line of sight points,
/// or sits in a detent with a reticle moving inside a fixed field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SightKind {
    /// Shaft and trunnion drive the optics; the field follows the line of
    /// sight. The CM's sextant and scanning telescope.
    Tracking,
    /// The field is fixed at a detent; shaft and trunnion move the reticle
    /// within it. The LM's Alignment Optical Telescope.
    Detented,
}

/// Setup of an [`OpticalSightBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpticalSightConfig {
    /// Sight id on the crew port.
    pub sight_id: u8,
    /// Sample period.
    pub period: TickSpan,
    /// How the field relates to the line of sight.
    pub kind: SightKind,
    /// Sight-to-navigation-base rotation. The shaft axis is sight `+z`; shaft
    /// angle runs from sight `+x` about it, and trunnion angle tilts the line
    /// of sight away from it.
    pub mount: DQuat,
    /// Half field of view, rad.
    pub field_half_angle_rad: f64,
    /// Separation from the line of sight that counts as centred, rad.
    pub reticle_half_angle_rad: f64,
    /// Shaft travel either side of its centre, rad.
    pub shaft_half_range_rad: f64,
    /// Lowest and highest trunnion angle, rad.
    pub trunnion_range_rad: (f64, f64),
}

impl OpticalSightConfig {
    /// The LM Alignment Optical Telescope: six 60° detents at 45° elevation.
    /// The reticle can reach any point of the 60° field.
    pub fn alignment_optical_telescope(sight_id: u8, period: TickSpan, mount: DQuat) -> Self {
        Self {
            sight_id,
            period,
            kind: SightKind::Detented,
            mount,
            field_half_angle_rad: AOT_FIELD_HALF_ANGLE_RAD,
            reticle_half_angle_rad: RETICLE_HALF_ANGLE_RAD,
            shaft_half_range_rad: FRAC_PI_4,
            trunnion_range_rad: (
                FRAC_PI_4 - AOT_FIELD_HALF_ANGLE_RAD,
                FRAC_PI_4 + AOT_FIELD_HALF_ANGLE_RAD,
            ),
        }
    }

    /// The CM's narrow 28-power sextant.
    pub fn sextant(sight_id: u8, period: TickSpan, mount: DQuat) -> Self {
        Self {
            sight_id,
            period,
            kind: SightKind::Tracking,
            mount,
            field_half_angle_rad: SEXTANT_FIELD_HALF_ANGLE_RAD,
            reticle_half_angle_rad: RETICLE_HALF_ANGLE_RAD,
            shaft_half_range_rad: SEXTANT_SHAFT_HALF_RANGE_RAD,
            trunnion_range_rad: SEXTANT_TRUNNION_RANGE_RAD,
        }
    }

    /// The CM's wide unity-power scanning telescope.
    pub fn scanning_telescope(sight_id: u8, period: TickSpan, mount: DQuat) -> Self {
        Self {
            field_half_angle_rad: SCANNING_FIELD_HALF_ANGLE_RAD,
            ..Self::sextant(sight_id, period, mount)
        }
    }
}

/// A `MARK`: where the optics pointed, with the gimbal angles the computer
/// sampled at the same instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SightMark {
    /// When the mark was taken.
    pub at: SimInstant,
    /// Sight that took it.
    pub sight_id: u8,
    /// Shaft angle, rad.
    pub shaft_rad: f64,
    /// Trunnion angle, rad.
    pub trunnion_rad: f64,
    /// Line of sight in stable-member axes at the mark.
    pub line_of_sight_platform: DVec3,
}

/// What an [`OpticalSightBlock`] reads each step.
#[derive(Debug, Clone, PartialEq)]
pub struct OpticalSightInput {
    /// The instant the step ends at.
    pub now: SimInstant,
    /// Crew actions due by `now`, from [`CrewStationPort::take_due`]; those
    /// for other sights and panels are ignored.
    pub actions: Vec<StampedCrewAction>,
    /// True navigation-base-to-J2000 rotation, which sets the sky in view.
    pub navbase_to_inertial: DQuat,
    /// Navigation-base-to-stable-member rotation from the gimbal angles.
    pub navbase_to_platform: DQuat,
}

/// What an [`OpticalSightBlock`] publishes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpticalSightOutput {
    /// Shaft angle, rad.
    pub shaft_rad: f64,
    /// Trunnion angle, rad.
    pub trunnion_rad: f64,
    /// Line of sight in J2000 axes.
    pub line_of_sight_inertial: DVec3,
    /// Octal ids of the stars in the field; the first `visible_count` count.
    pub visible_star_octals: [u8; STAR_SLOTS],
    /// `[shaft, trunnion]` slew onto each star in view, rad.
    pub visible_star_offsets_rad: [[f64; 2]; STAR_SLOTS],
    /// Stars in the field.
    pub visible_count: u8,
    /// Whether a star sits on the reticle.
    pub target_centered: bool,
}

/// Complete state of an [`OpticalSightBlock`].
#[derive(Debug, Clone, PartialEq)]
pub struct OpticalSightSnapshot {
    shaft_center_rad: f64,
    shaft_rate_rad_s: f64,
    trunnion_rate_rad_s: f64,
    output: OpticalSightOutput,
    marks: Vec<SightMark>,
}

/// A hand-driven star sight: the CM's sextant and scanning telescope, or the
/// LM's detented Alignment Optical Telescope.
///
/// It is operated through [`CrewStationPort`] with the same two controls
/// whoever is at the eyepiece: [`CrewAction::OpticalSlew`] holds a shaft and
/// trunnion rate, and [`CrewAction::OpticalMark`] records the angles. Each
/// step runs the slew up to `now`, applies the actions due, then looks at the
/// sky, so a mark sees the line of sight that the view at `now` reports.
///
/// The sky is the navigation stars alone: the Sun, Earth and Moon neither
/// appear in the field nor block it.
#[derive(Debug, Clone, PartialEq)]
pub struct OpticalSightBlock {
    config: OpticalSightConfig,
    snapshot: OpticalSightSnapshot,
}

impl OpticalSightBlock {
    /// A sight at the centre of its travel, stopped. Nonpositive angles or a
    /// zero period are errors.
    pub fn new(config: OpticalSightConfig) -> Result<Self, BlockError> {
        require_period(BLOCK, config.period)?;
        let (trunnion_min, trunnion_max) = config.trunnion_range_rad;
        let valid = config.field_half_angle_rad > 0.0
            && config.reticle_half_angle_rad > 0.0
            && config.shaft_half_range_rad > 0.0
            && trunnion_min <= trunnion_max
            && config.mount.is_normalized();
        if !valid {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "config",
                detail: format!("{config:?} is not a sight"),
            });
        }
        let trunnion_rad = (trunnion_min + trunnion_max) / 2.0;
        Ok(Self {
            config,
            snapshot: OpticalSightSnapshot {
                shaft_center_rad: 0.0,
                shaft_rate_rad_s: 0.0,
                trunnion_rate_rad_s: 0.0,
                output: OpticalSightOutput {
                    shaft_rad: 0.0,
                    trunnion_rad,
                    line_of_sight_inertial: DVec3::ZERO,
                    visible_star_octals: [0; STAR_SLOTS],
                    visible_star_offsets_rad: [[0.0; 2]; STAR_SLOTS],
                    visible_count: 0,
                    target_centered: false,
                },
                marks: Vec::new(),
            },
        })
    }

    /// Every mark taken so far, oldest first.
    pub fn marks(&self) -> &[SightMark] {
        &self.snapshot.marks
    }

    /// Publishes the current view at `at`.
    pub fn publish_view(
        &self,
        at: SimInstant,
        port: &mut CrewStationPort,
    ) -> Result<(), CrewPortError> {
        let output = &self.snapshot.output;
        port.publish(
            at,
            CrewObservation::OpticalSightView {
                sight_id: self.config.sight_id,
                shaft_rad: output.shaft_rad,
                trunnion_rad: output.trunnion_rad,
                visible_star_octals: output.visible_star_octals,
                visible_count: output.visible_count,
                visible_star_offsets_rad: output.visible_star_offsets_rad,
                target_centered: output.target_centered,
            },
        )
    }

    fn slewed(&self, dt_s: f64) -> (f64, f64) {
        let snapshot = &self.snapshot;
        let (trunnion_min, trunnion_max) = self.config.trunnion_range_rad;
        let center = snapshot.shaft_center_rad;
        let reach = self.config.shaft_half_range_rad;
        let shaft = snapshot.output.shaft_rad + snapshot.shaft_rate_rad_s * dt_s;
        let trunnion = snapshot.output.trunnion_rad + snapshot.trunnion_rate_rad_s * dt_s;
        (
            shaft.clamp(center - reach, center + reach),
            trunnion.clamp(trunnion_min, trunnion_max),
        )
    }

    fn apply(&mut self, action: &CrewAction, input: &OpticalSightInput) -> Result<(), BlockError> {
        let id = self.config.sight_id;
        match *action {
            CrewAction::OpticalSlew {
                sight_id,
                shaft_rate_rad_s,
                trunnion_rate_rad_s,
            } if sight_id == id => {
                self.snapshot.shaft_rate_rad_s = shaft_rate_rad_s;
                self.snapshot.trunnion_rate_rad_s = trunnion_rate_rad_s;
            }
            CrewAction::OpticalMark { sight_id } if sight_id == id => self.mark(input),
            CrewAction::SetSwitch {
                panel_id: AOT_DETENT_PANEL,
                switch_id,
                state,
            } if switch_id == u16::from(id) && self.config.kind == SightKind::Detented => {
                self.select_detent(state)?;
            }
            _ => {}
        }
        Ok(())
    }

    fn select_detent(&mut self, detent: i8) -> Result<(), BlockError> {
        if !(0..AOT_DETENTS).contains(&detent) {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "detent",
                detail: format!("{detent} is not one of {AOT_DETENTS} detents"),
            });
        }
        let (trunnion_min, trunnion_max) = self.config.trunnion_range_rad;
        let snapshot = &mut self.snapshot;
        snapshot.shaft_center_rad = f64::from(detent) * AOT_DETENT_STEP_RAD;
        snapshot.output.shaft_rad = snapshot.shaft_center_rad;
        snapshot.output.trunnion_rad = (trunnion_min + trunnion_max) / 2.0;
        snapshot.shaft_rate_rad_s = 0.0;
        snapshot.trunnion_rate_rad_s = 0.0;
        Ok(())
    }

    fn mark(&mut self, input: &OpticalSightInput) {
        let output = &self.snapshot.output;
        let line_of_sight_navbase =
            self.config.mount * sight_direction(output.shaft_rad, output.trunnion_rad);
        let mark = SightMark {
            at: input.now,
            sight_id: self.config.sight_id,
            shaft_rad: output.shaft_rad,
            trunnion_rad: output.trunnion_rad,
            line_of_sight_platform: input.navbase_to_platform * line_of_sight_navbase,
        };
        self.snapshot.marks.push(mark);
    }

    /// The direction the field is centred on, in sight axes.
    fn boresight(&self) -> DVec3 {
        let output = &self.snapshot.output;
        match self.config.kind {
            SightKind::Tracking => sight_direction(output.shaft_rad, output.trunnion_rad),
            SightKind::Detented => {
                let (trunnion_min, trunnion_max) = self.config.trunnion_range_rad;
                let elevation = (trunnion_min + trunnion_max) / 2.0;
                sight_direction(self.snapshot.shaft_center_rad, elevation)
            }
        }
    }

    fn look(&mut self, navbase_to_inertial: DQuat) {
        let sight_to_inertial = (navbase_to_inertial * self.config.mount).normalize();
        let boresight = sight_to_inertial * self.boresight();
        let reticle_half_angle = self.config.reticle_half_angle_rad;
        let field = self.config.field_half_angle_rad;
        let output = &mut self.snapshot.output;
        let (shaft, trunnion) = (output.shaft_rad, output.trunnion_rad);
        let line_of_sight = sight_direction(shaft, trunnion);
        output.line_of_sight_inertial = sight_to_inertial * line_of_sight;
        output.visible_star_octals = [0; STAR_SLOTS];
        output.visible_star_offsets_rad = [[0.0; 2]; STAR_SLOTS];
        output.visible_count = 0;
        output.target_centered = false;
        let slots = output
            .visible_star_octals
            .iter_mut()
            .zip(&mut output.visible_star_offsets_rad);
        for (star, (octal, offset)) in stars_in_cone(boresight, field).zip(slots) {
            let in_sight = sight_to_inertial.inverse() * star.direction_j2000;
            *octal = star.apollo_octal_id;
            *offset = offset_to(in_sight, shaft, trunnion);
            output.visible_count = output.visible_count.wrapping_add(1);
            output.target_centered |= separation(in_sight, line_of_sight) <= reticle_half_angle;
        }
    }
}

/// Line of sight in sight axes at `shaft_rad`, `trunnion_rad`.
fn sight_direction(shaft_rad: f64, trunnion_rad: f64) -> DVec3 {
    let (sin_shaft, cos_shaft) = shaft_rad.sin_cos();
    let (sin_trunnion, cos_trunnion) = trunnion_rad.sin_cos();
    DVec3::new(
        sin_trunnion * cos_shaft,
        sin_trunnion * sin_shaft,
        cos_trunnion,
    )
}

/// The `[shaft, trunnion]` change that centres the star at `in_sight`, taking
/// the short way round in shaft.
fn offset_to(in_sight: DVec3, shaft_rad: f64, trunnion_rad: f64) -> [f64; 2] {
    let star_shaft = in_sight.y.atan2(in_sight.x);
    let star_trunnion = in_sight.x.hypot(in_sight.y).atan2(in_sight.z);
    let shaft_turn = (star_shaft - shaft_rad + PI).rem_euclid(TAU) - PI;
    [shaft_turn, star_trunnion - trunnion_rad]
}

/// Angle between two unit vectors, accurate at the small separations of a
/// reticle.
fn separation(a: DVec3, b: DVec3) -> f64 {
    a.cross(b).length().atan2(a.dot(b))
}

impl Block for OpticalSightBlock {
    type Input = OpticalSightInput;
    type Output = OpticalSightOutput;
    type Snapshot = OpticalSightSnapshot;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &OpticalSightInput) -> Result<(), BlockError> {
        if !input.navbase_to_inertial.is_finite() || !input.navbase_to_platform.is_finite() {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "attitude",
                detail: format!("{input:?} is not finite"),
            });
        }
        let (shaft_rad, trunnion_rad) = self.slewed(self.config.period.as_seconds());
        self.snapshot.output.shaft_rad = shaft_rad;
        self.snapshot.output.trunnion_rad = trunnion_rad;
        for stamped in &input.actions {
            self.apply(&stamped.action, input)?;
        }
        self.look(input.navbase_to_inertial);
        Ok(())
    }

    fn output(&self) -> OpticalSightOutput {
        self.snapshot.output
    }

    fn snapshot(&self) -> OpticalSightSnapshot {
        self.snapshot.clone()
    }

    fn restore(&mut self, snapshot: &OpticalSightSnapshot) {
        self.snapshot = snapshot.clone();
    }
}

impl StarSighting {
    /// `mark` taken on `star`.
    pub fn from_mark(mark: &SightMark, star: &NavStar) -> Self {
        Self {
            line_of_sight_platform: mark.line_of_sight_platform,
            star_direction_j2000: star.direction_j2000,
        }
    }
}
