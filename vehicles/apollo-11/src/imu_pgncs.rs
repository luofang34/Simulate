use glam::{DQuat, DVec3, I64Vec3};
use mbd_core::{DigitalBusFrame, ElectricalPinState};
use sim_core::{
    Block, BlockError, CrewObservation, CrewPortError, CrewStationPort, SimInstant, TickSpan,
};

use crate::InertialSiteSample;
use crate::site::{require_period, rotation_over};

mod gimbal;

pub use gimbal::GimbalAngles;

#[cfg(test)]
mod tests;

const BLOCK: &str = "three_gimbal_imu";

/// Middle gimbal angle that lights the `GIMBAL LOCK` caution, degrees.
pub const GIMBAL_LOCK_WARNING_DEG: f64 = 70.0;
/// Middle gimbal angle at which the platform is captured, degrees.
pub const GIMBAL_LOCK_CAPTURE_DEG: f64 = 85.0;
/// LM PIPA scale, m/s per pulse: 1 cm/s.
pub const PIPA_PULSE_LM_M_S: f64 = 0.01;
/// CM PIPA scale, m/s per pulse: 5.85 cm/s.
pub const PIPA_PULSE_CM_M_S: f64 = 0.0585;
/// Lamp that [`ThreeGimbalImuBlock::publish_lamp`] drives.
pub const GIMBAL_LOCK_LAMP: u16 = 0o101;

/// CDU counts per turn: a 15-bit shaft encoding.
const CDU_COUNTS_PER_TURN: i32 = 1 << 15;
/// Excitation on the gimbal resolvers, V.
const RESOLVER_EXCITATION_V: f64 = 28.0;
/// Gyro drift of one milli-earth-rate unit, the IMU's specified class, rad/s.
const ONE_MERU_RAD_S: f64 = 7.292e-8;
/// Rate at which a captured stable member falls away from the base, rad/s.
const CAPTURE_TUMBLE_RATE_RAD_S: DVec3 = DVec3::new(0.15, -0.4, 0.25);

/// Setup of a [`ThreeGimbalImuBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThreeGimbalImuConfig {
    /// Sample period.
    pub period: TickSpan,
    /// Stable-member-to-J2000 rotation at the start, which the computer also
    /// starts out believing.
    pub initial_platform_to_inertial: DQuat,
    /// Stable-member gyro drift in stable-member axes, rad/s.
    pub gyro_drift_rad_s: DVec3,
    /// PIPA scale, m/s per pulse.
    pub pipa_m_s_per_pulse: f64,
    /// Rate of a captured stable member relative to the navigation base, in
    /// stable-member axes, rad/s.
    pub capture_tumble_rate_rad_s: DVec3,
}

impl ThreeGimbalImuConfig {
    /// A one-meru platform with LM PIPAs, aligned to `platform_to_inertial`.
    pub fn nominal(period: TickSpan, platform_to_inertial: DQuat) -> Self {
        Self {
            period,
            initial_platform_to_inertial: platform_to_inertial,
            gyro_drift_rad_s: DVec3::new(1.0, -0.6, 0.8) * ONE_MERU_RAD_S,
            pipa_m_s_per_pulse: PIPA_PULSE_LM_M_S,
            capture_tumble_rate_rad_s: CAPTURE_TUMBLE_RATE_RAD_S,
        }
    }
}

/// What a [`ThreeGimbalImuBlock`] publishes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuOutput {
    /// Outer gimbal angle, rad.
    pub oga_rad: f64,
    /// Middle gimbal angle, rad.
    pub mga_rad: f64,
    /// Inner gimbal angle, rad.
    pub iga_rad: f64,
    /// The `GIMBAL LOCK` caution: `|mga_rad|` is at or past the warning angle.
    pub gimbal_lock_warning: bool,
    /// Latched capture: the gimbals have run out of authority.
    pub gimbal_locked: bool,
    /// Whether the stable member holds an inertial reference.
    pub platform_valid: bool,
    /// Where the stable member actually points: stable-member-to-J2000.
    pub platform_to_inertial: DQuat,
    /// Where the guidance computer believes the stable member points.
    pub estimated_platform_to_inertial: DQuat,
    /// Navigation-base-to-stable-member rotation, as the gimbal resolvers
    /// read it.
    pub navbase_to_platform: DQuat,
    /// The attitude the computer derives: its belief composed with the gimbal
    /// angles, navigation-base-to-J2000.
    pub indicated_navbase_to_inertial: DQuat,
    /// Accumulated PIPA pulses per stable-member axis.
    pub pipa_counts: [i64; 3],
    /// 15-bit CDU shaft encodings: outer, inner, middle.
    pub cdu_counts: [u16; 3],
    /// Pulses since the previous sample, one frame per CDU.
    pub cdu_pulses: [DigitalBusFrame; 3],
    /// Resolver windings, one per CDU.
    pub resolvers: [ElectricalPinState; 3],
}

/// Complete state of a [`ThreeGimbalImuBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSnapshot {
    output: ImuOutput,
    navbase_to_inertial: DQuat,
    pipa_residual_m_s: DVec3,
}

/// The MIT/AC Electronics gimballed inertial platform, modelled with the
/// defect that shaped how Apollo was flown.
///
/// Three gimbals hold a gyro-stabilised stable member still while the vehicle
/// turns around it. The stable member and the computer's belief about it are
/// separate states: drift and capture move the first, star alignment corrects
/// the second, and their difference is the attitude error.
///
/// As the outer gimbal axis nears the inner one the middle gimbal angle nears
/// 90°, and at [`GIMBAL_LOCK_CAPTURE_DEG`] the capture latches: the gimbals
/// lose authority, the stable member is dragged with the vehicle and falls
/// away from it at the configured tumble rate, and only
/// [`ThreeGimbalImuBlock::coarse_align`] or
/// [`ThreeGimbalImuBlock::cage_to_zero`] bring it back. Capture is sampled, so
/// the period must be short enough that the vehicle cannot cross the 10° band
/// around 90° between samples.
///
/// PIPA and resolver errors are not modelled; drift is a constant bias.
#[derive(Debug, Clone, PartialEq)]
pub struct ThreeGimbalImuBlock {
    config: ThreeGimbalImuConfig,
    snapshot: ImuSnapshot,
}

impl ThreeGimbalImuBlock {
    /// A platform with zero gimbal angles, its base aligned to it. A zero
    /// period or nonpositive PIPA scale is an error.
    pub fn new(config: ThreeGimbalImuConfig) -> Result<Self, BlockError> {
        require_period(BLOCK, config.period)?;
        if !(config.pipa_m_s_per_pulse.is_finite() && config.pipa_m_s_per_pulse > 0.0) {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "pipa_m_s_per_pulse",
                detail: format!("{} is not a positive scale", config.pipa_m_s_per_pulse),
            });
        }
        let platform = config.initial_platform_to_inertial;
        let output = ImuOutput {
            oga_rad: 0.0,
            mga_rad: 0.0,
            iga_rad: 0.0,
            gimbal_lock_warning: false,
            gimbal_locked: false,
            platform_valid: true,
            platform_to_inertial: platform,
            estimated_platform_to_inertial: platform,
            navbase_to_platform: DQuat::IDENTITY,
            indicated_navbase_to_inertial: platform,
            pipa_counts: [0; 3],
            cdu_counts: [0; 3],
            cdu_pulses: pulses([0; 3], [0; 3]),
            resolvers: resolvers(&GimbalAngles::default()),
        };
        Ok(Self {
            config,
            snapshot: ImuSnapshot {
                output,
                navbase_to_inertial: platform,
                pipa_residual_m_s: DVec3::ZERO,
            },
        })
    }

    /// Whether the platform is captured and faulted.
    pub fn is_gimbal_locked(&self) -> bool {
        self.snapshot.output.gimbal_locked
    }

    /// Drives the stable member to `target_platform_to_inertial` and makes it
    /// the computer's belief, clearing a capture. The alignment is taken as
    /// exact, so accuracy is what the source of the target provides.
    pub fn coarse_align(&mut self, target_platform_to_inertial: DQuat) {
        self.reset_platform(target_platform_to_inertial);
        self.snapshot.output.estimated_platform_to_inertial = target_platform_to_inertial;
        self.refresh_indicated();
    }

    /// Cages the gimbals to zero, clearing a capture. The stable member then
    /// lines up with the navigation base wherever it happens to point; the
    /// computer's belief is untouched, so the indicated attitude stays wrong
    /// until a star alignment.
    pub fn cage_to_zero(&mut self) {
        self.reset_platform(self.snapshot.navbase_to_inertial);
        self.refresh_indicated();
    }

    /// Replaces the computer's belief with `measured_platform_to_inertial`,
    /// as a star alignment does. The stable member itself does not move.
    pub fn fine_align(&mut self, measured_platform_to_inertial: DQuat) {
        self.snapshot.output.estimated_platform_to_inertial = measured_platform_to_inertial;
        self.refresh_indicated();
    }

    /// Publishes the `GIMBAL LOCK` lamp at `at`.
    pub fn publish_lamp(
        &self,
        at: SimInstant,
        port: &mut CrewStationPort,
    ) -> Result<(), CrewPortError> {
        let output = &self.snapshot.output;
        port.publish(
            at,
            CrewObservation::AnnunciatorLamp {
                lamp_id: GIMBAL_LOCK_LAMP,
                lit: output.gimbal_lock_warning || output.gimbal_locked,
            },
        )
    }

    fn reset_platform(&mut self, platform_to_inertial: DQuat) {
        let output = &mut self.snapshot.output;
        output.platform_to_inertial = platform_to_inertial;
        output.gimbal_locked = false;
        output.platform_valid = true;
        self.apply_gimbals(self.snapshot.navbase_to_inertial);
    }

    /// Solves the gimbals against the base, with the warning and encoders.
    fn apply_gimbals(&mut self, navbase_to_inertial: DQuat) {
        let output = &mut self.snapshot.output;
        let navbase_to_platform =
            (output.platform_to_inertial.inverse() * navbase_to_inertial).normalize();
        let angles = GimbalAngles::of_navbase_to_platform(navbase_to_platform);
        let previous = output.cdu_counts;
        let counts = angles.cdu_counts();
        output.oga_rad = angles.oga_rad;
        output.mga_rad = angles.mga_rad;
        output.iga_rad = angles.iga_rad;
        output.navbase_to_platform = navbase_to_platform;
        output.gimbal_lock_warning = angles.mga_rad.abs() >= GIMBAL_LOCK_WARNING_DEG.to_radians();
        output.cdu_counts = counts;
        output.cdu_pulses = pulses(previous, counts);
        output.resolvers = resolvers(&angles);
    }

    fn refresh_indicated(&mut self) {
        let output = &mut self.snapshot.output;
        output.indicated_navbase_to_inertial =
            (output.estimated_platform_to_inertial * output.navbase_to_platform).normalize();
    }

    /// Moves the stable member over one period: gyro-held with drift, or
    /// dragged by the base and tumbling off it once captured.
    fn advance_platform(&mut self, navbase_to_inertial: DQuat, dt_s: f64) {
        let config = &self.config;
        let output = &mut self.snapshot.output;
        output.platform_to_inertial = if output.gimbal_locked {
            let fall = rotation_over(config.capture_tumble_rate_rad_s, dt_s);
            (navbase_to_inertial * output.navbase_to_platform.inverse() * fall).normalize()
        } else {
            (output.platform_to_inertial * rotation_over(config.gyro_drift_rad_s, dt_s)).normalize()
        };
    }

    fn integrate_pipas(&mut self, specific_force_navbase: DVec3, dt_s: f64) {
        let scale = self.config.pipa_m_s_per_pulse;
        let snapshot = &mut self.snapshot;
        let sensed = snapshot.output.navbase_to_platform * specific_force_navbase;
        let total = snapshot.pipa_residual_m_s + sensed * dt_s;
        let whole = (total / scale).trunc();
        snapshot.pipa_residual_m_s = total - whole * scale;
        let counts = I64Vec3::from_array(snapshot.output.pipa_counts) + whole.as_i64vec3();
        snapshot.output.pipa_counts = counts.to_array();
    }
}

fn pulses(previous: [u16; 3], current: [u16; 3]) -> [DigitalBusFrame; 3] {
    let [px, py, pz] = previous;
    let [cx, cy, cz] = current;
    let frame = |axis, from: u16, to: u16| {
        // The counter wraps, so the shorter way round is the motion.
        let half = CDU_COUNTS_PER_TURN / 2;
        let delta = (i32::from(to) - i32::from(from) + half).rem_euclid(CDU_COUNTS_PER_TURN) - half;
        DigitalBusFrame::AgcCduPulses {
            axis,
            delta_pulses: delta as i16,
        }
    };
    [frame(0, px, cx), frame(1, py, cy), frame(2, pz, cz)]
}

fn resolvers(angles: &GimbalAngles) -> [ElectricalPinState; 3] {
    let winding = |angle| ElectricalPinState::from_angle_rad(RESOLVER_EXCITATION_V, angle);
    [
        winding(angles.oga_rad),
        winding(angles.iga_rad),
        winding(angles.mga_rad),
    ]
}

impl Block for ThreeGimbalImuBlock {
    type Input = InertialSiteSample;
    type Output = ImuOutput;
    type Snapshot = ImuSnapshot;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &InertialSiteSample) -> Result<(), BlockError> {
        input.validate(BLOCK)?;
        let dt_s = self.config.period.as_seconds();
        self.advance_platform(input.site_to_inertial, dt_s);
        self.apply_gimbals(input.site_to_inertial);
        self.snapshot.navbase_to_inertial = input.site_to_inertial;
        let output = &mut self.snapshot.output;
        if !output.gimbal_locked && output.mga_rad.abs() >= GIMBAL_LOCK_CAPTURE_DEG.to_radians() {
            output.gimbal_locked = true;
            output.platform_valid = false;
        }
        self.integrate_pipas(input.specific_force_site_m_s2, dt_s);
        self.refresh_indicated();
        Ok(())
    }

    fn output(&self) -> ImuOutput {
        self.snapshot.output
    }

    fn snapshot(&self) -> ImuSnapshot {
        self.snapshot
    }

    fn restore(&mut self, snapshot: &ImuSnapshot) {
        self.snapshot = *snapshot;
    }
}
