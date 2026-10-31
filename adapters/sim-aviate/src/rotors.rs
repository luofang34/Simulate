use aviate_xil_direct::DirectActuatorCmd;
use glam::DVec3;
use sim_core::{ControlInputs, ThrusterWrench};

use crate::{AviateBridgeError, FlightControllerCommand};

#[cfg(test)]
mod tests;

/// X500 rotor constants, from the PX4 model the Aviate SITL flies.
const X500_ARM_M: f64 = 0.174;
const X500_ROTOR_HEIGHT_M: f64 = 0.06;
const X500_MOTOR_CONSTANT: f64 = 8.54858e-6;
const X500_MOMENT_CONSTANT_M: f64 = 0.016;
const X500_MAX_SPEED_RAD_S: f64 = 1_000.0;

/// Turning sense of a rotor seen from above.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RotorSpin {
    /// Counter-clockwise: the reaction yaws the airframe right.
    CounterClockwise,
    /// Clockwise: the reaction yaws the airframe left.
    Clockwise,
}

impl RotorSpin {
    /// Sign of the reaction moment about body `+z`.
    fn reaction_sign(self) -> f64 {
        match self {
            Self::CounterClockwise => 1.0,
            Self::Clockwise => -1.0,
        }
    }
}

/// One rotor, in the terms of the Gazebo model behind the contract: thrust
/// `motor_constant · ω²` along body `-z`, reaction moment `moment_constant_m`
/// times that thrust. The rotor answers its command instantly.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rotor {
    /// Hub position in the component tree's root axes, metres.
    pub position_body_m: DVec3,
    /// Turning sense.
    pub spin: RotorSpin,
    /// Thrust per squared rotor speed, N/(rad/s)².
    pub motor_constant: f64,
    /// Reaction moment per newton of thrust, metres.
    pub moment_constant_m: f64,
    /// Highest rotor speed, rad/s. Commands above it saturate; reversed rotors
    /// are not modelled, so commands below zero stop the rotor.
    pub max_speed_rad_s: f64,
}

impl Rotor {
    fn normalised_thrust(&self, speed_rad_s: f64) -> f64 {
        (speed_rad_s.clamp(0.0, self.max_speed_rad_s) / self.max_speed_rad_s).powi(2)
    }

    fn wrench(&self, speed_rad_s: f64) -> ThrusterWrench {
        let max_thrust_n = self.motor_constant * self.max_speed_rad_s.powi(2);
        let thrust_n = max_thrust_n * self.normalised_thrust(speed_rad_s);
        let reaction_nm = self.spin.reaction_sign() * self.moment_constant_m * thrust_n;
        ThrusterWrench {
            application_point_body_m: self.position_body_m,
            force_body_n: DVec3::new(0.0, 0.0, -thrust_n),
            moment_body_nm: DVec3::new(0.0, 0.0, reaction_nm),
        }
    }
}

/// The rotors a flight controller's lanes drive, in lane order. The contract
/// carries rotor speeds only, so anything else on the vehicle (control
/// surfaces included) has no command to follow.
#[derive(Debug, Clone, PartialEq)]
pub struct RotorLayout {
    rotors: Vec<Rotor>,
}

impl RotorLayout {
    /// Rotors in lane order.
    pub fn new(rotors: Vec<Rotor>) -> Self {
        Self { rotors }
    }

    /// The X500 quadrotor in the Aviate SITL's lane order: front right, rear
    /// left, front left, rear right.
    pub fn x500() -> Self {
        let rotor = |forward: f64, right: f64, spin| Rotor {
            position_body_m: DVec3::new(
                forward * X500_ARM_M,
                right * X500_ARM_M,
                -X500_ROTOR_HEIGHT_M,
            ),
            spin,
            motor_constant: X500_MOTOR_CONSTANT,
            moment_constant_m: X500_MOMENT_CONSTANT_M,
            max_speed_rad_s: X500_MAX_SPEED_RAD_S,
        };
        Self::new(vec![
            rotor(1.0, 1.0, RotorSpin::CounterClockwise),
            rotor(-1.0, -1.0, RotorSpin::CounterClockwise),
            rotor(1.0, -1.0, RotorSpin::Clockwise),
            rotor(-1.0, 1.0, RotorSpin::Clockwise),
        ])
    }

    /// The rotors, in lane order.
    pub fn rotors(&self) -> &[Rotor] {
        &self.rotors
    }

    /// One wrench per rotor for `command`. A flight controller not
    /// [`ready`](FlightControllerCommand::fc_ready) leaves the rotors stopped,
    /// whatever its lanes hold.
    pub fn thrusters(
        &self,
        command: &FlightControllerCommand,
    ) -> Result<Vec<ThrusterWrench>, AviateBridgeError> {
        let speeds = self.effective_speeds(command)?;
        Ok(self
            .rotors
            .iter()
            .zip(speeds)
            .map(|(rotor, speed)| rotor.wrench(speed))
            .collect())
    }

    /// One wrench per rotor for a Tier-0 [`DirectActuatorCmd`] whose active
    /// `outputs` carry force-normalised thrust fractions in `[0.0, 1.0]`.
    pub fn direct_thrusters(
        &self,
        command: &DirectActuatorCmd,
    ) -> Result<Vec<ThrusterWrench>, AviateBridgeError> {
        if !command.armed {
            return Ok(self.rotors.iter().map(|rotor| rotor.wrench(0.0)).collect());
        }
        let active = usize::from(command.count);
        if active != self.rotors.len() {
            return Err(AviateBridgeError::RotorCountMismatch {
                commanded: active,
                fitted: self.rotors.len(),
            });
        }
        let mut out = Vec::with_capacity(active);
        for (idx, rotor) in self.rotors.iter().enumerate() {
            let norm_thrust =
                f64::from(command.outputs.get(idx).copied().unwrap_or(0.0)).clamp(0.0, 1.0);
            let speed_rad_s = rotor.max_speed_rad_s * norm_thrust.sqrt();
            out.push(rotor.wrench(speed_rad_s));
        }
        Ok(out)
    }

    /// The pilot inputs that would produce the rotor spread in `command`, for
    /// a vehicle flown through [`ControlInputs`].
    ///
    /// The contract has no such inputs, so these are derived: throttle is mean
    /// normalised thrust, and each axis is the signed spread over the layout's
    /// authority about it, so one side at full and the other stopped reads
    /// `±1`.
    pub fn control_inputs(
        &self,
        command: &FlightControllerCommand,
    ) -> Result<ControlInputs, AviateBridgeError> {
        let speeds = self.effective_speeds(command)?;
        let (mut sum, mut roll, mut pitch, mut yaw) = (0.0, 0.0, 0.0, 0.0);
        let (mut roll_arm, mut pitch_arm) = (0.0, 0.0);
        for (rotor, speed) in self.rotors.iter().zip(speeds) {
            let thrust = rotor.normalised_thrust(speed);
            let position = rotor.position_body_m;
            sum += thrust;
            roll -= position.y * thrust;
            pitch += position.x * thrust;
            yaw += rotor.spin.reaction_sign() * thrust;
            roll_arm += position.y.abs();
            pitch_arm += position.x.abs();
        }
        let count = self.rotors.len() as f64;
        let spread = |value: f64, authority: f64| {
            if authority > 0.0 {
                2.0 * value / authority
            } else {
                0.0
            }
        };
        Ok(ControlInputs {
            roll: spread(roll, roll_arm),
            pitch: spread(pitch, pitch_arm),
            yaw: spread(yaw, count),
            throttle: spread(sum, 2.0 * count),
        }
        .clamped())
    }

    fn effective_speeds(
        &self,
        command: &FlightControllerCommand,
    ) -> Result<Vec<f64>, AviateBridgeError> {
        if !command.fc_ready {
            return Ok(vec![0.0; self.rotors.len()]);
        }
        let speeds = command.motors.speeds_rad_s();
        if speeds.len() != self.rotors.len() {
            return Err(AviateBridgeError::RotorCountMismatch {
                commanded: speeds.len(),
                fitted: self.rotors.len(),
            });
        }
        Ok(speeds.to_vec())
    }
}
