use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::{AxisCommand, ConfigMode};
use aviate_core::ekf::Ekf;
use aviate_core::kernel::DefaultAviateKernel;
use aviate_core::kernel::builder::AviateKernelBuilder;
use aviate_core::kernel::config::{ActuatorCurveKind, MixerGeometry, ResolvedKernelConfig};
use aviate_core::mixer::{
    ActuatorCmd, MAX_ACTUATORS, Mixer, ModeConfig, QuadXMixerX500, Sanitizer,
};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{Normalized, NormalizedThrust};
use aviate_xil_direct::DirectActuatorCmd;
use sim_core::ControlInputs;

fn zero_internal_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

/// Concrete `DefaultAviateKernel` type produced by [`build_direct_x500_kernel`].
pub type DirectX500Kernel = DefaultAviateKernel<MultirotorController, QuadXMixerX500>;

/// Builds a `wasm32`-clean, `no_std`-compatible X500 quadrotor Aviate kernel
/// (`Ekf` + `MultirotorController` + `QuadXMixerX500`) matching the canonical
/// X500 hover trim (`0.5929` force-domain normalized thrust) without pulling OS
/// runtime dependencies.
pub fn build_direct_x500_kernel() -> Option<DirectX500Kernel> {
    let gains = CascadeGains::x500_defaults();
    let hover = NormalizedThrust(0.5929);
    let cfg = ResolvedKernelConfig {
        cascade_gains: gains,
        hover_thrust_norm: hover,
        mixer_geometry: MixerGeometry::QuadXX500,
        actuator_curve: ActuatorCurveKind::QuadraticRotor,
        mode_config: ModeConfig {
            mode: ConfigMode::Hover,
            groups: &[],
        },
        ..ResolvedKernelConfig::default()
    };
    let mut kernel = AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(MultirotorController::from_gains(gains, hover.0))
        .mixer(QuadXMixerX500 {
            timestamp_source: zero_internal_timestamp,
        })
        .sanitizer(Sanitizer)
        .config(cfg)
        .build()
        .ok()?;
    kernel.state.checks.pre_arm.update_throttle(true);
    Some(kernel)
}

/// Four-channel fixed-wing pass-through mixer encoding `[throttle, roll, pitch, yaw]`
/// onto `outputs[0..4]` in `[0.0, 1.0]`.
#[derive(Debug, Clone, Copy, Default)]
pub struct PassthroughFixedWingMixer;

impl Mixer for PassthroughFixedWingMixer {
    const ALGORITHM_ID: u64 = 0x5349_4D5F_4657_4D58;
    const GEOMETRY: MixerGeometry = MixerGeometry::QuadX;

    fn mix(&self, axis: &AxisCommand) -> ActuatorCmd {
        let mut outputs = [Normalized(0.0); MAX_ACTUATORS];
        if let Some(s) = outputs.get_mut(0) {
            *s = Normalized(axis.collective.0.clamp(0.0, 1.0));
        }
        if let Some(s) = outputs.get_mut(1) {
            *s = Normalized((0.5 + 0.5 * axis.roll.0).clamp(0.0, 1.0));
        }
        if let Some(s) = outputs.get_mut(2) {
            *s = Normalized((0.5 + 0.5 * axis.pitch.0).clamp(0.0, 1.0));
        }
        if let Some(s) = outputs.get_mut(3) {
            *s = Normalized((0.5 + 0.5 * axis.yaw.0).clamp(0.0, 1.0));
        }
        ActuatorCmd {
            outputs,
            active_mask: 0b1111,
            sequence: 0,
            timestamp: zero_internal_timestamp(),
            fallback_mask: 0,
            sanitized: false,
        }
    }
}

/// Concrete `DefaultAviateKernel` type produced by [`build_direct_fixed_wing_kernel`].
pub type DirectFixedWingKernel =
    DefaultAviateKernel<FixedWingController, PassthroughFixedWingMixer>;

/// Builds an in-process Aviate fixed-wing kernel (`Ekf` + `FixedWingController` +
/// [`PassthroughFixedWingMixer`]) for closed-loop fixed-wing simulation.
pub fn build_direct_fixed_wing_kernel() -> Option<DirectFixedWingKernel> {
    let cfg = ResolvedKernelConfig {
        mixer_geometry: MixerGeometry::QuadX,
        mode_config: ModeConfig {
            mode: ConfigMode::Cruise,
            groups: &[],
        },
        ..ResolvedKernelConfig::default()
    };
    let mut kernel = AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(FixedWingController)
        .mixer(PassthroughFixedWingMixer)
        .sanitizer(Sanitizer)
        .config(cfg)
        .build()
        .ok()?;
    kernel.state.checks.pre_arm.update_throttle(true);
    Some(kernel)
}

/// Decodes a [`PassthroughFixedWingMixer`] [`DirectActuatorCmd`] into pilot-sense
/// [`ControlInputs`]. Disarmed commands hold neutral surfaces and zero throttle.
#[must_use]
pub fn direct_fixed_wing_controls(cmd: &DirectActuatorCmd) -> ControlInputs {
    if !cmd.armed {
        return ControlInputs::default();
    }
    let ch = |idx: usize, neutral: f32| f64::from(cmd.outputs.get(idx).copied().unwrap_or(neutral));
    ControlInputs {
        throttle: ch(0, 0.0),
        roll: 2.0 * ch(1, 0.5) - 1.0,
        pitch: 2.0 * ch(2, 0.5) - 1.0,
        yaw: 2.0 * ch(3, 0.5) - 1.0,
    }
    .clamped()
}
