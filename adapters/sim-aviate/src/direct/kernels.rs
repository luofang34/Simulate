use aviate_airframe_fixed_wing::FixedWingAirframe;
use aviate_core::control::ConfigMode;
use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::ekf::Ekf;
use aviate_core::kernel::DefaultAviateKernel;
use aviate_core::kernel::builder::AviateKernelBuilder;
use aviate_core::kernel::config::{ActuatorCurveKind, MixerGeometry, ResolvedKernelConfig};
use aviate_core::mixer::fixed_wing::FixedWingMixer;
use aviate_core::mixer::{ModeConfig, QuadXMixerX500, Sanitizer};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::NormalizedThrust;
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

/// Concrete `DefaultAviateKernel` type produced by [`build_direct_fixed_wing_kernel`].
pub type DirectFixedWingKernel = DefaultAviateKernel<FixedWingController, FixedWingMixer>;

/// Builds an in-process Aviate fixed-wing kernel for `airframe` as the airframe
/// assembles it: its estimator with air-data aiding, the fixed-wing control law,
/// the four-lane mixer (throttle, aileron, elevator, rudder; surfaces neutral at
/// 0.5) and the fixed-wing pre-arm checks.
pub fn build_direct_fixed_wing_kernel(
    airframe: &FixedWingAirframe,
) -> Option<DirectFixedWingKernel> {
    let mut kernel = airframe.kernel(zero_internal_timestamp).ok()?;
    kernel.state.checks.pre_arm.update_throttle(true);
    Some(kernel)
}

/// Decodes a fixed-wing [`DirectActuatorCmd`] into pilot-sense
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
