use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree};
use sim_pack::{LoadedSyspack, MemoryPackageSource, PackError};

const LM_MANIFEST: &[u8] = include_bytes!("../../packages/eagle-lm/syspack.json");

/// The massless reference at the ascent-descent interface; never separates.
pub const LM_ROOT: ComponentId = ComponentId(0);
/// Ascent stage structure and crew cabin.
pub const LM_ASCENT_STRUCTURE: ComponentId = ComponentId(1);
/// Ascent Propulsion System propellant.
pub const LM_APS_TANK: ComponentId = ComponentId(2);
/// Reaction Control System propellant.
pub const LM_RCS_TANK: ComponentId = ComponentId(3);
/// Navigation base, the rigid mount of the guidance sensors.
pub const LM_NAV_BASE: ComponentId = ComponentId(4);
/// Primary PGNCS inertial measurement unit.
pub const LM_IMU: ComponentId = ComponentId(5);
/// Alignment Optical Telescope.
pub const LM_AOT: ComponentId = ComponentId(6);
/// Abort Sensor Assembly.
pub const LM_ASA: ComponentId = ComponentId(7);
/// Rate Gyro Assembly.
pub const LM_RGA: ComponentId = ComponentId(8);
/// Rendezvous radar antenna.
pub const LM_RENDEZVOUS_RADAR: ComponentId = ComponentId(9);
/// Commander's design eye at the left window.
pub const LM_CDR_EYE: ComponentId = ComponentId(10);
/// Descent stage structure; separating it takes everything below with it.
pub const LM_DESCENT_STRUCTURE: ComponentId = ComponentId(11);
/// Descent Propulsion System propellant.
pub const LM_DPS_TANK: ComponentId = ComponentId(12);
/// Landing radar antenna, on the descent stage.
pub const LM_LANDING_RADAR: ComponentId = ComponentId(13);
/// Footpad of the forward leg, which carries the ladder and no probe.
pub const LM_LEG_PLUS_Z: ComponentId = ComponentId(14);
/// Footpad of the aft leg.
pub const LM_LEG_MINUS_Z: ComponentId = ComponentId(15);
/// Footpad of the right leg.
pub const LM_LEG_PLUS_Y: ComponentId = ComponentId(16);
/// Footpad of the left leg.
pub const LM_LEG_MINUS_Y: ComponentId = ComponentId(17);
/// Tips of the three lunar contact probes, on the aft, right and left legs.
pub const LM_PROBE_TIPS: [ComponentId; 3] = [ComponentId(18), ComponentId(19), ComponentId(20)];

/// Lunar contact probe length below the footpad, metres.
pub const PROBE_LENGTH_M: f64 = 1.71;
/// Descent Propulsion System full thrust, N.
pub const LM_DPS_THRUST_N: f64 = 45_040.0;
/// DPS effective exhaust velocity, m/s: 311 s of specific impulse.
pub const LM_DPS_EXHAUST_VELOCITY_M_S: f64 = 3_050.0;
/// DPS gimbal point in LM axes, metres.
pub const LM_DPS_THRUST_POINT_M: DVec3 = DVec3::new(-2.4, 0.0, 0.0);

/// AOT sight frame to navigation base: the sight's `+z` shaft axis along LM
/// `+x`, and its zero-azimuth `+x` forward along LM `+z`.
pub fn aot_mount() -> DQuat {
    DQuat::from_mat3(&DMat3::from_cols(DVec3::Z, DVec3::NEG_Y, DVec3::X)).normalize()
}

/// The embedded files of the Eagle package.
pub fn apollo_lm_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(LM_MANIFEST, &[])
}

/// The Eagle Lunar Module package.
pub fn apollo_lm_package() -> Result<LoadedSyspack, PackError> {
    LoadedSyspack::load(&apollo_lm_source())
}

/// The Eagle Lunar Module as it left the CSM, fully fuelled.
///
/// Axes are Apollo's LM axes: `+x` up the thrust axis, `+y` to the crew's
/// right, `+z` forward out of the hatch. The root stays with the ascent stage,
/// so separating [`LM_DESCENT_STRUCTURE`] drops the descent propellant, the
/// gear, the probes and the landing radar together. Masses are the flown
/// Apollo 11 figures; station and inertia values are representative.
pub fn build_apollo_lm_tree() -> Result<ComponentTree, PackError> {
    Ok(apollo_lm_package()?.tree().clone())
}
