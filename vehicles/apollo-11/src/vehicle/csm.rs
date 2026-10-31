use glam::DVec3;
use mbd_core::{BluntBodyCapsuleAero, ComponentId, ComponentTree};
use sim_pack::{LoadedSyspack, MemoryPackageSource, PackError, SyspackError};

const CSM_MANIFEST: &[u8] = include_bytes!("../../packages/columbia-csm/syspack.json");

/// The massless reference at the heat-shield apex; never separates.
pub const CM_ROOT: ComponentId = ComponentId(0);
/// Command Module capsule, carrying its offset centre of gravity.
pub const CM_STRUCTURE: ComponentId = ComponentId(1);
/// Navigation base, the rigid mount of the guidance sensors.
pub const CM_NAV_BASE: ComponentId = ComponentId(2);
/// Primary PGNCS inertial measurement unit.
pub const CM_IMU: ComponentId = ComponentId(3);
/// Body-mounted attitude gyros.
pub const CM_BMAG: ComponentId = ComponentId(4);
/// Sextant and Scanning Telescope.
pub const CM_OPTICS: ComponentId = ComponentId(5);
/// Entry Monitor System accelerometer.
pub const CM_EMS: ComponentId = ComponentId(6);
/// Service Module; separating it takes its propellant with it.
pub const CSM_SERVICE_MODULE: ComponentId = ComponentId(7);
/// Service Propulsion System propellant.
pub const CSM_SPS_TANK: ComponentId = ComponentId(8);

/// Service Propulsion System thrust, N.
pub const SM_SPS_THRUST_N: f64 = 91_190.0;

/// The embedded files of the Columbia package.
pub fn apollo_csm_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(CSM_MANIFEST, &[])
}

/// The Columbia Command and Service Module package.
pub fn apollo_csm_package() -> Result<LoadedSyspack, PackError> {
    LoadedSyspack::load(&apollo_csm_source())
}

/// The Columbia Command and Service Module, fully fuelled.
///
/// Axes are those of [`BluntBodyCapsuleAero`]: the origin is the heat-shield
/// apex and `+x` runs out through the shield, the side leading during entry,
/// which is opposite to Apollo's own CM `+x`. The root stays with the capsule,
/// so separating [`CSM_SERVICE_MODULE`] leaves the entry vehicle. Masses are
/// the flown Apollo 11 figures; station and inertia values are representative.
pub fn build_apollo_csm_tree() -> Result<ComponentTree, PackError> {
    Ok(apollo_csm_package()?.tree().clone())
}

/// The capsule aerodynamics for [`build_apollo_csm_tree`]'s axes: the moment
/// reference is the heat shield's centre of curvature. The normal-force slope
/// is chosen with the capsule's offset centre of gravity to trim near 20° with
/// a lift-to-drag ratio of 0.3.
pub fn apollo_cm_aero() -> Result<BluntBodyCapsuleAero, PackError> {
    Ok(apollo_csm_package()?.capsule_aero()?)
}

/// Incidence `atan2(z, x)` of the air velocity in body axes at which the
/// capsule alone trims: the force through the moment reference then passes
/// through the centre of gravity, whose `z` offset off the symmetry axis is
/// what makes the capsule fly at incidence with lift.
pub fn cm_analytic_trim_incidence_rad() -> Result<f64, PackError> {
    let package = apollo_csm_package()?;
    let aero = package.capsule_aero()?;
    let params = aero.params();
    let cg_m = package
        .tree()
        .spec(CM_STRUCTURE)
        .map_err(SyspackError::InvalidTree)?
        .placement
        .translation_m;
    let arm: DVec3 = params.reference_point_body_m - cg_m;
    Ok((params.axial_drag_coeff / params.normal_force_slope * arm.z / arm.x).atan())
}
