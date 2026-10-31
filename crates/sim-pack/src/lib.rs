//! The simulator's side of system packages. A built-in vehicle and a
//! downloaded one are the same thing: a `syspack.json` manifest plus
//! SHA-256-verified asset files, verified by [`syspack`] and bound here by
//! [`LoadedSyspack`] to the blocks that fly them.
//!
//! Loading fails closed: anything [`syspack`] refuses, or a device that cannot
//! be built, is a typed [`PackError`], and nothing of the package is usable.

mod builtin;
mod error;
mod loaded;

pub use builtin::{
    builtin_c172_package, builtin_c172_source, builtin_light_trainer_package,
    builtin_light_trainer_source,
};
pub use error::PackError;
pub use loaded::{CotsDevice, LoadedSyspack, MountedCotsDevice, PackageCamera};
#[cfg(not(target_arch = "wasm32"))]
pub use syspack::DirectoryPackageSource;
pub use syspack::{
    AssetDigest, ComponentRecord, ControlsSpec, DeviceKindSpec, DynamicsSpec,
    INITIAL_COMPONENT_TEMPERATURE_K, MANIFEST_FILE, MemoryPackageSource, MountedDeviceSpec,
    PackageSource, SCHEMA_VERSION, SyspackError, SyspackManifest, SystemKind, VerifiedSyspack,
};
