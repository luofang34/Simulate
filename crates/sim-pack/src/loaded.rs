use std::ops::Deref;

use jsbsim::MemoryResolver;
use syspack::{PackageSource, VerifiedSyspack};

use crate::PackError;

mod devices;

pub use devices::{CotsDevice, MountedCotsDevice, PackageCamera};

#[cfg(test)]
mod tests;

/// A verified system package bound to this simulator: everything a
/// [`VerifiedSyspack`] holds, reached through `Deref`, plus the blocks built
/// from it. Holding one proves the devices and cameras can be built too.
#[derive(Debug, Clone, PartialEq)]
pub struct LoadedSyspack {
    verified: VerifiedSyspack,
}

impl Deref for LoadedSyspack {
    type Target = VerifiedSyspack;

    fn deref(&self) -> &VerifiedSyspack {
        &self.verified
    }
}

impl LoadedSyspack {
    /// Reads and verifies the package in `source`, then builds its devices,
    /// failing on the first problem found.
    pub fn load(source: &dyn PackageSource) -> Result<Self, PackError> {
        let package = Self {
            verified: VerifiedSyspack::load(source)?,
        };
        package.instantiate_cots_devices()?;
        package.camera_mounts()?;
        Ok(package)
    }

    /// The aircraft directory and model name a [`sim_core::JsbsimFdmConfig`]
    /// needs.
    pub fn jsbsim_model(&self) -> Result<(&str, &str), PackError> {
        Ok(self.verified.jsbsim_model()?)
    }

    /// The verified aircraft files at their JSBSim virtual paths, so a
    /// [`sim_core::JsbsimFdmBlock`] flies the package with no file system.
    pub fn build_jsbsim_resolver(&self) -> Result<MemoryResolver, PackError> {
        let mut files = MemoryResolver::new();
        for (path, bytes) in self.jsbsim_files()? {
            files.insert(path, bytes);
        }
        Ok(files)
    }
}
