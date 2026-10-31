use crate::{LoadedSyspack, MemoryPackageSource, PackError};

#[cfg(test)]
mod tests;

const TRAINER_MANIFEST: &[u8] = include_bytes!("../../../vehicles/light-trainer/syspack.json");
const TRAINER_FDM_PATH: &str = "fdm/trainer.xml";
const TRAINER_FDM: &[u8] = include_bytes!("../../../vehicles/light-trainer/fdm/trainer.xml");
const C172_MANIFEST: &[u8] = include_bytes!("../../../vehicles/c172/syspack.json");
const C172_FDM_PATH: &str = "fdm/c172_class_reference.xml";
const C172_FDM: &[u8] = include_bytes!("../../../vehicles/c172/fdm/c172_class_reference.xml");

/// The light trainer's package files, embedded in the binary. A test can
/// tamper with these exactly as with a download.
pub fn builtin_light_trainer_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(TRAINER_MANIFEST, &[(TRAINER_FDM_PATH, TRAINER_FDM)])
}

/// The built-in light trainer, loaded and verified like any other package.
pub fn builtin_light_trainer_package() -> Result<LoadedSyspack, PackError> {
    LoadedSyspack::load(&builtin_light_trainer_source())
}

/// The C172-class aeroplane's package files, embedded in the binary.
pub fn builtin_c172_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(C172_MANIFEST, &[(C172_FDM_PATH, C172_FDM)])
}

/// The built-in C172-class aeroplane, loaded and verified like any other
/// package.
pub fn builtin_c172_package() -> Result<LoadedSyspack, PackError> {
    LoadedSyspack::load(&builtin_c172_source())
}
