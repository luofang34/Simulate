use crate::{
    AIRSPEED_ENVELOPE_ASSET, CALIBRATION_SUITE_ASSET, LIVERY_CATALOG_ASSET, LoadedSyspack,
    MemoryPackageSource, PITOT_STATIC_CALIBRATION_ASSET, PackError, STRUCTURAL_LANDMARKS_ASSET,
};

#[cfg(test)]
mod tests;

const C172_MANIFEST: &[u8] = include_bytes!("../../../vehicles/c172/syspack.json");
const C172_FDM_PATH: &str = "fdm/c172_class_reference.xml";
const C172_FDM: &[u8] = include_bytes!("../../../vehicles/c172/fdm/c172_class_reference.xml");
const C172_ENVELOPE: &[u8] =
    include_bytes!("../../../vehicles/c172/performance/airspeed_envelope.json");
const C172_PEC: &[u8] =
    include_bytes!("../../../vehicles/c172/performance/pitot_static_calibration.json");
const C172_BENCHMARKS: &[u8] =
    include_bytes!("../../../vehicles/c172/performance/poh_benchmarks.json");
const C172_LANDMARKS: &[u8] = include_bytes!("../../../vehicles/c172/geometry/landmarks.json");
const P38_MANIFEST: &[u8] = include_bytes!("../../../vehicles/p38g/syspack.json");
const P38_FDM_PATH: &str = "fdm/p38g.xml";
const P38_FDM: &[u8] = include_bytes!("../../../vehicles/p38g/fdm/p38g.xml");
const P38_ENVELOPE: &[u8] =
    include_bytes!("../../../vehicles/p38g/performance/airspeed_envelope.json");
const P38_BENCHMARKS: &[u8] =
    include_bytes!("../../../vehicles/p38g/performance/poh_benchmarks.json");
const P38_PEC: &[u8] =
    include_bytes!("../../../vehicles/p38g/performance/pitot_static_calibration.json");
const P38_LANDMARKS: &[u8] = include_bytes!("../../../vehicles/p38g/geometry/landmarks.json");
const P38_FG_SET_PATH: &str = "p38g-set.xml";
const P38_FG_SET: &[u8] = include_bytes!("../../../vehicles/p38g/p38g-set.xml");
const P38_LIVERY_CATALOG: &[u8] = include_bytes!("../../../vehicles/p38g/Liveries/catalog.json");
const P38_LIVERY_MISS_VIRGINIA_PATH: &str = "Models/Liveries/miss-virginia.xml";
const P38_LIVERY_MISS_VIRGINIA: &[u8] =
    include_bytes!("../../../vehicles/p38g/Models/Liveries/miss-virginia.xml");
const P38_LIVERY_BARE_METAL_PATH: &str = "Models/Liveries/bare-metal.xml";
const P38_LIVERY_BARE_METAL: &[u8] =
    include_bytes!("../../../vehicles/p38g/Models/Liveries/bare-metal.xml");
const P38_LIVERY_GLACIER_GIRL_PATH: &str = "Models/Liveries/glacier-girl.xml";
const P38_LIVERY_GLACIER_GIRL: &[u8] =
    include_bytes!("../../../vehicles/p38g/Models/Liveries/glacier-girl.xml");

/// The C172-class aeroplane's package files, embedded in the binary.
pub fn builtin_c172_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(
        C172_MANIFEST,
        &[
            (C172_FDM_PATH, C172_FDM),
            (AIRSPEED_ENVELOPE_ASSET, C172_ENVELOPE),
            (PITOT_STATIC_CALIBRATION_ASSET, C172_PEC),
            (CALIBRATION_SUITE_ASSET, C172_BENCHMARKS),
            (STRUCTURAL_LANDMARKS_ASSET, C172_LANDMARKS),
        ],
    )
}

/// The built-in C172-class aeroplane, loaded and verified like any other
/// package.
pub fn builtin_c172_package() -> Result<LoadedSyspack, PackError> {
    LoadedSyspack::load(&builtin_c172_source())
}

/// The Lockheed P-38G Lightning's package files, embedded in the binary.
#[must_use]
pub fn builtin_p38g_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(
        P38_MANIFEST,
        &[
            (P38_FDM_PATH, P38_FDM),
            (AIRSPEED_ENVELOPE_ASSET, P38_ENVELOPE),
            (CALIBRATION_SUITE_ASSET, P38_BENCHMARKS),
            (PITOT_STATIC_CALIBRATION_ASSET, P38_PEC),
            (STRUCTURAL_LANDMARKS_ASSET, P38_LANDMARKS),
            (P38_FG_SET_PATH, P38_FG_SET),
            (LIVERY_CATALOG_ASSET, P38_LIVERY_CATALOG),
            (P38_LIVERY_MISS_VIRGINIA_PATH, P38_LIVERY_MISS_VIRGINIA),
            (P38_LIVERY_BARE_METAL_PATH, P38_LIVERY_BARE_METAL),
            (P38_LIVERY_GLACIER_GIRL_PATH, P38_LIVERY_GLACIER_GIRL),
        ],
    )
}

/// The built-in Lockheed P-38G Lightning twin-boom fighter, loaded and verified
/// like any other package.
pub fn builtin_p38g_package() -> Result<LoadedSyspack, PackError> {
    LoadedSyspack::load(&builtin_p38g_source())
}
