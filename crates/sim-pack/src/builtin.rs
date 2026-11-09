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
const G4M1_MANIFEST: &[u8] = include_bytes!("../../../vehicles/g4m1/syspack.json");
const G4M1_KASEI15_MANIFEST: &[u8] = include_bytes!("../../../vehicles/g4m1/variants/kasei15.json");
/// The assets every G4M1 variant shares; each variant manifest declares the
/// ones it uses, and the loader verifies only those.
const G4M1_ASSETS: &[(&str, &[u8])] = &[
    (
        "fdm/g4m1.xml",
        include_bytes!("../../../vehicles/g4m1/fdm/g4m1.xml"),
    ),
    (
        "design/airframes.json",
        include_bytes!("../../../vehicles/g4m1/design/airframes.json"),
    ),
    (
        "design/configuration/kasei11.json",
        include_bytes!("../../../vehicles/g4m1/design/configuration/kasei11.json"),
    ),
    (
        "design/configuration/kasei15.json",
        include_bytes!("../../../vehicles/g4m1/design/configuration/kasei15.json"),
    ),
    (
        "performance/kasei11/poh_benchmarks.json",
        include_bytes!("../../../vehicles/g4m1/performance/kasei11/poh_benchmarks.json"),
    ),
    (
        "performance/kasei15/poh_benchmarks.json",
        include_bytes!("../../../vehicles/g4m1/performance/kasei15/poh_benchmarks.json"),
    ),
    (
        "design/sources.json",
        include_bytes!("../../../vehicles/g4m1/design/sources.json"),
    ),
    (
        "performance/kasei11/validation.json",
        include_bytes!("../../../vehicles/g4m1/performance/kasei11/validation.json"),
    ),
    (
        "performance/kasei15/validation.json",
        include_bytes!("../../../vehicles/g4m1/performance/kasei15/validation.json"),
    ),
];

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

/// The Mitsubishi G4M1 Model 11 with Kasei 11 engines (the calibrated reference
/// variant), as package files: its manifest and the airframe's shared assets.
#[must_use]
pub fn builtin_g4m1_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(G4M1_MANIFEST, G4M1_ASSETS)
}

/// The built-in G4M1 Model 11 with Kasei 11 engines, graded against the Navy
/// test of aircraft No. 1.
pub fn builtin_g4m1_package() -> Result<LoadedSyspack, PackError> {
    LoadedSyspack::load(&builtin_g4m1_source())
}

/// The G4M1 Model 11 with Kasei 15 engines: the same airframe pinning the
/// Kasei 15 engine part, as package files.
#[must_use]
pub fn builtin_g4m1_kasei15_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(G4M1_KASEI15_MANIFEST, G4M1_ASSETS)
}

/// The built-in G4M1 Model 11 with Kasei 15 engines. Its performance is the
/// Kasei 11 airframe's model flown with the other engine, checked against the
/// Navy test of Kasei 15 aircraft No. 241, to which nothing was fitted.
pub fn builtin_g4m1_kasei15_package() -> Result<LoadedSyspack, PackError> {
    LoadedSyspack::load(&builtin_g4m1_kasei15_source())
}
