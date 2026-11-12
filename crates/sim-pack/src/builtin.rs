use crate::{
    AIRSPEED_ENVELOPE_ASSET, CALIBRATION_SUITE_ASSET, CAMERA_VIEWS_ASSET, LIVERY_CATALOG_ASSET,
    LoadedSyspack, MemoryPackageSource, PITOT_STATIC_CALIBRATION_ASSET, PackError,
    STRUCTURAL_LANDMARKS_ASSET,
};

#[cfg(test)]
mod tests;

const C172S_MANIFEST: &[u8] = include_bytes!("../../../vehicles/c172s/syspack.json");
const C172S_ASSETS: &[(&str, &[u8])] = &[
    (
        LIVERY_CATALOG_ASSET,
        include_bytes!("../../../vehicles/c172s/Liveries/catalog.json"),
    ),
    (
        "Models/Liveries/n172sp.xml",
        include_bytes!("../../../vehicles/c172s/Models/Liveries/n172sp.xml"),
    ),
    (
        "Models/Liveries/n642sp.xml",
        include_bytes!("../../../vehicles/c172s/Models/Liveries/n642sp.xml"),
    ),
    (
        "c172s-set.xml",
        include_bytes!("../../../vehicles/c172s/c172s-set.xml"),
    ),
    (
        "fdm/c172s.xml",
        include_bytes!("../../../vehicles/c172s/fdm/c172s.xml"),
    ),
    (
        STRUCTURAL_LANDMARKS_ASSET,
        include_bytes!("../../../vehicles/c172s/geometry/landmarks.json"),
    ),
    (
        AIRSPEED_ENVELOPE_ASSET,
        include_bytes!("../../../vehicles/c172s/performance/airspeed_envelope.json"),
    ),
    (
        PITOT_STATIC_CALIBRATION_ASSET,
        include_bytes!("../../../vehicles/c172s/performance/pitot_static_calibration.json"),
    ),
    (
        CALIBRATION_SUITE_ASSET,
        include_bytes!("../../../vehicles/c172s/performance/poh_benchmarks.json"),
    ),
    (
        "usd/c172.usdc",
        include_bytes!("../../../vehicles/c172s/usd/c172.usdc"),
    ),
    (
        "usd/c172s_airframe.usda",
        include_bytes!("../../../vehicles/c172s/usd/c172s_airframe.usda"),
    ),
    (
        "usd/n172sp.usda",
        include_bytes!("../../../vehicles/c172s/usd/n172sp.usda"),
    ),
    (
        "usd/n172sp_livery.json",
        include_bytes!("../../../vehicles/c172s/usd/n172sp_livery.json"),
    ),
    (
        "usd/n172sp_livery.usda",
        include_bytes!("../../../vehicles/c172s/usd/n172sp_livery.usda"),
    ),
    (
        "usd/n642sp.usda",
        include_bytes!("../../../vehicles/c172s/usd/n642sp.usda"),
    ),
    (
        "usd/n642sp_livery.json",
        include_bytes!("../../../vehicles/c172s/usd/n642sp_livery.json"),
    ),
    (
        "usd/n642sp_livery.usda",
        include_bytes!("../../../vehicles/c172s/usd/n642sp_livery.usda"),
    ),
    (
        CAMERA_VIEWS_ASSET,
        include_bytes!("../../../vehicles/c172s/views/cameras.json"),
    ),
];

const C172P_MANIFEST: &[u8] = include_bytes!("../../../vehicles/c172p/syspack.json");
const C172P_ASSETS: &[(&str, &[u8])] = &[
    (
        LIVERY_CATALOG_ASSET,
        include_bytes!("../../../vehicles/c172p/Liveries/catalog.json"),
    ),
    (
        "Models/Liveries/d-ecjb.xml",
        include_bytes!("../../../vehicles/c172p/Models/Liveries/d-ecjb.xml"),
    ),
    (
        "Models/Liveries/n53295.xml",
        include_bytes!("../../../vehicles/c172p/Models/Liveries/n53295.xml"),
    ),
    (
        "c172p-set.xml",
        include_bytes!("../../../vehicles/c172p/c172p-set.xml"),
    ),
    (
        "fdm/c172p.xml",
        include_bytes!("../../../vehicles/c172p/fdm/c172p.xml"),
    ),
    (
        STRUCTURAL_LANDMARKS_ASSET,
        include_bytes!("../../../vehicles/c172p/geometry/landmarks.json"),
    ),
    (
        AIRSPEED_ENVELOPE_ASSET,
        include_bytes!("../../../vehicles/c172p/performance/airspeed_envelope.json"),
    ),
    (
        PITOT_STATIC_CALIBRATION_ASSET,
        include_bytes!("../../../vehicles/c172p/performance/pitot_static_calibration.json"),
    ),
    (
        CALIBRATION_SUITE_ASSET,
        include_bytes!("../../../vehicles/c172p/performance/poh_benchmarks.json"),
    ),
    (
        "usd/c172.usdc",
        include_bytes!("../../../vehicles/c172p/usd/c172.usdc"),
    ),
    (
        "usd/c172p_airframe.usda",
        include_bytes!("../../../vehicles/c172p/usd/c172p_airframe.usda"),
    ),
    (
        "usd/d_ecjb.usda",
        include_bytes!("../../../vehicles/c172p/usd/d_ecjb.usda"),
    ),
    (
        "usd/d_ecjb_livery.json",
        include_bytes!("../../../vehicles/c172p/usd/d_ecjb_livery.json"),
    ),
    (
        "usd/d_ecjb_livery.usda",
        include_bytes!("../../../vehicles/c172p/usd/d_ecjb_livery.usda"),
    ),
    (
        "usd/n53295.usda",
        include_bytes!("../../../vehicles/c172p/usd/n53295.usda"),
    ),
    (
        "usd/n53295_livery.json",
        include_bytes!("../../../vehicles/c172p/usd/n53295_livery.json"),
    ),
    (
        "usd/n53295_livery.usda",
        include_bytes!("../../../vehicles/c172p/usd/n53295_livery.usda"),
    ),
    (
        CAMERA_VIEWS_ASSET,
        include_bytes!("../../../vehicles/c172p/views/cameras.json"),
    ),
];

const P38_MANIFEST: &[u8] = include_bytes!("../../../vehicles/p38g/syspack.json");
const P38_ASSETS: &[(&str, &[u8])] = &[
    (
        "fdm/p38g.xml",
        include_bytes!("../../../vehicles/p38g/fdm/p38g.xml"),
    ),
    (
        AIRSPEED_ENVELOPE_ASSET,
        include_bytes!("../../../vehicles/p38g/performance/airspeed_envelope.json"),
    ),
    (
        CALIBRATION_SUITE_ASSET,
        include_bytes!("../../../vehicles/p38g/performance/poh_benchmarks.json"),
    ),
    (
        PITOT_STATIC_CALIBRATION_ASSET,
        include_bytes!("../../../vehicles/p38g/performance/pitot_static_calibration.json"),
    ),
    (
        STRUCTURAL_LANDMARKS_ASSET,
        include_bytes!("../../../vehicles/p38g/geometry/landmarks.json"),
    ),
    (
        CAMERA_VIEWS_ASSET,
        include_bytes!("../../../vehicles/p38g/views/cameras.json"),
    ),
    (
        "p38g-set.xml",
        include_bytes!("../../../vehicles/p38g/p38g-set.xml"),
    ),
    (
        LIVERY_CATALOG_ASSET,
        include_bytes!("../../../vehicles/p38g/Liveries/catalog.json"),
    ),
    (
        "Models/Liveries/miss-virginia.xml",
        include_bytes!("../../../vehicles/p38g/Models/Liveries/miss-virginia.xml"),
    ),
    (
        "Models/Liveries/bare-metal.xml",
        include_bytes!("../../../vehicles/p38g/Models/Liveries/bare-metal.xml"),
    ),
    (
        "Models/Liveries/glacier-girl.xml",
        include_bytes!("../../../vehicles/p38g/Models/Liveries/glacier-girl.xml"),
    ),
];

/// The Cessna 172S Skyhawk SP (`c172s`, `N642SP` default) package files, embedded in the binary.
#[must_use]
pub fn builtin_c172s_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(C172S_MANIFEST, C172S_ASSETS)
}

/// The built-in Cessna 172S Skyhawk SP (`c172s`), loaded and SHA-256-verified.
pub fn builtin_c172s_package() -> Result<LoadedSyspack, PackError> {
    LoadedSyspack::load(&builtin_c172s_source())
}

/// The Cessna 172P Skyhawk II (`c172p`, `D-ECJB` default) package files, embedded in the binary.
#[must_use]
pub fn builtin_c172p_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(C172P_MANIFEST, C172P_ASSETS)
}

/// The built-in Cessna 172P Skyhawk II (`c172p`), loaded and SHA-256-verified.
pub fn builtin_c172p_package() -> Result<LoadedSyspack, PackError> {
    LoadedSyspack::load(&builtin_c172p_source())
}

/// Compatibility alias for [`builtin_c172s_source`] (`"c172"` -> `"c172s"`).
#[must_use]
pub fn builtin_c172_source() -> MemoryPackageSource {
    builtin_c172s_source()
}

/// Compatibility alias for [`builtin_c172s_package`] (`"c172"` -> `"c172s"`).
pub fn builtin_c172_package() -> Result<LoadedSyspack, PackError> {
    builtin_c172s_package()
}

/// The Lockheed P-38G Lightning's package files, embedded in the binary.
#[must_use]
pub fn builtin_p38g_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(P38_MANIFEST, P38_ASSETS)
}

/// The built-in Lockheed P-38G Lightning twin-boom fighter, loaded and verified
/// like any other package.
pub fn builtin_p38g_package() -> Result<LoadedSyspack, PackError> {
    LoadedSyspack::load(&builtin_p38g_source())
}
