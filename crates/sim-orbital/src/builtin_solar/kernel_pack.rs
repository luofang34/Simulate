//! IO-free kernel-pack loader for epochs outside [`BuiltinSolarSystem2026_2028`]
//! (such as Apollo 11 in July 1969).

use std::collections::BTreeMap;

use crate::{
    builtin_solar::{
        BuiltinSolarError, BuiltinSolarSystem2026_2028, EARTH_ITRF93_BPC_2026_2028_BYTES,
        GM_DE440_TPC_BYTES, MOON_DE440_FK_BYTES, MOON_PA_BPC_2026_2028_BYTES, PCK00011_TPC_BYTES,
        SOLAR_SPK_2026_2028_BYTES,
    },
    constants::GravityConstantsCatalog,
    daf::{BpcFile, SpkFile},
    ephemeris::{MarsCenterApproximation, SpkEphemerisCatalog},
    ids::NaifId,
    orientation::{IauPckCatalog, LunarTargetFrame, OrientationPolicy, StackedOrientationCatalog},
    time::TimeScaleCatalog,
};

/// Byte-slice kernel pack extending [`BuiltinSolarSystem2026_2028`] to additional
/// historical or future epochs without file-system IO inside `sim-orbital`.
#[derive(Clone, Copy, Debug)]
pub struct SolarSystemKernelPack<'a> {
    /// Additional binary `DAF/SPK` slices (`Type 2` or `Type 3` Chebyshev).
    pub spk_files: &'a [&'a [u8]],
    /// Additional binary Earth `ITRF93` `DAF/PCK` slices.
    pub earth_bpc_files: &'a [&'a [u8]],
    /// Additional binary Lunar `PA` `DAF/PCK` slices.
    pub moon_bpc_files: &'a [&'a [u8]],
    /// Whether epochs outside binary `.bpc` coverage fall back to `pck00011.tpc` IAU models.
    pub allow_iau_outside_bpc: bool,
}

impl BuiltinSolarSystem2026_2028 {
    /// Verifies the built-in `2026..2028` assets and merges an additional [`SolarSystemKernelPack`].
    pub fn load_with_kernel_pack(
        pack: &SolarSystemKernelPack<'_>,
    ) -> Result<Self, BuiltinSolarError> {
        let base = Self::load_verified()?;
        let mut spks = Vec::with_capacity(1_usize.saturating_add(pack.spk_files.len()));
        spks.push(SpkFile::from_bytes(SOLAR_SPK_2026_2028_BYTES)?);
        for bytes in pack.spk_files {
            spks.push(SpkFile::from_bytes(bytes)?);
        }
        let ephemeris =
            SpkEphemerisCatalog::from_files(spks, MarsCenterApproximation::StrictRequire499);
        let constants = GravityConstantsCatalog::from_naif_gm_and_pck_bytes(
            GM_DE440_TPC_BYTES,
            PCK00011_TPC_BYTES,
        )?;
        let iau = IauPckCatalog::from_pck_bytes(PCK00011_TPC_BYTES)?;
        let mut earth_bpcs = vec![BpcFile::from_bytes(EARTH_ITRF93_BPC_2026_2028_BYTES)?];
        for bytes in pack.earth_bpc_files {
            earth_bpcs.push(BpcFile::from_bytes(bytes)?);
        }
        let mut moon_bpcs = vec![BpcFile::from_bytes(MOON_PA_BPC_2026_2028_BYTES)?];
        for bytes in pack.moon_bpc_files {
            moon_bpcs.push(BpcFile::from_bytes(bytes)?);
        }
        let fallback_policy = if pack.allow_iau_outside_bpc {
            OrientationPolicy::AllowApproximateIau
        } else {
            OrientationPolicy::RequirePrecision
        };
        let mut policies = BTreeMap::new();
        policies.insert(NaifId::EARTH, fallback_policy);
        policies.insert(NaifId::MOON, fallback_policy);
        policies.insert(NaifId::MARS, OrientationPolicy::AllowApproximateIau);
        let orientation = StackedOrientationCatalog::build(
            iau,
            earth_bpcs,
            moon_bpcs,
            Some(MOON_DE440_FK_BYTES),
            LunarTargetFrame::MeanEarth,
            policies,
            OrientationPolicy::AllowApproximateIau,
        )?;
        let time_scales = TimeScaleCatalog::builtin()?;
        let _ = base;
        Ok(Self {
            ephemeris,
            constants,
            orientation,
            time_scales,
        })
    }
}

/// Unaltered Type-2 Chebyshev granules of NASA NAIF `de440s.bsp` for the Sun, the
/// planetary barycentres, Earth and the Moon over the Apollo 11 mission,
/// `1969-07-14 .. 1969-07-27 TDB` (`9,216 B`), cut by
/// `scripts/extract_apollo11_1969_subset.py`. Mars's own centre is not in `de440s`.
const APOLLO11_SPK_1969_BYTES: &[u8] = include_bytes!("../../data/solar_de440s_apollo11_1969.bsp");
/// Expected SHA-256 hex of [`APOLLO11_SPK_1969_BYTES`].
const APOLLO11_SPK_1969_SHA256: &str =
    "ad652a46560d184824cb8abfcdc566aef3723a051d87e51a346a392ed72b5793";

/// The embedded Apollo 11 `de440s` subset, after checking it against its pinned hash.
pub fn apollo_1969_spk_bytes() -> Result<&'static [u8], BuiltinSolarError> {
    super::verify_asset(
        "apollo11_spk_1969",
        APOLLO11_SPK_1969_BYTES,
        APOLLO11_SPK_1969_SHA256,
    )?;
    Ok(APOLLO11_SPK_1969_BYTES)
}
