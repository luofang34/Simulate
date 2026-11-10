//! Embedded SHA-256-verified `2026-01-01 .. 2028-01-01 TDB` NAIF solar-system package.
//!
//! Combines unaltered Type-2 Chebyshev granules extracted from NASA NAIF `de440s.bsp`,
//! `mar099s.bsp` (`499 -> 4`), `moon_pa_de440_200625.bpc`, and `earth_1962_260806_2126_combined.bpc`
//! with `gm_de440.tpc`, `pck00011.tpc`, `moon_de440_250416.tf`, and `naif0012.tls`.

mod kernel_pack;

use std::collections::BTreeMap;

pub use kernel_pack::{SolarSystemKernelPack, apollo_1969_spk_bytes};

use crate::{
    constants::{ConstantsError, GravityConstantsCatalog},
    daf::{BpcFile, DafError, SpkFile},
    ephemeris::{EphemerisError, MarsCenterApproximation, SpkEphemerisCatalog},
    hash::{ContentHash, HashParseError},
    ids::NaifId,
    orientation::{
        IauPckCatalog, LunarTargetFrame, OrientationError, OrientationPolicy,
        StackedOrientationCatalog,
    },
    time::{TdbEpoch, TimeError, TimeScaleCatalog},
};

/// Embedded `de440s + mar099s` `2026..2028` binary SPK subset (`819,200 B`).
pub const SOLAR_SPK_2026_2028_BYTES: &[u8] =
    include_bytes!("../data/solar_de440s_mar099s_2026_2028.bsp");
/// Expected SHA-256 hex of [`SOLAR_SPK_2026_2028_BYTES`].
pub const SOLAR_SPK_2026_2028_SHA256: &str =
    "d1757d81d79a9c037d73f3e77d328008004a348205b1ec010ba71c76cab71067";

/// Embedded `moon_pa_de440` `2026..2028` binary PCK subset (`27,648 B`).
pub const MOON_PA_BPC_2026_2028_BYTES: &[u8] =
    include_bytes!("../data/moon_pa_de440_2026_2028.bpc");
/// Expected SHA-256 hex of [`MOON_PA_BPC_2026_2028_BYTES`].
pub const MOON_PA_BPC_2026_2028_SHA256: &str =
    "a18f1e17650555c45cce902bf72dd3a3c506c1dce5f0853dc0a4535d3a5baf30";

/// Embedded `earth_itrf93` `2026..2028` binary PCK subset (`384,000 B`).
pub const EARTH_ITRF93_BPC_2026_2028_BYTES: &[u8] =
    include_bytes!("../data/earth_itrf93_2026_2028.bpc");
/// Expected SHA-256 hex of [`EARTH_ITRF93_BPC_2026_2028_BYTES`].
pub const EARTH_ITRF93_BPC_2026_2028_SHA256: &str =
    "008083d9c977704dacf467c05b9ff54e5c57613678ba44215822f3386a44df20";

/// Embedded NASA NAIF `gm_de440.tpc` gravitational parameters (`12,406 B`).
pub const GM_DE440_TPC_BYTES: &[u8] = include_bytes!("../data/gm_de440.tpc");
/// Expected SHA-256 hex of [`GM_DE440_TPC_BYTES`].
pub const GM_DE440_TPC_SHA256: &str =
    "924ddf4fb9ead9fe8a1aa55780bcabde40b09d00065d58226e24b68d8092f140";

/// Embedded NASA NAIF `pck00011.tpc` IAU rotational & radii constants (`131,226 B`).
pub const PCK00011_TPC_BYTES: &[u8] = include_bytes!("../data/pck00011.tpc");
/// Expected SHA-256 hex of [`PCK00011_TPC_BYTES`].
pub const PCK00011_TPC_SHA256: &str =
    "3dff7b1dbeceaa01f25467767d3fa25816051c85d162d1edf04acb310ee28bb1";

/// Embedded NASA NAIF `moon_de440_250416.tf` lunar PA/ME frame kernel (`19,478 B`).
pub const MOON_DE440_FK_BYTES: &[u8] = include_bytes!("../data/moon_de440_250416.tf");
/// Expected SHA-256 hex of [`MOON_DE440_FK_BYTES`].
pub const MOON_DE440_FK_SHA256: &str =
    "a47c71e9c9f33796bdafb2c9d69a7ee447b6016ecad80f71cd6f3e479f9cf768";

/// Embedded NASA NAIF `naif0012.tls` leap-second kernel (`5,257 B`), shared
/// with every flight through [`TimeScaleCatalog::builtin`].
pub const NAIF0012_TLS_BYTES: &[u8] = sim_spacetime::BUILTIN_LSK_BYTES;
/// Expected SHA-256 hex of [`NAIF0012_TLS_BYTES`].
pub const NAIF0012_TLS_SHA256: &str = sim_spacetime::BUILTIN_LSK_SHA256;

/// Why loading or verifying [`BuiltinSolarSystem2026_2028`] failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum BuiltinSolarError {
    /// Hex digest constant failed to parse.
    #[error("invalid expected SHA-256 constant: {0}")]
    InvalidHex(#[from] HashParseError),
    /// Embedded asset SHA-256 did not match its pinned constant.
    #[error("SHA-256 mismatch on `{asset}`: expected {expected}, got {actual}")]
    ChecksumMismatch {
        /// Asset label.
        asset: &'static str,
        /// Pinned SHA-256 digest.
        expected: ContentHash,
        /// Actual SHA-256 digest.
        actual: ContentHash,
    },
    /// DAF container failed to parse.
    #[error("DAF error: {0}")]
    Daf(#[from] DafError),
    /// Ephemeris catalog failed to initialize.
    #[error("ephemeris error: {0}")]
    Ephemeris(#[from] EphemerisError),
    /// Gravity constants failed to initialize.
    #[error("constants error: {0}")]
    Constants(#[from] ConstantsError),
    /// Orientation catalog failed to initialize.
    #[error("orientation error: {0}")]
    Orientation(#[from] OrientationError),
    /// Time-scale catalog failed to initialize.
    #[error("time-scale error: {0}")]
    Time(#[from] TimeError),
}

/// Ready-to-use `2026-01-01 .. 2028-01-01 TDB` solar-system catalogs verified by SHA-256.
#[derive(Clone, Debug)]
pub struct BuiltinSolarSystem2026_2028 {
    /// SPK ephemeris catalog (`StrictRequire499`, covering Sun, Mercury, Venus, Earth, Moon, Mars `499`, Jupiter `5`, Saturn `6`).
    pub ephemeris: SpkEphemerisCatalog,
    /// DE440 gravitational constants + WGS84 / Moon / Mars $J_2$ harmonics.
    pub constants: GravityConstantsCatalog,
    /// Stacked precision orientation catalog (`ITRF93` for Earth, `MOON_ME` for Moon, `IAU_MARS` for Mars).
    pub orientation: StackedOrientationCatalog,
    /// Time-scale catalog (`naif0012.tls` with explicit `FreezeLastTaiOffset` through `2028`).
    pub time_scales: TimeScaleCatalog,
}

impl BuiltinSolarSystem2026_2028 {
    /// Inclusive lower coverage bound (`2026-01-01T00:00:00 TDB`).
    pub const START_TDB: TdbEpoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    /// Inclusive upper coverage bound (`2028-01-01T00:00:00 TDB`).
    pub const END_TDB: TdbEpoch = TdbEpoch::JAN_1_2028_00Z_TDB;
    /// Embedded `de440s + mar099s` `2026..2028` binary SPK subset (`819,200 B`).
    pub const SOLAR_SPK_BYTES: &'static [u8] = SOLAR_SPK_2026_2028_BYTES;
    /// Embedded NASA NAIF `pck00011.tpc` IAU rotational & radii constants (`131,226 B`).
    pub const PCK00011_TPC_BYTES: &'static [u8] = PCK00011_TPC_BYTES;
    /// Embedded NASA NAIF `moon_de440_250416.tf` lunar PA/ME frame kernel (`19,478 B`).
    pub const MOON_DE440_FK_BYTES: &'static [u8] = MOON_DE440_FK_BYTES;
    /// Embedded NASA NAIF `naif0012.tls` leap-second kernel (`5,257 B`).
    pub const NAIF0012_TLS_BYTES: &'static [u8] = NAIF0012_TLS_BYTES;

    /// Total embedded binary + text kernel byte footprint (`1,399,215 B` = `1.33 MiB`).
    #[must_use]
    pub const fn total_embedded_bytes() -> usize {
        SOLAR_SPK_2026_2028_BYTES.len()
            + MOON_PA_BPC_2026_2028_BYTES.len()
            + EARTH_ITRF93_BPC_2026_2028_BYTES.len()
            + GM_DE440_TPC_BYTES.len()
            + PCK00011_TPC_BYTES.len()
            + MOON_DE440_FK_BYTES.len()
            + NAIF0012_TLS_BYTES.len()
    }

    /// Verifies all 7 embedded SHA-256 digests and constructs the complete catalog bundle.
    pub fn load_verified() -> Result<Self, BuiltinSolarError> {
        verify_asset(
            "solar_spk",
            SOLAR_SPK_2026_2028_BYTES,
            SOLAR_SPK_2026_2028_SHA256,
        )?;
        verify_asset(
            "moon_bpc",
            MOON_PA_BPC_2026_2028_BYTES,
            MOON_PA_BPC_2028_HEX_ALIAS,
        )?;
        verify_asset(
            "earth_bpc",
            EARTH_ITRF93_BPC_2026_2028_BYTES,
            EARTH_ITRF93_BPC_2026_2028_SHA256,
        )?;
        verify_asset("gm_de440", GM_DE440_TPC_BYTES, GM_DE440_TPC_SHA256)?;
        verify_asset("pck00011", PCK00011_TPC_BYTES, PCK00011_TPC_SHA256)?;
        verify_asset("moon_fk", MOON_DE440_FK_BYTES, MOON_DE440_FK_SHA256)?;
        verify_asset("naif0012", NAIF0012_TLS_BYTES, NAIF0012_TLS_SHA256)?;

        let spk_file = SpkFile::from_bytes(SOLAR_SPK_2026_2028_BYTES)?;
        let ephemeris = SpkEphemerisCatalog::from_files(
            vec![spk_file],
            MarsCenterApproximation::StrictRequire499,
        );
        let constants = GravityConstantsCatalog::from_naif_gm_and_pck_bytes(
            GM_DE440_TPC_BYTES,
            PCK00011_TPC_BYTES,
        )?;

        let iau = IauPckCatalog::from_pck_bytes(PCK00011_TPC_BYTES)?;
        let earth_bpc = BpcFile::from_bytes(EARTH_ITRF93_BPC_2026_2028_BYTES)?;
        let moon_bpc = BpcFile::from_bytes(MOON_PA_BPC_2026_2028_BYTES)?;
        let mut policies = BTreeMap::new();
        policies.insert(NaifId::EARTH, OrientationPolicy::RequirePrecision);
        policies.insert(NaifId::MOON, OrientationPolicy::RequirePrecision);
        policies.insert(NaifId::MARS, OrientationPolicy::AllowApproximateIau);
        let orientation = StackedOrientationCatalog::build(
            iau,
            vec![earth_bpc],
            vec![moon_bpc],
            Some(MOON_DE440_FK_BYTES),
            LunarTargetFrame::MeanEarth,
            policies,
            OrientationPolicy::AllowApproximateIau,
        )?;

        let time_scales = TimeScaleCatalog::builtin()?;

        Ok(Self {
            ephemeris,
            constants,
            orientation,
            time_scales,
        })
    }
}

const MOON_PA_BPC_2028_HEX_ALIAS: &str = MOON_PA_BPC_2026_2028_SHA256;

fn verify_asset(
    asset: &'static str,
    bytes: &[u8],
    expected_hex: &str,
) -> Result<(), BuiltinSolarError> {
    let expected = ContentHash::from_hex(expected_hex)?;
    let actual = ContentHash::of_bytes(bytes);
    if actual != expected {
        return Err(BuiltinSolarError::ChecksumMismatch {
            asset,
            expected,
            actual,
        });
    }
    Ok(())
}
