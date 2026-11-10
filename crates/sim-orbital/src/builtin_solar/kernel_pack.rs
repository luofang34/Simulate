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

/// Builds an in-memory `4,096 B` little-endian `DAF/SPK` Type 2 Chebyshev kernel pack
/// covering the Apollo 11 mission window (`1969-07-16 .. 1969-07-25 TDB`).
#[must_use]
pub fn apollo_1969_spk_bytes() -> Vec<u8> {
    let mut bytes = vec![0_u8; 4_096];
    write_slice(&mut bytes, 0, b"DAF/SPK ");
    write_i32_le(&mut bytes, 8, 2);
    write_i32_le(&mut bytes, 12, 6);
    write_slice(&mut bytes, 16, b"APOLLO11_1969_SPK");
    write_i32_le(&mut bytes, 76, 2);
    write_i32_le(&mut bytes, 80, 2);
    write_i32_le(&mut bytes, 84, 433);
    write_slice(&mut bytes, 88, b"LTL-IEEE");

    let start_s = -961_243_200.0_f64;
    let end_s = -960_465_600.0_f64;
    let mid_s = 0.5 * (start_s + end_s);
    let radius_s = 0.5 * (end_s - start_s);

    // Record 2 (offset 1024): summary header + 4 segment summaries (40 bytes each).
    write_f64_le(&mut bytes, 1024, 0.0);
    write_f64_le(&mut bytes, 1032, 0.0);
    write_f64_le(&mut bytes, 1040, 4.0);

    let segs = [
        SpkSeedSegment {
            target: 10,
            center: 0,
            pos0_km: [0.0, 0.0, 0.0],
            pos1_km: [0.0, 0.0, 0.0],
            label: b"SUN_SSB_1969",
        },
        SpkSeedSegment {
            target: 3,
            center: 0,
            pos0_km: [72_963_000.0, -122_063_000.0, -52_903_000.0],
            pos1_km: [25.68 * radius_s, 13.06 * radius_s, 5.66 * radius_s],
            label: b"EMB_SSB_1969",
        },
        SpkSeedSegment {
            target: 399,
            center: 3,
            pos0_km: [2_580.0, 3_760.0, 1_510.0],
            pos1_km: [-0.009 * radius_s, 0.007 * radius_s, 0.003 * radius_s],
            label: b"EARTH_EMB_1969",
        },
        SpkSeedSegment {
            target: 301,
            center: 3,
            pos0_km: [-209_920.0, -306_040.0, -122_690.0],
            pos1_km: [0.75 * radius_s, -0.56 * radius_s, -0.24 * radius_s],
            label: b"MOON_EMB_1969",
        },
    ];

    for (idx, seg) in segs.iter().enumerate() {
        write_spk_segment(&mut bytes, idx, start_s, end_s, mid_s, radius_s, seg);
    }
    bytes
}

struct SpkSeedSegment<'a> {
    target: i32,
    center: i32,
    pos0_km: [f64; 3],
    pos1_km: [f64; 3],
    label: &'a [u8],
}

fn write_spk_segment(
    bytes: &mut [u8],
    idx: usize,
    start_s: f64,
    end_s: f64,
    mid_s: f64,
    radius_s: f64,
    seg: &SpkSeedSegment<'_>,
) {
    let start_word = 385_i32 + (idx as i32) * 12;
    let sum_off = 1048 + idx * 40;
    write_f64_le(bytes, sum_off, start_s);
    write_f64_le(bytes, sum_off + 8, end_s);
    write_i32_le(bytes, sum_off + 16, seg.target);
    write_i32_le(bytes, sum_off + 20, seg.center);
    write_i32_le(bytes, sum_off + 24, 1);
    write_i32_le(bytes, sum_off + 28, 2);
    write_i32_le(bytes, sum_off + 32, start_word);
    write_i32_le(bytes, sum_off + 36, start_word + 11);
    write_slice(bytes, 2048 + idx * 40, seg.label);

    let data_off = ((start_word as usize) - 1) * 8;
    let [p0x, p0y, p0z] = seg.pos0_km;
    let [p1x, p1y, p1z] = seg.pos1_km;
    let words = [
        mid_s,
        radius_s,
        p0x,
        p1x,
        p0y,
        p1y,
        p0z,
        p1z,
        start_s,
        end_s - start_s,
        8.0,
        1.0,
    ];
    for (w, &word) in words.iter().enumerate() {
        write_f64_le(bytes, data_off + w * 8, word);
    }
}

fn write_slice(buf: &mut [u8], offset: usize, src: &[u8]) {
    if let Some(dst) = buf.get_mut(offset..offset.saturating_add(src.len())) {
        dst.copy_from_slice(src);
    }
}

fn write_i32_le(buf: &mut [u8], offset: usize, val: i32) {
    write_slice(buf, offset, &val.to_le_bytes());
}

fn write_f64_le(buf: &mut [u8], offset: usize, val: f64) {
    write_slice(buf, offset, &val.to_le_bytes());
}
