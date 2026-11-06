use crate::airport_db::pack::crc32;
use crate::{AirportDatabase, CompactCode, SceneryError};

use super::parser::nasr;

fn database() -> AirportDatabase {
    AirportDatabase::from_records(nasr())
}

#[test]
fn crc32_matches_the_ieee_check_value() {
    assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    assert_eq!(crc32(b""), 0);
}

#[test]
fn a_pack_round_trips_records_to_the_centimetre() {
    let original = database();
    let bytes = original.to_pack_bytes().unwrap();
    assert_eq!(bytes.get(..8), Some(b"SIMAPT01".as_slice()));
    let decoded = AirportDatabase::from_pack_bytes(&bytes).unwrap();
    assert_eq!(decoded.len(), original.len());
    assert_eq!(
        decoded.to_pack_bytes().unwrap(),
        bytes,
        "re-encoding is byte-identical"
    );
    for (a, b) in original.records().iter().zip(decoded.records()) {
        assert!((a.lat_deg - b.lat_deg).abs() <= 5e-8 && (a.lon_deg - b.lon_deg).abs() <= 5e-8);
        let mut a = a.clone();
        a.lat_deg = b.lat_deg;
        a.lon_deg = b.lon_deg;
        let mut runways = a.runways.to_vec();
        for (runway, decoded) in runways.iter_mut().zip(b.runways.iter()) {
            for (end, decoded) in runway.ends.iter_mut().zip(decoded.ends.iter()) {
                assert!((end.lat_deg - decoded.lat_deg).abs() <= 5e-8);
                end.lat_deg = decoded.lat_deg;
                end.lon_deg = decoded.lon_deg;
            }
        }
        a.runways = runways.into_boxed_slice();
        assert_eq!(&a, b);
    }
}

#[test]
fn an_odd_magnetic_heading_and_runway_id_survive_the_pack() {
    let mut records = nasr();
    let runway = &mut records[0].runways.to_vec()[0].clone();
    runway.id = CompactCode::parse("NE/SW").unwrap();
    runway.ends[0].magnetic_heading_deg = 1.25;
    records[0].runways = vec![*runway].into_boxed_slice();
    let bytes = AirportDatabase::from_records(records)
        .to_pack_bytes()
        .unwrap();
    let decoded = AirportDatabase::from_pack_bytes(&bytes).unwrap();
    let runway = &decoded.records()[0].runways[0];
    assert_eq!(runway.id.as_str(), "NE/SW");
    assert_eq!(runway.ends[0].magnetic_heading_deg, 1.25);
}

#[test]
fn damaged_packs_are_rejected() {
    let bytes = database().to_pack_bytes().unwrap();
    let rejected = |bytes: &[u8]| {
        matches!(
            AirportDatabase::from_pack_bytes(bytes),
            Err(SceneryError::InvalidCatalog { .. })
        )
    };
    let mut flipped = bytes.clone();
    *flipped.last_mut().unwrap() ^= 0x40;
    assert!(rejected(&flipped), "CRC catches a flipped bit");
    assert!(rejected(&bytes[..bytes.len() - 1]), "truncated");
    let mut magic = bytes.clone();
    magic[0] = b'X';
    assert!(rejected(&magic));
    let mut version = bytes.clone();
    version[8] = 9;
    assert!(rejected(&version));
    let mut trailing = bytes.clone();
    trailing.push(0);
    assert!(rejected(&trailing));
    assert!(rejected(&[]));
}

#[test]
fn cycle_renewal_updates_regional_and_authority_sources_and_round_trips_pack() {
    use flightbag::AuthoritySource;

    let builtin = AirportDatabase::decode_builtin().unwrap();
    let (_, egll_before) = builtin.by_code("EGLL").unwrap();
    let renewed_base = super::parser::APT_BASE.replace("2026/10/29", "2026/11/26");
    let renewed_rwy = super::parser::APT_RWY.replace("2026/10/29", "2026/11/26");
    let us_renewed = builtin
        .renew(&crate::FaaNasrCsv {
            apt_base: &renewed_base,
            apt_rwy: &renewed_rwy,
            apt_rwy_end: super::parser::APT_RWY_END,
            frq: Some(super::parser::FRQ),
        })
        .unwrap();
    let (_, ktwo) = us_renewed.by_code("KTWO").unwrap();
    assert_eq!(ktwo.authority, AuthoritySource::FaaNasr);
    assert_eq!(ktwo.effective_ymd, [2026, 11, 26]);
    assert_eq!(
        us_renewed.by_code("EGLL").map(|(_, r)| r),
        Some(egll_before),
        "non-US records survive a US authority cycle renewal unchanged"
    );

    // Regional AIP / polar cycle upsert updates EGLL and NZSP without touching KTWO.
    let mut egll_aip = egll_before.clone();
    egll_aip.effective_ymd = [2027, 1, 21];
    let mut nzsp_aip = us_renewed.by_code("NZSP").unwrap().1.clone();
    nzsp_aip.effective_ymd = [2027, 1, 21];
    let regional_renewed = us_renewed.renew([egll_aip, nzsp_aip].as_slice()).unwrap();
    assert_eq!(
        regional_renewed.by_code("EGLL").unwrap().1.effective_ymd,
        [2027, 1, 21]
    );
    assert_eq!(
        regional_renewed.by_code("NZSP").unwrap().1.effective_ymd,
        [2027, 1, 21]
    );
    assert_eq!(
        regional_renewed.by_code("KTWO").unwrap().1.effective_ymd,
        [2026, 11, 26]
    );

    let dir = tempfile::tempdir().unwrap();
    let pack_path = dir.path().join("renewed_airports.bin");
    std::fs::write(&pack_path, regional_renewed.to_pack_bytes().unwrap()).unwrap();
    let from_disk = AirportDatabase::from_pack_file(&pack_path).unwrap();
    assert_eq!(
        from_disk.by_code("KTWO").unwrap().1.effective_ymd,
        [2026, 11, 26]
    );
    assert_eq!(
        from_disk.by_code("EGLL").unwrap().1.effective_ymd,
        [2027, 1, 21]
    );
    assert_eq!(
        from_disk.by_code("NZSP").unwrap().1.effective_ymd,
        [2027, 1, 21]
    );
}

#[test]
fn landing_runway_geometry_applies_surveyed_displaced_threshold_slope_and_glidepath() {
    use crate::CatalogVerticalDatum;
    use sim_geo::geodetic_to_ecef;

    let db = AirportDatabase::builtin();
    let (_, a47n) = db.by_code("47N").unwrap();
    let r07 = a47n
        .landing_runway("07", CatalogVerticalDatum::OrthometricMslEgm96)
        .unwrap();
    assert!((r07.displaced_threshold_m - 176.8).abs() < 1.0);
    let chord_shift =
        (geodetic_to_ecef(&r07.threshold) - geodetic_to_ecef(&r07.pavement_start)).length();
    assert!((chord_shift - r07.displaced_threshold_m).abs() < 0.2);
    assert!((r07.full_length_m - r07.landing_length_m - r07.displaced_threshold_m).abs() < 0.2);
    assert!(
        r07.far_elevation_msl_m < r07.start_elevation_msl_m - 5.0,
        "47N RWY 07 slopes downhill toward RWY 25: {} -> {}",
        r07.start_elevation_msl_m,
        r07.far_elevation_msl_m
    );
    assert!((r07.glidepath_rad.to_degrees() - 3.4).abs() < 0.05);

    let (_, kewr) = db.by_code("KEWR").unwrap();
    let r22l = kewr
        .landing_runway("22L", CatalogVerticalDatum::OrthometricMslEgm96)
        .unwrap();
    assert!(r22l.displaced_threshold_m > 200.0);
    assert!(r22l.landing_length_m < r22l.full_length_m - 200.0);
}
