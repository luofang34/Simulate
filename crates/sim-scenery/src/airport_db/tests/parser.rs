use flightbag::AuthoritySource;

use crate::airport_db::assemble::{parse_ymd, title_case};
use crate::airport_db::csv::CsvReader;
use crate::{
    AerodromeTier, AirportFrequency, FaaNasrCsv, FrequencyRole, OurAirportsCsv, RunwaySurfaceKind,
    parse_faa_nasr, parse_our_airports,
};

pub(super) const APT_BASE: &str = "\u{feff}\"EFF_DATE\",\"SITE_NO\",\"SITE_TYPE_CODE\",\"STATE_CODE\",\"ARPT_ID\",\"CITY\",\"COUNTRY_CODE\",\"ARPT_NAME\",\"OWNERSHIP_TYPE_CODE\",\"FACILITY_USE_CODE\",\"LAT_DECIMAL\",\"LONG_DECIMAL\",\"ELEV\",\"MAG_VARN\",\"MAG_HEMIS\",\"ARPT_STATUS\",\"TWR_TYPE_CODE\",\"ICAO_ID\"\r
\"2026/10/29\",\"001.\",\"A\",\"NJ\",\"T01\",\"TESTVILLE\",\"US\",\"SMITH, JONES \"\"FIELD\"\"\",\"PU\",\"PU\",40.5,-74.5,100,13,\"W\",\"O\",\"NON-ATCT\",\"\"\r
\"2026/10/29\",\"002.\",\"A\",\"CA\",\"T02\",\"EASTON\",\"US\",\"EAST INTL\",\"PU\",\"PU\",34.0,-118.0,328.1,11,\"E\",\"O\",\"ATCT\",\"KTWO\"\r
\"2026/10/29\",\"003.\",\"A\",\"DE\",\"T03\",\"BASE\",\"US\",\"MIL AFB\",\"MA\",\"PR\",39.0,-75.0,10,12,\"W\",\"O\",\"ATCT-RAPCON\",\"KMIL\"\r
\"2026/10/29\",\"004.\",\"A\",\"NJ\",\"T04\",\"PRIVATE\",\"US\",\"FARM\",\"PR\",\"PR\",40.0,-74.0,10,12,\"W\",\"O\",\"NON-ATCT\",\"\"\r
\"2026/10/29\",\"005.\",\"A\",\"NJ\",\"T05\",\"CLOSED\",\"US\",\"GONE\",\"PU\",\"PU\",40.1,-74.1,10,12,\"W\",\"CI\",\"NON-ATCT\",\"\"\r
";

pub(super) const APT_RWY: &str = "\"EFF_DATE\",\"SITE_NO\",\"RWY_ID\",\"RWY_LEN\",\"RWY_WIDTH\",\"SURFACE_TYPE_CODE\",\"RWY_LGT_CODE\"
\"2026/10/29\",\"001.\",\"07/25\",3000,50,\"TURF\",\"\"
\"2026/10/29\",\"001.\",\"H1\",60,60,\"CONC\",\"\"
\"2026/10/29\",\"001.\",\"18/36\",700,40,\"ASPH\",\"LOW\"
\"2026/10/29\",\"002.\",\"09/27\",10000,150,\"ASPH-CONC\",\"HIGH\"
\"2026/10/29\",\"003.\",\"01/19\",5000,150,\"CONC\",\"HIGH\"
\"2026/10/29\",\"004.\",\"02/20\",2000,30,\"TURF\",\"\"
\"2026/10/29\",\"005.\",\"03/21\",2000,30,\"ASPH\",\"\"
";

pub(super) const APT_RWY_END: &str = "\"SITE_NO\",\"RWY_ID\",\"RWY_END_ID\",\"LAT_DECIMAL\",\"LONG_DECIMAL\",\"RWY_END_ELEV\",\"DISPLACED_THR_LEN\",\"VISUAL_GLIDE_PATH_ANGLE\",\"ILS_TYPE\",\"VGSI_CODE\",\"APCH_LGT_SYSTEM_CODE\"
\"001.\",\"07/25\",\"07\",40.499,-74.505,98.5,200,,\"\",\"\",\"\"
\"001.\",\"07/25\",\"25\",40.501,-74.495,,,3.5,\"\",\"P2L\",\"NONE\"
\"001.\",\"H1\",\"H1\",40.5,-74.5,100,,,\"\",\"\",\"\"
\"001.\",\"18/36\",\"18\",40.51,-74.5,100,,,\"\",\"\",\"\"
\"001.\",\"18/36\",\"36\",40.508,-74.5,100,,,\"\",\"\",\"\"
\"002.\",\"09/27\",\"09\",34.0,-118.02,320,,3,\"ILS/DME\",\"P4L\",\"ALSF2\"
\"002.\",\"09/27\",\"27\",34.0,-117.98,330,,3,\"\",\"P4L\",\"MALSR\"
\"003.\",\"01/19\",\"01\",38.99,-75.0,9,,,\"ILS\",\"\",\"\"
\"003.\",\"01/19\",\"19\",39.01,-75.0,11,,,\"\",\"\",\"\"
\"004.\",\"02/20\",\"02\",39.99,-74.0,10,,,\"\",\"\",\"\"
\"004.\",\"02/20\",\"20\",40.0,-74.0,10,,,\"\",\"\",\"\"
\"005.\",\"03/21\",\"03\",40.09,-74.1,10,,,\"\",\"\",\"\"
\"005.\",\"03/21\",\"21\",40.1,-74.1,10,,,\"\",\"\",\"\"
";

pub(super) const FRQ: &str =
    "\"FACILITY_TYPE\",\"SERVICED_FACILITY\",\"SERVICED_SITE_TYPE\",\"FREQ\",\"FREQ_USE\"
\"NON-ATCT\",\"T01\",\"AIRPORT\",\"122.8\",\"CTAF\"
\"NON-ATCT\",\"T01\",\"AIRPORT\",\"122.8\",\"UNICOM\"
\"ATCT\",\"T02\",\"AIRPORT\",\"134.05\",\"LCL/S\"
\"ATCT\",\"T02\",\"AIRPORT\",\"118.3\",\"LCL/P\"
\"ATCT\",\"T02\",\"AIRPORT\",\"257.6\",\"LCL/P\"
\"ATCT\",\"T02\",\"AIRPORT\",\"121.8\",\"GND/P\"
\"TRACON\",\"T02\",\"AIRPORT\",\"132.8\",\"APCH/P DEP/P\"
\"NAVAID\",\"T02\",\"VOR\",\"113.75/84Y\",\"T02 DME\"
";

pub(super) fn nasr() -> Vec<crate::AirportRecord> {
    parse_faa_nasr(&FaaNasrCsv {
        apt_base: APT_BASE,
        apt_rwy: APT_RWY,
        apt_rwy_end: APT_RWY_END,
        frq: Some(FRQ),
    })
    .unwrap()
}

#[test]
fn csv_fields_keep_quoted_commas_quotes_and_line_breaks() {
    let mut reader = CsvReader::new(
        "t.csv",
        "a,\"b, c\",\"say \"\"hi\"\"\"\r\n\"multi\nline\",,x\nlast",
    );
    let mut row = Vec::new();
    assert!(reader.next_record(&mut row).unwrap());
    assert_eq!(row, ["a", "b, c", "say \"hi\""]);
    assert!(reader.next_record(&mut row).unwrap());
    assert_eq!(row, ["multi\nline", "", "x"]);
    assert!(reader.next_record(&mut row).unwrap());
    assert_eq!(row, ["last"]);
    assert!(!reader.next_record(&mut row).unwrap());
    let mut broken = CsvReader::new("t.csv", "\"open");
    assert!(broken.next_record(&mut row).is_err());
}

#[test]
fn nasr_rows_select_public_and_towered_military_fields() {
    let records = nasr();
    let codes: Vec<&str> = records.iter().map(|r| r.ids.primary.as_str()).collect();
    assert_eq!(codes, ["T01", "KTWO", "KMIL"]);
    assert!(
        records
            .iter()
            .all(|r| r.authority == AuthoritySource::FaaNasr)
    );
    assert!(records.iter().all(|r| r.effective_ymd == [2026, 10, 29]));
}

#[test]
fn nasr_values_convert_feet_variation_and_names() {
    let records = nasr();
    let t01 = &records[0];
    assert_eq!(&*t01.name, "SMITH, JONES \"FIELD\"");
    assert_eq!(t01.ids.faa_lid.unwrap().as_str(), "T01");
    assert!(t01.ids.icao.is_none());
    assert_eq!(t01.region_code.as_str(), "US-NJ");
    assert!((t01.elevation_msl_m - 30.48).abs() < 1e-4);
    assert!((t01.mag_var_deg + 13.0).abs() < 1e-6);
    // Helipad and the 700 ft strip are dropped.
    assert_eq!(t01.runways.len(), 1);
    let runway = &t01.runways[0];
    assert_eq!(runway.surface, RunwaySurfaceKind::Turf);
    assert!(!runway.lighted);
    assert!((runway.length_m - 914.4).abs() < 1e-3 && (runway.width_m - 15.24).abs() < 1e-3);
    let [r07, r25] = &runway.ends;
    assert!((r07.elevation_msl_m - 98.5 * 0.3048).abs() < 1e-4);
    // An end without its own elevation stands at the field's.
    assert!((r25.elevation_msl_m - 30.48).abs() < 1e-4);
    assert!((r07.displaced_threshold_m - 60.96).abs() < 1e-4);
    assert_eq!(r07.glidepath_deg, 3.0);
    assert_eq!(r25.glidepath_deg, 3.5);
    assert!(r25.has_vgsi && !r25.has_approach_lights && !r25.has_ils);
    let expected_true = r07.true_heading_deg;
    assert!((r07.magnetic_heading_deg - (expected_true + 13.0)).abs() < 1e-4);
    assert!(((r25.true_heading_deg - expected_true).rem_euclid(360.0) - 180.0).abs() < 0.1);
    assert_eq!(t01.tier, AerodromeTier::Tier3Local);

    let t02 = &records[1];
    assert!(t02.towered && t02.hard_surface);
    assert!((t02.mag_var_deg - 11.0).abs() < 1e-6);
    assert_eq!(t02.tier, AerodromeTier::Tier1Major);
    assert!(t02.runways[0].ends[0].has_ils && t02.runways[0].ends[0].has_approach_lights);
    assert_eq!(t02.runways[0].surface, RunwaySurfaceKind::Asphalt);
}

#[test]
fn nasr_frequencies_keep_vhf_roles_primary_first() {
    let records = nasr();
    assert_eq!(
        &*records[0].frequencies,
        [
            AirportFrequency {
                role: FrequencyRole::Ctaf,
                khz: 122_800
            },
            AirportFrequency {
                role: FrequencyRole::Unicom,
                khz: 122_800
            },
        ]
    );
    assert_eq!(
        records[0].primary_frequency_mhz(),
        Some((FrequencyRole::Ctaf, 122.8))
    );
    let roles: Vec<(FrequencyRole, u32)> = records[1]
        .frequencies
        .iter()
        .map(|f| (f.role, f.khz))
        .collect();
    assert_eq!(
        roles,
        [
            (FrequencyRole::Ground, 121_800),
            (FrequencyRole::Tower, 118_300),
            (FrequencyRole::Tower, 134_050),
            (FrequencyRole::Approach, 132_800),
            (FrequencyRole::Departure, 132_800),
        ]
    );
    assert_eq!(
        records[1].primary_frequency_mhz(),
        Some((FrequencyRole::Tower, 118.3))
    );
}

const OA_AIRPORTS: &str = "\"id\",\"ident\",\"type\",\"name\",\"latitude_deg\",\"longitude_deg\",\"elevation_ft\",\"continent\",\"iso_country\",\"iso_region\",\"municipality\",\"scheduled_service\",\"icao_code\",\"iata_code\",\"gps_code\",\"local_code\"
1,\"EGXX\",\"large_airport\",\"Test, Heath\",51.47,-0.46,83,\"EU\",\"GB\",\"GB-ENG\",\"London\",\"yes\",\"EGXX\",\"TXX\",\"EGXX\",\"\"
2,\"VNLK\",\"small_airport\",\"Lukla\",27.68,86.73,9380,\"AS\",\"NP\",\"NP-P1\",\"Lukla\",\"yes\",\"VNLK\",\"LUA\",\"VNLK\",\"\"
3,\"XX-0001\",\"small_airport\",\"Field\",10.0,10.0,100,\"AF\",\"NG\",\"NG-LA\",\"\",\"no\",\"\",\"\",\"\",\"\"
4,\"KZZZ\",\"large_airport\",\"US\",40.0,-74.0,10,\"NA\",\"US\",\"US-NJ\",\"\",\"yes\",\"KZZZ\",\"ZZZ\",\"KZZZ\",\"ZZZ\"
";

const OA_RUNWAYS: &str = "\"id\",\"airport_ref\",\"airport_ident\",\"length_ft\",\"width_ft\",\"surface\",\"lighted\",\"closed\",\"le_ident\",\"le_latitude_deg\",\"le_longitude_deg\",\"le_elevation_ft\",\"le_heading_degT\",\"le_displaced_threshold_ft\",\"he_ident\",\"he_latitude_deg\",\"he_longitude_deg\",\"he_elevation_ft\",\"he_heading_degT\",\"he_displaced_threshold_ft\"
10,1,\"EGXX\",12000,164,\"ASP\",1,0,\"09\",51.47,-0.49,79,90,1000,\"27\",51.47,-0.43,,270,
11,1,\"EGXX\",3000,100,\"GRS\",0,1,\"05\",51.46,-0.47,79,50,,\"23\",51.465,-0.46,79,230,
12,2,\"VNLK\",1729,98,\"asphalt\",0,0,\"06\",27.6857,86.7272,9134,60,,\"24\",27.6880,86.7317,9334,240,
13,3,\"XX-0001\",4000,,\"dirt\",0,0,\"18\",10.01,10.0,100,180,,\"36\",9.99,10.0,100,0,
";

const OA_FREQUENCIES: &str =
    "\"id\",\"airport_ref\",\"airport_ident\",\"type\",\"description\",\"frequency_mhz\"
1,1,\"EGXX\",\"TWR\",\"Tower\",118.5
2,1,\"EGXX\",\"TWR\",\"Tower\",118.7
3,1,\"EGXX\",\"ATIS\",\"ATIS\",128.075
4,2,\"VNLK\",\"A/G\",\"Radio\",122.0
";

#[test]
fn our_airports_rows_become_international_records() {
    let records = parse_our_airports(&OurAirportsCsv {
        airports: OA_AIRPORTS,
        runways: OA_RUNWAYS,
        frequencies: Some(OA_FREQUENCIES),
        snapshot_ymd: [2026, 11, 2],
    })
    .unwrap();
    let codes: Vec<&str> = records.iter().map(|r| r.ids.primary.as_str()).collect();
    // Field 3 is a small airport; the US row is left to the FAA.
    assert_eq!(codes, ["EGXX", "VNLK"]);
    let egxx = &records[0];
    assert_eq!(&*egxx.name, "Test, Heath");
    assert_eq!(egxx.ids.iata.unwrap().as_str(), "TXX");
    assert!(egxx.ids.faa_lid.is_none());
    assert!(egxx.towered);
    assert_eq!(egxx.authority, AuthoritySource::RegionalAip);
    assert_eq!(egxx.effective_ymd, [2026, 11, 2]);
    assert_eq!(egxx.runways.len(), 1, "the closed runway is dropped");
    let [r09, r27] = &egxx.runways[0].ends;
    assert!(
        (r27.elevation_msl_m - 83.0 * 0.3048).abs() < 1e-4,
        "field elevation stands in"
    );
    assert!((r09.displaced_threshold_m - 304.8).abs() < 1e-3);
    // Runway 09 heads 090° true, so the designators imply no variation.
    assert!(egxx.mag_var_deg.abs() < 0.5, "{}", egxx.mag_var_deg);
    assert_eq!(
        egxx.frequencies
            .iter()
            .filter(|f| f.role == FrequencyRole::Tower)
            .count(),
        2
    );
    assert_eq!(
        egxx.primary_frequency_mhz(),
        Some((FrequencyRole::Tower, 118.5))
    );
    let lukla = &records[1];
    assert!(!lukla.towered && lukla.hard_surface);
    assert!(lukla.primary_frequency_mhz().is_none());
}

#[test]
fn dates_names_and_surfaces_normalise() {
    assert_eq!(parse_ymd("2026/10/29"), Some([2026, 10, 29]));
    assert_eq!(parse_ymd("2026-11-02"), Some([2026, 11, 2]));
    assert_eq!(parse_ymd("2026/13/01"), None);
    assert_eq!(parse_ymd("2026/10"), None);
    assert_eq!(title_case("NEWARK LIBERTY INTL"), "Newark Liberty Intl");
    assert_eq!(title_case("O'HARE/CHICAGO"), "O'hare/Chicago");
    for (code, kind) in [
        ("ASPH-CONC", RunwaySurfaceKind::Asphalt),
        ("ASPH-G", RunwaySurfaceKind::Asphalt),
        ("ASPH-TURF", RunwaySurfaceKind::PavedTurf),
        ("ASPH-TURF-G", RunwaySurfaceKind::PavedTurf),
        ("CONC-TURF", RunwaySurfaceKind::PavedTurf),
        ("ASP/GRS", RunwaySurfaceKind::PavedTurf),
        ("CONC", RunwaySurfaceKind::Concrete),
        ("PEM", RunwaySurfaceKind::Paved),
        ("GRS", RunwaySurfaceKind::Turf),
        ("Grass", RunwaySurfaceKind::Turf),
        ("GRVL", RunwaySurfaceKind::Gravel),
        ("gravel", RunwaySurfaceKind::Gravel),
        ("WATER", RunwaySurfaceKind::Water),
        ("clay", RunwaySurfaceKind::Dirt),
        ("", RunwaySurfaceKind::Unknown),
    ] {
        assert_eq!(RunwaySurfaceKind::from_code(code), kind, "{code}");
    }
}
