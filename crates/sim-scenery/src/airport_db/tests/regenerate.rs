//! Rebuilds `data/global_airports.bin` from the source CSVs:
//!
//! ```sh
//! SIM_NAVDATA_DIR=/path/to/navdata cargo test -p sim-scenery --release \
//!     regenerate_builtin_pack -- --ignored
//! ```
//!
//! `SIM_NAVDATA_DIR` holds `faa_apt/APT_{BASE,RWY,RWY_END}.csv`,
//! `faa_frq/FRQ.csv` and `ourairports/{airports,runways,airport-frequencies}.csv`;
//! `SIM_OURAIRPORTS_DATE` (`YYYY-MM-DD`) dates the OurAirports snapshot.

use std::path::{Path, PathBuf};

use crate::airport_db::assemble::parse_ymd;
use crate::{
    AirportDatabase, FaaNasrCsv, OurAirportsCsv, our_airports_us_names, parse_faa_nasr,
    parse_our_airports,
};

fn read(dir: &Path, relative: &str) -> String {
    std::fs::read_to_string(dir.join(relative)).unwrap_or_else(|err| panic!("{relative}: {err}"))
}

#[test]
#[ignore = "needs SIM_NAVDATA_DIR with the FAA NASR and OurAirports CSVs"]
fn regenerate_builtin_pack() {
    let dir = PathBuf::from(std::env::var("SIM_NAVDATA_DIR").expect("SIM_NAVDATA_DIR"));
    let snapshot = std::env::var("SIM_OURAIRPORTS_DATE").expect("SIM_OURAIRPORTS_DATE");
    let (base, rwy, rwy_end, frq) = (
        read(&dir, "faa_apt/APT_BASE.csv"),
        read(&dir, "faa_apt/APT_RWY.csv"),
        read(&dir, "faa_apt/APT_RWY_END.csv"),
        read(&dir, "faa_frq/FRQ.csv"),
    );
    let faa = parse_faa_nasr(&FaaNasrCsv {
        apt_base: &base,
        apt_rwy: &rwy,
        apt_rwy_end: &rwy_end,
        frq: Some(&frq),
    })
    .unwrap();
    let (airports, runways, frequencies) = (
        read(&dir, "ourairports/airports.csv"),
        read(&dir, "ourairports/runways.csv"),
        read(&dir, "ourairports/airport-frequencies.csv"),
    );
    let international = parse_our_airports(&OurAirportsCsv {
        airports: &airports,
        runways: &runways,
        frequencies: Some(&frequencies),
        snapshot_ymd: parse_ymd(&snapshot).expect("SIM_OURAIRPORTS_DATE is YYYY-MM-DD"),
    })
    .unwrap();
    let us_names = our_airports_us_names(&airports).unwrap();
    let (faa_count, international_count) = (faa.len(), international.len());
    let database = AirportDatabase::compile(faa, international, &us_names);
    let bytes = database.to_pack_bytes().unwrap();
    let out = Path::new(env!("CARGO_MANIFEST_DIR")).join("src/airport_db/data/global_airports.bin");
    std::fs::write(&out, &bytes).unwrap();
    assert!(faa_count >= 4_000 && international_count >= 3_000);
    assert!(database.len() >= 6_000 && database.runway_count() >= 9_000);
}

#[test]
#[ignore = "rewrites data/global_airports.bin with surveyed polar overrides"]
fn refresh_surveyed_records_in_builtin_pack() {
    let database = AirportDatabase::decode_builtin().unwrap();
    let bytes = database.to_pack_bytes().unwrap();
    let out = Path::new(env!("CARGO_MANIFEST_DIR")).join("src/airport_db/data/global_airports.bin");
    std::fs::write(out, bytes).unwrap();
}
