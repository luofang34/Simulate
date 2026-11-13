//! Built-in heritage and ad-hoc aerodrome presets, including Mathias Rust's 28 May 1987
//! flight from Helsinki-Malmi (`EFHF`) to Bolshoy Moskvoretsky Bridge (`ZMRS`) beside Red Square.

use flightbag::AuthoritySource;

use super::{AdHocAerodrome, AdHocRunwayError, AdHocRunwaySpec};
use crate::airport_db::facility_code_of;
use crate::airport_db::geometry::{bearing_and_distance, offset_deg};
use crate::{
    AerodromeTier, AirportFrequency, AirportIds, AirportRecord, CompactCode, FrequencyRole,
    RunwayEndRecord, RunwayRecord, RunwaySurfaceKind, magnetic_heading_deg,
};

/// Historic route waypoints `(ident, lat_deg, lon_deg)` for Mathias Rust's 28 May 1987
/// flight (`C-172P D-ECJB`) from `EFHF` Helsinki-Malmi to Moscow (`KREML` sightseeing pass
/// over Red Square followed by `ORDNK` Bolshaya Ordynka south final onto `ZMRS/33`).
pub const MATHIAS_RUST_1987_WAYPOINTS: [(&str, f64, f64); 9] = [
    ("KOHTL", 59.4000, 27.2800),
    ("GDOV", 58.7400, 27.8200),
    ("DNO", 57.8300, 29.9600),
    ("SELIG", 57.1500, 33.1000),
    ("TORZH", 57.0400, 34.9600),
    ("KLIN", 56.3300, 36.7300),
    ("KHIMK", 55.9000, 37.4400),
    ("KREML", 55.7539, 37.6208),
    ("ORDNK", 55.7330, 37.6315),
];

impl AdHocRunwaySpec {
    /// Preset specification for **Bolshoy Moskvoretsky Bridge & Vasilevsky Descent (`ZMRS`)**
    /// (`RWY 33/15`, `~456 × 36 m` asphalt deck at `128.0 m` MSL): Mathias Rust's photo-verified
    /// 28 May 1987 south-to-north touchdown across the Moskva River (`55.74820° N, 37.62461° E`)
    /// rolling up Vasilevsky Descent (`Васильевский спуск`) between the Kremlin Wall and
    /// St. Basil's Cathedral to `55.75215° N, 37.62265° E`.
    #[must_use]
    pub fn bolshoy_moskvoretsky_bridge() -> Self {
        Self {
            code: "ZMRS".to_owned(),
            name: "Bolshoy Moskvoretsky Bridge & Vasilevsky Descent".to_owned(),
            municipality: "Moscow".to_owned(),
            country_iso2: *b"RU",
            region_code: "RU-MOW".to_owned(),
            // South bridge abutment (Bolshaya Ordynka / Moskva River) -> top of Vasilevsky Descent
            // beside St. Basil's Cathedral (true course ~344.4°, mag ~332.9° under +11.5°E -> RWY 33/15).
            threshold_a_deg: [55.74820, 37.62461],
            threshold_b_deg: [55.75215, 37.62265],
            width_m: 36.0,
            surface: RunwaySurfaceKind::Asphalt,
            mag_var_deg: 11.5,
            override_elevation_msl_m: Some(128.0),
            frequency_khz: Some(122_800),
            towered: false,
            lighted: true,
        }
    }

    /// Preset specification for **Red Square Cobblestone Plaza (`ZMRQ`)** (`Красная площадь`,
    /// `55.75300° N, 37.62208° E` near St. Basil's Cathedral / Lobnoye Mesto NW to
    /// `55.75475° N, 37.61857° E` before the State Historical Museum; `RWY 30/12`, `~294 × 45 m`
    /// at `144.0 m` MSL) — the plaza centerline Rust lined up on during his initial low passes.
    #[must_use]
    pub fn red_square_plaza() -> Self {
        Self {
            code: "ZMRQ".to_owned(),
            name: "Red Square Plaza (Krasnaya Ploshchad)".to_owned(),
            municipality: "Moscow".to_owned(),
            country_iso2: *b"RU",
            region_code: "RU-MOW".to_owned(),
            threshold_a_deg: [55.75300, 37.62208],
            threshold_b_deg: [55.75475, 37.61857],
            width_m: 45.0,
            surface: RunwaySurfaceKind::Asphalt,
            mag_var_deg: 11.5,
            override_elevation_msl_m: Some(144.0),
            frequency_khz: Some(122_800),
            towered: false,
            lighted: true,
        }
    }
}

impl AdHocAerodrome {
    /// Builds **Bolshoy Moskvoretsky Bridge & Vasilevsky Descent (`ZMRS`, `RWY 33/15`)**.
    pub fn bolshoy_moskvoretsky_bridge() -> Result<Self, AdHocRunwayError> {
        Self::from_spec(&AdHocRunwaySpec::bolshoy_moskvoretsky_bridge(), |_, _| {
            Some(128.0)
        })
    }

    /// Builds **Red Square Cobblestone Plaza (`ZMRQ`, `RWY 30/12`)** from
    /// `55.75300° N, 37.62208° E` to `55.75475° N, 37.61857° E`.
    pub fn red_square_plaza() -> Result<Self, AdHocRunwayError> {
        Self::from_spec(&AdHocRunwaySpec::red_square_plaza(), |_, _| Some(144.0))
    }

    /// Builds historic **Helsinki-Malmi Airport (`EFHF`)** (`60.2546°N, 25.0428°E`, `17.4 m` MSL)
    /// with its two crossed asphalt runways (`RWY 18/36` `1,400 × 30 m` and `RWY 09/27`
    /// `1,080 × 30 m`) for classic General Aviation departures.
    pub fn helsinki_malmi_efhf() -> Result<Self, AdHocRunwayError> {
        let code = CompactCode::parse("EFHF")
            .ok_or_else(|| AdHocRunwayError::InvalidCode("EFHF".to_owned()))?;
        let iata = CompactCode::parse("HEM");
        let ids = AirportIds::new(Some(code), None, iata)
            .ok_or_else(|| AdHocRunwayError::InvalidCode("EFHF".to_owned()))?;
        let facility = facility_code_of(ids.primary)
            .ok_or_else(|| AdHocRunwayError::InvalidCode("EFHF".to_owned()))?;
        let region_code = CompactCode::parse("FI-18").unwrap_or_default();
        let elev_m = 17.4_f32;
        let mag_var_deg = 9.5_f32;
        let rwy_18_36 = centred_runway(
            "18/36",
            ("18", "36"),
            (60.254_558, 25.042_828),
            188.5,
            (1_400.0, 30.0),
            elev_m,
            mag_var_deg,
        )?;
        let rwy_09_27 = centred_runway(
            "09/27",
            ("09", "27"),
            (60.253_800, 25.041_900),
            98.5,
            (1_080.0, 30.0),
            elev_m,
            mag_var_deg,
        )?;
        let record = AirportRecord {
            ids,
            facility,
            name: "Helsinki-Malmi Airport".into(),
            municipality: "Helsinki".into(),
            country_iso2: *b"FI",
            region_code,
            lat_deg: 60.254_558,
            lon_deg: 25.042_828,
            elevation_msl_m: elev_m,
            mag_var_deg,
            towered: true,
            tier: AerodromeTier::Tier2Regional,
            longest_runway_m: 1_400.0,
            hard_surface: true,
            authority: AuthoritySource::RegionalAip,
            effective_ymd: [1987, 5, 28],
            runways: Box::from([rwy_18_36, rwy_09_27]),
            frequencies: Box::from([AirportFrequency {
                role: FrequencyRole::Tower,
                khz: 118_900,
            }]),
        };
        Self::from_airport_record(record)
    }
}

fn centred_runway(
    id_str: &str,
    (desig_a_str, desig_b_str): (&str, &str),
    centre_deg: (f64, f64),
    true_ab_deg: f64,
    (length_m, width_m): (f32, f32),
    elevation_msl_m: f32,
    mag_var_deg: f32,
) -> Result<RunwayRecord, AdHocRunwayError> {
    let id = CompactCode::parse(id_str)
        .ok_or_else(|| AdHocRunwayError::InvalidCode(id_str.to_owned()))?;
    let desig_a = CompactCode::parse(desig_a_str)
        .ok_or_else(|| AdHocRunwayError::InvalidCode(desig_a_str.to_owned()))?;
    let desig_b = CompactCode::parse(desig_b_str)
        .ok_or_else(|| AdHocRunwayError::InvalidCode(desig_b_str.to_owned()))?;
    let fwd_rad = true_ab_deg.to_radians();
    let back_rad = fwd_rad + std::f64::consts::PI;
    let half_m = 0.5 * f64::from(length_m);
    let a = offset_deg(centre_deg, back_rad, half_m);
    let b = offset_deg(centre_deg, fwd_rad, half_m);
    let (brg_ab, _) = bearing_and_distance(a, b);
    let (brg_ba, _) = bearing_and_distance(b, a);
    let hdg_ab = brg_ab.to_degrees() as f32;
    let hdg_ba = brg_ba.to_degrees() as f32;
    let end = |designator, (lat_deg, lon_deg), true_heading_deg| RunwayEndRecord {
        designator,
        lat_deg,
        lon_deg,
        elevation_msl_m,
        displaced_threshold_m: 0.0,
        true_heading_deg,
        magnetic_heading_deg: magnetic_heading_deg(true_heading_deg, mag_var_deg),
        glidepath_deg: 3.0,
        has_ils: false,
        has_vgsi: true,
        has_approach_lights: false,
    };
    Ok(RunwayRecord {
        id,
        length_m,
        width_m,
        surface: RunwaySurfaceKind::Asphalt,
        lighted: true,
        ends: [end(desig_a, a, hdg_ab), end(desig_b, b, hdg_ba)],
    })
}
