//! Surveyed polar aerodrome records whose published skiway and tower data
//! OurAirports omits or misplaces near `±90°`.

use flightbag::AuthoritySource;

use super::geometry::{bearing_and_distance, offset_deg};
use super::model::{
    AerodromeTier, AirportFrequency, AirportIds, AirportRecord, CompactCode, FrequencyRole,
    RunwayEndRecord, RunwayRecord, RunwaySurfaceKind, facility_code_of, magnetic_heading_deg,
};
use super::pack::COORD_SCALE;

/// Quantizes `degrees` to the `1e-7` degree grid `SIMAPT01` stores, so a
/// surveyed record round-trips through [`super::pack`] bit-for-bit.
fn quantize_coord(degrees: f64) -> f64 {
    (degrees * COORD_SCALE).round() / COORD_SCALE
}

fn nzsp_runway(lat_0: f64, lon_0: f64, elevation_msl_m: f32) -> Option<RunwayRecord> {
    let rwy_id = CompactCode::parse("02T/20T")?;
    let desig_02t = CompactCode::parse("02T")?;
    let desig_20t = CompactCode::parse("20T")?;
    let length_m = 3_658.0_f32;
    let width_m = 46.0_f32;
    let heading_02t_deg = 20.0_f32;
    let (raw_lat_1, raw_lon_1) = offset_deg(
        (lat_0, lon_0),
        f64::from(heading_02t_deg).to_radians(),
        f64::from(length_m),
    );
    let (lat_1, lon_1) = (quantize_coord(raw_lat_1), quantize_coord(raw_lon_1));
    let (bearing_20t_rad, _) = bearing_and_distance((lat_1, lon_1), (lat_0, lon_0));
    let heading_20t_deg = bearing_20t_rad.to_degrees() as f32;
    let end = |designator, lat_deg, lon_deg, true_heading_deg| RunwayEndRecord {
        designator,
        lat_deg,
        lon_deg,
        elevation_msl_m,
        displaced_threshold_m: 0.0,
        true_heading_deg,
        magnetic_heading_deg: magnetic_heading_deg(true_heading_deg, 0.0),
        glidepath_deg: 3.0,
        has_ils: false,
        has_vgsi: false,
        has_approach_lights: false,
    };
    Some(RunwayRecord {
        id: rwy_id,
        length_m,
        width_m,
        surface: RunwaySurfaceKind::Gravel,
        lighted: false,
        ends: [
            end(desig_02t, lat_0, lon_0, heading_02t_deg),
            end(desig_20t, lat_1, lon_1, heading_20t_deg),
        ],
    })
}

/// Surveyed record for `NZSP` (Amundsen-Scott South Pole Station, `RWY 02T/20T`).
pub(super) fn nzsp_record() -> Option<AirportRecord> {
    let nzsp = CompactCode::parse("NZSP")?;
    let iata = CompactCode::parse("NPX")?;
    let ids = AirportIds::new(Some(nzsp), None, Some(iata))?;
    let facility = facility_code_of(ids.primary)?;
    let region_code = CompactCode::parse("AQ")?;
    let (lat_deg, lon_deg, elevation_msl_m) = (-89.9975, 139.2729, 2_835.0_f32);
    let runway = nzsp_runway(lat_deg, lon_deg, elevation_msl_m)?;
    let mut record = AirportRecord {
        ids,
        facility,
        name: "Amundsen-Scott South Pole Station".into(),
        municipality: "South Pole".into(),
        country_iso2: *b"AQ",
        region_code,
        lat_deg,
        lon_deg,
        elevation_msl_m,
        mag_var_deg: 0.0,
        towered: true,
        tier: AerodromeTier::Tier1Major,
        longest_runway_m: runway.length_m,
        hard_surface: false,
        authority: AuthoritySource::RegionalAip,
        effective_ymd: [2026, 11, 2],
        runways: Box::from([runway]),
        frequencies: Box::from([AirportFrequency {
            role: FrequencyRole::Tower,
            khz: 126_200,
        }]),
    };
    record.classify();
    Some(record)
}

/// Ensures `records` holds a surveyed `NZSP` record: keeps an existing `NZSP`
/// when it already carries runways from a newer cycle than [`nzsp_record`],
/// otherwise replaces any unverified `NZSP` row in place or inserts in
/// primary-code order.
pub(super) fn upsert_nzsp(records: &mut Vec<AirportRecord>) {
    let Some(surveyed) = nzsp_record() else {
        return;
    };
    if let Some(slot) = records
        .iter_mut()
        .find(|r| r.ids.primary == surveyed.ids.primary || r.ids.icao == Some(surveyed.ids.primary))
    {
        if slot.runways.is_empty() || slot.effective_ymd <= surveyed.effective_ymd {
            *slot = surveyed;
        }
        return;
    }
    let pos = records.partition_point(|r| r.ids.primary < surveyed.ids.primary);
    records.insert(pos, surveyed);
}

/// Replaces an existing `NZSP` row with the surveyed record when compiling a
/// dataset that includes Antarctica.
pub(super) fn replace_nzsp_if_present(records: &mut [AirportRecord]) {
    let Some(surveyed) = nzsp_record() else {
        return;
    };
    for slot in records {
        if slot.ids.primary == surveyed.ids.primary || slot.ids.icao == Some(surveyed.ids.primary) {
            *slot = surveyed.clone();
        }
    }
}
