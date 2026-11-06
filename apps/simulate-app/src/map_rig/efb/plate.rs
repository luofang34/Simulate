//! The procedural approach plate of an airport's runway end.

use sim_maplibre::{ApproachPlateOverlay, ProceduralPlateSpec, generate_procedural_approach_plate};
use sim_scenery::{AirportRecord, CatalogVerticalDatum, RunwayEndRecord};

use crate::error::AppSimError;

const FEET_PER_METRE: f64 = 3.280_84;

/// The runway end of `record` named `designator`, else its primary runway's first end.
#[must_use]
pub fn runway_end<'a>(record: &'a AirportRecord, designator: &str) -> Option<&'a RunwayEndRecord> {
    record
        .runways
        .iter()
        .flat_map(|runway| runway.ends.iter())
        .find(|end| end.designator.as_str() == designator)
        .or_else(|| record.primary_runway().and_then(|r| r.ends.first()))
}

/// The procedural plate spec of `record`'s runway end `designator`, on the airport's
/// first frequency to call (the localizer frequency is not in the database).
#[must_use]
pub fn plate_spec(record: &AirportRecord, designator: &str) -> Option<ProceduralPlateSpec> {
    let end = runway_end(record, designator)?;
    if end.lat_deg.abs() > 84.0 {
        return None;
    }
    let landing = record.landing_runway(
        end.designator.as_str(),
        CatalogVerticalDatum::OrthometricMslEgm96,
    );
    let (threshold_lat_deg, threshold_lon_deg, true_heading_deg, elev_msl_m) = match landing {
        Some(ref rwy) => (
            rwy.threshold.lat_deg(),
            rwy.threshold.lon_deg(),
            rwy.true_bearing_rad.to_degrees().rem_euclid(360.0),
            rwy.threshold_elevation_msl_m,
        ),
        None => (
            end.lat_deg,
            end.lon_deg,
            f64::from(end.true_heading_deg),
            f64::from(end.elevation_msl_m),
        ),
    };
    let freq_khz = record
        .primary_frequency_mhz()
        .map_or(0, |(_, mhz)| (mhz * 1_000.0).round().max(0.0) as u32);
    Some(ProceduralPlateSpec {
        airport_code: record.ids.primary.as_str().to_owned(),
        airport_name: record.name.to_uppercase(),
        runway_designator: end.designator.as_str().to_owned(),
        threshold_lat_deg,
        threshold_lon_deg,
        true_heading_deg,
        elevation_ft: elev_msl_m * FEET_PER_METRE,
        ils_freq_khz: freq_khz,
        glidepath_deg: f64::from(end.glidepath_deg),
    })
}

pub(super) fn plate_for(
    record: &AirportRecord,
    designator: &str,
) -> Result<Option<ApproachPlateOverlay>, AppSimError> {
    plate_spec(record, designator)
        .map(|spec| generate_procedural_approach_plate(&spec))
        .transpose()
        .map_err(AppSimError::from)
}
