//! Each charted runway as the two strips its ends describe, tagged with the runway they
//! belong to so a profile resolved for the runway reaches both.

use flightbag::RunwayEndSpec;
use sim_geo::{Geodetic, geodetic_to_ecef};

use crate::{
    AirportIdx, AirportRecord, CatalogVerticalDatum, RunwayEndRecord, RunwayStrip,
    RunwaySurfaceKind,
};

/// How close a strip's origin must lie to a runway end to be that end's strip.
const SAME_END_M: f64 = 1.0;

/// Which runway a strip belongs to, and whether it runs from the runway's second end.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct RunwayRef {
    pub(crate) airport: AirportIdx,
    pub(crate) runway: usize,
    pub(crate) reversed: bool,
}

/// A strip of the runway datum.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DatumStrip {
    pub(crate) strip: RunwayStrip,
    pub(crate) runway: RunwayRef,
}

fn charted(end: &RunwayEndRecord) -> f64 {
    f64::from(end.elevation_msl_m)
}

/// The strips of every runway of `record` that is ground (not water).
pub(crate) fn datum_strips(airport: AirportIdx, record: &AirportRecord) -> Vec<DatumStrip> {
    record
        .runways
        .iter()
        .enumerate()
        .filter(|(_, runway)| runway.surface != RunwaySurfaceKind::Water)
        .flat_map(|(index, runway)| {
            let ends: Vec<RunwayEndSpec> =
                runway.end_specs(charted).into_iter().flatten().collect();
            let [first, _] = &runway.ends;
            let first_ecef =
                geodetic_to_ecef(&Geodetic::from_degrees(first.lat_deg, first.lon_deg, 0.0));
            RunwayStrip::from_ends(&ends)
                .into_iter()
                .map(move |strip| DatumStrip {
                    strip,
                    runway: RunwayRef {
                        airport,
                        runway: index,
                        reversed: (strip.origin_ecef() - first_ecef).length() > SAME_END_M,
                    },
                })
        })
        .collect()
}

impl AirportRecord {
    /// The runway strips an [`crate::AuthoritativeWorldSurface`] read in `datum` should hold
    /// ([`crate::AuthoritativeWorldSurface::set_extra_runways`]), straight between their
    /// charted end elevations. A surface reads runway heights as charted — MSL in an
    /// orthometric world, the same number taken as ellipsoidal in a synthetic one — so
    /// every strip carries its surveyed MSL elevations in either datum. Water runways are
    /// not ground and have no strip.
    #[must_use]
    pub fn runway_strips(&self, _datum: CatalogVerticalDatum) -> Vec<RunwayStrip> {
        datum_strips(AirportIdx(0), self)
            .into_iter()
            .map(|datum_strip| datum_strip.strip)
            .collect()
    }
}
