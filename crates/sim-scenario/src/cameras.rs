//! The virtual camera suite each [`ScenarioProfile`] flies with, and the body
//! its world-anchored cameras stand on.

use sim_core::VirtualCameraSpec;
use sim_geo::BodyEllipsoid;
use sim_scenery::{CatalogVerticalDatum, OsmAerodromeDiagram};

use crate::ScenarioProfile;
use crate::pattern::pattern_cameras_on_datum;

mod lunar;
mod reentry;
#[cfg(test)]
mod tests;

/// The body and the five `30 Hz` cameras for `profile`.
///
/// The Apollo profiles get cameras built for a capsule and a lander on their
/// own bodies; aircraft profiles, and no profile, get the airport pattern
/// suite for `diagram`, whose heights are read in `datum` and raised onto the
/// WGS84 ellipsoid. Its tower and approach trackers hand over to flyby
/// cameras far from the airport, so they suit corridor flights too.
#[must_use]
pub fn scenario_virtual_cameras(
    profile: Option<ScenarioProfile>,
    diagram: &OsmAerodromeDiagram,
    datum: CatalogVerticalDatum,
) -> (BodyEllipsoid, [VirtualCameraSpec; 5]) {
    match profile {
        Some(ScenarioProfile::Apollo11CapsuleReentry) => (BodyEllipsoid::WGS84, reentry::cameras()),
        Some(ScenarioProfile::Apollo11LunarLanding) => (BodyEllipsoid::MOON_IAU, lunar::cameras()),
        Some(
            ScenarioProfile::CorridorNominalCruise
            | ScenarioProfile::CorridorOffSeedAcquisition
            | ScenarioProfile::CorridorAggressiveBank
            | ScenarioProfile::CorridorSameCellLoiter
            | ScenarioProfile::Airport47NUncontrolledPattern
            | ScenarioProfile::AirportKewrToweredPattern,
        )
        | None => (
            BodyEllipsoid::WGS84,
            pattern_cameras_on_datum(diagram, datum),
        ),
    }
}
