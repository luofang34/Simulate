//! A waypoint list as the mission engine takes a route: a navigation-data
//! snapshot of named fixes and a route string naming them in order.
//!
//! The engine has no waypoint-list constructor, so the list travels the road
//! published data does: packed into the navdata container, verified,
//! decoded, and expanded by ident. The snapshot is declared a fixture.

use aerocontext_core::{GeoPoint, NavDataCycle, NavDataSnapshot, NavPoint, NavPointKind};
use aerocontext_navdata::blob;
use pilotage_mission::fixture::DEMO_EFFECTIVE_DATE;
use pilotage_mission::{MissionBuildError, SnapshotProvenance, decode_snapshot};

/// Letters after the prefix: 26⁴ fixes, far more than a route holds.
const IDENT_LETTERS: u32 = 4;

/// One horizontal fix of a route.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteFix {
    /// Geodetic latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
}

/// The ident of fix `index`: `X` and four letters, a shape the route
/// tokenizer reads as a fix and never as an airway, a procedure or a
/// coordinate.
fn ident(index: usize) -> String {
    let mut rest = index;
    let mut letters = Vec::new();
    for _ in 0..IDENT_LETTERS {
        letters.push(char::from(b'A'.wrapping_add((rest % 26) as u8)));
        rest /= 26;
    }
    std::iter::once('X')
        .chain(letters.into_iter().rev())
        .collect()
}

/// `fixes` as a verified fixture snapshot and the route string that flies
/// them in order.
pub(super) fn packed_route(
    fixes: &[RouteFix],
) -> Result<(NavDataSnapshot, SnapshotProvenance, String), MissionBuildError> {
    let points = fixes
        .iter()
        .enumerate()
        .map(|(index, fix)| {
            let position = GeoPoint {
                lat: fix.lat_deg,
                lon: fix.lon_deg,
            };
            NavPoint::new(ident(index), NavPointKind::Waypoint, position)
        })
        .collect();
    let cycle = NavDataCycle::faa_nasr(DEMO_EFFECTIVE_DATE)?;
    let bytes = blob::encode(&NavDataSnapshot::new(cycle, points))?;
    let (snapshot, provenance) = decode_snapshot(&bytes, true)?;
    let route = (0..fixes.len()).map(ident).collect::<Vec<_>>().join(" ");
    Ok((snapshot, provenance, route))
}
