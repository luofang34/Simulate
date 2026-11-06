//! The diagram restated with true WGS84 ellipsoidal heights.
//!
//! A diagram carries every height as charted, metres above mean sea level,
//! so it blends with a catalog read in that number's datum. These helpers
//! give the heights a consumer reading `GeodeticPosition::ellipsoidal_height_m`
//! literally needs.

use flightbag::AirportSurfaceGraph;
use sim_geo::Geodetic;
use world_core::GeodeticPosition;

use super::OsmAerodromeDiagram;
use crate::CatalogVerticalDatum;

impl OsmAerodromeDiagram {
    /// The surface graph with every height restated as WGS84 ellipsoidal
    /// height for a world read in `datum`: charted MSL plus the EGM96
    /// undulation under [`CatalogVerticalDatum::OrthometricMslEgm96`], the
    /// charted number unchanged under [`CatalogVerticalDatum::EllipsoidWgs84`]
    /// (a synthetic world that takes charted heights as ellipsoidal).
    #[must_use]
    pub fn to_flightbag_graph_for_datum(&self, datum: CatalogVerticalDatum) -> AirportSurfaceGraph {
        let mut graph = self.surface_graph.clone();
        restate(&mut graph.reference_point, datum);
        for end in &mut graph.runway_ends {
            restate(&mut end.threshold, datum);
        }
        for node in &mut graph.hold_short_nodes {
            restate(&mut node.position, datum);
        }
        for spot in &mut graph.parking_spots {
            restate(&mut spot.position, datum);
        }
        graph
    }

    /// [`Self::tower_viewpoint`] at its WGS84 ellipsoidal height for a world
    /// read in `datum`, as [`Self::to_flightbag_graph_for_datum`] restates it.
    #[must_use]
    pub fn tower_viewpoint_for_datum(&self, datum: CatalogVerticalDatum) -> Geodetic {
        let tower = self.tower_viewpoint;
        let (ellipsoidal, _) = datum.heights(tower.lat_rad, tower.lon_rad, tower.height_m);
        Geodetic {
            height_m: ellipsoidal.0,
            ..tower
        }
    }
}

/// Rewrites a charted height in `datum` as the ellipsoidal height it names.
pub(crate) fn restate(position: &mut GeodeticPosition, datum: CatalogVerticalDatum) {
    let (ellipsoidal, _) = datum.heights(
        position.latitude_deg.to_radians(),
        position.longitude_deg.to_radians(),
        position.ellipsoidal_height_m,
    );
    position.ellipsoidal_height_m = ellipsoidal.0;
}
