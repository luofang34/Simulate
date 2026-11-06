//! Where each [`SpawnMode`] puts the vehicle at an airport, with every height a true WGS84
//! ellipsoidal height, the reference ground contact and JSBSim's terrain are measured from.

use flightbag::{AirportSurfaceGraph, RunwayEndSpec};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_scenery::{AirportRecord, CatalogVerticalDatum, OsmAerodromeDiagram};
use world::GeodeticPosition;

use super::manual::SpawnMode;
use super::mode::FreeSpawn;
use crate::world_surface::WorldSurfaceHandle;

/// Height above the field of an overhead spawn, metres.
const OVERHEAD_AGL_M: f64 = 300.0;
/// Distance of a short-final spawn from the threshold (`3 NM`), metres.
const SHORT_FINAL_M: f64 = 5_556.0;
/// Glide-path angle of a short-final spawn, degrees.
const GLIDE_PATH_DEG: f64 = 3.0;
/// Threshold crossing height of the glide path, metres.
const THRESHOLD_CROSSING_M: f64 = 15.0;
/// How far down the runway a threshold spawn lines up, metres.
const LINE_UP_M: f64 = 30.0;
/// Where a ramp spawn parks when the airport has no mapped stand: down the runway from
/// the threshold and right of the centreline, clear of the pavement, metres.
const RAMP_ALONG_M: f64 = 60.0;
const RAMP_RIGHT_M: f64 = 35.0;

/// An airport's spawn anchors, every height true WGS84 ellipsoidal.
#[derive(Debug, Clone, Copy)]
pub(super) struct SpawnSite {
    /// The aerodrome reference point at the field elevation.
    reference: Geodetic,
    /// The runway end to line up on, if the airport has one.
    end: Option<(Geodetic, f64)>,
    /// A mapped parking stand and its heading, if the airport has one.
    stand: Option<(Geodetic, f64)>,
}

impl SpawnSite {
    /// `diagram`'s runway end `runway` (else its primary runway), its first stand, and its
    /// reference point, restated for a world read in `datum`.
    pub(super) fn of_diagram(
        diagram: &OsmAerodromeDiagram,
        runway: Option<usize>,
        datum: CatalogVerticalDatum,
    ) -> Self {
        let graph = diagram.to_flightbag_graph_for_datum(datum);
        let primary = diagram
            .primary_runway_end()
            .and_then(|primary| graph.runway_end(primary.designator));
        let end = runway
            .and_then(|index| graph.runway_ends.get(index))
            .or(primary)
            .or_else(|| graph.runway_ends.first());
        Self::of_graph(&graph, end)
    }

    /// `record`'s runway end `runway_end` (else its first), for a world read in `datum`.
    pub(super) fn of_record(
        record: &AirportRecord,
        runway_end: usize,
        datum: CatalogVerticalDatum,
    ) -> Self {
        let graph = record.to_flightbag_graph(datum);
        let end = graph
            .runway_ends
            .get(runway_end)
            .or_else(|| graph.runway_ends.first());
        Self::of_graph(&graph, end)
    }

    fn of_graph(graph: &AirportSurfaceGraph, end: Option<&RunwayEndSpec>) -> Self {
        Self {
            reference: geodetic(&graph.reference_point),
            end: end.map(|end| (geodetic(&end.threshold), end.true_bearing_rad)),
            stand: graph
                .parking_spots
                .first()
                .map(|stand| (geodetic(&stand.position), stand.heading_rad)),
        }
    }

    /// Where `mode` puts the vehicle, standing on `surface`'s ground where it spawns on
    /// the ground.
    pub(super) fn spawn(&self, mode: SpawnMode, surface: &WorldSurfaceHandle) -> FreeSpawn {
        let field_m = self.reference.height_m;
        let ground = |at: &Geodetic| Geodetic {
            height_m: surface.ellipsoidal_ground_m(at).unwrap_or(at.height_m),
            ..*at
        };
        let (threshold, heading_rad) = self.end.unwrap_or((self.reference, 0.0));
        match mode {
            SpawnMode::Threshold => {
                let (lined_up, local_heading_rad) = offset(&threshold, heading_rad, LINE_UP_M, 0.0);
                FreeSpawn::Ground {
                    spawn: ground(&lined_up),
                    heading_rad: local_heading_rad,
                    cold: false,
                }
            }
            SpawnMode::Ramp => {
                let (spot, heading_rad) = self
                    .stand
                    .unwrap_or_else(|| offset(&threshold, heading_rad, RAMP_ALONG_M, RAMP_RIGHT_M));
                FreeSpawn::Ground {
                    spawn: ground(&spot),
                    heading_rad,
                    cold: true,
                }
            }
            SpawnMode::ShortFinal => {
                let threshold = ground(&threshold);
                let above_m =
                    SHORT_FINAL_M * GLIDE_PATH_DEG.to_radians().tan() + THRESHOLD_CROSSING_M;
                let (out, local_heading_rad) = offset(&threshold, heading_rad, -SHORT_FINAL_M, 0.0);
                FreeSpawn::At {
                    spawn: Geodetic {
                        height_m: threshold.height_m + above_m,
                        ..out
                    },
                    heading_rad: local_heading_rad,
                    field_elev_m: surface
                        .ellipsoidal_ground_m(&out)
                        .unwrap_or(threshold.height_m),
                }
            }
            SpawnMode::Overhead => FreeSpawn::At {
                spawn: Geodetic {
                    height_m: field_m + OVERHEAD_AGL_M,
                    ..threshold
                },
                heading_rad,
                field_elev_m: field_m,
            },
        }
    }
}

fn geodetic(position: &GeodeticPosition) -> Geodetic {
    Geodetic::from_degrees(
        position.latitude_deg,
        position.longitude_deg,
        position.ellipsoidal_height_m,
    )
}

/// `from` moved `along_m` along true `bearing_rad` and `right_m` to its right over the
/// local tangent plane, keeping `from`'s height, plus `ahead` expressed in the destination's
/// local true-north frame so polar approaches crossing `±90°` keep pointing at the runway.
fn offset(from: &Geodetic, bearing_rad: f64, along_m: f64, right_m: f64) -> (Geodetic, f64) {
    let local = LocalFrame::at(from);
    let (sin_b, cos_b) = bearing_rad.sin_cos();
    let ahead = local.north() * cos_b + local.east() * sin_b;
    let right = local.east() * cos_b - local.north() * sin_b;
    let moved = ecef_to_geodetic(geodetic_to_ecef(from) + ahead * along_m + right * right_m);
    let dest = Geodetic {
        height_m: from.height_m,
        ..moved
    };
    let dest_local = LocalFrame::at(&dest);
    let n = ahead.dot(dest_local.north());
    let e = ahead.dot(dest_local.east());
    let dest_heading_rad = if n.hypot(e) > 1e-12 {
        e.atan2(n).rem_euclid(std::f64::consts::TAU)
    } else {
        bearing_rad.rem_euclid(std::f64::consts::TAU)
    };
    (dest, dest_heading_rad)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polar_short_final_across_south_pole_points_at_threshold() {
        let threshold = Geodetic::from_degrees(-89.9975, 139.2729, 2_835.0);
        let bearing_rad = 20.0_f64.to_radians();
        let (out, local_heading_rad) = offset(&threshold, bearing_rad, -SHORT_FINAL_M, 0.0);
        // 3 NM behind NZSP RWY 02T (outbound 200° true from 139.2729° E) crosses 90S into
        // the Western Hemisphere near 139.2729° + 200° - 360° = -20.73° (~-19.69° at 3 NM).
        let out_lon_deg = out.lon_rad.to_degrees();
        assert!(
            (-22.0..-18.0).contains(&out_lon_deg),
            "expected opposite meridian across 90S (~-19.7 deg), got lon={out_lon_deg}",
        );
        let dest_local = LocalFrame::at(&out);
        let (sin_h, cos_h) = local_heading_rad.sin_cos();
        let nose_ecef = dest_local.north() * cos_h + dest_local.east() * sin_h;
        let to_threshold_ecef = (geodetic_to_ecef(&threshold) - geodetic_to_ecef(&out)).normalize();
        assert!(
            nose_ecef.dot(to_threshold_ecef) > 0.9999,
            "spawn nose must point straight toward NZSP RWY 02T threshold across 90S: dot={}",
            nose_ecef.dot(to_threshold_ecef)
        );
    }
}
