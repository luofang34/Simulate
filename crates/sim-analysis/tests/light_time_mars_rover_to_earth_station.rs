//! Line of sight between a rover on Mars and a station on Earth with the signal's travel
//! time: a window edge set by one planet's horizon is seen a light time later at the
//! other, so downlink and uplink windows differ, and each differs from the geometric
//! answer at the far end only.

use sim_analysis::{
    AccessQuerySpec, AccessWindow, AnalysisEndpoint, GroundStationModel,
    OccultationEnvironmentModel, OccultingBodySpec, OccultingShape, SensorFovModel,
    SignalTimeModel, evaluate_endpoint_link_access,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{NaifId, SensorMount, TdbEpoch, ValidEpochInterval};

const MIN_ELEVATION_RAD: f64 = 0.174_532_925_199_432_95;

#[cfg(test)]
fn station(
    station_id: u32,
    name: &str,
    body: NaifId,
    ellipsoid: BodyEllipsoid,
    lat_deg: f64,
    lon_deg: f64,
) -> GroundStationModel {
    GroundStationModel {
        station_id,
        name: name.to_owned(),
        body,
        geodetic: Geodetic::from_degrees(lat_deg, lon_deg, 0.0),
        ellipsoid,
        mount: SensorMount::IDENTITY,
        fov: SensorFovModel::OmnidirectionalHemisphere,
        min_elevation_rad: MIN_ELEVATION_RAD,
    }
}

#[cfg(test)]
struct Link {
    solar: BuiltinSolarSystem2026_2028,
    rover: GroundStationModel,
    goldstone: GroundStationModel,
    bodies: OccultationEnvironmentModel,
}

#[cfg(test)]
impl Link {
    fn new() -> Self {
        let body = |body, shape| OccultingBodySpec { body, shape };
        Self {
            solar: BuiltinSolarSystem2026_2028::load_verified().expect("solar"),
            rover: station(
                1,
                "Jezero rover",
                NaifId::MARS,
                BodyEllipsoid::MARS_IAU,
                18.44,
                77.45,
            ),
            goldstone: station(
                14,
                "DSS-14 Goldstone",
                NaifId::EARTH,
                BodyEllipsoid::WGS84,
                35.426,
                -116.89,
            ),
            bodies: OccultationEnvironmentModel {
                bodies: vec![
                    body(
                        NaifId::EARTH,
                        OccultingShape::Ellipsoid(BodyEllipsoid::WGS84),
                    ),
                    body(
                        NaifId::MARS,
                        OccultingShape::Ellipsoid(BodyEllipsoid::MARS_IAU),
                    ),
                    body(
                        NaifId::MOON,
                        OccultingShape::Sphere {
                            radius_m: 1_737_400.0,
                        },
                    ),
                    body(
                        NaifId::SUN,
                        OccultingShape::Sphere {
                            radius_m: 695_700_000.0,
                        },
                    ),
                ],
            },
        }
    }

    /// Windows of the link from `source` to `target` over two days, and its light time.
    fn windows(
        &self,
        downlink: bool,
        signal_time: SignalTimeModel,
    ) -> (Vec<AccessWindow>, Option<f64>) {
        // 2027-03-01T00:00:00 TDB, ten days after Mars's opposition.
        let start = TdbEpoch::new(820_497_600 + 424 * 86_400, 0);
        let query = AccessQuerySpec {
            query_interval: ValidEpochInterval::new(start, start.advanced_seconds(2.0 * 86_400.0)),
            sample_step_s: 120.0,
            boundary_time_tolerance_s: 0.5,
            min_range_m: None,
            max_range_m: None,
            signal_time,
        };
        let end = |station| AnalysisEndpoint::SurfaceStation {
            station,
            altitude_spec: None,
        };
        let (source, target) = if downlink {
            (end(&self.rover), end(&self.goldstone))
        } else {
            (end(&self.goldstone), end(&self.rover))
        };
        let report = evaluate_endpoint_link_access(
            source,
            target,
            &self.bodies,
            None,
            &query,
            &self.solar.ephemeris,
            &self.solar.orientation,
        )
        .expect("link access");
        let light_time = report
            .samples
            .first()
            .and_then(|sample| sample.light_time_s);
        (report.windows, light_time)
    }
}

/// Every window edge strictly inside the query interval, seconds past the first window's
/// start.
#[cfg(test)]
fn edges(windows: &[AccessWindow]) -> Vec<TdbEpoch> {
    windows
        .iter()
        .flat_map(|window| [window.start_epoch, window.end_epoch])
        .collect()
}

#[test]
fn downlink_and_uplink_windows_shift_by_the_light_time_at_the_far_planet_only() {
    let link = Link::new();
    let (geometric, no_light_time) = link.windows(true, SignalTimeModel::Geometric);
    let (downlink, light_time) = link.windows(true, SignalTimeModel::LightTime);
    let (uplink, _) = link.windows(false, SignalTimeModel::LightTime);
    assert_eq!(no_light_time, None);
    let light_time_s = light_time.expect("light time");
    assert!(
        (300.0..420.0).contains(&light_time_s),
        "Mars is {light_time_s} light-seconds away just after opposition"
    );
    assert!(
        geometric.len() >= 2,
        "{} windows in two days",
        geometric.len()
    );
    assert_eq!(geometric.len(), downlink.len());
    assert_eq!(geometric.len(), uplink.len());

    // An edge set by Mars's horizon is seen a light time later at Earth and not at all
    // later at Mars; an edge set by Earth's horizon the other way round.
    let (mut set_at_mars, mut set_at_earth) = (0, 0);
    for ((at, down), up) in edges(&geometric)
        .into_iter()
        .zip(edges(&downlink))
        .zip(edges(&uplink))
    {
        let down_shift = down.signed_offset_seconds_from(at);
        let up_shift = up.signed_offset_seconds_from(at);
        let near = |shift: f64, expected: f64| (shift - expected).abs() < 5.0;
        if near(down_shift, light_time_s) && near(up_shift, 0.0) {
            set_at_mars += 1;
        } else if near(down_shift, 0.0) && near(up_shift, light_time_s) {
            set_at_earth += 1;
        } else {
            panic!("an edge moved {down_shift} s on the downlink and {up_shift} s on the uplink");
        }
    }
    assert!(
        set_at_mars > 0 && set_at_earth > 0,
        "{set_at_mars} and {set_at_earth}"
    );
}

#[test]
fn a_signal_from_behind_mars_is_blocked_by_mars_where_it_was_when_the_signal_passed() {
    use sim_analysis::check_signal_path_occultation;
    use sim_spacetime::{DerivativeOrder, EphemerisCatalog, SPEED_OF_LIGHT_M_S};

    let link = Link::new();
    let receive = TdbEpoch::new(820_497_600 + 424 * 86_400, 0);
    let ssb = |body, at| {
        link.solar
            .ephemeris
            .geometric_state_icrf(body, NaifId::SSB, at, DerivativeOrder::Position)
            .expect("body")
            .pos_m
    };
    let earth = ssb(NaifId::EARTH, receive);
    let light_time_s = (ssb(NaifId::MARS, receive) - earth).length() / SPEED_OF_LIGHT_M_S;
    let emit = receive.advanced_seconds(-light_time_s);
    let mars_then = ssb(NaifId::MARS, emit);
    let away_from_earth = (mars_then - earth).normalize();
    let behind_mars = mars_then + away_from_earth * 4_000_000.0;
    let blocked_by = |emitter_host| {
        check_signal_path_occultation(
            emit,
            receive,
            emitter_host,
            Some(NaifId::EARTH),
            behind_mars,
            earth + (mars_then - earth).normalize() * 6_400_000.0,
            &link.bodies,
            &link.solar.ephemeris,
            &link.solar.orientation,
        )
        .expect("occultation")
        .map(|(body, _)| body)
    };
    assert_eq!(blocked_by(None), Some(NaifId::MARS));

    // Mars moved about a planet's width while the signal crossed. Judged where Mars is on
    // arrival instead, the same path would look clear.
    let mars_now = ssb(NaifId::MARS, receive);
    assert!((mars_now - mars_then).length() > 3_400_000.0);
    let miss = (behind_mars - mars_now).cross(away_from_earth).length();
    assert!(miss > 3_400_000.0, "the late Mars is {miss} m off the path");
}

#[cfg(test)]
fn eval_mars_dem_link(
    link: &Link,
    dem: &sim_analysis::DemElevationGrid,
    downlink: bool,
) -> sim_analysis::EndpointLinkReport {
    use sim_analysis::{AltitudeSpec, TerrainElevationSource, TerrainLosConfig, VerticalDatumSpec};

    let start = TdbEpoch::new(820_497_600 + 424 * 86_400, 0);
    let query = AccessQuerySpec {
        query_interval: ValidEpochInterval::new(start, start.advanced_seconds(86_400.0)),
        sample_step_s: 120.0,
        boundary_time_tolerance_s: 0.5,
        min_range_m: None,
        max_range_m: None,
        signal_time: SignalTimeModel::LightTime,
    };
    let rover_agl = AltitudeSpec {
        datum: VerticalDatumSpec::DemSurfaceAgl {
            dataset_id: dem.dataset_id().to_owned(),
        },
        value_m: 10.0,
    };
    let terrain_cfg = TerrainLosConfig {
        sample_step_m: 500.0,
        min_clearance_m: 2.0,
        max_terrain_interaction_altitude_m: 8_000.0,
        max_ray_samples: 50_000,
    };
    let rover_ep = AnalysisEndpoint::SurfaceStation {
        station: &link.rover,
        altitude_spec: Some(&rover_agl),
    };
    let earth_ep = AnalysisEndpoint::SurfaceStation {
        station: &link.goldstone,
        altitude_spec: None,
    };
    let (src, dst) = if downlink {
        (rover_ep, earth_ep)
    } else {
        (earth_ep, rover_ep)
    };
    evaluate_endpoint_link_access(
        src,
        dst,
        &link.bodies,
        Some((dem as &dyn TerrainElevationSource, &terrain_cfg)),
        &query,
        &link.solar.ephemeris,
        &link.solar.orientation,
    )
    .expect("mars dem link access")
}

#[test]
fn a_martian_crater_rim_occults_low_elevation_links_and_shifts_downlink_edges_by_light_time() {
    use sim_analysis::{AccessRestrictionReason, DemElevationGrid};

    let link = Link::new();
    let flat_dem = DemElevationGrid::synthetic_mars_walled_basin(0.0, 0.0);
    let rim_dem = DemElevationGrid::synthetic_mars_walled_basin(0.0, 7_500.0);
    let flat_down = eval_mars_dem_link(&link, &flat_dem, true);
    let rim_down = eval_mars_dem_link(&link, &rim_dem, true);
    let rim_up = eval_mars_dem_link(&link, &rim_dem, false);
    assert!(!flat_down.windows.is_empty() && !rim_down.windows.is_empty());
    assert_eq!(rim_down.windows.len(), rim_up.windows.len());
    assert!(rim_down.samples.iter().any(|s| {
        s.active_restrictions.iter().any(|r| {
            matches!(
                r,
                AccessRestrictionReason::OccultedByTerrain {
                    body: NaifId::MARS,
                    ..
                }
            )
        })
    }));
    let flat_total_s: f64 = flat_down.windows.iter().map(|w| w.duration_s).sum();
    let rim_total_s: f64 = rim_down.windows.iter().map(|w| w.duration_s).sum();
    assert!(rim_total_s + 600.0 < flat_total_s);
    let light_time_s = rim_down
        .samples
        .first()
        .and_then(|s| s.light_time_s)
        .expect("lt");
    assert!(
        edges(&rim_down.windows)
            .into_iter()
            .zip(edges(&rim_up.windows))
            .any(
                |(down_edge, up_edge)| (down_edge.signed_offset_seconds_from(up_edge)
                    - light_time_s)
                    .abs()
                    < 5.0
            )
    );
}
