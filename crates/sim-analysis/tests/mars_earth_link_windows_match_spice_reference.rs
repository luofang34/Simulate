//! Access windows between a Mars rover and a deep-space station on Earth, computed with
//! light time by `sim-analysis`, against a reference built from the kernel files through
//! `spicekit` alone: SPK body states, the binary Earth PCK (ITRF93), and the Mars IAU
//! rotation evaluated from the constants in the text PCK.

use glam::{DMat3, DVec3};

#[path = "mars_earth_link_windows_match_spice_reference/iau.rs"]
mod iau;
use iau::{MarsIau, data_blocks, pck_ellipsoid};
use sim_analysis::{
    AccessQuerySpec, AccessWindow, AnalysisEndpoint, GroundStationModel,
    OccultationEnvironmentModel, SensorFovModel, SignalTimeModel, evaluate_endpoint_link_access,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{NaifId, SensorMount, TdbEpoch, ValidEpochInterval};
use spicekit::{KernelPool, NaifFrame};

const C_M_S: f64 = 299_792_458.0;
/// Class id the binary PCK stores its ITRF93 segments under.
const ITRF93_PCK_CLASS: i32 = 3_000;
const EDGE_TOLERANCE_S: f64 = 2.0;
const SHIFT_VS_REFERENCE_TOLERANCE_S: f64 = 1.0;
/// The two crossings of one horizon are about two light times apart, over which the far
/// planet's direction changes, so the shift is the light time only to first order.
const SHIFT_VS_LIGHT_TIME_TOLERANCE_S: f64 = 5.0;
const SCAN_STEP_S: f64 = 30.0;
const BISECT_RESOLUTION_S: f64 = 0.05;
const MOON_RADIUS_M: f64 = 1_737_400.0;
const SUN_RADIUS_M: f64 = 695_700_000.0;
const QUERY_DURATION_S: f64 = 2.0 * 86_400.0;

#[cfg(test)]
fn min_elevation_rad() -> f64 {
    10.0_f64.to_radians()
}

#[cfg(test)]
fn query_start() -> TdbEpoch {
    TdbEpoch::new(820_497_600 + 424 * 86_400, 0)
}

#[cfg(test)]
fn data_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../sim-orbital/data")
}

// ---- reference geometry ----------------------------------------------------------------

#[cfg(test)]
struct Site {
    host: i32,
    fixed: DVec3,
    up: DVec3,
}

#[cfg(test)]
impl Site {
    /// Station at zero height: prime-vertical radius of curvature formula.
    fn new(host: i32, ellipsoid: (f64, f64), lat_deg: f64, lon_deg: f64) -> Self {
        let (a, e2) = ellipsoid;
        let (lat, lon) = (lat_deg.to_radians(), lon_deg.to_radians());
        let (sin_lat, cos_lat) = lat.sin_cos();
        let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
        let up = DVec3::new(cos_lat * lon.cos(), cos_lat * lon.sin(), sin_lat);
        let fixed = DVec3::new(n * up.x, n * up.y, n * (1.0 - e2) * sin_lat);
        Self { host, fixed, up }
    }
}

#[cfg(test)]
struct Reference {
    pool: KernelPool,
    mars: MarsIau,
    rover: Site,
    dsn: Site,
}

#[cfg(test)]
struct Eval {
    light_time_s: f64,
    el_src_rad: f64,
    el_dst_rad: f64,
    blocked: bool,
}

#[cfg(test)]
impl Eval {
    fn accessible(&self) -> bool {
        self.el_src_rad >= min_elevation_rad()
            && self.el_dst_rad >= min_elevation_rad()
            && !self.blocked
    }
}

#[cfg(test)]
impl Reference {
    fn load() -> Self {
        let dir = data_dir();
        let mut pool = KernelPool::new();
        pool.furnish(dir.join("solar_de440s_mar099s_2026_2028.bsp"))
            .expect("spk");
        pool.furnish(dir.join("earth_itrf93_2026_2028.bpc"))
            .expect("bpc");
        let text = std::fs::read_to_string(dir.join("pck00011.tpc")).expect("tpc");
        let data = data_blocks(&text);
        let mars_shape = pck_ellipsoid(&data, "BODY499_RADII");
        let wgs84_f = 1.0 / 298.257_223_563;
        let wgs84 = (6_378_137.0, wgs84_f * (2.0 - wgs84_f));
        Self {
            pool,
            mars: MarsIau::from_data(&data),
            rover: Site::new(NaifId::MARS.0, mars_shape, 18.44, 77.45),
            dsn: Site::new(NaifId::EARTH.0, wgs84, 35.426, -116.89),
        }
    }

    fn body_ssb(&self, body: i32, et: f64) -> DVec3 {
        let state = self.pool.spk_state(body, NaifId::SSB.0, et).expect("spk");
        DVec3::new(state[0], state[1], state[2]) * 1.0e3
    }

    fn j2000_to_fixed(&self, host: i32, et: f64) -> DMat3 {
        if host == NaifId::EARTH.0 {
            let rows = self
                .pool
                .pck_pxform(ITRF93_PCK_CLASS, NaifFrame::J2000, et)
                .expect("itrf93");
            DMat3::from_cols_array_2d(&rows).transpose()
        } else {
            self.mars.j2000_to_fixed(et)
        }
    }

    fn site_ssb(&self, site: &Site, et: f64) -> DVec3 {
        let to_icrf = self.j2000_to_fixed(site.host, et).transpose();
        self.body_ssb(site.host, et) + to_icrf * site.fixed
    }

    /// Elevation of direction `los` (J2000) above the local geodetic horizon of `site` at `et`.
    fn elevation_rad(&self, site: &Site, et: f64, los: DVec3) -> f64 {
        let local = self.j2000_to_fixed(site.host, et) * los;
        (local.dot(site.up) / local.length()).asin()
    }

    /// Whether a sphere of `radius_m` at `centre` cuts the segment `from`..`to`.
    fn sphere_cuts(centre: DVec3, radius_m: f64, from: DVec3, to: DVec3) -> bool {
        let chord = to - from;
        let along = ((centre - from).dot(chord) / chord.length_squared()).clamp(0.0, 1.0);
        (from + chord * along - centre).length() < radius_m
    }

    /// The link for a signal arriving at `dst` at `rx_et`, with the emitter one light time
    /// earlier, solved by fixed-point iteration in barycentric coordinates.
    fn eval(&self, src: &Site, dst: &Site, rx_et: f64) -> Eval {
        let p_dst = self.site_ssb(dst, rx_et);
        let (mut emit_et, mut p_src) = (rx_et, self.site_ssb(src, rx_et));
        for _ in 0..8 {
            emit_et = rx_et - (p_dst - p_src).length() / C_M_S;
            p_src = self.site_ssb(src, emit_et);
        }
        let rho = p_dst - p_src;
        let mid_et = 0.5 * (emit_et + rx_et);
        let blocked = [
            (NaifId::MOON.0, MOON_RADIUS_M),
            (NaifId::SUN.0, SUN_RADIUS_M),
        ]
        .into_iter()
        .any(|(body, radius)| Self::sphere_cuts(self.body_ssb(body, mid_et), radius, p_src, p_dst));
        Eval {
            light_time_s: rx_et - emit_et,
            el_src_rad: self.elevation_rad(src, emit_et, rho),
            el_dst_rad: self.elevation_rad(dst, rx_et, -rho),
            blocked,
        }
    }
}

// ---- window edges ----------------------------------------------------------------------

/// What made the link accessible or inaccessible at an edge.
#[cfg(test)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Limiter {
    SourceHorizon,
    TargetHorizon,
    Other,
    QueryBound,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug)]
struct Edge {
    et: f64,
    limiter: Limiter,
}

#[cfg(test)]
fn limiter_between(before: &Eval, after: &Eval) -> Limiter {
    let min = min_elevation_rad();
    let flips = |a: f64, b: f64| (a >= min) != (b >= min);
    if flips(before.el_src_rad, after.el_src_rad) {
        Limiter::SourceHorizon
    } else if flips(before.el_dst_rad, after.el_dst_rad) {
        Limiter::TargetHorizon
    } else {
        Limiter::Other
    }
}

#[cfg(test)]
impl Reference {
    fn bisect_edge(&self, src: &Site, dst: &Site, mut lo: f64, mut hi: f64) -> Edge {
        let state_lo = self.eval(src, dst, lo);
        let state_hi = self.eval(src, dst, hi);
        let was = state_lo.accessible();
        while hi - lo > BISECT_RESOLUTION_S {
            let mid = 0.5 * (lo + hi);
            if self.eval(src, dst, mid).accessible() == was {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        Edge {
            et: 0.5 * (lo + hi),
            limiter: limiter_between(&state_lo, &state_hi),
        }
    }

    /// Rise and set edges of the link over `[t0, t1]`, in order, with query bounds
    /// standing in for a window that is open at either end.
    fn edges(&self, src: &Site, dst: &Site, t0: f64, t1: f64) -> Vec<Edge> {
        let bound = |et| Edge {
            et,
            limiter: Limiter::QueryBound,
        };
        let steps = ((t1 - t0) / SCAN_STEP_S).ceil() as usize;
        let mut prev_et = t0;
        let mut prev = self.eval(src, dst, t0).accessible();
        let mut edges = if prev { vec![bound(t0)] } else { Vec::new() };
        for k in 1..=steps {
            let et = (t0 + k as f64 * SCAN_STEP_S).min(t1);
            let now = self.eval(src, dst, et).accessible();
            if now != prev {
                edges.push(self.bisect_edge(src, dst, prev_et, et));
            }
            (prev_et, prev) = (et, now);
        }
        if prev {
            edges.push(bound(t1));
        }
        edges
    }
}

// ---- library side ----------------------------------------------------------------------

#[cfg(test)]
fn station(
    id: u32,
    body: NaifId,
    ellipsoid: BodyEllipsoid,
    lat_deg: f64,
    lon_deg: f64,
) -> GroundStationModel {
    GroundStationModel {
        station_id: id,
        name: format!("station {id}"),
        body,
        geodetic: Geodetic::from_degrees(lat_deg, lon_deg, 0.0),
        ellipsoid,
        mount: SensorMount::IDENTITY,
        fov: SensorFovModel::OmnidirectionalHemisphere,
        min_elevation_rad: min_elevation_rad(),
    }
}

/// Library window edges (J2000 seconds) for the rover-to-station link or its reverse.
#[cfg(test)]
fn library_edges(downlink: bool) -> Vec<f64> {
    use sim_analysis::{OccultingBodySpec, OccultingShape};
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let rover = station(1, NaifId::MARS, BodyEllipsoid::MARS_IAU, 18.44, 77.45);
    let dsn = station(14, NaifId::EARTH, BodyEllipsoid::WGS84, 35.426, -116.89);
    let body = |body, shape| OccultingBodySpec { body, shape };
    let sphere = |radius_m| OccultingShape::Sphere { radius_m };
    let bodies = OccultationEnvironmentModel {
        bodies: vec![
            body(
                NaifId::EARTH,
                OccultingShape::Ellipsoid(BodyEllipsoid::WGS84),
            ),
            body(
                NaifId::MARS,
                OccultingShape::Ellipsoid(BodyEllipsoid::MARS_IAU),
            ),
            body(NaifId::MOON, sphere(MOON_RADIUS_M)),
            body(NaifId::SUN, sphere(SUN_RADIUS_M)),
        ],
    };
    let start = query_start();
    let query = AccessQuerySpec {
        query_interval: ValidEpochInterval::new(start, start.advanced_seconds(QUERY_DURATION_S)),
        sample_step_s: 120.0,
        boundary_time_tolerance_s: 0.5,
        min_range_m: None,
        max_range_m: None,
        signal_time: SignalTimeModel::LightTime,
    };
    let end = |station| AnalysisEndpoint::SurfaceStation {
        station,
        altitude_spec: None,
    };
    let (source, target) = if downlink {
        (end(&rover), end(&dsn))
    } else {
        (end(&dsn), end(&rover))
    };
    let report = evaluate_endpoint_link_access(
        source,
        target,
        &bodies,
        None,
        &query,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("link access");
    window_edges(&report.windows)
}

#[cfg(test)]
fn window_edges(windows: &[AccessWindow]) -> Vec<f64> {
    windows
        .iter()
        .flat_map(|w| [w.start_epoch, w.end_epoch])
        .map(TdbEpoch::as_j2000_seconds_f64)
        .collect()
}

// ---- tests -----------------------------------------------------------------------------

#[cfg(test)]
fn reference_edges(reference: &Reference, downlink: bool) -> Vec<Edge> {
    let t0 = query_start().as_j2000_seconds_f64();
    let (src, dst) = if downlink {
        (&reference.rover, &reference.dsn)
    } else {
        (&reference.dsn, &reference.rover)
    };
    reference.edges(src, dst, t0, t0 + QUERY_DURATION_S)
}

#[test]
fn rover_and_station_window_edges_match_the_kernel_reference_to_two_seconds() {
    let reference = Reference::load();
    for (name, downlink) in [("downlink", true), ("uplink", false)] {
        let ours = library_edges(downlink);
        let theirs = reference_edges(&reference, downlink);
        assert_eq!(ours.len(), theirs.len(), "{name} edge count");
        assert!(ours.len() >= 4, "{name} has {} edges", ours.len());
        for (k, (lib, spice)) in ours.iter().zip(&theirs).enumerate() {
            let diff = lib - spice.et;
            assert!(
                diff.abs() <= EDGE_TOLERANCE_S,
                "{name} edge {k} differs by {diff} s ({:?})",
                spice.limiter
            );
        }
    }
}

#[test]
fn downlink_and_uplink_edges_of_one_horizon_differ_by_the_light_time() {
    let reference = Reference::load();
    let down_ref = reference_edges(&reference, true);
    let up_ref = reference_edges(&reference, false);
    let (down, up) = (library_edges(true), library_edges(false));
    assert!(down.len() == up.len() && down.len() == down_ref.len());
    let (mut mars_limited, mut earth_limited) = (0, 0);
    let mut misses = Vec::new();
    for (k, (edge, up_edge)) in down_ref.iter().zip(&up_ref).enumerate() {
        let sign = match (edge.limiter, up_edge.limiter) {
            (Limiter::SourceHorizon, Limiter::TargetHorizon) => {
                mars_limited += 1;
                1.0
            }
            (Limiter::TargetHorizon, Limiter::SourceHorizon) => {
                earth_limited += 1;
                -1.0
            }
            (Limiter::QueryBound, Limiter::QueryBound) => continue,
            other => panic!("edge {k}: the horizon that set it is {other:?} on the two links"),
        };
        let light_time_s = reference
            .eval(&reference.rover, &reference.dsn, edge.et)
            .light_time_s;
        let shift =
            down.get(k).copied().unwrap_or(f64::NAN) - up.get(k).copied().unwrap_or(f64::NAN);
        let ref_shift = edge.et - up_ref.get(k).map_or(f64::NAN, |e| e.et);
        let against_reference = (shift - ref_shift).abs();
        let against_light_time = (shift.abs() - light_time_s).abs();
        if sign * shift < 0.0
            || against_reference > SHIFT_VS_REFERENCE_TOLERANCE_S
            || against_light_time > SHIFT_VS_LIGHT_TIME_TOLERANCE_S
        {
            misses.push(format!(
                "edge {k}: down - up = {shift:.3} s (reference {ref_shift:.3} s), light time {light_time_s:.3} s"
            ));
        }
    }
    assert!(misses.is_empty(), "{misses:?}");
    assert!(
        mars_limited > 0 && earth_limited > 0,
        "{mars_limited} edges set at Mars and {earth_limited} at Earth"
    );
}

#[test]
fn mars_iau_rotation_sends_the_pole_to_the_fixed_z_axis() {
    let reference = Reference::load();
    let et = query_start().as_j2000_seconds_f64();
    let (ra, dec, _) = reference.mars.pole_and_meridian(et);
    let pole = DVec3::new(dec.cos() * ra.cos(), dec.cos() * ra.sin(), dec.sin());
    let fixed = reference.mars.j2000_to_fixed(et) * pole;
    assert!((fixed - DVec3::Z).length() < 1.0e-12, "{fixed}");
}
