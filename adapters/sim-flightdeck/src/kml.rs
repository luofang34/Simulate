//! FlightAware / Google Earth KML (`<gx:Track>`, `<LineString>`) and GPX (`<trkpt>`) flight
//! track replay with smoothed ground speed, track, vertical rate, and coordinated turn attitude
//! on an explicitly declared [`PlanetaryReference`].

mod kinematics;
mod parser;

use crate::error::FlightDeckError;
use crate::planet::PlanetaryReference;
use crate::telemetry::{
    ActiveRoute, KNOTS_TO_MPS, OwnshipIdentity, OwnshipState, RouteWaypoint, TelemetryGroup,
};
use kinematics::{build_smoothed_points, calibrate_baro_qnh_offset, normalize_sample_offsets};
use parser::parse_track_xml;
use world_core::DecimalYear;

const MPS_TO_KNOTS: f64 = 1.0 / KNOTS_TO_MPS;
const MPS_TO_FPM: f64 = 60.0 / 0.3048;

/// Single kinematically smoothed flight track sample relative to track start (`0.0 s`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KmlTrackPoint {
    /// Elapsed seconds from the first track fix (`>= 0.0`).
    pub offset_s: f64,
    /// Planetographic latitude in degrees on the declared body.
    pub latitude_deg: f64,
    /// Planetographic longitude in degrees on the declared body.
    pub longitude_deg: f64,
    /// Geometric altitude above the declared vertical datum in metres.
    pub altitude_msl_m: f64,
    /// Smoothed horizontal ground speed in knots.
    pub ground_speed_kt: f64,
    /// True ground track in degrees `[0, 360)`.
    pub ground_track_true_deg: f64,
    /// Smoothed vertical speed in feet per minute.
    pub vertical_speed_fpm: f64,
    /// Derived pitch attitude in degrees (flight-path angle plus nominal cruise AoA).
    pub pitch_deg: f64,
    /// Derived coordinated-turn bank angle in degrees (`+` right wing down).
    pub roll_deg: f64,
    east_mps: f64,
    north_mps: f64,
    up_mps: f64,
}

/// Parsed and smoothed KML / GPX flight recording ready for deterministic 60 Hz replay.
#[derive(Clone, Debug, PartialEq)]
pub struct KmlFlightTrack {
    /// Aircraft tail number / callsign (e.g. `"N9758H"`).
    pub callsign: String,
    /// Origin-to-destination route summary (e.g. `"N51 → N51"`).
    pub route_label: String,
    /// Source KML `<Document><name>` title.
    pub document_title: String,
    /// Total replay duration in seconds.
    pub duration_s: f64,
    /// Monotonic smoothed track vertices (`len >= 2`).
    pub points: Vec<KmlTrackPoint>,
    /// Named airport or route waypoints parsed from KML `<Point>` placemarks.
    pub waypoints: Vec<RouteWaypoint>,
}

/// Snapshot of active KML replay state exposed to Swift UI controls.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct KmlReplayStatus {
    /// Whether a KML/GPX track is currently loaded.
    pub has_track: bool,
    /// Whether playback progression is paused.
    pub paused: bool,
    /// Playback speed multiplier (`1.0` = real-time, `5.0` = `5x`).
    pub speed_multiplier: f64,
    /// Current playback offset in seconds (`0.0..=duration_s`).
    pub elapsed_s: f64,
    /// Total track duration in seconds.
    pub duration_s: f64,
    /// Total number of track points.
    pub point_count: usize,
    /// Loaded aircraft callsign.
    pub callsign: String,
    /// Loaded route summary string.
    pub route_label: String,
}

impl KmlFlightTrack {
    /// Parses UTF-8 KML or GPX bytes into a smoothed [`KmlFlightTrack`] on the explicitly
    /// declared [`PlanetaryReference`].
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::KmlTrack`] if UTF-8 decoding fails or fewer than two valid fixes exist.
    pub fn parse_bytes_on(
        bytes: &[u8],
        planet: &PlanetaryReference,
    ) -> Result<Self, FlightDeckError> {
        let xml = core::str::from_utf8(bytes).map_err(|e| FlightDeckError::KmlTrack {
            reason: format!("invalid UTF-8 in flight track: {e}"),
        })?;
        Self::parse_str_on(xml, planet)
    }

    /// Parses KML or GPX XML text into a smoothed [`KmlFlightTrack`] on the explicitly
    /// declared [`PlanetaryReference`].
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::KmlTrack`] if fewer than two valid fixes exist.
    pub fn parse_str_on(xml: &str, planet: &PlanetaryReference) -> Result<Self, FlightDeckError> {
        let doc = parse_track_xml(xml)?;
        let timed = normalize_sample_offsets(&doc.samples, planet);
        if timed.len() < 2 {
            return Err(FlightDeckError::KmlTrack {
                reason: "fewer than 2 distinct track fixes after timestamp normalization"
                    .to_owned(),
            });
        }
        let mut points = build_smoothed_points(&timed, planet);
        calibrate_baro_qnh_offset(&mut points, &doc.waypoints, &doc.route_label, planet);
        let duration_s = points.last().map_or(0.0, |p| p.offset_s);
        Ok(Self {
            callsign: doc.callsign,
            route_label: doc.route_label,
            document_title: doc.document_title,
            duration_s,
            points,
            waypoints: doc.waypoints,
        })
    }

    /// Interpolates aircraft position, ground vector, vertical rate, and coordinated attitude at
    /// `offset_s` (`0.0..=duration_s`) into `ownship` on `planet` and marks telemetry arrival.
    pub fn sample_into_ownship(
        &self,
        offset_s: f64,
        now_ms: u64,
        epoch: DecimalYear,
        planet: &PlanetaryReference,
        ownship: &mut OwnshipState,
    ) {
        let clamped = offset_s.clamp(0.0, self.duration_s.max(0.0));
        let Some(interp) = self.interpolate_point(clamped) else {
            return;
        };

        ownship.latitude_deg = Some(interp.latitude_deg);
        ownship.longitude_deg = Some(interp.longitude_deg);
        ownship.gnss_altitude_m = Some(interp.altitude_msl_m);
        ownship.satellites = Some(12);
        ownship.gnss_valid = true;
        ownship.pressure_altitude_ft = None;
        ownship.ground_track_true_deg = Some(interp.ground_track_true_deg);
        ownship.ground_speed_kt = Some(interp.ground_speed_kt);
        ownship.ground_vector_valid = true;
        ownship.vertical_speed_fpm = Some(interp.vertical_speed_fpm);
        ownship.indicated_airspeed_kt = None;
        ownship.true_heading_deg = Some(interp.ground_track_true_deg);
        ownship.pitch_deg = Some(interp.pitch_deg);
        ownship.roll_deg = Some(interp.roll_deg);
        ownship.slip_skid_g = Some(0.0);
        ownship.identity = Some(OwnshipIdentity {
            mode_s_code: 0,
            callsign: Some(self.callsign.clone()),
        });
        ownship.route = self.build_active_route();
        ownship.refresh_geomag_on(epoch, planet);
        if let Some(var_east) = ownship.magnetic_variation_deg_east {
            ownship.magnetic_heading_deg =
                Some((interp.ground_track_true_deg - var_east).rem_euclid(360.0));
        }

        for group in [
            TelemetryGroup::Position,
            TelemetryGroup::GroundVector,
            TelemetryGroup::Attitude,
            TelemetryGroup::VerticalSpeed,
        ] {
            ownship.clock.mark(group, now_ms);
        }
    }

    /// Serializes track metadata and `[lon, lat, alt_m]` polyline coordinates as compact JSON.
    #[must_use]
    pub fn to_summary_json(&self) -> String {
        let mut out =
            String::with_capacity(self.points.len().saturating_mul(28).saturating_add(160));
        out.push_str("{\"callsign\":\"");
        push_json_escaped(&mut out, &self.callsign);
        out.push_str("\",\"route\":\"");
        push_json_escaped(&mut out, &self.route_label);
        out.push_str("\",\"title\":\"");
        push_json_escaped(&mut out, &self.document_title);
        out.push_str(&format!(
            "\",\"duration_s\":{:.2},\"point_count\":{},\"coordinates\":[",
            self.duration_s,
            self.points.len()
        ));
        for (idx, pt) in self.points.iter().enumerate() {
            if idx > 0 {
                out.push(',');
            }
            out.push_str(&format!(
                "[{:.6},{:.6},{:.1}]",
                pt.longitude_deg, pt.latitude_deg, pt.altitude_msl_m
            ));
        }
        out.push_str("]}");
        out
    }

    fn interpolate_point(&self, offset_s: f64) -> Option<KmlTrackPoint> {
        let first = *self.points.first()?;
        let last = *self.points.last()?;
        if offset_s <= first.offset_s {
            return Some(first);
        }
        if offset_s >= last.offset_s {
            return Some(last);
        }
        let upper_idx = self.points.partition_point(|p| p.offset_s <= offset_s);
        let a = *self.points.get(upper_idx.saturating_sub(1))?;
        let b = *self.points.get(upper_idx)?;
        let span = (b.offset_s - a.offset_s).max(1e-6);
        let u = ((offset_s - a.offset_s) / span).clamp(0.0, 1.0);

        let ve = lerp(a.east_mps, b.east_mps, u);
        let vn = lerp(a.north_mps, b.north_mps, u);
        let vu = lerp(a.up_mps, b.up_mps, u);
        let horiz_mps = libm::hypot(ve, vn);
        let track_deg = if horiz_mps > 0.5 {
            libm::atan2(ve, vn).to_degrees().rem_euclid(360.0)
        } else {
            lerp_angle_deg(a.ground_track_true_deg, b.ground_track_true_deg, u)
        };

        Some(KmlTrackPoint {
            offset_s,
            latitude_deg: lerp(a.latitude_deg, b.latitude_deg, u),
            longitude_deg: lerp_lon_deg(a.longitude_deg, b.longitude_deg, u),
            altitude_msl_m: lerp(a.altitude_msl_m, b.altitude_msl_m, u),
            ground_speed_kt: horiz_mps * MPS_TO_KNOTS,
            ground_track_true_deg: track_deg,
            vertical_speed_fpm: vu * MPS_TO_FPM,
            pitch_deg: lerp(a.pitch_deg, b.pitch_deg, u),
            roll_deg: lerp(a.roll_deg, b.roll_deg, u),
            east_mps: ve,
            north_mps: vn,
            up_mps: vu,
        })
    }

    fn build_active_route(&self) -> ActiveRoute {
        if let (Some(first_wp), Some(last_wp)) = (self.waypoints.first(), self.waypoints.last())
            && self.waypoints.len() >= 2
        {
            return ActiveRoute {
                waypoints: self.waypoints.clone(),
                active_leg: Some((first_wp.clone(), last_wp.clone())),
            };
        }
        if let (Some(first_pt), Some(last_pt)) = (self.points.first(), self.points.last()) {
            let start = RouteWaypoint {
                ident: "ORIG".to_owned(),
                latitude_deg: first_pt.latitude_deg,
                longitude_deg: first_pt.longitude_deg,
            };
            let end = RouteWaypoint {
                ident: "DEST".to_owned(),
                latitude_deg: last_pt.latitude_deg,
                longitude_deg: last_pt.longitude_deg,
            };
            return ActiveRoute {
                waypoints: vec![start.clone(), end.clone()],
                active_leg: Some((start, end)),
            };
        }
        ActiveRoute::default()
    }
}

/// Stateful playback cursor driving [`KmlFlightTrack`] inside [`crate::session::FlightDeckSession`].
#[derive(Clone, Debug)]
pub(crate) struct KmlReplayPlayer {
    track: KmlFlightTrack,
    elapsed_s: f64,
    last_wall_ms: Option<u64>,
    paused: bool,
    speed_multiplier: f64,
}

impl KmlReplayPlayer {
    pub(crate) fn new(track: KmlFlightTrack, now_ms: u64) -> Self {
        Self {
            track,
            elapsed_s: 0.0,
            last_wall_ms: Some(now_ms),
            paused: false,
            speed_multiplier: 1.0,
        }
    }

    pub(crate) fn set_playback(&mut self, paused: bool, speed_multiplier: f64, now_ms: u64) {
        self.paused = paused;
        if speed_multiplier.is_finite() {
            self.speed_multiplier = speed_multiplier.clamp(0.25, 30.0);
        }
        self.last_wall_ms = Some(now_ms);
    }

    pub(crate) fn seek_fraction(&mut self, fraction: f64, now_ms: u64) {
        if fraction.is_finite() {
            self.elapsed_s = fraction.clamp(0.0, 1.0) * self.track.duration_s.max(0.0);
        }
        self.last_wall_ms = Some(now_ms);
    }

    pub(crate) fn advance_and_sample(
        &mut self,
        now_ms: u64,
        epoch: DecimalYear,
        planet: &PlanetaryReference,
        ownship: &mut OwnshipState,
    ) {
        if let Some(prev_ms) = self.last_wall_ms
            && !self.paused
            && self.track.duration_s > 0.0
        {
            let dt_wall_s = (now_ms.saturating_sub(prev_ms) as f64) * 0.001;
            let next = self.elapsed_s + dt_wall_s * self.speed_multiplier;
            self.elapsed_s = if next > self.track.duration_s {
                next.rem_euclid(self.track.duration_s)
            } else {
                next
            };
        }
        self.last_wall_ms = Some(now_ms);
        self.track
            .sample_into_ownship(self.elapsed_s, now_ms, epoch, planet, ownship);
    }

    pub(crate) fn status(&self) -> KmlReplayStatus {
        KmlReplayStatus {
            has_track: true,
            paused: self.paused,
            speed_multiplier: self.speed_multiplier,
            elapsed_s: self.elapsed_s,
            duration_s: self.track.duration_s,
            point_count: self.track.points.len(),
            callsign: self.track.callsign.clone(),
            route_label: self.track.route_label.clone(),
        }
    }

    pub(crate) fn summary_json(&self) -> String {
        self.track.to_summary_json()
    }

    pub(crate) fn route_label(&self) -> &str {
        &self.track.route_label
    }
}

fn lerp(a: f64, b: f64, u: f64) -> f64 {
    a + (b - a) * u
}

fn lerp_lon_deg(a: f64, b: f64, u: f64) -> f64 {
    let delta = (b - a + 540.0).rem_euclid(360.0) - 180.0;
    let raw = a + delta * u;
    ((raw + 180.0).rem_euclid(360.0)) - 180.0
}

fn lerp_angle_deg(a: f64, b: f64, u: f64) -> f64 {
    let delta = (b - a + 540.0).rem_euclid(360.0) - 180.0;
    (a + delta * u).rem_euclid(360.0)
}

fn push_json_escaped(out: &mut String, s: &str) {
    for ch in s.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if c.is_control() => {}
            c => out.push(c),
        }
    }
}
