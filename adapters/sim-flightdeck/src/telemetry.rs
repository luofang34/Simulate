//! Ownship telemetry state, per-group arrival clock, and explicit body heading derivation.
//!
//! Mirrors `avionics-link`'s `TelemetryClock` and `PanelInstrumentInput` discipline:
//! each sensor group records its monotonic arrival timestamp (`ms`) so stale or missing
//! channels age out cleanly on display refresh, and void GNSS fixes never feed live
//! flight instruments while remaining inspectable for link diagnostics. Coordinates and
//! heights are interpreted strictly against the caller's declared [`PlanetaryReference`].

use crate::planet::PlanetaryReference;
use sim_geo::Geodetic;
use world_core::{DecimalYear, GeodeticPosition};
use world_geomag::wmm2025;

/// Feet-to-metres exact international conversion factor.
pub const FEET_TO_METRES: f64 = 0.3048;
/// Knots-to-metres-per-second exact nautical conversion factor.
pub const KNOTS_TO_MPS: f64 = 1852.0 / 3600.0;
/// Feet-per-minute to metres-per-second conversion factor.
pub const FPM_TO_MPS: f64 = 0.3048 / 60.0;

/// Sensor group tracked independently for freshness aging.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TelemetryGroup {
    /// Horizontal GNSS position and geometric altitude.
    Position,
    /// GNSS ground track and ground speed vector.
    GroundVector,
    /// Barometric / pressure altitude.
    PressureAltitude,
    /// AHRS pitch, roll, and heading.
    Attitude,
    /// Vertical speed (when supplied by GDL-90 or simulation; absent on Connext).
    VerticalSpeed,
    /// Indicated airspeed (when supplied by air-data computer or simulation; absent on Connext).
    Airspeed,
}

/// Monotonic arrival timestamps (`ms`) per telemetry group.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TelemetryClock {
    position_ms: Option<u64>,
    ground_vector_ms: Option<u64>,
    pressure_altitude_ms: Option<u64>,
    attitude_ms: Option<u64>,
    vertical_speed_ms: Option<u64>,
    airspeed_ms: Option<u64>,
}

impl TelemetryClock {
    /// Records arrival of `group` at monotonic `now_ms`.
    pub fn mark(&mut self, group: TelemetryGroup, now_ms: u64) {
        let slot = self.slot_mut(group);
        *slot = Some(slot.map_or(now_ms, |prev| prev.max(now_ms)));
    }

    /// Age of `group` in milliseconds at monotonic `now_ms`, or `None` if never received.
    #[must_use]
    pub fn age_ms(&self, group: TelemetryGroup, now_ms: u64) -> Option<f64> {
        let arrived = (*self.slot(group))?;
        Some(now_ms.saturating_sub(arrived) as f64)
    }

    /// Stalest (maximum) age across `groups` that have arrived, matching `AvionicsLinkKit`.
    #[must_use]
    pub fn stalest_age_ms(&self, groups: &[TelemetryGroup], now_ms: u64) -> Option<f64> {
        let mut worst: Option<f64> = None;
        for &group in groups {
            if let Some(age) = self.age_ms(group, now_ms) {
                worst = Some(worst.map_or(age, |cur| cur.max(age)));
            }
        }
        worst
    }

    fn slot(&self, group: TelemetryGroup) -> &Option<u64> {
        match group {
            TelemetryGroup::Position => &self.position_ms,
            TelemetryGroup::GroundVector => &self.ground_vector_ms,
            TelemetryGroup::PressureAltitude => &self.pressure_altitude_ms,
            TelemetryGroup::Attitude => &self.attitude_ms,
            TelemetryGroup::VerticalSpeed => &self.vertical_speed_ms,
            TelemetryGroup::Airspeed => &self.airspeed_ms,
        }
    }

    fn slot_mut(&mut self, group: TelemetryGroup) -> &mut Option<u64> {
        match group {
            TelemetryGroup::Position => &mut self.position_ms,
            TelemetryGroup::GroundVector => &mut self.ground_vector_ms,
            TelemetryGroup::PressureAltitude => &mut self.pressure_altitude_ms,
            TelemetryGroup::Attitude => &mut self.attitude_ms,
            TelemetryGroup::VerticalSpeed => &mut self.vertical_speed_ms,
            TelemetryGroup::Airspeed => &mut self.airspeed_ms,
        }
    }
}

/// Ownship Mode-S 24-bit address and flight ID / tail number.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OwnshipIdentity {
    /// 24-bit ICAO Mode-S address (`0x000000..=0xFFFFFF`).
    pub mode_s_code: u32,
    /// Optional trimmed ASCII callsign (e.g. `"N19880"` or `"N9758H"`).
    pub callsign: Option<String>,
}

/// One waypoint decoded from an active flight plan (`FPN/RP:` or scenario route).
#[derive(Clone, Debug, PartialEq)]
pub struct RouteWaypoint {
    /// Waypoint identifier (e.g. `"47N"`, `"SBJ"`, `"KEWR"`).
    pub ident: String,
    /// Planetographic latitude in degrees on the declared body.
    pub latitude_deg: f64,
    /// Planetographic east-positive longitude in degrees on the declared body.
    pub longitude_deg: f64,
}

/// Published full route and active leg endpoints.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ActiveRoute {
    /// Ordered route waypoints.
    pub waypoints: Vec<RouteWaypoint>,
    /// Active leg origin and destination (`from`, `to`).
    pub active_leg: Option<(RouteWaypoint, RouteWaypoint)>,
}

/// Complete ownship navigation and attitude snapshot with explicit validity flags.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct OwnshipState {
    /// Latest GNSS latitude in degrees (retained even if `gnss_valid == false` for diagnosis).
    pub latitude_deg: Option<f64>,
    /// Latest GNSS longitude in degrees.
    pub longitude_deg: Option<f64>,
    /// Geometric / GNSS altitude above MSL in metres.
    pub gnss_altitude_m: Option<f64>,
    /// Tracked GNSS satellite count when reported.
    pub satellites: Option<u8>,
    /// Whether the GNSS position fix is valid for flight instruments.
    pub gnss_valid: bool,
    /// Pressure altitude in feet (`service 0x22`, GDL-90 `0x0A`, or baro sensor).
    pub pressure_altitude_ft: Option<f64>,
    /// True ground track in degrees `[0, 360)`.
    pub ground_track_true_deg: Option<f64>,
    /// Ground speed in knots.
    pub ground_speed_kt: Option<f64>,
    /// Whether the ground track/speed vector is valid for flight instruments.
    pub ground_vector_valid: bool,
    /// Vertical speed in feet per minute (`None` on Connext-only links so VSI honestly flags).
    pub vertical_speed_fpm: Option<f64>,
    /// Indicated airspeed in knots (`None` on Connext-only links so IAS honestly flags).
    pub indicated_airspeed_kt: Option<f64>,
    /// Confirmed magnetic heading in degrees `[0, 360)`.
    pub magnetic_heading_deg: Option<f64>,
    /// True heading in degrees `[0, 360)` when directly sensed or resolved via WMM2025.
    pub true_heading_deg: Option<f64>,
    /// Pitch angle in degrees, positive nose-up.
    pub pitch_deg: Option<f64>,
    /// Roll angle in degrees, positive right-wing-down.
    pub roll_deg: Option<f64>,
    /// Slip/skid lateral acceleration in Gs when reported.
    pub slip_skid_g: Option<f64>,
    /// Raw Connext AHRS dynamic counts (`axes[3..6]`) kept for link diagnostics.
    pub raw_ahrs_axes_456: Option<[i16; 3]>,
    /// Magnetic variation in degrees (east positive) from NMEA RMC or WMM2025.
    pub magnetic_variation_deg_east: Option<f64>,
    /// Ownship Mode-S and callsign identity.
    pub identity: Option<OwnshipIdentity>,
    /// Published panel route and active leg.
    pub route: ActiveRoute,
    /// Monotonic arrival clock per telemetry group.
    pub clock: TelemetryClock,
}

impl OwnshipState {
    /// Best flight-valid altitude above the declared vertical datum in metres (pressure altitude
    /// preferred when present, else valid geometric altitude).
    #[must_use]
    pub fn best_altitude_m(&self) -> Option<f64> {
        if let Some(pa_ft) = self.pressure_altitude_ft {
            return Some(pa_ft * FEET_TO_METRES);
        }
        if self.gnss_valid {
            return self.gnss_altitude_m;
        }
        None
    }

    /// Flight-valid `sim-geo` [`Geodetic`] on the explicitly declared [`PlanetaryReference`].
    #[must_use]
    pub fn valid_body_geodetic(&self, planet: &PlanetaryReference) -> Option<Geodetic> {
        if !self.gnss_valid {
            return None;
        }
        let lat = self.latitude_deg?;
        let lon = self.longitude_deg?;
        if !lat.is_finite()
            || !lon.is_finite()
            || !(-90.0..=90.0).contains(&lat)
            || !(-180.0..=180.0).contains(&lon)
        {
            return None;
        }
        let alt_datum_m = self
            .gnss_altitude_m
            .or_else(|| self.pressure_altitude_ft.map(|ft| ft * FEET_TO_METRES))
            .unwrap_or(0.0);
        Some(planet.geodetic_from_datum(lat, lon, alt_datum_m))
    }

    /// Flight-valid `world_core::GeodeticPosition` **only** when `planet.is_earth_wgs84()` holds.
    /// Returns `None` on non-Earth bodies (`Moon`, `Mars`) so Earth-specific crates (`WMM2025`,
    /// `FAA NASR`, `ADS-B`) never receive lunar or Martian coordinates.
    #[must_use]
    pub fn valid_earth_wgs84_position(
        &self,
        planet: &PlanetaryReference,
    ) -> Option<GeodeticPosition> {
        let geo = self.valid_body_geodetic(planet)?;
        let lat = self.latitude_deg?;
        let lon = self.longitude_deg?;
        planet.earth_wgs84_position(lat, lon, geo.height_m)
    }

    /// Resolves geomagnetic declination and true heading on `planet`:
    /// - On `WGS-84` Earth, evaluates `WMM2025` at `epoch`.
    /// - On non-Earth bodies without a global dipole model, clears geomagnetic variation so
    ///   instruments present true planetographic heading honestly.
    pub fn refresh_geomag_on(&mut self, epoch: DecimalYear, planet: &PlanetaryReference) {
        if !planet.is_earth_wgs84() {
            self.magnetic_variation_deg_east = None;
            self.magnetic_heading_deg = None;
            return;
        }
        if let Some(pos) = self.valid_earth_wgs84_position(planet)
            && let Ok(field) = wmm2025(pos, epoch)
        {
            self.magnetic_variation_deg_east = Some(field.declination_rad().to_degrees());
        }
        if let Some(mag_deg) = self.magnetic_heading_deg
            && let Some(var_east) = self.magnetic_variation_deg_east
        {
            self.true_heading_deg = Some((mag_deg + var_east).rem_euclid(360.0));
        }
    }
}
