//! Deterministic XR flight scenarios and `avionics-link` Connext capture continuation on an
//! explicitly declared [`PlanetaryReference`].

mod targets;

use crate::connext::{
    ConnextIngestor, apply_connext_event, encode_ahrs_record, encode_data_record,
};
use crate::error::FlightDeckError;
use crate::planet::{PlanetaryBodyId, PlanetaryReference};
use crate::scenarios::targets::{
    APPROACH_TARGETS, CPA_TARGETS, CRUISE_TARGETS, ingest_scripted_targets,
};
use crate::telemetry::{
    ActiveRoute, FEET_TO_METRES, KNOTS_TO_MPS, OwnshipIdentity, OwnshipState, RouteWaypoint,
    TelemetryClock, TelemetryGroup,
};
use crate::traffic::TrafficCoordinator;
use glam::DVec3;
use world::surveillance::DeliveryPath;
use world_core::DecimalYear;

/// Built-in deterministic synthetic-vision scenarios over the NY/NJ corridor.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FlightDeckScenario {
    /// VFR/SVS cruise at `2,800 ft MSL` from `47N` toward `KEWR` across Watchung ridges and Class B shelves.
    #[default]
    NjCorridorCruise = 0,
    /// High-density NY/NJ traffic encounter demonstrating `TA`/`RA` CPA promotion and callsign placards.
    HighDensityTrafficCpa = 1,
    /// Stabilized `KEWR RWY 22L` `3.0°` approach inside Newark Class B with Somerset TFR off the right wing.
    KewrApproachWithTfr = 2,
}

impl FlightDeckScenario {
    /// Converts a wire discriminant (`0..=2`) into an [`FlightDeckScenario`].
    #[must_use]
    pub const fn from_code(code: u8) -> Self {
        match code {
            1 => Self::HighDensityTrafficCpa,
            2 => Self::KewrApproachWithTfr,
            _ => Self::NjCorridorCruise,
        }
    }

    /// Short title displayed in the Vision Pro control bar.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::NjCorridorCruise => "47N → KEWR Corridor Cruise",
            Self::HighDensityTrafficCpa => "NY/NJ Traffic & CPA Alert",
            Self::KewrApproachWithTfr => "KEWR RWY 22L Approach + TFR",
        }
    }
}

/// Evaluates `scenario` at `elapsed_ms` (`monotonic now_ms`) on `planet` and populates
/// `ownship` + `traffic`.
///
/// # Errors
///
/// Returns [`FlightDeckError::Surveillance`] if any scripted track fails validation.
pub fn evaluate_scenario(
    scenario: FlightDeckScenario,
    elapsed_ms: u64,
    now_ms: u64,
    epoch: DecimalYear,
    planet: &PlanetaryReference,
    ownship: &mut OwnshipState,
    traffic: &mut TrafficCoordinator,
) -> Result<(), FlightDeckError> {
    let t_s = (elapsed_ms % 180_000) as f64 * 1e-3;
    if !planet.is_earth_wgs84() {
        traffic.clear();
        populate_non_earth_ownship(t_s, now_ms, planet, ownship);
        return Ok(());
    }
    match scenario {
        FlightDeckScenario::NjCorridorCruise => {
            populate_cruise_ownship(t_s, now_ms, epoch, planet, ownship);
            ingest_scripted_targets(ownship, planet, t_s, now_ms, &CRUISE_TARGETS, traffic)?;
        }
        FlightDeckScenario::HighDensityTrafficCpa => {
            populate_cpa_ownship(t_s, now_ms, epoch, planet, ownship);
            ingest_scripted_targets(ownship, planet, t_s % 45.0, now_ms, &CPA_TARGETS, traffic)?;
        }
        FlightDeckScenario::KewrApproachWithTfr => {
            populate_kewr_approach_ownship(t_s % 90.0, now_ms, epoch, planet, ownship);
            ingest_scripted_targets(
                ownship,
                planet,
                t_s % 90.0,
                now_ms,
                &APPROACH_TARGETS,
                traffic,
            )?;
        }
    }
    Ok(())
}

/// Feeds synthetic airborne Connext inner records (`0x20`, `0x21`, `0x22`) after the static `5 s`
/// `REAL_SESSION_TSV` ground capture so replay continues smoothly along `47N -> SBJ` while keeping
/// `IAS` and `VSI` absent exactly like a real Garmin Flight Stream / Connext link.
pub fn advance_connext_replay_stream(
    elapsed_ms: u64,
    now_ms: u64,
    epoch: DecimalYear,
    planet: &PlanetaryReference,
    ingestor: &mut ConnextIngestor,
    ownship: &mut OwnshipState,
    traffic: &mut TrafficCoordinator,
) {
    if !planet.is_earth_wgs84() {
        return;
    }
    if elapsed_ms <= 5_200 {
        ownship.refresh_geomag_on(epoch, planet);
        return;
    }
    let flight_s = ((elapsed_ms - 5_200) % 120_000) as f64 * 1e-3;
    let base = planet.geodetic_from_datum(40.5267, -74.5961, 25.0);
    let track_deg = 294.0_f64;
    let gs_kt = 108.0_f64;
    let dist_m = gs_kt * KNOTS_TO_MPS * flight_s;
    let trk_rad = track_deg.to_radians();
    let pos = planet.offset_geodetic_enu(
        &base,
        DVec3::new(
            dist_m * libm::sin(trk_rad),
            dist_m * libm::cos(trk_rad),
            0.0,
        ),
    );
    let alt_ft = (350.0 + (flight_s * 11.0).min(2_150.0)) as f32;
    let alt_m = f64::from(alt_ft) * FEET_TO_METRES;
    let roll_deg = 6.0 * libm::sin(flight_s * 0.22);
    let pitch_deg = if alt_ft < 2_450.0 { 6.5 } else { 2.5 };
    let mag_hdg = (track_deg + 13.0).rem_euclid(360.0);

    let mut wire = Vec::with_capacity(256);
    wire.extend_from_slice(&encode_data_record(0x0c, 0, 0x22, 0, &alt_ft.to_le_bytes()));
    let nmea = format_gga_rmc(
        pos.lat_rad.to_degrees(),
        pos.lon_rad.to_degrees(),
        alt_m,
        gs_kt,
        track_deg,
    );
    wire.extend_from_slice(&encode_data_record(0x0c, 0, 0x20, 0, nmea.as_bytes()));
    wire.extend_from_slice(&encode_ahrs_record(mag_hdg, pitch_deg, roll_deg));

    let batch = ingestor.ingest(&wire, now_ms);
    for ev in &batch.events {
        let targets = apply_connext_event(ownship, ev, now_ms);
        for t in &targets {
            traffic
                .ingest_connext_target(t, DeliveryPath::Replay, now_ms)
                .ok();
        }
    }
    ownship.refresh_geomag_on(epoch, planet);
}

fn populate_non_earth_ownship(
    t_s: f64,
    now_ms: u64,
    planet: &PlanetaryReference,
    ownship: &mut OwnshipState,
) {
    let (base_lat, base_lon, callsign, route) = non_earth_flight_plan(planet.id);
    let origin = planet.geodetic_from_datum(base_lat, base_lon, 0.0);
    let trk_deg = 282.0 + 5.0 * libm::sin(t_s * 0.07);
    let gs_kt = 96.0;
    let dist_m = gs_kt * KNOTS_TO_MPS * (t_s % 120.0);
    let trk_rad = 282.0_f64.to_radians();
    let pos = planet.offset_geodetic_enu(
        &origin,
        DVec3::new(
            dist_m * libm::sin(trk_rad),
            dist_m * libm::cos(trk_rad),
            0.0,
        ),
    );
    let alt_m = 1_150.0 + 45.0 * libm::sin(t_s * 0.10);
    let vs_fpm = (45.0 * 0.10 * libm::cos(t_s * 0.10)) * (60.0 / FEET_TO_METRES);
    let g_local = planet.gravity_mps2(&pos);
    let omega = (5.0_f64.to_radians() * 0.07) * libm::cos(t_s * 0.07);
    let roll_deg = libm::atan2((gs_kt * KNOTS_TO_MPS) * omega, g_local).to_degrees();

    let mut clock = TelemetryClock::default();
    for grp in [
        TelemetryGroup::Position,
        TelemetryGroup::GroundVector,
        TelemetryGroup::Attitude,
        TelemetryGroup::VerticalSpeed,
    ] {
        clock.mark(grp, now_ms);
    }
    *ownship = OwnshipState {
        latitude_deg: Some(pos.lat_rad.to_degrees()),
        longitude_deg: Some(pos.lon_rad.to_degrees()),
        gnss_altitude_m: Some(alt_m),
        satellites: None,
        gnss_valid: true,
        pressure_altitude_ft: None,
        ground_track_true_deg: Some(trk_deg.rem_euclid(360.0)),
        ground_speed_kt: Some(gs_kt),
        ground_vector_valid: true,
        vertical_speed_fpm: Some(vs_fpm),
        indicated_airspeed_kt: None,
        magnetic_heading_deg: None,
        true_heading_deg: Some(trk_deg.rem_euclid(360.0)),
        pitch_deg: Some(1.8),
        roll_deg: Some(roll_deg),
        slip_skid_g: Some(0.0),
        raw_ahrs_axes_456: None,
        magnetic_variation_deg_east: None,
        identity: Some(OwnshipIdentity {
            mode_s_code: 0,
            callsign: Some(callsign.to_owned()),
        }),
        route,
        clock,
    };
}

fn non_earth_flight_plan(body: PlanetaryBodyId) -> (f64, f64, &'static str, ActiveRoute) {
    let (lat, lon, callsign, a, b) = match body {
        PlanetaryBodyId::MoonIauSphere => (
            0.674,
            23.473,
            "LM-EAGLE",
            ("TRANQ", 0.674, 23.473),
            ("SABINE", 1.380, 20.080),
        ),
        _ => (
            18.444,
            77.451,
            "INGENUITY",
            ("JEZERO", 18.444, 77.451),
            ("NERETVA", 18.520, 77.320),
        ),
    };
    let w0 = RouteWaypoint {
        ident: a.0.to_owned(),
        latitude_deg: a.1,
        longitude_deg: a.2,
    };
    let w1 = RouteWaypoint {
        ident: b.0.to_owned(),
        latitude_deg: b.1,
        longitude_deg: b.2,
    };
    let route = ActiveRoute {
        waypoints: vec![w0.clone(), w1.clone()],
        active_leg: Some((w0, w1)),
    };
    (lat, lon, callsign, route)
}

fn populate_cruise_ownship(
    t_s: f64,
    now_ms: u64,
    epoch: DecimalYear,
    planet: &PlanetaryReference,
    ownship: &mut OwnshipState,
) {
    let origin = planet.geodetic_from_datum(40.5420, -74.5600, 0.0);
    let trk_deg = 62.0 + 4.0 * libm::sin(t_s * 0.08);
    let gs_kt = 124.0;
    let dist_m = gs_kt * KNOTS_TO_MPS * t_s;
    let rad = 62.0_f64.to_radians();
    let pos = planet.offset_geodetic_enu(
        &origin,
        DVec3::new(dist_m * libm::sin(rad), dist_m * libm::cos(rad), 0.0),
    );
    let pa_ft = 2_800.0 + 60.0 * libm::sin(t_s * 0.12);
    let vs_fpm = 60.0 * 0.12 * libm::cos(t_s * 0.12) * 60.0;
    let roll_deg = 8.5 * libm::cos(t_s * 0.08);
    let pitch_deg = 3.0 + 0.8 * libm::cos(t_s * 0.12);

    write_full_sim_ownship(
        ownship,
        pos.lat_rad.to_degrees(),
        pos.lon_rad.to_degrees(),
        pa_ft,
        trk_deg,
        gs_kt,
        120.0,
        vs_fpm,
        pitch_deg,
        roll_deg,
        "N9758H",
        ("47N", 40.5243, -74.5984),
        ("KEWR", 40.6895, -74.1745),
        now_ms,
        epoch,
        planet,
    );
}

fn populate_cpa_ownship(
    t_s: f64,
    now_ms: u64,
    epoch: DecimalYear,
    planet: &PlanetaryReference,
    ownship: &mut OwnshipState,
) {
    let origin = planet.geodetic_from_datum(40.5950, -74.4400, 0.0);
    let gs_kt = 132.0;
    let dist_m = gs_kt * KNOTS_TO_MPS * (t_s % 45.0);
    let rad = 55.0_f64.to_radians();
    let pos = planet.offset_geodetic_enu(
        &origin,
        DVec3::new(dist_m * libm::sin(rad), dist_m * libm::cos(rad), 0.0),
    );
    write_full_sim_ownship(
        ownship,
        pos.lat_rad.to_degrees(),
        pos.lon_rad.to_degrees(),
        3_200.0,
        55.0,
        gs_kt,
        128.0,
        0.0,
        2.6,
        2.0 * libm::sin(t_s * 0.3),
        "N9758H",
        ("SBJ", 40.5833, -74.7417),
        ("KEWR", 40.6895, -74.1745),
        now_ms,
        epoch,
        planet,
    );
}

fn populate_kewr_approach_ownship(
    t_s: f64,
    now_ms: u64,
    epoch: DecimalYear,
    planet: &PlanetaryReference,
    ownship: &mut OwnshipState,
) {
    let thr_22l = planet.geodetic_from_datum(40.7010, -74.1615, 0.0);
    let course_deg = 209.0_f64;
    let gs_kt = 134.0_f64;
    let rem_dist_m = (9_800.0 - gs_kt * KNOTS_TO_MPS * t_s).max(550.0);
    let back_rad = (course_deg + 180.0).to_radians();
    let pos = planet.offset_geodetic_enu(
        &thr_22l,
        DVec3::new(
            rem_dist_m * libm::sin(back_rad),
            rem_dist_m * libm::cos(back_rad),
            0.0,
        ),
    );
    let gs_alt_m = 18.0 + rem_dist_m * libm::tan(3.0_f64.to_radians());
    let pa_ft = gs_alt_m / FEET_TO_METRES;
    write_full_sim_ownship(
        ownship,
        pos.lat_rad.to_degrees(),
        pos.lon_rad.to_degrees(),
        pa_ft,
        course_deg,
        gs_kt,
        132.0,
        -680.0,
        1.2,
        1.5 * libm::sin(t_s * 0.25),
        "N9758H",
        ("TEB", 40.8501, -74.0608),
        ("KEWR", 40.6895, -74.1745),
        now_ms,
        epoch,
        planet,
    );
}

#[allow(clippy::too_many_arguments)]
fn write_full_sim_ownship(
    ownship: &mut OwnshipState,
    lat_deg: f64,
    lon_deg: f64,
    pa_ft: f64,
    trk_true_deg: f64,
    gs_kt: f64,
    ias_kt: f64,
    vs_fpm: f64,
    pitch_deg: f64,
    roll_deg: f64,
    callsign: &str,
    from_wp: (&str, f64, f64),
    to_wp: (&str, f64, f64),
    now_ms: u64,
    epoch: DecimalYear,
    planet: &PlanetaryReference,
) {
    let mut clock = TelemetryClock::default();
    for grp in [
        TelemetryGroup::Position,
        TelemetryGroup::GroundVector,
        TelemetryGroup::PressureAltitude,
        TelemetryGroup::Attitude,
        TelemetryGroup::VerticalSpeed,
        TelemetryGroup::Airspeed,
    ] {
        clock.mark(grp, now_ms);
    }
    let w0 = RouteWaypoint {
        ident: from_wp.0.to_owned(),
        latitude_deg: from_wp.1,
        longitude_deg: from_wp.2,
    };
    let w1 = RouteWaypoint {
        ident: to_wp.0.to_owned(),
        latitude_deg: to_wp.1,
        longitude_deg: to_wp.2,
    };
    *ownship = OwnshipState {
        latitude_deg: Some(lat_deg),
        longitude_deg: Some(lon_deg),
        gnss_altitude_m: Some(pa_ft * FEET_TO_METRES),
        satellites: Some(16),
        gnss_valid: true,
        pressure_altitude_ft: Some(pa_ft),
        ground_track_true_deg: Some(trk_true_deg.rem_euclid(360.0)),
        ground_speed_kt: Some(gs_kt),
        ground_vector_valid: true,
        vertical_speed_fpm: Some(vs_fpm),
        indicated_airspeed_kt: Some(ias_kt),
        magnetic_heading_deg: None,
        true_heading_deg: Some(trk_true_deg.rem_euclid(360.0)),
        pitch_deg: Some(pitch_deg),
        roll_deg: Some(roll_deg),
        slip_skid_g: Some(0.0),
        raw_ahrs_axes_456: None,
        magnetic_variation_deg_east: None,
        identity: Some(OwnshipIdentity {
            mode_s_code: 0xAD_9758,
            callsign: Some(callsign.to_owned()),
        }),
        route: ActiveRoute {
            waypoints: vec![w0.clone(), w1.clone()],
            active_leg: Some((w0, w1)),
        },
        clock,
    };
    ownship.refresh_geomag_on(epoch, planet);
    if let Some(var_east) = ownship.magnetic_variation_deg_east {
        ownship.magnetic_heading_deg = Some((trk_true_deg - var_east).rem_euclid(360.0));
    }
}

fn format_gga_rmc(lat_deg: f64, lon_deg: f64, alt_m: f64, gs_kt: f64, trk_deg: f64) -> String {
    let (lat_dm, lat_h) = to_nmea_dm(lat_deg, true);
    let (lon_dm, lon_h) = to_nmea_dm(lon_deg, false);
    format!(
        "$GPGGA,120000.00,{lat_dm},{lat_h},{lon_dm},{lon_h},2,12,0.8,{alt_m:.1},M,0.0,M,,*00\r\n\
         $GPRMC,120000.00,A,{lat_dm},{lat_h},{lon_dm},{lon_h},{gs_kt:.1},{trk_deg:.1},291026,13.0,W*00\r\n"
    )
}

fn to_nmea_dm(deg: f64, is_lat: bool) -> (String, char) {
    let abs = deg.abs();
    let whole = libm::floor(abs) as u32;
    let mins = (abs - f64::from(whole)) * 60.0;
    if is_lat {
        let h = if deg >= 0.0 { 'N' } else { 'S' };
        (format!("{whole:02}{mins:07.4}"), h)
    } else {
        let h = if deg >= 0.0 { 'E' } else { 'W' };
        (format!("{whole:03}{mins:07.4}"), h)
    }
}
