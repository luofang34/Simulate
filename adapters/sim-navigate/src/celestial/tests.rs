use glam::DVec3;
use sim_geo::{BodyEllipsoid, Geodetic, geodetic_to_ecef};

use super::earth::visible_fraction;
use super::stars::{ballesteros_temperature_k, ecef_to_j2000, star_rgb};
use super::{CelestialClock, ReferenceBody, SkyTimeMode};

/// 2024-04-08T00:00:00Z.
const APRIL_8_2024_UTC_S: f64 = 1_712_534_400.0;
const MAZATLAN_LAT_DEG: f64 = 23.2494;
const MAZATLAN_LON_DEG: f64 = -106.4111;

fn mazatlan() -> (DVec3, DVec3) {
    let site = Geodetic::from_degrees(MAZATLAN_LAT_DEG, MAZATLAN_LON_DEG, 0.0);
    let (sin_lat, cos_lat) = site.lat_rad.sin_cos();
    let (sin_lon, cos_lon) = site.lon_rad.sin_cos();
    let up = DVec3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);
    (geodetic_to_ecef(&site), up)
}

fn elevation_deg(direction: DVec3, up: DVec3) -> f64 {
    direction.dot(up).clamp(-1.0, 1.0).asin().to_degrees()
}

fn preset(mode: SkyTimeMode) -> super::CelestialFrame {
    let (observer, _) = mazatlan();
    CelestialClock::new().evaluate(
        mode,
        ReferenceBody::Earth,
        observer,
        0.0,
        None,
        APRIL_8_2024_UTC_S,
    )
}

fn eclipse_fraction(utc_s: f64) -> f32 {
    let (observer, _) = mazatlan();
    CelestialClock::new()
        .evaluate(
            SkyTimeMode::Auto,
            ReferenceBody::Earth,
            observer,
            utc_s - APRIL_8_2024_UTC_S,
            Some(APRIL_8_2024_UTC_S),
            0.0,
        )
        .sun_visible_fraction
}

#[test]
fn noon_preset_puts_the_sun_high() {
    let (_, up) = mazatlan();
    let frame = preset(SkyTimeMode::Noon);
    let elevation = elevation_deg(frame.sun_direction_fixed, up);
    assert!(elevation > 40.0, "noon elevation {elevation}");
    assert!((frame.sun_direction_fixed.length() - 1.0).abs() < 1e-9);
    assert!(
        frame
            .sun_angular_radius_deg
            .is_some_and(|r| (0.26..0.28).contains(&r))
    );
    assert!(frame.moon_disc.is_some() && frame.earth_shadow.is_some());
}

#[test]
fn night_preset_has_a_starfield_and_no_sun() {
    let (_, up) = mazatlan();
    let frame = preset(SkyTimeMode::Night);
    let elevation = elevation_deg(frame.sun_direction_fixed, up);
    assert!(elevation < -6.0, "night elevation {elevation}");
    let stars = frame.stars_update.unwrap();
    assert!(stars.len() >= 1000, "{} stars", stars.len());
    for star in &stars {
        assert!((star.direction_ecef.length() - 1.0).abs() < 1e-9);
        assert!(star.linear_rgb.iter().all(|c| (0.0..=1.0).contains(c)));
        assert!(star.linear_rgb.contains(&1.0));
    }
}

#[test]
fn presets_follow_local_mean_solar_time() {
    let (_, up) = mazatlan();
    let dawn = elevation_deg(preset(SkyTimeMode::Dawn).sun_direction_fixed, up);
    let dusk = elevation_deg(preset(SkyTimeMode::Dusk).sun_direction_fixed, up);
    // Near the equinox the Sun crosses the horizon close to 06:00 and 18:00
    // mean time, shifted by the equation of time (about -2 min on 8 April).
    assert!((-5.0..8.0).contains(&dawn), "dawn elevation {dawn}");
    assert!((-5.0..8.0).contains(&dusk), "dusk elevation {dusk}");
}

#[test]
fn star_uploads_are_throttled_and_reset_on_body_change() {
    let (observer, _) = mazatlan();
    let mut clock = CelestialClock::new();
    let mut at = |elapsed: f64, body: ReferenceBody| {
        let position = match body {
            ReferenceBody::Earth => observer,
            ReferenceBody::Moon => DVec3::new(BodyEllipsoid::MOON_IAU.semi_major_m, 0.0, 0.0),
        };
        clock
            .evaluate(
                SkyTimeMode::Auto,
                body,
                position,
                elapsed,
                Some(APRIL_8_2024_UTC_S),
                0.0,
            )
            .stars_update
            .is_some()
    };
    assert!(at(0.0, ReferenceBody::Earth));
    assert!(!at(1.0, ReferenceBody::Earth));
    assert!(at(2.0, ReferenceBody::Earth));
    assert!(!at(2.5, ReferenceBody::Earth));
    assert!(at(2.5, ReferenceBody::Moon));
    assert!(at(2.5, ReferenceBody::Earth));
}

#[test]
fn mazatlan_2024_total_eclipse_hides_the_sun() {
    let second_contact = eclipse_fraction(APRIL_8_2024_UTC_S + 18.0 * 3600.0 + 7.0 * 60.0 + 30.0);
    let mid_totality = eclipse_fraction(APRIL_8_2024_UTC_S + 18.0 * 3600.0 + 9.0 * 60.0 + 30.0);
    let partial = eclipse_fraction(APRIL_8_2024_UTC_S + 17.0 * 3600.0 + 30.0 * 60.0);
    let morning = eclipse_fraction(APRIL_8_2024_UTC_S + 12.0 * 3600.0);
    assert!(second_contact < 0.05, "18:07:30Z visible {second_contact}");
    assert!(mid_totality < 1e-6, "18:09:30Z visible {mid_totality}");
    assert!((0.05..0.95).contains(&partial), "17:30Z visible {partial}");
    assert!(morning > 0.999, "12:00Z visible {morning}");
}

#[test]
fn disc_overlap_limits() {
    assert_eq!(visible_fraction(1.0, 0.25, 0.25), 1.0);
    assert_eq!(visible_fraction(0.0, 0.25, 0.26), 0.0);
    assert!((visible_fraction(0.0, 0.25, 0.125) - 0.75).abs() < 1e-12);
    // Equal discs offset by one radius overlap by 39.1 %.
    assert!((visible_fraction(0.25, 0.25, 0.25) - 0.608_998).abs() < 1e-5);
}

#[test]
fn star_colours_run_blue_to_red() {
    assert!((ballesteros_temperature_k(0.65) - 5778.0).abs() < 150.0);
    let blue = star_rgb(-0.3);
    let red = star_rgb(1.6);
    assert!(blue[2] > blue[0] && red[0] > red[2]);
}

#[test]
fn modes_cycle_both_ways() {
    assert_eq!(SkyTimeMode::default(), SkyTimeMode::Auto);
    for (index, mode) in SkyTimeMode::ALL.iter().enumerate() {
        let next = SkyTimeMode::ALL[(index + 1) % SkyTimeMode::ALL.len()];
        assert_eq!(mode.next(), next);
        assert_eq!(next.prev(), *mode);
        assert!(!mode.label().is_empty());
    }
}

#[test]
fn moon_observer_gets_moon_fixed_sun_and_stars() {
    let site = Geodetic::from_degrees(0.674, 23.473, 0.0);
    let observer = BodyEllipsoid::MOON_IAU.geodetic_to_fixed(&site);
    let frame = CelestialClock::new().evaluate(
        SkyTimeMode::Auto,
        ReferenceBody::Moon,
        observer,
        0.0,
        Some(APRIL_8_2024_UTC_S),
        0.0,
    );
    assert!((frame.sun_direction_fixed.length() - 1.0).abs() < 1e-9);
    assert!(frame.moon_disc.is_none() && frame.earth_shadow.is_none());
    assert_eq!(frame.sun_visible_fraction, 1.0);
    let stars = frame.stars_update.unwrap();
    assert!(stars.len() >= 1000);
    assert!(
        stars
            .iter()
            .all(|s| (s.direction_ecef.length() - 1.0).abs() < 1e-9)
    );
}

#[test]
fn moon_stars_keep_their_angular_separations() {
    let mut clock = CelestialClock::new();
    let at = |clock: &mut CelestialClock, body, observer| {
        clock
            .evaluate(
                SkyTimeMode::Auto,
                body,
                observer,
                0.0,
                Some(APRIL_8_2024_UTC_S),
                0.0,
            )
            .stars_update
            .unwrap()
    };
    let earth = at(&mut clock, ReferenceBody::Earth, mazatlan().0);
    let moon = at(&mut clock, ReferenceBody::Moon, DVec3::X * 1.7374e6);
    let (e0, e1, m0, m1) = (earth[0], earth[1], moon[0], moon[1]);
    let earth_dot = e0.direction_ecef.dot(e1.direction_ecef);
    let moon_dot = m0.direction_ecef.dot(m1.direction_ecef);
    assert!((earth_dot - moon_dot).abs() < 1e-9);
    assert_eq!(e0.v_magnitude, m0.v_magnitude);
}

#[test]
fn recovered_rotation_returns_sirius_to_its_j2000_place() {
    let epoch = navigate_celestial::Epoch::from_unix_seconds(APRIL_8_2024_UTC_S);
    let sirius = navigate_celestial::stars_ecef(epoch, 2.0)
        .unwrap()
        .into_iter()
        .find(|star| star.hr == 2491)
        .unwrap();
    let j2000 = ecef_to_j2000(epoch).unwrap() * DVec3::from_array(sirius.direction);
    let (ra, dec) = (101.2872_f64.to_radians(), (-16.7161_f64).to_radians());
    let catalogue = DVec3::new(dec.cos() * ra.cos(), dec.cos() * ra.sin(), dec.sin());
    let error_deg = super::earth::separation_deg(j2000, catalogue);
    assert!(error_deg < 0.1, "Sirius off by {error_deg} deg");
}
