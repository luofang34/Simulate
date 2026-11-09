use sim_geo::Geodetic;

use super::*;

const TWILIGHT: TwilightRamp = TwilightRamp {
    night_end_deg: -6.0,
    day_start_deg: 2.0,
    night_brightness: 0.22,
};

fn observer() -> (Geodetic, DVec3) {
    let position = Geodetic::from_degrees(39.1, -75.5, 0.0);
    (position, BodyEllipsoid::WGS84.geodetic_to_fixed(&position))
}

/// The Sun at `elevation_deg` above the observer's horizon, due east.
fn sun_at(elevation_deg: f64) -> (EnvironmentFrame, DVec3) {
    let (position, eye) = observer();
    let local = LocalFrame::at(&position);
    let elevation = elevation_deg.to_radians();
    let direction = local.up() * elevation.sin() + local.east() * elevation.cos();
    let frame = EnvironmentFrame::clear(
        BodyEllipsoid::WGS84,
        Sunlight::uneclipsed(direction, Some(0.267)),
        TWILIGHT,
    );
    (frame, eye)
}

#[test]
fn the_sun_elevation_is_measured_from_the_local_horizon() {
    for elevation in [-30.0, -3.0, 0.0, 10.0, 89.0] {
        let (frame, eye) = sun_at(elevation);
        assert!((frame.sun_elevation_deg_at(eye) - elevation).abs() < 1e-9);
    }
}

#[test]
fn the_sky_brightens_through_twilight_and_is_black_at_night() {
    let sky_blue = |elevation| {
        let (frame, eye) = sun_at(elevation);
        frame.sky_linear_rgb_at(eye)[2]
    };
    assert_eq!(sky_blue(-10.0), 0.0);
    assert_eq!(sky_blue(-6.001), 0.0);
    assert!(sky_blue(-4.0) > 0.0);
    assert!(sky_blue(-4.0) < sky_blue(0.0));
    assert!(sky_blue(0.0) < sky_blue(2.0));
    assert_eq!(sky_blue(2.0), Atmosphere::EARTH.day_sky_linear_rgb[2]);
    assert_eq!(sky_blue(60.0), Atmosphere::EARTH.day_sky_linear_rgb[2]);
}

#[test]
fn an_eclipse_and_the_exposure_dim_the_sky_in_proportion() {
    let (frame, eye) = sun_at(45.0);
    let full = frame.sky_linear_rgb_at(eye);
    let eclipsed = EnvironmentFrame {
        sun: Sunlight {
            visible_fraction: 0.25,
            ..frame.sun
        },
        ..frame
    };
    let exposed = EnvironmentFrame {
        exposure: 0.5,
        ..frame
    };
    for ((e, x), f) in eclipsed
        .sky_linear_rgb_at(eye)
        .iter()
        .zip(exposed.sky_linear_rgb_at(eye))
        .zip(full)
    {
        assert!((e * 4.0 - f).abs() < 1e-6);
        assert!((x * 2.0 - f).abs() < 1e-6);
    }
}

#[test]
fn only_earth_has_a_modelled_atmosphere() {
    assert_eq!(
        Atmosphere::of_body(&BodyEllipsoid::WGS84),
        Some(Atmosphere::EARTH)
    );
    assert_eq!(Atmosphere::of_body(&BodyEllipsoid::MOON_IAU), None);
    let lunar = EnvironmentFrame::clear(
        BodyEllipsoid::MOON_IAU,
        Sunlight::uneclipsed(DVec3::Z, None),
        TWILIGHT,
    );
    let north_pole = DVec3::Z * BodyEllipsoid::MOON_IAU.semi_major_m;
    assert!((lunar.sun_elevation_deg_at(north_pole) - 90.0).abs() < 1e-9);
    assert_eq!(lunar.sky_linear_rgb_at(north_pole), [0.0; 3]);
}

#[test]
fn a_clear_frame_has_no_fog_clouds_or_exposure_change() {
    let (frame, _) = sun_at(30.0);
    assert_eq!(frame.fog, Fog::CLEAR);
    assert_eq!(frame.cloud_layer, None);
    assert_eq!(frame.exposure, 1.0);
    assert!((frame.sun.direction_fixed.length() - 1.0).abs() < 1e-12);
}

#[test]
fn a_degenerate_twilight_ramp_is_a_step() {
    let step = TwilightRamp {
        night_end_deg: 0.0,
        day_start_deg: 0.0,
        night_brightness: 0.0,
    };
    assert_eq!(step.daylight(-0.1), 0.0);
    assert_eq!(step.daylight(0.0), 1.0);
}

#[test]
fn the_day_sky_darkens_with_height_and_is_black_from_orbit() {
    let (frame, _) = sun_at(60.0);
    let blue_at = |height_m| {
        let position = Geodetic::from_degrees(39.1, -75.5, height_m);
        frame.sky_linear_rgb_at(BodyEllipsoid::WGS84.geodetic_to_fixed(&position))[2]
    };
    let surface = Atmosphere::EARTH.day_sky_linear_rgb[2];
    assert!((blue_at(0.0) - surface).abs() < 1e-6);
    let one_scale_height = blue_at(Atmosphere::EARTH.scale_height_m);
    assert!((one_scale_height - surface / std::f32::consts::E).abs() < 1e-4);
    assert!(blue_at(400_000.0) < 1e-6, "low Earth orbit sees black sky");
    assert!(
        (blue_at(-100.0) - surface).abs() < 1e-6,
        "below datum is surface sky"
    );
}
