use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};

use super::{HEIGHT, TWILIGHT, WIDTH, block, gpu, nadir, render, visible};
use crate::{CameraPose, EnvironmentFrame, Fog, GlobeMesh, Sunlight, TerrainSources};

/// The camera's 8-bit sRGB reading of a linear value.
fn srgb_byte(linear: f32) -> u8 {
    let c = linear.clamp(0.0, 1.0);
    let encoded = if c <= 0.003_130_8 {
        12.92 * c
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    };
    (encoded * 255.0).round() as u8
}

fn noon(body: BodyEllipsoid, position: &Geodetic) -> EnvironmentFrame {
    EnvironmentFrame::clear(
        body,
        Sunlight::uneclipsed(LocalFrame::at(position).up(), Some(0.267)),
        TWILIGHT,
    )
}

/// Looks straight up from `height_m` above `position`.
fn zenith(body: &BodyEllipsoid, position: &Geodetic, height_m: f64) -> CameraPose {
    let frame = LocalFrame::at(position);
    let eye = body.geodetic_to_fixed(&Geodetic {
        height_m,
        ..*position
    });
    CameraPose::looking(eye, frame.up(), frame.north())
}

fn sky_pixel(body: BodyEllipsoid, environment: EnvironmentFrame) -> Option<([u8; 4], [f32; 3])> {
    let gpu = gpu()?;
    let centre = Geodetic::from_degrees(39.1, -75.5, 0.0);
    let mesh = GlobeMesh::build_region(&block(body, &centre, 0.001, 2), TerrainSources::default())
        .unwrap();
    let pose = zenith(&body, &centre, 1_000.0);
    let image = render(&gpu, &mesh, None, 200.0, &visible(pose, environment));
    let sky = environment.sky_linear_rgb_at(pose.position_ecef);
    Some((image.pixel(WIDTH / 2, HEIGHT / 2).unwrap(), sky))
}

#[test]
fn the_rendered_sky_is_the_frames_sky_radiance() {
    let body = BodyEllipsoid::WGS84;
    let centre = Geodetic::from_degrees(39.1, -75.5, 0.0);
    let Some((pixel, sky)) = sky_pixel(body, noon(body, &centre)) else {
        return;
    };
    assert!(sky[2] > 0.5, "noon on Earth has a bright sky: {sky:?}");
    for (channel, linear) in pixel.iter().zip(sky) {
        assert!(
            channel.abs_diff(srgb_byte(linear)) <= 1,
            "{pixel:?} vs {sky:?}"
        );
    }
}

#[test]
fn the_lunar_sky_renders_black_at_noon() {
    let body = BodyEllipsoid::MOON_IAU;
    let centre = Geodetic::from_degrees(39.1, -75.5, 0.0);
    let environment = noon(body, &centre);
    assert!(environment.atmosphere.is_none());
    let Some((pixel, sky)) = sky_pixel(body, environment) else {
        return;
    };
    assert_eq!(sky, [0.0; 3]);
    assert_eq!(pixel, [0, 0, 0, 255]);
}

#[test]
fn clear_air_fog_leaves_terrain_untouched_and_thick_fog_hides_it() {
    let Some(gpu) = gpu() else { return };
    let body = BodyEllipsoid::WGS84;
    let centre = Geodetic::from_degrees(39.1, -75.5, 0.0);
    let mesh =
        GlobeMesh::build_region(&block(body, &centre, 0.02, 8), TerrainSources::default()).unwrap();
    let pose = nadir(&body, &centre, 20_000.0);
    let clear = noon(body, &centre);
    let fog_colour = [0.5, 0.25, 0.75];
    let tinted_but_clear = EnvironmentFrame {
        fog: Fog {
            extinction_per_m: 0.0,
            colour_linear: fog_colour,
        },
        ..clear
    };
    let thick = EnvironmentFrame {
        fog: Fog {
            extinction_per_m: 1.0e-3,
            colour_linear: fog_colour,
        },
        ..clear
    };
    let at_centre = |environment| {
        render(&gpu, &mesh, None, 400.0, &visible(pose, environment))
            .pixel(WIDTH / 2, HEIGHT / 2)
            .unwrap()
    };
    let ground = at_centre(clear);
    assert_eq!(at_centre(tinted_but_clear), ground);
    // 20 km through 1/km extinction transmits e^-20.
    let fogged = at_centre(thick);
    let expected = fog_colour.map(srgb_byte);
    for (channel, want) in fogged.iter().zip(expected) {
        assert!(channel.abs_diff(want) <= 1, "{fogged:?} vs {expected:?}");
    }
    assert_ne!(fogged, ground);
}
