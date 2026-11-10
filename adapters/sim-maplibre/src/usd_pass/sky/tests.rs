use glam::{DVec3, Vec3};

use super::{SKY_BYTES, Shape, SkyLight, cie_clear};

/// Brute-force sky-dome irradiance on `n`, for checking the harmonics.
fn integrate(light: &SkyLight, to_sun: Vec3, n: Vec3) -> Vec3 {
    let shape = Shape {
        to_sun,
        zenith: light.zenith,
        horizon: light.horizon,
        cie_zenith: light.cie_zenith,
    };
    let (rings, sectors) = (96, 192);
    let d_theta = std::f32::consts::FRAC_PI_2 / rings as f32;
    let d_phi = std::f32::consts::TAU / sectors as f32;
    let mut sum = Vec3::ZERO;
    for ring in 0..rings {
        let theta = (ring as f32 + 0.5) * d_theta;
        for sector in 0..sectors {
            let phi = (sector as f32 + 0.5) * d_phi;
            let dir = Vec3::new(
                theta.sin() * phi.cos(),
                theta.sin() * phi.sin(),
                theta.cos(),
            );
            sum += shape.radiance(dir) * dir.dot(n).max(0.0) * theta.sin() * d_theta * d_phi;
        }
    }
    sum
}

#[test]
fn harmonics_reproduce_the_dome_irradiance_on_every_facing() {
    for elevation_deg in [8.0_f64, 35.0, 70.0] {
        let e = elevation_deg.to_radians();
        let to_sun = DVec3::new(0.0, e.cos(), e.sin());
        let light = SkyLight::new(to_sun, 1.0);
        let s = to_sun.as_vec3();
        let up = integrate(&light, s, Vec3::Z);
        for n in [
            Vec3::Z,
            Vec3::Y,
            -Vec3::Y,
            Vec3::X,
            -Vec3::Z,
            Vec3::new(0.0, 0.7, -0.7).normalize(),
        ] {
            let exact = integrate(&light, s, n);
            let fitted = light.sky_irradiance(n);
            let error = (fitted - exact).abs().max_element() / up.max_element();
            assert!(error < 0.07, "{elevation_deg}° n {n}: {fitted} vs {exact}");
        }
    }
}

#[test]
fn shade_to_sun_ratios_follow_a_clear_day() {
    let high = SkyLight::new(DVec3::new(0.0, 0.34, 0.94), 1.0);
    let ratio = high.sky_irradiance(Vec3::Z).y / (high.sun.y * 0.94);
    assert!((0.09..0.2).contains(&ratio), "high Sun sky/beam {ratio}");
    let low = SkyLight::new(DVec3::new(0.0, 0.99, 0.14), 1.0);
    let low_ratio = low.sky_irradiance(Vec3::Z).y / (low.sun.y * 0.14);
    assert!(
        low_ratio > 0.6,
        "low Sun sky/beam on the ground {low_ratio}"
    );
    // A low Sun is red against a blue sky.
    assert!(low.sun.x / low.sun.z > 1.2 * high.sun.x / high.sun.z);
    let sky = low.sky_irradiance(Vec3::Z);
    assert!(sky.z > sky.x);
    // The sky is brightest around the Sun.
    let toward = cie_clear(
        Vec3::new(0.0, 0.95, 0.3).normalize(),
        Vec3::new(0.0, 0.99, 0.14),
    );
    let away = cie_clear(
        Vec3::new(0.0, -0.95, 0.3).normalize(),
        Vec3::new(0.0, 0.99, 0.14),
    );
    assert!(toward > 3.0 * away);
}

#[test]
fn night_keeps_a_dim_floor_and_the_block_packs_in_shader_order() {
    let night = SkyLight::new(DVec3::new(0.0, 0.5, -0.5), 0.0);
    assert_eq!(night.sun, Vec3::ZERO);
    let floor = night.sky_irradiance(Vec3::Z).y;
    assert!(floor > 0.0 && floor < 0.05, "{floor}");
    let day = SkyLight::new(DVec3::Z, 1.0);
    let bytes = day.block();
    assert_eq!(bytes.len(), SKY_BYTES);
    let read = |i: usize| f32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap());
    assert_eq!(read(0), day.irradiance_sh[0].x);
    assert_eq!(read(36), day.sun.x);
    assert_eq!(read(43), day.cie_zenith);
    assert_eq!(read(48), day.ground.x);
    assert_eq!(read(51), day.shaded_ground);
    // Shaded ground keeps the sky's share of its light: little under a high Sun.
    assert!((0.05..0.25).contains(&day.shaded_ground), "{day:?}");
}
