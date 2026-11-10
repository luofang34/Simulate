// Daylight of the slot (`usd_pass/sky.rs`): the Sun's beam after its air mass, the clear
// sky's dome irradiance as nine cosine-convolved spherical harmonics, and the sunlit ground.

struct Sky {
    irradiance_sh: array<vec4<f32>, 9>,
    // rgb beam irradiance at normal incidence, w exposure gain over a clear noon
    sun: vec4<f32>,
    // rgb zenith radiance, w the CIE clear-sky luminance at the zenith
    zenith: vec4<f32>,
    // rgb radiance of the horizon colour at zenith luminance
    horizon: vec4<f32>,
    // rgb radiance of the sunlit ground below the horizon, w the share of it shaded ground keeps
    ground: vec4<f32>,
};

@group(1) @binding(3) var<uniform> sky: Sky;

// Sky-dome irradiance on a surface facing `n`.
fn sky_irradiance(n: vec3<f32>) -> vec3<f32> {
    let c = sky.irradiance_sh;
    var e = c[0].rgb * 0.282095
        + c[1].rgb * (0.488603 * n.y) + c[2].rgb * (0.488603 * n.z) + c[3].rgb * (0.488603 * n.x)
        + c[4].rgb * (1.092548 * n.x * n.y) + c[5].rgb * (1.092548 * n.y * n.z)
        + c[6].rgb * (0.315392 * (3.0 * n.z * n.z - 1.0)) + c[7].rgb * (1.092548 * n.x * n.z)
        + c[8].rgb * (0.546274 * (n.x * n.x - n.y * n.y));
    return max(e, vec3<f32>(0.0));
}

// Irradiance from the ground below the horizon on a surface facing `n`; `lit` dims it where
// that ground lies in shade.
fn ground_irradiance(n: vec3<f32>, lit: f32) -> vec3<f32> {
    return sky.ground.rgb * PI * clamp(0.5 - 0.5 * n.z, 0.0, 1.0) * lit;
}

// CIE standard clear sky (type 12) relative luminance; mirrors `sky::cie_clear`.
fn cie_clear(dir: vec3<f32>, to_sun: vec3<f32>) -> f32 {
    let cos_zenith = max(dir.z, 0.01);
    let gradation = 1.0 - exp(-0.32 / cos_zenith);
    let chi = acos(clamp(dot(dir, to_sun), -1.0, 1.0));
    let indicatrix = 1.0 + 10.0 * (exp(-3.0 * chi) - exp(-3.0 * 1.5707963)) + 0.45 * cos(chi) * cos(chi);
    return gradation * indicatrix;
}

// Radiance seen along `dir`: the clear sky above the horizon (mirrors `sky::Shape::radiance`)
// and the ground below, blended across a narrow horizon band.
fn sky_radiance(dir: vec3<f32>, to_sun: vec3<f32>, ground_lit: f32) -> vec3<f32> {
    let up = max(dir.z, 0.0);
    let colour = mix(sky.horizon.rgb, sky.zenith.rgb, sqrt(up));
    let above = colour * cie_clear(vec3<f32>(dir.xy, max(dir.z, 0.0)), to_sun) / max(sky.zenith.w, 1e-6);
    return mix(sky.ground.rgb * ground_lit, above, smoothstep(-0.04, 0.02, dir.z));
}

// What a glossy surface reflects along `r`: the sky's radiance sharpened for a smooth
// finish, the dome's cosine-blurred light for a rough one.
fn sky_reflection(r: vec3<f32>, to_sun: vec3<f32>, roughness: f32, ground_lit: f32) -> vec3<f32> {
    let blurred = (sky_irradiance(r) + ground_irradiance(r, ground_lit)) / PI;
    let sharp = sky_radiance(r, to_sun, ground_lit);
    // A satin paint still mirrors the bright sky near the horizon at a glancing view;
    // only a flat one sees the dome evenly.
    return mix(sharp, blurred, smoothstep(0.3, 0.9, roughness));
}
