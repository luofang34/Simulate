// Shading shared by the aircraft pass over terrain and the hangar studio, prepended to
// each pass's own shader so both put the same airframe through the same BRDF and curve.

const PI: f32 = 3.14159265;

fn ggx(n_dot_h: f32, roughness: f32) -> f32 {
    let a = max(roughness * roughness, 0.02);
    let a2 = a * a;
    let d = n_dot_h * n_dot_h * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

fn fresnel(f0: vec3<f32>, cos_theta: f32) -> vec3<f32> {
    return f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - clamp(cos_theta, 0.0, 1.0), 5.0);
}

// Stephen Hill's fit of the ACES RRT + sRGB ODT. Highlights roll off and desaturate
// instead of clipping, and the toe deepens contact shadows; output stays linear for the
// sRGB target to encode.
const ACES_INPUT: mat3x3<f32> = mat3x3<f32>(
    vec3<f32>(0.59719, 0.07600, 0.02840),
    vec3<f32>(0.35458, 0.90834, 0.13383),
    vec3<f32>(0.04823, 0.01566, 0.83777),
);
const ACES_OUTPUT: mat3x3<f32> = mat3x3<f32>(
    vec3<f32>(1.60475, -0.10208, -0.00327),
    vec3<f32>(-0.53108, 1.10813, -0.07276),
    vec3<f32>(-0.07367, -0.00605, 1.07602),
);

fn rrt_and_odt(v: vec3<f32>) -> vec3<f32> {
    let a = v * (v + vec3<f32>(0.0245786)) - vec3<f32>(0.000090537);
    let b = v * (0.983729 * v + vec3<f32>(0.4329510)) + vec3<f32>(0.238081);
    return a / b;
}

// The fit maps scene 0.18 to about 0.105; an exposure near 1.5 keeps mid-grey where the
// terrain under the aircraft puts it.
fn filmic(radiance: vec3<f32>, exposure: f32) -> vec3<f32> {
    let graded = ACES_OUTPUT * rrt_and_odt(ACES_INPUT * (max(radiance, vec3<f32>(0.0)) * exposure));
    return clamp(graded, vec3<f32>(0.0), vec3<f32>(1.0));
}
