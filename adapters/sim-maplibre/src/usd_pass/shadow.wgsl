// Ownship sun shadow: the depth-only caster stage, the ground receiver under the
// airframe, and the percentage-closer lookup with a blocker search, so penumbrae widen
// with the gap between occluder and receiver as they do under the Sun's disc.

struct Shadow {
    clip_from_enu: mat4x4<f32>,
    // ENU to the light's orthographic clip volume (x, y in [-1, 1], depth in [0, 1])
    light_from_enu: mat4x4<f32>,
    // xyz ground receiver centre in ENU, w darkening strength (0 without a receiver)
    ground_centre: vec4<f32>,
    // xyz half axis along the light's azimuth, w depth range of the light volume (m)
    ground_u: vec4<f32>,
    // xyz half axis across it, w 1 when the slot has a light volume
    ground_v: vec4<f32>,
    // x texel (uv), y receiver normal offset (m), z penumbra per metre of gap, w uv per metre
    params: vec4<f32>,
};

@group(1) @binding(0) var<uniform> shadow: Shadow;
@group(1) @binding(1) var shadow_map: texture_depth_2d;
@group(1) @binding(2) var shadow_sampler: sampler_comparison;

var<private> POISSON: array<vec2<f32>, 12> = array<vec2<f32>, 12>(
    vec2<f32>(-0.326, -0.406),
    vec2<f32>(-0.840, -0.074),
    vec2<f32>(-0.696, 0.457),
    vec2<f32>(-0.203, 0.621),
    vec2<f32>(0.962, -0.195),
    vec2<f32>(0.473, -0.480),
    vec2<f32>(0.519, 0.767),
    vec2<f32>(0.185, -0.893),
    vec2<f32>(0.507, 0.064),
    vec2<f32>(0.896, 0.412),
    vec2<f32>(-0.322, -0.933),
    vec2<f32>(-0.792, -0.598),
);

@vertex
fn vs_shadow(in: VertexIn) -> @builtin(position) vec4<f32> {
    return shadow.light_from_enu * (draw.enu_from_part * vec4<f32>(in.position, 1.0));
}

// Per-pixel rotation of the tap pattern; neighbouring pixels average the residual noise.
fn interleaved_noise(pixel: vec2<f32>) -> f32 {
    return fract(52.9829189 * fract(dot(pixel, vec2<f32>(0.06711056, 0.00583715))));
}

// Fraction of the Sun reaching `world`: 1 outside the light volume or without a shadow.
fn sun_visibility(world: vec3<f32>, normal: vec3<f32>, pixel: vec2<f32>) -> f32 {
    if (shadow.ground_v.w < 0.5) {
        return 1.0;
    }
    let p = shadow.light_from_enu * vec4<f32>(world + normal * shadow.params.y, 1.0);
    let uv = vec2<f32>(p.x * 0.5 + 0.5, 0.5 - p.y * 0.5);
    if (any(uv < vec2<f32>(0.0)) || any(uv > vec2<f32>(1.0)) || p.z < 0.0) {
        return 1.0;
    }
    // Receivers past the far plane (distant ground) clamp to it and stay comparable.
    let depth = min(p.z, 1.0);
    let range = shadow.ground_u.w;
    let uv_per_m = shadow.params.w;
    let bias = 0.012 / range;
    let angle = interleaved_noise(pixel) * 6.2831853;
    let rotate = mat2x2<f32>(cos(angle), sin(angle), -sin(angle), cos(angle));
    let size = vec2<f32>(textureDimensions(shadow_map));
    var blocker_sum = 0.0;
    var blockers = 0.0;
    for (var i = 0u; i < 12u; i++) {
        let tap = clamp(uv + rotate * POISSON[i] * (0.45 * uv_per_m), vec2<f32>(0.0), vec2<f32>(1.0));
        let d = textureLoad(shadow_map, vec2<i32>(tap * (size - vec2<f32>(1.0))), 0);
        if (d < depth - bias) {
            blocker_sum += d;
            blockers += 1.0;
        }
    }
    if (blockers < 0.5) {
        return 1.0;
    }
    let gap_m = (depth - blocker_sum / blockers) * range;
    let radius = max(shadow.params.x * 1.5, gap_m * shadow.params.z * uv_per_m);
    var lit = 0.0;
    for (var i = 0u; i < 12u; i++) {
        lit += textureSampleCompareLevel(shadow_map, shadow_sampler, uv + rotate * POISSON[i] * radius, depth - bias);
    }
    return lit / 12.0;
}

struct GroundOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world: vec3<f32>,
    @location(1) quad: vec2<f32>,
};

@vertex
fn vs_ground(@builtin(vertex_index) index: u32) -> GroundOut {
    let corner = vec2<f32>(f32(index & 1u) * 2.0 - 1.0, f32((index >> 1u) & 1u) * 2.0 - 1.0);
    let world = shadow.ground_centre.xyz + shadow.ground_u.xyz * corner.x + shadow.ground_v.xyz * corner.y;
    var out: GroundOut;
    out.clip = shadow.clip_from_enu * vec4<f32>(world, 1.0);
    out.world = world;
    out.quad = corner;
    return out;
}

@fragment
fn fs_ground(in: GroundOut) -> @location(0) vec4<f32> {
    let edge = 1.0 - smoothstep(0.85, 1.0, max(abs(in.quad.x), abs(in.quad.y)));
    let dark = (1.0 - sun_visibility(in.world, vec3<f32>(0.0), in.clip.xy)) * shadow.ground_centre.w * edge;
    return vec4<f32>(0.0, 0.0, 0.0, dark);
}
