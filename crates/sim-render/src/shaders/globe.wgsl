// Camera-relative globe. Vertex positions arrive relative to their patch
// centre; `patch_offsets` holds each patch centre minus the eye, computed in
// f64 on the CPU, so the eye is always at the origin here. The view matrix is
// rotation-only, so world axes are ECEF axes and the sun direction is ECEF.

const MAX_PATCHES: u32 = 512u;
const GRATICULE_COARSE_DEG: f32 = 10.0;
const GRATICULE_FINE_DEG: f32 = 1.0;
// Stefan-Boltzmann constant, W/m²/K⁴.
const SIGMA: f32 = 5.670374e-8;
// Scales low bare-ground albedo into the sensor's range.
const ALBEDO_EXPOSURE: f32 = 5.0;

struct Frame {
    view_proj: mat4x4<f32>,
    // Toward the Sun in xyz; exposure in w.
    sun_direction: vec4<f32>,
    // Black exitance, white exitance (W/m²), NEDT (K), noise seed.
    thermal: vec4<f32>,
    // Fog colour (linear, exposed) in rgb; extinction per metre in w.
    fog: vec4<f32>,
    patch_offsets: array<vec4<f32>, MAX_PATCHES>,
};

@group(0) @binding(0) var<uniform> frame: Frame;
@group(1) @binding(0) var ortho_texture: texture_2d<f32>;
@group(1) @binding(1) var ortho_sampler: sampler;

struct VertexIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) lat_lon: vec2<f32>,
    @location(3) patch_index: f32,
    @location(4) ortho_uv: vec2<f32>,
    // Albedo, thermal exitance (W/m²); negative where there is no data.
    @location(5) radiometry: vec2<f32>,
};

struct VertexOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) lat_lon_deg: vec2<f32>,
    @location(2) ortho_uv: vec2<f32>,
    @location(3) radiometry: vec2<f32>,
    @location(4) eye_relative: vec3<f32>,
};

@vertex
fn vs_main(v: VertexIn) -> VertexOut {
    let patch_slot = min(u32(v.patch_index + 0.5), MAX_PATCHES - 1u);
    let eye_relative = v.position + frame.patch_offsets[patch_slot].xyz;
    var out: VertexOut;
    out.clip = frame.view_proj * vec4<f32>(eye_relative, 1.0);
    out.normal = v.normal;
    out.lat_lon_deg = degrees(v.lat_lon);
    out.ortho_uv = v.ortho_uv;
    out.radiometry = v.radiometry;
    out.eye_relative = eye_relative;
    return out;
}

fn grid_line(coord_deg: vec2<f32>, spacing_deg: f32) -> f32 {
    let scaled = coord_deg / spacing_deg;
    let width = fwidth(scaled);
    let distance = abs(fract(scaled - 0.5) - 0.5) / max(width, vec2<f32>(1e-6));
    return 1.0 - clamp(min(distance.x, distance.y), 0.0, 1.0);
}

fn procedural_surface(in: VertexOut, daylight: f32) -> vec3<f32> {
    // Derivatives must run on every fragment, so this precedes the branch.
    let lines = max(grid_line(in.lat_lon_deg, GRATICULE_COARSE_DEG),
                    0.4 * grid_line(in.lat_lon_deg, GRATICULE_FINE_DEG));
    if in.radiometry.x >= 0.0 {
        return vec3<f32>(in.radiometry.x * ALBEDO_EXPOSURE * daylight);
    }
    let base = vec3<f32>(0.10, 0.22, 0.38);
    let lit = base * (0.08 + 0.92 * daylight);
    return mix(lit, vec3<f32>(0.85, 0.80, 0.55) * (0.25 + 0.75 * daylight), lines);
}

// Exposure, then Beer-Lambert extinction toward the fog colour. Clear air
// returns early so the exposed colour is not perturbed by `mix` rounding.
fn fogged(lit: vec3<f32>, eye_relative: vec3<f32>) -> vec3<f32> {
    let exposed = lit * frame.sun_direction.w;
    if frame.fog.w <= 0.0 {
        return exposed;
    }
    let transmittance = exp(-frame.fog.w * length(eye_relative));
    return mix(frame.fog.rgb, exposed, transmittance);
}

// Imagery already holds the light of its own exposure, so the Sun only
// modulates it. Where nothing is draped, zero alpha falls through to the
// procedural surface.
@fragment
fn fs_main(in: VertexOut) -> @location(0) vec4<f32> {
    let n = normalize(in.normal);
    let daylight = max(dot(n, normalize(frame.sun_direction.xyz)), 0.0);
    let ortho = textureSample(ortho_texture, ortho_sampler, clamp(in.ortho_uv, vec2<f32>(0.0), vec2<f32>(1.0)));
    let in_atlas = all(in.ortho_uv >= vec2<f32>(0.0)) && all(in.ortho_uv <= vec2<f32>(1.0));
    let procedural = procedural_surface(in, daylight);
    if in_atlas && ortho.a > 0.5 {
        return vec4<f32>(fogged(ortho.rgb * (0.35 + 0.65 * daylight), in.eye_relative), 1.0);
    }
    return vec4<f32>(fogged(procedural, in.eye_relative), 1.0);
}

fn pcg(input: u32) -> u32 {
    let state = input * 747796405u + 2891336453u;
    let word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

// A standard normal variate, the same for a given pixel and seed on every run.
fn pixel_gaussian(pixel: vec2<u32>, seed: u32) -> f32 {
    let first = pcg(pcg(pixel.x ^ seed) ^ pixel.y);
    let second = pcg(first);
    let a = max(f32(first) / 4294967296.0, 1e-6);
    let b = f32(second) / 4294967296.0;
    return sqrt(-2.0 * log(a)) * cos(6.2831855 * b);
}

fn srgb_to_linear(encoded: f32) -> f32 {
    if encoded <= 0.04045 {
        return encoded / 12.92;
    }
    return pow((encoded + 0.055) / 1.055, 2.4);
}

// The target encodes sRGB on write, so the level is decoded first and the
// stored byte is linear in exitance.
@fragment
fn fs_thermal(in: VertexOut) -> @location(0) vec4<f32> {
    let exitance = in.radiometry.y;
    if exitance < 0.0 {
        return vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }
    let kelvin = pow(exitance / SIGMA, 0.25);
    let noise_per_kelvin = 4.0 * SIGMA * kelvin * kelvin * kelvin;
    let noise = noise_per_kelvin * frame.thermal.z
        * pixel_gaussian(vec2<u32>(in.clip.xy), u32(frame.thermal.w));
    let span = max(frame.thermal.y - frame.thermal.x, 1e-6);
    let level = (exitance + noise - frame.thermal.x) / span;
    let grey = srgb_to_linear(clamp(level, 0.0, 1.0));
    return vec4<f32>(grey, grey, grey, 1.0);
}

// Brightness temperature in kelvin for a float target, with no display
// mapping or noise; -1 where the ground has no radiometry.
@fragment
fn fs_thermal_kelvin(in: VertexOut) -> @location(0) vec4<f32> {
    let exitance = in.radiometry.y;
    if exitance < 0.0 {
        return vec4<f32>(-1.0, 0.0, 0.0, 1.0);
    }
    return vec4<f32>(pow(exitance / SIGMA, 0.25), 0.0, 0.0, 1.0);
}
