// Camera-relative globe. Vertex positions arrive relative to their patch
// centre; `patch_offsets` holds each patch centre minus the eye, computed in
// f64 on the CPU, so the eye is always at the origin here. The view matrix is
// rotation-only, so world axes are ECEF axes and the sun direction is ECEF.

const MAX_PATCHES: u32 = 512u;
const GRATICULE_COARSE_DEG: f32 = 10.0;
const GRATICULE_FINE_DEG: f32 = 1.0;

struct Frame {
    view_proj: mat4x4<f32>,
    sun_direction: vec4<f32>,
    patch_offsets: array<vec4<f32>, MAX_PATCHES>,
};

@group(0) @binding(0) var<uniform> frame: Frame;

struct VertexIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) lat_lon: vec2<f32>,
    @location(3) patch_index: f32,
};

struct VertexOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) lat_lon_deg: vec2<f32>,
};

@vertex
fn vs_main(v: VertexIn) -> VertexOut {
    let patch_slot = min(u32(v.patch_index + 0.5), MAX_PATCHES - 1u);
    let eye_relative = v.position + frame.patch_offsets[patch_slot].xyz;
    var out: VertexOut;
    out.clip = frame.view_proj * vec4<f32>(eye_relative, 1.0);
    out.normal = v.normal;
    out.lat_lon_deg = degrees(v.lat_lon);
    return out;
}

fn grid_line(coord_deg: vec2<f32>, spacing_deg: f32) -> f32 {
    let scaled = coord_deg / spacing_deg;
    let width = fwidth(scaled);
    let distance = abs(fract(scaled - 0.5) - 0.5) / max(width, vec2<f32>(1e-6));
    return 1.0 - clamp(min(distance.x, distance.y), 0.0, 1.0);
}

@fragment
fn fs_main(in: VertexOut) -> @location(0) vec4<f32> {
    let n = normalize(in.normal);
    let daylight = max(dot(n, normalize(frame.sun_direction.xyz)), 0.0);
    let base = vec3<f32>(0.10, 0.22, 0.38);
    let lines = max(grid_line(in.lat_lon_deg, GRATICULE_COARSE_DEG),
                    0.4 * grid_line(in.lat_lon_deg, GRATICULE_FINE_DEG));
    let lit = base * (0.08 + 0.92 * daylight);
    let colour = mix(lit, vec3<f32>(0.85, 0.80, 0.55) * (0.25 + 0.75 * daylight), lines);
    return vec4<f32>(colour, 1.0);
}
