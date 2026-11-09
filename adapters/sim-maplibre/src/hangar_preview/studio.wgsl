// Hangar studio: a ray-cast backdrop (graded cyclorama, floor grid, turntable ring, and
// contact shadow) behind OpenUSD aircraft parts lit by a warm key, a cool rim, a soft fill,
// and a backdrop-matched environment. Positions are camera-relative aircraft axes
// (`+X` right, `+Y` forward, `+Z` up, eye at the origin); depth is reversed-Z.
// Output is linear; the sRGB render target encodes it.

struct Frame {
    world_from_clip: mat4x4<f32>,
    // xyz eye in aircraft axes, w floor height
    eye: vec4<f32>,
    // xy turntable centre, z ring radius, w grid spacing
    floor: vec4<f32>,
    // xyz unit direction to the light, w intensity
    key: vec4<f32>,
    rim: vec4<f32>,
    fill: vec4<f32>,
    // xy contact-shadow half extents
    shadow: vec4<f32>,
};

struct Draw {
    clip_from_part: mat4x4<f32>,
    enu_from_part: mat4x4<f32>,
    // rgb diffuse reflectance, a opacity
    base_color: vec4<f32>,
    // rgb emissive radiance (already scaled), w metallic
    emissive: vec4<f32>,
    // x roughness, y propeller phase (rad), z shutter blur (rad), w kind (0 opaque, 1 glass, 2 disc)
    params: vec4<f32>,
    // xyz unit direction to the key light, w light level
    sun: vec4<f32>,
    // xyz panel seam pitch along part-local x, y, z (0 = none), w weathering
    detail: vec4<f32>,
    // x soot source |x|, y start station, z fade length, w strength
    soot: vec4<f32>,
};

@group(0) @binding(0) var<uniform> frame: Frame;
@group(0) @binding(1) var<uniform> draw: Draw;

const ZENITH: vec3<f32> = vec3<f32>(0.030, 0.036, 0.050);
const HORIZON: vec3<f32> = vec3<f32>(0.200, 0.215, 0.240);
const FLOOR: vec3<f32> = vec3<f32>(0.060, 0.063, 0.070);
const KEY_RGB: vec3<f32> = vec3<f32>(1.00, 0.88, 0.72);
const RIM_RGB: vec3<f32> = vec3<f32>(0.55, 0.72, 1.00);
const FILL_RGB: vec3<f32> = vec3<f32>(0.80, 0.85, 0.95);

const EXPOSURE: f32 = 1.5;

fn tonemap(c: vec3<f32>) -> vec4<f32> {
    return vec4<f32>(filmic(c, EXPOSURE), 1.0);
}

// Cyclorama radiance seen along `dir`: the graded wall above, the floor tone below.
fn environment(dir: vec3<f32>) -> vec3<f32> {
    let up = clamp(dir.z, -1.0, 1.0);
    let wall = mix(HORIZON, ZENITH, smoothstep(0.0, 0.75, up));
    let glow = pow(max(dot(dir, frame.key.xyz), 0.0), 6.0) * 0.12 * KEY_RGB;
    let below = mix(HORIZON * 0.7, FLOOR, smoothstep(0.0, -0.25, up));
    return select(wall + glow, below, up < 0.0);
}

struct BackdropOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) ndc: vec2<f32>,
};

@vertex
fn vs_backdrop(@builtin(vertex_index) index: u32) -> BackdropOut {
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    var out: BackdropOut;
    out.ndc = uv * 2.0 - vec2<f32>(1.0);
    // Reversed-Z far plane, so every aircraft fragment passes in front of it.
    out.clip = vec4<f32>(out.ndc, 0.0, 1.0);
    return out;
}

fn grid_lines(p: vec2<f32>, spacing: f32) -> f32 {
    let cell = p / spacing;
    let width = max(fwidth(cell), vec2<f32>(1e-4));
    let dist = abs(fract(cell - 0.5) - 0.5) / width;
    return 1.0 - clamp(min(dist.x, dist.y), 0.0, 1.0);
}

@fragment
fn fs_backdrop(in: BackdropOut) -> @location(0) vec4<f32> {
    let far = frame.world_from_clip * vec4<f32>(in.ndc, 0.5, 1.0);
    let dir = normalize(far.xyz / far.w);
    // Every derivative below is taken in uniform control flow; the floor is selected last.
    let denom = select(dir.z, -1e-4, dir.z > -1e-4);
    let t = (frame.eye.w - frame.eye.z) / denom;
    let hit = frame.eye.xyz + dir * max(t, 0.0);
    let rel = hit.xy - frame.floor.xy;
    let r = length(rel);
    let ring_radius = frame.floor.z;
    let pool = exp(-(r * r) / (2.0 * ring_radius * ring_radius * 2.2));
    var floor_rgb = FLOOR * (0.55 + 0.9 * pool) + KEY_RGB * 0.05 * pool;
    let minor = grid_lines(hit.xy, frame.floor.w) * 0.35 + grid_lines(hit.xy, frame.floor.w * 4.0) * 0.4;
    floor_rgb += vec3<f32>(0.05, 0.058, 0.07) * minor * (0.35 + 0.65 * pool);
    let ring_width = max(fwidth(r), 1e-4);
    let ring = 1.0 - clamp(abs(r - ring_radius) / (ring_width * 1.2 + ring_radius * 0.006), 0.0, 1.0);
    let deck = smoothstep(ring_radius + ring_width, ring_radius - ring_width, r);
    floor_rgb = mix(floor_rgb, floor_rgb * 1.25 + vec3<f32>(0.010, 0.011, 0.013), deck);
    floor_rgb += vec3<f32>(0.30, 0.36, 0.46) * ring * 0.45;
    let s = rel / max(frame.shadow.xy, vec2<f32>(1e-3));
    let shadow = exp(-dot(s, s) * 1.6) * 0.78;
    floor_rgb *= 1.0 - shadow;
    let fade = exp(-max(t, 0.0) / (ring_radius * 9.0));
    let ground = mix(environment(dir), floor_rgb, fade);
    let blend = smoothstep(-0.002, -0.03, dir.z);
    return tonemap(mix(environment(dir), ground, blend));
}

struct VertexIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) occlusion: f32,
};

struct VertexOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) occlusion: f32,
    @location(4) local: vec3<f32>,
    @location(5) local_normal: vec3<f32>,
};

@vertex
fn vs(in: VertexIn) -> VertexOut {
    var out: VertexOut;
    out.clip = draw.clip_from_part * vec4<f32>(in.position, 1.0);
    out.world = (draw.enu_from_part * vec4<f32>(in.position, 1.0)).xyz;
    out.normal = (draw.enu_from_part * vec4<f32>(in.normal, 0.0)).xyz;
    out.uv = in.uv;
    out.occlusion = in.occlusion;
    out.local = in.position;
    out.local_normal = in.normal;
    return out;
}

fn light(n: vec3<f32>, v: vec3<f32>, l: vec3<f32>, radiance: vec3<f32>, albedo: vec3<f32>, f0: vec3<f32>, roughness: f32) -> vec3<f32> {
    let metallic = draw.emissive.w;
    let n_dot_l = max(dot(n, l), 0.0);
    let n_dot_v = max(dot(n, v), 0.001);
    let h = normalize(l + v);
    let f = fresnel(f0, dot(h, v));
    let spec = f * ggx(max(dot(n, h), 0.0), roughness) * 0.25 / max(n_dot_v * max(n_dot_l, 0.001), 0.05);
    let diffuse = (vec3<f32>(1.0) - f) * (1.0 - metallic) * albedo / PI;
    return (diffuse + spec * min(n_dot_l * 4.0, 1.0)) * radiance * n_dot_l;
}

fn studio(in: VertexOut, front: bool) -> vec3<f32> {
    var n = normalize(in.normal);
    if (!front) {
        n = -n;
    }
    let v = normalize(-in.world);
    let metallic = draw.emissive.w;
    let finish = paint(draw.base_color.rgb, draw.params.x, in.local, in.local_normal, in.occlusion, draw.detail, draw.soot);
    let roughness = finish.roughness;
    let albedo = finish.albedo;
    let f0 = mix(vec3<f32>(0.04), albedo, metallic);
    var color = light(n, v, frame.key.xyz, KEY_RGB * frame.key.w, albedo, f0, roughness);
    color += light(n, v, frame.rim.xyz, RIM_RGB * frame.rim.w, albedo, f0, roughness);
    color += light(n, v, frame.fill.xyz, FILL_RGB * frame.fill.w, albedo, f0, roughness);
    // Studio bounce: the bright floor pool lights undersides, the dark ceiling the tops.
    let hemi = mix(FLOOR * 4.0, HORIZON * 1.6, n.z * 0.5 + 0.5);
    let ao = clamp(in.occlusion, 0.0, 1.0) * finish.cavity;
    color += hemi * albedo * (1.0 - 0.7 * metallic) * ao;
    // Metals and gloss coats mirror the cyclorama so they never read as flat black.
    let r = reflect(-v, n);
    let gloss = 1.0 - roughness * 0.75;
    color += environment(r) * 3.0 * fresnel(f0, dot(n, v)) * gloss * mix(ao * ao, 1.0, 0.35);
    return color + draw.emissive.rgb;
}

@fragment
fn fs_opaque(in: VertexOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    return tonemap(studio(in, front));
}

fn glass(in: VertexOut, front: bool) -> vec4<f32> {
    var n = normalize(in.normal);
    if (!front) {
        n = -n;
    }
    let v = normalize(-in.world);
    let r = reflect(-v, n);
    let env = environment(r) * 2.2 + KEY_RGB * pow(max(dot(r, frame.key.xyz), 0.0), 40.0) * 2.0;
    let f = 0.04 + 0.96 * pow(1.0 - clamp(abs(dot(n, v)), 0.0, 1.0), 5.0);
    let tint = draw.base_color.rgb * 0.25;
    let alpha = clamp(draw.base_color.a + f * 0.6, 0.0, 0.92);
    let rgb = mix(tint, env, clamp(f * 1.6 + 0.25, 0.0, 1.0));
    let mapped = tonemap(rgb).rgb;
    return vec4<f32>(mapped * alpha, alpha);
}

fn propeller_disc(in: VertexOut) -> vec4<f32> {
    // The disc's texture coordinates span the unit disc about the hub.
    let uv = in.uv;
    let r = length(uv);
    if (r > 1.0 || r < 0.12) {
        discard;
    }
    let radial = smoothstep(0.12, 0.22, r) * (1.0 - smoothstep(0.86, 0.99, r)) * (0.55 + 0.45 * sin(r * PI));
    let blades = select(2.0, 3.0, draw.params.w > 2.5);
    let theta = atan2(uv.y, uv.x) - draw.params.y;
    let blur = max(draw.params.z, 0.05);
    let wedge = pow(0.5 + 0.5 * cos(blades * theta), 1.0 + 6.0 / (1.0 + blur * 4.0));
    let tip = smoothstep(0.945, 0.962, r) * (1.0 - smoothstep(0.986, 0.998, r));
    let base = draw.base_color.a;
    // `base` is the disc's peak opacity (the articulation's `disc_opacity`).
    let blade_alpha = base * radial * (0.30 + 0.70 * wedge);
    let tip_alpha = base * tip * (0.45 + 0.55 * wedge) * 0.48;
    let blade_rgb = draw.base_color.rgb;
    let tip_rgb = select(vec3<f32>(0.86, 0.88, 0.90), draw.emissive.rgb, length(draw.emissive.rgb) > 0.05);
    let alpha = clamp(blade_alpha + tip_alpha, 0.0, 0.34);
    return vec4<f32>(blade_rgb * blade_alpha + tip_rgb * tip_alpha, alpha);
}

@fragment
fn fs_translucent(in: VertexOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    if (draw.params.w > 1.5) {
        return propeller_disc(in);
    }
    return glass(in, front);
}
