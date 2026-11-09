// OpenUSD aircraft parts over maplibre terrain: UsdPreviewSurface-style PBR for opaque
// parts, Fresnel glass, and a shutter-blurred two-blade propeller disc. Positions and
// lighting are in the view slot's camera-anchored ENU frame (camera at the origin), and
// clip depth is reversed-Z so terrain occludes aircraft with GreaterEqual testing.

struct Draw {
    clip_from_part: mat4x4<f32>,
    enu_from_part: mat4x4<f32>,
    // rgb diffuse reflectance, a opacity
    base_color: vec4<f32>,
    // rgb emissive radiance (already scaled), w metallic
    emissive: vec4<f32>,
    // x roughness, y propeller phase (rad), z shutter blur (rad), w kind (0 opaque, 1 glass, 2 disc)
    params: vec4<f32>,
    // xyz unit direction to the sun in ENU, w daylight factor
    sun: vec4<f32>,
};

@group(0) @binding(0) var<uniform> draw: Draw;

struct VertexIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

struct VertexOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

@vertex
fn vs(in: VertexIn) -> VertexOut {
    var out: VertexOut;
    out.clip = draw.clip_from_part * vec4<f32>(in.position, 1.0);
    out.world = (draw.enu_from_part * vec4<f32>(in.position, 1.0)).xyz;
    out.normal = (draw.enu_from_part * vec4<f32>(in.normal, 0.0)).xyz;
    out.uv = in.uv;
    return out;
}

const PI: f32 = 3.14159265;
const SKY: vec3<f32> = vec3<f32>(0.42, 0.55, 0.72);
const GROUND: vec3<f32> = vec3<f32>(0.20, 0.19, 0.16);

fn ggx(n_dot_h: f32, roughness: f32) -> f32 {
    let a = max(roughness * roughness, 0.02);
    let a2 = a * a;
    let d = n_dot_h * n_dot_h * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

fn fresnel(f0: vec3<f32>, cos_theta: f32) -> vec3<f32> {
    return f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - clamp(cos_theta, 0.0, 1.0), 5.0);
}

fn shade(n_in: vec3<f32>, world: vec3<f32>, front: bool) -> vec3<f32> {
    var n = normalize(n_in);
    if (!front) {
        n = -n;
    }
    let v = normalize(-world);
    let l = normalize(draw.sun.xyz);
    let h = normalize(l + v);
    let daylight = draw.sun.w;
    let metallic = draw.emissive.w;
    let roughness = clamp(draw.params.x, 0.04, 1.0);
    let albedo = draw.base_color.rgb;
    let n_dot_l = max(dot(n, l), 0.0);
    let n_dot_v = max(dot(n, v), 0.001);
    let f0 = mix(vec3<f32>(0.04), albedo, metallic);
    let f = fresnel(f0, dot(h, v));
    let spec = f * ggx(max(dot(n, h), 0.0), roughness) * 0.25 / max(n_dot_v * max(n_dot_l, 0.001), 0.05);
    let diffuse = (vec3<f32>(1.0) - f) * (1.0 - metallic) * albedo / PI;
    let sun = vec3<f32>(3.1, 2.95, 2.75) * daylight;
    // Sky above and sunlit ground below both light the airframe, so undersides never go black.
    let hemi = mix(GROUND * 1.6, SKY * 1.15, n.z * 0.5 + 0.5) * (0.18 + 0.82 * daylight);
    let r = reflect(-v, n);
    let env = mix(GROUND * 1.35, SKY * 1.45, clamp(r.z * 0.5 + 0.5, 0.0, 1.0)) * (0.18 + 0.82 * daylight);
    let gloss = 1.0 - roughness * 0.72;
    let env_spec = env * fresnel(f0, n_dot_v) * gloss * (0.45 + 0.95 * metallic);
    let ambient = hemi * albedo * (1.0 - 0.6 * metallic) + hemi * f0 * 0.25 + env_spec;
    return (diffuse + spec * min(n_dot_l * 4.0, 1.0)) * sun * n_dot_l + ambient + draw.emissive.rgb;
}

@fragment
fn fs_opaque(in: VertexOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    let color = shade(in.normal, in.world, front);
    return vec4<f32>(color / (vec3<f32>(1.0) + color * 0.18), 1.0);
}

fn glass(in: VertexOut, front: bool) -> vec4<f32> {
    var n = normalize(in.normal);
    if (!front) {
        n = -n;
    }
    let v = normalize(-in.world);
    let r = reflect(-v, n);
    let sky = mix(GROUND, SKY * 1.4, clamp(r.z * 0.5 + 0.5, 0.0, 1.0)) * (0.2 + 0.8 * draw.sun.w);
    let f = 0.04 + 0.96 * pow(1.0 - clamp(abs(dot(n, v)), 0.0, 1.0), 5.0);
    let tint = draw.base_color.rgb * 0.25;
    let alpha = clamp(draw.base_color.a + f * 0.6, 0.0, 0.92);
    let rgb = mix(tint, sky, clamp(f * 1.6 + 0.25, 0.0, 1.0));
    return vec4<f32>(rgb * alpha, alpha);
}

fn propeller_disc(in: VertexOut) -> vec4<f32> {
    // The disc's texture coordinates span the unit disc about the hub.
    let uv = in.uv;
    let r = length(uv);
    if (r > 1.0 || r < 0.12) {
        discard;
    }
    // Blade density peaks mid-span and fades at the spinner and the tips.
    let radial = smoothstep(0.12, 0.22, r) * (1.0 - smoothstep(0.86, 0.99, r)) * (0.55 + 0.45 * sin(r * PI));
    // Two or three blades smeared over the shutter interval.
    let blades = select(2.0, 3.0, draw.params.w > 2.5);
    let theta = atan2(uv.y, uv.x) - draw.params.y;
    let blur = max(draw.params.z, 0.05);
    let wedge = pow(0.5 + 0.5 * cos(blades * theta), 1.0 + 6.0 / (1.0 + blur * 4.0));
    let tip = smoothstep(0.945, 0.962, r) * (1.0 - smoothstep(0.986, 0.998, r));
    let base = draw.base_color.a;
    let blade_alpha = base * 0.34 * radial * (0.30 + 0.70 * wedge);
    let tip_alpha = base * tip * (0.45 + 0.55 * wedge) * 0.48;
    let blade_rgb = draw.base_color.rgb * (0.4 + 0.6 * draw.sun.w);
    let raw_tip = select(vec3<f32>(0.86, 0.88, 0.90), draw.emissive.rgb, length(draw.emissive.rgb) > 0.05);
    let tip_rgb = raw_tip * (0.4 + 0.6 * draw.sun.w);
    let alpha = clamp(blade_alpha + tip_alpha, 0.0, 0.34);
    let rgb = (blade_rgb * blade_alpha + tip_rgb * tip_alpha);
    return vec4<f32>(rgb, alpha);
}

@fragment
fn fs_translucent(in: VertexOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    if (draw.params.w > 1.5) {
        return propeller_disc(in);
    }
    return glass(in, front);
}
