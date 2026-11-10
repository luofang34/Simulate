// OpenUSD aircraft parts over maplibre terrain: UsdPreviewSurface-style PBR for opaque
// parts with ownship's sun shadow (shadow.wgsl), Fresnel glass, and a shutter-blurred
// two-blade propeller disc. Positions and
// lighting are in the view slot's camera-anchored ENU frame (camera at the origin), and
// clip depth is reversed-Z so terrain occludes aircraft with GreaterEqual testing.

struct Draw {
    clip_from_part: mat4x4<f32>,
    enu_from_part: mat4x4<f32>,
    // rgb diffuse reflectance, a opacity
    base_color: vec4<f32>,
    // rgb emissive radiance (already scaled), w metallic
    emissive: vec4<f32>,
    // x roughness, y propeller phase (rad), z shutter blur (rad), w kind (0 opaque, 1 glass,
    // 2 disc, 4 printed from the display atlas, 5 painted by the baked skin)
    params: vec4<f32>,
    // xyz unit direction to the sun in ENU, w daylight factor
    sun: vec4<f32>,
    // xyz panel seam pitch along part-local x, y, z (0 = none), w weathering
    detail: vec4<f32>,
    // x soot source |x|, y start station, z fade length, w strength
    soot: vec4<f32>,
    // where a display part's uv 0..1 lands in the atlas: xy origin, zw extent
    display: vec4<f32>,
    // xyz the ultraviolet flood lamp in ENU, w its level; w < 0 for a part
    // whose emission it does not drive
    lamp: vec4<f32>,
};

// Distance from the UV flood lamp at which luminous paint shows its rated glow.
const LAMP_REFERENCE_M: f32 = 0.35;

@group(0) @binding(0) var<uniform> draw: Draw;
// The asset's baked skin (see `surface_finish`).
@group(2) @binding(0) var skin_albedo: texture_2d<f32>;
@group(2) @binding(1) var skin_normal: texture_2d<f32>;
@group(2) @binding(2) var skin_orm: texture_2d<f32>;
@group(2) @binding(3) var skin_sampler: sampler;
// Dial faces printed once: sRGB colour, and in alpha the glow of luminous paint.
@group(2) @binding(4) var display_atlas: texture_2d<f32>;
@group(2) @binding(5) var display_sampler: sampler;

struct VertexIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) occlusion: f32,
    @location(4) tangent: vec4<f32>,
};

struct VertexOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) occlusion: f32,
    @location(4) local: vec3<f32>,
    @location(5) local_normal: vec3<f32>,
    @location(6) tangent: vec4<f32>,
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
    out.tangent = vec4<f32>((draw.enu_from_part * vec4<f32>(in.tangent.xyz, 0.0)).xyz, in.tangent.w);
    return out;
}

const SKY: vec3<f32> = vec3<f32>(0.42, 0.55, 0.72);
const GROUND: vec3<f32> = vec3<f32>(0.20, 0.19, 0.16);
const EXPOSURE: f32 = 1.5;
// Share of the occluded sky light that comes back off the surrounding surfaces.
const BOUNCE_FILL: f32 = 0.35;

// A display part's printed colour and glow; other parts keep their material.
struct Surface {
    albedo: vec3<f32>,
    emissive: vec3<f32>,
};

fn surface(in: VertexOut) -> Surface {
    // Sampled in uniform control flow for every part, then chosen by kind.
    let printed = textureSample(display_atlas, display_sampler, draw.display.xy + clamp(in.uv, vec2<f32>(0.0), vec2<f32>(1.0)) * draw.display.zw);
    let display = abs(draw.params.w - 4.0) < 0.5;
    var out: Surface;
    out.albedo = select(draw.base_color.rgb, printed.rgb, display);
    // Luminous paint glows as the UV lamp's irradiance falls on it.
    let to_lamp = draw.lamp.xyz - in.world;
    let lamp_d = max(length(to_lamp), 0.05);
    let irradiance = max(dot(normalize(in.normal), to_lamp / lamp_d), 0.0)
        * (LAMP_REFERENCE_M * LAMP_REFERENCE_M) / (lamp_d * lamp_d);
    let flood = select(1.0, draw.lamp.w * irradiance, draw.lamp.w >= 0.0);
    out.emissive = draw.emissive.rgb * select(1.0, printed.a, display) * flood;
    return out;
}

fn shade(in: VertexOut, front: bool, printed: Surface) -> vec3<f32> {
    let world = in.world;
    var geometric = normalize(in.normal);
    if (!front) {
        geometric = -geometric;
    }
    let finish = surface_finish(printed.albedo, geometric, in.local, in.local_normal, in.occlusion, in.uv, in.tangent);
    let n = finish.normal;
    let v = normalize(-world);
    let l = normalize(draw.sun.xyz);
    let h = normalize(l + v);
    let daylight = draw.sun.w;
    let metallic = finish.metallic;
    let roughness = finish.roughness;
    let albedo = finish.albedo;
    let n_dot_l = max(dot(n, l), 0.0);
    let n_dot_v = max(dot(n, v), 0.001);
    let f0 = mix(vec3<f32>(0.04), albedo, metallic);
    let f = fresnel(f0, dot(h, v));
    let spec = f * ggx(max(dot(n, h), 0.0), roughness) * 0.25 / max(n_dot_v * max(n_dot_l, 0.001), 0.05);
    let diffuse = (vec3<f32>(1.0) - f) * (1.0 - metallic) * albedo / PI;
    let sun = vec3<f32>(3.1, 2.95, 2.75) * daylight * sun_visibility(world, geometric, in.clip.xy);
    // Sky above and sunlit ground below both light the airframe, so undersides never go black.
    // Ground light comes mostly from sunlit ground; under the airframe that
    // ground lies in its own shadow, so wells and bellies see less of it.
    let ground_light = ground_bounce(world, in.clip.xy);
    let hemi = mix(GROUND * 1.6 * ground_light, SKY * 1.15, n.z * 0.5 + 0.5) * (0.18 + 0.82 * daylight);
    let r = reflect(-v, n);
    let env = mix(GROUND * 1.35 * ground_light, SKY * 1.45, clamp(r.z * 0.5 + 0.5, 0.0, 1.0)) * (0.18 + 0.82 * daylight);
    let gloss = 1.0 - roughness * 0.72;
    let env_spec = env * fresnel(f0, n_dot_v) * gloss * (0.45 + 0.95 * metallic);
    // Baked occlusion shuts out sky and bounce light; reflections fade less, since a glancing
    // view still sees open sky past a junction.
    let ao = finish.occlusion;
    let specular_ao = mix(ao * ao, 1.0, 0.35);
    // What baked occlusion takes from the sky the surfaces around still pass on:
    // a cockpit or wheel well is lit by its own walls, so its shade is never black.
    let bounce = hemi * albedo * (1.0 - metallic) * (1.0 - ao) * BOUNCE_FILL;
    let ambient = (hemi * albedo * (1.0 - 0.6 * metallic) + hemi * f0 * 0.25) * ao + bounce + env_spec * specular_ao;
    return (diffuse + spec * min(n_dot_l * 4.0, 1.0)) * sun * n_dot_l + ambient + printed.emissive;
}

// Share of the ground light reaching `world` from the ground straight below
// it: 1 in flight, or where that ground is sunlit; down to the sky-lit
// share where ownship's shadow covers it.
const SHADED_GROUND: f32 = 0.3;

fn ground_bounce(world: vec3<f32>, pixel: vec2<f32>) -> f32 {
    if (shadow.ground_centre.w <= 0.0) {
        return 1.0;
    }
    let below = vec3<f32>(world.xy, shadow.ground_centre.z);
    let lit = sun_visibility(below, vec3<f32>(0.0), pixel);
    return mix(SHADED_GROUND, 1.0, lit);
}

@fragment
fn fs_opaque(in: VertexOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    let color = shade(in, front, surface(in));
    return vec4<f32>(filmic(color, EXPOSURE), 1.0);
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
    let rgb = filmic(mix(tint, sky, clamp(f * 1.6 + 0.25, 0.0, 1.0)), EXPOSURE);
    // The Sun's own mirror image on the pane, where the airframe does not shade it.
    let l = normalize(draw.sun.xyz);
    let h = normalize(l + v);
    let n_dot_l = max(dot(n, l), 0.0);
    let f_sun = 0.04 + 0.96 * pow(1.0 - clamp(dot(h, v), 0.0, 1.0), 5.0);
    let glint = ggx(max(dot(n, h), 0.0), 0.08) * f_sun * n_dot_l * 0.25 / max(abs(dot(n, v)), 0.2)
        * draw.sun.w * sun_visibility(in.world, n, in.clip.xy);
    let glint_rgb = filmic(vec3<f32>(3.1, 2.95, 2.75) * glint, EXPOSURE);
    return vec4<f32>(rgb * alpha + glint_rgb, alpha);
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
    // `base` is the disc's peak opacity (the articulation's `disc_opacity`).
    let blade_alpha = base * radial * (0.30 + 0.70 * wedge);
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
