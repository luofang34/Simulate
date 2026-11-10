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

// Exposure at a clear noon, metered as a photographer meters full Sun: an 18 % grey card
// lit by beam and sky (radiance about 0.2 here) lands on the tone curve where it comes
// out display mid-grey.
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
    let metallic = finish.metallic;
    let roughness = finish.roughness;
    let albedo = finish.albedo;
    let n_dot_l = max(dot(n, l), 0.0);
    let n_dot_v = max(dot(n, v), 0.001);
    let f0 = mix(vec3<f32>(0.04), albedo, metallic);
    let f = fresnel(f0, dot(h, v));
    let spec = f * ggx(max(dot(n, h), 0.0), roughness) * 0.25 / max(n_dot_v * max(n_dot_l, 0.001), 0.05);
    let diffuse = (vec3<f32>(1.0) - f) * (1.0 - metallic) * albedo / PI;
    let sun = sky.sun.rgb * sun_visibility(world, geometric, in.clip.xy);
    // The sky dome and the sunlit ground fill the shade; their balance against the beam
    // sets how dark a shaded side reads. Ground light comes mostly from sunlit ground;
    // under the airframe that ground lies in its own shadow, so wells and bellies see
    // less of it.
    let ground_lit = ground_bounce(world, in.clip.xy);
    let fill = sky_irradiance(n) + ground_irradiance(n, ground_lit);
    let reflected = sky_reflection(reflect(-v, n), l, roughness, ground_lit);
    // Fresnel of a rough surface's reflection of its surroundings (Fdez-Aguera's fit).
    let env_f = f0 + (max(vec3<f32>(1.0 - roughness), f0) - f0) * pow(1.0 - n_dot_v, 5.0);
    // Baked occlusion shuts out sky and ground light, less in a pale recess than a dark
    // one; reflections go with it except at a glancing view of a smooth finish.
    let ao = finish.occlusion;
    let diffuse_ao = multi_bounce(ao, albedo * (1.0 - metallic));
    let specular_ao = specular_occlusion(n_dot_v, ao, roughness);
    let ambient = fill * (vec3<f32>(1.0) - env_f) * (1.0 - metallic) * albedo / PI * diffuse_ao + reflected * env_f * specular_ao;
    return (diffuse + spec * min(n_dot_l * 4.0, 1.0)) * sun * n_dot_l + ambient + printed.emissive;
}

// Share of the ground light reaching `world` from the ground straight below
// it: 1 in flight, or where that ground is sunlit; down to the sky-lit
// share where ownship's shadow covers it.

fn ground_bounce(world: vec3<f32>, pixel: vec2<f32>) -> f32 {
    if (shadow.ground_centre.w <= 0.0) {
        return 1.0;
    }
    let below = vec3<f32>(world.xy, shadow.ground_centre.z);
    let lit = sun_visibility(below, vec3<f32>(0.0), pixel);
    return mix(sky.ground.w, 1.0, lit);
}

@fragment
fn fs_opaque(in: VertexOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    let color = shade(in, front, surface(in));
    return vec4<f32>(display(color, EXPOSURE * sky.sun.w), 1.0);
}

// Share of the light crossing a pane that its material's opacity absorbs: clean acrylic
// passes about 92 %, and the opacity an airframe gives its glazing (0.3 for a canopy)
// scales the tint of thicker or laminated panes (assumed).
const GLASS_ABSORPTION: f32 = 0.4;
// Share of the absorbed light dust, crazing and scratches scatter back (assumed).
const GLASS_HAZE: f32 = 0.5;

// A pane of glazing over whatever is drawn behind it: what it mirrors and scatters is
// added, what it reflects away or absorbs is taken from the background.
fn glass(in: VertexOut, front: bool) -> vec4<f32> {
    var n = normalize(in.normal);
    if (!front) {
        n = -n;
    }
    let v = normalize(-in.world);
    let l = normalize(draw.sun.xyz);
    let n_dot_v = clamp(abs(dot(n, v)), 0.0, 1.0);
    // One air-glass surface (n = 1.5), then the pane's two with the bounces between.
    let surface = 0.04 + 0.96 * pow(1.0 - n_dot_v, 5.0);
    let pane = 2.0 * surface / (1.0 + surface);
    let absorbed = draw.base_color.a * GLASS_ABSORPTION;
    var radiance = draw.base_color.rgb * absorbed * GLASS_HAZE * (sky_irradiance(n) + ground_irradiance(n, 1.0)) / PI;
    var reflected = 0.0;
    // Seen from outside the pane mirrors the sky and glints in the Sun; seen from inside,
    // through the far side of a canopy, it mirrors the dark cockpit.
    if (front) {
        reflected = pane;
        radiance += sky_reflection(reflect(-v, n), l, draw.params.x, 1.0) * pane;
        let h = normalize(l + v);
        let n_dot_l = max(dot(n, l), 0.0);
        let glint = ggx(max(dot(n, h), 0.0), max(draw.params.x, 0.05)) * 0.25 / max(n_dot_v * max(n_dot_l, 0.001), 0.05);
        let f_h = 0.04 + 0.96 * pow(1.0 - clamp(dot(h, v), 0.0, 1.0), 5.0);
        radiance += sky.sun.rgb * sun_visibility(in.world, n, in.clip.xy) * glint * f_h * n_dot_l;
    }
    let alpha = 1.0 - (1.0 - reflected) * (1.0 - absorbed);
    return vec4<f32>(display(radiance, EXPOSURE * sky.sun.w), alpha);
}

// Peak of the opacity an articulation gives a propeller's blur (`disc_opacity` of
// `sim_core::PropellerVisualState`): its share of that peak is how far the solid blades
// have given way to their blur.
const DISC_PEAK_OPACITY: f32 = 0.34;
// Widest chord of a blade as a share of its radius, and the radius shares its root and
// yellow tip reach (assumed: a paddle blade of the period, 10 in wide on a 5 ft 9 in
// radius, tipped 4 in).
const BLADE_CHORD: f32 = 0.145;
const BLADE_ROOT: f32 = 0.14;
const BLADE_TIP_PAINT: f32 = 0.94;

// Chord of a blade at radius share `x`, as a share of the radius: a shank widening to the
// broadest section near mid-span, then narrowing to a rounded tip.
fn blade_chord(x: f32) -> f32 {
    let shank = smoothstep(BLADE_ROOT, 0.45, x);
    let tip = sqrt(max(1.0 - pow(max(x - 0.55, 0.0) / 0.45, 2.0), 0.0));
    return BLADE_CHORD * (0.45 + 0.55 * shank) * tip;
}

// A turning propeller as a camera records it: each point of the disc is covered by a
// blade for the share of the exposure the blade takes to cross it. A short exposure or a
// slow propeller leaves separate ghost blades; a long one an even disc, denser toward
// the hub where the blades are wide for their circle, thinning to nothing at the tips.
fn propeller_disc(in: VertexOut, front: bool) -> vec4<f32> {
    // The disc's texture coordinates span the unit disc about the hub.
    let uv = in.uv;
    let r = length(uv);
    if (r > 1.0 || r < BLADE_ROOT) {
        discard;
    }
    let blades = select(2.0, 3.0, draw.params.w > 2.5);
    // Angle a blade's chord spans at this radius, and the angle it sweeps while the
    // shutter is open.
    let width = blade_chord(r) / r;
    let sweep = max(draw.params.z, 0.02);
    let theta = atan2(uv.y, uv.x) - draw.params.y;
    let spacing = 2.0 * PI / blades;
    var covered = 0.0;
    for (var i = -1; i <= 1; i++) {
        // Nearest blade and its neighbours: a sweep wider than the spacing overlaps them.
        let d = abs(theta - spacing * (round(theta / spacing) + f32(i)));
        covered += clamp(0.5 * (width + sweep) - d, 0.0, min(width, sweep));
    }
    let presence = clamp(draw.base_color.a / DISC_PEAK_OPACITY, 0.0, 1.0);
    let alpha = clamp(covered / sweep, 0.0, 1.0) * presence;
    // The blur is lit as the blades' flat faces are: by the Sun, sky and ground on the
    // side of the disc that faces the viewer.
    var n = normalize(in.normal);
    if (!front) {
        n = -n;
    }
    let tip_paint = select(draw.base_color.rgb, draw.emissive.rgb, length(draw.emissive.rgb) > 0.05);
    let paint = mix(draw.base_color.rgb, tip_paint, smoothstep(BLADE_TIP_PAINT - 0.005, BLADE_TIP_PAINT + 0.005, r));
    let light = sky.sun.rgb * max(dot(n, normalize(draw.sun.xyz)), 0.0) + sky_irradiance(n) + ground_irradiance(n, 1.0);
    let rgb = display(paint * light / PI, EXPOSURE * sky.sun.w);
    return vec4<f32>(rgb * alpha, alpha);
}

@fragment
fn fs_translucent(in: VertexOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    if (draw.params.w > 1.5) {
        return propeller_disc(in, front);
    }
    return glass(in, front);
}
