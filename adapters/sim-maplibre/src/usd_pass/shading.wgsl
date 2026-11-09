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

// Procedural paint finish (see `SurfaceDetail`): seams, rivet rows, fading, soot and
// cavity grime, from the part-local position so they stay fixed on the skin.
struct Paint {
    albedo: vec3<f32>,
    roughness: f32,
    // Extra occlusion of seams, multiplied into the baked vertex occlusion.
    cavity: f32,
};

fn hash31(p: vec3<f32>) -> f32 {
    let q = fract(p * vec3<f32>(0.1031, 0.1030, 0.0973));
    let r = q + vec3<f32>(dot(q, q.yzx + vec3<f32>(33.33)));
    return fract((r.x + r.y) * r.z);
}

fn value_noise(p: vec3<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (vec3<f32>(3.0) - 2.0 * f);
    let x00 = mix(hash31(i), hash31(i + vec3<f32>(1.0, 0.0, 0.0)), u.x);
    let x10 = mix(hash31(i + vec3<f32>(0.0, 1.0, 0.0)), hash31(i + vec3<f32>(1.0, 1.0, 0.0)), u.x);
    let x01 = mix(hash31(i + vec3<f32>(0.0, 0.0, 1.0)), hash31(i + vec3<f32>(1.0, 0.0, 1.0)), u.x);
    let x11 = mix(hash31(i + vec3<f32>(0.0, 1.0, 1.0)), hash31(i + vec3<f32>(1.0, 1.0, 1.0)), u.x);
    return mix(mix(x00, x10, u.y), mix(x01, x11, u.y), u.z);
}

// Distance in metres from `coord` to the nearest multiple of `pitch`.
fn seam_distance(coord: f32, pitch: f32) -> f32 {
    return abs(fract(coord / max(pitch, 1e-3) + 0.5) - 0.5) * pitch;
}

// Coverage of a `width_m` line at that distance. A seam narrower than the pixel footprint
// is widened and dimmed in proportion, so distant skins keep their average tone without
// aliasing into moiré.
fn line_coverage(distance: f32, width_m: f32, footprint: f32) -> f32 {
    let w = max(width_m, footprint);
    return (1.0 - smoothstep(0.5 * w, 0.5 * w + footprint, distance)) * (width_m / w);
}

fn paint(albedo_in: vec3<f32>, roughness_in: f32, local: vec3<f32>, local_normal: vec3<f32>, occlusion: f32, detail: vec4<f32>, soot: vec4<f32>) -> Paint {
    let n = normalize(local_normal);
    let footprint = max(length(fwidth(local)), 1e-5);
    var seams = 0.0;
    var rivets = 0.0;
    // A seam across an axis shows where the skin runs along that axis, not where it faces it.
    let tangent = vec3<f32>(1.0) - smoothstep(vec3<f32>(0.55), vec3<f32>(0.85), abs(n));
    let along = vec3<f32>(local.y + local.z, local.x + local.z, local.x + local.y);
    let rivet_fade = 1.0 - smoothstep(0.004, 0.012, footprint);
    for (var a = 0; a < 3; a++) {
        let pitch = detail[a];
        if (pitch > 0.0) {
            let d = seam_distance(local[a], pitch);
            seams += line_coverage(d, 0.0035, footprint) * tangent[a];
            // Rows of flush rivets either side of the seam, 4 cm apart.
            let row = abs(d - 0.016);
            let dots = length(vec2<f32>(row, seam_distance(along[a], 0.04)));
            rivets += (1.0 - smoothstep(0.0025, 0.0025 + footprint, dots)) * tangent[a] * rivet_fade;
        }
    }
    seams = clamp(seams, 0.0, 1.0);
    let weathering = detail.w;
    let up = clamp(n.z, 0.0, 1.0);
    let blotch = value_noise(local * 0.9) * 0.6 + value_noise(local * 3.7) * 0.4;
    let cell = floor(local / max(detail.xyz, vec3<f32>(1e-3))) * step(vec3<f32>(1e-4), detail.xyz);
    let panel = hash31(cell + vec3<f32>(17.0)) - 0.5;
    // Olive drab under sun turns lighter, greyer and browner, then chalks; neighbouring
    // panels, replaced or repainted at different times, differ slightly.
    let fade = weathering * (0.2 + 0.8 * up) * (0.45 + 0.55 * blotch);
    let grey = vec3<f32>(dot(albedo_in, vec3<f32>(0.2126, 0.7152, 0.0722)));
    let faded = mix(albedo_in, grey * vec3<f32>(1.06, 1.0, 0.86), 0.4) * 1.4 + vec3<f32>(0.01);
    var albedo = mix(albedo_in, faded, clamp(fade * 0.8, 0.0, 1.0)) * (1.0 + panel * 0.1 * weathering);
    var roughness = roughness_in + weathering * (fade * 0.2 + (blotch - 0.5) * 0.1 + panel * 0.06);
    // Dirt settles in the seams and in occluded corners.
    let grime = weathering * (1.0 - clamp(occlusion, 0.0, 1.0)) * 0.35;
    albedo *= (1.0 - 0.35 * seams) * (1.0 - grime) * (1.0 - 0.12 * clamp(rivets, 0.0, 1.0));
    // Exhaust soot runs aft from the source along the slipstream, spreading and thinning.
    let aft = soot.y - local.y;
    let spread = 0.14 + 0.05 * max(aft, 0.0);
    let lateral = abs(abs(local.x) - soot.x);
    let streaks = 0.5 + 0.5 * value_noise(vec3<f32>(local.x * 16.0, local.y * 0.7, 0.0));
    let deposit = soot.w * smoothstep(0.0, 0.15, aft) * exp(-1.1 * max(aft, 0.0) / max(soot.z, 0.1))
        * (1.0 - smoothstep(0.35 * spread, spread, lateral)) * streaks * smoothstep(-0.2, 0.4, n.z);
    albedo = mix(albedo, vec3<f32>(0.03, 0.028, 0.025), clamp(deposit, 0.0, 1.0));
    roughness = mix(roughness, 0.9, clamp(deposit, 0.0, 1.0));
    var out: Paint;
    out.albedo = albedo;
    out.roughness = clamp(roughness, 0.04, 1.0);
    out.cavity = 1.0 - 0.5 * seams;
    return out;
}

// A part's finished surface: what the BRDF lights.
struct Finish {
    albedo: vec3<f32>,
    roughness: f32,
    metallic: f32,
    // Baked vertex occlusion times the skin's or the seams' own.
    occlusion: f32,
    // Shading normal, the geometric one bent by the skin's normal map.
    normal: vec3<f32>,
};

// Parts of kind 5 take everything from the asset's baked skin (the pass binds `skin_albedo`,
// `skin_normal`, `skin_orm` and `skin_sampler`); others keep their material under the
// procedural paint of `SurfaceDetail`. `n` faces the viewer, `tangent` is in its frame with
// the handedness in `w`. The branch is on a uniform, so sampling and derivatives stay legal.
fn surface_finish(albedo_in: vec3<f32>, n: vec3<f32>, local: vec3<f32>, local_normal: vec3<f32>, occlusion: f32, uv: vec2<f32>, tangent: vec4<f32>) -> Finish {
    var out: Finish;
    if (draw.params.w > 4.5) {
        let orm = textureSample(skin_orm, skin_sampler, uv);
        let bump = textureSample(skin_normal, skin_sampler, uv).rg * 2.0 - vec2<f32>(1.0);
        let t = normalize(tangent.xyz - n * dot(n, tangent.xyz) + vec3<f32>(1e-6, 0.0, 0.0));
        let b = cross(n, t) * select(-1.0, 1.0, tangent.w >= 0.0);
        let z = sqrt(max(1.0 - dot(bump, bump), 0.0));
        out.albedo = textureSample(skin_albedo, skin_sampler, uv).rgb;
        out.occlusion = clamp(occlusion, 0.0, 1.0) * orm.r;
        out.roughness = clamp(orm.g, 0.04, 1.0);
        out.metallic = orm.b;
        out.normal = normalize(t * bump.x + b * bump.y + n * z);
    } else {
        let paint = paint(albedo_in, draw.params.x, local, local_normal, occlusion, draw.detail, draw.soot);
        out.albedo = paint.albedo;
        out.occlusion = clamp(occlusion, 0.0, 1.0) * paint.cavity;
        out.roughness = paint.roughness;
        out.metallic = draw.emissive.w;
        out.normal = n;
    }
    return out;
}
