// Full-frame passes of a sensor aperture. Positions here are continuous image
// pixels, top-left origin, derived from the clip position so that they do not
// rest on how a backend reports @builtin(position).

const STAR_SLOTS: u32 = 37u;
const RETICLE_SXT: f32 = 1.0;
const RETICLE_AOT: f32 = 2.0;
const RETICLE_LPD: f32 = 3.0;
const TAU: f32 = 6.2831855;
const LINE_HALF_WIDTH_PX: f32 = 0.75;
const RETICLE_RGB: vec3<f32> = vec3<f32>(1.0, 0.72, 0.12);
// Spiral spacing and detent-mark placement, as fractions of image height.
const AOT_SPIRAL_PITCH: f32 = 0.06;
const AOT_MARK_INNER: f32 = 0.40;
const AOT_MARK_OUTER: f32 = 0.46;
const AOT_MARK_HALF_WIDTH_RAD: f32 = 0.012;
const SXT_LINE_GAP_PX: f32 = 4.0;
const LPD_MARK_HALF_LENGTH_PX: f32 = 10.0;

struct Overlay {
    // Width, height.
    viewport: vec4<f32>,
    // fx, fy, and the principal point in continuous pixels.
    optics: vec4<f32>,
    // Kind, rotation (rad), mark spacing (rad).
    reticle: vec4<f32>,
    // Semi-axes (px), vignette width (px), enabled.
    bezel: vec4<f32>,
    bezel_colour: vec4<f32>,
    // Hub x, y, tip radius (px), enabled.
    hub: vec4<f32>,
    // Blade count, half width (rad), phase (rad), hub radius (px).
    blades: vec4<f32>,
    blade_colour: vec4<f32>,
    // Continuous pixel x, y, peak linear intensity, spread (px).
    stars: array<vec4<f32>, STAR_SLOTS>,
};

@group(0) @binding(0) var<uniform> overlay: Overlay;

struct FullFrameOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) pixel: vec2<f32>,
};

// One oversized triangle, at depth zero: the infinitely far plane, so a
// depth test against the terrain leaves only open sky.
@vertex
fn vs_full_frame(@builtin(vertex_index) index: u32) -> FullFrameOut {
    let ndc = vec2<f32>(f32(index & 1u) * 4.0 - 1.0, f32(index >> 1u) * 4.0 - 1.0);
    var out: FullFrameOut;
    out.clip = vec4<f32>(ndc, 0.0, 1.0);
    out.pixel = vec2<f32>(ndc.x + 1.0, 1.0 - ndc.y) * 0.5 * overlay.viewport.xy;
    return out;
}

@fragment
fn fs_stars(in: FullFrameOut) -> @location(0) vec4<f32> {
    var light = 0.0;
    for (var i = 0u; i < STAR_SLOTS; i += 1u) {
        let star = overlay.stars[i];
        if star.z > 0.0 {
            let offset = in.pixel - star.xy;
            light += star.z * exp(-dot(offset, offset) / (2.0 * star.w * star.w));
        }
    }
    return vec4<f32>(vec3<f32>(light), 1.0);
}

fn line(distance_px: f32) -> bool {
    return abs(distance_px) <= LINE_HALF_WIDTH_PX;
}

fn sxt_reticle(offset: vec2<f32>) -> bool {
    return line(abs(offset.x) - SXT_LINE_GAP_PX) || line(abs(offset.y) - SXT_LINE_GAP_PX);
}

fn aot_reticle(offset: vec2<f32>, rotation: f32) -> bool {
    let sine = sin(rotation);
    let cosine = cos(rotation);
    let turned = vec2<f32>(cosine * offset.x + sine * offset.y,
                           cosine * offset.y - sine * offset.x);
    if line(turned.x) || line(turned.y) {
        return true;
    }
    let radius = length(turned);
    let pitch = AOT_SPIRAL_PITCH * overlay.viewport.y;
    let angle = atan2(turned.y, turned.x);
    let turns = radius / pitch - angle / TAU;
    if line((abs(fract(turns + 0.5) - 0.5)) * pitch) {
        return true;
    }
    let sector = TAU / 6.0;
    let from_mark = abs(fract(angle / sector + 0.5) - 0.5) * sector;
    let height = overlay.viewport.y;
    return from_mark <= AOT_MARK_HALF_WIDTH_RAD
        && radius >= AOT_MARK_INNER * height && radius <= AOT_MARK_OUTER * height;
}

fn lpd_reticle(offset: vec2<f32>, spacing: f32) -> bool {
    if offset.y < 0.0 {
        return false;
    }
    if line(offset.x) {
        return true;
    }
    let angle = atan(offset.y / overlay.optics.y);
    let mark = tan(round(angle / spacing) * spacing) * overlay.optics.y;
    return line(offset.y - mark) && abs(offset.x) <= LPD_MARK_HALF_LENGTH_PX;
}

fn on_reticle(pixel: vec2<f32>) -> bool {
    let kind = overlay.reticle.x;
    let offset = pixel - overlay.optics.zw;
    if kind == RETICLE_SXT {
        return sxt_reticle(offset);
    }
    if kind == RETICLE_AOT {
        return aot_reticle(offset, overlay.reticle.y);
    }
    if kind == RETICLE_LPD {
        return lpd_reticle(offset, overlay.reticle.z);
    }
    return false;
}

// Coverage of the window frame: one at and beyond its edge, fading out over
// the vignette inside it.
fn bezel_alpha(pixel: vec2<f32>) -> f32 {
    if overlay.bezel.w == 0.0 {
        return 0.0;
    }
    let semi = overlay.bezel.xy;
    let scaled = length((pixel - overlay.viewport.xy * 0.5) / semi);
    if scaled >= 1.0 {
        return 1.0;
    }
    let inside_px = (1.0 - scaled) * min(semi.x, semi.y);
    let vignette = max(overlay.bezel.z, 1e-3);
    return 1.0 - clamp(inside_px / vignette, 0.0, 1.0);
}

fn on_propeller(pixel: vec2<f32>) -> bool {
    if overlay.hub.w == 0.0 {
        return false;
    }
    let offset = pixel - overlay.hub.xy;
    let radius = length(offset);
    let hub_radius = overlay.blades.w;
    if radius < hub_radius {
        return true;
    }
    let blade_count = overlay.blades.x;
    if radius >= overlay.hub.z || blade_count == 0.0 {
        return false;
    }
    let sector = TAU / blade_count;
    let turned = atan2(offset.y, offset.x) - overlay.blades.z;
    let within = turned - floor(turned / sector) * sector;
    return min(within, sector - within) <= overlay.blades.y;
}

// Nearest the lens wins: reticle, then window frame, then propeller.
@fragment
fn fs_overlay(in: FullFrameOut) -> @location(0) vec4<f32> {
    if on_reticle(in.pixel) {
        return vec4<f32>(RETICLE_RGB, 1.0);
    }
    let frame = bezel_alpha(in.pixel);
    if frame >= 1.0 {
        return vec4<f32>(overlay.bezel_colour.rgb, 1.0);
    }
    if on_propeller(in.pixel) {
        let blade = overlay.blade_colour.rgb;
        return vec4<f32>(mix(blade, overlay.bezel_colour.rgb, frame), 1.0);
    }
    return vec4<f32>(overlay.bezel_colour.rgb, frame);
}
