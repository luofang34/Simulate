struct QuadParams {
    // Normalized top-left viewport rectangle: (x, y, width, height) in [0, 1].
    rect: vec4<f32>,
    // Quad physical size in pixels: (width_px, height_px).
    pixel_size: vec2<f32>,
    // Bit 0 (1u): decode sRGB-encoded Unorm RGB to linear RGB.
    // Bit 1 (2u): draw subtle glass-cockpit bezel border around the quad.
    // Bit 2 (4u): force source alpha to 1.0 (opaque viewport underlay).
    flags: u32,
    opacity: f32,
};

@group(0) @binding(0) var<uniform> params: QuadParams;
@group(0) @binding(1) var quad_tex: texture_2d<f32>;
@group(0) @binding(2) var quad_sampler: sampler;

struct VertexOut {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

fn quad_corner(vid: u32) -> vec2<f32> {
    switch vid {
        case 0u: { return vec2<f32>(0.0, 0.0); }
        case 1u: { return vec2<f32>(1.0, 0.0); }
        case 2u: { return vec2<f32>(0.0, 1.0); }
        case 3u: { return vec2<f32>(0.0, 1.0); }
        case 4u: { return vec2<f32>(1.0, 0.0); }
        default: { return vec2<f32>(1.0, 1.0); }
    }
}

fn srgb_channel_to_linear(c: f32) -> f32 {
    let clamped = clamp(c, 0.0, 1.0);
    if clamped <= 0.04045 {
        return clamped / 12.92;
    }
    return pow((clamped + 0.055) / 1.055, 2.4);
}

fn srgb_to_linear(rgb: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(
        srgb_channel_to_linear(rgb.r),
        srgb_channel_to_linear(rgb.g),
        srgb_channel_to_linear(rgb.b),
    );
}

@vertex
fn vs_quad(@builtin(vertex_index) vid: u32) -> VertexOut {
    let corner = quad_corner(vid);
    let screen = params.rect.xy + corner * params.rect.zw;
    let ndc = vec2<f32>(screen.x * 2.0 - 1.0, 1.0 - screen.y * 2.0);
    var out: VertexOut;
    out.clip_position = vec4<f32>(ndc, 0.0, 1.0);
    out.uv = corner;
    return out;
}

@fragment
fn fs_quad(in: VertexOut) -> @location(0) vec4<f32> {
    let sample = textureSample(quad_tex, quad_sampler, in.uv);
    var rgb = sample.rgb;
    if (params.flags & 1u) != 0u {
        rgb = srgb_to_linear(rgb);
    }
    var alpha = sample.a;
    if (params.flags & 4u) != 0u {
        alpha = 1.0;
    }

    if (params.flags & 2u) != 0u {
        let px = in.uv * params.pixel_size;
        let edge_dist = min(
            min(px.x, params.pixel_size.x - px.x),
            min(px.y, params.pixel_size.y - px.y),
        );
        if edge_dist < 1.25 {
            rgb = vec3<f32>(0.02, 0.03, 0.05);
            alpha = 0.96;
        } else if edge_dist < 2.25 {
            rgb = vec3<f32>(0.10, 0.16, 0.24);
            alpha = max(alpha, 0.92);
        }
    }

    return vec4<f32>(rgb, clamp(alpha * params.opacity, 0.0, 1.0));
}
