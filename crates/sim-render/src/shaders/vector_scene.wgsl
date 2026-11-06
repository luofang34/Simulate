struct Viewport {
    // Target size in physical pixels: (width_px, height_px, 0, 0).
    size: vec4<f32>,
};

@group(0) @binding(0) var<uniform> viewport: Viewport;

struct VertexIn {
    // Physical target pixels, top-left origin.
    @location(0) pos_px: vec2<f32>,
    // Linear colour premultiplied by alpha.
    @location(1) color_premul: vec4<f32>,
    // Clip rectangle in physical pixels: (min_x, min_y, max_x, max_y).
    @location(2) clip_rect_px: vec4<f32>,
    // Paint-order depth; later paint operations carry larger values.
    @location(3) depth: f32,
};

struct VertexOut {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color_premul: vec4<f32>,
    // Every vertex of a triangle shares one clip rectangle.
    @location(1) @interpolate(flat) clip_rect_px: vec4<f32>,
};

@vertex
fn vs_vector(in: VertexIn) -> VertexOut {
    let size = max(viewport.size.xy, vec2<f32>(1.0, 1.0));
    let ndc = vec2<f32>(in.pos_px.x / size.x * 2.0 - 1.0, 1.0 - in.pos_px.y / size.y * 2.0);
    var out: VertexOut;
    out.clip_position = vec4<f32>(ndc, clamp(in.depth, 0.0, 1.0), 1.0);
    out.color_premul = in.color_premul;
    out.clip_rect_px = in.clip_rect_px;
    return out;
}

// The fragment position is the pixel centre, so a clip edge is a whole-pixel
// step; clip rectangles are pixel-aligned by construction.
@fragment
fn fs_vector(in: VertexOut) -> @location(0) vec4<f32> {
    let p = in.clip_position.xy;
    let clip = in.clip_rect_px;
    if p.x < clip.x || p.y < clip.y || p.x >= clip.z || p.y >= clip.w {
        discard;
    }
    return in.color_premul;
}
