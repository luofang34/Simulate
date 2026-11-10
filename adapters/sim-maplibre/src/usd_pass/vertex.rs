//! GPU vertex layout of aircraft parts, shared by the pass over terrain and the hangar
//! studio so both shaders read the same attributes.

use crate::UsdMeshVertex;

/// Bytes per vertex: position, normal, texture coordinate, baked occlusion,
/// tangent with handedness.
pub(crate) const VERTEX_STRIDE: u64 = 52;

/// Attribute locations `0..=4` the shaders' `VertexIn` declares.
pub(crate) const VERTEX_ATTRIBUTES: [wgpu::VertexAttribute; 5] = wgpu::vertex_attr_array![
    0 => Float32x3,
    1 => Float32x3,
    2 => Float32x2,
    3 => Float32,
    4 => Float32x4,
];

/// The interleaved vertex buffer layout of one part.
pub(crate) fn vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    wgpu::VertexBufferLayout {
        array_stride: VERTEX_STRIDE,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &VERTEX_ATTRIBUTES,
    }
}

/// Little-endian interleaved vertex bytes.
pub(crate) fn vertex_bytes(vertices: &[UsdMeshVertex]) -> Vec<u8> {
    vertices
        .iter()
        .flat_map(|v| {
            v.position
                .into_iter()
                .chain(v.normal)
                .chain(v.uv)
                .chain([v.occlusion])
                .chain(v.tangent)
        })
        .flat_map(f32::to_le_bytes)
        .collect()
}

/// Index bytes of the whole triangles in `indices`, and their index count (`None` if none).
pub(crate) fn index_bytes(indices: &[u32]) -> Option<(Vec<u8>, u32)> {
    let whole = indices.len() - indices.len() % 3;
    let count = u32::try_from(whole).ok().filter(|count| *count > 0)?;
    let bytes = indices
        .iter()
        .take(whole)
        .flat_map(|index| index.to_le_bytes())
        .collect();
    Some((bytes, count))
}
