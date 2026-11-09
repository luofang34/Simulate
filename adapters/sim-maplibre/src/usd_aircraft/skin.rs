//! Baked physically based skins: an airframe's albedo, tangent-space normal
//! and occlusion/roughness/metallic maps, sampled through the texture
//! coordinates its textured parts carry.

mod map;
#[cfg(test)]
mod tests;

pub use map::{SkinFormat, SkinMap};

/// The three maps of an airframe's skin, sharing one texture-coordinate
/// layout.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SkinTextures {
    /// Base colour ([`SkinFormat::Srgba8`]); alpha is unused.
    pub albedo: SkinMap,
    /// Tangent-space normal ([`SkinFormat::Normal8`]).
    pub normal: SkinMap,
    /// Occlusion, roughness and metallic in `r`, `g`, `b`
    /// ([`SkinFormat::Rgba8`]).
    pub orm: SkinMap,
}

impl SkinTextures {
    /// Bytes of all three maps and their mips: the skin's GPU memory.
    #[must_use]
    pub fn byte_len(&self) -> usize {
        self.albedo.byte_len() + self.normal.byte_len() + self.orm.byte_len()
    }
}
