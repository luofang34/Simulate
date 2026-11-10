//! An airframe's textures on the GPU, one bind group per asset: the albedo,
//! normal and occlusion/roughness/metallic maps of its baked skin. An asset
//! without a skin binds one-texel blanks, so every part draws through the
//! same layout; only parts of the skin draw kind sample them.

use maplibre::render::upload_queue::UploadQueue;
use wgpu::util::DeviceExt;

use crate::usd_aircraft::{SkinFormat, SkinMap, SkinTextures};

/// Anisotropic taps for skin seen along a wing or boom, where texels are
/// foreshortened many times over.
const SKIN_ANISOTROPY: u16 = 16;

/// The bind-group layout, its sampler, and the blank maps of an unskinned asset.
pub(crate) struct AssetTexturesGpu {
    pub(crate) layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    pub(crate) blank: wgpu::BindGroup,
}

impl AssetTexturesGpu {
    pub(crate) fn new(device: &wgpu::Device) -> Self {
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("usd asset textures"),
            entries: &[
                texture_entry(0),
                texture_entry(1),
                texture_entry(2),
                sampler_entry(3),
            ],
        });
        let sampler = sampler(device, "usd skin", SKIN_ANISOTROPY);
        // Never sampled by a part that draws from them, so wgpu's zeroed
        // contents serve.
        let blank = |format| device.create_texture(&descriptor(format, 1, 1, 1));
        let blank = bind(
            device,
            &layout,
            &sampler,
            [
                &blank(SkinFormat::Srgba8),
                &blank(SkinFormat::Normal8),
                &blank(SkinFormat::Rgba8),
            ],
        );
        Self {
            layout,
            sampler,
            blank,
        }
    }

    /// Uploads `skin`, each map with its mip chain, and binds it; a map wider
    /// than the device allows starts at the first mip that fits.
    pub(crate) fn upload(
        &self,
        device: &wgpu::Device,
        queue: &impl TextureUpload,
        skin: &SkinTextures,
    ) -> wgpu::BindGroup {
        let limit = device.limits().max_texture_dimension_2d;
        let [albedo, normal, orm] =
            [&skin.albedo, &skin.normal, &skin.orm].map(|map| upload(device, queue, map, limit));
        bind(
            device,
            &self.layout,
            &self.sampler,
            [&albedo, &normal, &orm],
        )
    }
}

/// The bind group of the three skin maps, in binding order.
fn bind(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    sampler: &wgpu::Sampler,
    textures: [&wgpu::Texture; 3],
) -> wgpu::BindGroup {
    let [albedo, normal, orm] =
        textures.map(|t| t.create_view(&wgpu::TextureViewDescriptor::default()));
    let texture = |binding, view| wgpu::BindGroupEntry {
        binding,
        resource: wgpu::BindingResource::TextureView(view),
    };
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("usd asset textures"),
        layout,
        entries: &[
            texture(0, &albedo),
            texture(1, &normal),
            texture(2, &orm),
            wgpu::BindGroupEntry {
                binding: 3,
                resource: wgpu::BindingResource::Sampler(sampler),
            },
        ],
    })
}

fn texture_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Texture {
            sample_type: wgpu::TextureSampleType::Float { filterable: true },
            view_dimension: wgpu::TextureViewDimension::D2,
            multisampled: false,
        },
        count: None,
    }
}

fn sampler_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
        count: None,
    }
}

fn sampler(device: &wgpu::Device, label: &str, anisotropy: u16) -> wgpu::Sampler {
    device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some(label),
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        anisotropy_clamp: anisotropy,
        ..Default::default()
    })
}

fn upload(
    device: &wgpu::Device,
    queue: &impl TextureUpload,
    map: &SkinMap,
    limit: u32,
) -> wgpu::Texture {
    let (width, height) = map.size();
    upload_levels(
        device,
        queue,
        map.format(),
        width,
        height,
        map.levels(),
        limit,
    )
}

/// Uploads `levels` from the first one no wider or taller than `limit`.
fn upload_levels(
    device: &wgpu::Device,
    queue: &impl TextureUpload,
    format: SkinFormat,
    width: u32,
    height: u32,
    levels: &[Vec<u8>],
    limit: u32,
) -> wgpu::Texture {
    let skip = first_fitting_level(width, height, limit);
    let kept = levels.get(skip as usize..).unwrap_or(&[]);
    let bytes: Vec<u8> = kept.iter().flatten().copied().collect();
    let mips = u32::try_from(kept.len()).unwrap_or(1).max(1);
    let desc = descriptor(
        format,
        (width >> skip).max(1),
        (height >> skip).max(1),
        mips,
    );
    queue.texture_with_data(device, &desc, &bytes)
}

/// The finest mip level of a `width` x `height` map no side of which
/// exceeds `limit`.
pub(crate) fn first_fitting_level(width: u32, height: u32, limit: u32) -> u32 {
    let mut skip = 0_u32;
    while skip < 31 && (width >> skip).max(height >> skip) > limit.max(1) {
        skip = skip.wrapping_add(1);
    }
    skip
}

fn descriptor(
    format: SkinFormat,
    width: u32,
    height: u32,
    mips: u32,
) -> wgpu::TextureDescriptor<'static> {
    wgpu::TextureDescriptor {
        label: Some("usd asset texture"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: mips,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: match format {
            SkinFormat::Srgba8 => wgpu::TextureFormat::Rgba8UnormSrgb,
            SkinFormat::Normal8 => wgpu::TextureFormat::Rg8Unorm,
            SkinFormat::Rgba8 => wgpu::TextureFormat::Rgba8Unorm,
        },
        usage: wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    }
}

/// A queue a texture and its mip chain can be created through: the map
/// renderer's counting queue, or a plain one.
pub(crate) trait TextureUpload {
    /// A texture of `desc` holding `bytes`, levels finest first.
    fn texture_with_data(
        &self,
        device: &wgpu::Device,
        desc: &wgpu::TextureDescriptor<'_>,
        bytes: &[u8],
    ) -> wgpu::Texture;
}

impl TextureUpload for UploadQueue {
    fn texture_with_data(
        &self,
        device: &wgpu::Device,
        desc: &wgpu::TextureDescriptor<'_>,
        bytes: &[u8],
    ) -> wgpu::Texture {
        self.create_texture_with_data(
            device,
            desc,
            wgpu::util::TextureDataOrder::LayerMajor,
            bytes,
        )
    }
}

impl TextureUpload for wgpu::Queue {
    fn texture_with_data(
        &self,
        device: &wgpu::Device,
        desc: &wgpu::TextureDescriptor<'_>,
        bytes: &[u8],
    ) -> wgpu::Texture {
        device.create_texture_with_data(self, desc, wgpu::util::TextureDataOrder::LayerMajor, bytes)
    }
}
