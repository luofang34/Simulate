//! An airframe's textures on the GPU, one bind group per asset: the albedo,
//! normal and occlusion/roughness/metallic maps of its baked skin, then the
//! display atlas its printed faces sample. An asset without a skin or an
//! atlas binds one-texel blanks in their place, so every part draws through
//! the same layout; only parts of the skin and display draw kinds sample them.

use maplibre::render::upload_queue::UploadQueue;
use wgpu::util::DeviceExt;

use crate::usd_aircraft::{DisplayAtlas, SkinFormat, SkinMap, SkinTextures};

/// Anisotropic taps for skin seen along a wing or boom, where texels are
/// foreshortened many times over.
const SKIN_ANISOTROPY: u16 = 16;
/// Anisotropic taps for a dial seen at a glancing angle.
const DISPLAY_ANISOTROPY: u16 = 8;

/// The bind-group layout, its samplers, and the blank textures bound for an
/// asset with neither skin nor atlas.
pub(crate) struct AssetTexturesGpu {
    pub(crate) layout: wgpu::BindGroupLayout,
    skin_sampler: wgpu::Sampler,
    display_sampler: wgpu::Sampler,
    blank_maps: [wgpu::Texture; 4],
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
                texture_entry(4),
                sampler_entry(5),
            ],
        });
        let skin_sampler = sampler(device, "usd skin", SKIN_ANISOTROPY);
        let display_sampler = sampler(device, "usd display atlas", DISPLAY_ANISOTROPY);
        // Never sampled by a part that draws from them, so wgpu's zeroed
        // contents serve.
        let blank = |format| device.create_texture(&descriptor(format, 1, 1, 1));
        let blank_maps = [
            blank(SkinFormat::Srgba8),
            blank(SkinFormat::Normal8),
            blank(SkinFormat::Rgba8),
            blank(SkinFormat::Srgba8),
        ];
        let [albedo, normal, orm, atlas] = &blank_maps;
        let blank = bind(
            device,
            &layout,
            [&skin_sampler, &display_sampler],
            [albedo, normal, orm, atlas],
        );
        Self {
            layout,
            skin_sampler,
            display_sampler,
            blank_maps,
            blank,
        }
    }

    /// Uploads `skin` and `atlas`, each map with its mip chain, and binds
    /// them, with a blank where the asset has none; a skin map wider than the
    /// device allows starts at the first mip that fits.
    pub(crate) fn upload(
        &self,
        device: &wgpu::Device,
        queue: &impl TextureUpload,
        skin: Option<&SkinTextures>,
        atlas: Option<&DisplayAtlas>,
    ) -> wgpu::BindGroup {
        let limit = device.limits().max_texture_dimension_2d;
        let skin = skin.map(|skin| {
            [&skin.albedo, &skin.normal, &skin.orm].map(|map| upload(device, queue, map, limit))
        });
        let atlas = atlas.map(|atlas| {
            let (width, height) = atlas.size();
            let levels = atlas.levels();
            upload_levels(
                device,
                queue,
                SkinFormat::Srgba8,
                width,
                height,
                levels,
                limit,
            )
        });
        let [blank_albedo, blank_normal, blank_orm, blank_atlas] = &self.blank_maps;
        let [albedo, normal, orm] = skin
            .as_ref()
            .map_or([blank_albedo, blank_normal, blank_orm], |[a, n, o]| {
                [a, n, o]
            });
        bind(
            device,
            &self.layout,
            [&self.skin_sampler, &self.display_sampler],
            [albedo, normal, orm, atlas.as_ref().unwrap_or(blank_atlas)],
        )
    }
}

/// The bind group of the three skin maps and the display atlas, in binding
/// order, with the skin's sampler and then the atlas's.
fn bind(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    [skin_sampler, display_sampler]: [&wgpu::Sampler; 2],
    textures: [&wgpu::Texture; 4],
) -> wgpu::BindGroup {
    let [albedo, normal, orm, atlas] =
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
                resource: wgpu::BindingResource::Sampler(skin_sampler),
            },
            texture(4, &atlas),
            wgpu::BindGroupEntry {
                binding: 5,
                resource: wgpu::BindingResource::Sampler(display_sampler),
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
