use std::sync::{Arc, Mutex};

use crate::RenderError;

/// The first GPU error reported outside an error scope. wgpu's default
/// handler panics; recording it instead lets the next frame fail with a typed
/// error the app can report.
pub(crate) type DeviceErrorSlot = Arc<Mutex<Option<String>>>;

/// Depth format shared by every pass; `Depth32Float` with reversed-Z is
/// supported by both WebGPU and WebGL2.
pub(crate) const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// The device, queue, and the surface they present to.
pub(crate) struct SurfaceGpu {
    pub(crate) surface: wgpu::Surface<'static>,
    pub(crate) device: wgpu::Device,
    pub(crate) queue: wgpu::Queue,
    pub(crate) config: wgpu::SurfaceConfiguration,
    /// Format the frame is rendered through: always sRGB, so shaders output
    /// linear colour on every backend.
    pub(crate) view_format: wgpu::TextureFormat,
    pub(crate) depth_view: wgpu::TextureView,
    pub(crate) device_error: DeviceErrorSlot,
}

impl SurfaceGpu {
    pub(crate) async fn new(
        target: wgpu::SurfaceTarget<'static>,
        width: u32,
        height: u32,
    ) -> Result<Self, RenderError> {
        // Browsers may expose `navigator.gpu` yet grant no adapter; detection
        // drops WebGPU in that case so WebGL2 is tried instead.
        let instance = wgpu::util::new_instance_with_webgpu_detection(wgpu::InstanceDescriptor {
            backends: platform_backends(),
            ..wgpu::InstanceDescriptor::new_without_display_handle()
        })
        .await;
        let surface = instance
            .create_surface(target)
            .map_err(RenderError::CreateSurface)?;
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
                apply_limit_buckets: false,
            })
            .await
            .map_err(RenderError::RequestAdapter)?;
        let info = adapter.get_info();
        let adapter_name = format!("{} ({:?})", info.name, info.backend);
        tracing::info!(adapter = %adapter_name, "selected GPU adapter");

        // WebGL2 limits everywhere keep native from silently depending on
        // something the browser fallback cannot provide.
        let required_limits =
            wgpu::Limits::downlevel_webgl2_defaults().using_resolution(adapter.limits());
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("fsg device"),
                required_limits,
                ..Default::default()
            })
            .await
            .map_err(|source| RenderError::RequestDevice {
                adapter: adapter_name.clone(),
                source,
            })?;

        let device_error = DeviceErrorSlot::default();
        let slot = Arc::clone(&device_error);
        device.on_uncaptured_error(Arc::new(move |err: wgpu::Error| {
            tracing::error!(error = %err, "uncaptured GPU error");
            if let Ok(mut first) = slot.lock() {
                first.get_or_insert_with(|| err.to_string());
            }
        }));

        let mut config = surface
            .get_default_config(&adapter, width.max(1), height.max(1))
            .ok_or(RenderError::UnsupportedSurface {
                adapter: adapter_name,
            })?;
        let capabilities = surface.get_capabilities(&adapter);
        if let Some(srgb) = capabilities.formats.iter().copied().find(|f| f.is_srgb()) {
            config.format = srgb;
        }
        // Browser WebGPU offers only non-sRGB swap chains; an sRGB view of the
        // same texture still gets gamma encoding.
        let view_format = config.format.add_srgb_suffix();
        if view_format != config.format {
            config.view_formats.push(view_format);
        }
        surface.configure(&device, &config);
        let depth_view = create_depth_view(&device, &config);
        Ok(Self {
            surface,
            device,
            queue,
            config,
            view_format,
            depth_view,
            device_error,
        })
    }

    pub(crate) fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }
        self.config.width = width;
        self.config.height = height;
        self.reconfigure();
    }

    pub(crate) fn reconfigure(&mut self) {
        self.surface.configure(&self.device, &self.config);
        self.depth_view = create_depth_view(&self.device, &self.config);
    }

    /// Takes the recorded uncaptured GPU error, if any.
    pub(crate) fn take_device_error(&self) -> Option<String> {
        self.device_error
            .lock()
            .ok()
            .and_then(|mut first| first.take())
    }

    pub(crate) fn aspect(&self) -> f64 {
        f64::from(self.config.width) / f64::from(self.config.height.max(1))
    }
}

fn create_depth_view(
    device: &wgpu::Device,
    config: &wgpu::SurfaceConfiguration,
) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"),
            size: wgpu::Extent3d {
                width: config.width.max(1),
                height: config.height.max(1),
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        })
        .create_view(&wgpu::TextureViewDescriptor::default())
}

#[cfg(target_arch = "wasm32")]
fn platform_backends() -> wgpu::Backends {
    wgpu::Backends::BROWSER_WEBGPU | wgpu::Backends::GL
}

#[cfg(not(target_arch = "wasm32"))]
fn platform_backends() -> wgpu::Backends {
    wgpu::Backends::PRIMARY
}
