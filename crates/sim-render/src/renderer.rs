use crate::compositor::{CompositorPass, CompositorTarget, OverlayQuad};

mod panel;

use crate::frame::FrameParams;
use crate::gpu::SurfaceGpu;
use crate::pipeline::{GlobeLayouts, globe_pipeline};
use crate::scene::GlobeScene;
use crate::{
    Camera, EnvironmentFrame, FrameOutcome, GlobeMesh, MAX_PATCHES, OrthoAtlas, RenderError,
};
pub use panel::PanelSurface;

/// Globe tessellation: 12 × 24 patches of 15° × 15°, each a 16 × 16 grid.
const GLOBE_LAT_PATCHES: u32 = 12;
const GLOBE_LON_PATCHES: u32 = 24;
const GLOBE_PATCH_GRID: u32 = 16;

/// Owns the GPU, the globe geometry, the split-viewport compositor, and the per-frame uniform block.
pub struct Renderer {
    gpu: SurfaceGpu,
    layouts: GlobeLayouts,
    pipeline: wgpu::RenderPipeline,
    compositor: CompositorPass,
    scene: GlobeScene,
}

impl Renderer {
    /// Creates a renderer presenting to `target` at `width × height` pixels.
    pub async fn new(
        target: impl Into<wgpu::SurfaceTarget<'static>>,
        width: u32,
        height: u32,
    ) -> Result<Self, RenderError> {
        let gpu = SurfaceGpu::new(target.into(), width, height).await?;
        let requested = (GLOBE_LAT_PATCHES * GLOBE_LON_PATCHES) as usize;
        let mesh = GlobeMesh::build(GLOBE_LAT_PATCHES, GLOBE_LON_PATCHES, GLOBE_PATCH_GRID).ok_or(
            RenderError::TooManyPatches {
                requested,
                max: MAX_PATCHES,
            },
        )?;
        let device = &gpu.device;
        let layouts = GlobeLayouts::new(device);
        let scene = GlobeScene::new(device, &gpu.queue, &layouts, &mesh, None)?;
        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let pipeline = globe_pipeline(device, &layouts, gpu.view_format, "fs_main");
        if let Some(err) = scope.pop().await {
            return Err(RenderError::PipelineValidation(err));
        }
        let compositor = CompositorPass::new(device, gpu.view_format).await?;
        Ok(Self {
            gpu,
            layouts,
            pipeline,
            compositor,
            scene,
        })
    }

    /// Replaces the drawn terrain and its imagery.
    pub fn set_scene(
        &mut self,
        mesh: &GlobeMesh,
        atlas: Option<&OrthoAtlas>,
    ) -> Result<(), RenderError> {
        self.scene = GlobeScene::new(
            &self.gpu.device,
            &self.gpu.queue,
            &self.layouts,
            mesh,
            atlas,
        )?;
        Ok(())
    }

    /// Underlying `wgpu::Instance`, for sharing a `HostGpu` with `sim-maplibre`.
    #[must_use]
    pub fn instance(&self) -> &wgpu::Instance {
        &self.gpu.instance
    }

    /// Selected `wgpu::Adapter`, for sharing a `HostGpu` with `sim-maplibre`.
    #[must_use]
    pub fn adapter(&self) -> &wgpu::Adapter {
        &self.gpu.adapter
    }

    /// Selected graphics backend (`Metal`, `Vulkan`, `Dx12`, `BrowserWebGpu`, or `Gl`).
    #[must_use]
    pub const fn backend(&self) -> wgpu::Backend {
        self.gpu.backend
    }

    /// The device, for sharing with [`crate::SensorAperturePass`] or `sim-maplibre`.
    #[must_use]
    pub fn device(&self) -> &wgpu::Device {
        &self.gpu.device
    }

    /// The queue, for sharing with [`crate::SensorAperturePass`] or `sim-maplibre`.
    #[must_use]
    pub fn queue(&self) -> &wgpu::Queue {
        &self.gpu.queue
    }

    /// Current swap-chain dimensions `[width, height]` in physical pixels.
    #[must_use]
    pub const fn surface_size(&self) -> [u32; 2] {
        [self.gpu.config.width, self.gpu.config.height]
    }

    /// Target sRGB texture format used when presenting to the window surface.
    #[must_use]
    pub const fn view_format(&self) -> wgpu::TextureFormat {
        self.gpu.view_format
    }

    /// Resizes the swap chain and depth buffer; zero sizes are ignored.
    pub fn resize(&mut self, width: u32, height: u32) {
        self.gpu.resize(width, height);
    }

    /// Draws and presents one globe frame with no overlay quads.
    pub fn render(
        &mut self,
        camera: &Camera,
        environment: &EnvironmentFrame,
    ) -> Result<FrameOutcome, RenderError> {
        self.render_composite(Some((camera, environment)), &[])
    }

    /// Draws an optional 3D `GlobeScene` pass followed by zero-copy [`OverlayQuad`] viewports/panels.
    ///
    /// Passing `globe = None` skips the `GlobeScene` pass completely when a full-window `maplibre`
    /// `OTW`, `SensorAperture`, or `ChartMap` quad covers the presentation surface.
    pub fn render_composite(
        &mut self,
        globe: Option<(&Camera, &EnvironmentFrame)>,
        quads: &[OverlayQuad<'_>],
    ) -> Result<FrameOutcome, RenderError> {
        if let Some(message) = self.gpu.take_device_error() {
            return Err(RenderError::Device { message });
        }
        let surface_texture = match self.gpu.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(texture)
            | wgpu::CurrentSurfaceTexture::Suboptimal(texture) => texture,
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => {
                return Ok(FrameOutcome::Skipped);
            }
            wgpu::CurrentSurfaceTexture::Outdated | wgpu::CurrentSurfaceTexture::Lost => {
                self.gpu.reconfigure();
                return Ok(FrameOutcome::Skipped);
            }
            wgpu::CurrentSurfaceTexture::Validation => return Err(RenderError::SurfaceValidation),
        };
        let view = surface_texture
            .texture
            .create_view(&wgpu::TextureViewDescriptor {
                format: Some(self.gpu.view_format),
                ..Default::default()
            });
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("frame"),
            });
        let clear_compositor = if let Some((camera, environment)) = globe {
            let view_proj = camera.view_projection(self.gpu.aspect());
            let params = FrameParams::visible(view_proj, environment, [0.0; 4]);
            self.scene
                .write_frame(&self.gpu.queue, &params, camera.pose.position_ecef);
            let sky = environment.sky_linear_rgb_at(camera.pose.position_ecef);
            self.encode_globe_pass(&mut encoder, &view, sky);
            None
        } else {
            Some(wgpu::Color {
                r: 0.01,
                g: 0.01,
                b: 0.03,
                a: 1.0,
            })
        };
        self.compositor.encode(
            &self.gpu.device,
            &self.gpu.queue,
            &mut encoder,
            CompositorTarget {
                view: &view,
                clear: clear_compositor,
                size_px: self.surface_size(),
            },
            quads,
        );
        self.gpu.queue.submit([encoder.finish()]);
        self.gpu.queue.present(surface_texture);
        Ok(FrameOutcome::Presented)
    }

    fn encode_globe_pass(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        view: &wgpu::TextureView,
        [r, g, b]: [f32; 3],
    ) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("globe"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: f64::from(r),
                        g: f64::from(g),
                        b: f64::from(b),
                        a: 1.0,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &self.gpu.depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(0.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(&self.pipeline);
        self.scene.draw(&mut pass);
    }
}
