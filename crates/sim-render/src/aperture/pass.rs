use sim_scenery::SurfaceRadiometry;
use wgpu::util::DeviceExt;

use super::overlay::{OVERLAY_UNIFORM_FLOATS, overlay_uniform};
use super::stars::{STAR_SLOTS, star_spots};
use super::{
    APERTURE_COLOUR_FORMAT, APERTURE_DEPTH_FORMAT, ApertureFrame, ApertureWaveband,
    OffscreenAperture, ThermalScale,
};
use crate::frame::FrameParams;
use crate::pipeline::{GlobeLayouts, globe_pipeline};
use crate::scene::GlobeScene;
use crate::{GlobeMesh, OrthoAtlas, RenderError};

#[cfg(test)]
mod tests;

/// Pipelines and terrain for rendering simulated camera frames. One pass can
/// serve any number of [`OffscreenAperture`]s on the same device.
pub struct SensorAperturePass {
    layouts: GlobeLayouts,
    scene: GlobeScene,
    visible: wgpu::RenderPipeline,
    thermal: wgpu::RenderPipeline,
    #[cfg(not(target_arch = "wasm32"))]
    thermal_kelvin: wgpu::RenderPipeline,
    stars: wgpu::RenderPipeline,
    overlay: wgpu::RenderPipeline,
    overlay_buffer: wgpu::Buffer,
    overlay_bind_group: wgpu::BindGroup,
}

impl SensorAperturePass {
    /// Creates the pipelines and uploads `mesh` and its imagery.
    pub async fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        mesh: &GlobeMesh,
        atlas: Option<&OrthoAtlas>,
    ) -> Result<Self, RenderError> {
        let layouts = GlobeLayouts::new(device);
        let scene = GlobeScene::new(device, queue, &layouts, mesh, atlas)?;
        let overlay_layout = overlay_bind_group_layout(device);
        let overlay_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("aperture overlay"),
            contents: bytemuck::cast_slice(&[0.0_f32; OVERLAY_UNIFORM_FLOATS]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let overlay_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("aperture overlay"),
            layout: &overlay_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: overlay_buffer.as_entire_binding(),
            }],
        });
        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let visible = globe_pipeline(device, &layouts, APERTURE_COLOUR_FORMAT, "fs_main");
        let thermal = globe_pipeline(device, &layouts, APERTURE_COLOUR_FORMAT, "fs_thermal");
        #[cfg(not(target_arch = "wasm32"))]
        let thermal_kelvin = globe_pipeline(
            device,
            &layouts,
            super::THERMAL_KELVIN_FORMAT,
            "fs_thermal_kelvin",
        );
        let (stars, overlay) = full_frame_pipelines(device, &overlay_layout);
        if let Some(err) = scope.pop().await {
            return Err(RenderError::PipelineValidation(err));
        }
        Ok(Self {
            layouts,
            scene,
            visible,
            thermal,
            #[cfg(not(target_arch = "wasm32"))]
            thermal_kelvin,
            stars,
            overlay,
            overlay_buffer,
            overlay_bind_group,
        })
    }

    /// Replaces the terrain and its imagery.
    pub fn set_scene(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        mesh: &GlobeMesh,
        atlas: Option<&OrthoAtlas>,
    ) -> Result<(), RenderError> {
        self.scene = GlobeScene::new(device, queue, &self.layouts, mesh, atlas)?;
        Ok(())
    }

    /// Renders `frame` into `target` and submits it.
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        target: &OffscreenAperture,
        frame: &ApertureFrame,
    ) {
        let projection = target.projection();
        let params = FrameParams {
            view_proj: projection.view_projection(&frame.pose),
            sun_direction: frame.lighting.sun_direction_ecef,
            thermal: match frame.waveband {
                ApertureWaveband::ThermalLwir(scale) => thermal_uniform(&scale),
                _ => [0.0; 4],
            },
        };
        self.scene
            .write_frame(queue, &params, frame.pose.position_ecef);
        let (stars, reticle) = match &frame.waveband {
            ApertureWaveband::StarfieldOptical { inertial, reticle } => (
                Some(star_spots(projection, &frame.pose, inertial)),
                reticle.as_ref(),
            ),
            _ => (None, None),
        };
        let spots = stars.unwrap_or([[0.0; 4]; STAR_SLOTS]);
        let overlay = overlay_uniform(projection, &frame.occlusion, reticle, &spots);
        queue.write_buffer(&self.overlay_buffer, 0, bytemuck::cast_slice(&overlay));
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("aperture"),
        });
        {
            let mut pass = begin_pass(&mut encoder, target, &frame.waveband);
            pass.set_pipeline(match frame.waveband {
                ApertureWaveband::ThermalLwir(_) => &self.thermal,
                _ => &self.visible,
            });
            self.scene.draw(&mut pass);
            pass.set_bind_group(0, &self.overlay_bind_group, &[]);
            if stars.is_some() {
                pass.set_pipeline(&self.stars);
                pass.draw(0..3, 0..1);
            }
            let obstructed = frame.occlusion.bezel.is_some() || frame.occlusion.propeller.is_some();
            if obstructed || reticle.is_some() {
                pass.set_pipeline(&self.overlay);
                pass.draw(0..3, 0..1);
            }
        }
        queue.submit([encoder.finish()]);
    }
}

// The kelvin target is read back on the CPU, which only native builds can do;
// see `OffscreenAperture::read_blocking`.
#[cfg(not(target_arch = "wasm32"))]
impl SensorAperturePass {
    /// Renders the terrain's brightness temperature at `pose` into `target`
    /// and submits it.
    pub fn render_scene_temperature(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        target: &super::ThermalSceneTarget,
        pose: &crate::CameraPose,
    ) {
        let params = FrameParams {
            view_proj: target.projection().view_projection(pose),
            sun_direction: glam::DVec3::ZERO,
            thermal: [0.0; 4],
        };
        self.scene.write_frame(queue, &params, pose.position_ecef);
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("thermal scene"),
        });
        {
            let mut pass = target.begin_pass(&mut encoder);
            pass.set_pipeline(&self.thermal_kelvin);
            self.scene.draw(&mut pass);
        }
        queue.submit([encoder.finish()]);
    }
}

/// Maps the black and white temperatures to exitance for a black body, which
/// makes the scale independent of any one material's emissivity.
fn thermal_uniform(scale: &ThermalScale) -> [f32; 4] {
    let exitance = |kelvin: f32| SurfaceRadiometry::STEFAN_BOLTZMANN * kelvin.powi(4);
    [
        exitance(scale.black_k),
        exitance(scale.white_k),
        scale.nedt_k,
        scale.noise_seed as f32,
    ]
}

fn begin_pass<'a>(
    encoder: &'a mut wgpu::CommandEncoder,
    target: &OffscreenAperture,
    waveband: &ApertureWaveband,
) -> wgpu::RenderPass<'a> {
    let [r, g, b] = match waveband {
        ApertureWaveband::VisibleRgb { sky_linear_rgb } => *sky_linear_rgb,
        _ => [0.0; 3],
    };
    encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
        label: Some("aperture"),
        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
            view: target.colour_view(),
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
            view: target.depth_view(),
            depth_ops: Some(wgpu::Operations {
                load: wgpu::LoadOp::Clear(0.0),
                store: wgpu::StoreOp::Store,
            }),
            stencil_ops: None,
        }),
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    })
}

fn overlay_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("aperture overlay"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
    })
}

/// The star and overlay pipelines. Neither writes depth: stars pass only
/// where depth is still the cleared far plane, so the limb hides them, and
/// the overlay sits in front of everything.
fn full_frame_pipelines(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
) -> (wgpu::RenderPipeline, wgpu::RenderPipeline) {
    let shader = device.create_shader_module(wgpu::include_wgsl!("../shaders/aperture.wgsl"));
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("aperture overlay"),
        bind_group_layouts: &[Some(layout)],
        ..Default::default()
    });
    let pipeline = |entry: &str, blend, depth_compare| {
        device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some(entry),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_full_frame"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some(entry),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: APERTURE_COLOUR_FORMAT,
                    blend: Some(blend),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: Some(wgpu::DepthStencilState {
                format: APERTURE_DEPTH_FORMAT,
                depth_write_enabled: Some(false),
                depth_compare: Some(depth_compare),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        })
    };
    let additive = wgpu::BlendState {
        color: wgpu::BlendComponent {
            src_factor: wgpu::BlendFactor::One,
            dst_factor: wgpu::BlendFactor::One,
            operation: wgpu::BlendOperation::Add,
        },
        alpha: wgpu::BlendComponent::OVER,
    };
    (
        pipeline("fs_stars", additive, wgpu::CompareFunction::GreaterEqual),
        pipeline(
            "fs_overlay",
            wgpu::BlendState::ALPHA_BLENDING,
            wgpu::CompareFunction::Always,
        ),
    )
}
