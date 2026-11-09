//! Draw-graph node issuing the executing slot's aircraft draws.

use maplibre::{
    render::{
        RenderResources,
        eventually::Eventually::Initialized,
        eye_covering::CurrentViewSlot,
        graph::{Node, RenderContext},
        resource::Texture,
    },
    tcs::world::World,
};

use super::USD_AIRCRAFT_PASS;
use super::gpu::{SlotGpu, UsdPassGpu, UsdPassState};
use super::shadow::gpu::begin_caster_pass;

/// Loads the slot's colour and depth and draws its aircraft parts into them.
pub(super) struct UsdAircraftPassNode;

impl Node for UsdAircraftPassNode {
    fn run(&self, render_context: &mut RenderContext, state: &RenderResources, world: &World) {
        let Some(slot) = world
            .resources
            .get::<CurrentViewSlot>()
            .and_then(|current| current.0)
        else {
            return;
        };
        let Some(gpu) = world
            .resources
            .get::<UsdPassState>()
            .and_then(|pass| pass.gpu.as_ref())
        else {
            return;
        };
        let Some(slot_gpu) = gpu.slots.get(&slot).filter(|s| !s.commands.is_empty()) else {
            return;
        };
        let (
            Initialized(render_target),
            Initialized(multisampling_texture),
            Initialized(depth_texture),
        ) = (
            &state.render_target,
            &state.multisampling_texture,
            &state.depth_texture,
        )
        else {
            return;
        };
        if slot_gpu.shadowed {
            draw_casters(&mut render_context.command_encoder, gpu, slot_gpu);
        }
        let mut pass = begin_pass(
            &mut render_context.command_encoder,
            render_target,
            multisampling_texture.as_ref(),
            depth_texture,
        );
        pass.set_bind_group(1, &slot_gpu.shadow.receiver_group, &[]);
        if slot_gpu.ground_shadow {
            pass.set_pipeline(&gpu.shadow.ground);
            pass.set_bind_group(0, &slot_gpu.bind_group, &[0]);
            pass.draw(0..4, 0..1);
        }
        let mut bound = None;
        for command in &slot_gpu.commands {
            let Some(Some(mesh)) = gpu
                .meshes
                .get(&command.asset)
                .and_then(|meshes| meshes.get(command.part_index))
            else {
                continue;
            };
            if bound != Some(command.pipeline) {
                pass.set_pipeline(gpu.pipeline(command.pipeline));
                bound = Some(command.pipeline);
            }
            pass.set_bind_group(0, &slot_gpu.bind_group, &[command.offset]);
            pass.set_vertex_buffer(0, mesh.vertices.slice(..));
            pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..mesh.count, 0, 0..1);
        }
    }
}

/// Refills the shared shadow map with ownship's opaque parts seen from this slot's Sun.
fn draw_casters(encoder: &mut wgpu::CommandEncoder, gpu: &UsdPassGpu, slot_gpu: &SlotGpu) {
    let mut pass = begin_caster_pass(encoder, &gpu.shadow.map);
    pass.set_pipeline(&gpu.shadow.caster);
    pass.set_bind_group(1, &slot_gpu.shadow.caster_group, &[]);
    for command in slot_gpu.commands.iter().filter(|c| c.casts_shadow) {
        let Some(Some(mesh)) = gpu
            .meshes
            .get(&command.asset)
            .and_then(|meshes| meshes.get(command.part_index))
        else {
            continue;
        };
        pass.set_bind_group(0, &slot_gpu.bind_group, &[command.offset]);
        pass.set_vertex_buffer(0, mesh.vertices.slice(..));
        pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
        pass.draw_indexed(0..mesh.count, 0, 0..1);
    }
}

/// Opens a pass that keeps the map's colour (resolving MSAA when enabled) and depth.
fn begin_pass<'e>(
    encoder: &'e mut wgpu::CommandEncoder,
    render_target: &'e wgpu::TextureView,
    multisampling_texture: Option<&'e Texture>,
    depth_texture: &'e Texture,
) -> wgpu::RenderPass<'e> {
    let has_stencil = depth_texture.texture.format().has_stencil_aspect();
    let (view, resolve_target) = match multisampling_texture {
        Some(texture) => (&*texture.view, Some(render_target)),
        None => (render_target, None),
    };
    let keep_depth = wgpu::Operations {
        load: wgpu::LoadOp::Load,
        store: wgpu::StoreOp::Store,
    };
    encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
        multiview_mask: None,
        label: Some(USD_AIRCRAFT_PASS),
        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
            depth_slice: None,
            view,
            resolve_target,
            ops: wgpu::Operations {
                load: wgpu::LoadOp::Load,
                store: wgpu::StoreOp::Store,
            },
        })],
        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
            view: &depth_texture.view,
            depth_ops: Some(keep_depth),
            stencil_ops: has_stencil.then_some(wgpu::Operations {
                load: wgpu::LoadOp::Load,
                store: wgpu::StoreOp::Store,
            }),
        }),
        timestamp_writes: None,
        occlusion_query_set: None,
    })
}
