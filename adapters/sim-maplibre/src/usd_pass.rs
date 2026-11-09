//! `maplibre` render-graph plugin drawing OpenUSD aircraft into a map's view slots.
//!
//! The pass runs after `SCENE_3D_PASS` and before `OVERLAY_PASS`, loading the slot's
//! colour (MSAA when enabled) and reversed-Z depth so terrain hides aircraft and
//! aircraft hide one another. Each view slot owns its own uniform buffer, so several
//! slots drawn in one multi-view frame never overwrite each other's draws.
//!
//! Matrices are composed in `f64` relative to the slot's camera anchor, on the same
//! mean sphere `maplibre` draws terrain on, and only then narrowed to `f32`.

use std::{cell::RefCell, collections::BTreeMap, rc::Rc};

use glam::{DMat3, DMat4, DQuat, DVec3, Mat4};
use maplibre::{
    headless::{Placement3d, ViewSlotId, environment::HeadlessEnvironment},
    kernel::Kernel,
    overlay::OVERLAY_PASS,
    plugin::{Plugin, PluginError},
    render::{RenderStageLabel, Renderer, draw_graph::node::SCENE_3D_PASS},
    schedule::Schedule,
    tcs::world::World,
};
use sim_core::AirframeArticulation;
use sim_geo::{Geodetic, LocalFrame};

use crate::{
    ConformalFrameStamp, PartRole, SlotCameraRole,
    pose::{anchor_enu_from_local_enu, clip_from_anchor_enu, local_enu_from_geodetic},
};

mod gpu;
mod library;
mod node;
mod shadow;
mod skiway;
#[cfg(test)]
mod tests;
pub(crate) mod vertex;

pub(crate) use library::{AircraftLibrary, AssetKey, SharedAircraftLibrary};
pub use library::{OwnshipVisualKind, StructuralDatum};
pub(crate) use shadow::{ShadowFrame, ownship_shadow};
pub(crate) use skiway::PolarSkiwayKey;

/// Draw-graph node label of the aircraft pass.
pub const USD_AIRCRAFT_PASS: &str = "usd_aircraft_pass";

/// Bytes per draw in a slot's uniform buffer (the dynamic-offset alignment).
pub(crate) const DRAW_STRIDE: u64 = 256;
/// Bytes of the `Draw` uniform block the shader reads.
pub(crate) const DRAW_BYTES: usize = 224;

/// One aircraft to draw: placement, articulation, and whether it is ownship.
#[derive(Clone, Copy, Debug)]
pub struct AircraftInstance {
    /// Position (MSL) and orientation of the model's `+X` right, `+Y` forward, `+Z` up axes
    /// in local East-North-Up, with per-slot visibility.
    pub placement: Placement3d,
    /// Control surfaces, propeller, gear, and lights.
    pub articulation: AirframeArticulation,
    /// Whether this is the vehicle the cameras ride on; its parts follow each slot's
    /// [`SlotCameraRole`], while traffic is always seen whole.
    pub ownship: bool,
    /// Height of the ground beneath, in the same mean-sea-level reference as
    /// `placement.altitude_meters`; ownship's sun shadow falls on it, and `None` (unknown
    /// ground) leaves the shadow on the airframe only.
    pub ground_height_m: Option<f64>,
}

/// One packed draw of one part in one slot.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PackedDraw {
    pub(crate) asset: AssetKey,
    pub(crate) part_index: usize,
    pub(crate) translucent: bool,
    pub(crate) double_sided: bool,
    /// Ownship's opaque parts fill the slot's shadow map.
    pub(crate) casts_shadow: bool,
    pub(crate) uniform: [u8; DRAW_BYTES],
}

/// One slot's aircraft this frame: its draws, opaque before translucent and far before
/// near, and ownship's sun shadow.
#[derive(Clone, Debug, Default)]
pub(crate) struct SlotScene {
    pub(crate) draws: Vec<PackedDraw>,
    pub(crate) shadow: Option<ShadowFrame>,
}

/// Every slot's aircraft scene this frame.
pub(crate) type SlotDraws = Rc<RefCell<BTreeMap<ViewSlotId, SlotScene>>>;

/// Registers the aircraft pass and its prepare system on a truth-scene map.
pub(crate) struct UsdScene3dPlugin {
    pub(crate) library: SharedAircraftLibrary,
    pub(crate) draws: SlotDraws,
}

impl Plugin<HeadlessEnvironment> for UsdScene3dPlugin {
    fn build(
        &self,
        schedule: &mut Schedule,
        _kernel: Rc<Kernel<HeadlessEnvironment>>,
        world: &mut World,
        renderer: &mut Renderer,
    ) -> Result<(), PluginError> {
        world.resources.insert(gpu::UsdPassState::default());
        let library = Rc::clone(&self.library);
        let draws = Rc::clone(&self.draws);
        schedule.add_system_to_stage(RenderStageLabel::Prepare, move |ctx: &mut _| {
            gpu::prepare(ctx, &library, &draws);
            Ok(())
        })?;
        let draw_graph = &mut renderer.render_graph;
        draw_graph.add_node(USD_AIRCRAFT_PASS, node::UsdAircraftPassNode)?;
        draw_graph.add_node_edge(SCENE_3D_PASS, USD_AIRCRAFT_PASS)?;
        draw_graph.add_node_edge(USD_AIRCRAFT_PASS, OVERLAY_PASS)?;
        Ok(())
    }
}

/// Inputs shared by every draw of one slot.
pub(crate) struct AircraftSlotView<'a> {
    pub(crate) stamp: &'a ConformalFrameStamp,
    pub(crate) sun_enu: DVec3,
    pub(crate) daylight: f32,
    pub(crate) ownship_role: SlotCameraRole,
    pub(crate) slot: ViewSlotId,
}

/// Packs every visible aircraft part of `aircraft` (plus any polar skiway) for one slot;
/// ownship draws the library's selected airframe (nothing when hidden) and traffic the
/// scene's asset.
pub(crate) fn pack_slot_draws(
    library: &AircraftLibrary,
    aircraft: &[AircraftInstance],
    polar_skiway: Option<(AssetKey, &sim_scenery::LandingRunwayGeometry)>,
    view: &AircraftSlotView<'_>,
) -> Vec<PackedDraw> {
    let clip_from_enu = clip_from_anchor_enu(view.stamp);
    let mut packed: Vec<(f64, PackedDraw)> = Vec::new();
    if let Some((asset_key, runway)) = polar_skiway
        && let Some(asset) = library.get(asset_key)
        && let Some(enu_from_skiway) =
            enu_from_aircraft(view.stamp, &skiway::skiway_placement(runway, view.stamp))
    {
        for part_draw in
            asset.evaluate_draws(&AirframeArticulation::default(), SlotCameraRole::External)
        {
            if let Some(part) = asset.parts().get(part_draw.part_index) {
                packed.push(pack_part(
                    asset_key,
                    part,
                    &part_draw,
                    enu_from_skiway * part_draw.part_to_aircraft,
                    clip_from_enu,
                    view,
                ));
            }
        }
    }
    for instance in aircraft {
        if !instance
            .placement
            .view_mask
            .matches_current(Some(view.slot))
        {
            continue;
        }
        let Some((asset_key, asset)) = library.asset_for(instance.ownship) else {
            continue;
        };
        let Some(enu_from_aircraft) = enu_from_aircraft(view.stamp, &instance.placement) else {
            continue;
        };
        let role = if instance.ownship {
            view.ownship_role
        } else {
            SlotCameraRole::External
        };
        for part_draw in asset.evaluate_draws(&instance.articulation, role) {
            let Some(part) = asset.parts().get(part_draw.part_index) else {
                continue;
            };
            let enu_from_part = enu_from_aircraft * part_draw.part_to_aircraft;
            let (distance, mut draw) = pack_part(
                asset_key,
                part,
                &part_draw,
                enu_from_part,
                clip_from_enu,
                view,
            );
            draw.casts_shadow = instance.ownship && !draw.translucent;
            packed.push((distance, draw));
        }
    }
    // Opaque first; translucent back to front so blending composes correctly.
    packed.sort_by(|(da, a), (db, b)| {
        a.translucent
            .cmp(&b.translucent)
            .then_with(|| db.total_cmp(da))
    });
    packed.into_iter().map(|(_, draw)| draw).collect()
}

/// One part's draw and its distance from the camera, for back-to-front sorting.
fn pack_part(
    asset: AssetKey,
    part: &crate::UsdMeshPart,
    part_draw: &crate::UsdPartDraw,
    enu_from_part: DMat4,
    clip_from_enu: DMat4,
    view: &AircraftSlotView<'_>,
) -> (f64, PackedDraw) {
    let translucent = part_draw.is_prop_disc || part_draw.opacity < 0.999;
    let kind = if part_draw.is_prop_disc {
        if part.prim_path.contains("P38") {
            3.0
        } else {
            2.0
        }
    } else if translucent {
        1.0
    } else {
        0.0
    };
    let material = part.material;
    let opacity = if part.role == PartRole::CabinGlass {
        part_draw.opacity.min(0.28)
    } else {
        part_draw.opacity
    };
    let [r, g, b] = material.diffuse_rgb;
    let block = DrawBlock {
        clip_from_part: (clip_from_enu * enu_from_part).as_mat4(),
        enu_from_part: enu_from_part.as_mat4(),
        base_color: [r, g, b, opacity],
        emissive: emissive(
            material.emissive_rgb,
            part_draw.emissive_scale,
            material.metallic,
        ),
        params: [
            material.roughness,
            part_draw.prop_phase_rad,
            part_draw.shutter_blur_rad,
            kind,
        ],
        sun: [
            view.sun_enu.x as f32,
            view.sun_enu.y as f32,
            view.sun_enu.z as f32,
            view.daylight,
        ],
        detail: part.detail.uniform_rows(),
    };
    (
        enu_from_part.w_axis.truncate().length(),
        PackedDraw {
            asset,
            part_index: part_draw.part_index,
            translucent,
            double_sided: part.double_sided || translucent,
            casts_shadow: false,
            uniform: block.to_le_bytes(),
        },
    )
}

fn emissive(rgb: [f32; 3], scale: f32, metallic: f32) -> [f32; 4] {
    let [r, g, b] = rgb;
    [r * scale, g * scale, b * scale, metallic]
}

/// Model transform from the aircraft's USD axes into the slot's anchor ENU frame.
pub(super) fn enu_from_aircraft(
    stamp: &ConformalFrameStamp,
    placement: &Placement3d,
) -> Option<DMat4> {
    let geodetic = Geodetic {
        lat_rad: placement.position.latitude.to_radians(),
        lon_rad: placement.position.longitude.to_radians(),
        height_m: placement.altitude_meters + stamp.geoid_undulation_m,
    };
    let offset = local_enu_from_geodetic(stamp, &geodetic, stamp.geoid_undulation_m)?;
    let [x, y, z, w] = placement.orientation.enu_from_body;
    let local_from_body = DMat3::from_quat(DQuat::from_xyzw(x, y, z, w).normalize());
    let anchor_from_local = anchor_enu_from_local_enu(
        stamp,
        placement.position.latitude,
        placement.position.longitude,
    );
    let rotation = anchor_from_local * local_from_body;
    rotation.is_finite().then(|| {
        DMat4::from_cols(
            rotation.x_axis.extend(0.0),
            rotation.y_axis.extend(0.0),
            rotation.z_axis.extend(0.0),
            offset.extend(1.0),
        )
    })
}

/// Direction to the sun in the slot's anchor ENU frame and a daylight factor in `[0, 1]`.
pub(crate) fn sun_in_anchor_enu(stamp: &ConformalFrameStamp, sun_ecef: DVec3) -> (DVec3, f32) {
    let anchor = Geodetic::from_degrees(stamp.anchor.lat_deg, stamp.anchor.lon_deg, 0.0);
    let local = LocalFrame::at(&anchor);
    let sun = sun_ecef.normalize_or(local.up());
    let enu = DVec3::new(
        sun.dot(local.east()),
        sun.dot(local.north()),
        sun.dot(local.up()),
    );
    let t = ((enu.z + 0.05) / 0.25).clamp(0.0, 1.0);
    (enu, (t * t * (3.0 - 2.0 * t)) as f32)
}

/// The shader's `Draw` block, packed little-endian without `unsafe`.
pub(crate) struct DrawBlock {
    pub(crate) clip_from_part: Mat4,
    pub(crate) enu_from_part: Mat4,
    pub(crate) base_color: [f32; 4],
    pub(crate) emissive: [f32; 4],
    pub(crate) params: [f32; 4],
    pub(crate) sun: [f32; 4],
    /// [`crate::SurfaceDetail`] rows: panel pitch and weathering, then soot.
    pub(crate) detail: [[f32; 4]; 2],
}

impl DrawBlock {
    pub(crate) fn to_le_bytes(&self) -> [u8; DRAW_BYTES] {
        let floats = self
            .clip_from_part
            .to_cols_array()
            .into_iter()
            .chain(self.enu_from_part.to_cols_array())
            .chain(self.base_color)
            .chain(self.emissive)
            .chain(self.params)
            .chain(self.sun)
            .chain(self.detail.into_iter().flatten());
        let mut out = [0_u8; DRAW_BYTES];
        for (chunk, value) in out.as_chunks_mut::<4>().0.iter_mut().zip(floats) {
            *chunk = value.to_le_bytes();
        }
        out
    }
}
