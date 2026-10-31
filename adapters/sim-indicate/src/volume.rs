//! 3D volumetric instrument scene emission and spatial gesture evolution (`VOL-01`, `IXN-01`).

use indicate_alerts::AlertOutput;
use indicate_instrument_descriptor::{
    AuxEntry, DrawContext, InstrumentDescriptor, RenderSpace, SceneTarget, ViewLimits,
};
use indicate_instrument_scene::volume::{
    Extent3D, MAX_VOLUME_SCENE_BYTES, VolumeBandReport, VolumeSceneWriter, validate_volume_bands,
};
use indicate_instrument_state::PanelData;
use indicate_instrument_volumes::{
    CANONICAL_EXTENT, GLOBE_VOLUME_DESCRIPTOR, ORRERY_VOLUME_DESCRIPTOR, TERRAIN_VOLUME_DESCRIPTOR,
};
use indicate_interaction::{InteractionOutcome, SpatialEvent, ViewPose3D, apply_event};

use crate::PanelError;

#[cfg(test)]
mod tests;

/// Scratch buffer length capable of holding any valid 3D volumetric scene.
pub const VOLUME_SCENE_BYTES: usize = MAX_VOLUME_SCENE_BYTES;

/// Shipped 3D volumetric instruments exposed to `Simulate` and Vision Pro / WebXR hosts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VolumeInstrument {
    /// Local 3D terrain relief, flight-path vector, and waypoint handles (`terrain-volume`).
    Terrain,
    /// Planetary 3D globe, graticule, route, and airspace volumes (`globe-volume`).
    Globe,
    /// Multi-body celestial / cislunar system with time-scrubbable transfer arc (`orrery-volume`).
    Orrery,
}

impl VolumeInstrument {
    /// Returns the static [`InstrumentDescriptor`] backing this volumetric instrument.
    #[must_use]
    pub const fn descriptor(self) -> &'static InstrumentDescriptor {
        match self {
            Self::Terrain => &TERRAIN_VOLUME_DESCRIPTOR,
            Self::Globe => &GLOBE_VOLUME_DESCRIPTOR,
            Self::Orrery => &ORRERY_VOLUME_DESCRIPTOR,
        }
    }

    /// Returns the view manipulation limits declared by this volumetric instrument.
    #[must_use]
    pub const fn view_limits(self) -> ViewLimits {
        match self.descriptor().space {
            RenderSpace::Volume3D { view_limits, .. } => view_limits,
            RenderSpace::Planar2D { .. } | RenderSpace::AngularSphere { .. } => ViewLimits::LOCKED,
        }
    }
}

/// Summary of a validated 3D volumetric scene frame emitted into a caller buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VolumeRender {
    /// Byte length written to the caller's scene slice.
    pub scene_bytes: usize,
    /// Validated band presence and command counts.
    pub report: VolumeBandReport,
}

/// Applies one spatial gesture or key event to `pose` within `instrument`'s bounds.
pub fn apply_volume_gesture(
    instrument: VolumeInstrument,
    pose: &mut ViewPose3D,
    event: &SpatialEvent,
) -> Result<InteractionOutcome, PanelError> {
    apply_event(pose, event, &instrument.view_limits())
        .map_err(|source| PanelError::Interaction { source })
}

/// Emits and validates a 3D volumetric scene for `instrument` at `extent_m` (defaulting to
/// [`CANONICAL_EXTENT`] when `None`).
pub fn emit_volume_scene(
    instrument: VolumeInstrument,
    data: &PanelData,
    view: ViewPose3D,
    alerts: Option<&AlertOutput>,
    aux: &[AuxEntry<'_>],
    extent_m: Option<Extent3D>,
    out_scene: &mut [u8],
) -> Result<VolumeRender, PanelError> {
    let descriptor = instrument.descriptor();
    let capacity = out_scene.len();
    let chosen_extent = extent_m.unwrap_or(CANONICAL_EXTENT);
    let mut writer = VolumeSceneWriter::new(out_scene).map_err(|cause| PanelError::Scene {
        panel: descriptor.id,
        capacity,
        cause,
    })?;
    let ctx = DrawContext::new(data)
        .with_alerts(alerts)
        .with_view(view)
        .with_aux(aux);
    (descriptor.draw)(
        &ctx,
        SceneTarget::Volume3D {
            extent_m: chosen_extent,
            writer: &mut writer,
        },
    )
    .map_err(|source| PanelError::VolumeDraw {
        instrument: descriptor.id,
        source,
    })?;
    let scene_bytes = writer.finish();
    let written = out_scene.get(..scene_bytes).unwrap_or_default();
    let report = validate_volume_bands(written).map_err(|cause| PanelError::VolumeValidation {
        instrument: descriptor.id,
        cause,
    })?;
    Ok(VolumeRender {
        scene_bytes,
        report,
    })
}
