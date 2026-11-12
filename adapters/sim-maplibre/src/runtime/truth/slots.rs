//! Which built-in truth slots a frame renders.
//!
//! The OTW and sensor slots share one `HeadlessMap`. A registered slot left out of a
//! frame keeps the tiles and drapes it last retained (`maplibre`'s multi-view residency
//! floor unions them into the frame's retention set), so a host may render the sensor
//! slot only on frames with a sensor exposure without evicting the OTW slot's terrain.

/// Which built-in [`super::TruthSceneMap`] slots a frame may render.
///
/// A slot renders only when its bit is set *and* the frame supplies its stamp. Slots left
/// out are not stepped at all: their camera, tiles, and drapes stay untouched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TruthSlotMask {
    /// Primary OTW slot (and its stereo companion).
    pub otw: bool,
    /// Optical / EVS sensor slot.
    pub sensor: bool,
}

impl TruthSlotMask {
    /// OTW and sensor slots.
    pub const BOTH: Self = Self {
        otw: true,
        sensor: true,
    };
    /// OTW slot (and stereo companion) only.
    pub const OTW_ONLY: Self = Self {
        otw: true,
        sensor: false,
    };
    /// Sensor slot only.
    pub const SENSOR_ONLY: Self = Self {
        otw: false,
        sensor: true,
    };
}

/// Built-in slots one frame renders, after combining the mask with the supplied stamps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct SlotPlan {
    pub(super) otw: bool,
    pub(super) stereo: bool,
    pub(super) sensor: bool,
}

/// Plans which built-in slots render. The stereo companion reuses the primary OTW covering,
/// so it renders only alongside it.
pub(super) const fn plan_slots(
    mask: TruthSlotMask,
    has_otw_stamp: bool,
    has_stereo: bool,
    has_sensor_stamp: bool,
) -> SlotPlan {
    let otw = mask.otw && has_otw_stamp;
    SlotPlan {
        otw,
        stereo: otw && has_stereo,
        sensor: mask.sensor && has_sensor_stamp,
    }
}

impl super::TruthSceneMap {
    /// Terrain drapes the last frame drew again for the views it rendered. A view
    /// draws a bounded number of its drapes a frame, so a host that needs a view
    /// whole, such as one reading its pixels back, draws until a frame redraws none.
    #[must_use]
    pub fn drape_redraws_last_frame(&self) -> u32 {
        self.map.last_frame_stats().drape_redraws
    }
}
