//! Slot attachment, readback, stamp, and probe accessors of [`TruthSceneMap`].

use maplibre::{headless::ViewSlotId, overlay::ProbeResults};

use super::TruthSceneMap;
use crate::{ConformalFrameStamp, MapAdapterError};

impl TruthSceneMap {
    /// Primary cockpit OTW [`ViewSlotId`].
    #[must_use]
    pub const fn otw_slot(&self) -> ViewSlotId {
        self.otw_slot
    }

    /// Optical / EVS sensor [`ViewSlotId`].
    #[must_use]
    pub const fn sensor_slot(&self) -> ViewSlotId {
        self.sensor_slot
    }

    /// Zero-copy [`wgpu::TextureView`] of any registered slot's color attachment.
    #[must_use]
    pub fn slot_color_view(&self, slot: ViewSlotId) -> Option<&wgpu::TextureView> {
        self.map.view_slot_color_view(slot)
    }

    /// Zero-copy [`wgpu::Texture`] of any registered slot's color attachment.
    #[must_use]
    pub fn slot_color_texture(&self, slot: ViewSlotId) -> Option<&wgpu::Texture> {
        self.map.view_slot_color_texture(slot)
    }

    /// Zero-copy [`wgpu::TextureView`] of any registered slot's `Depth32Float` attachment.
    #[must_use]
    pub fn slot_depth_view(&self, slot: ViewSlotId) -> Option<&wgpu::TextureView> {
        self.map.view_slot_depth_view(slot)
    }

    /// Zero-copy [`wgpu::Texture`] of any registered slot's `Depth32Float` attachment.
    #[must_use]
    pub fn slot_depth_texture(&self, slot: ViewSlotId) -> Option<&wgpu::Texture> {
        self.map.view_slot_depth_texture(slot)
    }

    /// Whether `slot` has allocated a persistent CPU `MAP_READ` staging buffer.
    #[must_use]
    pub fn slot_has_readback_buffer(&self, slot: ViewSlotId) -> Option<bool> {
        self.map.view_slot_has_readback_buffer(slot)
    }

    /// Lazily copies `slot`'s color attachment into tight RGBA8 bytes (`width * height * 4`).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn read_slot_rgba(&self, slot: ViewSlotId) -> Result<Vec<u8>, MapAdapterError> {
        let texture =
            self.map
                .view_slot_color_texture(slot)
                .ok_or_else(|| MapAdapterError::Readback {
                    detail: format!("slot {slot:?} has no color attachment"),
                })?;
        let gpu = self.runtime.host_gpu();
        super::super::readback::read_texture_rgba_blocking(&gpu.device, &gpu.queue, texture)
    }

    /// Zero-copy [`wgpu::TextureView`] of the cockpit OTW color attachment.
    #[must_use]
    pub fn otw_color_view(&self) -> Option<&wgpu::TextureView> {
        self.map.view_slot_color_view(self.otw_slot)
    }

    /// Zero-copy [`wgpu::Texture`] of the cockpit OTW color attachment.
    #[must_use]
    pub fn otw_color_texture(&self) -> Option<&wgpu::Texture> {
        self.map.view_slot_color_texture(self.otw_slot)
    }

    /// Zero-copy [`wgpu::TextureView`] of the cockpit OTW reversed-Z `Depth32Float` attachment.
    #[must_use]
    pub fn otw_depth_view(&self) -> Option<&wgpu::TextureView> {
        self.map.view_slot_depth_view(self.otw_slot)
    }

    /// Zero-copy [`wgpu::TextureView`] of the optical sensor color attachment.
    #[must_use]
    pub fn sensor_color_view(&self) -> Option<&wgpu::TextureView> {
        self.map.view_slot_color_view(self.sensor_slot)
    }

    /// Zero-copy [`wgpu::Texture`] of the optical sensor color attachment.
    #[must_use]
    pub fn sensor_color_texture(&self) -> Option<&wgpu::Texture> {
        self.map.view_slot_color_texture(self.sensor_slot)
    }

    /// Zero-copy [`wgpu::TextureView`] of the sensor reversed-Z `Depth32Float` attachment.
    #[must_use]
    pub fn sensor_depth_view(&self) -> Option<&wgpu::TextureView> {
        self.map.view_slot_depth_view(self.sensor_slot)
    }

    /// Zero-copy [`wgpu::Texture`] of the sensor reversed-Z `Depth32Float` attachment.
    #[must_use]
    pub fn sensor_depth_texture(&self) -> Option<&wgpu::Texture> {
        self.map.view_slot_depth_texture(self.sensor_slot)
    }

    /// Whether the OTW slot has allocated a CPU `MAP_READ` staging buffer (`false` in zero-copy operation).
    #[must_use]
    pub fn otw_has_readback_buffer(&self) -> Option<bool> {
        self.map.view_slot_has_readback_buffer(self.otw_slot)
    }

    /// Whether the sensor slot has allocated a CPU `MAP_READ` staging buffer (`false` unless CPU readback is requested).
    #[must_use]
    pub fn sensor_has_readback_buffer(&self) -> Option<bool> {
        self.map.view_slot_has_readback_buffer(self.sensor_slot)
    }

    /// Latest [`ConformalFrameStamp`] rendered on the OTW slot.
    #[must_use]
    pub const fn last_otw_stamp(&self) -> Option<ConformalFrameStamp> {
        self.last_otw_stamp
    }

    /// Latest [`ConformalFrameStamp`] rendered on the sensor slot.
    #[must_use]
    pub const fn last_sensor_stamp(&self) -> Option<ConformalFrameStamp> {
        self.last_sensor_stamp
    }

    /// Whether background tile workers have delivered new arrivals or animated layers require redraw.
    #[must_use]
    pub fn needs_redraw(&self) -> bool {
        self.map.needs_redraw()
    }

    /// Takes the latest completed [`ProbeResults`] for any registered `slot`, if ready.
    pub fn take_slot_probe_results(
        &mut self,
        slot: ViewSlotId,
    ) -> Result<Option<ProbeResults>, MapAdapterError> {
        self.map
            .take_view_slot_depth_probe_results(slot)
            .map_err(|source| MapAdapterError::MultiView { source })
    }

    /// Takes the latest completed [`ProbeResults`] for the OTW slot, if ready.
    pub fn take_otw_probe_results(&mut self) -> Result<Option<ProbeResults>, MapAdapterError> {
        self.take_slot_probe_results(self.otw_slot)
    }

    /// Takes the latest completed [`ProbeResults`] for the sensor slot, if ready.
    pub fn take_sensor_probe_results(&mut self) -> Result<Option<ProbeResults>, MapAdapterError> {
        self.take_slot_probe_results(self.sensor_slot)
    }
}
