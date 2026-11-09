//! Rendered camera frames as Navigate's imaging-sensor input.
//!
//! The parts of an [`OwnedCameraFrame`] are the camera-side fields of
//! `navigate_visual_flight::LiveFrameInput`:
//!
//! | `LiveFrameInput` field | Source here |
//! |---|---|
//! | `stamp` | [`OwnedCameraFrame::stamp`], from [`MasterClockBridge`] at the capture instant |
//! | `pixels` | [`OwnedCameraFrame::frame`]'s [`ImageView`] (`Gray8` or `Rgb8`) |
//! | `intrinsics` | [`ImagerModel::intrinsics`] from [`imager_model`] |
//! | `mount` | [`navigate_mount`], fixed configuration of the airframe |
//! | `static_valid_mask` | airframe geometry known before flight only |
//! | `imu_attitude`, `baro_alt_msl_m`, `filter_prior` | sensor and estimate records, never [`sim_core::CameraFrameCapture`]'s truth |
//!
//! A per-frame rendered occlusion mask is simulation truth
//! ([`crate::SensorTruth`]); feeding it as `static_valid_mask` would hand the
//! masker the answer it is being scored on.

use glam::{DMat3, DVec3};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use navigate_camera::{ImageFrame, ImageView, ImagerModel, Intrinsics, OPTICAL_FROM_GL, Pixels};
use navigate_contract::{ObservationStamp, SourceEpoch, SourceId, WrappingSequence};
use sim_core::{CameraFrameCapture, CameraMount};
use sim_geo::SensorIntrinsics;

use crate::{MasterClockBridge, NavigateBridgeError};

#[cfg(test)]
mod tests;

/// Rec. 709 luma weights, as `navigate_camera::ImageView::luma` applies them
/// to sRGB bytes, so a frame converted here reads as Navigate would read the
/// same pixels as `Rgb8`.
const LUMA_WEIGHTS: [f32; 3] = [0.2126, 0.7152, 0.0722];

/// The pixel layout a frame is handed to Navigate in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum FramePixelFormat {
    /// One 8-bit luma sample per pixel.
    Gray8,
    /// Three 8-bit sRGB samples per pixel; alpha is dropped.
    Rgb8,
}

/// Navigate's description of a visible pinhole camera with `intrinsics`.
pub fn imager_model(intrinsics: &SensorIntrinsics) -> Result<ImagerModel, NavigateBridgeError> {
    let model = ImagerModel::pinhole(Intrinsics {
        width: intrinsics.width,
        height: intrinsics.height,
        fx: intrinsics.fx,
        fy: intrinsics.fy,
        cx: intrinsics.cx,
        cy: intrinsics.cy,
    });
    model
        .validate()
        .map_err(|source| NavigateBridgeError::Camera {
            what: "sensor intrinsics",
            source,
        })?;
    Ok(model)
}

/// `mount` in Navigate's convention.
///
/// Both use forward-right-down body axes. The simulator's camera axes are the
/// renderer's (`+Y` up, `-Z` forward) and its offset is from the component
/// tree's root; Navigate's are optical (`+Y` down, `+Z` forward) and its
/// lever arm is from the body position, which the simulator reports at the
/// centre of gravity `cg_body_m`.
pub fn navigate_mount(mount: &CameraMount, cg_body_m: DVec3) -> navigate_camera::CameraMount {
    let optical_from_render = DMat3::from_cols_array(&OPTICAL_FROM_GL).transpose();
    let cam_from_body = optical_from_render * DMat3::from_quat(mount.body_to_camera);
    let rotation =
        Rotation3::from_matrix(&Matrix3::from_column_slice(&cam_from_body.to_cols_array()));
    let lever = mount.offset_body_m - cg_body_m;
    navigate_camera::CameraMount {
        lever_arm_body_m: Vector3::new(lever.x, lever.y, lever.z),
        cam_from_body: UnitQuaternion::from_rotation_matrix(&rotation),
    }
}

/// One rendered exposure with owned pixels in a layout Navigate reads.
#[derive(Debug, Clone, PartialEq)]
pub struct OwnedCameraFrame {
    stamp: ObservationStamp,
    imager: ImagerModel,
    format: FramePixelFormat,
    pixels: Vec<u8>,
}

impl OwnedCameraFrame {
    /// Converts tightly packed, top-row-first sRGB RGBA8 pixels of the
    /// capture's size, such as `sim_render::ApertureImage::rgba8`.
    pub fn from_rgba8(
        stamp: ObservationStamp,
        intrinsics: &SensorIntrinsics,
        rgba8: &[u8],
        format: FramePixelFormat,
    ) -> Result<Self, NavigateBridgeError> {
        let imager = imager_model(intrinsics)?;
        let needed = (intrinsics.width as usize)
            .saturating_mul(intrinsics.height as usize)
            .saturating_mul(4);
        if rgba8.len() != needed {
            return Err(NavigateBridgeError::FrameSize {
                what: "RGBA8 buffer",
                width: intrinsics.width,
                height: intrinsics.height,
                needed,
                actual: rgba8.len(),
            });
        }
        // The length check leaves no partial pixel.
        let (rgba, _) = rgba8.as_chunks::<4>();
        let pixels = match format {
            FramePixelFormat::Gray8 => rgba.iter().map(rec709_luma).collect(),
            FramePixelFormat::Rgb8 => rgba.iter().flat_map(|&[r, g, b, _]| [r, g, b]).collect(),
        };
        Ok(Self {
            stamp,
            imager,
            format,
            pixels,
        })
    }

    /// Source identity, sequence, and acquisition time.
    pub const fn stamp(&self) -> ObservationStamp {
        self.stamp
    }

    /// The sensor that made the frame.
    pub const fn imager(&self) -> &ImagerModel {
        &self.imager
    }

    /// The layout of [`Self::pixels`].
    pub const fn format(&self) -> FramePixelFormat {
        self.format
    }

    /// Tightly packed samples, top row first.
    pub fn pixels(&self) -> &[u8] {
        &self.pixels
    }

    /// The frame as Navigate's imaging-sensor input, borrowing the pixels.
    pub fn frame(&self) -> Result<ImageFrame<'_>, NavigateBridgeError> {
        let k = &self.imager.intrinsics;
        let pixels = match self.format {
            FramePixelFormat::Gray8 => Pixels::Gray8 {
                data: &self.pixels,
                stride: k.width as usize,
            },
            FramePixelFormat::Rgb8 => Pixels::Rgb8 {
                data: &self.pixels,
                stride: k.width as usize * 3,
            },
        };
        let camera = |source| NavigateBridgeError::Camera {
            what: "rendered frame",
            source,
        };
        let frame = ImageFrame {
            stamp: self.stamp,
            imager: self.imager,
            image: ImageView::new(k.width, k.height, pixels).map_err(camera)?,
        };
        frame.validate().map_err(camera)?;
        Ok(frame)
    }
}

fn rec709_luma(px: &[u8; 4]) -> u8 {
    let luma: f32 = px
        .iter()
        .zip(LUMA_WEIGHTS)
        .map(|(v, w)| f32::from(*v) * w)
        .sum();
    (luma + 0.5).clamp(0.0, 255.0) as u8
}

/// Stamps each rendered exposure of one camera on the master clock, in
/// capture order.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraFrameLink {
    clock: MasterClockBridge,
    source: SourceId,
    epoch: SourceEpoch,
    next: WrappingSequence,
    format: FramePixelFormat,
}

impl CameraFrameLink {
    /// A link for the camera that Navigate knows as `source`.
    pub const fn new(
        clock: MasterClockBridge,
        source: SourceId,
        epoch: SourceEpoch,
        format: FramePixelFormat,
    ) -> Self {
        Self {
            clock,
            source,
            epoch,
            next: WrappingSequence::new(0),
            format,
        }
    }

    /// The frame of `capture` rendered as `rgba8`, stamped at its exposure.
    pub fn link(
        &mut self,
        capture: &CameraFrameCapture,
        rgba8: &[u8],
    ) -> Result<OwnedCameraFrame, NavigateBridgeError> {
        let stamp = self
            .clock
            .stamp(self.source, self.epoch, self.next, capture.timestamp)?;
        let frame = OwnedCameraFrame::from_rgba8(stamp, &capture.intrinsics, rgba8, self.format)?;
        self.next = self.next.next();
        Ok(frame)
    }
}
