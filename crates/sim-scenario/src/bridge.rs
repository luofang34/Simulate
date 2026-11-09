use sim_core::CameraFrameCapture;
use sim_render::{
    ApertureFrame, ApertureWaveband, CameraPose, EnvironmentFrame, OcclusionOverlayConfig,
    OffscreenAperture, PinholeProjection, SensorAperturePass,
};

use crate::ScenarioError;

#[cfg(test)]
mod tests;

/// How a capture is to be developed: what the camera captured says where it
/// was, and this says what it saw.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CaptureExposure {
    /// The light and air of the frame, shared with every other renderer.
    pub environment: EnvironmentFrame,
    /// What the camera senses.
    pub waveband: ApertureWaveband,
    /// What obstructs the view.
    pub occlusion: OcclusionOverlayConfig,
    /// Near plane, metres.
    pub near_m: f64,
}

/// The offscreen frame at a capture's exact pose. A camera pose has the same
/// meaning on both sides, so `f64` positions cross unchanged.
pub fn aperture_frame(capture: &CameraFrameCapture, exposure: &CaptureExposure) -> ApertureFrame {
    ApertureFrame {
        pose: CameraPose {
            position_ecef: capture.pose.position_ecef,
            orientation: capture.pose.orientation,
        },
        environment: exposure.environment,
        waveband: exposure.waveband,
        occlusion: exposure.occlusion,
    }
}

/// Renders `capture` through `pass` into a new aperture with the capture's
/// intrinsics, and returns the aperture holding the frame.
pub fn render_capture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    pass: &mut SensorAperturePass,
    capture: &CameraFrameCapture,
    exposure: &CaptureExposure,
) -> Result<OffscreenAperture, ScenarioError> {
    let projection = PinholeProjection::new(capture.intrinsics, exposure.near_m)?;
    let target = OffscreenAperture::new(device, projection)?;
    pass.render(device, queue, &target, &aperture_frame(capture, exposure));
    Ok(target)
}
