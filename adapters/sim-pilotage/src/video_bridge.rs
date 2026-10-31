//! Synthetic camera producer for Pilotage's `pilotage.bridge.v1` TCP sidecar
//! video link (`Pilotage:schemas/pilotage/bridge/v1/bridge.proto`).
//!
//! Turns [`sim_render::ApertureImage`] frames and vehicle kinematics into
//! length-delimited `BridgeEnvelope` messages (`"RGB_INT8"`) accepted by
//! `pilotage-sim-video`'s `BridgeClient::accept_producer`, while folding
//! inbound `BridgeCameraCommand`, `BridgeControl`, and `BridgeViewPose`
//! updates into latest-value camera view state (`FPV`, stabilized `Gimbal`
//! with zoom detents, or operator `Free` camera).

mod link;
mod pose;
#[cfg(test)]
mod tests;
mod wire;

#[cfg(not(target_arch = "wasm32"))]
pub use link::SimVideoSidecar;
pub use link::{
    CAMERA_ID_CHASE, CAMERA_ID_FPV, CAMERA_ID_GIMBAL, DEFAULT_SIDECAR_PORT, LatestFrameQueue,
    PIXEL_FORMAT_RGB_INT8, aperture_image_to_bridge_frame, rgba8_to_rgb_int8,
};
pub use pose::{
    BridgeCameraMode, CameraViewState, EllipsoidalOnlyDatum, ResolvedCameraView,
    StaticDatumSurfaces, VerticalDatum, VerticalDatumResolver, ZoomDetent,
    resolve_vehicle_camera_pose, vehicle_state_from_view_pose,
};
pub use wire::{
    BridgeCameraCommand, BridgeControl, BridgeEnvelope, BridgeFrame, BridgeGimbalPose,
    BridgeNavSat, BridgeOdometry, BridgePayload, BridgeViewPose, MAX_ENVELOPE_BYTES,
    decode_envelope, decode_length_delimited_envelope, encode_envelope,
    encode_length_delimited_envelope,
};
