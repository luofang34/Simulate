//! Cameras for the Command Module's entry. Its body `+x` leads into the flow
//! behind the heat shield and `+z` faces the Earth while the lift is up.

use glam::DVec3;
use sim_core::{
    CAMERA_DEFAULT_PERIOD, CameraMount, FlybyTracker, VirtualCameraPlacement, VirtualCameraSpec,
};

/// Tilt of the crew's view from the heat-shield axis towards the Earth, as
/// the reentry run's own eye uses.
const WINDOW_PITCH_DEG: f64 = -65.0;

/// A tracking aircraft flying abeam at the capsule's height, far enough out
/// to keep a hypersonic capsule in a telephoto frame for several seconds.
/// The trackers aim without lead: at entry speed even a tenth of a second
/// leads by more than a narrow frame holds.
const RANGE_TRACKER: FlybyTracker = FlybyTracker {
    grid_step_m: 60_000.0,
    lateral_offset_m: 20_000.0,
    vertical_offset_m: 0.0,
    lead_time_s: 0.0,
    reference_distance_m: 20_000.0,
    reference_fov_y_deg: 1.0,
    fov_clamp_deg: (0.3, 30.0),
};

/// A closer observer the capsule streaks past.
const ENTRY_FLYBY: FlybyTracker = FlybyTracker {
    grid_step_m: 20_000.0,
    lateral_offset_m: 1_500.0,
    vertical_offset_m: 0.0,
    lead_time_s: 0.0,
    reference_distance_m: 1_500.0,
    reference_fov_y_deg: 3.0,
    fov_clamp_deg: (0.5, 60.0),
};

pub(super) fn cameras() -> [VirtualCameraSpec; 5] {
    let base = VirtualCameraSpec::pinhole_800x600(55.0_f64.to_radians());
    let camera = |id, title, placement| VirtualCameraSpec {
        id,
        title,
        placement,
        base_intrinsics: base,
        period: CAMERA_DEFAULT_PERIOD,
        depth_readout: true,
    };
    let sky = DVec3::NEG_Z;
    [
        camera(
            "capsule_window",
            "Capsule Forward Window",
            VirtualCameraPlacement::BodyMounted(CameraMount::oblique(
                DVec3::ZERO,
                0.0,
                WINDOW_PITCH_DEG.to_radians(),
                0.0,
            )),
        ),
        camera(
            "heatshield_wake",
            "Heatshield Wake Cam",
            VirtualCameraPlacement::BodyMounted(CameraMount::looking_along(
                DVec3::new(-1.2, 0.0, 0.0),
                DVec3::NEG_X,
                sky,
            )),
        ),
        camera(
            "starboard_horizon",
            "Starboard Horizon Cam",
            VirtualCameraPlacement::BodyMounted(CameraMount::looking_along(
                DVec3::new(0.0, 1.9, 0.0),
                DVec3::Y,
                sky,
            )),
        ),
        camera(
            "range_tracker",
            "High-Altitude Range Tracker",
            VirtualCameraPlacement::FlybyAutoTrack(RANGE_TRACKER),
        ),
        camera(
            "entry_flyby",
            "Entry Flyby Tracker",
            VirtualCameraPlacement::FlybyAutoTrack(ENTRY_FLYBY),
        ),
    ]
}
