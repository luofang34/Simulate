//! A vehicle's default camera views (`views/cameras.json`): chase, formation,
//! flyby, profile and planform views, airframe-mounted cameras, and cockpit
//! look presets, declared in the package's own body axes and resolved into
//! [`VirtualCameraSpec`]s the operator cycles through.
//!
//! A package that declares no views gets [`CameraViewSet::derived`], sized
//! from its JSBSim wingspan.

use std::borrow::Cow;
use std::collections::HashSet;

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_core::{
    AirframeVisibility, CAMERA_DEFAULT_PERIOD, CameraMount, FlybyTracker, VirtualCameraPlacement,
    VirtualCameraSpec,
};

use crate::PackError;

mod derived;
#[cfg(test)]
mod tests;

/// Canonical asset path for a package's camera views.
pub const CAMERA_VIEWS_ASSET: &str = "views/cameras.json";

/// The only schema [`CameraViewSet::from_json_bytes`] reads.
const SCHEMA_VERSION: u32 = 1;
/// Narrowest and widest vertical field of view a view may declare, degrees.
const FOV_LIMITS_DEG: (f64, f64) = (1.0, 150.0);

/// One vehicle's ordered camera views.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraViewSet {
    /// Schema version (`1`).
    pub schema_version: u32,
    /// Views in the order the operator cycles them.
    pub views: Vec<CameraView>,
}

/// One named camera view.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraView {
    /// Stable identifier, unique within the set (e.g. `"chase"`, `"wingtip_left"`).
    pub id: String,
    /// Human-readable title the operator sees.
    pub title: String,
    /// Vertical field of view, degrees; a spotter's zoom overrides it.
    pub fov_y_deg: f64,
    /// How much of its own airframe the view sees.
    pub airframe: ViewAirframe,
    /// Where the camera stands and what it looks at.
    pub placement: ViewPlacement,
}

/// [`AirframeVisibility`] as package data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViewAirframe {
    /// Sees the whole airframe.
    External,
    /// A pilot's eye: everything but the pilot.
    PilotEye,
    /// A cabin side window: the wing, strut and gear, not the cabin.
    CabinWindow,
    /// Sees none of the airframe.
    Hidden,
}

impl From<ViewAirframe> for AirframeVisibility {
    fn from(value: ViewAirframe) -> Self {
        match value {
            ViewAirframe::External => Self::External,
            ViewAirframe::PilotEye => Self::PilotEye,
            ViewAirframe::CabinWindow => Self::CabinWindow,
            ViewAirframe::Hidden => Self::Hidden,
        }
    }
}

/// Where a view's camera stands. Body-axis points are FRD metres (`+x`
/// forward, `+y` right, `+z` down) from the drawn airframe's origin.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ViewPlacement {
    /// Horizon-level camera trailing the vehicle's heading, tilted down to
    /// the airframe origin.
    Chase {
        /// Distance behind the origin, metres.
        back_m: f64,
        /// Height above the origin along the local vertical, metres.
        up_m: f64,
    },
    /// Camera fixed to the airframe looking at a point on it, rolling with it.
    LookAt {
        /// Camera position, FRD metres.
        eye_frd_m: [f64; 3],
        /// Point the optical axis passes through, FRD metres.
        target_frd_m: [f64; 3],
        /// Body direction the image's top leans toward; body up when omitted.
        #[serde(default = "body_up")]
        up_frd: [f64; 3],
    },
    /// The pilot's eye from the cockpit contract, turned and leaned.
    CockpitLook {
        /// Turn right of dead ahead, degrees.
        yaw_deg: f64,
        /// Tilt above the horizon, degrees.
        pitch_deg: f64,
        /// Head movement from the design eyepoint, FRD metres.
        #[serde(default)]
        eye_shift_frd_m: [f64; 3],
    },
    /// A stationary observer beside the track that watches the vehicle pass
    /// and zooms to keep it `frame_span_m` tall.
    Spotter {
        /// Along-track length of one observer station's cell, metres.
        grid_step_m: f64,
        /// Observer offset right of the track (negative is left), metres.
        lateral_m: f64,
        /// Observer height above the vehicle, metres.
        height_m: f64,
        /// Aim lead along the velocity, seconds.
        lead_time_s: f64,
        /// Scene height kept in frame at the abeam range, metres.
        frame_span_m: f64,
    },
}

const fn body_up() -> [f64; 3] {
    [0.0, 0.0, -1.0]
}

impl CameraViewSet {
    /// Parses and validates a `views/cameras.json` document.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, PackError> {
        let set: Self =
            serde_json::from_slice(bytes).map_err(|source| PackError::CameraViewsParse {
                path: CAMERA_VIEWS_ASSET.to_owned(),
                source,
            })?;
        set.validate()?;
        Ok(set)
    }

    /// Default views for a vehicle `span_m` across: see [`derived`].
    #[must_use]
    pub fn derived(span_m: f64) -> Self {
        derived::views_for_span(span_m)
    }

    /// The first [`ViewPlacement::Chase`] view's `(back_m, up_m)`.
    #[must_use]
    pub fn chase_offsets_m(&self) -> Option<(f64, f64)> {
        self.views.iter().find_map(|view| match view.placement {
            ViewPlacement::Chase { back_m, up_m } => Some((back_m, up_m)),
            _ => None,
        })
    }

    /// Every view as a `30 Hz` [`VirtualCameraSpec`], cockpit looks taken from
    /// `cockpit_eye_frd_m`, the vehicle's design eyepoint.
    #[must_use]
    pub fn virtual_cameras(&self, cockpit_eye_frd_m: DVec3) -> Vec<VirtualCameraSpec> {
        self.views
            .iter()
            .map(|view| view.virtual_camera(cockpit_eye_frd_m))
            .collect()
    }

    fn validate(&self) -> Result<(), PackError> {
        let bad = |detail: String| PackError::InvalidCameraViews {
            path: CAMERA_VIEWS_ASSET.to_owned(),
            detail,
        };
        if self.schema_version != SCHEMA_VERSION {
            return Err(bad(format!(
                "unsupported schema_version {}",
                self.schema_version
            )));
        }
        if self.views.is_empty() {
            return Err(bad("declares no views".to_owned()));
        }
        let mut ids = HashSet::new();
        for view in &self.views {
            if !ids.insert(view.id.as_str()) {
                return Err(bad(format!("view id {:?} repeats", view.id)));
            }
            view.validate()
                .map_err(|why| bad(format!("{}: {why}", view.id)))?;
        }
        Ok(())
    }
}

impl CameraView {
    fn validate(&self) -> Result<(), String> {
        let (lo, hi) = FOV_LIMITS_DEG;
        if !(self.fov_y_deg >= lo && self.fov_y_deg <= hi) {
            return Err(format!("fov_y_deg {} outside {lo}..={hi}", self.fov_y_deg));
        }
        if self.id.is_empty() || self.title.is_empty() {
            return Err("empty id or title".to_owned());
        }
        let numbers: Vec<f64> = match &self.placement {
            ViewPlacement::Chase { back_m, up_m } => vec![*back_m, *up_m],
            ViewPlacement::LookAt {
                eye_frd_m,
                target_frd_m,
                up_frd,
            } => {
                let (eye, target) = (DVec3::from(*eye_frd_m), DVec3::from(*target_frd_m));
                let forward = (target - eye).try_normalize();
                if forward.is_none_or(|f| f.cross(DVec3::from(*up_frd)).length() < 1e-3) {
                    return Err("look_at has no direction or looks along up_frd".to_owned());
                }
                eye_frd_m
                    .iter()
                    .chain(target_frd_m)
                    .chain(up_frd)
                    .copied()
                    .collect()
            }
            ViewPlacement::CockpitLook {
                yaw_deg,
                pitch_deg,
                eye_shift_frd_m,
            } => [*yaw_deg, *pitch_deg]
                .iter()
                .chain(eye_shift_frd_m)
                .copied()
                .collect(),
            ViewPlacement::Spotter {
                grid_step_m,
                lateral_m,
                height_m,
                lead_time_s,
                frame_span_m,
            } => {
                if !(*grid_step_m > 0.0 && lateral_m.abs() >= 1.0 && *frame_span_m > 0.0) {
                    return Err("spotter needs a positive grid, frame and abeam range".to_owned());
                }
                vec![
                    *grid_step_m,
                    *lateral_m,
                    *height_m,
                    *lead_time_s,
                    *frame_span_m,
                ]
            }
        };
        if numbers.iter().all(|v| v.is_finite()) {
            Ok(())
        } else {
            Err("non-finite number".to_owned())
        }
    }

    /// This view as a `30 Hz` [`VirtualCameraSpec`].
    #[must_use]
    pub fn virtual_camera(&self, cockpit_eye_frd_m: DVec3) -> VirtualCameraSpec {
        let base_intrinsics = VirtualCameraSpec::pinhole_800x600(self.fov_y_deg.to_radians());
        let placement = match &self.placement {
            ViewPlacement::Chase { back_m, up_m } => VirtualCameraPlacement::ChaseOffset {
                back_m: *back_m,
                right_m: 0.0,
                up_m: *up_m,
                pitch_rad: -up_m.atan2(back_m.max(1e-3)),
            },
            ViewPlacement::LookAt {
                eye_frd_m,
                target_frd_m,
                up_frd,
            } => {
                let eye = DVec3::from(*eye_frd_m);
                VirtualCameraPlacement::BodyMounted(CameraMount::looking_along(
                    eye,
                    DVec3::from(*target_frd_m) - eye,
                    DVec3::from(*up_frd),
                ))
            }
            ViewPlacement::CockpitLook {
                yaw_deg,
                pitch_deg,
                eye_shift_frd_m,
            } => VirtualCameraPlacement::BodyMounted(CameraMount::oblique(
                cockpit_eye_frd_m + DVec3::from(*eye_shift_frd_m),
                yaw_deg.to_radians(),
                pitch_deg.to_radians(),
                0.0,
            )),
            ViewPlacement::Spotter {
                grid_step_m,
                lateral_m,
                height_m,
                lead_time_s,
                frame_span_m,
            } => {
                let range = lateral_m.abs();
                VirtualCameraPlacement::FlybyAutoTrack(FlybyTracker {
                    grid_step_m: *grid_step_m,
                    lateral_offset_m: *lateral_m,
                    vertical_offset_m: *height_m,
                    lead_time_s: *lead_time_s,
                    reference_distance_m: range,
                    reference_fov_y_deg: 2.0 * (0.5 * frame_span_m / range).atan().to_degrees(),
                    fov_clamp_deg: (2.0, self.fov_y_deg.max(2.0)),
                })
            }
        };
        VirtualCameraSpec {
            id: Cow::Owned(self.id.clone()),
            title: Cow::Owned(self.title.clone()),
            placement,
            airframe: self.airframe.into(),
            base_intrinsics,
            period: CAMERA_DEFAULT_PERIOD,
            depth_readout: true,
        }
    }
}

/// The span a JSBSim aircraft document declares in `<metrics><wingspan>`,
/// metres, when it reads as a positive length in `FT` or `M`.
#[must_use]
pub fn jsbsim_wingspan_m(xml: &str) -> Option<f64> {
    let start = xml.find("<wingspan")?;
    let rest = xml.get(start..)?;
    let open_end = rest.find('>')?;
    let close = rest.find("</wingspan>")?;
    let tag = rest.get(..open_end)?;
    let value: f64 = rest
        .get(open_end.checked_add(1)?..close)?
        .trim()
        .parse()
        .ok()?;
    let metres = if tag.contains("\"M\"") {
        value
    } else if tag.contains("\"FT\"") || !tag.contains("unit") {
        value * 0.3048
    } else {
        return None;
    };
    (metres.is_finite() && metres > 0.0).then_some(metres)
}
