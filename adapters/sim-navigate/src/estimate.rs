use glam::{DMat3, DQuat, DVec3};
use nalgebra::{Matrix6, Quaternion, UnitQuaternion, Vector3};
use navigate_visual::{
    CameraModel, CameraPose, Estimate, EstimateQuality, FrameStamp, LocalFrame, MapRevision,
};
use sha2::{Digest, Sha256};
use sim_core::{SensorIntrinsics, SimInstant};
use sim_geo::{Geodetic, LocalFrame as EnuFrame, ecef_to_geodetic, geodetic_to_ecef};

use crate::NavigateBridgeError;

#[cfg(test)]
mod tests;

/// Image-geometry attitude error declared for the synthetic estimator, radians.
/// A truth-derived estimate has no residuals to compute one from.
const GEOMETRY_ATTITUDE_SIGMA_RAD: f64 = 0.0205;
/// Names estimates that came from no image match.
const BACKEND: &str = "simulate-synthetic-estimator";

/// The corridor catalog's anchor frame in both simulators' conventions: Simulate's
/// tangent frame at the anchor, and Navigate's WGS84 east, north, up frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CorridorFrame {
    origin_ecef: DVec3,
    enu: EnuFrame,
    local: LocalFrame,
}

impl CorridorFrame {
    /// The frame anchored at `anchor`.
    pub fn new(anchor: &Geodetic) -> Result<Self, NavigateBridgeError> {
        let (lat_deg, lon_deg) = (anchor.lat_deg(), anchor.lon_deg());
        let local = LocalFrame::wgs84_enu(lat_deg, lon_deg).map_err(|source| {
            NavigateBridgeError::InvalidAnchor {
                lat_deg,
                lon_deg,
                source,
            }
        })?;
        Ok(Self {
            origin_ecef: geodetic_to_ecef(anchor),
            enu: EnuFrame::at(anchor),
            local,
        })
    }

    /// Navigate's view of the frame.
    pub const fn local(&self) -> LocalFrame {
        self.local
    }

    /// The ECEF position of a point given in Simulate's ENU anchor frame.
    pub fn enu_to_ecef(&self, position_enu_m: DVec3) -> DVec3 {
        self.origin_ecef + self.enu.enu_to_ecef() * position_enu_m
    }

    /// A camera pose in Navigate's frame from its ECEF position and
    /// camera-to-ECEF rotation. Simulate's and Navigate's camera axes are both
    /// right, up, back here.
    pub fn camera_pose(
        &self,
        position_ecef: DVec3,
        camera_to_ecef: DQuat,
    ) -> Result<CameraPose, NavigateBridgeError> {
        let position = ecef_to_geodetic(position_ecef);
        let [x, y, z] = position_ecef.to_array();
        let local = self
            .local
            .local([position.lat_deg(), position.lon_deg(), position.height_m])
            .map_err(|source| NavigateBridgeError::OutsideFrame {
                position_ecef_m: [x, y, z],
                source,
            })?;
        let ecef_to_enu = DQuat::from_mat3(&self.enu.enu_to_ecef()).inverse();
        Ok(CameraPose {
            position: local,
            orientation: to_nalgebra(ecef_to_enu * camera_to_ecef),
        })
    }
}

/// The same rotation in nalgebra.
pub(crate) fn to_nalgebra(rotation: DQuat) -> UnitQuaternion<f64> {
    let DQuat { x, y, z, w } = rotation;
    UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))
}

/// Navigate's name for a sensor's optics.
pub(crate) const fn camera_model(intrinsics: &SensorIntrinsics) -> CameraModel {
    CameraModel {
        width: intrinsics.width,
        height: intrinsics.height,
        fx: intrinsics.fx,
        fy: intrinsics.fy,
        cx: intrinsics.cx,
        cy: intrinsics.cy,
    }
}

/// Evidence digest of one simulated exposure. With no pixels to hash, the
/// stamp, optics, and instant stand in for them, in the same role as
/// `navigate_visual::Frame::evidence_sha256`: unique per exposure, the same on
/// every replay.
pub(crate) fn evidence_digest(
    sequence: u64,
    capture_time_ns: u64,
    captured_at: SimInstant,
    intrinsics: &SensorIntrinsics,
) -> String {
    let mut hash = Sha256::new();
    hash.update(b"simulate-visual-frame-v1");
    hash.update(sequence.to_le_bytes());
    hash.update(capture_time_ns.to_le_bytes());
    hash.update(captured_at.ticks_u128().to_le_bytes());
    hash.update(intrinsics.width.to_le_bytes());
    hash.update(intrinsics.height.to_le_bytes());
    for value in [intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy] {
        hash.update(value.to_le_bytes());
    }
    hash.finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

/// Everything one [`navigate_visual::Estimate`] says.
pub(crate) struct EstimateParts<'a> {
    pub(crate) stamp: FrameStamp,
    pub(crate) observation_sha256: String,
    pub(crate) map: &'a MapRevision,
    pub(crate) frame: LocalFrame,
    pub(crate) pose: CameraPose,
    pub(crate) covariance_enu_m2: DMat3,
}

/// A map-relative pose estimate as Navigate's localizer reports one. The
/// quality counters stay zero and the backend names the synthetic source: no
/// image was matched, and nothing downstream may read otherwise.
pub(crate) fn estimate(parts: EstimateParts) -> Estimate {
    let mut covariance = Matrix6::zeros();
    for (column, axis) in parts
        .covariance_enu_m2
        .to_cols_array_2d()
        .iter()
        .enumerate()
    {
        for (row, value) in axis.iter().enumerate() {
            if let Some(cell) = covariance.get_mut((row, column)) {
                *cell = *value;
            }
        }
    }
    let attitude_variance = GEOMETRY_ATTITUDE_SIGMA_RAD * GEOMETRY_ATTITUDE_SIGMA_RAD;
    for axis in 3..6 {
        if let Some(cell) = covariance.get_mut((axis, axis)) {
            *cell = attitude_variance;
        }
    }
    Estimate {
        stamp: parts.stamp,
        observation_sha256: parts.observation_sha256,
        map: parts.map.clone(),
        frame: parts.frame,
        pose: parts.pose,
        quality: EstimateQuality {
            depth_matches: 0,
            inliers: 0,
            spatial_support: 0,
            reprojection_rms_px: 0.0,
            occupied_cells: 0,
            condition_number: 0.0,
        },
        geometry_covariance: covariance,
        query_ray_domain: None,
        backend: BACKEND.to_owned(),
    }
}

/// The camera's offset from the vehicle reference point, ECEF metres.
pub(crate) fn lever_arm_ecef(offset_body_m: DVec3, cg_body_m: DVec3, body_to_ecef: DQuat) -> DVec3 {
    body_to_ecef * (offset_body_m - cg_body_m)
}

/// A [`nalgebra`] vector from a `glam` one.
pub(crate) const fn to_vector(value: DVec3) -> Vector3<f64> {
    Vector3::new(value.x, value.y, value.z)
}
