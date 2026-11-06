//! The five virtual cameras of the active flight, on the body it flies over, with the
//! airport trackers standing at whichever database airport free flight is at.

use flightbag::RunwayEndSpec;
use glam::DVec3;
use sim_core::{VirtualCameraPlacement, VirtualCameraSpec};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_scenario::{ScenarioProfile, scenario_virtual_cameras};
use sim_scenery::{AirportRecord, CatalogVerticalDatum};

use super::SupervisorRig;

/// How far down the runway from the threshold the approach tripod stands, metres.
const APPROACH_TRIPOD_ALONG_M: f64 = 65.0;
/// How far outside the runway edge the approach tripod stands, metres.
const APPROACH_TRIPOD_CLEARANCE_M: f64 = 18.0;
/// Approach tripod height above the threshold, metres.
const APPROACH_TRIPOD_AGL_M: f64 = 2.8;

/// Cameras in every suite; [`SupervisorRig::cycle_virtual_camera`] visits them in order.
pub(super) const CAMERA_COUNT: usize = 5;

impl SupervisorRig {
    /// The body the active flight's cameras stand on and its selected camera: the
    /// scenario's suite, else the airport pattern suite at the free-flight airport —
    /// tower and approach trackers at that airport anywhere on Earth.
    #[must_use]
    pub fn active_virtual_camera_on_body(
        &self,
        profile: Option<ScenarioProfile>,
    ) -> (BodyEllipsoid, VirtualCameraSpec) {
        let diagram = self
            .diagram(self.effective_aerodrome(profile))
            .or_else(|| self.diagrams.first());
        let Some(diagram) = diagram else {
            return (BodyEllipsoid::WGS84, default_nose_camera());
        };
        let (body, mut specs) = scenario_virtual_cameras(profile, diagram, self.datum);
        if profile.is_none()
            && let Some(record) = self.active_airport_record(None)
            && record.osm_aerodrome().is_none()
        {
            let runway = self.cached_runway(None).copied();
            relocate_trackers(&mut specs, record, runway.as_ref(), self.datum);
        }
        let spec = specs
            .get(self.virtual_camera_index % CAMERA_COUNT)
            .copied()
            .unwrap_or_else(default_nose_camera);
        (body, spec)
    }
}

/// Moves the tower tracker to `record`'s tower and the approach tracker beside `runway`,
/// keeping their zoom and hand-over behaviour.
fn relocate_trackers(
    specs: &mut [VirtualCameraSpec],
    record: &AirportRecord,
    runway: Option<&RunwayEndSpec>,
    datum: CatalogVerticalDatum,
) {
    let tower = record.tower_viewpoint(datum);
    let approach = runway.map_or(tower, approach_tripod);
    for spec in specs {
        let at = match spec.id {
            "tower_ptz" => tower,
            "approach_flyby" => approach,
            _ => continue,
        };
        if let VirtualCameraPlacement::RangeLimitedAutoTrack { position, .. } = &mut spec.placement
        {
            *position = at;
        }
    }
}

/// A tripod down the runway from `runway`'s threshold, left of the pavement, from where
/// short final and the touchdown are both in view.
fn approach_tripod(runway: &RunwayEndSpec) -> Geodetic {
    let body = BodyEllipsoid::WGS84;
    let threshold = Geodetic::from_degrees(
        runway.threshold.latitude_deg,
        runway.threshold.longitude_deg,
        runway.threshold.ellipsoidal_height_m,
    );
    let frame = LocalFrame::at(&threshold);
    let (sin_b, cos_b) = runway.true_bearing_rad.sin_cos();
    let ahead = frame.north() * cos_b + frame.east() * sin_b;
    let right = frame.east() * cos_b - frame.north() * sin_b;
    let left_m = 0.5 * runway.width_m + APPROACH_TRIPOD_CLEARANCE_M;
    let tripod =
        body.geodetic_to_fixed(&threshold) + ahead * APPROACH_TRIPOD_ALONG_M - right * left_m;
    Geodetic {
        height_m: threshold.height_m + APPROACH_TRIPOD_AGL_M,
        ..body.fixed_to_geodetic(tripod)
    }
}

/// The nose camera, for a rig without any airport diagram.
fn default_nose_camera() -> VirtualCameraSpec {
    VirtualCameraSpec {
        id: "nose_nav",
        title: "Nose Nav Camera",
        placement: VirtualCameraPlacement::BodyMounted(sim_core::CameraMount::forward_nose(
            DVec3::new(1.25, 0.0, -0.15),
        )),
        base_intrinsics: VirtualCameraSpec::pinhole_800x600(53.0_f64.to_radians()),
        period: sim_core::CAMERA_DEFAULT_PERIOD,
        depth_readout: true,
    }
}
