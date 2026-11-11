//! Where the EFB map looks: display mode, follow mode, per-mode zoom with polar scale
//! compensation, and 3D unit-sphere panning across the poles (`±90°`).

use std::f64::consts::FRAC_PI_2;
use std::time::Duration;

use glam::DVec3;
use maplibre::projection::globe::scale;

use crate::{ChartCameraSpec, ChartOwnship, geodesy::wrap_lon_deg};

/// Default compact-corner zoom (about 18 NM of tactical awareness across the corner view).
pub const DEFAULT_COMPACT_ZOOM: f64 = 10.2;
/// Default expanded-EFB zoom.
pub const DEFAULT_EXPANDED_ZOOM: f64 = 11.6;
/// Lowest and highest EFB zoom.
pub const EFB_ZOOM_RANGE: (f64, f64) = (3.5, 18.0);

/// Below this ground speed the track is meaningless, so track-up follows the heading.
const MIN_TRACK_SPEED_MPS: f64 = 2.0;

/// Compensates globe style zoom poleward of `±60°` so ground metres-per-pixel stay matched
/// to the `60°` parallel scale all the way to `±90°`.
#[must_use]
pub(crate) fn effective_globe_zoom(zoom: f64, center_lat_deg: f64) -> f64 {
    if !zoom.is_finite() || !center_lat_deg.is_finite() || center_lat_deg.abs() <= 60.0 {
        return zoom;
    }
    let parallel_scale = center_lat_deg
        .abs()
        .min(scale::MERCATOR_LATITUDE_LIMIT)
        .to_radians()
        .cos();
    (zoom + (parallel_scale / 0.5).log2()).max(0.0)
}

/// Which of the EFB map's two viewports is shown.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EfbDisplayMode {
    /// Small moving map in a corner of the cockpit display.
    #[default]
    CompactCorner,
    /// Full electronic flight bag page.
    ExpandedEfb,
}

/// Monotonic wall-clock refresh policy for an [`EfbMapSurface`](crate::EfbMapSurface),
/// decoupling display pacing from simulation time warp, pause, or in-flight network I/O.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EfbRefreshPolicy {
    /// Adaptive cockpit pacing: the expanded EFB page draws arrived tiles immediately and
    /// follows ownship at `30 Hz` (`33 ms`), while the compact corner inset coalesces both
    /// ownship motion and streaming tile arrivals to `15 Hz` (`66 ms`).
    #[default]
    CockpitAdaptive,
    /// Standalone or dedicated tablet EFB pacing: arrived tiles and animations draw immediately
    /// (`0 ms`), and ownship/traffic updates pace at `30 Hz` (`33 ms`).
    StandaloneInteractive,
}

impl EfbRefreshPolicy {
    /// Whether arrived worker tiles or map animations bypass the motion interval in `mode`.
    #[must_use]
    pub const fn immediate_on_ready_work(self, mode: EfbDisplayMode) -> bool {
        match self {
            Self::StandaloneInteractive => true,
            Self::CockpitAdaptive => matches!(mode, EfbDisplayMode::ExpandedEfb),
        }
    }

    /// Wall-clock interval between coalesced motion (or inset tile-arrival) frames in `mode`.
    #[must_use]
    pub const fn motion_interval(self, mode: EfbDisplayMode) -> Duration {
        match (self, mode) {
            (Self::CockpitAdaptive, EfbDisplayMode::CompactCorner) => Duration::from_millis(66),
            (Self::CockpitAdaptive, EfbDisplayMode::ExpandedEfb)
            | (Self::StandaloneInteractive, _) => Duration::from_millis(33),
        }
    }
}

/// How the EFB map's camera follows ownship.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EfbFollowMode {
    /// Centred on ownship, rotated so the ground track points up.
    #[default]
    FollowTrackUp,
    /// Centred on ownship, north up.
    FollowNorthUp,
    /// Centred on ownship, rotated to a custom bearing set by a gesture.
    FollowBearing,
    /// Left where the pilot panned it; ownship may move off screen.
    FreePan,
}

/// Which EFB overlays are drawn.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EfbLayerFlags {
    /// Georeferenced approach plate.
    pub approach_plate: bool,
    /// OpenStreetMap airport surface diagram.
    pub airport_diagram: bool,
    /// Airspace boundaries.
    pub airspace: bool,
    /// Selected runway feather and traffic symbology.
    pub route_and_traffic: bool,
}

impl Default for EfbLayerFlags {
    fn default() -> Self {
        Self {
            approach_plate: true,
            airport_diagram: true,
            airspace: true,
            route_and_traffic: true,
        }
    }
}

/// The EFB camera's state across both display modes.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct EfbCamera {
    pub(super) mode: EfbDisplayMode,
    pub(super) follow: EfbFollowMode,
    /// The follow mode a recentre returns to.
    resume_follow: EfbFollowMode,
    /// Zoom of `[compact, expanded]`.
    zooms: [f64; 2],
    /// `[lat, lon]` of the centre, degrees.
    center: [f64; 2],
    bearing_deg: f64,
}

impl EfbCamera {
    pub(super) fn new(center_lat_deg: f64, center_lon_deg: f64) -> Self {
        Self {
            mode: EfbDisplayMode::default(),
            follow: EfbFollowMode::default(),
            resume_follow: EfbFollowMode::default(),
            zooms: [DEFAULT_COMPACT_ZOOM, DEFAULT_EXPANDED_ZOOM],
            center: sanitize_center(center_lat_deg, center_lon_deg),
            bearing_deg: 0.0,
        }
    }

    /// This camera, looking the same way, for `mode`'s view.
    pub(super) fn with_mode(&self, mode: EfbDisplayMode) -> Self {
        Self {
            mode,
            ..self.clone()
        }
    }

    /// This camera with the zoom `cameras[index]` keeps for its own view.
    pub(super) fn with_zoom_of(mut self, cameras: &[Self; 2], index: usize) -> Self {
        let kept = cameras
            .get(index)
            .and_then(|camera| camera.zooms.get(index))
            .copied();
        if let (Some(kept), Some(zoom)) = (kept, self.zooms.get_mut(index)) {
            *zoom = kept;
        }
        self
    }

    const fn mode_index(&self) -> usize {
        match self.mode {
            EfbDisplayMode::CompactCorner => 0,
            EfbDisplayMode::ExpandedEfb => 1,
        }
    }

    /// The shown mode's zoom.
    pub(super) fn zoom(&self) -> f64 {
        self.zooms
            .get(self.mode_index())
            .copied()
            .unwrap_or(DEFAULT_COMPACT_ZOOM)
    }

    /// Changes the shown mode's zoom by `delta`, within [`EFB_ZOOM_RANGE`].
    pub(super) fn zoom_by(&mut self, delta: f64) {
        let index = self.mode_index();
        if let Some(zoom) = self.zooms.get_mut(index)
            && delta.is_finite()
        {
            *zoom = (*zoom + delta).clamp(EFB_ZOOM_RANGE.0, EFB_ZOOM_RANGE.1);
        }
    }

    pub(super) fn set_follow(&mut self, follow: EfbFollowMode) {
        if follow != EfbFollowMode::FreePan {
            self.resume_follow = follow;
        }
        if follow == EfbFollowMode::FollowNorthUp {
            self.bearing_deg = 0.0;
        }
        self.follow = follow;
    }

    /// Returns to following ownship in the follow mode last chosen.
    pub(super) fn recenter(&mut self) {
        self.set_follow(self.resume_follow);
    }

    pub(super) const fn has_custom_bearing(&self) -> bool {
        matches!(self.follow, EfbFollowMode::FollowBearing)
            || (matches!(self.follow, EfbFollowMode::FreePan)
                && matches!(self.resume_follow, EfbFollowMode::FollowBearing))
    }

    /// Rotates the shown view's bearing by `delta_deg` (`> 0` rotates map features
    /// counter-clockwise, matching `winit::event::WindowEvent::RotationGesture`), keeping
    /// ownship centred unless already in [`EfbFollowMode::FreePan`].
    pub(super) fn rotate_by_deg(&mut self, delta_deg: f64) {
        if !delta_deg.is_finite() || delta_deg == 0.0 {
            return;
        }
        self.bearing_deg = (self.bearing_deg + delta_deg).rem_euclid(360.0);
        self.resume_follow = EfbFollowMode::FollowBearing;
        if self.follow != EfbFollowMode::FreePan {
            self.follow = EfbFollowMode::FollowBearing;
        }
    }

    /// Centres on (and, track-up, rotates with) ownship unless panned free.
    pub(super) fn follow_ownship(&mut self, ownship: Option<&ChartOwnship>) {
        let Some(own) = ownship.filter(|own| own.lat_deg.is_finite() && own.lon_deg.is_finite())
        else {
            return;
        };
        match self.follow {
            EfbFollowMode::FreePan => {}
            EfbFollowMode::FollowBearing => {
                self.center = sanitize_center(own.lat_deg, own.lon_deg);
            }
            EfbFollowMode::FollowNorthUp => {
                self.center = sanitize_center(own.lat_deg, own.lon_deg);
                self.bearing_deg = 0.0;
            }
            EfbFollowMode::FollowTrackUp => {
                self.center = sanitize_center(own.lat_deg, own.lon_deg);
                self.bearing_deg = track_deg(own);
            }
        }
    }

    /// Moves the centre `dx` right and `dy` down in screen pixels along a 3D unit-sphere
    /// great circle (parallel-transporting screen orientation across `±90°`), and stops
    /// following ownship.
    pub(super) fn pan_by_pixels(&mut self, dx_px: f64, dy_px: f64) {
        if !(dx_px.is_finite() && dy_px.is_finite()) || (dx_px == 0.0 && dy_px == 0.0) {
            return;
        }
        let px_dist = dx_px.hypot(dy_px);
        let [center_lat, center_lon] = self.center;
        let eff_zoom = effective_globe_zoom(self.zoom(), center_lat);
        let radius_px = scale::radius_pixels(eff_zoom, center_lat);
        if px_dist <= 0.0 || !radius_px.is_finite() || radius_px <= 0.0 {
            return;
        }
        let (lat, lon) = (center_lat.to_radians(), center_lon.to_radians());
        let (east, north, up) = sphere_basis(lat, lon);
        let (sin_b, cos_b) = self.bearing_deg.to_radians().sin_cos();
        let screen_up = north * cos_b + east * sin_b;
        let screen_right = east * cos_b - north * sin_b;
        let dir = (screen_right * dx_px - screen_up * dy_px) / px_dist;
        let angle = (px_dist / radius_px).min(FRAC_PI_2);
        let (sin_a, cos_a) = angle.sin_cos();
        let new_up = (up * cos_a + dir * sin_a).normalize();
        let transported = screen_up + dir.dot(screen_up) * ((cos_a - 1.0) * dir - sin_a * up);
        let new_screen_up = (transported - new_up * transported.dot(new_up)).normalize_or_zero();
        let new_lat = new_up.z.clamp(-1.0, 1.0).asin();
        let new_lon = if new_up.x.hypot(new_up.y) > 1e-12 {
            new_up.y.atan2(new_up.x)
        } else {
            lon
        };
        let (new_east, new_north, _) = sphere_basis(new_lat, new_lon);
        if new_screen_up.length_squared() > 0.5 {
            self.bearing_deg = new_screen_up
                .dot(new_east)
                .atan2(new_screen_up.dot(new_north))
                .to_degrees()
                .rem_euclid(360.0);
        }
        self.center = sanitize_center(new_lat.to_degrees(), new_lon.to_degrees());
        self.set_follow(EfbFollowMode::FreePan);
    }

    /// The chart camera of the shown mode.
    pub(super) fn spec(&self) -> ChartCameraSpec {
        let [lat, lon] = self.center;
        ChartCameraSpec {
            center_lat_deg: lat,
            center_lon_deg: lon,
            zoom: self.zoom(),
            bearing_deg: self.bearing_deg,
            pitch_deg: 0.0,
            surveillance_revision: 0,
            notices_revision: 0,
            symbology_revision: 0,
        }
    }
}

fn sanitize_center(lat_deg: f64, lon_deg: f64) -> [f64; 2] {
    let lat = if lat_deg.is_finite() {
        lat_deg.clamp(-90.0, 90.0)
    } else {
        0.0
    };
    let lon = if lon_deg.is_finite() {
        wrap_lon_deg(lon_deg)
    } else {
        0.0
    };
    [lat, lon]
}

fn sphere_basis(lat_rad: f64, lon_rad: f64) -> (DVec3, DVec3, DVec3) {
    let (sin_lat, cos_lat) = lat_rad.sin_cos();
    let (sin_lon, cos_lon) = lon_rad.sin_cos();
    let east = DVec3::new(-sin_lon, cos_lon, 0.0);
    let north = DVec3::new(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    let up = DVec3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);
    (east, north, up)
}

/// Ground track from the velocity, or the heading when nearly stopped.
fn track_deg(own: &ChartOwnship) -> f64 {
    let [east, north] = own.ground_velocity_en_mps;
    if east.hypot(north) >= MIN_TRACK_SPEED_MPS {
        east.atan2(north).to_degrees().rem_euclid(360.0)
    } else if own.true_heading_deg.is_finite() {
        own.true_heading_deg.rem_euclid(360.0)
    } else {
        0.0
    }
}
