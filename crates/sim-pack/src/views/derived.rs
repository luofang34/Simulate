//! Views for a package that declares none, scaled from its wingspan so a
//! glider, a light single and a quadcopter each fill the frame.

use super::{CameraView, CameraViewSet, SCHEMA_VERSION, ViewAirframe, ViewPlacement};

/// Span assumed when a package states none, metres (a light single).
pub const DEFAULT_SPAN_M: f64 = 11.0;
/// The chase camera's distance behind and above the origin per metre of
/// span: a light single's `22 m` back and `6.5 m` up.
const CHASE_BACK_PER_SPAN: f64 = 2.0;
const CHASE_UP_PER_SPAN: f64 = 0.59;
/// Closest any derived external camera stands to the origin, metres, so a
/// small drone's views keep some of the ground in frame.
const MIN_STANDOFF_M: f64 = 4.0;

/// Chase, wingman, flyby, profile, planform, spotter and cockpit views for a
/// vehicle `span_m` across; a non-positive span reads as [`DEFAULT_SPAN_M`].
pub(super) fn views_for_span(span_m: f64) -> CameraViewSet {
    let span = if span_m.is_finite() && span_m > 0.0 {
        span_m
    } else {
        DEFAULT_SPAN_M
    };
    let mut views = formation_views(span);
    views.extend(observer_views(span));
    views.push(view(
        "cockpit_forward",
        "Cockpit: Forward",
        45.0,
        ViewAirframe::Hidden,
        ViewPlacement::CockpitLook {
            yaw_deg: 0.0,
            pitch_deg: -3.0,
            eye_shift_frd_m: [0.0; 3],
        },
    ));
    CameraViewSet {
        schema_version: SCHEMA_VERSION,
        views,
    }
}

fn at_least(m: f64) -> f64 {
    m.max(MIN_STANDOFF_M)
}

fn view(
    id: &str,
    title: &str,
    fov_y_deg: f64,
    airframe: ViewAirframe,
    placement: ViewPlacement,
) -> CameraView {
    CameraView {
        id: id.to_owned(),
        title: title.to_owned(),
        fov_y_deg,
        airframe,
        placement,
    }
}

/// Views that ride with the vehicle: chase, wingman, profile and planform.
fn formation_views(span: f64) -> Vec<CameraView> {
    let external = ViewAirframe::External;
    let look_at = |eye_frd_m, up_frd| ViewPlacement::LookAt {
        eye_frd_m,
        target_frd_m: [0.0; 3],
        up_frd,
    };
    let body_up = [0.0, 0.0, -1.0];
    vec![
        view(
            "chase",
            "Chase",
            55.0,
            external,
            ViewPlacement::Chase {
                back_m: at_least(CHASE_BACK_PER_SPAN * span),
                up_m: (CHASE_UP_PER_SPAN * span).max(0.3 * MIN_STANDOFF_M),
            },
        ),
        view(
            "wingman",
            "Wingman (3/4 rear)",
            40.0,
            external,
            look_at(
                [-at_least(0.9 * span), at_least(0.7 * span), -0.25 * span],
                body_up,
            ),
        ),
        view(
            "side_profile",
            "Side Profile",
            35.0,
            external,
            look_at([0.0, -at_least(1.6 * span), 0.0], body_up),
        ),
        view(
            "top_down",
            "Top-Down",
            35.0,
            external,
            look_at([0.0, 0.0, -at_least(1.9 * span)], [1.0, 0.0, 0.0]),
        ),
    ]
}

/// Observers that stand beside the track and watch the vehicle pass.
fn observer_views(span: f64) -> [CameraView; 2] {
    [
        view(
            "flyby_front",
            "Low Front-Quarter Flyby",
            50.0,
            ViewAirframe::External,
            ViewPlacement::Spotter {
                grid_step_m: at_least(60.0 * span),
                lateral_m: at_least(3.0 * span),
                height_m: 1.0,
                lead_time_s: 0.0,
                frame_span_m: 1.6 * span,
            },
        ),
        view(
            "ground_spotter",
            "Ground Spotter",
            45.0,
            ViewAirframe::External,
            ViewPlacement::Spotter {
                grid_step_m: at_least(150.0 * span),
                lateral_m: -at_least(25.0 * span),
                height_m: 2.0,
                lead_time_s: 0.0,
                frame_span_m: 2.0 * span,
            },
        ),
    ]
}
