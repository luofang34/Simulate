//! Canvas multi-touch routing (`WindowEvent::Touch`): single-finger look drag (with
//! double-tap reset), EFB/3D picking, tap/scroll on the Flight Hub, and two-finger
//! pinch/twist gestures on EFB maps and Hub 3D previews.

use std::f64::consts::PI;

use web_time::Instant;
use winit::event::{Touch, TouchPhase};
use winit::event_loop::ActiveEventLoop;

use super::FlightApp;
use crate::esc_menu::MenuInput;

const HUB_SCROLL_SLOP_PX: f64 = 10.0;
const HUB_SCROLL_ROW_PX: f64 = 28.0;
const LOOK_TAP_SLOP_PX: f64 = 6.0;
const DOUBLE_TAP_WINDOW_S: f64 = 0.35;
const DOUBLE_TAP_DIST_PX: f64 = 32.0;
const MIN_PINCH_SEP_PX: f64 = 8.0;
const LOOK_FULL_SWIPE_RAD: f64 = 2.0;

#[derive(Debug, Clone, Copy, PartialEq)]
struct ActiveTouch {
    id: u64,
    pos: [f64; 2],
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
enum SingleTouchMode {
    #[default]
    Idle,
    Delegated,
    HubPending {
        origin: [f64; 2],
        last_y: f64,
        accum_px: f64,
        scrolled: bool,
    },
    LookDrag {
        origin: [f64; 2],
        last: [f64; 2],
        moved: bool,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct TwoFingerGesture {
    last_dist: f64,
    last_angle_rad: f64,
}

/// Active fingers and gesture state for `WindowEvent::Touch`.
#[derive(Debug, Default)]
pub(super) struct TouchTracker {
    active: Vec<ActiveTouch>,
    single: SingleTouchMode,
    two: Option<TwoFingerGesture>,
    last_tap: Option<(Instant, [f64; 2])>,
    manual_look: bool,
}

impl TouchTracker {
    /// Whether the pilot manually dragged the view via touch and has not re-centered yet.
    #[must_use]
    pub(super) const fn is_manual_look(&self) -> bool {
        self.manual_look
    }

    /// Clears the manual-look latch so right-stick spring-centering resumes.
    pub(super) fn clear_manual_look(&mut self) {
        self.manual_look = false;
    }

    /// Clears any active drag or pinch state (for example when opening the Flight Hub).
    pub(super) fn cancel_active(&mut self) {
        self.active.clear();
        self.single = SingleTouchMode::Idle;
        self.two = None;
    }

    fn upsert(&mut self, id: u64, pos: [f64; 2]) {
        if let Some(existing) = self.active.iter_mut().find(|t| t.id == id) {
            existing.pos = pos;
            return;
        }
        if self.active.len() >= 8 {
            self.active.remove(0);
        }
        self.active.push(ActiveTouch { id, pos });
    }

    fn pair(&self) -> Option<([f64; 2], [f64; 2])> {
        let first = self.active.first()?;
        let second = self.active.get(1)?;
        Some((first.pos, second.pos))
    }
}

/// Midpoint, Euclidean separation (pixels), and chord angle (radians) of two touch points.
#[must_use]
pub(super) fn two_finger_metrics(a: [f64; 2], b: [f64; 2]) -> ([f64; 2], f64, f64) {
    let mid = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    (mid, dx.hypot(dy), dy.atan2(dx))
}

/// Converts previous and current finger separations into `pinch_gesture` magnification delta.
#[must_use]
pub(super) fn pinch_delta_from_distances(prev: f64, curr: f64) -> Option<f64> {
    if !(prev.is_finite() && curr.is_finite() && prev >= MIN_PINCH_SEP_PX) {
        return None;
    }
    let ratio_delta = (curr / prev) - 1.0;
    (ratio_delta.abs() > 1e-4).then(|| ratio_delta.clamp(-0.75, 1.5))
}

/// Shortest signed rotation delta between two chord angles in degrees.
#[must_use]
pub(super) fn twist_delta_deg(prev_rad: f64, curr_rad: f64) -> f64 {
    if !(prev_rad.is_finite() && curr_rad.is_finite()) {
        return 0.0;
    }
    let mut delta = curr_rad - prev_rad;
    while delta > PI {
        delta -= 2.0 * PI;
    }
    while delta < -PI {
        delta += 2.0 * PI;
    }
    delta.to_degrees()
}

/// Converts a finger drag delta into `[dyaw_rad, dpitch_rad]` look angles.
#[must_use]
pub(super) fn look_delta_rad(delta_px: [f64; 2], surface_px: [u32; 2]) -> [f64; 2] {
    let min_edge = f64::from(surface_px[0].min(surface_px[1]).max(1));
    let dx = if delta_px[0].is_finite() {
        delta_px[0]
    } else {
        0.0
    };
    let dy = if delta_px[1].is_finite() {
        delta_px[1]
    } else {
        0.0
    };
    [
        -(dx / min_edge) * LOOK_FULL_SWIPE_RAD,
        -(dy / min_edge) * LOOK_FULL_SWIPE_RAD,
    ]
}

impl FlightApp {
    /// Routes a `winit::event::Touch` through two-finger zoom/twist, Hub tap/scroll/drag,
    /// EFB tap/pan, or single-finger cockpit/chase camera look.
    pub(super) fn touch_event(&mut self, event_loop: &ActiveEventLoop, touch: Touch) {
        let pos = self.window_pos_to_surface([touch.location.x, touch.location.y]);
        match touch.phase {
            TouchPhase::Started => self.touch_started(event_loop, touch.id, pos),
            TouchPhase::Moved => self.touch_moved(event_loop, touch.id, pos),
            TouchPhase::Ended | TouchPhase::Cancelled => {
                self.touch_ended(
                    event_loop,
                    touch.id,
                    pos,
                    touch.phase == TouchPhase::Cancelled,
                );
            }
        }
    }

    fn touch_started(&mut self, event_loop: &ActiveEventLoop, id: u64, pos: [f64; 2]) {
        self.touch.upsert(id, pos);
        if let Some((a, b)) = self.touch.pair() {
            self.mouse_released();
            self.touch.single = SingleTouchMode::Idle;
            let (mid, dist, angle) = two_finger_metrics(a, b);
            self.cursor_px = mid;
            self.touch.two = Some(TwoFingerGesture {
                last_dist: dist,
                last_angle_rad: angle,
            });
            return;
        }
        self.cursor_px = pos;
        if self.esc_menu.is_open() {
            if self.cursor_in_preview() {
                self.mouse_moved(pos);
                self.mouse_pressed(event_loop);
                self.touch.single = SingleTouchMode::Delegated;
            } else {
                self.touch.single = SingleTouchMode::HubPending {
                    origin: pos,
                    last_y: pos[1],
                    accum_px: 0.0,
                    scrolled: false,
                };
            }
            return;
        }
        self.mouse_moved(pos);
        if self.efb_pressed() {
            self.touch.single = SingleTouchMode::Delegated;
        } else if self.efb_workspace.ad_hoc.pick_mode.is_world_3d() {
            self.world_3d_pick_pressed();
            self.touch.single = SingleTouchMode::Delegated;
        } else {
            self.touch.single = SingleTouchMode::LookDrag {
                origin: pos,
                last: pos,
                moved: false,
            };
        }
    }

    fn touch_moved(&mut self, event_loop: &ActiveEventLoop, id: u64, pos: [f64; 2]) {
        self.touch.upsert(id, pos);
        if let Some((a, b)) = self.touch.pair() {
            self.touch_two_moved(a, b);
            return;
        }
        match self.touch.single {
            SingleTouchMode::Idle => {}
            SingleTouchMode::Delegated => self.mouse_moved(pos),
            SingleTouchMode::HubPending {
                origin,
                last_y,
                mut accum_px,
                mut scrolled,
            } => {
                if (pos[0] - origin[0]).hypot(pos[1] - origin[1]) > HUB_SCROLL_SLOP_PX {
                    scrolled = true;
                }
                accum_px += last_y - pos[1];
                while accum_px >= HUB_SCROLL_ROW_PX {
                    accum_px -= HUB_SCROLL_ROW_PX;
                    self.handle_menu_input(event_loop, MenuInput::Scroll(1));
                }
                while accum_px <= -HUB_SCROLL_ROW_PX {
                    accum_px += HUB_SCROLL_ROW_PX;
                    self.handle_menu_input(event_loop, MenuInput::Scroll(-1));
                }
                self.touch.single = SingleTouchMode::HubPending {
                    origin,
                    last_y: pos[1],
                    accum_px,
                    scrolled,
                };
            }
            SingleTouchMode::LookDrag {
                origin,
                last,
                mut moved,
            } => {
                if (pos[0] - origin[0]).hypot(pos[1] - origin[1]) > LOOK_TAP_SLOP_PX {
                    moved = true;
                }
                let [dyaw, dpitch] =
                    look_delta_rad([pos[0] - last[0], pos[1] - last[1]], self.surface_size());
                if dyaw != 0.0 || dpitch != 0.0 {
                    self.look.nudge(dyaw, dpitch);
                    self.touch.manual_look = true;
                }
                self.touch.single = SingleTouchMode::LookDrag {
                    origin,
                    last: pos,
                    moved,
                };
            }
        }
    }

    fn touch_two_moved(&mut self, a: [f64; 2], b: [f64; 2]) {
        let (mid, dist, angle) = two_finger_metrics(a, b);
        self.cursor_px = mid;
        let Some(prev) = self.touch.two else {
            self.touch.two = Some(TwoFingerGesture {
                last_dist: dist,
                last_angle_rad: angle,
            });
            return;
        };
        if let Some(delta) = pinch_delta_from_distances(prev.last_dist, dist) {
            self.pinch_gesture(delta);
        }
        let deg = twist_delta_deg(prev.last_angle_rad, angle);
        if deg.abs() > 0.1 {
            self.rotation_gesture(deg as f32);
        }
        self.touch.two = Some(TwoFingerGesture {
            last_dist: dist,
            last_angle_rad: angle,
        });
    }

    fn touch_ended(
        &mut self,
        event_loop: &ActiveEventLoop,
        id: u64,
        pos: [f64; 2],
        cancelled: bool,
    ) {
        self.touch.active.retain(|t| t.id != id);
        if self.touch.active.len() < 2 {
            self.touch.two = None;
        }
        if !self.touch.active.is_empty() {
            return;
        }
        match std::mem::take(&mut self.touch.single) {
            SingleTouchMode::Idle => {}
            SingleTouchMode::Delegated => {
                self.mouse_moved(pos);
                self.mouse_released();
            }
            SingleTouchMode::HubPending {
                origin, scrolled, ..
            } if !cancelled && !scrolled => {
                self.mouse_moved(origin);
                self.mouse_pressed(event_loop);
                self.mouse_released();
            }
            SingleTouchMode::LookDrag {
                origin,
                moved: false,
                ..
            } if !cancelled => {
                let now = Instant::now();
                let is_double = self.touch.last_tap.is_some_and(|(t, prev_pos)| {
                    now.duration_since(t).as_secs_f64() <= DOUBLE_TAP_WINDOW_S
                        && (origin[0] - prev_pos[0]).hypot(origin[1] - prev_pos[1])
                            <= DOUBLE_TAP_DIST_PX
                });
                if is_double {
                    self.look.reset();
                    self.touch.manual_look = false;
                    self.touch.last_tap = None;
                } else {
                    self.touch.last_tap = Some((now, origin));
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{look_delta_rad, pinch_delta_from_distances, twist_delta_deg, two_finger_metrics};

    #[test]
    fn two_finger_metrics_and_pinch_twist_behave_continuously() {
        let (mid, dist, angle) = two_finger_metrics([100.0, 200.0], [300.0, 200.0]);
        assert_eq!(mid, [200.0, 200.0]);
        assert!((dist - 200.0).abs() < 1e-9);
        assert!(angle.abs() < 1e-9);

        assert_eq!(pinch_delta_from_distances(4.0, 20.0), None);
        assert!((pinch_delta_from_distances(100.0, 150.0).unwrap() - 0.5).abs() < 1e-9);
        assert!(
            (twist_delta_deg(3.0, -3.0) - (2.0 * std::f64::consts::PI - 6.0).to_degrees()).abs()
                < 1e-6
        );

        let look = look_delta_rad([100.0, -50.0], [800, 400]);
        assert_eq!(look, [-0.5, 0.25]);
    }
}
