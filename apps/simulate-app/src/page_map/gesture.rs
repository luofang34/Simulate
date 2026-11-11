//! A page map's pointer and wheel input as EFB commands, the way a tablet map takes it:
//! one finger or the mouse drags the map, two fingers pinch to zoom and twist to rotate,
//! a tap picks the fix under it, and the wheel zooms.
//!
//! The mapping only reads where pointers are, so a later editing mode can claim a drag
//! that starts on a fix before it becomes a pan.

use std::collections::BTreeMap;

use crate::map_rig::EfbCommand;

/// Furthest a pointer may travel, CSS pixels, and still tap.
const TAP_SLOP_CSS_PX: f64 = 6.0;
/// Wheel travel, CSS pixels, that zooms one level.
const WHEEL_PX_PER_ZOOM: f64 = 300.0;
/// Most zoom one wheel event may change, levels: a mouse's notch is a few hundred pixels.
const MAX_WHEEL_ZOOM: f64 = 1.0;

/// One pointer or wheel event on the map, CSS pixels from its top left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PointerInput {
    /// A pointer pressed.
    Down {
        /// The pointer.
        id: i32,
        /// Where.
        at: [f64; 2],
    },
    /// A pointer moved.
    Move {
        /// The pointer.
        id: i32,
        /// Where.
        at: [f64; 2],
    },
    /// A pointer lifted.
    Up {
        /// The pointer.
        id: i32,
        /// Where.
        at: [f64; 2],
    },
    /// A pointer left without lifting, as when the browser takes the gesture.
    Cancel {
        /// The pointer.
        id: i32,
    },
    /// The wheel turned by this many CSS pixels, positive scrolling down.
    Wheel {
        /// Vertical travel.
        delta_y: f64,
    },
}

/// The pointers on the map and what they have done since the first went down.
#[derive(Debug, Clone, Default)]
pub struct Gestures {
    pointers: BTreeMap<i32, [f64; 2]>,
    /// Where the first pointer went down, while it may still tap.
    tap_from: Option<(i32, [f64; 2])>,
}

impl Gestures {
    /// The commands `input` makes, the canvas drawing `ratio` buffer pixels per CSS pixel.
    pub fn handle(&mut self, input: PointerInput, ratio: f64) -> Vec<EfbCommand> {
        let px = |[x, y]: [f64; 2]| [(x * ratio) as f32, (y * ratio) as f32];
        match input {
            PointerInput::Down { id, at } => {
                self.tap_from = self.pointers.is_empty().then_some((id, at));
                self.pointers.insert(id, at);
                Vec::new()
            }
            PointerInput::Move { id, at } => self.moved(id, at, ratio),
            PointerInput::Up { id, at } => {
                self.pointers.remove(&id);
                match self.tap_from.take() {
                    Some((tap, from)) if tap == id && distance(from, at) <= TAP_SLOP_CSS_PX => {
                        vec![EfbCommand::Pick(px(at))]
                    }
                    _ => Vec::new(),
                }
            }
            PointerInput::Cancel { id } => {
                self.pointers.remove(&id);
                self.tap_from = None;
                Vec::new()
            }
            PointerInput::Wheel { delta_y } if delta_y.is_finite() && delta_y != 0.0 => {
                let levels = (-delta_y / WHEEL_PX_PER_ZOOM).clamp(-MAX_WHEEL_ZOOM, MAX_WHEEL_ZOOM);
                vec![EfbCommand::ZoomBy(levels)]
            }
            PointerInput::Wheel { .. } => Vec::new(),
        }
    }

    fn moved(&mut self, id: i32, at: [f64; 2], ratio: f64) -> Vec<EfbCommand> {
        let Some(mut last) = self.pointers.get(&id).copied() else {
            return Vec::new();
        };
        // A drag that leaves the tap's reach pans from where it began.
        if let Some((_, from)) = self
            .tap_from
            .filter(|(_, from)| distance(*from, at) > TAP_SLOP_CSS_PX)
        {
            self.tap_from = None;
            last = from;
        }
        let other = self
            .pointers
            .iter()
            .find(|(other, _)| **other != id)
            .map(|(_, at)| *at);
        self.pointers.insert(id, at);
        match other {
            None if self.tap_from.is_none() => {
                let [dx, dy] = [(last[0] - at[0]) * ratio, (last[1] - at[1]) * ratio];
                vec![EfbCommand::PanPixels([dx as f32, dy as f32])]
            }
            None => Vec::new(),
            Some(pivot) => twist_and_pinch(pivot, last, at),
        }
    }
}

/// Zoom and rotation of two fingers, one held at `pivot`, the other moved from `last` to
/// `at`: the map follows the fingers, so spreading them zooms in and turning them
/// counter-clockwise turns the map counter-clockwise.
fn twist_and_pinch(pivot: [f64; 2], last: [f64; 2], at: [f64; 2]) -> Vec<EfbCommand> {
    let (before, after) = (distance(pivot, last), distance(pivot, at));
    let mut commands = Vec::new();
    let zoom = (after / before).log2();
    if before > 1.0 && after > 1.0 && zoom.abs() > 1e-3 {
        commands.push(EfbCommand::ZoomBy(zoom));
    }
    let angle = |p: [f64; 2]| (p[1] - pivot[1]).atan2(p[0] - pivot[0]);
    let turn = (angle(at) - angle(last)).to_degrees();
    let turn = (turn + 180.0).rem_euclid(360.0) - 180.0;
    if turn.abs() > 0.05 {
        // Screen y grows downward, so a counter-clockwise turn decreases the angle.
        commands.push(EfbCommand::RotateBy(-turn));
    }
    commands
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
