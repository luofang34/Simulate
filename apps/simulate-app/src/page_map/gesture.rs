//! A page map's pointer and wheel input as EFB commands, the way a tablet map takes it:
//! one finger or the mouse drags the map, two fingers pinch to zoom and twist to rotate,
//! or, moved up or down together, tilt it; the right mouse button (or a drag with Shift)
//! tilts with up and down and turns with left and right; a tap picks the fix under it, and
//! the wheel zooms.
//!
//! The mapping only reads where pointers are, so a later editing mode can claim a drag
//! that starts on a fix before it becomes a pan.

use std::collections::BTreeMap;

use crate::map_rig::EfbCommand;

/// Furthest a pointer may travel, CSS pixels, and still tap.
const TAP_SLOP_CSS_PX: f64 = 6.0;
/// Wheel travel, CSS pixels, that zooms one level.
const WHEEL_PX_PER_ZOOM: f64 = 300.0;
/// Browser `ctrlKey` wheel travel, CSS pixels, that zooms one level on a trackpad pinch.
const TRACKPAD_PINCH_PX_PER_ZOOM: f64 = 50.0;
/// Most zoom one wheel event may change, levels: a mouse's notch is a few hundred pixels.
const MAX_WHEEL_ZOOM: f64 = 1.0;
/// Tilt per CSS pixel of a tilting drag, degrees: the whole range in about 150 pixels.
const TILT_DEG_PER_PX: f64 = 0.4;
/// Turn per CSS pixel of a tilting drag sideways, degrees.
const TURN_DEG_PER_PX: f64 = 0.3;
/// How far two fingers move before their gesture is read as a tilt or a pinch, CSS px.
const PAIR_SLOP_CSS_PX: f64 = 8.0;

/// One pointer or wheel event on the map, CSS pixels from its top left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PointerInput {
    /// A pointer pressed.
    Down {
        /// The pointer.
        id: i32,
        /// Where.
        at: [f64; 2],
        /// Whether its drag tilts and turns the map instead of panning it.
        tilts: bool,
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
    /// A browser two-finger trackpad pinch (`ctrlKey` wheel event), positive pinching closed.
    TrackpadPinch {
        /// Pinch delta in CSS pixels.
        delta_y: f64,
    },
}

/// The pointers on the map and what they have done since the first went down.
#[derive(Debug, Clone, Default)]
pub struct Gestures {
    pointers: BTreeMap<i32, [f64; 2]>,
    /// Where the first pointer went down, while it may still tap.
    tap_from: Option<(i32, [f64; 2])>,
    /// The single pointer's drag tilts and turns.
    tilting: bool,
    /// Two fingers' gesture, once read.
    pair: Option<Pair>,
}

/// Two fingers on the map: where they went down, and what their gesture was read as.
#[derive(Debug, Clone, Copy)]
struct Pair {
    start: [[f64; 2]; 2],
    read: Option<PairGesture>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PairGesture {
    /// Both move up or down together.
    Tilt,
    /// Anything else: they spread, close or turn.
    PinchTwist,
}

impl Gestures {
    /// The commands `input` makes, the canvas drawing `ratio` buffer pixels per CSS pixel.
    pub fn handle(&mut self, input: PointerInput, ratio: f64) -> Vec<EfbCommand> {
        let px = |[x, y]: [f64; 2]| [(x * ratio) as f32, (y * ratio) as f32];
        match input {
            PointerInput::Down { id, at, tilts } => {
                self.tap_from = self.pointers.is_empty().then_some((id, at));
                self.tilting = tilts && self.pointers.is_empty();
                self.pointers.insert(id, at);
                self.pair = self.start_of_pair().map(|start| Pair { start, read: None });
                Vec::new()
            }
            PointerInput::Move { id, at } => self.moved(id, at, ratio),
            PointerInput::Up { id, at } => {
                self.pointers.remove(&id);
                self.pair = None;
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
                self.pair = None;
                Vec::new()
            }
            PointerInput::Wheel { delta_y } if delta_y.is_finite() && delta_y != 0.0 => {
                let levels = (-delta_y / WHEEL_PX_PER_ZOOM).clamp(-MAX_WHEEL_ZOOM, MAX_WHEEL_ZOOM);
                vec![EfbCommand::ZoomBy(levels)]
            }
            PointerInput::TrackpadPinch { delta_y } if delta_y.is_finite() && delta_y != 0.0 => {
                let levels =
                    (-delta_y / TRACKPAD_PINCH_PX_PER_ZOOM).clamp(-MAX_WHEEL_ZOOM, MAX_WHEEL_ZOOM);
                vec![EfbCommand::ZoomBy(levels)]
            }
            PointerInput::Wheel { .. } | PointerInput::TrackpadPinch { .. } => Vec::new(),
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
            None if self.tap_from.is_none() && self.tilting => tilt_and_turn(last, at),
            None if self.tap_from.is_none() => {
                let [dx, dy] = [(last[0] - at[0]) * ratio, (last[1] - at[1]) * ratio];
                vec![EfbCommand::PanPixels([dx as f32, dy as f32])]
            }
            None => Vec::new(),
            Some(pivot) => match self.read_pair() {
                Some(PairGesture::Tilt) => {
                    // The fingers' midpoint moves half as far as the finger moved.
                    tilt_and_turn([0.0, last[1] / 2.0], [0.0, at[1] / 2.0])
                }
                Some(PairGesture::PinchTwist) => twist_and_pinch(pivot, last, at),
                None => Vec::new(),
            },
        }
    }

    /// Where two fingers on the map went down, when two are.
    fn start_of_pair(&self) -> Option<[[f64; 2]; 2]> {
        let mut at = self.pointers.values();
        match (at.next(), at.next(), at.next()) {
            (Some(a), Some(b), None) => Some([*a, *b]),
            _ => None,
        }
    }

    /// What two fingers' gesture is, once they moved far enough to tell.
    fn read_pair(&mut self) -> Option<PairGesture> {
        let pair = self.pair.as_mut()?;
        if pair.read.is_none() {
            let now = self.pointers.values().copied().collect::<Vec<_>>();
            let moved: Vec<[f64; 2]> = pair
                .start
                .iter()
                .zip(&now)
                .map(|(from, to)| [to[0] - from[0], to[1] - from[1]])
                .collect();
            let lengths: Vec<f64> = moved.iter().map(|[dx, dy]| dx.hypot(*dy)).collect();
            let most = lengths.iter().copied().fold(0.0, f64::max);
            let least = lengths.iter().copied().fold(f64::MAX, f64::min);
            let up_or_down = |sign: f64| moved.iter().all(|[dx, dy]| dy * sign > dx.abs() * 2.0);
            pair.read = if most < PAIR_SLOP_CSS_PX {
                None
            } else if least > PAIR_SLOP_CSS_PX / 2.0 {
                // Both have moved: together up or down tilts, anything else pinches.
                Some(if up_or_down(1.0) || up_or_down(-1.0) {
                    PairGesture::Tilt
                } else {
                    PairGesture::PinchTwist
                })
            } else if most > PAIR_SLOP_CSS_PX * 3.0 {
                // One moves while the other holds: a pinch or a twist about it.
                Some(PairGesture::PinchTwist)
            } else {
                None
            };
        }
        pair.read
    }
}

/// A tilting drag from `last` to `at`: up tilts toward the horizon, sideways turns.
fn tilt_and_turn(last: [f64; 2], at: [f64; 2]) -> Vec<EfbCommand> {
    let [dx, dy] = [at[0] - last[0], at[1] - last[1]];
    let mut commands = Vec::new();
    if dy != 0.0 {
        commands.push(EfbCommand::PitchBy(-dy * TILT_DEG_PER_PX));
    }
    if dx != 0.0 {
        commands.push(EfbCommand::RotateBy(-dx * TURN_DEG_PER_PX));
    }
    commands
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
