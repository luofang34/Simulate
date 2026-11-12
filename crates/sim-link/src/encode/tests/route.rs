//! Routes drawn at random: every path terminator where it may stand, each
//! with the keys it needs and some it may carry, in every unit.

use super::Draws;
use crate::{
    Bearing, Distance, DistanceUnit, Duration, Fix, Height, HeightUnit, HoldLength, Key, Leg,
    Limit, Micro, Milli, North, PathTerminator, Role, SecondAltitude, SecondAltitudeKind, Speed,
    SpeedUnit, TimeUnit, TurnDirection, distance_from_navaid, shape,
};

impl Draws {
    fn pick<T: Copy>(&mut self, choices: &[T]) -> T {
        let index = self.within(0, choices.len() as i64 - 1) as usize;
        choices[index]
    }
}

/// 2 to 12 legs: an `IF` or `DF` first, any leg but `IF` in the middle,
/// any leg that needs none to follow it last.
pub(super) fn route(draws: &mut Draws) -> Vec<Leg> {
    let count = draws.within(2, 12) as usize;
    let middle: Vec<PathTerminator> = PathTerminator::ALL
        .into_iter()
        .filter(|path| *path != PathTerminator::If)
        .collect();
    let last: Vec<PathTerminator> = PathTerminator::ALL
        .into_iter()
        .filter(|path| *path != PathTerminator::If && !path.intercepts_next())
        .collect();
    (0..count)
        .map(|index| {
            let path = match index {
                0 => draws.pick(&[PathTerminator::If, PathTerminator::Df]),
                _ if index + 1 == count => draws.pick(&last),
                _ => draws.pick(&middle),
            };
            leg(draws, path)
        })
        .collect()
}

fn leg(draws: &mut Draws, path: PathTerminator) -> Leg {
    let shape = shape(path);
    let mut leg = Leg::new(path, shape.fix.then(|| fix(draws)));
    for key in Key::ALL {
        let wanted =
            shape.required.contains(&key) || (shape.allows(key) && draws.within(0, 3) == 0);
        if wanted {
            set(draws, &mut leg, key);
        }
    }
    let measured = leg.theta.is_some() || (leg.distance.is_some() && distance_from_navaid(path));
    if measured && leg.navaid.is_none() {
        leg.navaid = Some(fix(draws));
    }
    leg
}

/// A fix named from a few names, so names repeat at the same and at other
/// places.
fn fix(draws: &mut Draws) -> Fix {
    let name = draws.pick(&[None, Some("A"), Some("B"), Some("NAV-1"), Some("c_2")]);
    let places = [
        (41_500_000, 2_000_000),
        (-33_861_234, 151_210_987),
        (0, -180_000_000),
    ];
    let (lat, lon) = if draws.flag() {
        draws.pick(&places)
    } else {
        (
            draws.within(-90_000_000, 90_000_000),
            draws.within(-180_000_000, 180_000_000),
        )
    };
    Fix {
        name: name.map(str::to_owned),
        lat: Micro::from_scaled(lat),
        lon: Micro::from_scaled(lon),
    }
}

fn set(draws: &mut Draws, leg: &mut Leg, key: Key) {
    match key {
        Key::O => leg.fly_over = true,
        Key::M => leg.missed_approach = true,
        Key::R => leg.roles = roles(draws),
        Key::T => leg.turn = Some(draws.pick(&[TurnDirection::Left, TurnDirection::Right])),
        Key::A => leg.altitude = Some(limit(draws, height)),
        Key::G => {
            leg.second_altitude = Some(SecondAltitude {
                kind: draws.pick(&SecondAltitudeKind::ALL),
                height: height(draws),
            });
        }
        Key::V => leg.vertical_angle = Some(Milli::from_scaled(draws.within(-90_000, 90_000))),
        Key::X => {
            leg.crossing_height = Some(Height {
                value: Milli::from_scaled(draws.within(0, 300_000)),
                unit: draws.pick(&[HeightUnit::Metre, HeightUnit::Foot]),
            });
        }
        Key::S => leg.speed = Some(limit(draws, speed)),
        Key::C => leg.course = Some(bearing(draws)),
        Key::H => leg.heading = Some(bearing(draws)),
        Key::D => leg.distance = Some(distance(draws)),
        Key::B => leg.theta = Some(bearing(draws)),
        Key::N => leg.navaid = Some(fix(draws)),
        Key::Z => leg.centre = Some(fix(draws)),
        Key::L => leg.hold_length = Some(hold_length(draws)),
        Key::P => leg.rnp = Some(distance(draws)),
    }
}

fn roles(draws: &mut Draws) -> Vec<Role> {
    let mut roles = Vec::new();
    for _ in 0..draws.within(1, 3) {
        let role = draws.pick(&Role::ALL);
        if !roles.contains(&role) {
            roles.push(role);
        }
    }
    roles
}

/// One of the four forms; a range's ends are drawn apart.
fn limit<T: Copy + Ordered>(draws: &mut Draws, value: fn(&mut Draws) -> T) -> Limit<T> {
    match draws.within(0, 3) {
        0 => Limit::At(value(draws)),
        1 => Limit::AtOrAbove(value(draws)),
        2 => Limit::AtOrBelow(value(draws)),
        _ => loop {
            let (lower, upper) = (value(draws), value(draws));
            if lower.scale() < upper.scale() {
                break Limit::Between { lower, upper };
            }
        },
    }
}

/// A value on one scale, to draw a range.
trait Ordered {
    fn scale(&self) -> f64;
}

impl Ordered for Height {
    fn scale(&self) -> f64 {
        self.metres()
    }
}

impl Ordered for Speed {
    fn scale(&self) -> f64 {
        self.metres_per_second()
    }
}

fn height(draws: &mut Draws) -> Height {
    match draws.within(0, 2) {
        0 => Height {
            value: Milli::from_scaled(draws.within(-500_000, 30_000_000)),
            unit: HeightUnit::Metre,
        },
        1 => Height {
            value: Milli::from_scaled(draws.within(-1_640_000, 98_425_000)),
            unit: HeightUnit::Foot,
        },
        _ => Height {
            value: Milli::from_whole(draws.within(0, 984)),
            unit: HeightUnit::FlightLevel,
        },
    }
}

fn speed(draws: &mut Draws) -> Speed {
    if draws.flag() {
        Speed {
            value: Milli::from_scaled(draws.within(1, 350_000)),
            unit: SpeedUnit::MetrePerSecond,
        }
    } else {
        Speed {
            value: Milli::from_scaled(draws.within(1, 680_000)),
            unit: SpeedUnit::Knot,
        }
    }
}

fn bearing(draws: &mut Draws) -> Bearing {
    Bearing {
        degrees: Milli::from_scaled(draws.within(0, 359_999)),
        north: draws.pick(&[North::True, North::Magnetic]),
    }
}

fn distance(draws: &mut Draws) -> Distance {
    if draws.flag() {
        Distance {
            value: Milli::from_scaled(draws.within(1, 2_000_000_000)),
            unit: DistanceUnit::Metre,
        }
    } else {
        Distance {
            value: Milli::from_scaled(draws.within(1, 1_000_000)),
            unit: DistanceUnit::NauticalMile,
        }
    }
}

fn hold_length(draws: &mut Draws) -> HoldLength {
    match draws.within(0, 2) {
        0 => HoldLength::Time(Duration {
            value: Milli::from_scaled(draws.within(1, 3_600_000)),
            unit: TimeUnit::Second,
        }),
        1 => HoldLength::Time(Duration {
            value: Milli::from_scaled(draws.within(1, 60_000)),
            unit: TimeUnit::Minute,
        }),
        _ => HoldLength::Distance(distance(draws)),
    }
}
