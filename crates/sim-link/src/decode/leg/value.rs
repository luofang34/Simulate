//! The value of each leg attribute: heights, speeds, distances, times and
//! bearings with their units, constraints, roles and fixes.

use super::super::number::{Bounds, out_of_range, ranged, whole};
use super::super::token::{Site, Token};
use super::super::word;
use super::{Names, fix};
use crate::{
    Bearing, Distance, DistanceUnit, Duration, Height, HeightUnit, HoldLength, Key, Leg, Limit,
    Milli, North, Reason, Refusal, Role, SecondAltitude, SecondAltitudeKind, Speed, SpeedUnit,
    TimeUnit, TurnDirection,
};

/// Reads the value of `key` into `leg`.
pub(super) fn apply(
    site: Site,
    key: Key,
    value: Token<'_>,
    leg: &mut Leg,
    names: &mut Names,
) -> Result<(), Refusal> {
    match key {
        Key::O => leg.fly_over = flag(site, value, "o")?,
        Key::M => leg.missed_approach = flag(site, value, "m")?,
        Key::R => leg.roles = roles(site, value)?,
        Key::T => leg.turn = Some(turn(site, value)?),
        Key::A => {
            leg.altitude = Some(limit(
                site,
                value,
                |token| height(site, token, true),
                Height::metres,
            )?)
        }
        Key::G => leg.second_altitude = Some(second_altitude(site, value)?),
        Key::V => leg.vertical_angle = Some(vertical_angle(site, value)?),
        Key::X => leg.crossing_height = Some(height(site, value, false)?),
        Key::S => {
            leg.speed = Some(limit(
                site,
                value,
                |token| speed(site, token),
                Speed::metres_per_second,
            )?)
        }
        Key::C => leg.course = Some(bearing(site, value)?),
        Key::H => leg.heading = Some(bearing(site, value)?),
        Key::D => leg.distance = Some(distance(site, value)?),
        Key::B => leg.theta = Some(bearing(site, value)?),
        Key::N => leg.navaid = Some(fix(site, value, names)?),
        Key::Z => leg.centre = Some(fix(site, value, names)?),
        Key::L => leg.hold_length = Some(hold_length(site, value)?),
        Key::P => leg.rnp = Some(distance(site, value)?),
    }
    Ok(())
}

fn malformed(site: Site, token: Token<'_>, expected: &'static str) -> Refusal {
    site.refuse(
        token.offset,
        Reason::Malformed {
            text: token.text.to_owned(),
            expected,
        },
    )
}

/// A key that says everything by being there.
fn flag(site: Site, value: Token<'_>, expected: &'static str) -> Result<bool, Refusal> {
    if value.text.is_empty() {
        return Ok(true);
    }
    Err(malformed(site, value, expected))
}

fn roles(site: Site, value: Token<'_>) -> Result<Vec<Role>, Refusal> {
    value.non_empty(site, "roles joined by .")?;
    let mut roles = Vec::new();
    for token in value.split('.') {
        let role = word(
            site,
            token,
            "role",
            "IAF IF FACF FAF SDF MAP MAHF FEP RWY",
            &Role::ALL.map(|role| (role.word(), role)),
        )?;
        if roles.contains(&role) {
            return Err(malformed(site, token, "roles named once each"));
        }
        roles.push(role);
    }
    Ok(roles)
}

fn turn(site: Site, value: Token<'_>) -> Result<TurnDirection, Refusal> {
    word(
        site,
        value,
        "turn direction",
        "L, R",
        &[("L", TurnDirection::Left), ("R", TurnDirection::Right)],
    )
}

/// `value`, `value+`, `value-` or `lower..upper`; `order` puts two values
/// on one scale to check a range.
fn limit<T: Copy>(
    site: Site,
    token: Token<'_>,
    read: impl Fn(Token<'_>) -> Result<T, Refusal>,
    order: impl Fn(T) -> f64,
) -> Result<Limit<T>, Refusal> {
    token.non_empty(site, "a value, value+, value- or lower..upper")?;
    if let Some((lower, upper)) = token.split_once_text("..") {
        let (lower, upper) = (read(lower)?, read(upper)?);
        if order(lower) >= order(upper) {
            return Err(site.refuse(
                token.offset,
                Reason::ReversedLimit {
                    text: token.text.to_owned(),
                },
            ));
        }
        return Ok(Limit::Between { lower, upper });
    }
    if let Some(value) = token.strip_suffix("+") {
        return read(value).map(Limit::AtOrAbove);
    }
    if let Some(value) = token.strip_suffix("-") {
        return read(value).map(Limit::AtOrBelow);
    }
    read(token).map(Limit::At)
}

/// `1500` metres, `5000ft`, or, where `flight_level`, `FL120`.
fn height(site: Site, token: Token<'_>, flight_level: bool) -> Result<Height, Refusal> {
    token.non_empty(site, "a height: 1500 (m), 5000ft or FL120")?;
    if let Some(level) = token.strip_prefix("FL") {
        if !flight_level {
            return Err(malformed(site, token, "a height in metres or ft"));
        }
        let value = whole(site, level)?
            .filter(|level| *level <= 984)
            .and_then(|level| i64::try_from(level).ok())
            .ok_or_else(|| out_of_range(site, token, "flight level", "FL0 to FL984"))?;
        return Ok(Height {
            value: Milli::from_whole(value),
            unit: HeightUnit::FlightLevel,
        });
    }
    if let Some(feet) = token.strip_suffix("ft") {
        let value = ranged::<3>(
            site,
            feet,
            &bounds("height", -1_640, 98_425, "-1640 to 98425 ft"),
        )?;
        return Ok(Height {
            value,
            unit: HeightUnit::Foot,
        });
    }
    let value = ranged::<3>(
        site,
        token,
        &bounds("height", -500, 30_000, "-500 to 30000 m"),
    )?;
    Ok(Height {
        value,
        unit: HeightUnit::Metre,
    })
}

const fn bounds(what: &'static str, min: i64, max: i64, range: &'static str) -> Bounds {
    Bounds {
        what,
        min,
        max,
        range,
    }
}

/// A number inside `0 < value <= max`.
fn positive(
    site: Site,
    token: Token<'_>,
    what: &'static str,
    max: i64,
    range: &'static str,
) -> Result<Milli, Refusal> {
    let value = ranged::<3>(site, token, &bounds(what, 0, max, range))?;
    if value <= Milli::from_whole(0) {
        return Err(out_of_range(site, token, what, range));
    }
    Ok(value)
}

/// `60` m/s or `120kt`.
fn speed(site: Site, token: Token<'_>) -> Result<Speed, Refusal> {
    token.non_empty(site, "a speed: 60 (m/s) or 120kt")?;
    if let Some(knots) = token.strip_suffix("kt") {
        let value = positive(site, knots, "speed", 680, "more than 0 up to 680 kt")?;
        return Ok(Speed {
            value,
            unit: SpeedUnit::Knot,
        });
    }
    let value = positive(site, token, "speed", 350, "more than 0 up to 350 m/s")?;
    Ok(Speed {
        value,
        unit: SpeedUnit::MetrePerSecond,
    })
}

/// `7400` metres or `4nm`.
fn distance(site: Site, token: Token<'_>) -> Result<Distance, Refusal> {
    token.non_empty(site, "a distance: 7400 (m) or 4nm")?;
    if let Some(miles) = token.strip_suffix("nm") {
        let value = positive(site, miles, "distance", 540, "more than 0 up to 540 nm")?;
        return Ok(Distance {
            value,
            unit: DistanceUnit::NauticalMile,
        });
    }
    let value = positive(
        site,
        token,
        "distance",
        1_000_000,
        "more than 0 up to 1000000 m",
    )?;
    Ok(Distance {
        value,
        unit: DistanceUnit::Metre,
    })
}

/// A leg time or distance, always with its unit: `60s`, `1min`, `4nm`,
/// `7400m`.
fn hold_length(site: Site, token: Token<'_>) -> Result<HoldLength, Refusal> {
    let time = |value: Token<'_>, unit: TimeUnit, max: i64, range: &'static str| {
        positive(site, value, "leg time", max, range)
            .map(|value| HoldLength::Time(Duration { value, unit }))
    };
    if let Some(minutes) = token.strip_suffix("min") {
        return time(minutes, TimeUnit::Minute, 60, "more than 0 up to 60 min");
    }
    if token.text.ends_with("nm") {
        return distance(site, token).map(HoldLength::Distance);
    }
    if let Some(seconds) = token.strip_suffix("s") {
        return time(seconds, TimeUnit::Second, 3_600, "more than 0 up to 3600 s");
    }
    if let Some(metres) = token.strip_suffix("m") {
        return distance(site, metres).map(HoldLength::Distance);
    }
    Err(malformed(
        site,
        token,
        "a leg time or distance with its unit: 60s, 1min, 4nm or 7400m",
    ))
}

/// Degrees, 0 up to but not 360, then `T` (true) or `M` (magnetic).
fn bearing(site: Site, token: Token<'_>) -> Result<Bearing, Refusal> {
    let shape = "degrees then T (true) or M (magnetic), e.g. 271.5T";
    let (degrees, north) = match (token.strip_suffix("T"), token.strip_suffix("M")) {
        (Some(degrees), _) => (degrees, North::True),
        (_, Some(degrees)) => (degrees, North::Magnetic),
        _ => return Err(malformed(site, token, shape)),
    };
    let range = "0 up to but not 360 degrees";
    let value = ranged::<3>(site, degrees, &bounds("bearing", 0, 360, range))?;
    if value >= Milli::from_whole(360) {
        return Err(out_of_range(site, degrees, "bearing", range));
    }
    Ok(Bearing {
        degrees: value,
        north,
    })
}

fn vertical_angle(site: Site, token: Token<'_>) -> Result<Milli, Refusal> {
    token.non_empty(site, "an angle in degrees, negative descending")?;
    ranged::<3>(
        site,
        token,
        &bounds("vertical angle", -90, 90, "-90 to 90 degrees"),
    )
}

/// The kind letter `G`, `I` or `V`, then a height.
fn second_altitude(site: Site, token: Token<'_>) -> Result<SecondAltitude, Refusal> {
    let kind = SecondAltitudeKind::ALL
        .into_iter()
        .find(|kind| token.text.starts_with(kind.letter()))
        .ok_or_else(|| {
            malformed(
                site,
                token,
                "G (glide path), I (intercept) or V (vertical angle), then a height",
            )
        })?;
    let height_text = token
        .strip_prefix(&kind.letter().to_string())
        .unwrap_or(token);
    Ok(SecondAltitude {
        kind,
        height: height(site, height_text, true)?,
    })
}
