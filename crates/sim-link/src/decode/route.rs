//! `r`, `h` and `start`: positions, heights, turns and names.

use super::number::{Bounds, out_of_range, ranged};
use super::token::{Site, Token};
use super::word;
use crate::{MAX_WAYPOINTS, Micro, Milli, Parameter, Reason, Refusal, Start, Turn, Waypoint};

const WAYPOINT_SHAPE: &str = "lat,lon[,height[,turn[,name]]]";
const START_SHAPE: &str = "lat,lon[,height[,heading_deg]]";

/// The waypoints of `r`. `has_default_height` is whether the link carries
/// `h` for the waypoints that give no height.
pub(super) fn route(value: Token<'_>, has_default_height: bool) -> Result<Vec<Waypoint>, Refusal> {
    let site = Site::of(Parameter::R);
    value.non_empty(site, "waypoints joined by ;")?;
    let count = value.split(';').count();
    if count > MAX_WAYPOINTS {
        return Err(site.refuse(
            value.offset,
            Reason::TooManyWaypoints {
                count,
                limit: MAX_WAYPOINTS,
            },
        ));
    }
    if count < 2 {
        return Err(site.refuse(value.offset, Reason::TooFewWaypoints { count }));
    }
    value
        .split(';')
        .enumerate()
        .map(|(index, token)| waypoint(site.entry(index), token, has_default_height))
        .collect()
}

fn waypoint(site: Site, token: Token<'_>, has_default_height: bool) -> Result<Waypoint, Refusal> {
    let mut fields = fields(site, token, 5, WAYPOINT_SHAPE)?.into_iter();
    let (Some(lat), Some(lon)) = (fields.next(), fields.next()) else {
        return Err(misshapen(site, token, WAYPOINT_SHAPE));
    };
    let (lat, lon) = position(site, lat, lon)?;
    let height = optional_height(site, fields.next(), token, has_default_height)?;
    let turn = match fields.next() {
        Some(turn) if !turn.text.is_empty() => word(
            site,
            turn,
            "turn",
            "o (fly-over), b or nothing (fly-by)",
            &[("o", Turn::FlyOver), ("b", Turn::FlyBy)],
        )?,
        _ => Turn::FlyBy,
    };
    let name = fields
        .next()
        .filter(|name| !name.text.is_empty())
        .map(|name| self::name(site, name))
        .transpose()?;
    Ok(Waypoint {
        lat,
        lon,
        height,
        turn,
        name,
    })
}

/// The start state of `start`.
pub(super) fn start(value: Token<'_>, has_default_height: bool) -> Result<Start, Refusal> {
    let site = Site::of(Parameter::Start);
    value.non_empty(site, START_SHAPE)?;
    let mut fields = fields(site, value, 4, START_SHAPE)?.into_iter();
    let (Some(lat), Some(lon)) = (fields.next(), fields.next()) else {
        return Err(misshapen(site, value, START_SHAPE));
    };
    let (lat, lon) = position(site, lat, lon)?;
    let height = optional_height(site, fields.next(), value, has_default_height)?;
    let heading = fields
        .next()
        .filter(|heading| !heading.text.is_empty())
        .map(|heading| self::heading(site, heading))
        .transpose()?;
    Ok(Start {
        lat,
        lon,
        height,
        heading,
    })
}

/// A flight height, metres.
pub(super) fn height(site: Site, token: Token<'_>) -> Result<Milli, Refusal> {
    ranged::<3>(
        site,
        token.non_empty(site, "a height in metres")?,
        &Bounds {
            what: "height",
            min: -500,
            max: 30_000,
            range: "-500 to 30000 m",
        },
    )
}

/// The comma-separated fields of `token`: two to `most`.
fn fields<'a>(
    site: Site,
    token: Token<'a>,
    most: usize,
    shape: &'static str,
) -> Result<Vec<Token<'a>>, Refusal> {
    let fields: Vec<Token<'a>> = token.split(',').collect();
    if !(2..=most).contains(&fields.len()) {
        return Err(misshapen(site, token, shape));
    }
    Ok(fields)
}

fn misshapen(site: Site, token: Token<'_>, shape: &'static str) -> Refusal {
    site.refuse(
        token.offset,
        Reason::Malformed {
            text: token.text.to_owned(),
            expected: shape,
        },
    )
}

fn position(site: Site, lat: Token<'_>, lon: Token<'_>) -> Result<(Micro, Micro), Refusal> {
    let lat = ranged::<6>(
        site,
        lat.non_empty(site, "a latitude in degrees")?,
        &Bounds {
            what: "latitude",
            min: -90,
            max: 90,
            range: "-90 to 90 degrees",
        },
    )?;
    let lon = ranged::<6>(
        site,
        lon.non_empty(site, "a longitude in degrees")?,
        &Bounds {
            what: "longitude",
            min: -180,
            max: 180,
            range: "-180 to 180 degrees",
        },
    )?;
    Ok((lat, lon))
}

/// The height field when it says something; without it the link must carry
/// a default, or the height would have to be invented.
fn optional_height(
    site: Site,
    field: Option<Token<'_>>,
    whole: Token<'_>,
    has_default_height: bool,
) -> Result<Option<Milli>, Refusal> {
    match field.filter(|field| !field.text.is_empty()) {
        Some(field) => height(site, field).map(Some),
        None if has_default_height => Ok(None),
        None => Err(site.refuse(whole.offset, Reason::MissingHeight)),
    }
}

fn heading(site: Site, token: Token<'_>) -> Result<Milli, Refusal> {
    let range = "0 up to but not 360 degrees";
    let heading = ranged::<3>(
        site,
        token,
        &Bounds {
            what: "heading",
            min: 0,
            max: 360,
            range,
        },
    )?;
    if heading >= Milli::from_whole(360) {
        return Err(out_of_range(site, token, "heading", range));
    }
    Ok(heading)
}

fn name(site: Site, token: Token<'_>) -> Result<String, Refusal> {
    let allowed = |byte: u8| byte.is_ascii_alphanumeric() || byte == b'_' || byte == b'-';
    if token.text.len() > 32 || !token.text.bytes().all(allowed) {
        return Err(site.refuse(
            token.offset,
            Reason::BadName {
                text: token.text.to_owned(),
            },
        ));
    }
    Ok(token.text.to_owned())
}
