//! `r`: the legs, each `CODE[,fix]{,attribute}`.

mod value;

use std::collections::BTreeMap;

use super::route::{name, position};
use super::token::{Site, Token};
use super::word;
use crate::{
    Fix, Key, Leg, MAX_LEGS, Micro, Parameter, PathTerminator, Reason, Refusal,
    distance_from_navaid, shape,
};

const CODES: &str = "IF TF CF DF FA FC FD FM CA CD CI CR RF AF VA VD VI VM VR PI HA HF HM";
const KEYS: &str = "o m r t a g v x s c h d b n z l p";

/// The positions of the fixes named so far: a bare name stands for the
/// latest of them.
pub(super) type Names = BTreeMap<String, (Micro, Micro)>;

/// The legs of `r`.
pub(super) fn route(value: Token<'_>) -> Result<Vec<Leg>, Refusal> {
    let site = Site::of(Parameter::R);
    value.non_empty(site, "legs joined by ;")?;
    let count = value.split(';').count();
    if count > MAX_LEGS {
        return Err(site.refuse(
            value.offset,
            Reason::TooManyLegs {
                count,
                limit: MAX_LEGS,
            },
        ));
    }
    if count < 2 {
        return Err(site.refuse(value.offset, Reason::TooFewLegs { count }));
    }
    let mut names = Names::new();
    value
        .split(';')
        .enumerate()
        .map(|(index, token)| {
            let site = site.entry(index);
            let leg = leg(site, token, &mut names)?;
            placement(site, token, leg.path, index, count)?;
            Ok(leg)
        })
        .collect()
}

fn leg(site: Site, token: Token<'_>, names: &mut Names) -> Result<Leg, Refusal> {
    let mut fields = token.split(',');
    let code = fields
        .next()
        .unwrap_or(token)
        .non_empty(site, "a path terminator code such as TF")?;
    let path = word(
        site,
        code,
        "path terminator",
        CODES,
        &PathTerminator::ALL.map(|path| (path.code(), path)),
    )?;
    let shape = shape(path);
    let fix = if shape.fix {
        let field = fields
            .next()
            .ok_or_else(|| site.refuse(token.end(), Reason::MissingFix { leg: path.code() }))?;
        Some(fix(site, field, names)?)
    } else {
        None
    };
    let attributes = attributes(site, path, fields)?;
    for key in shape.required {
        if !attributes.contains_key(key) {
            return Err(missing(site, token.end(), path, *key));
        }
    }
    let measured = attributes.get(&Key::B).or_else(|| {
        attributes
            .get(&Key::D)
            .filter(|_| distance_from_navaid(path))
    });
    if let (Some(measured), false) = (measured, attributes.contains_key(&Key::N)) {
        return Err(missing(site, measured.offset, path, Key::N));
    }
    let mut leg = Leg::new(path, fix);
    for (key, value) in attributes {
        value::apply(site, key, value, &mut leg, names)?;
    }
    Ok(leg)
}

fn missing(site: Site, offset: usize, path: PathTerminator, key: Key) -> Refusal {
    site.refuse(
        offset,
        Reason::MissingKey {
            key: key.letter(),
            leg: path.code(),
            meaning: key.meaning(),
        },
    )
}

/// The attributes by key, each value without its key letter. Refuses an
/// unknown key, a key the leg does not take and a key given twice.
fn attributes<'a>(
    site: Site,
    path: PathTerminator,
    fields: impl Iterator<Item = Token<'a>>,
) -> Result<BTreeMap<Key, Token<'a>>, Refusal> {
    let shape = shape(path);
    let mut attributes = BTreeMap::new();
    for field in fields {
        field.non_empty(site, "an attribute: a key letter and its value")?;
        let letter = field.text.chars().next().unwrap_or_default();
        let key = Key::ALL
            .into_iter()
            .find(|key| key.letter() == letter)
            .ok_or_else(|| {
                site.refuse(
                    field.offset,
                    Reason::UnknownWord {
                        what: "attribute key",
                        name: letter.to_string(),
                        choices: KEYS,
                    },
                )
            })?;
        if !shape.allows(key) {
            return Err(site.refuse(
                field.offset,
                Reason::KeyNotAllowed {
                    key: letter,
                    leg: path.code(),
                },
            ));
        }
        let value = field.strip_prefix(&letter.to_string()).unwrap_or(field);
        if attributes.insert(key, value).is_some() {
            return Err(site.refuse(field.offset, Reason::RepeatedKey { key: letter }));
        }
    }
    Ok(attributes)
}

/// A fix: `NAME:lat/lon` names a position, `lat/lon` is an unnamed one,
/// and a bare `NAME` is the latest earlier fix of that name.
pub(super) fn fix(site: Site, token: Token<'_>, names: &mut Names) -> Result<Fix, Refusal> {
    token.non_empty(site, "a fix: NAME:lat/lon, lat/lon or an earlier NAME")?;
    if let Some((label, at)) = token.split_once(':') {
        let label = name(site, label)?;
        let (lat, lon) = lat_lon(site, at)?;
        names.insert(label.clone(), (lat, lon));
        return Ok(Fix {
            name: Some(label),
            lat,
            lon,
        });
    }
    if token.text.contains('/') {
        let (lat, lon) = lat_lon(site, token)?;
        return Ok(Fix {
            name: None,
            lat,
            lon,
        });
    }
    let label = name(site, token)?;
    let (lat, lon) = names.get(&label).copied().ok_or_else(|| {
        site.refuse(
            token.offset,
            Reason::UnknownFix {
                name: label.clone(),
            },
        )
    })?;
    Ok(Fix {
        name: Some(label),
        lat,
        lon,
    })
}

fn lat_lon(site: Site, token: Token<'_>) -> Result<(Micro, Micro), Refusal> {
    let (lat, lon) = token.split_once('/').ok_or_else(|| {
        site.refuse(
            token.offset,
            Reason::Malformed {
                text: token.text.to_owned(),
                expected: "lat/lon in decimal degrees",
            },
        )
    })?;
    position(site, lat, lon)
}

/// Refuses a leg where its path terminator cannot stand.
fn placement(
    site: Site,
    token: Token<'_>,
    path: PathTerminator,
    index: usize,
    count: usize,
) -> Result<(), Refusal> {
    let last = index.saturating_add(1) == count;
    let rule = match path {
        PathTerminator::If if index != 0 => Some("starts the route only"),
        PathTerminator::Tf if index == 0 => {
            Some("needs a fix before it: start the route with IF or DF")
        }
        _ if path.intercepts_next() && last => {
            Some("intercepts the next leg, so a leg must follow it")
        }
        _ => None,
    };
    match rule {
        Some(rule) => Err(site.refuse(
            token.offset,
            Reason::MisplacedLeg {
                leg: path.code(),
                rule,
            },
        )),
        None => Ok(()),
    }
}
