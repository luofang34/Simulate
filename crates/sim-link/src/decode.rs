//! Reading a link: percent escapes, `key=value` pairs, then each parameter.

mod camera;
mod nav;
mod number;
mod outage;
mod route;
mod time;
mod token;

#[cfg(test)]
mod tests;

use std::collections::BTreeMap;

use token::{Site, Token};

use crate::{HeightDatum, MAX_LINK_CHARS, Parameter, Reason, Refusal, SimulationLink, Vehicle};

/// Reads a whole URL. `Ok(None)` is a URL with no fragment: no link, so the
/// page behaves as it does without one.
///
/// # Errors
///
/// Refuses a URL longer than [`MAX_LINK_CHARS`] and everything
/// [`decode_fragment`] refuses.
pub fn decode_url(url: &str) -> Result<Option<SimulationLink>, Refusal> {
    let length = url.chars().count();
    if length > MAX_LINK_CHARS {
        return Err(too_long(length));
    }
    match url.split_once('#') {
        Some((_, fragment)) if !fragment.is_empty() => decode_fragment(fragment).map(Some),
        _ => Ok(None),
    }
}

/// Reads the fragment of a link, the text after `#`.
///
/// # Errors
///
/// Refuses anything that is not a version 1 link, naming the parameter, the
/// character and the reason.
pub fn decode_fragment(fragment: &str) -> Result<SimulationLink, Refusal> {
    let length = fragment.chars().count();
    if length > MAX_LINK_CHARS {
        return Err(too_long(length));
    }
    let text = resolve_escapes(fragment)?;
    let pairs = pairs(&text)?;
    version(&pairs, text.len())?;
    let mut fields = BTreeMap::new();
    for (key, value) in pairs {
        let parameter = Parameter::from_key(key.text).ok_or_else(|| {
            Site::of(Parameter::Link).refuse(
                key.offset,
                Reason::UnknownParameter {
                    name: key.text.to_owned(),
                },
            )
        })?;
        if fields.insert(parameter, value).is_some() {
            return Err(Site::of(parameter).refuse(key.offset, Reason::Repeated));
        }
    }
    build(&Fields {
        fields,
        end: text.len(),
    })
}

fn too_long(length: usize) -> Refusal {
    Site::of(Parameter::Link).refuse(
        MAX_LINK_CHARS,
        Reason::TooLong {
            length,
            limit: MAX_LINK_CHARS,
        },
    )
}

/// Resolves the percent escapes of a fragment, which chat tools put in place
/// of `;`, `,` and `:`. A [`Refusal`]'s offset counts characters of this
/// text. Every character of a link is ASCII, so the result is too.
///
/// # Errors
///
/// Refuses a character outside ASCII and an escape that is not `%` and two
/// hex digits of an ASCII character.
pub fn resolve_escapes(fragment: &str) -> Result<String, Refusal> {
    let refuse = |offset: usize, text: String| {
        Site::of(Parameter::Link).refuse(offset, Reason::BadEscape { text })
    };
    let mut text = String::with_capacity(fragment.len());
    let mut characters = fragment.chars();
    while let Some(character) = characters.next() {
        if !character.is_ascii() {
            return Err(refuse(text.len(), character.to_string()));
        }
        if character != '%' {
            text.push(character);
            continue;
        }
        let escape: String = characters.by_ref().take(2).collect();
        let byte = (escape.len() == 2 && escape.bytes().all(|byte| byte.is_ascii_hexdigit()))
            .then(|| u8::from_str_radix(&escape, 16).ok())
            .flatten()
            .filter(u8::is_ascii)
            .ok_or_else(|| refuse(text.len(), format!("%{escape}")))?;
        text.push(char::from(byte));
    }
    Ok(text)
}

/// The `key=value` pairs between the `&`s, in order.
fn pairs(text: &str) -> Result<Vec<(Token<'_>, Token<'_>)>, Refusal> {
    Token::new(text, 0)
        .split('&')
        .map(|pair| {
            pair.split_once('=').ok_or_else(|| {
                Site::of(Parameter::Link).refuse(
                    pair.offset,
                    Reason::Malformed {
                        text: pair.text.to_owned(),
                        expected: "key=value",
                    },
                )
            })
        })
        .collect()
}

/// Judges the version before anything else: a later version may carry keys
/// and shapes this build would call malformed.
fn version(pairs: &[(Token<'_>, Token<'_>)], end: usize) -> Result<(), Refusal> {
    let site = Site::of(Parameter::V);
    let mut versions = pairs
        .iter()
        .filter(|(key, _)| key.text == Parameter::V.key());
    let (_, value) = versions
        .next()
        .ok_or_else(|| site.refuse(end, Reason::Missing))?;
    if value.text != "1" {
        return Err(site.refuse(
            value.offset,
            Reason::UnsupportedVersion {
                found: value.text.to_owned(),
            },
        ));
    }
    Ok(())
}

/// The link's values by parameter.
struct Fields<'a> {
    fields: BTreeMap<Parameter, Token<'a>>,
    end: usize,
}

impl<'a> Fields<'a> {
    fn optional(&self, parameter: Parameter) -> Option<Token<'a>> {
        self.fields.get(&parameter).copied()
    }

    fn required(&self, parameter: Parameter) -> Result<Token<'a>, Refusal> {
        self.optional(parameter)
            .ok_or_else(|| Site::of(parameter).refuse(self.end, Reason::Missing))
    }
}

fn build(fields: &Fields<'_>) -> Result<SimulationLink, Refusal> {
    let default_height = fields
        .optional(Parameter::H)
        .map(|value| route::height(Site::of(Parameter::H), value))
        .transpose()?;
    let route = route::route(fields.required(Parameter::R)?, default_height.is_some())?;
    let datum = word(
        Site::of(Parameter::Hd),
        fields.required(Parameter::Hd)?,
        "datum",
        "msl, hae",
        &HeightDatum::ALL.map(|datum| (datum.key(), datum)),
    )?;
    let start = fields
        .optional(Parameter::Start)
        .map(|value| route::start(value, default_height.is_some()))
        .transpose()?;
    let time = time::start_time(fields.required(Parameter::T)?)?;
    let vehicle = word(
        Site::of(Parameter::Veh),
        fields.required(Parameter::Veh)?,
        "vehicle",
        "c172, x500",
        &Vehicle::ALL.map(|vehicle| (vehicle.key(), vehicle)),
    )?;
    let cruise_speed = fields
        .optional(Parameter::Spd)
        .map(number::cruise_speed)
        .transpose()?;
    let seed = number::seed(fields.required(Parameter::Seed)?)?;
    let sets = nav::sets(fields.required(Parameter::Nav)?)?;
    let steering_set = fields
        .optional(Parameter::Fly)
        .map_or(Ok(0), |value| nav::steering_set(value, &sets))?;
    let outages = fields
        .optional(Parameter::Out)
        .map_or_else(|| Ok(Vec::new()), outage::outages)?;
    let camera = fields
        .optional(Parameter::Cam)
        .map(camera::camera)
        .transpose()?;
    let auto_start = fields
        .optional(Parameter::Go)
        .map(go)
        .transpose()?
        .is_some();
    let budget_mb = number::budget_mb(fields.required(Parameter::Mb)?)?;
    Ok(SimulationLink {
        route,
        default_height,
        datum,
        start,
        time,
        vehicle,
        cruise_speed,
        seed,
        sets,
        steering_set,
        outages,
        camera,
        auto_start,
        budget_mb,
    })
}

/// The entry of `words` that `token` spells.
fn word<T: Copy>(
    site: Site,
    token: Token<'_>,
    what: &'static str,
    choices: &'static str,
    words: &[(&'static str, T)],
) -> Result<T, Refusal> {
    words
        .iter()
        .find(|(key, _)| *key == token.text)
        .map(|(_, value)| *value)
        .ok_or_else(|| {
            site.refuse(
                token.offset,
                Reason::UnknownWord {
                    what,
                    name: token.text.to_owned(),
                    choices,
                },
            )
        })
}

fn go(value: Token<'_>) -> Result<(), Refusal> {
    if value.text == "1" {
        return Ok(());
    }
    Err(Site::of(Parameter::Go).refuse(
        value.offset,
        Reason::Malformed {
            text: value.text.to_owned(),
            expected: "1; leave go out to be asked first",
        },
    ))
}
