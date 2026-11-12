//! `out`: when each sensor delivers nothing.

use super::number::fixed;
use super::token::{Site, Token};
use super::word;
use crate::{Milli, Outage, Parameter, Reason, Refusal, Sensor, Span};

const ENTRY_SHAPE: &str = "sensor:start-end[,start-end...]";

/// The entries of `out`, one a sensor.
pub(super) fn outages(value: Token<'_>) -> Result<Vec<Outage>, Refusal> {
    let site = Site::of(Parameter::Out);
    value.non_empty(site, "outages joined by ;")?;
    let mut outages: Vec<Outage> = Vec::new();
    for (index, token) in value.split(';').enumerate() {
        let site = site.entry(index);
        let (sensor_token, spans) = token.split_once(':').ok_or_else(|| {
            site.refuse(
                token.offset,
                Reason::Malformed {
                    text: token.text.to_owned(),
                    expected: ENTRY_SHAPE,
                },
            )
        })?;
        let sensor = word(
            site,
            sensor_token,
            "sensor",
            "gnss, baro, stars, cam",
            &Sensor::ALL.map(|sensor| (sensor.key(), sensor)),
        )?;
        if outages.iter().any(|earlier| earlier.sensor == sensor) {
            return Err(site.refuse(
                sensor_token.offset,
                Reason::RepeatedSensor {
                    sensor: sensor.key(),
                },
            ));
        }
        outages.push(Outage {
            sensor,
            spans: self::spans(site, spans)?,
        });
    }
    Ok(outages)
}

/// Spans in increasing order; one may begin where the one before ends.
pub(super) fn spans(site: Site, value: Token<'_>) -> Result<Vec<Span>, Refusal> {
    let mut spans: Vec<Span> = Vec::new();
    for token in value.split(',') {
        let span = span(site, token)?;
        // A span that runs to the end of the flight leaves no room after it.
        let clear = spans
            .last()
            .is_none_or(|before| before.end.is_some_and(|end| span.start >= end));
        if !clear {
            return Err(site.refuse(
                token.offset,
                Reason::OverlappingSpans {
                    span: token.text.to_owned(),
                },
            ));
        }
        spans.push(span);
    }
    Ok(spans)
}

fn span(site: Site, token: Token<'_>) -> Result<Span, Refusal> {
    let (start, end) = token
        .non_empty(site, "start-end in seconds")?
        .split_once('-')
        .ok_or_else(|| {
            site.refuse(
                token.offset,
                Reason::Malformed {
                    text: token.text.to_owned(),
                    expected: "start-end in seconds; an empty end runs to the end of the flight",
                },
            )
        })?;
    let start: Milli = fixed(site, start.non_empty(site, "a start in seconds")?, false)?;
    let end: Option<Milli> = if end.text.is_empty() {
        None
    } else {
        Some(fixed(site, end, false)?)
    };
    if end.is_some_and(|end| end <= start) {
        return Err(site.refuse(
            token.offset,
            Reason::ReversedSpan {
                span: token.text.to_owned(),
            },
        ));
    }
    Ok(Span { start, end })
}
