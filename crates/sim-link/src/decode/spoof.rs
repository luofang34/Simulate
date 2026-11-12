//! `spoof`: when a spoofer drags the GNSS receiver's fixes, how fast and
//! which way.

use super::number::{Bounds, out_of_range, ranged};
use super::outage::spans;
use super::token::{Site, Token};
use super::word;
use crate::{Milli, Parameter, Reason, Refusal, Spoof};

const SHAPE: &str = "gnss:rate,bearing:start-end[,start-end...]";
const RATE_RANGE: &str = "above 0 to 100 m/s";
const BEARING_RANGE: &str = "0 to under 360 degrees";

/// The spoofing of `spoof`.
pub(super) fn spoof(value: Token<'_>) -> Result<Spoof, Refusal> {
    let site = Site::of(Parameter::Spoof);
    let misshapen = |token: Token<'_>| {
        site.refuse(
            token.offset,
            Reason::Malformed {
                text: token.text.to_owned(),
                expected: SHAPE,
            },
        )
    };
    value.non_empty(site, SHAPE)?;
    let (sensor, rest) = value.split_once(':').ok_or_else(|| misshapen(value))?;
    // The receiver is the one device a spoofer of this link can capture.
    word(site, sensor, "sensor", "gnss", &[("gnss", ())])?;
    let (drag, when) = rest.split_once(':').ok_or_else(|| misshapen(rest))?;
    let (rate_token, bearing_token) = drag.split_once(',').ok_or_else(|| misshapen(drag))?;
    let rate: Milli = ranged(
        site,
        rate_token,
        &Bounds {
            what: "drag",
            min: 0,
            max: 100,
            range: RATE_RANGE,
        },
    )?;
    if rate <= Milli::from_whole(0) {
        return Err(out_of_range(site, rate_token, "drag", RATE_RANGE));
    }
    let bearing: Milli = ranged(
        site,
        bearing_token,
        &Bounds {
            what: "bearing",
            min: 0,
            max: 360,
            range: BEARING_RANGE,
        },
    )?;
    if bearing >= Milli::from_whole(360) {
        return Err(out_of_range(site, bearing_token, "bearing", BEARING_RANGE));
    }
    Ok(Spoof {
        rate,
        bearing,
        spans: spans(site, when)?,
    })
}
