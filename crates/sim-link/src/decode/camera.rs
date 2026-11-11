//! `cam`: the terrain camera.

use super::number::{Bounds, out_of_range, ranged, whole};
use super::token::{Site, Token};
use crate::{Camera, Milli, Parameter, Reason, Refusal};

const SHAPE: &str = "WxH,fov_deg,depression_deg,rate_hz";

/// The camera of `cam`.
pub(super) fn camera(value: Token<'_>) -> Result<Camera, Refusal> {
    let site = Site::of(Parameter::Cam);
    let misshapen = || {
        site.refuse(
            value.offset,
            Reason::Malformed {
                text: value.text.to_owned(),
                expected: SHAPE,
            },
        )
    };
    value.non_empty(site, SHAPE)?;
    let fields: Vec<Token<'_>> = value.split(',').collect();
    let [size, fov, depression, rate] = fields.as_slice() else {
        return Err(misshapen());
    };
    let (width, height) = size.split_once('x').ok_or_else(misshapen)?;
    let rate_range = "0.1 to 120 Hz";
    let rate_hz: Milli = ranged(
        site,
        *rate,
        &Bounds {
            what: "frame rate",
            min: 0,
            max: 120,
            range: rate_range,
        },
    )?;
    if rate_hz < Milli::from_scaled(100) {
        return Err(out_of_range(site, *rate, "frame rate", rate_range));
    }
    Ok(Camera {
        width_px: side(site, width, "image width")?,
        height_px: side(site, height, "image height")?,
        fov_deg: ranged(
            site,
            *fov,
            &Bounds {
                what: "field of view",
                min: 1,
                max: 179,
                range: "1 to 179 degrees",
            },
        )?,
        depression_deg: ranged(
            site,
            *depression,
            &Bounds {
                what: "depression",
                min: 0,
                max: 90,
                range: "0 to 90 degrees",
            },
        )?,
        rate_hz,
    })
}

/// One side of the image, pixels.
fn side(site: Site, token: Token<'_>, what: &'static str) -> Result<u32, Refusal> {
    whole(site, token)?
        .and_then(|pixels| u32::try_from(pixels).ok())
        .filter(|pixels| (16..=8192).contains(pixels))
        .ok_or_else(|| out_of_range(site, token, what, "16 to 8192 pixels"))
}
