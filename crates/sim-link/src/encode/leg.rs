//! The canonical form of the legs: the code, the fix, then the attributes
//! in key order; a fix whose name was given at the same position before is
//! written by its name alone.

use std::collections::BTreeMap;

use crate::{Fix, Key, Leg, Limit, Micro};

/// The positions of the fixes written so far, by name.
type Written = BTreeMap<String, (Micro, Micro)>;

/// The legs joined by `;`.
pub(super) fn legs(route: &[Leg]) -> String {
    let mut written = Written::new();
    let legs: Vec<String> = route
        .iter()
        .map(|leg| leg_text(leg, &mut written))
        .collect();
    legs.join(";")
}

fn leg_text(leg: &Leg, written: &mut Written) -> String {
    let mut fields = vec![leg.path.code().to_owned()];
    fields.extend(leg.fix.as_ref().map(|fix| fix_text(fix, written)));
    for key in leg.keys() {
        fields.push(format!("{}{}", key.letter(), value_text(leg, key, written)));
    }
    fields.join(",")
}

fn fix_text(fix: &Fix, written: &mut Written) -> String {
    let at = format!("{}/{}", fix.lat.text(0), fix.lon.text(0));
    match &fix.name {
        Some(name) if written.get(name) == Some(&(fix.lat, fix.lon)) => name.clone(),
        Some(name) => {
            written.insert(name.clone(), (fix.lat, fix.lon));
            format!("{name}:{at}")
        }
        None => at,
    }
}

/// The value of `key`, which `leg` carries.
fn value_text(leg: &Leg, key: Key, written: &mut Written) -> String {
    let text = match key {
        Key::O | Key::M => Some(String::new()),
        Key::R => Some(
            leg.roles
                .iter()
                .map(|role| role.word())
                .collect::<Vec<_>>()
                .join("."),
        ),
        Key::T => leg.turn.map(|turn| turn.letter().to_string()),
        Key::A => leg.altitude.map(|limit| limit.text(|height| height.text())),
        Key::G => leg
            .second_altitude
            .map(|second| format!("{}{}", second.kind.letter(), second.height.text())),
        Key::V => leg.vertical_angle.map(|angle| angle.text(0)),
        Key::X => leg.crossing_height.map(|height| height.text()),
        Key::S => leg
            .speed
            .map(|limit: Limit<_>| limit.text(|speed| speed.text())),
        Key::C => leg.course.map(|bearing| bearing.text()),
        Key::H => leg.heading.map(|bearing| bearing.text()),
        Key::D => leg.distance.map(|distance| distance.text()),
        Key::B => leg.theta.map(|bearing| bearing.text()),
        Key::N => leg.navaid.as_ref().map(|fix| fix_text(fix, written)),
        Key::Z => leg.centre.as_ref().map(|fix| fix_text(fix, written)),
        Key::L => leg.hold_length.map(|length| length.text()),
        Key::P => leg.rnp.map(|distance| distance.text()),
    };
    text.unwrap_or_default()
}
