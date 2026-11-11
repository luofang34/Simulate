//! `nav` and `fly`: the navigation sets and the one that steers.

use super::token::{Site, Token};
use super::word;
use crate::{Means, NavSet, Parameter, Reason, Refusal};

/// The sets of `nav`.
pub(super) fn sets(value: Token<'_>) -> Result<Vec<NavSet>, Refusal> {
    let site = Site::of(Parameter::Nav);
    value.non_empty(site, "sets joined by ;")?;
    let mut sets: Vec<NavSet> = Vec::new();
    for (index, token) in value.split(';').enumerate() {
        let site = site.entry(index);
        let set = set(site, token)?;
        if sets.iter().any(|earlier| earlier.same_means(&set)) {
            return Err(site.refuse(
                token.offset,
                Reason::RepeatedSet {
                    set: token.text.to_owned(),
                },
            ));
        }
        sets.push(set);
    }
    Ok(sets)
}

/// The index in `sets` of the set `fly` names, in any order of its means.
pub(super) fn steering_set(value: Token<'_>, sets: &[NavSet]) -> Result<usize, Refusal> {
    let site = Site::of(Parameter::Fly);
    let wanted = set(site, value)?;
    sets.iter()
        .position(|set| set.same_means(&wanted))
        .ok_or_else(|| {
            site.refuse(
                value.offset,
                Reason::FlyNotInNav {
                    set: value.text.to_owned(),
                },
            )
        })
}

/// One set: distinct means joined by `.`, the inertial navigator among them.
fn set(site: Site, token: Token<'_>) -> Result<NavSet, Refusal> {
    let mut means: Vec<Means> = Vec::new();
    for part in token.split('.') {
        let one = word(
            site,
            part.non_empty(site, "a means: ins, baro, gnss, stars, visual")?,
            "means",
            "ins, baro, gnss, stars, visual",
            &Means::ALL.map(|means| (means.key(), means)),
        )?;
        if means.contains(&one) {
            return Err(site.refuse(
                part.offset,
                Reason::RepeatedMeans {
                    set: token.text.to_owned(),
                    means: one.key(),
                },
            ));
        }
        means.push(one);
    }
    let set = NavSet(means);
    if !set.contains(Means::Ins) {
        return Err(site.refuse(
            token.offset,
            Reason::SetWithoutIns {
                set: token.text.to_owned(),
            },
        ));
    }
    Ok(set)
}
