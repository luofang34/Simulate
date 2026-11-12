//! Writing a link in the one form every encoder agrees on.

mod leg;

#[cfg(test)]
mod tests;

use crate::{
    MAX_LINK_CHARS, Outage, Parameter, Reason, Refusal, SimulationLink, Span, Start,
    decode_fragment,
};

impl SimulationLink {
    /// The fragment of the link, the text after `#`: parameters in the
    /// format's order, the legs in their canonical form, the start's
    /// position with six decimals, heights and the heading with at least
    /// one, every other number with no spare zeros.
    ///
    /// # Errors
    ///
    /// Refuses a value no link can carry, exactly as [`decode_fragment`]
    /// would refuse its written form: what this returns always reads back
    /// as the same value.
    pub fn fragment(&self) -> Result<String, Refusal> {
        let text = self.written()?;
        if decode_fragment(&text)? != *self {
            return Err(whole_link(0, Reason::NotRepresentable));
        }
        Ok(text)
    }

    /// The whole link on `base`, the address the simulator is served at;
    /// anything from a `#` in `base` on is dropped.
    ///
    /// # Errors
    ///
    /// As [`Self::fragment`], and a link longer than [`MAX_LINK_CHARS`].
    pub fn url(&self, base: &str) -> Result<String, Refusal> {
        let base = base.split_once('#').map_or(base, |(before, _)| before);
        let link = format!("{base}#{}", self.fragment()?);
        let length = link.chars().count();
        if length > MAX_LINK_CHARS {
            return Err(whole_link(
                MAX_LINK_CHARS,
                Reason::TooLong {
                    length,
                    limit: MAX_LINK_CHARS,
                },
            ));
        }
        Ok(link)
    }

    fn written(&self) -> Result<String, Refusal> {
        let steering = self.steering().ok_or_else(|| Refusal {
            parameter: Parameter::Fly,
            item: None,
            offset: 0,
            reason: Reason::FlyNotInNav {
                set: format!("set {}", self.steering_set.saturating_add(1)),
            },
        })?;
        let joined = |parts: Vec<String>| parts.join(";");
        let mut pairs: Vec<(Parameter, String)> = vec![
            (Parameter::V, "2".to_owned()),
            (Parameter::R, leg::legs(&self.route)),
        ];
        let mut push = |parameter: Parameter, value: Option<String>| {
            pairs.extend(value.map(|value| (parameter, value)));
        };
        push(
            Parameter::H,
            self.default_height.map(|height| height.text(1)),
        );
        push(Parameter::Hd, Some(self.datum.key().to_owned()));
        push(Parameter::Start, self.start.as_ref().map(start));
        push(Parameter::T, Some(self.time.compact()));
        push(Parameter::Veh, Some(self.vehicle.key().to_owned()));
        push(Parameter::Spd, self.cruise_speed.map(|speed| speed.text(0)));
        push(Parameter::Seed, Some(self.seed.to_string()));
        push(
            Parameter::Nav,
            Some(joined(self.sets.iter().map(|set| set.text()).collect())),
        );
        push(Parameter::Fly, Some(steering.text()));
        push(
            Parameter::Out,
            (!self.outages.is_empty()).then(|| joined(self.outages.iter().map(outage).collect())),
        );
        push(
            Parameter::Spoof,
            self.spoof.as_ref().map(|spoof| {
                format!(
                    "gnss:{},{}:{}",
                    spoof.rate.text(0),
                    spoof.bearing.text(0),
                    spans(&spoof.spans)
                )
            }),
        );
        push(
            Parameter::Cam,
            self.camera.map(|camera| {
                format!(
                    "{}x{},{},{},{}",
                    camera.width_px,
                    camera.height_px,
                    camera.fov_deg.text(0),
                    camera.depression_deg.text(0),
                    camera.rate_hz.text(0)
                )
            }),
        );
        push(Parameter::Go, self.auto_start.then(|| "1".to_owned()));
        push(Parameter::Mb, Some(self.budget_mb.to_string()));
        let pairs: Vec<String> = pairs
            .into_iter()
            .map(|(parameter, value)| format!("{parameter}={value}"))
            .collect();
        Ok(pairs.join("&"))
    }
}

fn whole_link(offset: usize, reason: Reason) -> Refusal {
    Refusal {
        parameter: Parameter::Link,
        item: None,
        offset,
        reason,
    }
}

/// Fields joined by `,` with the empty ones at the end dropped.
fn trimmed(mut fields: Vec<String>) -> String {
    while fields.last().is_some_and(String::is_empty) {
        fields.pop();
    }
    fields.join(",")
}

fn start(start: &Start) -> String {
    trimmed(vec![
        start.lat.text(6),
        start.lon.text(6),
        start
            .height
            .map_or_else(String::new, |height| height.text(1)),
        start
            .heading
            .map_or_else(String::new, |heading| heading.text(1)),
    ])
}

fn outage(outage: &Outage) -> String {
    format!("{}:{}", outage.sensor.key(), spans(&outage.spans))
}

fn spans(spans: &[Span]) -> String {
    let spans: Vec<String> = spans
        .iter()
        .map(|span| {
            let end = span.end.map_or_else(String::new, |end| end.text(0));
            format!("{}-{end}", span.start.text(0))
        })
        .collect();
    spans.join(",")
}
