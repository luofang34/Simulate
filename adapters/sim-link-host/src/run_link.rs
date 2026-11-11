//! The link of a run as it was flown: the visitor's switches written into
//! the link's outages, so opening it flies the same flight with the same
//! sensors withheld at the same instants.

use std::collections::BTreeMap;

use sim_link::{Milli, Outage, Refusal, Sensor, SimulationLink, Span};
use sim_navigate::trial::SensorKind;

#[cfg(test)]
mod tests;

/// The link's word for a sensor of the trial.
const fn link_sensor(sensor: SensorKind) -> Sensor {
    match sensor {
        SensorKind::Gnss => Sensor::Gnss,
        SensorKind::Baro => Sensor::Baro,
        SensorKind::Stars => Sensor::Stars,
        SensorKind::TerrainCamera => Sensor::Cam,
    }
}

/// Route seconds as the link writes them, to the millisecond.
fn milli(t_s: f64) -> Milli {
    Milli::from_scaled((t_s.max(0.0) * 1000.0).round() as i64)
}

/// The sensors the visitor switched off during a run, and when.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SwitchLog {
    spans: BTreeMap<Sensor, Vec<Span>>,
}

impl SwitchLog {
    /// `sensor` went off at route time `t_s`, the instant the flight
    /// applied it. A sensor already off stays off from when it went.
    pub fn off(&mut self, sensor: SensorKind, t_s: f64) {
        let spans = self.spans.entry(link_sensor(sensor)).or_default();
        if spans.last().is_none_or(|span| span.end.is_some()) {
            spans.push(Span {
                start: milli(t_s),
                end: None,
            });
        }
    }

    /// `sensor` came back at route time `t_s`. A sensor that is on stays
    /// as it was.
    pub fn on(&mut self, sensor: SensorKind, t_s: f64) {
        let Some(spans) = self.spans.get_mut(&link_sensor(sensor)) else {
            return;
        };
        if let Some(span) = spans.last_mut().filter(|span| span.end.is_none()) {
            let end = milli(t_s);
            if end > span.start {
                span.end = Some(end);
            } else {
                spans.pop();
            }
        }
    }

    /// The log of a flight's outages as it reports them: each sensor out
    /// from its start to its end, open while it stays out.
    pub fn of_outages(outages: &[(SensorKind, f64, Option<f64>)]) -> Self {
        let mut log = Self::default();
        for (sensor, start_s, end_s) in outages {
            log.off(*sensor, *start_s);
            if let Some(end_s) = end_s {
                log.on(*sensor, *end_s);
            }
        }
        log
    }

    /// Whether the visitor switched nothing.
    pub fn is_empty(&self) -> bool {
        self.spans.values().all(Vec::is_empty)
    }

    /// `link` with the switched spans added to its outages: each sensor's
    /// spans from both, joined where they overlap, in order.
    pub fn link_of_run(&self, link: &SimulationLink) -> SimulationLink {
        let mut all: BTreeMap<Sensor, Vec<Span>> = BTreeMap::new();
        for outage in &link.outages {
            all.entry(outage.sensor).or_default().extend(&outage.spans);
        }
        for (sensor, spans) in &self.spans {
            all.entry(*sensor).or_default().extend(spans);
        }
        let mut run = link.clone();
        run.outages = all
            .into_iter()
            .filter_map(|(sensor, spans)| {
                let spans = joined(spans);
                (!spans.is_empty()).then_some(Outage { sensor, spans })
            })
            .collect();
        run
    }

    /// The address of the run: the link of [`Self::link_of_run`] on `base`.
    ///
    /// # Errors
    ///
    /// The codec's refusal, such as a link grown past its length limit.
    pub fn url(&self, link: &SimulationLink, base: &str) -> Result<String, Refusal> {
        self.link_of_run(link).url(base)
    }
}

/// `spans` in order of start with overlapping ones joined; spans that only
/// touch stay apart, as the link may write them. An open span swallows
/// everything after its start.
fn joined(mut spans: Vec<Span>) -> Vec<Span> {
    spans.sort_by_key(|span| span.start);
    let mut out: Vec<Span> = Vec::new();
    for span in spans {
        match out.last_mut() {
            Some(last) if last.end.is_none_or(|end| span.start < end) => {
                last.end = match (last.end, span.end) {
                    (Some(a), Some(b)) => Some(a.max(b)),
                    _ => None,
                };
            }
            _ => out.push(span),
        }
    }
    out
}
