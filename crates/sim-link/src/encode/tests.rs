use crate::{
    Camera, HeightDatum, MAX_LINK_CHARS, Means, Micro, Milli, NavSet, Outage, Parameter, Reason,
    Sensor, SimulationLink, Span, Start, StartTime, Turn, Vehicle, Waypoint, decode_fragment,
};

/// A deterministic stream of values, so the round trip covers many links
/// without a table of them.
struct Draws(u64);

impl Draws {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0 >> 20
    }

    fn within(&mut self, low: i64, high: i64) -> i64 {
        low + (self.next() % (high - low + 1) as u64) as i64
    }

    fn flag(&mut self) -> bool {
        self.next().is_multiple_of(2)
    }
}

fn waypoint(draws: &mut Draws, with_height: bool) -> Waypoint {
    Waypoint {
        lat: Micro::from_scaled(draws.within(-90_000_000, 90_000_000)),
        lon: Micro::from_scaled(draws.within(-180_000_000, 180_000_000)),
        height: with_height.then(|| Milli::from_scaled(draws.within(-500_000, 30_000_000))),
        turn: if draws.flag() {
            Turn::FlyOver
        } else {
            Turn::FlyBy
        },
        name: draws
            .flag()
            .then(|| format!("W_{}-x", draws.within(0, 999))),
    }
}

fn outage(draws: &mut Draws, sensor: Sensor) -> Outage {
    let mut at = draws.within(0, 5_000);
    let count = draws.within(1, 4);
    let spans = (0..count)
        .map(|index| {
            let start = at;
            at += draws.within(1, 90_000);
            let open = index + 1 == count && draws.flag();
            let span = Span {
                start: Milli::from_scaled(start),
                end: (!open).then(|| Milli::from_scaled(at)),
            };
            at += draws.within(0, 3_000);
            span
        })
        .collect();
    Outage { sensor, spans }
}

fn link(draws: &mut Draws) -> SimulationLink {
    let all_heights = draws.flag();
    let mut route = Vec::new();
    for _ in 0..draws.within(2, 12) {
        let with_height = all_heights || draws.flag();
        route.push(waypoint(draws, with_height));
    }
    let mut outages = Vec::new();
    for sensor in Sensor::ALL.into_iter().rev() {
        if draws.flag() {
            outages.push(outage(draws, sensor));
        }
    }
    let orders: [&[Means]; 5] = [
        &[Means::Ins],
        &[Means::Baro, Means::Ins],
        &[Means::Ins, Means::Gnss, Means::Baro],
        &[Means::Stars, Means::Ins, Means::Baro],
        &[
            Means::Ins,
            Means::Baro,
            Means::Gnss,
            Means::Stars,
            Means::Visual,
        ],
    ];
    let sets: Vec<NavSet> = orders
        .iter()
        .filter(|set| set.len() == 1 || draws.flag())
        .map(|set| NavSet(set.to_vec()))
        .collect();
    SimulationLink {
        route,
        default_height: (!all_heights || draws.flag())
            .then(|| Milli::from_scaled(draws.within(-500_000, 30_000_000))),
        datum: if draws.flag() {
            HeightDatum::Ellipsoid
        } else {
            HeightDatum::MeanSeaLevel
        },
        start: draws.flag().then(|| Start {
            lat: Micro::from_scaled(draws.within(-90_000_000, 90_000_000)),
            lon: Micro::from_scaled(draws.within(-180_000_000, 180_000_000)),
            height: all_heights.then(|| Milli::from_scaled(draws.within(-500_000, 30_000_000))),
            heading: draws
                .flag()
                .then(|| Milli::from_scaled(draws.within(0, 359_999))),
        }),
        time: StartTime {
            year: draws.within(1, 9999) as u16,
            month: draws.within(1, 12) as u8,
            day: draws.within(1, 28) as u8,
            hour: draws.within(0, 23) as u8,
            minute: draws.within(0, 59) as u8,
            second: draws.within(0, 59) as u8,
        },
        vehicle: if draws.flag() {
            Vehicle::C172
        } else {
            Vehicle::X500
        },
        cruise_speed: draws
            .flag()
            .then(|| Milli::from_scaled(draws.within(1_000, 350_000))),
        seed: draws.next() << 20 | draws.next(),
        steering_set: draws.within(0, sets.len() as i64 - 1) as usize,
        sets,
        outages,
        camera: draws.flag().then(|| Camera {
            width_px: draws.within(16, 8192) as u32,
            height_px: draws.within(16, 8192) as u32,
            fov_deg: Milli::from_scaled(draws.within(1_000, 179_000)),
            depression_deg: Milli::from_scaled(draws.within(0, 90_000)),
            rate_hz: Milli::from_scaled(draws.within(100, 120_000)),
        }),
        auto_start: draws.flag(),
        budget_mb: draws.within(1, 65_536) as u32,
    }
}

#[test]
fn every_link_written_reads_back_as_the_same_value() {
    let mut draws = Draws(0x5eed);
    for round in 0..500 {
        let link = link(&mut draws);
        let fragment = link
            .fragment()
            .unwrap_or_else(|refusal| panic!("{round}: {refusal}"));
        assert_eq!(decode_fragment(&fragment).as_ref(), Ok(&link), "{fragment}");
        let again = decode_fragment(&fragment).unwrap().fragment().unwrap();
        assert_eq!(again, fragment);
    }
}

#[test]
fn parameters_are_written_in_the_order_of_the_format() {
    let mut draws = Draws(7);
    for _ in 0..50 {
        let fragment = link(&mut draws).fragment().unwrap();
        let written: Vec<Parameter> = fragment
            .split('&')
            .map(|pair| Parameter::from_key(pair.split_once('=').unwrap().0).unwrap())
            .collect();
        let mut sorted = written.clone();
        sorted.sort();
        assert_eq!(written, sorted, "{fragment}");
        assert_eq!(written.first(), Some(&Parameter::V));
    }
}

/// A change to a valid link, and the refusal writing it must give.
type Unwritable = (fn(&mut SimulationLink), Parameter, fn(&Reason) -> bool);

fn valid() -> SimulationLink {
    decode_fragment(
        "v=1&r=41.5,2.0;41.6,2.1&h=1500&hd=msl&t=20260615T120000Z&veh=c172&seed=1&nav=ins.baro;ins&mb=200",
    )
    .unwrap()
}

#[test]
fn numbers_take_the_written_form() {
    let mut link = valid();
    link.cruise_speed = Some(Milli::from_whole(55));
    link.outages = vec![Outage {
        sensor: Sensor::Gnss,
        spans: vec![Span {
            start: Milli::from_whole(300),
            end: None,
        }],
    }];
    assert_eq!(
        link.fragment().unwrap(),
        "v=1&r=41.500000,2.000000;41.600000,2.100000&h=1500.0&hd=msl&t=20260615T120000Z\
&veh=c172&spd=55&seed=1&nav=ins.baro;ins&fly=ins.baro&out=gnss:300-&mb=200"
    );
}

#[test]
fn a_value_no_link_can_carry_is_refused_as_its_written_form_would_be() {
    let cases: [Unwritable; 9] = [
        (
            |l| l.route.truncate(1),
            Parameter::R,
            |r| matches!(r, Reason::TooFewWaypoints { .. }),
        ),
        (
            |l| l.default_height = None,
            Parameter::R,
            |r| matches!(r, Reason::MissingHeight),
        ),
        (
            |l| l.route[0].lat = Micro::from_scaled(90_000_001),
            Parameter::R,
            |r| matches!(r, Reason::OutOfRange { .. }),
        ),
        (
            |l| l.route[1].name = Some("no good".into()),
            Parameter::R,
            |r| matches!(r, Reason::BadName { .. }),
        ),
        (
            |l| l.steering_set = 2,
            Parameter::Fly,
            |r| matches!(r, Reason::FlyNotInNav { .. }),
        ),
        (
            |l| l.sets.push(NavSet(vec![Means::Baro, Means::Ins])),
            Parameter::Nav,
            |r| matches!(r, Reason::RepeatedSet { .. }),
        ),
        (
            |l| {
                l.outages = vec![Outage {
                    sensor: Sensor::Cam,
                    spans: vec![
                        Span {
                            start: Milli::from_whole(5),
                            end: Some(Milli::from_whole(9)),
                        },
                        Span {
                            start: Milli::from_whole(8),
                            end: None,
                        },
                    ],
                }];
            },
            Parameter::Out,
            |r| matches!(r, Reason::OverlappingSpans { .. }),
        ),
        (
            |l| l.budget_mb = 0,
            Parameter::Mb,
            |r| matches!(r, Reason::OutOfRange { .. }),
        ),
        (
            |l| l.route[0].name = Some(String::new()),
            Parameter::Link,
            |r| matches!(r, Reason::NotRepresentable),
        ),
    ];
    for (index, (change, parameter, expected)) in cases.into_iter().enumerate() {
        let mut link = valid();
        change(&mut link);
        let refusal = link.fragment().unwrap_err();
        assert_eq!(refusal.parameter, parameter, "case {index}: {refusal}");
        assert!(expected(&refusal.reason), "case {index}: {refusal:?}");
    }
}

#[test]
fn the_url_drops_the_fragment_of_the_base_and_keeps_to_the_length_limit() {
    let link = valid();
    let fragment = link.fragment().unwrap();
    assert_eq!(
        link.url("http://localhost:8080/#old").unwrap(),
        format!("http://localhost:8080/#{fragment}")
    );
    let long_base = format!("https://example.invalid/{}", "a".repeat(MAX_LINK_CHARS));
    let refusal = link.url(&long_base).unwrap_err();
    assert!(matches!(
        refusal.reason,
        Reason::TooLong {
            limit: MAX_LINK_CHARS,
            ..
        }
    ));
}
