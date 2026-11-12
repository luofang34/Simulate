use super::{GreyImage, MatchLine, MatchShot};
use crate::{Monitor, MonitorFeed};

fn shot(at_s: f64, outcome: &str, images: bool) -> MatchShot {
    let image = GreyImage {
        width: 2,
        height: 1,
        pixels: vec![10, 20],
    };
    MatchShot {
        set: "ins.baro.visual".to_owned(),
        at_s,
        outcome: outcome.to_owned(),
        path: images.then(|| "render".to_owned()),
        reference: images.then(|| image.clone()),
        frame: images.then_some(image),
        lines: vec![MatchLine {
            reference_px: [0.5, 0.5],
            frame_px: [1.5, 0.5],
            inlier: true,
        }],
        solved: None,
        fix: None,
    }
}

#[test]
fn a_skipped_frame_keeps_the_last_comparison_beside_its_outcome() {
    let mut monitor = Monitor::default();
    monitor.apply(MonitorFeed::Match(Box::new(shot(10.0, "accepted", true))));
    monitor.apply(MonitorFeed::Match(Box::new(shot(
        11.0,
        "unused:shared_map_error:unmatched",
        false,
    ))));
    let log = monitor.matches();
    assert_eq!(log.latest("ins.baro.visual").unwrap().at_s, 11.0);
    let compared = log.compared("ins.baro.visual").unwrap();
    assert_eq!(
        (compared.at_s, compared.outcome.as_str()),
        (10.0, "accepted")
    );
    assert!(log.latest("another").is_none());

    monitor.apply(MonitorFeed::Match(Box::new(shot(12.0, "accepted", true))));
    assert_eq!(
        monitor.matches().compared("ins.baro.visual").unwrap().at_s,
        12.0
    );
}
