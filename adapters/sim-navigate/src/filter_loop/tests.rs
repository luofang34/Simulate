use glam::DVec3;
use navigate_contract::{
    GeodeticPosition, SensorClass, SourceComposition, SourceEpoch, SourceId, SymmetricCov3,
    WrappingSequence,
};
use navigate_fusion::ObservationValue;
use sim_core::TICKS_PER_SECOND;
use sim_devices::WhiteNoise;
use sim_geo::{Geodetic, LocalFrame, geodetic_to_ecef};

use super::*;
use crate::{KinematicAiding, compare_with_truth};

const HOLD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 10 * 3);
const SPEED_M_S: f64 = 55.0;

fn after(millis: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(TICKS_PER_SECOND / 1_000 * millis))
}

fn position_fix(sequence: u32, at: SimInstant, position: &Geodetic) -> Observation {
    let stamp = MasterClockBridge::default()
        .stamp(
            SourceId::new(1),
            SourceEpoch::new(0),
            WrappingSequence::new(sequence),
            at,
        )
        .unwrap();
    Observation::new(
        stamp,
        ObservationValue::PositionFix {
            position: GeodeticPosition::new(position.lat_rad, position.lon_rad, position.height_m),
            covariance: SymmetricCov3::from_diagonal(1.0, 1.0, 1.0),
        },
        SourceComposition::of(SensorClass::VisualLandmark),
    )
}

/// The eastbound position at `millis`.
fn eastbound(millis: u64) -> (Geodetic, DVec3) {
    let start = Geodetic::from_degrees(39.1, -75.6, 900.0);
    let frame = LocalFrame::at(&start);
    let along = frame.east() * SPEED_M_S * (millis as f64 / 1_000.0);
    (start, geodetic_to_ecef(&start) + along)
}

/// The defect the hold exists for: an anchor delivered late, after fresher
/// velocity aiding, must land where it was taken, not 16 m further on.
#[test]
fn a_delayed_fix_lands_at_its_acquisition_time_not_the_present() {
    let (start, _) = eastbound(0);
    let east = LocalFrame::at(&start).east() * SPEED_M_S;
    let mut navigation = NavigationLoop::new(
        airborne_fusion_config(SPEED_M_S, 1.0, 0.05),
        MasterClockBridge::default(),
        HOLD,
    )
    .unwrap();
    let mut aiding = KinematicAiding::new(
        MasterClockBridge::default(),
        SourceId::new(2),
        SourceEpoch::new(0),
        0.1,
    );
    navigation.offer(position_fix(0, after(0), &start));
    let mut solution = None;
    for millis in (0..2_000).step_by(50) {
        let (_, truth) = eastbound(millis);
        navigation.offer(aiding.observe(after(millis), truth, east).unwrap());
        // The anchor taken at 1 s arrives 300 ms later, behind 6 velocity fixes.
        if millis == 1_300 {
            let (_, at_exposure) = eastbound(1_000);
            let position = sim_geo::ecef_to_geodetic(at_exposure);
            navigation.offer(position_fix(1, after(1_000), &position));
        }
        solution = navigation.advance(after(millis)).unwrap();
    }
    let rejections = navigation.filter().rejections();
    assert_eq!(rejections.innovation_gate, 0);
    assert_eq!(rejections.stale, 0);
    let (_, truth) = eastbound(1_950);
    let comparison = compare_with_truth(&solution.unwrap(), truth).unwrap();
    assert!(comparison.horizontal_error_m < 3.0, "{comparison:?}");
}

#[test]
fn observations_replay_in_acquisition_order_whatever_order_they_arrive() {
    let (start, _) = eastbound(0);
    let mut navigation =
        NavigationLoop::new(FusionConfig::default(), MasterClockBridge::default(), HOLD).unwrap();
    navigation.offer(position_fix(1, after(200), &start));
    navigation.offer(position_fix(0, after(100), &start));
    navigation.advance(after(600)).unwrap();
    assert_eq!(navigation.accepted(), 2);
    assert_eq!(
        navigation.filter().rejections().sequence_not_admitted,
        0,
        "the later sequence ran first"
    );
}

#[test]
fn nothing_reaches_the_filter_before_its_hold_is_over() {
    let (start, _) = eastbound(0);
    let mut navigation =
        NavigationLoop::new(FusionConfig::default(), MasterClockBridge::default(), HOLD).unwrap();
    navigation.offer(position_fix(0, after(0), &start));
    assert!(navigation.advance(after(299)).unwrap().is_none());
    assert_eq!(navigation.accepted(), 0);
    assert!(navigation.advance(after(300)).unwrap().is_some());
    assert_eq!(navigation.accepted(), 1);
}

#[test]
fn a_hold_the_filter_would_reject_as_stale_is_refused() {
    let hold = TickSpan::from_ticks(TICKS_PER_SECOND / 2);
    let refused = NavigationLoop::new(FusionConfig::default(), MasterClockBridge::default(), hold);
    assert!(matches!(
        refused,
        Err(NavigateBridgeError::HoldReachesStaleness { .. })
    ));
}

/// The default prior is the reason this setting exists: without it a fast
/// vehicle's velocity aiding never gets in.
#[test]
fn the_default_velocity_prior_refuses_an_aircraft_and_the_airborne_one_admits_it() {
    let (start, truth) = eastbound(0);
    let east = LocalFrame::at(&start).east() * SPEED_M_S;
    let admitted_with = |config: FusionConfig| {
        let mut navigation =
            NavigationLoop::new(config, MasterClockBridge::default(), TickSpan::ZERO).unwrap();
        let mut aiding = KinematicAiding::new(
            MasterClockBridge::default(),
            SourceId::new(2),
            SourceEpoch::new(0),
            0.1,
        );
        navigation.offer(position_fix(0, after(0), &start));
        navigation.offer(aiding.observe(after(10), truth, east).unwrap());
        navigation.advance(after(10)).unwrap();
        navigation.filter().rejections().innovation_gate
    };
    assert_eq!(admitted_with(FusionConfig::default()), 1);
    assert_eq!(
        admitted_with(airborne_fusion_config(SPEED_M_S, 1.0, 0.05)),
        0
    );
}

/// A 6.8 m/s² turn with noisy 10 Hz aiding, the case a loiter flew into: under
/// the default process noise one noise spike is refused, the next innovation is
/// larger, and aiding drops out.
#[test]
fn a_turn_keeps_its_velocity_aiding_under_the_airborne_process_noise() {
    const SIGMA_M_S: f64 = 0.3;
    const SPEED: f64 = 32.0;
    const RADIUS_M: f64 = 150.0;
    let (start, _) = eastbound(0);
    let frame = LocalFrame::at(&start);
    let refused_with = |config: FusionConfig| {
        let mut navigation =
            NavigationLoop::new(config, MasterClockBridge::default(), TickSpan::ZERO).unwrap();
        let mut aiding = KinematicAiding::new(
            MasterClockBridge::default(),
            SourceId::new(2),
            SourceEpoch::new(0),
            SIGMA_M_S,
        );
        let mut noise = WhiteNoise::from_seed(5);
        navigation.offer(position_fix(0, after(0), &start));
        for millis in (100..120_000).step_by(100) {
            let angle = SPEED / RADIUS_M * millis as f64 / 1_000.0;
            let velocity = (frame.north() * angle.cos() + frame.east() * angle.sin()) * SPEED
                + noise.next_vec3(DVec3::splat(SIGMA_M_S));
            let truth = geodetic_to_ecef(&start);
            navigation.offer(aiding.observe(after(millis), truth, velocity).unwrap());
            navigation.advance(after(millis)).unwrap();
        }
        navigation.filter().rejections().innovation_gate
    };
    let default_process_noise = FusionConfig {
        initial_velocity_variance_m2_per_s2: 40_000.0,
        ..FusionConfig::default()
    };
    assert!(refused_with(default_process_noise) > 100);
    assert_eq!(refused_with(airborne_fusion_config(200.0, 10.0, 0.1)), 0);
}
