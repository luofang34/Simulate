use super::GuidanceSource;
use super::report::ModelStatement;

/// Navigate's own code in the trial, and what each stage runs.
const NAVIGATE: [(&str, &str); 5] = [
    (
        "navigation filter",
        "navigate_onboard::ReferenceNavigator linked as a navigate_onboard::Navigator: navigate_fusion::InertialNavigationFilter, 22 error states, on WGS84, fed the declared sensor suite's inputs; every sensor set feeds this one composition, and a missing or withheld sensor sends no input",
    ),
    (
        "route execution",
        "navigate_onboard::RouteDirector linked as a navigate_onboard::FlightDirector: navigate_route::PlanExecution over a navigate_contract::FlightPlan on WGS84, leg sequencing, capture radius and fly-by turn anticipation",
    ),
    (
        "guidance",
        "the flight director's navigate_guidance::guide deviation tracking, evaluated on the true state: the flight is open loop and identical for every sensor set",
    ),
    (
        "star tracker",
        "navigate_startrack: detection in the rendered counts on Simulate's side; inside the navigator, lost-in-space and prior-aided identification (prior from its own solution at mid-exposure), attitude solve, line-of-sight observations",
    ),
    (
        "sky ephemeris",
        "navigate_celestial analytic ephemeris: catalogue star places and the topocentric Sun",
    ),
];

/// Simulate's models in the trial.
const SIMULATED: [(&str, &str); 8] = [
    (
        "vehicle",
        "sim_core::KinematicModel: stick to body rate (at most 1.0, 0.35, 0.2 rad/s), throttle to 30-120 m/s, velocity along the nose, no aerodynamics or wind; flown by Simulate's attitude autopilot under a bank- and climb-limited route follower",
    ),
    (
        "imu",
        "sim_navigate::SimulatedImu: exact coning and sculling increments of the true motion at 120 Hz through a seeded sim_devices::ImuErrorModel of the requested grade; truth gravity is the WGS84 J2 field the filter also uses",
    ),
    (
        "initial alignment",
        "sim_navigate::InitialAlignment: the true state plus seeded errors of the declared 1-sigma, declared as the filter's prior",
    ),
    (
        "gnss",
        "position-domain receiver, not a pseudorange or constellation model: true antenna position (at the centre of gravity) plus white noise plus a first-order Gauss-Markov error, true velocity plus white noise; the declared covariance is the marginal white plus correlated variance, and the filter has no state for the correlated part",
    ),
    (
        "barometric altimeter",
        "ellipsoidal height plus a first-order Gauss-Markov bias plus white noise, the error the filter's barometric bias state models with the same statistics; the BMP390 device model is not used because it quantises pressure without noise or bias",
    ),
    (
        "star camera",
        "sim_render::StarCamera radiometric 1024 x 1024, 20 degree tracker on a roof mount, moving 0.1 s exposure from the true poses; sky brightness from the Sun's elevation and the line of sight (night, twilight, day table with airmass and Rayleigh phase), no Moon, cloud or aerosol",
    ),
    (
        "reproducibility",
        "a result is bit-reproducible on one platform; across platforms it agrees only to numeric tolerances, not to the bit, because floating-point functions and fused multiply-adds differ between targets",
    ),
    (
        "scoring",
        "errors are truth minus estimate at the centre of gravity, in the estimate's local NED axes; NEES uses the filter's published covariances",
    ),
];

/// Guidance in closed loop.
const CLOSED_LOOP_GUIDANCE: (&str, &str) = (
    "guidance",
    "the flight director's navigate_guidance::guide deviation tracking on each set's own navigation solution, and Simulate's route follower's attitude loops on its attitude and the IMU's sensed rates: closed loop, each set flies its own truth, and sensor noise is drawn per sample so sets meet the same realisation at the same route time",
);

/// What this trial has measured of Navigate's behaviour, in plain words.
const FINDINGS: [(&str, &str); 6] = [
    (
        "finding: correlated GNSS error",
        "navigate_fusion::InertialNavigationFilter has no state for a time-correlated GNSS error; against the default receiver (0.6 m white plus 1.2 m with a 300 s correlation time per horizontal axis) the GNSS-aided tactical run averages position NEES 51.3 and the outage run 28.3, against 3.2 with the correlated term removed",
    ),
    (
        "finding: barometric updates move the horizontal estimate",
        "once the horizontal 1-sigma reaches about 800 m (tactical INS with baro, no position aid), each barometric altitude update moves the horizontal estimate by 5 to 80 m, so the estimated track jitters by up to about 150 m",
    ),
    (
        "finding: a passed fly-over fix sequences on the perpendicular",
        "navigate_route::PlanExecution sequences a fly-over waypoint whose position passed it outside the capture radius when the position crosses the perpendicular through the fix, and reports the cross-track; measured: an INS-only navigator kilometres lost passed two fly-over fixes 120 m and 123 m off with a 100 m radius and completed the route, truly 17.7 km from the last fix",
    ),
    (
        "finding: guidance refuses above the integrity floor",
        "with Navigate's default floor (Degraded) guidance refuses a solution once its horizontal 1-sigma exceeds 50 m (IntegrityBelowFloor), and the vehicle holds its track; GNSS-denied closed-loop flight therefore needs an aiding means that keeps the 1-sigma below 50 m or an explicitly lower floor",
    ),
    (
        "finding: tick advances the solution sequence",
        "InertialNavigationFilter::tick publishes a solution with a new sequence number without changing the state, so a consumer that reads a solution (the star tracker's prior) changes the sequence numbers of later solutions but nothing else",
    ),
    (
        "finding: stars and short-horizon horizontal error",
        "measured in this trial, open loop, night, tactical and navigation grades: over a 94-minute loop stars reduce horizontal error at 90 minutes from 120 km to 8 km (tactical) and from 2.1 km to 90 m (navigation); at short horizons on the tactical grade they can increase it on some seeds: at 10 minutes 583 m without against 1371 m with, and on three further seeds 1394 m against 2926 m, 289 m against 1916 m and 3077 m against 2995 m; a likely cause, unverified, is that without stars the tilt error partly offsets the accelerometer bias",
    ),
];

/// What every component of the trial is, with guidance as `source` runs it.
pub(super) fn model_statements(source: GuidanceSource) -> Vec<ModelStatement> {
    let tagged = |origin: &'static str, entries: &'static [(&'static str, &'static str)]| {
        entries
            .iter()
            .map(move |(component, statement)| ModelStatement {
                component: (*component).into(),
                origin: origin.into(),
                statement: (*statement).into(),
            })
    };
    let navigate: Vec<ModelStatement> = tagged("navigate", &NAVIGATE)
        .map(|statement| match source {
            GuidanceSource::Estimate if statement.component == CLOSED_LOOP_GUIDANCE.0 => {
                ModelStatement {
                    statement: CLOSED_LOOP_GUIDANCE.1.into(),
                    ..statement
                }
            }
            _ => statement,
        })
        .collect();
    navigate
        .into_iter()
        .chain(tagged("simulated", &SIMULATED))
        .chain(tagged("measured", &FINDINGS))
        .collect()
}
