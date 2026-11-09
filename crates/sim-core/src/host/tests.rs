// The tests compare both hosts, and only native targets have threads.
#![cfg(not(target_arch = "wasm32"))]

use std::sync::Arc;
use std::time::Duration;

use sim_geo::Geodetic;

use super::*;
use crate::{
    BlockError, ControlInputs, KinematicInput, KinematicModel, SensorChannelId, TICKS_PER_SECOND,
};

const CAMERA: SensorChannelId = SensorChannelId(3);
const CAMERA_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 30);
const WAIT: Duration = Duration::from_secs(30);

fn start_state() -> VehicleState {
    VehicleState::level_flight(&Geodetic::from_degrees(39.1582, -75.5244, 800.0), 0.3, 60.0)
}

fn session() -> SimSession<KinematicModel> {
    let mut session = SimSession::new(
        KinematicModel::new(start_state()),
        KinematicInput::default(),
    );
    session.register_sensor(CAMERA, CAMERA_PERIOD).unwrap();
    session
}

/// Stick inputs stamped off every frame and physics grid, as a pilot's
/// events arrive.
fn inputs() -> Vec<(SimInstant, KinematicInput)> {
    (0_u64..12)
        .map(|k| {
            let s = k as f64;
            let controls = ControlInputs {
                roll: (s * 0.7).sin(),
                pitch: 0.4 * (s * 1.3).cos(),
                yaw: 0.2 * (s * 0.5).sin(),
                throttle: 0.3 + 0.05 * s,
            };
            (
                SimInstant::from_nanos(k * 413_000_029 + 7),
                KinematicInput { controls },
            )
        })
        .collect()
}

/// Frame spans jittering between roughly 4 ms and 33 ms, the way a
/// vsync-less window's frame times wander.
fn jittery_spans(count: usize) -> Vec<TickSpan> {
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    (0..count)
        .map(|_| {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let jitter = (state >> 33) % (TICKS_PER_SECOND / 34);
            TickSpan::from_ticks(TICKS_PER_SECOND / 250 + jitter)
        })
        .collect()
}

/// Drives `host` like a frame loop: each frame submits the inputs that fell
/// due since the last one, requests the frame's span, and records the frame
/// published for that request.
fn drive<H>(host: &mut H, spans: &[TickSpan]) -> Vec<Arc<DisplayFrame>>
where
    H: SimHost<Input = KinematicInput>,
{
    let mut pending = inputs().into_iter().peekable();
    let mut frames = vec![host.latest().unwrap()];
    for span in spans {
        let target = host.requested().advanced(*span);
        while let Some((at, input)) = pending.next_if(|(at, _)| *at <= target) {
            host.submit_at(at, input).unwrap();
        }
        let seq = host.request_until(target).unwrap();
        let frame = host.frame_blocking(seq, WAIT).unwrap();
        assert_eq!(frame.seq, seq);
        frames.push(frame);
    }
    frames
}

/// Debug text keeps the sign of zero and every digit, so equal text is
/// equal bits.
fn bits<T: std::fmt::Debug>(value: &T) -> String {
    format!("{value:?}")
}

#[test]
fn inline_and_threaded_hosts_publish_bit_identical_frames_and_exposures() {
    let spans = jittery_spans(400);
    let mut inline = InlineHost::new(session());
    let inline_frames = drive(&mut inline, &spans);
    let inline_exposures = inline.take_exposures();

    let mut threaded = ThreadedHost::spawn(session(), "sim-host-determinism").unwrap();
    let threaded_frames = drive(&mut threaded, &spans);
    let threaded_exposures = threaded.take_exposures();
    let report = threaded.shutdown(WAIT).unwrap();

    assert_eq!(report.joined, vec!["sim-host-determinism".to_owned()]);
    assert!(report.stuck.is_empty());
    assert!(inline_exposures.len() > 100, "{}", inline_exposures.len());
    assert_eq!(bits(&threaded_frames), bits(&inline_frames));
    assert_eq!(bits(&threaded_exposures), bits(&inline_exposures));
    let last = inline_frames.last().unwrap();
    assert_ne!(last.current, start_state());
}

#[test]
fn threaded_truth_does_not_depend_on_how_requests_are_sliced() {
    let fine = jittery_spans(400);
    let end = fine
        .iter()
        .fold(SimInstant::START, |at, span| at.advanced(*span));
    let mut sliced = ThreadedHost::spawn(session(), "sim-host-sliced").unwrap();
    let sliced_last = drive(&mut sliced, &fine).pop().unwrap();

    let mut whole = ThreadedHost::spawn(session(), "sim-host-whole").unwrap();
    for (at, input) in inputs() {
        whole.submit_at(at, input).unwrap();
    }
    let seq = whole.request_until(end).unwrap();
    let whole_last = whole.frame_blocking(seq, WAIT).unwrap();

    assert_eq!(bits(&whole_last.current), bits(&sliced_last.current));
    assert_eq!(whole_last.physics_time, sliced_last.physics_time);
    assert_eq!(
        bits(&whole.take_exposures()),
        bits(&sliced.take_exposures())
    );
}

#[test]
fn latest_skips_frames_but_never_exposures() {
    let mut host = ThreadedHost::spawn(session(), "sim-host-latest").unwrap();
    let mut last = FrameSeq(0);
    for _ in 0..90 {
        last = host
            .request_by(TickSpan::from_ticks(TICKS_PER_SECOND / 90))
            .unwrap();
    }
    let frame = host.frame_blocking(last, WAIT).unwrap();
    assert_eq!(host.latest().unwrap().seq, last);
    assert_eq!(
        frame.now,
        SimInstant::START.advanced(TickSpan::from_ticks(TICKS_PER_SECOND))
    );
    let exposures = host.take_exposures();
    assert_eq!(exposures.len(), 30);
    assert!(exposures.iter().all(|e| e.channel == CAMERA));
    host.shutdown(WAIT).unwrap();
}

/// A plant whose step announces itself and then waits for the test to let
/// it finish, so the test controls exactly when the simulation thread is
/// busy.
#[derive(Debug)]
struct GatedBlock {
    entered: std::sync::mpsc::Sender<()>,
    release: std::sync::mpsc::Receiver<()>,
}

impl Block for GatedBlock {
    type Input = ();
    type Output = VehicleState;
    type Snapshot = ();

    fn period(&self) -> TickSpan {
        TickSpan::from_ticks(TICKS_PER_SECOND / 100)
    }

    fn step(&mut self, _input: &()) -> Result<(), BlockError> {
        self.entered.send(()).ok();
        self.release.recv().ok();
        Ok(())
    }

    fn output(&self) -> VehicleState {
        start_state()
    }

    fn snapshot(&self) {}

    fn restore(&mut self, _snapshot: &()) {}
}

#[test]
fn shutdown_reports_a_thread_stuck_in_a_step_and_joins_a_free_one() {
    let (entered_tx, entered) = std::sync::mpsc::channel();
    let (release, release_rx) = std::sync::mpsc::channel();
    let block = GatedBlock {
        entered: entered_tx,
        release: release_rx,
    };
    let mut host = ThreadedHost::spawn(SimSession::new(block, ()), "sim-host-gated").unwrap();
    host.request_by(TickSpan::from_ticks(TICKS_PER_SECOND / 100))
        .unwrap();
    entered.recv_timeout(WAIT).unwrap();

    let report = host.shutdown(Duration::from_millis(50)).unwrap();
    assert_eq!(report.stuck, vec!["sim-host-gated".to_owned()]);
    assert!(report.joined.is_empty());

    // Released, the detached thread finishes its step, reads the queued
    // shutdown, and exits.
    release.send(()).unwrap();

    let mut free = ThreadedHost::spawn(session(), "sim-host-free").unwrap();
    let report = free.shutdown(WAIT).unwrap();
    assert_eq!(report.joined, vec!["sim-host-free".to_owned()]);
    assert!(report.stuck.is_empty());
}

fn assert_closed<H: SimHost<Input = KinematicInput>>(mut host: H) {
    host.shutdown(WAIT).unwrap();
    assert!(matches!(
        host.request_by(TickSpan::from_ticks(1)),
        Err(HostError::ShutDown)
    ));
    assert!(matches!(
        host.submit_at(SimInstant::START, KinematicInput::default()),
        Err(HostError::ShutDown)
    ));
    assert!(matches!(host.latest(), Err(HostError::ShutDown)));
    assert!(matches!(host.shutdown(WAIT), Err(HostError::ShutDown)));
}

#[test]
fn commands_after_shutdown_are_refused_with_a_typed_error() {
    assert_closed(InlineHost::new(session()));
    assert_closed(ThreadedHost::spawn(session(), "sim-host-closed").unwrap());
}

/// A plant that refuses its third step.
#[derive(Debug, Default)]
struct FailingBlock {
    steps: u32,
}

impl Block for FailingBlock {
    type Input = ();
    type Output = VehicleState;
    type Snapshot = ();

    fn period(&self) -> TickSpan {
        TickSpan::from_ticks(TICKS_PER_SECOND / 100)
    }

    fn step(&mut self, _input: &()) -> Result<(), BlockError> {
        self.steps = self.steps.wrapping_add(1);
        if self.steps == 3 {
            return Err(BlockError::NonFiniteState {
                block: "failing",
                detail: "third step".to_owned(),
            });
        }
        Ok(())
    }

    fn output(&self) -> VehicleState {
        start_state()
    }

    fn snapshot(&self) {}

    fn restore(&mut self, _snapshot: &()) {}
}

fn assert_fault_sticks<H: SimHost<Input = ()>>(mut host: H) {
    let step = TickSpan::from_ticks(TICKS_PER_SECOND / 100);
    let seq = host.request_by(step).unwrap();
    host.frame_blocking(seq, WAIT).unwrap();
    // The failing request may be refused at once (inline) or accepted and
    // failed on the simulation thread (threaded); either way the fault
    // surfaces and every later command reports it.
    let outcome = host
        .request_by(TickSpan::from_ticks(step.ticks() * 5))
        .and_then(|seq| host.frame_blocking(seq, WAIT));
    let Err(HostError::Session(fault)) = outcome else {
        panic!("expected a block fault, got {outcome:?}");
    };
    let SessionError::Block { at, .. } = *fault else {
        panic!("expected a block fault, got {fault:?}");
    };
    assert_eq!(
        at,
        SimInstant::START.advanced(TickSpan::from_ticks(step.ticks() * 2))
    );
    assert!(matches!(
        host.request_by(step),
        Err(HostError::Session(fault)) if matches!(*fault, SessionError::Block { .. })
    ));
    host.shutdown(WAIT).unwrap();
}

#[test]
fn a_block_fault_stops_both_hosts_with_the_session_error() {
    assert_fault_sticks(InlineHost::new(SimSession::new(
        FailingBlock::default(),
        (),
    )));
    assert_fault_sticks(
        ThreadedHost::spawn(
            SimSession::new(FailingBlock::default(), ()),
            "sim-host-fault",
        )
        .unwrap(),
    );
}

#[test]
fn waiting_for_an_unissued_frame_is_refused() {
    let mut host = ThreadedHost::spawn(session(), "sim-host-unissued").unwrap();
    assert!(matches!(
        host.frame_blocking(FrameSeq(1), WAIT),
        Err(HostError::NotRequested { .. })
    ));
    host.shutdown(WAIT).unwrap();
}

#[test]
fn the_jsbsim_plant_can_move_to_a_simulation_thread() {
    fn host_on_thread<B: Block<Output = VehicleState> + Send + 'static>() {}
    host_on_thread::<crate::JsbsimFdmBlock>();
}
