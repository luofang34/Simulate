//! A desktop frame loop over a [`ThreadedHost`]: wall-clock frame times go
//! through a [`RealTimeClock`] into requested instants, the loop reads only
//! the newest published frame and never waits on physics, and the recorded
//! spans replayed through an [`InlineHost`] must reach the same bits. Exits
//! non-zero if they do not.

// Browsers give a page one thread, so there the example has nothing to run.
#[cfg(not(target_arch = "wasm32"))]
mod native {
    use std::time::{Duration, Instant};

    use sim_core::{
        ControlInputs, DisplayFrame, HostError, InlineHost, KinematicInput, KinematicModel,
        RealTimeClock, SensorChannelId, SensorExposureEvent, SimHost, SimInstant, SimSession,
        TICKS_PER_SECOND, ThreadedHost, TickSpan, VehicleState,
    };
    use sim_geo::Geodetic;

    const RUN: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND);
    const CAMERA: SensorChannelId = SensorChannelId(0);
    const CAMERA_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 30);
    /// Stand-in for the time a renderer spends drawing one frame.
    const DRAW: Duration = Duration::from_millis(4);
    const WAIT: Duration = Duration::from_secs(10);

    #[derive(Debug, thiserror::Error)]
    pub(crate) enum LoopError {
        #[error(transparent)]
        Host(#[from] HostError),
        #[error("threaded and inline hosts diverged: {what}")]
        Diverged { what: &'static str },
    }

    fn session() -> Result<SimSession<KinematicModel>, HostError> {
        let start =
            VehicleState::level_flight(&Geodetic::from_degrees(39.16, -75.52, 800.0), 0.3, 60.0);
        let mut session = SimSession::new(KinematicModel::new(start), KinematicInput::default());
        session.register_sensor(CAMERA, CAMERA_PERIOD)?;
        Ok(session)
    }

    fn bank_input() -> KinematicInput {
        KinematicInput {
            controls: ControlInputs {
                roll: 0.4,
                throttle: 0.7,
                ..ControlInputs::default()
            },
        }
    }

    /// The interactive loop; returns the spans it requested and what the host
    /// finally published.
    fn fly_threaded() -> Result<(Vec<TickSpan>, DisplayFrame, Vec<SensorExposureEvent>), LoopError>
    {
        let mut host = ThreadedHost::spawn(session()?, "sim-host-example")?;
        host.submit_at(SimInstant::from_nanos(250_000_000), bank_input())?;
        let end = SimInstant::START.advanced(RUN);
        let mut clock = RealTimeClock::new(TickSpan::from_ticks(TICKS_PER_SECOND / 10));
        let mut spans = Vec::new();
        let mut last_frame = Instant::now();
        let mut seq = None;
        while host.requested() < end {
            let wall = last_frame.elapsed();
            last_frame = Instant::now();
            let remaining = end.duration_since(host.requested());
            let span = clock.advance(wall.as_secs_f64(), 1.0).min(remaining);
            seq = Some(host.request_by(span)?);
            spans.push(span);
            let _display = host.latest()?.display_state();
            std::thread::sleep(DRAW);
        }
        let frame = match seq {
            Some(seq) => host.frame_blocking(seq, WAIT)?,
            None => host.latest()?,
        };
        let exposures = host.take_exposures();
        let report = host.shutdown(WAIT)?;
        if !report.stuck.is_empty() {
            return Err(LoopError::Diverged {
                what: "simulation thread did not exit",
            });
        }
        Ok((spans, (*frame).clone(), exposures))
    }

    pub(crate) fn run() -> Result<(), LoopError> {
        let (spans, threaded_frame, threaded_exposures) = fly_threaded()?;

        let mut inline = InlineHost::new(session()?);
        inline.submit_at(SimInstant::from_nanos(250_000_000), bank_input())?;
        for span in &spans {
            inline.request_by(*span)?;
        }
        let inline_frame = inline.latest()?;
        let inline_exposures = inline.take_exposures();

        if format!("{:?}", *inline_frame) != format!("{threaded_frame:?}") {
            return Err(LoopError::Diverged {
                what: "final frame",
            });
        }
        if format!("{inline_exposures:?}") != format!("{threaded_exposures:?}") {
            return Err(LoopError::Diverged { what: "exposures" });
        }
        tracing::info!(
            frames = spans.len(),
            exposures = inline_exposures.len(),
            "threaded and inline hosts agree bit for bit"
        );
        Ok(())
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn main() -> Result<(), native::LoopError> {
    native::run()
}

#[cfg(target_arch = "wasm32")]
fn main() {}
