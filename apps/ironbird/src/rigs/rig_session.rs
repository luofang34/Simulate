use sim_core::{
    Block, SensorChannelId, SensorExposureEvent, SimInstant, SimSession, TICKS_PER_SECOND,
    TickSpan, VehicleState,
};

use crate::IronbirdError;

/// The optical sensor channel every rig registers, for camera-frame
/// consumers such as visual navigation.
pub const OPTICAL_CHANNEL: SensorChannelId = SensorChannelId(0);

/// Exposure period of [`OPTICAL_CHANNEL`]: 30 Hz on the master clock.
pub const OPTICAL_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 30);

/// A rig's plant inside a [`SimSession`], stepped exactly one physics period
/// per closed-loop iteration so controller exchanges stay on the step grid,
/// with the optical channel's exposures tallied as they fall due.
pub(super) struct RigSession<B>
where
    B: Block<Output = VehicleState>,
    B::Input: Clone,
{
    session: SimSession<B>,
    period: TickSpan,
    optical_exposures: u64,
    last_optical_exposure: Option<SensorExposureEvent>,
}

impl<B> RigSession<B>
where
    B: Block<Output = VehicleState>,
    B::Input: Clone,
{
    pub(super) fn new(plant: B, initial_input: B::Input) -> Result<Self, IronbirdError> {
        let period = plant.period();
        let mut session = SimSession::new(plant, initial_input);
        session.register_sensor(OPTICAL_CHANNEL, OPTICAL_PERIOD)?;
        Ok(Self {
            session,
            period,
            optical_exposures: 0,
            last_optical_exposure: None,
        })
    }

    /// The physics instant the next step starts at.
    pub(super) fn now(&self) -> SimInstant {
        self.session.physics_time()
    }

    /// `origin` moved forward by the session time elapsed so far, for a
    /// controller whose clock ran ahead of the plant during warm-up.
    pub(super) fn clock_from(&self, origin: SimInstant) -> SimInstant {
        origin.advanced(self.now().duration_since(SimInstant::START))
    }

    pub(super) fn plant(&self) -> &B {
        self.session.block()
    }

    pub(super) fn period(&self) -> TickSpan {
        self.period
    }

    pub(super) fn truth_state(&self) -> VehicleState {
        self.session.truth_state()
    }

    /// Latches `input` at the current instant and runs one physics step.
    pub(super) fn step(&mut self, input: B::Input) -> Result<(), IronbirdError> {
        self.session.latch_now(input);
        self.session.advance(self.period)?;
        for exposure in self.session.take_exposures() {
            if exposure.channel == OPTICAL_CHANNEL {
                self.optical_exposures = self.optical_exposures.wrapping_add(1);
                self.last_optical_exposure = Some(exposure);
            }
        }
        Ok(())
    }

    pub(super) fn optical_exposures(&self) -> u64 {
        self.optical_exposures
    }

    pub(super) fn last_optical_exposure(&self) -> Option<SensorExposureEvent> {
        self.last_optical_exposure
    }
}
