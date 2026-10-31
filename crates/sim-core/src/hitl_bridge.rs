use std::collections::BTreeMap;

use crate::{SimInstant, TickSpan};

#[cfg(test)]
mod tests;

/// Why an external node could not answer.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("external endpoint failed: {detail}")]
pub struct EndpointError {
    /// What the endpoint reported.
    pub detail: String,
}

/// A response from an external node, stamped with the master-clock instant it
/// reaches the simulation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StampedResponse<R> {
    /// When the response is delivered.
    pub deliver_at: SimInstant,
    /// The response.
    pub response: R,
}

/// An external node standing in for an internal subsystem: a companion
/// computer processing rendered frames, an external flight controller.
pub trait ExternalEndpoint {
    /// What the simulation sends.
    type Stimulus;
    /// What the node returns.
    type Response;

    /// Sends `stimulus` and waits for the node's answer, for
    /// [`HitlSyncMode::LockstepBarrier`].
    fn exchange_blocking(
        &mut self,
        at: SimInstant,
        stimulus: &Self::Stimulus,
    ) -> Result<Self::Response, EndpointError>;

    /// Sends `stimulus` without waiting, for [`HitlSyncMode::RealTimeStamped`].
    fn post(&mut self, at: SimInstant, stimulus: &Self::Stimulus) -> Result<(), EndpointError>;

    /// The next response that has arrived, if any, without waiting.
    fn try_receive(&mut self) -> Result<Option<StampedResponse<Self::Response>>, EndpointError>;
}

/// How a bridge keeps the simulation and an external node in step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HitlSyncMode {
    /// The simulation stops at each stimulus until the node answers, then
    /// delivers the answer exactly `processing_delay` later. Results are
    /// deterministic however long the node really took.
    LockstepBarrier {
        /// Master-clock time charged to the node.
        processing_delay: TickSpan,
    },
    /// The simulation never waits. Responses carry their own delivery stamps;
    /// when several fall due together the newest wins and the others count as
    /// dropped.
    RealTimeStamped,
}

/// A delivered response and how late it was.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeliveredResponse<R> {
    /// The instant the response was stamped for.
    pub deliver_at: SimInstant,
    /// `now` minus [`DeliveredResponse::deliver_at`] at delivery.
    pub staleness: TickSpan,
    /// The response.
    pub response: R,
}

/// Health of a bridge since it was built.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BridgeStats {
    /// Responses handed to the simulation.
    pub delivered: u64,
    /// Responses superseded by a newer one, or stamped earlier than one
    /// already delivered.
    pub dropped: u64,
    /// Largest staleness of any delivered response.
    pub max_staleness: TickSpan,
}

/// Why a bridge operation failed.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum BridgeError {
    /// The external node failed.
    #[error("exchange at {at:?} failed")]
    Endpoint {
        /// The instant of the exchange.
        at: SimInstant,
        /// The node's failure.
        #[source]
        source: EndpointError,
    },
}

/// Everything a bridge carries between steps. The endpoint itself lies outside
/// the simulation and is not part of it.
#[derive(Debug, Clone, PartialEq)]
pub struct BridgeSnapshot<R> {
    mailbox: BTreeMap<(SimInstant, u64), R>,
    next_sequence: u64,
    newest_delivered: Option<SimInstant>,
    stats: BridgeStats,
}

/// Connects a simulation to an [`ExternalEndpoint`] in place of an internal
/// subsystem, without the rest of the vehicle knowing which it is talking to.
pub struct ExternalNodeBridge<E: ExternalEndpoint> {
    endpoint: E,
    mode: HitlSyncMode,
    mailbox: BTreeMap<(SimInstant, u64), E::Response>,
    next_sequence: u64,
    newest_delivered: Option<SimInstant>,
    stats: BridgeStats,
}

impl<E: ExternalEndpoint> ExternalNodeBridge<E> {
    /// A bridge to `endpoint` with nothing in flight.
    pub fn new(endpoint: E, mode: HitlSyncMode) -> Self {
        Self {
            endpoint,
            mode,
            mailbox: BTreeMap::new(),
            next_sequence: 0,
            newest_delivered: None,
            stats: BridgeStats::default(),
        }
    }

    /// Health counters.
    pub fn stats(&self) -> BridgeStats {
        self.stats
    }

    /// The bridge's synchronisation mode.
    pub fn mode(&self) -> HitlSyncMode {
        self.mode
    }

    /// Sends `stimulus` at `at`. In lockstep this blocks until the node has
    /// answered.
    pub fn stimulate(&mut self, at: SimInstant, stimulus: &E::Stimulus) -> Result<(), BridgeError> {
        let endpoint_error = |source| BridgeError::Endpoint { at, source };
        match self.mode {
            HitlSyncMode::LockstepBarrier { processing_delay } => {
                let response = self
                    .endpoint
                    .exchange_blocking(at, stimulus)
                    .map_err(endpoint_error)?;
                self.file(at.advanced(processing_delay), response);
            }
            HitlSyncMode::RealTimeStamped => {
                self.endpoint.post(at, stimulus).map_err(endpoint_error)?;
            }
        }
        Ok(())
    }

    /// Everything to hand over at `now`, oldest first: every due response in
    /// lockstep, the newest due one in real time.
    pub fn poll(
        &mut self,
        now: SimInstant,
    ) -> Result<Vec<DeliveredResponse<E::Response>>, BridgeError> {
        let source_error = |source| BridgeError::Endpoint { at: now, source };
        while let Some(stamped) = self.endpoint.try_receive().map_err(source_error)? {
            self.file(stamped.deliver_at, stamped.response);
        }
        let mut due = self.take_due(now);
        if self.mode == HitlSyncMode::RealTimeStamped {
            let superseded = due.len().saturating_sub(1);
            due.drain(..superseded);
            self.stats.dropped = self.stats.dropped.wrapping_add(superseded as u64);
        }
        Ok(due
            .into_iter()
            .filter_map(|(deliver_at, response)| self.deliver(now, deliver_at, response))
            .collect())
    }

    fn file(&mut self, deliver_at: SimInstant, response: E::Response) {
        self.mailbox
            .insert((deliver_at, self.next_sequence), response);
        self.next_sequence = self.next_sequence.wrapping_add(1);
    }

    fn take_due(&mut self, now: SimInstant) -> Vec<(SimInstant, E::Response)> {
        let mut due = Vec::new();
        while let Some(entry) = self.mailbox.first_entry() {
            let (deliver_at, _) = *entry.key();
            if deliver_at > now {
                break;
            }
            due.push((deliver_at, entry.remove()));
        }
        due
    }

    /// Counts one response through the gate: anything older than what the
    /// simulation has already seen would take its clock backwards.
    fn deliver(
        &mut self,
        now: SimInstant,
        deliver_at: SimInstant,
        response: E::Response,
    ) -> Option<DeliveredResponse<E::Response>> {
        if self
            .newest_delivered
            .is_some_and(|newest| deliver_at < newest)
        {
            self.stats.dropped = self.stats.dropped.wrapping_add(1);
            return None;
        }
        let staleness = now.duration_since(deliver_at);
        self.newest_delivered = Some(deliver_at);
        self.stats.delivered = self.stats.delivered.wrapping_add(1);
        self.stats.max_staleness = self.stats.max_staleness.max(staleness);
        Some(DeliveredResponse {
            deliver_at,
            staleness,
            response,
        })
    }
}

impl<E: ExternalEndpoint> ExternalNodeBridge<E>
where
    E::Response: Clone,
{
    /// Captures everything in flight.
    pub fn snapshot(&self) -> BridgeSnapshot<E::Response> {
        BridgeSnapshot {
            mailbox: self.mailbox.clone(),
            next_sequence: self.next_sequence,
            newest_delivered: self.newest_delivered,
            stats: self.stats,
        }
    }

    /// Restores state captured by [`ExternalNodeBridge::snapshot`].
    pub fn restore(&mut self, snapshot: &BridgeSnapshot<E::Response>) {
        self.mailbox.clone_from(&snapshot.mailbox);
        self.next_sequence = snapshot.next_sequence;
        self.newest_delivered = snapshot.newest_delivered;
        self.stats = snapshot.stats;
    }
}
