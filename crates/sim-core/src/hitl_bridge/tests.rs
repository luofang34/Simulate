use std::collections::VecDeque;

use super::*;

const DELAY: TickSpan = TickSpan::from_ticks(1_612_800_000);

fn at(ticks: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(ticks))
}

/// Doubles each stimulus, and records what it saw. In real-time mode, it hands
/// out whatever a test put in `inbox`.
#[derive(Default)]
struct Doubler {
    exchanged: Vec<(SimInstant, u32)>,
    posted: Vec<(SimInstant, u32)>,
    inbox: VecDeque<StampedResponse<u32>>,
    failing: bool,
}

impl ExternalEndpoint for Doubler {
    type Stimulus = u32;
    type Response = u32;

    fn exchange_blocking(&mut self, at: SimInstant, stimulus: &u32) -> Result<u32, EndpointError> {
        if self.failing {
            return Err(EndpointError {
                detail: "link down".to_owned(),
            });
        }
        self.exchanged.push((at, *stimulus));
        Ok(stimulus * 2)
    }

    fn post(&mut self, at: SimInstant, stimulus: &u32) -> Result<(), EndpointError> {
        self.posted.push((at, *stimulus));
        Ok(())
    }

    fn try_receive(&mut self) -> Result<Option<StampedResponse<u32>>, EndpointError> {
        Ok(self.inbox.pop_front())
    }
}

fn lockstep() -> ExternalNodeBridge<Doubler> {
    ExternalNodeBridge::new(
        Doubler::default(),
        HitlSyncMode::LockstepBarrier {
            processing_delay: DELAY,
        },
    )
}

fn real_time(inbox: impl IntoIterator<Item = (u64, u32)>) -> ExternalNodeBridge<Doubler> {
    let inbox = inbox
        .into_iter()
        .map(|(ticks, response)| StampedResponse {
            deliver_at: at(ticks),
            response,
        })
        .collect();
    ExternalNodeBridge::new(
        Doubler {
            inbox,
            ..Doubler::default()
        },
        HitlSyncMode::RealTimeStamped,
    )
}

#[test]
fn lockstep_delivers_at_exactly_the_processing_delay() {
    let mut bridge = lockstep();
    let stimulus_at = at(1_000);
    bridge.stimulate(stimulus_at, &21).unwrap();
    assert_eq!(bridge.endpoint.exchanged, [(stimulus_at, 21)]);

    let due_at = stimulus_at.advanced(DELAY);
    let one_tick_early = stimulus_at.advanced(TickSpan::from_ticks(DELAY.ticks() - 1));
    assert!(bridge.poll(one_tick_early).unwrap().is_empty());

    let delivered = bridge.poll(due_at).unwrap();
    assert_eq!(
        delivered,
        [DeliveredResponse {
            deliver_at: due_at,
            staleness: TickSpan::ZERO,
            response: 42,
        }]
    );
    assert!(bridge.poll(due_at).unwrap().is_empty());
}

#[test]
fn a_lockstep_response_is_never_lost_when_the_host_polls_late() {
    let mut bridge = lockstep();
    bridge.stimulate(at(0), &1).unwrap();
    bridge.stimulate(at(10), &2).unwrap();
    let late = at(0).advanced(DELAY).advanced(TickSpan::from_ticks(100));
    let delivered = bridge.poll(late).unwrap();

    assert_eq!(delivered.len(), 2);
    assert_eq!(delivered[0].response, 2);
    assert_eq!(delivered[1].response, 4);
    assert_eq!(delivered[0].staleness, TickSpan::from_ticks(100));
    assert_eq!(delivered[1].staleness, TickSpan::from_ticks(90));
    let stats = bridge.stats();
    assert_eq!(stats.delivered, 2);
    assert_eq!(stats.dropped, 0);
    assert_eq!(stats.max_staleness, TickSpan::from_ticks(100));
}

#[test]
fn zero_delay_lockstep_answers_in_the_same_instant() {
    let mut bridge = ExternalNodeBridge::new(
        Doubler::default(),
        HitlSyncMode::LockstepBarrier {
            processing_delay: TickSpan::ZERO,
        },
    );
    bridge.stimulate(at(5), &3).unwrap();
    assert_eq!(bridge.poll(at(5)).unwrap()[0].response, 6);
}

#[test]
fn a_failed_exchange_is_reported_with_its_instant() {
    let mut bridge = lockstep();
    bridge.endpoint.failing = true;
    let error = bridge.stimulate(at(7), &1).unwrap_err();
    assert_eq!(
        error,
        BridgeError::Endpoint {
            at: at(7),
            source: EndpointError {
                detail: "link down".to_owned(),
            },
        }
    );
    assert!(std::error::Error::source(&error).is_some());
}

#[test]
fn a_real_time_stimulus_does_not_wait_for_an_answer() {
    let mut bridge = real_time([]);
    bridge.stimulate(at(3), &9).unwrap();
    assert_eq!(bridge.endpoint.posted, [(at(3), 9)]);
    assert!(bridge.endpoint.exchanged.is_empty());
    assert!(bridge.poll(at(3)).unwrap().is_empty());
}

#[test]
fn a_real_time_response_waits_for_its_stamp() {
    let mut bridge = real_time([(100, 5)]);
    assert!(bridge.poll(at(99)).unwrap().is_empty());
    let delivered = bridge.poll(at(130)).unwrap();
    assert_eq!(delivered.len(), 1);
    assert_eq!(delivered[0].response, 5);
    assert_eq!(delivered[0].staleness, TickSpan::from_ticks(30));
    assert_eq!(bridge.stats().max_staleness, TickSpan::from_ticks(30));
}

#[test]
fn the_newest_due_response_wins_and_the_rest_count_as_dropped() {
    let mut bridge = real_time([(30, 3), (10, 1), (20, 2), (50, 5)]);
    let delivered = bridge.poll(at(40)).unwrap();
    assert_eq!(delivered.len(), 1);
    assert_eq!(delivered[0].response, 3);
    assert_eq!(bridge.stats().delivered, 1);
    assert_eq!(bridge.stats().dropped, 2);

    assert_eq!(bridge.poll(at(50)).unwrap()[0].response, 5);
    assert_eq!(bridge.stats().delivered, 2);
}

#[test]
fn a_response_older_than_one_already_delivered_is_dropped() {
    let mut bridge = real_time([(30, 3)]);
    assert_eq!(bridge.poll(at(30)).unwrap().len(), 1);
    bridge.endpoint.inbox.push_back(StampedResponse {
        deliver_at: at(20),
        response: 2,
    });
    assert!(bridge.poll(at(40)).unwrap().is_empty());
    assert_eq!(bridge.stats().dropped, 1);
}

#[test]
fn a_restored_bridge_redelivers_what_was_in_flight() {
    let mut bridge = lockstep();
    bridge.stimulate(at(0), &4).unwrap();
    let saved = bridge.snapshot();
    let due = at(0).advanced(DELAY);
    let first = bridge.poll(due).unwrap();
    bridge.restore(&saved);
    assert_eq!(bridge.stats().delivered, 0);
    assert_eq!(bridge.poll(due).unwrap(), first);
}

#[test]
fn advance_epoch_discards_queued_and_in_flight_replies_from_the_old_timeline() {
    let mut bridge = real_time([]);
    bridge.stimulate(at(10), &1).unwrap();
    bridge.stimulate(at(20), &2).unwrap();
    bridge.endpoint.inbox.push_back(StampedResponse {
        deliver_at: at(15),
        response: 100,
    });
    assert!(bridge.poll(at(12)).unwrap().is_empty());
    assert_eq!(bridge.mailbox.len(), 1);

    bridge.advance_epoch();
    assert_eq!(bridge.timeline_epoch(), 1);
    assert!(bridge.mailbox.is_empty());
    assert_eq!(bridge.stats().stale_epoch_discards, 1);

    bridge.endpoint.inbox.push_back(StampedResponse {
        deliver_at: at(25),
        response: 200,
    });
    bridge.stimulate(at(5), &3).unwrap();
    bridge.endpoint.inbox.push_back(StampedResponse {
        deliver_at: at(6),
        response: 6,
    });
    let delivered = bridge.poll(at(30)).unwrap();
    assert_eq!(delivered.len(), 1);
    assert_eq!(delivered[0].response, 6);
    assert_eq!(bridge.stats().stale_epoch_discards, 2);
    assert_eq!(bridge.stats().delivered, 1);
}

#[test]
fn a_new_epoch_delivers_responses_earlier_than_the_abandoned_timeline_reached() {
    let mut bridge = real_time([(300, 3)]);
    assert_eq!(bridge.poll(at(300)).unwrap().len(), 1);
    bridge.advance_epoch();
    bridge.endpoint.inbox.push_back(StampedResponse {
        deliver_at: at(40),
        response: 4,
    });
    let delivered = bridge.poll(at(50)).unwrap();
    assert_eq!(delivered.len(), 1);
    assert_eq!(delivered[0].response, 4);
    assert_eq!(bridge.stats().dropped, 0);
    assert_eq!(bridge.stats().stale_epoch_discards, 0);
}

#[test]
fn lockstep_epoch_change_drops_undelivered_answers_and_restore_keeps_the_epoch() {
    let mut bridge = lockstep();
    let saved = bridge.snapshot();
    bridge.stimulate(at(0), &4).unwrap();
    bridge.advance_epoch();
    assert!(bridge.poll(at(0).advanced(DELAY)).unwrap().is_empty());
    assert_eq!(bridge.stats().stale_epoch_discards, 1);

    bridge.stimulate(at(0), &5).unwrap();
    let epoch_one = bridge.snapshot();
    assert_eq!(epoch_one.timeline_epoch(), 1);
    assert_eq!(bridge.poll(at(0).advanced(DELAY)).unwrap()[0].response, 10);
    bridge.restore(&epoch_one);
    assert_eq!(bridge.timeline_epoch(), 1);
    assert_eq!(bridge.poll(at(0).advanced(DELAY)).unwrap()[0].response, 10);

    bridge.restore(&saved);
    assert_eq!(bridge.timeline_epoch(), 0);
    assert_eq!(bridge.stats(), BridgeStats::default());
}

#[test]
fn epoch_counter_wraps() {
    let mut bridge = lockstep();
    bridge.timeline_epoch = u32::MAX;
    bridge.advance_epoch();
    assert_eq!(bridge.timeline_epoch(), 0);
}
