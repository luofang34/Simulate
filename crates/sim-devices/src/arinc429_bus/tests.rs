use mbd_core::Arinc429Fields;

use super::*;

const CHANNEL: u8 = 2;
const HIGH_WORD_TICKS: u64 = 14_745_600;
const LOW_WORD_TICKS: u64 = 117_964_800;

fn word(data: u32) -> DigitalBusFrame {
    DigitalBusFrame::arinc429(
        CHANNEL,
        &Arinc429Fields {
            label: 0o203,
            sdi: 0,
            data,
            ssm: 0b11,
        },
    )
    .unwrap()
}

fn at(ticks: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(ticks))
}

#[test]
fn word_times_are_exact_tick_counts() {
    assert_eq!(Arinc429Speed::High.word_time().ticks(), HIGH_WORD_TICKS);
    assert_eq!(Arinc429Speed::Low.word_time().ticks(), LOW_WORD_TICKS);
    assert_eq!(
        Arinc429Speed::High.word_time(),
        TickSpan::period_of_ratio(HIGH_SPEED_BPS, ARINC429_WORD_BITS).unwrap()
    );
    assert_eq!(
        Arinc429Speed::Low.word_time(),
        TickSpan::period_of_ratio(LOW_SPEED_BPS, ARINC429_WORD_BITS).unwrap()
    );
    assert_eq!(Arinc429Speed::High.bit_time().ticks(), 460_800);
}

#[test]
fn a_word_arrives_a_word_time_after_it_starts_and_not_a_tick_sooner() {
    for (speed, word_ticks) in [
        (Arinc429Speed::High, HIGH_WORD_TICKS),
        (Arinc429Speed::Low, LOW_WORD_TICKS),
    ] {
        let mut bus = Arinc429Bus::new(CHANNEL, speed);
        let start = at(1_000);
        let done = bus.transmit(start, word(5)).unwrap();
        assert_eq!(done, at(1_000 + word_ticks));
        assert!(bus.take_due(at(1_000 + word_ticks - 1)).is_empty());
        let delivered = bus.take_due(done);
        assert_eq!(
            delivered,
            [Arinc429Delivery {
                started_at: start,
                completed_at: done,
                frame: word(5),
            }]
        );
        assert!(bus.take_due(done).is_empty());
    }
}

#[test]
fn words_queued_together_go_out_back_to_back_in_order() {
    let mut bus = Arinc429Bus::new(CHANNEL, Arinc429Speed::High);
    for data in 0..3 {
        bus.transmit(SimInstant::START, word(data)).unwrap();
    }
    let delivered = bus.take_due(at(3 * HIGH_WORD_TICKS));
    assert_eq!(delivered.len(), 3);
    for (index, delivery) in (0..).zip(&delivered) {
        assert_eq!(delivery.frame, word(index as u32));
        assert_eq!(delivery.started_at, at(index * HIGH_WORD_TICKS));
        assert_eq!(delivery.completed_at, at((index + 1) * HIGH_WORD_TICKS));
    }
}

#[test]
fn the_wire_goes_idle_between_widely_spaced_words() {
    let mut bus = Arinc429Bus::new(CHANNEL, Arinc429Speed::High);
    bus.transmit(at(0), word(1)).unwrap();
    let later = at(10 * HIGH_WORD_TICKS);
    assert_eq!(
        bus.transmit(later, word(2)).unwrap(),
        at(11 * HIGH_WORD_TICKS)
    );
}

#[test]
fn a_configured_gap_separates_words_by_whole_bit_times() {
    let mut bus = Arinc429Bus::new(CHANNEL, Arinc429Speed::Low).with_gap_bits(4);
    bus.transmit(at(0), word(1)).unwrap();
    let second = bus.transmit(at(0), word(2)).unwrap();
    let gap = 4 * Arinc429Speed::Low.bit_time().ticks();
    assert_eq!(second, at(2 * LOW_WORD_TICKS + gap));
}

#[test]
fn frames_the_channel_cannot_carry_are_refused() {
    let mut bus = Arinc429Bus::new(CHANNEL + 1, Arinc429Speed::High);
    assert!(matches!(
        bus.transmit(at(0), word(1)),
        Err(DeviceError::WrongChannel { .. })
    ));
    let can = DigitalBusFrame::Can {
        bus_id: 0,
        id: 1,
        extended: false,
        data: [0; 8],
        dlc: 0,
    };
    assert!(matches!(
        bus.transmit(at(0), can),
        Err(DeviceError::WrongChannel { .. })
    ));
    let even_parity = DigitalBusFrame::Arinc429 {
        channel: CHANNEL + 1,
        word: 0b11,
    };
    assert!(matches!(
        bus.transmit(at(0), even_parity),
        Err(DeviceError::InvalidFrame { .. })
    ));
}

#[test]
fn requests_must_come_in_time_order() {
    let mut bus = Arinc429Bus::new(CHANNEL, Arinc429Speed::High);
    bus.transmit(at(100), word(1)).unwrap();
    assert_eq!(
        bus.transmit(at(99), word(2)),
        Err(DeviceError::OutOfOrder {
            at: at(99),
            previous: at(100),
        })
    );
}

#[test]
fn a_cloned_bus_replays_what_was_on_the_wire() {
    let mut bus = Arinc429Bus::new(CHANNEL, Arinc429Speed::High);
    bus.transmit(at(0), word(1)).unwrap();
    let mut replay = bus.clone();
    let done = at(HIGH_WORD_TICKS);
    assert_eq!(bus.take_due(done), replay.take_due(done));
    assert_eq!(bus, replay);
}
