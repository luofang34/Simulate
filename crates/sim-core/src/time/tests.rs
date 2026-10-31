use super::*;

#[test]
fn frame_and_bus_rates_divide_the_master_clock() {
    // Frame and physics rates.
    for hz in [
        24, 25, 30, 36, 45, 48, 50, 60, 72, 75, 90, 100, 120, 144, 180, 200, 240, 300, 360, 400,
        480, 500, 600, 800, 960, 1_000, 1_200, 2_400,
    ] {
        let period = TickSpan::period_of_hz(hz).unwrap();
        assert_eq!(period.ticks() * hz, TICKS_PER_SECOND);
    }

    // ARINC 429 high-speed (100 kbps / 200 kHz RZ) and low-speed (12.5 kbps / 25 kHz RZ).
    for hz in [12_500, 25_000, 100_000, 200_000] {
        let period = TickSpan::period_of_hz(hz).unwrap();
        assert_eq!(period.ticks() * hz, TICKS_PER_SECOND);
    }

    // Standard UART baud rates.
    for baud in [
        9_600, 19_200, 38_400, 57_600, 115_200, 230_400, 460_800, 921_600,
    ] {
        let period = TickSpan::period_of_hz(baud).unwrap();
        assert_eq!(period.ticks() * baud, TICKS_PER_SECOND);
    }

    // Fractional sensor ODRs via period_of_ratio (12.5 Hz, 6.25 Hz, 1.5625 Hz).
    for denom in [2, 4, 16] {
        assert_eq!(
            TickSpan::period_of_ratio(25, denom).unwrap().ticks() * 25,
            TICKS_PER_SECOND * denom
        );
    }
}

#[test]
fn oscillators_and_guidance_clocks_divide_the_master_clock() {
    // MIL-STD-1553 (1 Mbps / 2 MHz), CAN / CAN-FD, I2C, RTC, and MCU/FPGA crystals.
    for hz in [
        32_768,      // RTC crystal (2^15 Hz)
        65_536,      // 2^16 Hz counter
        125_000,     // CAN 125 kbps
        250_000,     // CAN 250 kbps
        400_000,     // I2C fast mode
        500_000,     // CAN 500 kbps
        1_000_000,   // MIL-STD-1553 / CAN 1 Mbps / AGS 1 MHz
        1_843_200,   // UART crystal
        2_000_000,   // MIL-STD-1553 Manchester / CAN-FD 2 Mbps
        2_048_000,   // Apollo AGC master crystal
        5_000_000,   // CAN-FD 5 Mbps
        7_372_800,   // UART crystal
        8_000_000,   // 8 MHz crystal / CAN-FD 8 Mbps
        10_000_000,  // 10 MHz reference oscillator
        12_000_000,  // 12 MHz USB/MCU crystal
        14_745_600,  // UART crystal
        16_000_000,  // 16 MHz MCU crystal
        20_000_000,  // 20 MHz oscillator
        24_000_000,  // 24 MHz oscillator
        32_000_000,  // 32 MHz oscillator
        40_000_000,  // 40 MHz oscillator
        48_000_000,  // 48 MHz USB/MCU clock
        64_000_000,  // 64 MHz clock
        72_000_000,  // 72 MHz STM32 clock
        80_000_000,  // 80 MHz clock
        96_000_000,  // 96 MHz clock
        120_000_000, // 120 MHz clock
        160_000_000, // 160 MHz clock
        240_000_000, // 240 MHz clock
        480_000_000, // 480 MHz STM32H7 / USB HS clock
    ] {
        let period = TickSpan::period_of_hz(hz).unwrap();
        assert_eq!(period.ticks() * hz, TICKS_PER_SECOND);
    }

    // Apollo AGC scaler (1,600 Hz), gyro/PIPA interrogation (3,200 Hz), and MCT (12 / 1,024,000 s).
    assert_eq!(
        TickSpan::period_of_hz(1_600).unwrap().ticks() * 1_600,
        TICKS_PER_SECOND
    );
    assert_eq!(
        TickSpan::period_of_hz(3_200).unwrap().ticks() * 3_200,
        TICKS_PER_SECOND
    );
    assert_eq!(
        TickSpan::period_of_ratio(1_024_000, 12),
        Some(TickSpan::AGC_MCT)
    );
    assert_eq!(
        TickSpan::AGC_MCT.ticks_u128() * 1_024_000 / 12,
        TICKS_PER_SECOND as u128
    );
}

#[test]
fn non_dividing_rates_return_none() {
    assert_eq!(TickSpan::period_of_hz(0), None);
    assert_eq!(TickSpan::period_of_ratio(100, 0), None);
    assert_eq!(TickSpan::period_of_hz(7), None);
    // GPS fundamental 10.23 MHz has prime factors 11 and 31; handled via RateAdapter::from_hz_ratio.
    assert_eq!(TickSpan::period_of_hz(10_230_000), None);
}

#[test]
fn instants_and_spans_preserve_picosecond_precision_over_centuries() {
    let t = SimInstant::START
        .advanced(TickSpan::AGC_MCT)
        .advanced(TickSpan::AGC_MCT);
    assert_eq!(t.ticks(), TickSpan::AGC_MCT.ticks() * 2);
    assert!((TickSpan::SECOND.as_seconds() - 1.0).abs() < f64::EPSILON);

    // 500 Julian years (500 * 365.25 * 86,400 s = 15,778,800,000 s) plus 1 AGC MCT.
    let five_centuries_s: u64 = 500 * 31_557_600;
    let span = TickSpan::from_seconds_and_subsec_ticks(five_centuries_s, TickSpan::AGC_MCT.ticks());
    let epic = SimInstant::START.advanced(span);
    assert_eq!(
        epic.seconds_and_subsec_ticks(),
        (five_centuries_s, TickSpan::AGC_MCT.ticks())
    );
    assert_eq!(
        epic.duration_since(SimInstant::from_century(five_centuries_s)),
        TickSpan::AGC_MCT
    );
}

impl SimInstant {
    const fn from_century(seconds: u64) -> Self {
        Self::START.advanced(TickSpan::from_seconds_and_subsec_ticks(seconds, 0))
    }
}

#[test]
fn nanoseconds_round_trip_exactly_through_ticks() {
    for nanos in [0, 1, 24, 25, 999_999_999, 1_000_000_001, u64::MAX] {
        assert_eq!(SimInstant::from_nanos(nanos).to_nanos(), Some(nanos));
    }
}

#[test]
fn instants_round_down_to_nanoseconds_without_inverting() {
    let one_tick = TickSpan::from_ticks(1);
    assert_eq!(SimInstant::START.advanced(one_tick).to_nanos(), Some(0));
    let frame = SimInstant::START.advanced(TickSpan::from_ticks(TICKS_PER_SECOND / 30));
    assert_eq!(frame.to_nanos(), Some(33_333_333));
    let mut previous = 0;
    for ticks in (0..2_000).map(|t| t * 7) {
        let nanos = SimInstant::START
            .advanced(TickSpan::from_ticks(ticks))
            .to_nanos()
            .unwrap();
        assert!(nanos >= previous);
        previous = nanos;
    }
}

#[test]
fn instants_past_the_u64_nanosecond_range_are_refused() {
    let last = SimInstant::from_nanos(u64::MAX);
    assert_eq!(last.advanced(TickSpan::SECOND).to_nanos(), None);
    let centuries = TickSpan::from_seconds_and_subsec_ticks(u64::MAX, 0);
    assert_eq!(SimInstant::START.advanced(centuries).to_nanos(), None);
}
