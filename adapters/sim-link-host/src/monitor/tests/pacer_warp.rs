use std::cell::Cell;

use super::super::{
    FLIGHT_SHARE, FlightStepper, Pacer, SLICE_BUDGET_S, high_warp_flight_share, run_slice,
    slice_budget_for_speed,
};
use super::{Costly, Ticks};

#[test]
fn decoupled_presentation_and_high_warp_slice_budget_lift_64x_from_throttled_to_full_warp() {
    // Models one browser main thread where each route second of physics + navigation
    // costs 12 ms (`83×` raw CPU capacity), each 3D OTW + map presentation redraw costs
    // 15 ms, and each 2D monitor redraw costs 3 ms.
    let simulate_ten_wall_seconds =
        |throttled_presentation: bool, budget_s: f64, flight_share: f64| {
            let clock = Costly {
                now: Cell::new(0.0),
                cost_s: 0.0012,
            };
            let mut flight = Ticks {
                tick: 0,
                ticks: 100_000,
            };
            let mut pacer = Pacer::new(Some(64.0));
            pacer.set_flight_share(flight_share);
            let mut out = Vec::new();
            let mut last_otw_s = -1.0;
            let mut last_dom_s = -1.0;
            while clock.now.get() < 10.0 {
                let wall_s = clock.now.get();
                let target_s = pacer.frame(wall_s, flight.sim_time_s());
                let slice = run_slice(&mut flight, &clock, budget_s, target_s, &mut out).unwrap();
                pacer.observe(&slice, 0.0);
                out.clear();
                let now_s = clock.now.get();
                let otw_interval = if throttled_presentation { 0.1 } else { 0.0 };
                let dom_interval = if throttled_presentation {
                    1.0 / 15.0
                } else {
                    0.0
                };
                if now_s - last_otw_s >= otw_interval {
                    clock.now.set(clock.now.get() + 0.015);
                    last_otw_s = now_s;
                }
                if now_s - last_dom_s >= dom_interval {
                    clock.now.set(clock.now.get() + 0.003);
                    last_dom_s = now_s;
                }
            }
            flight.sim_time_s() / clock.now.get()
        };

    let legacy_speed = simulate_ten_wall_seconds(false, SLICE_BUDGET_S, FLIGHT_SHARE);
    let decoupled_speed = simulate_ten_wall_seconds(
        true,
        slice_budget_for_speed(Some(64.0)),
        high_warp_flight_share(Some(64.0)),
    );
    assert!(
        legacy_speed < 30.0,
        "60 Hz OTW + 8 ms slices throttled 64× down to {legacy_speed:.2}×"
    );
    assert!(
        decoupled_speed >= 55.0,
        "10 Hz OTW + 36 ms slices achieved {decoupled_speed:.2}× at 64× warp"
    );
    assert!(decoupled_speed > legacy_speed * 2.0);
}
