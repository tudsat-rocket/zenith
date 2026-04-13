//! Full nominal flight: arm the vehicle, run the simulation to landing, and
//! assert the mode sequence passes through every expected stage in order.

mod common;

use common::{Harness, block_on};
use rapid_dialect::FlightMode;

/// Max ticks (ms) we will allow for a full flight. Descent alone at the
/// default -15 m/s drogue / -5 m/s main rates from apogee to ground is on
/// the order of 150s; 400s is a comfortable upper bound.
const MAX_TICKS: u32 = 400_000;

#[test]
fn state_machine_advances_correctly() {
    block_on(async {
        let mut h = Harness::new(None).await;

        let expected = [
            FlightMode::Idle,
            FlightMode::Armed,
            FlightMode::Burn,
            FlightMode::Coast,
            FlightMode::RecoveryDrogue,
            FlightMode::RecoveryMain,
            FlightMode::Landed,
        ];

        let mut seen: Vec<FlightMode> = vec![h.mode()];
        h.arm();
        if *seen.last().unwrap() != h.mode() {
            seen.push(h.mode());
        }

        let result = h
            .run_until(MAX_TICKS, |h| {
                let m = h.vehicle.mode();
                if *seen.last().unwrap() != m {
                    seen.push(m);
                }
                m == FlightMode::Landed
            })
            .await;

        assert!(
            result.is_ok(),
            "flight did not reach Landed within {MAX_TICKS} ticks, got sequence {seen:?}"
        );

        assert_eq!(
            seen, expected,
            "unexpected mode sequence: got {seen:?}, expected {expected:?}"
        );
    });
}
