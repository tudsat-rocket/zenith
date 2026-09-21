#![cfg(feature = "hybrid")]
//! The uplink-loss failsafe. With no ground station contact the vehicle first shuts everything
//! (`Idle`), then, much later, opens the vents - hybrid build, because that is where the two modes
//! actually move valves.

mod common;

use common::{Harness, block_on};
use mission::inventory::InventoryId;
use mission::{FailsafeParams, Params};
use rapid_dialect::FlightMode;
use rapid_dialect::rapid::enums::ValveId;

const IDLE_TIMEOUT: u32 = 10_000;
const VENT_TIMEOUT: u32 = 120_000;

fn valve(h: &Harness, id: ValveId) -> f32 {
    h.sim.lock().unwrap().hybrid.valve_state(id)
}

/// A loaded vehicle sitting in `FillOxidizer`, the state the failsafe exists for.
async fn filling(params: Option<Params>) -> Harness {
    let mut h = Harness::new(params).await;
    h.fill_pressurant().await;
    h.vehicle.set_mode(FlightMode::FillOxidizer);
    h.run_ticks(30_000).await;
    h
}

#[test]
fn ground_ops_continue_while_the_ground_station_is_talking() {
    block_on(async {
        let mut h = filling(None).await;

        h.run_ticks(2 * VENT_TIMEOUT).await;

        assert_eq!(
            h.mode(),
            FlightMode::FillOxidizer,
            "left FillOxidizer despite a live uplink"
        );
    });
}

#[test]
fn losing_the_uplink_shuts_the_valves_then_vents() {
    block_on(async {
        let mut h = filling(None).await;
        h.set_uplink(false);

        let to_idle = h
            .run_until(2 * IDLE_TIMEOUT, |h| h.mode() != FlightMode::FillOxidizer)
            .await
            .expect("no failsafe transition within twice the idle timeout");

        assert_eq!(
            h.mode(),
            FlightMode::Idle,
            "first stage went somewhere else"
        );
        assert!(
            (IDLE_TIMEOUT..IDLE_TIMEOUT + 10).contains(&to_idle),
            "fell back to Idle after {to_idle}ms, expected ~{IDLE_TIMEOUT}ms"
        );
        // The sim only sees the new valve setpoints on the following tick.
        const SETTLE: u32 = 1000;
        h.run_ticks(SETTLE).await;
        for id in ValveId::ALL {
            assert_eq!(valve(&h, id), 0.0, "{id:?} still open in Idle");
        }

        let to_vent = h
            .run_until(VENT_TIMEOUT, |h| h.mode() != FlightMode::Idle)
            .await
            .expect("no escalation to Vent within the vent timeout");

        assert_eq!(
            h.mode(),
            FlightMode::Vent,
            "second stage went somewhere else"
        );
        assert!(
            (VENT_TIMEOUT..VENT_TIMEOUT + 10).contains(&(to_idle + SETTLE + to_vent)),
            "vented after {}ms, expected ~{VENT_TIMEOUT}ms",
            to_idle + SETTLE + to_vent
        );

        // Let the valves follow the mode, then check the vents are the ones that opened.
        h.run_ticks(1000).await;
        assert!(
            valve(&h, ValveId::PressurantVent) > 0.9 && valve(&h, ValveId::OxidizerVent) > 0.9,
            "vents did not open: pressurant {}, oxidizer {}",
            valve(&h, ValveId::PressurantVent),
            valve(&h, ValveId::OxidizerVent)
        );
    });
}

#[test]
fn zeroed_timeouts_disable_the_failsafe() {
    block_on(async {
        let params = Params {
            failsafe: FailsafeParams {
                uplink_idle_timeout: 0,
                uplink_vent_timeout: 0,
            },
            ..Params::default()
        };

        let mut h = filling(Some(params)).await;
        h.set_uplink(false);

        h.run_ticks(2 * VENT_TIMEOUT).await;

        assert_eq!(
            h.mode(),
            FlightMode::FillOxidizer,
            "failsafe fired with both timeouts at 0"
        );
    });
}
