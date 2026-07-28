//! Telemetry scheduling, as opposed to telemetry contents.
//!
//! The schedule gives every message a tick of its own so that no tick of the 1 kHz main loop ever
//! builds two; `mission::schedule` documents why that matters and works the phases out at compile
//! time. What the spreading must not cost is rate, which is what these tests pin - end to end,
//! through the real `Vehicle`, rather than against the allocator's own arithmetic.

mod common;

use common::{Harness, block_on};
use mission::TankId;
use mission::inventory::InventoryId;
use rapid_dialect::Rapid;
use rapid_dialect::rapid::enums::ValveId;

/// Two full cycles of the slowest (1000 ms) interval, so every combination of phases that can
/// coincide has had the chance to. Every interval in the schedule has to divide this, or the
/// expected counts below stop being whole numbers.
const TICKS: u32 = 2000;

/// How often each message is expected on the downlink, restated independently of the schedule that
/// implements it - a message that loses its offset or ends up on the wrong interval is otherwise
/// invisible until it turns up missing from a flight log.
macro_rules! assert_rates {
    ($sent:expr, $($message:ident every $interval_ms:expr),+ $(,)?) => {
        $(
            assert_eq!(
                $sent.iter().filter(|m| matches!(m, Rapid::$message(_))).count(),
                (TICKS / $interval_ms) as usize,
                concat!(stringify!($message), " did not go out every {} ms"),
                $interval_ms,
            );
        )+
    };
}

#[test]
fn every_message_goes_out_at_its_intended_rate() {
    block_on(async {
        let mut harness = Harness::new(None).await;
        let per_tick = harness.collect_telemetry_by_tick(TICKS).await;

        for (tick, messages) in per_tick.iter().enumerate() {
            let built = messages.len();
            assert!(built <= 1, "tick {tick} built {built} messages");
        }

        let sent: Vec<&Rapid> = per_tick.iter().flatten().collect();

        assert_rates! { sent,
            Attitude every 50,
            ScaledImu every 50,
            LocalPositionNed every 100,
            VfrHud every 100,
            ScaledImu2 every 100,
            ScaledImu3 every 100,
            ScaledPressure every 100,
            ScaledPressure2 every 100,
            ScaledPressure3 every 100,
            BatteryStatus every 200,
            Heartbeat every 500,
            SysStatus every 500,
            GlobalPositionInt every 500,
            GpsRawInt every 500,
            RocketInfo every 1000,
            AutopilotVersion every 1000,
        }

        // The instance messages take one slot each, so a single component can silently drop out
        // while its siblings keep reporting. Walk the inventory, not just the ids that turned up.
        let propulsion_reports = (TICKS / 200) as usize;

        for tank in TankId::ALL {
            let reports = sent
                .iter()
                .filter(|m| matches!(m, Rapid::PressureVessel(p) if p.id == tank as u8))
                .count();
            assert_eq!(
                reports, propulsion_reports,
                "tank {tank:?} reported {reports} times"
            );
        }

        for valve in ValveId::ALL {
            let reports = sent
                .iter()
                .filter(|m| matches!(m, Rapid::Valve(v) if v.id == valve))
                .count();
            assert_eq!(
                reports, propulsion_reports,
                "valve {valve:?} reported {reports} times"
            );
        }
    });
}
