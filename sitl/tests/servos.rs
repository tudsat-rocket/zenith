//! Manual servo commanding, through the real `Vehicle` and out on the downlink.

mod common;

use common::{Harness, block_on};
use mission::inventory::ServoId;
use rapid_dialect::Rapid;
use rapid_dialect::rapid::messages::ServoOutputRaw;

async fn last_servo_output(h: &mut Harness) -> ServoOutputRaw {
    h.collect_telemetry_by_tick(200)
        .await
        .into_iter()
        .flatten()
        .filter_map(|m| match m.message {
            Rapid::ServoOutputRaw(raw) => Some(raw),
            _ => None,
        })
        .last()
        .expect("no SERVO_OUTPUT_RAW in 200 ms")
}

fn raw(out: &ServoOutputRaw) -> [u16; 4] {
    [
        out.servo1_raw,
        out.servo2_raw,
        out.servo3_raw,
        out.servo4_raw,
    ]
}

#[test]
fn servos_are_not_driven_until_commanded() {
    block_on(async {
        let mut h = Harness::new(None).await;

        assert!(h.vehicle.bus_outputs.servo.iter().all(|(_, s)| s.is_none()));
        assert_eq!(raw(&last_servo_output(&mut h).await), [0; 4]);
    });
}

#[test]
fn commanded_servos_hold_their_position() {
    block_on(async {
        let mut h = Harness::new(None).await;

        h.vehicle.try_command_servo(1, 1000).unwrap();
        h.vehicle.try_command_servo(3, 250).unwrap();
        assert_eq!(raw(&last_servo_output(&mut h).await), [0, 2000, 0, 1250]);

        // A later command moves only the servo it names.
        h.vehicle.try_command_servo(3, 0).unwrap();
        assert_eq!(raw(&last_servo_output(&mut h).await), [0, 2000, 0, 1000]);

        let servos = &h.vehicle.bus_outputs.servo;
        assert!(servos[ServoId::PressurantDisconnect].is_none());
        assert_eq!(
            servos[ServoId::OxidizerDisconnect].map(|s| s.promille()),
            Some(1000)
        );
    });
}

#[test]
fn a_command_naming_a_missing_servo_changes_nothing() {
    block_on(async {
        let mut h = Harness::new(None).await;

        assert!(h.vehicle.try_command_servo(4, 500).is_err());
        assert!(h.vehicle.bus_outputs.servo.iter().all(|(_, s)| s.is_none()));
    });
}
