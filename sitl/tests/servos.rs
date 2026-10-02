//! Servo commanding and the servo modes, through the real `Vehicle` and out on the downlink.

mod common;

use common::{Harness, block_on};
use rapid_dialect::FlightMode;
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

fn promille(h: &Harness) -> [u16; 4] {
    h.vehicle.bus_outputs.servo.values().map(|s| s.promille())
}

#[test]
fn servos_start_closed() {
    block_on(async {
        let mut h = Harness::new(None).await;

        assert_eq!(raw(&last_servo_output(&mut h).await), [1000; 4]);
    });
}

#[test]
fn commanded_servos_hold_their_position() {
    block_on(async {
        let mut h = Harness::new(None).await;

        h.vehicle.try_command_servo(1, 1000).unwrap();
        h.vehicle.try_command_servo(3, 250).unwrap();
        assert_eq!(
            raw(&last_servo_output(&mut h).await),
            [1000, 2000, 1000, 1250]
        );

        // A later command moves only the servo it names.
        h.vehicle.try_command_servo(3, 0).unwrap();
        assert_eq!(
            raw(&last_servo_output(&mut h).await),
            [1000, 2000, 1000, 1000]
        );
    });
}

#[test]
fn a_command_naming_a_missing_servo_changes_nothing() {
    block_on(async {
        let mut h = Harness::new(None).await;

        assert!(h.vehicle.try_command_servo(4, 500).is_err());
        h.run_ticks(1).await;
        assert_eq!(promille(&h), [0; 4]);
    });
}

#[test]
fn disconnect_then_retract_with_default_params() {
    block_on(async {
        let mut h = Harness::new(None).await;

        h.vehicle.set_mode(FlightMode::Disconnect);
        h.run_ticks(500).await;
        assert_eq!(promille(&h), [1000, 1000, 0, 0]);

        h.run_ticks(1000).await;
        assert_eq!(promille(&h), [1000, 1000, 100, 100]);

        h.run_ticks(1000).await;
        assert_eq!(promille(&h), [0, 0, 100, 100]);

        h.vehicle.set_mode(FlightMode::Retract);
        h.run_ticks(1).await;
        assert_eq!(promille(&h), [0, 0, 1000, 1000]);

        h.vehicle.set_mode(FlightMode::Pressurize);
        assert_eq!(
            raw(&last_servo_output(&mut h).await),
            [1000, 1000, 2000, 2000]
        );
    });
}

#[test]
fn a_manual_command_overrides_disconnect_until_the_mode_changes() {
    block_on(async {
        let mut h = Harness::new(None).await;

        h.vehicle.set_mode(FlightMode::Disconnect);
        h.vehicle.try_command_servo(2, 300).unwrap();
        h.run_ticks(1500).await;
        assert_eq!(promille(&h), [1000, 1000, 300, 100]);

        h.vehicle.set_mode(FlightMode::Retract);
        h.run_ticks(1).await;
        assert_eq!(promille(&h), [0, 0, 1000, 1000]);
    });
}
