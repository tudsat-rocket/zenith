//! Servo and quick disconnect commanding, through the real `Vehicle` and out on the downlink.

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

/// `None` is released.
fn promille(h: &Harness) -> [Option<u16>; 4] {
    h.vehicle
        .bus_outputs
        .servo
        .values()
        .map(|s| s.map(|s| s.promille()))
}

#[test]
fn servos_start_released() {
    block_on(async {
        let mut h = Harness::new(None).await;

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
    });
}

#[test]
fn a_command_naming_a_missing_servo_changes_nothing() {
    block_on(async {
        let mut h = Harness::new(None).await;

        assert!(h.vehicle.try_command_servo(4, 500).is_err());
        h.run_ticks(1).await;
        assert_eq!(promille(&h), [None; 4]);
    });
}

#[test]
fn a_gripper_release_runs_the_sequence_with_default_params() {
    block_on(async {
        let mut h = Harness::new(None).await;

        h.vehicle.try_command_gripper(1, false).unwrap();
        h.run_ticks(500).await;
        assert_eq!(promille(&h), [Some(1000), None, Some(0), None]);

        h.run_ticks(1000).await;
        assert_eq!(promille(&h), [Some(1000), None, Some(100), None]);

        h.run_ticks(1000).await;
        assert_eq!(promille(&h), [Some(0), None, Some(100), None]);

        assert!(h.vehicle.try_command_gripper(0, false).is_err());
        assert!(h.vehicle.try_command_gripper(3, false).is_err());
    });
}

#[test]
fn positions_survive_mode_changes() {
    block_on(async {
        let mut h = Harness::new(None).await;

        h.vehicle.try_command_servo(0, 1000).unwrap();
        h.vehicle.try_command_servo(3, 400).unwrap();
        for mode in [
            FlightMode::Hold,
            FlightMode::Disconnect,
            FlightMode::Retract,
            FlightMode::Idle,
        ] {
            h.vehicle.set_mode(mode);
            h.run_ticks(10).await;
            assert_eq!(
                promille(&h),
                [Some(1000), None, None, Some(400)],
                "{mode:?}"
            );
        }
    });
}

#[test]
fn a_winch_command_aborts_a_running_release() {
    block_on(async {
        let mut h = Harness::new(None).await;

        h.vehicle.try_command_gripper(2, false).unwrap();
        h.run_ticks(1500).await;
        h.vehicle.try_command_servo(3, 300).unwrap();
        h.run_ticks(2000).await;
        assert_eq!(promille(&h), [None, Some(1000), None, Some(300)]);
    });
}
