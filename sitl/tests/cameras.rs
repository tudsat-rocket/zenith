//! Camera recording: commanded through the real `Vehicle`, reported back as CAMERA_CAPTURE_STATUS
//! from what the bus says the camera outputs are doing.

mod common;

use common::{Harness, block_on};
#[cfg(feature = "hybrid")]
use rapid_dialect::FlightMode;
use rapid_dialect::Rapid;
#[cfg(feature = "hybrid")]
use rapid_dialect::rapid::messages::CameraCaptureStatus;

/// The last `video_status` of each camera, by camera id, over one second of telemetry.
#[cfg(feature = "hybrid")]
async fn recording(h: &mut Harness) -> [Option<u8>; 3] {
    let mut status = [None; 3];
    for msg in h.collect_telemetry(1000).await {
        if let Rapid::CameraCaptureStatus(CameraCaptureStatus {
            camera_device_id,
            video_status,
            ..
        }) = msg
        {
            let slot = usize::from(camera_device_id)
                .checked_sub(1)
                .and_then(|i| status.get_mut(i))
                .expect("camera id out of range");
            *slot = Some(video_status);
        }
    }
    status
}

#[cfg(feature = "hybrid")]
#[test]
fn cameras_are_off_until_armed() {
    block_on(async {
        let mut h = Harness::new(None).await;
        assert_eq!(recording(&mut h).await, [Some(0); 3]);

        h.arm();
        assert_eq!(recording(&mut h).await, [Some(1); 3]);
    });
}

#[cfg(feature = "hybrid")]
#[test]
fn cameras_start_and_stop_on_command() {
    block_on(async {
        let mut h = Harness::new(None).await;

        h.vehicle.try_command_camera(2, true).unwrap();
        assert_eq!(recording(&mut h).await, [Some(0), Some(1), Some(0)]);

        h.vehicle.try_command_camera(0, true).unwrap();
        assert_eq!(recording(&mut h).await, [Some(1); 3]);

        h.vehicle.try_command_camera(3, false).unwrap();
        assert_eq!(recording(&mut h).await, [Some(1), Some(1), Some(0)]);

        h.vehicle.try_command_camera(0, false).unwrap();
        assert_eq!(recording(&mut h).await, [Some(0); 3]);
    });
}

/// Stopping a camera in flight only lasts until the next mode change, which turns it back on.
#[cfg(feature = "hybrid")]
#[test]
fn mode_changes_restart_stopped_cameras() {
    block_on(async {
        let mut h = Harness::new(None).await;
        h.arm();

        h.vehicle.try_command_camera(1, false).unwrap();
        assert_eq!(recording(&mut h).await, [Some(0), Some(1), Some(1)]);

        h.vehicle.set_mode(FlightMode::Burn);
        assert_eq!(recording(&mut h).await, [Some(1); 3]);
    });
}

#[test]
fn a_command_naming_a_missing_camera_changes_nothing() {
    block_on(async {
        let mut h = Harness::new(None).await;
        let before = *h.vehicle.bus_outputs.binary_output.values();

        assert!(h.vehicle.try_command_camera(4, true).is_err());
        h.run_ticks(1).await;
        assert_eq!(*h.vehicle.bus_outputs.binary_output.values(), before);
    });
}

/// Without IO boards nothing reports the outputs, so there is no status to send.
#[cfg(not(feature = "hybrid"))]
#[test]
fn no_status_without_a_bus() {
    block_on(async {
        let mut h = Harness::new(None).await;
        h.vehicle.try_command_camera(0, true).unwrap();

        let statuses = h
            .collect_telemetry(1000)
            .await
            .into_iter()
            .filter(|m| matches!(m, Rapid::CameraCaptureStatus(_)))
            .count();
        assert_eq!(statuses, 0);
    });
}
