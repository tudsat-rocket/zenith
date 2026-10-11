//! The estimator's world frame.
//!
//! Orientation comes from an AHRS that references magnetic north, position from GPS, and those are
//! two different world frames.

use core::num::Wrapping;

use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use rapid_dialect::FlightMode;
use state_estimator::{StateEstimator, StateEstimatorParams};

/// Earth's field with no declination, so magnetic north is east-north-up's north and the only
/// thing left to get wrong is the frame itself.
const FIELD_ENU: Vector3<f32> = Vector3::new(0.0, 20.0, -43.0);

const GRAVITY: f32 = 9.80665;

/// Rocket on a rail: 250 deg, 84 deg above the horizon. Its body z runs along the airframe and its
/// body x is horizontal.
fn on_the_rail() -> Matrix3<f32> {
    let (elevation, heading) = (84f32.to_radians(), 250f32.to_radians());
    let body_z = Vector3::new(
        elevation.cos() * heading.sin(),
        elevation.cos() * heading.cos(),
        elevation.sin(),
    );
    let body_x = Vector3::z().cross(&body_z).normalize();

    Matrix3::from_columns(&[body_x, body_z.cross(&body_x), body_z])
}

#[test]
fn orientation_comes_back_in_the_frame_the_positions_use() {
    let truth = on_the_rail();
    let accel = truth.transpose() * Vector3::new(0.0, 0.0, GRAVITY);
    let mag = truth.transpose() * FIELD_ENU;

    // Start well away from the answer, so the filter has to find it rather than be handed it.
    let start = UnitQuaternion::from_matrix(&truth)
        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 40f32.to_radians());
    let mut estimator =
        StateEstimator::new_with_quat(1000.0, StateEstimatorParams::default(), start);

    for t in 0..120_000 {
        estimator.update(
            Wrapping(t),
            FlightMode::Idle,
            Some(Vector3::zeros()),
            Some(accel),
            None,
            Some(mag),
            Some(0.0),
            None,
        );
    }

    let orientation = estimator
        .orientation
        .expect("the estimator never settled on an orientation");

    for (name, axis) in [
        ("x", Vector3::x()),
        ("y", Vector3::y()),
        ("z", Vector3::z()),
    ] {
        let expected = truth * axis;
        let recovered = orientation.transform_vector(&axis);
        let off_by = recovered.angle(&expected).to_degrees();
        assert!(
            off_by < 1.0,
            "body {name} is {off_by} deg out: {recovered:?}"
        );
    }
}

#[test]
fn a_missing_sample_keeps_the_orientation() {
    let truth = on_the_rail();
    let accel = truth.transpose() * Vector3::new(0.0, 0.0, GRAVITY);
    let mag = truth.transpose() * FIELD_ENU;
    let mut estimator = StateEstimator::new_with_quat(
        1000.0,
        StateEstimatorParams::default(),
        UnitQuaternion::from_matrix(&truth),
    );

    let gaps = [
        (FlightMode::Idle, Some(Vector3::zeros()), Some(mag)),
        (FlightMode::Idle, None, Some(mag)),
        (FlightMode::Idle, Some(Vector3::zeros()), None),
        (
            FlightMode::Idle,
            Some(Vector3::zeros()),
            Some(Vector3::zeros()),
        ),
        (FlightMode::Burn, None, None),
        (FlightMode::Coast, Some(Vector3::zeros()), None),
    ];
    for (t, (mode, gyro, mag)) in (0..).zip(gaps.into_iter().flat_map(|gap| [gap; 1000])) {
        estimator.update(
            Wrapping(t),
            mode,
            gyro,
            Some(accel),
            None,
            mag,
            Some(0.0),
            None,
        );

        assert!(
            estimator.orientation.is_some(),
            "{mode:?}: lost the orientation"
        );
        let vertical = estimator
            .acceleration_world_raw()
            .expect("no world-frame acceleration")
            .z;
        assert!(
            vertical.abs() < 0.1,
            "{mode:?}: standing still at {vertical} m/s^2"
        );
    }
}
