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
