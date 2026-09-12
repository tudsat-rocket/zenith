use core::f32::consts::{FRAC_PI_2, PI};

use half::f16;
use serde::{Deserialize, Serialize};

use mission::mavlink::{VehicleSnapshot, throttle_percent};
use rapid_dialect::FlightMode;
use rapid_dialect::rapid::enums::{MavAutopilot, MavModeFlag, MavState, MavType};
use rapid_dialect::rapid::messages::{Altitude, Attitude, Heartbeat, LocalPositionNed, VfrHud};

use super::{ConnectionContext, DownlinkTelemetryMessage, f16_le};

/// Roll and yaw cover a full turn, pitch only a half one.
const ROLL_YAW_CODES_PER_RAD: f32 = (i8::MAX as f32) / PI;
const PITCH_CODES_PER_RAD: f32 = (i8::MAX as f32) / FRAC_PI_2;

const ALTITUDE_OFFSET_M: f32 = 300.0;
const ALTITUDE_CODES_PER_M: f32 = 10.0;
const ALTITUDE_MAX_CODE: u32 = 0x1_FFFF;

/// What the ALTITUDE message documents as "no measurement" for these two fields. Unlike the rest
/// of it, they are not free to be NaN.
const ALTITUDE_TERRAIN_UNKNOWN: f32 = -1001.0;
const BOTTOM_CLEARANCE_UNKNOWN: f32 = -1.0;

/// The most important message, expected to be transmitted by every system fairly regularly.
///
/// Contains mode, attitude, altitude and velocity information, packed from HEARTBEAT,
/// LOCAL_POSITION_NED, ATTITUDE and VFR_HUD.
///
/// The receiver can also recover ALTITUDE.
#[derive(Debug, Serialize, Deserialize)]
pub struct HeartbeatMessage {
    /// 3 bits of MAV_STATE (uninit variant omitted),
    /// 3 bits of profile ID,
    /// 1 bit for the SAFETY_ARMED flag
    mav_state_profile_and_armed: u8,
    /// 6 bits of mode ID
    ///     these don't correspond to any MAVLink mode enum value, the receiver is aware of
    ///     the necessary mode metadata using the vehicle profile.
    /// 1 bit reserved?
    /// 1 more bit for altitude_local
    mode_and_altitude: u8,
    /// altitude in local coordinate system above origin in (m+300)/10
    /// (taken from LOCAL_POSITION_NED.z, but up-positive)
    /// these are the least-significant 16 bits, with one more in mode_and_altitude, giving
    /// us a range of -300.0-12807.1 meters. Saturates at either end.
    #[serde(with = "postcard::fixint::le")]
    altitude_local: u16,
    /// 8 bits of roll  (-180 - +180 deg, in 1.41deg)
    /// 8 bits of pitch ( -90 -  +90 deg, in 0.70deg)
    /// 8 bits of yaw   (-180 - +180 deg, in 1.41deg)
    euler_angles: (i8, i8, i8),
    /// Climb rate, up-positive (VFR_HUD.climb, or -LOCAL_POSITION_NED.vz).
    #[serde(with = "f16_le")]
    vertical_speed: f16,
    /// Magnitude of the horizontal velocity (VFR_HUD.groundspeed). Its direction is not
    /// transmitted.
    #[serde(with = "f16_le")]
    ground_speed: f16,
    // TODO: vertical acceleration in the spare byte?
}

impl DownlinkTelemetryMessage for HeartbeatMessage {
    const ID: u8 = 0x01;
    type Input<'a> = &'a VehicleSnapshot<'a>;
    type Output = (Heartbeat, LocalPositionNed, Attitude, Altitude, VfrHud);

    fn pack(snapshot: Self::Input<'_>) -> Self {
        let heartbeat: Heartbeat = snapshot.into();
        let local_position: LocalPositionNed = snapshot.into();
        let attitude: Attitude = snapshot.into();
        let vfr_hud: VfrHud = snapshot.into();

        #[allow(
            clippy::arithmetic_side_effects,
            reason = "u8::max(_, 1) is >= 1, so -1 cannot underflow"
        )]
        let mav_state = u8::max(heartbeat.system_status as u8, 1) - 1;
        let profile = 0x01; // TODO
        let armed = heartbeat.base_mode.contains(MavModeFlag::SAFETY_ARMED) as u8;
        let mav_state_profile_and_armed = ((mav_state & 0b111) << 5) | (profile << 1) | (armed);

        let mode = (heartbeat.custom_mode as u8) & 0b11_1111;
        let agl = -local_position.z;
        let alt = ((agl + ALTITUDE_OFFSET_M) * ALTITUDE_CODES_PER_M)
            .clamp(0.0, ALTITUDE_MAX_CODE as f32) as u32;
        let mode_and_altitude = (mode << 2) | ((alt >> 16) & 0b1) as u8;

        let (roll, pitch, yaw) = (attitude.roll, attitude.pitch, attitude.yaw);

        Self {
            mav_state_profile_and_armed,
            mode_and_altitude,
            euler_angles: (
                pack_angle(roll, ROLL_YAW_CODES_PER_RAD),
                pack_angle(pitch, PITCH_CODES_PER_RAD),
                pack_angle(yaw, ROLL_YAW_CODES_PER_RAD),
            ),
            altitude_local: alt as u16,
            vertical_speed: f16::from_f32(vfr_hud.climb),
            ground_speed: f16::from_f32(vfr_hud.groundspeed),
        }
    }

    #[allow(
        clippy::similar_names,
        reason = "address complaints to the english language"
    )]
    fn unpack(self, context: &mut ConnectionContext) -> Self::Output {
        let mode = u32::from(self.mode_and_altitude >> 2);
        let mav_state = (self.mav_state_profile_and_armed >> 5) & 0b111;
        let armed = (self.mav_state_profile_and_armed & 0b1) != 0;

        let heartbeat = Heartbeat {
            type_: MavType::Rocket,           // TODO: from the vehicle profile
            autopilot: MavAutopilot::Generic, // TODO: from the vehicle profile
            // The inverse of the offset `pack` applies to skip the uninit variant.
            system_status: MavState::try_from(mav_state.saturating_add(1))
                .unwrap_or(MavState::Uninit),
            // Without CUSTOM_MODE_ENABLED a ground station never looks at custom_mode.
            base_mode: if armed {
                MavModeFlag::CUSTOM_MODE_ENABLED | MavModeFlag::SAFETY_ARMED
            } else {
                MavModeFlag::CUSTOM_MODE_ENABLED
            },
            custom_mode: mode,
            mavlink_version: 2,
        };

        let altitude_code =
            (u32::from(self.mode_and_altitude & 0b1) << 16) | u32::from(self.altitude_local);

        let altitude_agl = (altitude_code as f32) / ALTITUDE_CODES_PER_M - ALTITUDE_OFFSET_M;
        context.altitude_agl = Some(altitude_agl);

        // The packet carries AGL; the ground's own altitude has to come from somewhere else.
        let altitude_amsl = context
            .altitude_ground_asl
            .map_or(f32::NAN, |ground| ground + altitude_agl);

        let vertical_speed = self.vertical_speed.to_f32();
        let ground_speed = self.ground_speed.to_f32();

        let local_position = LocalPositionNed {
            time_boot_ms: context.time,
            x: f32::NAN,
            y: f32::NAN,
            z: -altitude_agl,
            // We could lie here and use yaw to deconstruct ground speed, assuming aoa=0
            vx: f32::NAN,
            vy: f32::NAN,
            vz: -vertical_speed,
        };

        let (roll, pitch, yaw) = self.euler_angles;
        let yaw = unpack_angle(yaw, ROLL_YAW_CODES_PER_RAD);
        let attitude = Attitude {
            time_boot_ms: context.time,
            roll: unpack_angle(roll, ROLL_YAW_CODES_PER_RAD),
            pitch: unpack_angle(pitch, PITCH_CODES_PER_RAD),
            yaw,
            // Body rates ride SensorsMessage instead.
            rollspeed: f32::NAN,
            pitchspeed: f32::NAN,
            yawspeed: f32::NAN,
        };

        #[allow(
            clippy::arithmetic_side_effects,
            reason = "u32 time widened to u64 before *1000, cannot overflow"
        )]
        let altitude = Altitude {
            time_usec: u64::from(context.time) * 1000,
            altitude_monotonic: altitude_amsl,
            altitude_amsl,
            altitude_local: altitude_agl,
            altitude_relative: altitude_agl,
            altitude_terrain: ALTITUDE_TERRAIN_UNKNOWN,
            bottom_clearance: BOTTOM_CLEARANCE_UNKNOWN,
        };

        let vfr_hud = VfrHud {
            airspeed: f32::NAN,
            groundspeed: ground_speed,
            // `mission` builds this field from the quaternion's euler yaw instead.
            heading: {
                let degrees = yaw.to_degrees();
                (if degrees < 0.0 {
                    degrees + 360.0
                } else {
                    degrees
                }) as i16
            },
            throttle: FlightMode::try_from(mode as u8)
                .map(throttle_percent)
                .unwrap_or(0),
            alt: altitude_amsl,
            climb: vertical_speed,
        };

        (heartbeat, local_position, attitude, altitude, vfr_hud)
    }
}

/// Angles travel as a signed fraction of their range.
fn pack_angle(radians: f32, codes_per_rad: f32) -> i8 {
    (radians * codes_per_rad) as i8
}

fn unpack_angle(code: i8, codes_per_rad: f32) -> f32 {
    f32::from(code) / codes_per_rad
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::tests::{SnapshotParts, through_packet};
    use super::*;

    use nalgebra::UnitQuaternion;

    use mission::AdcData;
    use state_estimator::{StateEstimator, StateEstimatorParams};

    use crate::messages::DownlinkMessage;

    pub(crate) fn flying_estimator() -> StateEstimator {
        let mut estimator = StateEstimator::new(1000.0, StateEstimatorParams::default());
        estimator.kalman.x[2] = 12_000.0;
        estimator.kalman.x[3] = 200.0;
        estimator.kalman.x[4] = -300.0;
        estimator.kalman.x[5] = 500.0;
        estimator.orientation = Some(UnitQuaternion::from_euler_angles(0.4, -0.8, 2.0));
        estimator.altitude_ground = 2500.0;
        estimator
    }

    fn round_trip(
        parts: &SnapshotParts,
        context: &mut ConnectionContext,
    ) -> (Heartbeat, LocalPositionNed, Attitude, Altitude, VfrHud) {
        let msg = DownlinkMessage::Heartbeat(HeartbeatMessage::pack(&parts.snapshot()));
        let DownlinkMessage::Heartbeat(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        decoded.unpack(context)
    }

    /// A ground station only interprets `custom_mode` once it is told to, so the mode is only
    /// really transmitted if CUSTOM_MODE_ENABLED comes with it.
    #[test]
    fn mode_state_and_armed_survive_the_packet() {
        for mode in FlightMode::ALL {
            for armed in [false, true] {
                let mut parts = SnapshotParts {
                    mode,
                    ..Default::default()
                };
                if armed {
                    parts.readings.power = Some(AdcData {
                        recovery_voltage: 8000,
                        ..Default::default()
                    });
                }

                let (heartbeat, ..) = round_trip(&parts, &mut ConnectionContext::init(0));
                let expected_state: MavState = mode.into();

                assert_eq!(heartbeat.custom_mode, mode as u32, "{mode:?}");
                assert_eq!(heartbeat.system_status, expected_state, "{mode:?}");
                assert!(
                    heartbeat
                        .base_mode
                        .contains(MavModeFlag::CUSTOM_MODE_ENABLED),
                    "{mode:?}"
                );
                assert_eq!(
                    heartbeat.base_mode.contains(MavModeFlag::SAFETY_ARMED),
                    armed,
                    "{mode:?}"
                );
            }
        }
    }

    #[test]
    fn throttle_is_rebuilt_from_the_mode() {
        for mode in FlightMode::ALL {
            let parts = SnapshotParts {
                mode,
                ..Default::default()
            };
            let (.., vfr_hud) = round_trip(&parts, &mut ConnectionContext::init(0));
            assert_eq!(vfr_hud.throttle, throttle_percent(mode), "{mode:?}");
        }
    }

    #[test]
    fn altitude_round_trips_within_a_decimetre() {
        for agl in [-250.0, -0.4, 0.0, 123.4, 3000.0, 12_800.0] {
            let mut parts = SnapshotParts::default();
            parts.estimator.kalman.x[2] = agl;

            let (_, local_position, _, altitude, _) =
                round_trip(&parts, &mut ConnectionContext::init(0));

            assert!(
                (altitude.altitude_local - agl).abs() <= 0.1,
                "{agl} m came back as {}",
                altitude.altitude_local
            );
            assert_eq!(altitude.altitude_relative, altitude.altitude_local);
            assert_eq!(local_position.z, -altitude.altitude_local);
        }
    }

    /// The field is 17 bits wide; before it was clamped, an altitude past the top of its range
    /// wrapped and decoded as a plausible low one.
    #[test]
    fn out_of_range_altitudes_clamp_rather_than_wrap() {
        for (agl, expected) in [(50_000.0, 12_807.1), (-5000.0, -300.0)] {
            let mut parts = SnapshotParts::default();
            parts.estimator.kalman.x[2] = agl;

            let (.., altitude, _) = round_trip(&parts, &mut ConnectionContext::init(0));
            assert!(
                (altitude.altitude_local - expected).abs() <= 0.1,
                "{agl} m came back as {}",
                altitude.altitude_local
            );
        }
    }

    #[test]
    fn euler_angles_round_trip_within_their_resolution() {
        for (roll, pitch, yaw) in [
            (0.0, 0.0, 0.0),
            (0.3, -0.7, 2.9),
            (-1.2, 1.4, -2.9),
            (2.0, 0.2, 1.0),
        ] {
            let mut parts = SnapshotParts::default();
            parts.estimator.orientation = Some(UnitQuaternion::from_euler_angles(roll, pitch, yaw));

            let snapshot = parts.snapshot();
            let expected: Attitude = (&snapshot).into();
            let (_, _, attitude, _, _) = round_trip(&parts, &mut ConnectionContext::init(0));

            assert!(
                (attitude.pitch - expected.pitch).abs() <= 1.0 / PITCH_CODES_PER_RAD,
                "pitch {} came back as {}",
                expected.pitch,
                attitude.pitch
            );
            assert!(
                (attitude.yaw - expected.yaw).abs() <= 1.0 / ROLL_YAW_CODES_PER_RAD,
                "yaw {} came back as {}",
                expected.yaw,
                attitude.yaw
            );
            assert!(
                (attitude.roll - expected.roll).abs() <= 1.0 / ROLL_YAW_CODES_PER_RAD,
                "roll {} came back as {}",
                expected.roll,
                attitude.roll
            );
        }
    }

    /// The speeds are carried, everything else about the velocity is not. A regression that
    /// reports the missing half as zero rather than as unknown fails here.
    #[test]
    fn speeds_survive_and_unknown_fields_stay_unknown() {
        let mut parts = SnapshotParts::default();
        parts.estimator.kalman.x[2] = 1500.0;
        parts.estimator.kalman.x[3] = 30.0;
        parts.estimator.kalman.x[4] = 40.0;
        parts.estimator.kalman.x[5] = 120.0;

        let (_, local_position, attitude, altitude, vfr_hud) =
            round_trip(&parts, &mut ConnectionContext::init(0));

        assert!((vfr_hud.climb - 120.0).abs() <= 0.5, "{}", vfr_hud.climb);
        assert_eq!(local_position.vz, -vfr_hud.climb);
        assert!(
            (vfr_hud.groundspeed - 50.0).abs() <= 0.5,
            "{}",
            vfr_hud.groundspeed
        );

        for unknown in [
            local_position.x,
            local_position.y,
            local_position.vx,
            local_position.vy,
            attitude.rollspeed,
            attitude.pitchspeed,
            attitude.yawspeed,
            vfr_hud.airspeed,
            vfr_hud.alt,
            altitude.altitude_amsl,
            altitude.altitude_monotonic,
        ] {
            assert!(unknown.is_nan());
        }

        assert_eq!(altitude.altitude_terrain, ALTITUDE_TERRAIN_UNKNOWN);
        assert_eq!(altitude.bottom_clearance, BOTTOM_CLEARANCE_UNKNOWN);
    }

    /// The ground altitude arrives on [`super::super::GpsMessage`]; every AMSL hangs off it.
    #[test]
    fn a_known_ground_altitude_turns_agl_into_amsl() {
        let mut parts = SnapshotParts::default();
        parts.estimator.kalman.x[2] = 800.0;

        let mut context = ConnectionContext::init(0);
        context.altitude_ground_asl = Some(150.0);
        let (.., altitude, vfr_hud) = round_trip(&parts, &mut context);

        assert!((altitude.altitude_amsl - 950.0).abs() <= 0.1);
        assert!((altitude.altitude_local - 800.0).abs() <= 0.1);
        assert_eq!(vfr_hud.alt, altitude.altitude_amsl);
    }
}
