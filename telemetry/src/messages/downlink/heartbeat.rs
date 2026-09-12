use core::f32::consts::PI;

use half::f16;
use serde::{Deserialize, Serialize};

use mission::mavlink::VehicleSnapshot;
use rapid_dialect::rapid::enums::{MavAutopilot, MavModeFlag, MavState, MavType};
use rapid_dialect::rapid::messages::{Altitude, Attitude, Heartbeat, LocalPositionNed, VfrHud};

use super::{ConnectionContext, DownlinkTelemetryMessage, f16_le};

/// The most important message, expected to be transmitted by every system fairly regularly.
///
/// Contains mode, attitude, altitude and velocity information.
///
/// Can be built from HEARTBEAT, LOCAL_POSITION_NED & ATTITUDE messages.
///
/// ALTITUDE and VFR_HUD can be partially reconstructed by the receiver.
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
    /// us a range of -300.0-12807.2 meters.
    #[serde(with = "postcard::fixint::le")]
    altitude_local: u16,
    /// 8 bits of roll  (-180 - +180 deg, in 1.41deg)
    /// 8 bits of pitch ( -90 -  +90 deg, in 0.70deg)
    /// 8 bits of yaw   (-180 - +180 deg, in 1.41deg)
    euler_angles: (i8, i8, i8),
    #[serde(with = "f16_le")]
    vertical_speed: f16, // TODO
    #[serde(with = "f16_le")]
    ground_speed: f16, // TODO
                       // TODO: throttle or vertical acceleration
}

impl DownlinkTelemetryMessage for HeartbeatMessage {
    const ID: u8 = 0x01;
    type Input<'a> = &'a VehicleSnapshot<'a>;
    type Output = (Heartbeat, LocalPositionNed, Attitude, Altitude, VfrHud);

    fn pack(snapshot: Self::Input<'_>) -> Self {
        let heartbeat: Heartbeat = snapshot.into();
        let local_position: LocalPositionNed = snapshot.into();
        let attitude: Attitude = snapshot.into();

        #[allow(
            clippy::arithmetic_side_effects,
            reason = "u8::max(_, 1) is >= 1, so -1 cannot underflow"
        )]
        let mav_state = u8::max(heartbeat.system_status as u8, 1) - 1;
        let profile = 0x01; // TODO
        let armed = heartbeat.base_mode.contains(MavModeFlag::SAFETY_ARMED) as u8;
        let mav_state_profile_and_armed = ((mav_state & 0b111) << 5) | (profile << 1) | (armed);

        let mode = heartbeat.custom_mode; // TODO
        let alt = (local_position.z * -10.0 + 3000.0) as u32;
        let mode_and_altitude = ((mode << 2) | ((alt >> 16) & 0b1)) as u8;

        let roll = (attitude.roll * (i8::MAX as f32) / PI) as i8;
        let pitch = (attitude.pitch * (i8::MAX as f32) / PI) as i8;
        let yaw = (attitude.yaw * (i8::MAX as f32) / PI) as i8;

        Self {
            mav_state_profile_and_armed,
            mode_and_altitude,
            euler_angles: (roll, pitch, yaw),
            altitude_local: alt as u16,
            vertical_speed: f16::from_f32(0.0),
            ground_speed: f16::from_f32(0.0),
        }
    }

    #[allow(
        clippy::similar_names,
        reason = "address complaints to the english language"
    )]
    fn unpack(self, context: &mut ConnectionContext) -> Self::Output {
        let heartbeat = Heartbeat {
            type_: MavType::Rocket,                            // TODO
            autopilot: MavAutopilot::Generic,                  // TODO
            system_status: MavState::Active,                   // TODO
            base_mode: MavModeFlag::empty(),                   // TODO
            custom_mode: (self.mode_and_altitude >> 2) as u32, // TODO
            mavlink_version: 2,
        };

        // TODO
        let alt = (((self.mode_and_altitude & 0b1) as u32) << 16) | self.altitude_local as u32;

        let local_position = LocalPositionNed {
            time_boot_ms: context.time,
            z: ((alt as f32) - 3000.0) / -10.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            ..Default::default()
        };

        // TODO
        let (roll, pitch, yaw) = self.euler_angles;
        let attitude = Attitude {
            time_boot_ms: context.time,
            roll: (roll as f32) / ((i8::MAX as f32) / PI),
            pitch: (pitch as f32) / ((i8::MAX as f32) / PI),
            yaw: (yaw as f32) / ((i8::MAX as f32) / PI),
            ..Default::default()
        };

        // TODO
        #[allow(
            clippy::arithmetic_side_effects,
            reason = "u32 time widened to u64 before *1000, cannot overflow"
        )]
        let altitude = Altitude {
            time_usec: (context.time as u64) * 1000,
            altitude_monotonic: 0.0,
            altitude_amsl: 0.0,
            altitude_local: 0.0,
            altitude_relative: 0.0,
            altitude_terrain: 0.0,
            bottom_clearance: 0.0,
        };

        // TODO
        let vfr_hud = VfrHud {
            alt: 0.0,
            climb: 0.0,
            throttle: 0,
            heading: 0,
            airspeed: 0.0,
            groundspeed: 0.0,
        };

        (heartbeat, local_position, attitude, altitude, vfr_hud)
    }
}
