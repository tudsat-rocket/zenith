use core::cmp::Ord;
use core::f32::consts::PI;

// Once f16 is in core (currently nightly), we can retire this crate.
use half::f16;

use rapid_dialect::rapid::enums::{
    MavAutopilot, MavModeFlag, MavState, MavSysStatusSensor, MavType,
};
use rapid_dialect::rapid::messages::{
    Altitude, Attitude, BatteryInfo, BatteryStatus, GlobalPositionInt, GpsRawInt, GpsStatus,
    Heartbeat, LocalPositionNed, LocalPositionNedCov, RadioStatus, ScaledPressure, ScaledPressure2,
    ScaledPressure3, SysStatus, VfrHud, altitude,
};

pub const DOWNLINK_PACKET_SIZE: usize = 16;

pub struct DownlinkPacket {
    time: u16,
    // TODO: exact split
    profile_and_message_id: u8,
    payload: [u8; DOWNLINK_PACKET_SIZE - 3 - 2],
}

/// Stuff the telemetry receiver can just "keep in mind" based on previously received messages.
/// These are used to enrich received data, especially when values can reasonably be converted or
/// reconstructed from other messages, such as conversion between different altitude reference
/// frames.
struct ConnectionContext {
    pub time: u32,
    /// The receivers current best guess at the absolute time since boot in ms.
    /// Since the main time included in every packet frequently overflows, this may "jump" when the
    /// absolute time since boot is discovered to be higher than the previous estimate.
    pub absolute_time_ms: u32,
    /// The last received ground altitude in meters above sea-level, if known
    pub altitude_ground_asl: Option<f32>,
}

enum DownlinkMessage {
    Heartbeat(HeartbeatMessage),
    GlobalPosition(()),
    GpsRaw(GpsMessage),
    Battery(BatteryMessage),
    Diagnostics(DiagnosticsMessage),
    Barometers(BarometersMessage),
    PressureVesselsMessage(PressureVesselsMessage),
    Actuators(ActuatorsMessage),
    StateEstimator(StateEstimatorMessage),
    Components(ComponentsMessage),
}

pub trait DownlinkTelemetryMessage {
    const ID: u8;

    type Input;
    type Output;

    fn pack(input: Self::Input) -> Self;
    fn unpack<PROFILE>(self, context: &mut ConnectionContext) -> Self::Output;
}

/// The most important message, expected to be transmitted by every system fairly regularly.
///
/// Contains mode, attitude, altitude and velocity information.
///
/// Can be built from HEARTBEAT, LOCAL_POSITION_NED & ATTITUDE messages.
///
/// ALTITUDE and VFR_HUD can be partially reconstructed by the receiver.
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
    altitude_local: u16,
    /// 8 bits of roll  (-180 - +180 deg, in 1.41deg)
    /// 8 bits of pitch ( -90 -  +90 deg, in 0.70deg)
    /// 8 bits of yaw   (-180 - +180 deg, in 1.41deg)
    euler_angles: (i8, i8, i8),
    vertical_speed: f16, // TODO
    ground_speed: f16,   // TODO
                         // TODO: throttle or vertical acceleration
}

impl DownlinkTelemetryMessage for HeartbeatMessage {
    const ID: u8 = 0x01;
    type Input = (Heartbeat, LocalPositionNed, Attitude);
    type Output = (Heartbeat, LocalPositionNed, Attitude, Altitude, VfrHud);

    fn pack((heartbeat, local_position, attitude): Self::Input) -> Self {
        let mav_state = u8::max(heartbeat.system_status as u8, 1) - 1;
        let profile = 0x01; // TODO
        let armed = heartbeat.base_mode.contains(MavModeFlag::SAFETY_ARMED) as u8;
        let mav_state_profile_and_armed = ((mav_state & 0b111) << 5) | (profile << 1) | (armed);

        let mode = heartbeat.custom_mode; // TODO
        let altitude = (local_position.z * -10.0 + 3000.0) as u32;
        let mode_and_altitude = ((mode << 2) | ((altitude >> 16) & 0b1)) as u8;

        let roll = (attitude.roll * (i8::MAX as f32) / PI) as i8;
        let pitch = (attitude.pitch * (i8::MAX as f32) / (0.5 * PI)) as i8;
        let yaw = (attitude.yaw * (i8::MAX as f32) / PI) as i8;

        Self {
            mav_state_profile_and_armed,
            mode_and_altitude,
            euler_angles: (roll, pitch, yaw),
            altitude_local: altitude as u16,
            vertical_speed: f16::from_f32(0.0),
            ground_speed: f16::from_f32(0.0),
        }
    }

    fn unpack<PROFILE>(self, context: &mut ConnectionContext) -> Self::Output {
        let heartbeat = Heartbeat {
            type_: MavType::Rocket,                            // TODO
            autopilot: MavAutopilot::Generic,                  // TODO
            system_status: MavState::Active,                   // TODO
            base_mode: MavModeFlag::empty(),                   // TODO
            custom_mode: (self.mode_and_altitude >> 2) as u32, // TODO
            mavlink_version: 2,
        };

        // TODO
        let local_position = LocalPositionNed {
            time_boot_ms: context.time,
            z: 0.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            ..Default::default()
        };

        // TODO
        let attitude = Attitude {
            time_boot_ms: context.time,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            ..Default::default()
        };

        // TODO
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

// 0x02 is for global position

/// TODO
pub struct GpsMessage {
    fix_type: u8,
    latitude: i32,
    longitude: i32,
    altitude: i32,
    eph: u8,
    epv: u8,
    satellites_visible: u8,
}

impl DownlinkTelemetryMessage for GpsMessage {
    const ID: u8 = 0x03;
    type Input = GpsRawInt;
    type Output = GpsRawInt;

    fn pack(input: Self::Input) -> Self {
        Self {
            fix_type: input.fix_type as u8,
            latitude: input.lat,
            longitude: input.lon,
            altitude: input.alt,
            eph: input.eph as u8, // TODO
            epv: input.epv as u8, // TODO
            satellites_visible: input.satellites_visible,
        }
    }

    fn unpack<PROFILE>(self) -> Self::Output {
        GpsRawInt {
            time_usec: 0,                                // TODO
            fix_type: self.fix_type.try_into().unwrap(), // TODO
            lat: self.latitude,
            lon: self.longitude,
            alt: self.altitude,
            eph: self.eph as u16,
            epv: self.epv as u16,
            vel: u16::MAX,
            cog: u16::MAX,
            satellites_visible: self.satellites_visible,
            ..Default::default()
        }
    }
}

/// TODO
pub struct BatteryMessage {
    /// Determines the IDs of the batteries contained in this message:
    ///     0: 0&1,  1: 2&3,  etc.
    id_block: u8,
    /// reserved for charging states or errors
    reserved_for_states_or_modes: [u8; 2],
    temperature: [u8; 2],
    voltage: [u16; 2],
    current: [u16; 2],
    current_consumed: [u16; 2],
}

impl DownlinkTelemetryMessage for BatteryMessage {
    const ID: u8 = 0x04;
    type Input = (BatteryStatus, Option<BatteryStatus>);
    type Output = (
        BatteryStatus,
        BatteryInfo,
        Option<BatteryStatus>,
        Option<BatteryInfo>,
    );
}

/// TODO
pub struct DiagnosticsMessage {
    time_since_boot_ms: u32,
    // sensor health
    gyro_healthy: bool,
    accel_healthy: bool,
    mag_healthy: bool,
    absolute_pressure_healthy: bool,
    differential_pressure_healthy: bool,
    gps_healthy: bool,
    other_positioning_healthy: bool,
    battery_healthy: bool,
    // subsystems / outputs / other checks
    prearm_check: bool,
    ahrs_healthy: bool,
    rc_link_healthy: bool,
    propulsion_healthy: bool,
    recovery_system_healthy: bool,
    proximity_or_obstacle: bool,
    geofence_or_terrain: bool,
    motors_reversed: bool,
    // error counts
    load: u16,
    drop_rate_comm: u8,
    communication_errors: u8,
    errors_count1: u8,
    errors_count2: u8,
    errors_count3: u8,
    remote_rssi: (), // TODO
    remote_snr: (),
}

impl DownlinkTelemetryMessage for DiagnosticsMessage {
    const ID: u8 = 0x05;
    type Input = (SysStatus, RadioStatus);
    type Output = (SysStatus, RadioStatus);

    fn pack((sys_status, radio_status): Self::Input) -> Self {
        Self {
            load: sys_status.load,
            drop_rate_comm: sys_status.drop_rate_comm,
        }
    }

    fn unpack<PROFILE>(self) -> Self::Output {
        let sys_status = SysStatus {
            onboard_control_sensors_present: MavSysStatusSensor::empty(),
            onboard_control_sensors_enabled: MavSysStatusSensor::empty(),
            onboard_control_sensors_health: MavSysStatusSensor::empty(),
            load: self.load,
            voltage_battery: u16::MAX,
            current_battery: -1,
            battery_remaining: -1,
            drop_rate_comm: self.drop_rate_comm,
            errors_comm: 0,
            errors_count1: 0,
            errors_count2: 0,
            errors_count3: 0,
            errors_count4: 0,
            ..Default::default()
        };

        let radio_status = RadioStatus {
            ..Default::default()
        };

        (sys_status, radio_status)
    }
}

/// TODO
pub struct BarometersMessage {
    pressures: [f32; 3],
    temperatures: [u8; 3],
    // TODO
}

impl DownlinkTelemetryMessage for BarometersMessage {
    const ID: u8 = 0x06;
    type Input = (
        ScaledPressure,
        Option<ScaledPressure2>,
        Option<ScaledPressure3>,
    );
    type Output = (
        ScaledPressure,
        Option<ScaledPressure2>,
        Option<ScaledPressure3>,
    );
}

// 0x06 reserved for imu message

/// TODO
pub struct PressureVesselsMessage {
    id_block: u8,
    pressure1: [u16; 2],
    pressure2: [u16; 2],
    temperature1: [u8; 2],
    temperature2: [u8; 2],
    fill_level: [u8; 2],
}

impl DownlinkTelemetryMessage for PressureVesselsMessage {
    const ID: u8 = 0xff; // TODO
    type Input = ();
    type Output = ();
}

/// TODO
pub struct ActuatorsMessage {
    id_block: u8,
    actuators: [u8; 8],
}

impl DownlinkTelemetryMessage for PressureVesselsMessage {
    const ID: u8 = 0xff; // TODO
    type Input = (GlobalPositionInt, GpsStatus);
    type Output = (GlobalPositionInt, GpsStatus);
}

/// TODO
pub struct StateEstimatorMessage {
    position_xy: (f16, f16),
    acceleration: (f16, f16, f16),
    position_variance: f16,
    horizontal_velocity_variance: f16,
    vertical_velocity_variance: f16,
}

impl DownlinkTelemetryMessage for StateEstimatorMessage {
    const ID: u8 = 0xff; // TODO
    type Input = LocalPositionNedCov;
    type Output = LocalPositionNedCov;
}

/// TODO
pub struct ComponentsMessage {
    id_block: u8,
    modes: [u8; 4],
    flags_or_errors: [u8; 4],
    // TODO
}

impl DownlinkTelemetryMessage for ComponentsMessage {
    const ID: u8 = 0xff; // TODO
    type Input = ();
    type Output = ();
}
