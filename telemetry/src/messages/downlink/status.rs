use serde::{Deserialize, Serialize};

use links::protocols::log::{StorageUsage, storage_information};
use mission::flight_log::LOG_USAGE;
use mission::mavlink::{VehicleSnapshot, autopilot_version};
use rapid_dialect::rapid::enums::{MavSysStatusSensor, MavSysStatusSensorExtended};
use rapid_dialect::rapid::messages::{
    AutopilotVersion, RadioStatus, StorageInformation, SysStatus, SystemTime,
};

use super::component_status::set_error_counts;
use super::{ConnectionContext, DownlinkTelemetryMessage, UNKNOWN};

/// Subset of SYS_STATUS bits we actually care about
const TELEMETERED_SENSORS: [MavSysStatusSensor; 12] = [
    MavSysStatusSensor::_3D_GYRO,
    MavSysStatusSensor::_3D_ACCEL,
    MavSysStatusSensor::_3D_MAG,
    MavSysStatusSensor::ABSOLUTE_PRESSURE,
    MavSysStatusSensor::GPS,
    MavSysStatusSensor::MOTOR_OUTPUTS,
    MavSysStatusSensor::MAV_SYS_STATUS_AHRS,
    MavSysStatusSensor::MAV_SYS_STATUS_LOGGING,
    MavSysStatusSensor::BATTERY,
    MavSysStatusSensor::MAV_SYS_STATUS_PREARM_CHECK,
    MavSysStatusSensor::PROPULSION,
    MavSysStatusSensor::MAV_SYS_STATUS_EXTENSION_USED,
];

/// The one extended sensor bit this vehicle uses, packed above the table.
const RECOVERY_SYSTEM_BIT: usize = TELEMETERED_SENSORS.len();

/// 0 - 25.5 V for 8 bits
const BATTERY_MV_PER_CODE: u16 = 100;

/// The log usage byte: the capacity as a power of two MiB in the top bits, how full it is in
/// 31sts below.
const LOG_CAPACITY_SHIFT: u8 = 5;
const LOG_FILL_MAX: u8 = (1 << LOG_CAPACITY_SHIFT) - 1;
/// A capacity exponent no flash has, for no usable flash at all.
const LOG_NO_STORAGE: u8 = u8::MAX >> LOG_CAPACITY_SHIFT;
const MIB: u32 = 1024 * 1024;

/// Vehicle health and link quality.
#[derive(Debug, Serialize, Deserialize)]
pub struct StatusMessage {
    /// [`TELEMETERED_SENSORS`] bits, from SYS_STATUS.onboard_control_sensors_enabled.
    #[serde(with = "postcard::fixint::le")]
    sensors_enabled: u16,
    /// [`TELEMETERED_SENSORS`] bits, from SYS_STATUS.onboard_control_sensors_health.
    #[serde(with = "postcard::fixint::le")]
    sensors_health: u16,
    /// Main bus voltage in 100mV, [`UNKNOWN`] if the ADC has not reported.
    battery_voltage: u8,
    /// Main loop load in percent (SYS_STATUS carries deci-percent).
    load: u8,
    uplink_packet_loss: u8,
    uplink_rssi: u8,
    uplink_noise: u8,
    /// The bits above the time bits in the header, which together give us the full time since boot.
    #[serde(with = "postcard::fixint::le")]
    absolute_time: u16,
    /// The flight log's flash usage, see [`pack_log_usage`].
    log_usage: u8,
}

impl DownlinkTelemetryMessage for StatusMessage {
    const ID: u8 = 0x02;
    type Input<'a> = (&'a VehicleSnapshot<'a>, RadioStatus);
    type Output = (
        SysStatus,
        RadioStatus,
        SystemTime,
        AutopilotVersion,
        StorageInformation,
    );

    fn pack((snapshot, radio_status): Self::Input<'_>) -> Self {
        let sys_status: SysStatus = snapshot.into();

        let battery_voltage = match sys_status.voltage_battery {
            u16::MAX => UNKNOWN,
            mv => (mv / BATTERY_MV_PER_CODE).min(u16::from(UNKNOWN - 1)) as u8,
        };

        Self {
            sensors_enabled: pack_sensors(
                sys_status.onboard_control_sensors_enabled,
                sys_status.onboard_control_sensors_enabled_extended,
            ),
            sensors_health: pack_sensors(
                sys_status.onboard_control_sensors_health,
                sys_status.onboard_control_sensors_health_extended,
            ),
            battery_voltage,
            load: (sys_status.load / 10) as u8,
            absolute_time: (snapshot.time.0 >> ConnectionContext::PACKET_TIME_BITS) as u16,
            uplink_packet_loss: radio_status.fixed as u8,
            uplink_rssi: radio_status.remrssi,
            uplink_noise: radio_status.remnoise,
            log_usage: pack_log_usage(LOG_USAGE.get()),
        }
    }

    fn unpack(self, context: &mut ConnectionContext) -> Self::Output {
        context.anchor_time(self.absolute_time);

        let present = TELEMETERED_SENSORS
            .into_iter()
            .fold(MavSysStatusSensor::empty(), |acc, s| acc | s);
        let (enabled, enabled_extended) = unpack_sensors(self.sensors_enabled);
        let (health, health_extended) = unpack_sensors(self.sensors_health);

        let mut sys_status = SysStatus {
            onboard_control_sensors_present: present,
            onboard_control_sensors_enabled: enabled,
            onboard_control_sensors_health: health,
            onboard_control_sensors_present_extended:
                MavSysStatusSensorExtended::MAV_SYS_STATUS_RECOVERY_SYSTEM,
            onboard_control_sensors_enabled_extended: enabled_extended,
            onboard_control_sensors_health_extended: health_extended,
            load: u16::from(self.load).saturating_mul(10),
            voltage_battery: match self.battery_voltage {
                UNKNOWN => u16::MAX,
                code => u16::from(code).saturating_mul(BATTERY_MV_PER_CODE),
            },
            current_battery: -1,
            battery_remaining: -1,
            ..Default::default()
        };

        set_error_counts(&mut sys_status, context.self_errors.unwrap_or_default());

        let radio_status = RadioStatus {
            rssi: context.rx_rssi.unwrap_or(u8::MAX),
            remrssi: self.uplink_rssi,
            noise: context.rx_noise.unwrap_or(u8::MAX),
            remnoise: self.uplink_noise,
            // we abuse the rxerrors and fixed fields for packet loss
            rxerrors: context.rx_packet_loss.unwrap_or_default(),
            fixed: self.uplink_packet_loss as u16,
            txbuf: 100,
        };

        let system_time = SystemTime {
            time_boot_ms: context.time,
            time_unix_usec: 0,
        };

        let storage = storage_information(context.time, unpack_log_usage(self.log_usage));

        (
            sys_status,
            radio_status,
            system_time,
            autopilot_version(),
            storage,
        )
    }
}

fn pack_sensors(base: MavSysStatusSensor, extended: MavSysStatusSensorExtended) -> u16 {
    let mut bits = 0u16;

    for (i, sensor) in TELEMETERED_SENSORS.into_iter().enumerate() {
        if base.contains(sensor) {
            bits |= 1u16 << i;
        }
    }

    if extended.contains(MavSysStatusSensorExtended::MAV_SYS_STATUS_RECOVERY_SYSTEM) {
        bits |= 1u16 << RECOVERY_SYSTEM_BIT;
    }

    bits
}

/// Squeezes the log usage into a byte. The capacity comes back rounded up to a power of two MiB,
/// which every W25Q is (less the param region), and the fill is rounded up so that an empty log is
/// the only one that reads as empty.
fn pack_log_usage(usage: Option<StorageUsage>) -> u8 {
    let Some(StorageUsage { total, used }) = usage else {
        return LOG_NO_STORAGE << LOG_CAPACITY_SHIFT;
    };
    let capacity = total.max(MIB).next_power_of_two();
    let exponent = (capacity / MIB).trailing_zeros() as u8;
    if exponent >= LOG_NO_STORAGE {
        return LOG_NO_STORAGE << LOG_CAPACITY_SHIFT;
    }

    let fill = (u64::from(used).saturating_mul(u64::from(LOG_FILL_MAX)))
        .div_ceil(u64::from(total))
        .min(u64::from(LOG_FILL_MAX)) as u8;
    (exponent << LOG_CAPACITY_SHIFT) | fill
}

#[allow(
    clippy::arithmetic_side_effects,
    reason = "the exponent is at most 6 and LOG_FILL_MAX is a nonzero constant"
)]
fn unpack_log_usage(code: u8) -> Option<StorageUsage> {
    let exponent = code >> LOG_CAPACITY_SHIFT;
    if exponent == LOG_NO_STORAGE {
        return None;
    }
    let total = MIB << exponent;
    let used =
        u64::from(total).saturating_mul(u64::from(code & LOG_FILL_MAX)) / u64::from(LOG_FILL_MAX);
    Some(StorageUsage {
        total,
        used: used as u32,
    })
}

fn unpack_sensors(bits: u16) -> (MavSysStatusSensor, MavSysStatusSensorExtended) {
    let mut base = MavSysStatusSensor::empty();

    for (i, sensor) in TELEMETERED_SENSORS.into_iter().enumerate() {
        if bits & (1u16 << i) != 0 {
            base |= sensor;
        }
    }

    let extended = if bits & (1u16 << RECOVERY_SYSTEM_BIT) != 0 {
        MavSysStatusSensorExtended::MAV_SYS_STATUS_RECOVERY_SYSTEM
    } else {
        MavSysStatusSensorExtended::empty()
    };

    (base, extended)
}

#[cfg(test)]
mod tests {
    use super::super::tests::{SnapshotParts, through_packet};
    use super::*;

    use core::num::Wrapping;

    use mission::AdcData;
    use mission::SensorReadings;
    use rapid_dialect::rapid::messages::SysStatus;

    use crate::messages::DownlinkMessage;

    /// A receiver that starts listening mid-flight has only the low bits of the time, and no way
    /// to know how many wraps it missed, until one of these turns up with the rest.
    #[test]
    fn a_status_packet_anchors_the_time_since_boot() {
        const BOOT_MS: u32 = 10 * 60 * 1000;

        let parts = SnapshotParts {
            time: Wrapping(BOOT_MS),
            ..Default::default()
        };

        let msg = DownlinkMessage::Status(StatusMessage::pack((
            &parts.snapshot(),
            RadioStatus::default(),
        )));
        let DownlinkMessage::Status(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };

        // What the header alone gave the receiver when it acquired: the low bits, nothing more.
        let mut context = ConnectionContext::init(BOOT_MS as u16);
        assert_ne!(context.time, BOOT_MS);

        let (_, _, system_time, _, _) = decoded.unpack(&mut context);

        assert_eq!(context.time, BOOT_MS);
        assert_eq!(system_time.time_boot_ms, BOOT_MS);
    }

    /// The LUT is a hand-picked subset, so a sensor bit that `mission` starts reporting would
    /// otherwise be dropped on the RF link and nowhere else. `present` is exactly the set the
    /// vehicle can ever report, which makes it the right thing to check against.
    #[test]
    fn no_sensor_bit_is_dropped() {
        let parts = SnapshotParts::default();
        let sys_status: SysStatus = (&parts.snapshot()).into();

        let telemetered = TELEMETERED_SENSORS
            .into_iter()
            .fold(MavSysStatusSensor::empty(), |acc, s| acc | s);

        assert_eq!(
            sys_status.onboard_control_sensors_present, telemetered,
            "the vehicle reports a sensor the downlink LUT does not carry"
        );
        assert_eq!(
            sys_status.onboard_control_sensors_present_extended,
            MavSysStatusSensorExtended::MAV_SYS_STATUS_RECOVERY_SYSTEM,
        );
    }

    #[test]
    fn sensor_bits_survive_the_packet() {
        let mut parts = SnapshotParts::default();
        parts.readings = SensorReadings {
            imu1_gyro: Some(Default::default()),
            imu2_gyro: Some(Default::default()),
            imu3_gyro: Some(Default::default()),
            mag: Some(Default::default()),
            power: Some(AdcData {
                bus_main_voltage: 12_400,
                ..Default::default()
            }),
            ..Default::default()
        };

        let snapshot = parts.snapshot();
        let sent: SysStatus = (&snapshot).into();

        let msg = DownlinkMessage::Status(StatusMessage::pack((&snapshot, RadioStatus::default())));
        let DownlinkMessage::Status(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        let (received, _, _, _, _) = decoded.unpack(&mut ConnectionContext::init(0));

        assert_eq!(
            received.onboard_control_sensors_present,
            sent.onboard_control_sensors_present
        );
        assert_eq!(
            received.onboard_control_sensors_enabled,
            sent.onboard_control_sensors_enabled
        );
        assert_eq!(
            received.onboard_control_sensors_health,
            sent.onboard_control_sensors_health
        );
        assert_eq!(
            received.onboard_control_sensors_health_extended,
            sent.onboard_control_sensors_health_extended
        );

        // Within one 100mV code of the 12.4 V the ADC reported.
        assert!(received.voltage_battery.abs_diff(12_400) <= 100);
    }

    #[test]
    fn a_missing_battery_stays_missing() {
        let parts = SnapshotParts::default();
        let snapshot = parts.snapshot();

        let msg = StatusMessage::pack((&snapshot, RadioStatus::default()));
        let (sys_status, _, _, _, _) = msg.unpack(&mut ConnectionContext::init(0));

        assert_eq!(sys_status.voltage_battery, u16::MAX);
    }

    #[test]
    fn log_usage_survives_its_byte() {
        const REGION: u32 = 16 * MIB - 64 * 1024;

        assert_eq!(unpack_log_usage(pack_log_usage(None)), None);

        for used in [0, 1, REGION / 3, REGION - 1, REGION] {
            let usage = unpack_log_usage(pack_log_usage(Some(StorageUsage {
                total: REGION,
                used,
            })))
            .expect("a flash came back as none");

            assert_eq!(usage.total, 16 * MIB);
            assert_eq!(usage.used == 0, used == 0, "{used} bytes");
            assert!(
                usage.used.abs_diff(used) <= usage.total / u32::from(LOG_FILL_MAX) + 64 * 1024,
                "{used} bytes came back as {}",
                usage.used
            );
        }

        for total in [MIB, 64 * MIB] {
            let usage = unpack_log_usage(pack_log_usage(Some(StorageUsage { total, used: total })));
            assert_eq!(usage, Some(StorageUsage { total, used: total }));
        }
    }
}
