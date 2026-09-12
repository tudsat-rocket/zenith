use serde::{Deserialize, Serialize};

use mission::mavlink::VehicleSnapshot;
use rapid_dialect::rapid::enums::{MavSysStatusSensor, MavSysStatusSensorExtended};
use rapid_dialect::rapid::messages::{RadioStatus, SysStatus, SystemTime};

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
    /// The left-most 15 bits of the time are already contained in every packet, so we just add 16
    /// more here, for a maximum of 2^31 milliseconds, or ~25days.
    #[serde(with = "postcard::fixint::le")]
    absolute_time: u16,
    reserved: u8,
}

impl DownlinkTelemetryMessage for StatusMessage {
    const ID: u8 = 0x02;
    type Input<'a> = (&'a VehicleSnapshot<'a>, RadioStatus);
    type Output = (SysStatus, RadioStatus, SystemTime);

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
            absolute_time: (snapshot.time.0 >> 15) as u16,
            uplink_packet_loss: radio_status.fixed as u8,
            uplink_rssi: radio_status.remrssi,
            uplink_noise: radio_status.remnoise,
            reserved: 0,
        }
    }

    fn unpack(self, context: &mut ConnectionContext) -> Self::Output {
        let present = TELEMETERED_SENSORS
            .into_iter()
            .fold(MavSysStatusSensor::empty(), |acc, s| acc | s);
        let (enabled, enabled_extended) = unpack_sensors(self.sensors_enabled);
        let (health, health_extended) = unpack_sensors(self.sensors_health);

        let sys_status = SysStatus {
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

        // TODO
        let system_time = SystemTime {
            ..Default::default()
        };

        (sys_status, radio_status, system_time)
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

    use mission::AdcData;
    use mission::SensorReadings;
    use rapid_dialect::rapid::messages::SysStatus;

    use crate::messages::DownlinkMessage;

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
        let (received, _, _) = decoded.unpack(&mut ConnectionContext::init(0));

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
        let (sys_status, _, _) = msg.unpack(&mut ConnectionContext::init(0));

        assert_eq!(sys_status.voltage_battery, u16::MAX);
    }
}
