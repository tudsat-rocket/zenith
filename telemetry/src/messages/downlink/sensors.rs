use serde::{Deserialize, Serialize};

use mission::mavlink::VehicleSnapshot;
use rapid_dialect::rapid::messages::{ScaledImu, ScaledPressure};

use super::{ConnectionContext, DownlinkTelemetryMessage};

/// Accelerometer scale: 1/16 g per code, so +/-7.9 g
const M_S2_PER_ACCEL_CODE: f32 = 9.80665 / 16.0;
const MILLI_G_PER_ACCEL_CODE: f32 = 1000.0 / 16.0;

/// Gyro scale: 2 deg/s per code, so +/-254 deg/s.
const DEG_S_PER_GYRO_CODE: f32 = 2.0;
const MILLI_RAD_S_PER_GYRO_CODE: f32 = 34.906_586;

/// Magnetometer scale: the LIS3MDL's configured +/-16 gauss full scale.
const MAG_FULL_SCALE_UT: f32 = 1600.0;
const UT_PER_MAG_CODE: f32 = MAG_FULL_SCALE_UT / (i8::MAX as f32);
const MILLI_GAUSS_PER_MAG_CODE: f32 = UT_PER_MAG_CODE * 10.0;

/// Barometer scale: 0.1 hPa per code, 0 - 6553.4 hPa.
const HPA_PER_PRESSURE_CODE: f32 = 0.1;
const PRESSURE_UNKNOWN: u16 = u16::MAX;
const TEMPERATURE_UNKNOWN: i8 = i8::MIN;

/// Raw sensor readings, first IMU and baro only
#[derive(Debug, Serialize, Deserialize)]
pub struct SensorsMessage {
    /// IMU1 acceleration, 1/16 g per code.
    accel: [i8; 3],
    /// IMU1 angular rate, 2 deg/s per code.
    gyro: [i8; 3],
    /// Magnetometer, [`MAG_FULL_SCALE_UT`] over the code range.
    mag: [i8; 3],
    /// Barometer 1, 0.1 hPa per code, [`PRESSURE_UNKNOWN`] if it has not reported.
    #[serde(with = "postcard::fixint::le")]
    baro_pressure: u16,
    /// Barometer 1, whole degrees C, [`TEMPERATURE_UNKNOWN`] if it has not reported.
    baro_temperature: i8,
}

impl DownlinkTelemetryMessage for SensorsMessage {
    const ID: u8 = 0x05;
    type Input<'a> = &'a VehicleSnapshot<'a>;
    type Output = (ScaledImu, ScaledPressure);

    fn pack(snapshot: Self::Input<'_>) -> Self {
        let readings = snapshot.readings;

        let baro_pressure = readings
            .baro1
            .pressure
            .filter(|p| p.is_finite())
            .map(|hpa| (hpa / HPA_PER_PRESSURE_CODE).clamp(0.0, f32::from(u16::MAX - 1)) as u16)
            .unwrap_or(PRESSURE_UNKNOWN);

        let baro_temperature = readings
            .baro1
            .temperature
            .filter(|t| t.is_finite())
            .map(|c| c.clamp(f32::from(i8::MIN + 1), f32::from(i8::MAX)) as i8)
            .unwrap_or(TEMPERATURE_UNKNOWN);

        Self {
            accel: pack_axes(axes(readings.imu1_accel), M_S2_PER_ACCEL_CODE),
            gyro: pack_axes(axes(readings.imu1_gyro), DEG_S_PER_GYRO_CODE),
            mag: pack_axes(axes(readings.mag), UT_PER_MAG_CODE),
            baro_pressure,
            baro_temperature,
        }
    }

    fn unpack(self, context: &mut ConnectionContext) -> Self::Output {
        let scaled =
            |codes: [i8; 3], per_code: f32| codes.map(|c| (f32::from(c) * per_code) as i16);

        let [xacc, yacc, zacc] = scaled(self.accel, MILLI_G_PER_ACCEL_CODE);
        let [xgyro, ygyro, zgyro] = scaled(self.gyro, MILLI_RAD_S_PER_GYRO_CODE);
        let [xmag, ymag, zmag] = scaled(self.mag, MILLI_GAUSS_PER_MAG_CODE);

        let imu = ScaledImu {
            time_boot_ms: context.time,
            xacc,
            yacc,
            zacc,
            xgyro,
            ygyro,
            zgyro,
            xmag,
            ymag,
            zmag,
            temperature: 0,
        };

        let pressure = ScaledPressure {
            time_boot_ms: context.time,
            press_abs: match self.baro_pressure {
                PRESSURE_UNKNOWN => 0.0,
                code => f32::from(code) * HPA_PER_PRESSURE_CODE,
            },
            press_diff: 0.0,
            temperature: match self.baro_temperature {
                TEMPERATURE_UNKNOWN => i16::MAX,
                c => i16::from(c).saturating_mul(100),
            },
            temperature_press_diff: 0,
        };

        (imu, pressure)
    }
}

/// The readings are nalgebra vectors, which is more than this module needs to know about them.
fn axes<V: Into<[f32; 3]>>(value: Option<V>) -> Option<[f32; 3]> {
    value.map(Into::into)
}

fn pack_axes(value: Option<[f32; 3]>, per_code: f32) -> [i8; 3] {
    value.unwrap_or_default().map(|c| {
        if c.is_finite() {
            (c / per_code).clamp(f32::from(i8::MIN), f32::from(i8::MAX)) as i8
        } else {
            0
        }
    })
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::tests::{SnapshotParts, through_packet};
    use super::*;

    use mission::BaroReading;
    use mission::SensorReadings;
    use nalgebra::Vector3;

    use crate::messages::DownlinkMessage;

    /// Readings at the top of every scale, for the payload-length check.
    pub(crate) fn saturated_readings() -> SensorReadings {
        SensorReadings {
            imu1_accel: Some(Vector3::new(70.0, -70.0, 70.0)),
            imu1_gyro: Some(Vector3::new(250.0, -250.0, 250.0)),
            mag: Some(Vector3::new(120.0, -120.0, 120.0)),
            baro1: BaroReading {
                pressure: Some(1100.0),
                temperature: Some(50.0),
                altitude: Some(0.0),
            },
            ..Default::default()
        }
    }

    #[test]
    fn sensor_readings_survive_the_packet() {
        let mut parts = SnapshotParts::default();
        parts.readings = SensorReadings {
            // One g down, the vehicle sitting on the pad.
            imu1_accel: Some(Vector3::new(0.0, 0.0, 9.80665)),
            imu1_gyro: Some(Vector3::new(10.0, -20.0, 0.0)),
            mag: Some(Vector3::new(20.0, -5.0, 44.0)),
            baro1: BaroReading {
                pressure: Some(1013.2),
                temperature: Some(21.0),
                altitude: Some(0.0),
            },
            ..Default::default()
        };

        let msg = DownlinkMessage::Sensors(SensorsMessage::pack(&parts.snapshot()));
        let DownlinkMessage::Sensors(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        let (imu, pressure) = decoded.unpack(&mut ConnectionContext::init(0));

        // 1 g in mG, within the 1/16 g the code carries.
        assert!(imu.zacc.abs_diff(1000) <= 63, "{}", imu.zacc);
        assert_eq!(imu.xacc, 0);

        // 10 deg/s and -20 deg/s in mrad/s, within one 2 deg/s code.
        assert!((imu.xgyro - 175).abs() <= 35, "{}", imu.xgyro);
        assert!((imu.ygyro + 349).abs() <= 35, "{}", imu.ygyro);

        // 44 uT in mgauss.
        assert!(
            (imu.zmag - 440).abs() <= MILLI_GAUSS_PER_MAG_CODE as i16,
            "{}",
            imu.zmag
        );

        // Ambient, within 0.1 hPa.
        assert!(
            (pressure.press_abs - 1013.2).abs() <= 0.1,
            "{}",
            pressure.press_abs
        );
        assert_eq!(pressure.temperature, 2100);
    }

    #[test]
    fn a_missing_barometer_stays_missing() {
        let parts = SnapshotParts::default();
        let (_, pressure) =
            SensorsMessage::pack(&parts.snapshot()).unpack(&mut ConnectionContext::init(0));

        assert_eq!(pressure.press_abs, 0.0);
        assert_eq!(pressure.temperature, i16::MAX);
    }

    /// Acceleration past the scale has to saturate, not wrap into a plausible small reading.
    #[test]
    fn over_range_readings_saturate() {
        let mut parts = SnapshotParts::default();
        parts.readings.imu1_accel = Some(Vector3::new(500.0, -500.0, 0.0));
        parts.readings.imu1_gyro = Some(Vector3::new(2000.0, 0.0, 0.0));

        let (imu, _) =
            SensorsMessage::pack(&parts.snapshot()).unpack(&mut ConnectionContext::init(0));

        assert!(imu.xacc > 7000, "{}", imu.xacc);
        assert!(imu.yacc < -7000, "{}", imu.yacc);
        assert!(imu.xgyro > 4000, "{}", imu.xgyro);
    }
}
