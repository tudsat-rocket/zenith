use serde::{Deserialize, Serialize};

use mission::mavlink::VehicleSnapshot;
use rapid_dialect::rapid::enums::GpsFixType;
use rapid_dialect::rapid::messages::{GlobalPositionInt, GpsRawInt};

use super::{ConnectionContext, DownlinkTelemetryMessage, UNKNOWN};

/// Coordinates travel as an unsigned fraction of their range over three bytes: 1.19 m of latitude
/// and 2.39 m of longitude at the equator.
const COORDINATE_UNKNOWN: u32 = 0xFF_FFFF;
const COORDINATE_FULL_SCALE: f64 = (COORDINATE_UNKNOWN - 1) as f64;
const LATITUDE_RANGE_DEG: f64 = 180.0;
const LONGITUDE_RANGE_DEG: f64 = 360.0;

/// Altitudes are offset and scaled into 16 bits: -1000.0 - +64534.0m
const ALTITUDE_OFFSET_M: f32 = 1000.0;
const ALTITUDE_UNKNOWN: u16 = u16::MAX;

/// Satellite count shares its byte with the fix type.
const FIX_BITS: u8 = 2;
const FIX_MASK: u8 = 0b11;
const SATELLITES_UNKNOWN: u8 = 0b11_1111;

/// The dilution of precision travels in tenths; [`GpsDatum::hdop`] is in hundredths.
const HDOP_CODES_PER_UNIT: u16 = 10;

/// Position and GPS information
///
/// The receiver recovers GPS_RAW_INT and GLOBAL_POSITION_INT.
///
/// Also carries ground altitude as a reference for the receiver, enabling conversions between
/// AGL and ASL.
#[derive(Debug, Serialize, Deserialize)]
pub struct GpsMessage {
    /// `(deg + 90) / 180` of the code range, big-endian, [`COORDINATE_UNKNOWN`] without a fix.
    latitude: [u8; 3],
    /// `(deg + 180) / 360` of the code range, big-endian, [`COORDINATE_UNKNOWN`] without a fix.
    longitude: [u8; 3],
    /// GPS altitude above mean sea level in (m+1000), [`ALTITUDE_UNKNOWN`] without a fix.
    #[serde(with = "postcard::fixint::le")]
    altitude_gps: u16,
    /// Ground altitude above sea level in (m+1000), as latched by the state estimator at arming.
    #[serde(with = "postcard::fixint::le")]
    altitude_ground: u16,
    /// 6 bits of satellite count ([`SATELLITES_UNKNOWN`] if the receiver has not reported),
    /// 2 bits of GPS_FIX_TYPE (only the first four of them are reachable).
    fix_and_satellites: u8,
    /// Horizontal dilution of precision in 0.1, [`UNKNOWN`] if the receiver has not reported.
    hdop: u8,
}

impl DownlinkTelemetryMessage for GpsMessage {
    const ID: u8 = 0x06;
    type Input<'a> = &'a VehicleSnapshot<'a>;
    type Output = (GpsRawInt, GlobalPositionInt);

    fn pack(snapshot: Self::Input<'_>) -> Self {
        let raw: GpsRawInt = snapshot.into();
        let gps = snapshot.readings.gps.as_ref();

        let satellites = match raw.satellites_visible {
            u8::MAX => SATELLITES_UNKNOWN,
            n => n.min(SATELLITES_UNKNOWN - 1),
        };
        let hdop = match raw.eph {
            u16::MAX => UNKNOWN,
            dop => (dop / HDOP_CODES_PER_UNIT).min(u16::from(UNKNOWN - 1)) as u8,
        };

        let fix_and_satellites = (satellites << FIX_BITS) | (raw.fix_type as u8 & FIX_MASK);

        Self {
            latitude: pack_coordinate(gps.and_then(|g| g.latitude), LATITUDE_RANGE_DEG),
            longitude: pack_coordinate(gps.and_then(|g| g.longitude), LONGITUDE_RANGE_DEG),
            altitude_gps: pack_altitude(gps.and_then(|g| g.altitude)),
            altitude_ground: pack_altitude(Some(snapshot.state_estimator.altitude_ground)),
            fix_and_satellites,
            hdop,
        }
    }

    fn unpack(self, context: &mut ConnectionContext) -> Self::Output {
        context.altitude_ground_asl = unpack_altitude(self.altitude_ground);

        let lat = coordinate_dege7(self.latitude, LATITUDE_RANGE_DEG);
        let lon = coordinate_dege7(self.longitude, LONGITUDE_RANGE_DEG);
        let alt = altitude_mm(self.altitude_gps);

        let satellites = match self.fix_and_satellites >> FIX_BITS {
            SATELLITES_UNKNOWN => u8::MAX,
            n => n,
        };
        let eph = match self.hdop {
            UNKNOWN => u16::MAX,
            code => u16::from(code).saturating_mul(HDOP_CODES_PER_UNIT),
        };

        #[allow(
            clippy::arithmetic_side_effects,
            reason = "u32 time widened to u64 before *1000, cannot overflow"
        )]
        let raw = GpsRawInt {
            time_usec: u64::from(context.time) * 1000,
            fix_type: GpsFixType::try_from(self.fix_and_satellites & FIX_MASK)
                .unwrap_or(GpsFixType::NoGps),
            lat,
            lon,
            alt,
            eph,
            // The vertical dilution and the velocity do not fit; the heartbeat carries the
            // vertical and ground speeds instead.
            epv: u16::MAX,
            vel: u16::MAX,
            cog: u16::MAX,
            satellites_visible: satellites,
            alt_ellipsoid: 0,
            h_acc: 0,
            v_acc: 0,
            vel_acc: 0,
            hdg_acc: 0,
            yaw: 0,
        };

        let global = GlobalPositionInt {
            time_boot_ms: context.time,
            lat,
            lon,
            alt,
            relative_alt: (context.altitude_agl.unwrap_or_default() * 1000.0) as i32,
            // GLOBAL_POSITION_INT has no way to say a velocity is unknown.
            vx: 0,
            vy: 0,
            vz: 0,
            hdg: u16::MAX,
        };

        (raw, global)
    }
}

fn truncate(code: u32) -> [u8; 3] {
    let [_, a, b, c] = code.to_be_bytes();
    [a, b, c]
}

fn widen(code: [u8; 3]) -> u32 {
    let [a, b, c] = code;
    u32::from_be_bytes([0, a, b, c])
}

/// Scales a coordinate into the code range its span gets. The float-to-int cast saturates, so a
/// coordinate outside the span clamps rather than wrapping into the opposite hemisphere.
fn pack_coordinate(degrees: Option<f32>, range_deg: f64) -> [u8; 3] {
    let Some(degrees) = degrees.filter(|d| d.is_finite()) else {
        return truncate(COORDINATE_UNKNOWN);
    };

    let code = ((f64::from(degrees) + range_deg / 2.0) / range_deg * COORDINATE_FULL_SCALE)
        .clamp(0.0, COORDINATE_FULL_SCALE) as u32;

    truncate(code)
}

fn unpack_coordinate(code: [u8; 3], range_deg: f64) -> Option<f64> {
    let code = widen(code);

    (code != COORDINATE_UNKNOWN)
        .then(|| f64::from(code) / COORDINATE_FULL_SCALE * range_deg - range_deg / 2.0)
}

/// A coordinate in degE7, which is how both MAVLink messages want it.
fn coordinate_dege7(code: [u8; 3], range_deg: f64) -> i32 {
    unpack_coordinate(code, range_deg).map_or(i32::MAX, |deg| (deg * 1e7) as i32)
}

fn pack_altitude(metres: Option<f32>) -> u16 {
    match metres.filter(|m| m.is_finite()) {
        Some(m) => (m + ALTITUDE_OFFSET_M).clamp(0.0, f32::from(ALTITUDE_UNKNOWN - 1)) as u16,
        None => ALTITUDE_UNKNOWN,
    }
}

fn unpack_altitude(code: u16) -> Option<f32> {
    (code != ALTITUDE_UNKNOWN).then(|| f32::from(code) - ALTITUDE_OFFSET_M)
}

/// Millimetres, which is how both MAVLink messages want their altitudes.
fn altitude_mm(code: u16) -> i32 {
    unpack_altitude(code).map_or(i32::MAX, |m| (m * 1000.0) as i32)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::tests::{SnapshotParts, through_packet};
    use super::*;

    use state_estimator::GpsDatum;

    use crate::messages::DownlinkMessage;

    /// One code of latitude and of longitude, in degrees.
    const LATITUDE_RESOLUTION_DEG: f64 = LATITUDE_RANGE_DEG / COORDINATE_FULL_SCALE;
    const LONGITUDE_RESOLUTION_DEG: f64 = LONGITUDE_RANGE_DEG / COORDINATE_FULL_SCALE;

    /// A fix at the top of every scale, for the payload-length check.
    pub(crate) fn saturated_gps() -> GpsDatum {
        GpsDatum {
            latitude: Some(89.9),
            longitude: Some(179.9),
            altitude: Some(60_000.0),
            hdop: 9999,
            num_satellites: 99,
            seq: u32::MAX,
        }
    }

    fn round_trip(parts: &SnapshotParts) -> (GpsRawInt, GlobalPositionInt) {
        let msg = DownlinkMessage::Gps(GpsMessage::pack(&parts.snapshot()));
        let DownlinkMessage::Gps(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        decoded.unpack(&mut ConnectionContext::init(0))
    }

    fn with_fix(latitude: f32, longitude: f32, altitude: f32) -> SnapshotParts {
        let mut parts = SnapshotParts::default();
        parts.readings.gps = Some(GpsDatum {
            latitude: Some(latitude),
            longitude: Some(longitude),
            altitude: Some(altitude),
            hdop: 120,
            num_satellites: 11,
            seq: 1,
        });
        parts
    }

    #[test]
    fn a_position_survives_the_packet() {
        // The EuRoC launch site.
        let parts = with_fix(39.389_72, -8.288_58, 95.0);
        let (raw, global) = round_trip(&parts);

        assert_eq!(raw.fix_type, GpsFixType::_3dFix);
        assert!(
            (f64::from(raw.lat) / 1e7 - 39.389_72).abs() <= LATITUDE_RESOLUTION_DEG,
            "{}",
            raw.lat
        );
        assert!(
            (f64::from(raw.lon) / 1e7 + 8.288_58).abs() <= LONGITUDE_RESOLUTION_DEG,
            "{}",
            raw.lon
        );
        assert!((raw.alt - 95_000).abs() <= 1000, "{}", raw.alt);

        assert_eq!(raw.satellites_visible, 11);
        // Within one 0.1 code of the HDOP 1.2 the receiver reported.
        assert!(raw.eph.abs_diff(120) <= 10, "{}", raw.eph);

        assert_eq!(global.lat, raw.lat);
        assert_eq!(global.lon, raw.lon);
        assert_eq!(global.alt, raw.alt);
    }

    /// The coordinates are offset into unsigned codes, which is exactly where a sign error hides.
    #[test]
    fn both_hemispheres_survive_the_packet() {
        for (lat, lon) in [
            (39.389_72, -8.288_58),
            (-33.9, 18.4),
            (-45.0, -170.0),
            (67.8, 20.3),
            (0.0, 0.0),
        ] {
            let (raw, _) = round_trip(&with_fix(lat, lon, 0.0));

            assert!(
                (f64::from(raw.lat) / 1e7 - f64::from(lat)).abs() <= LATITUDE_RESOLUTION_DEG,
                "{lat} came back as {}",
                raw.lat
            );
            assert!(
                (f64::from(raw.lon) / 1e7 - f64::from(lon)).abs() <= LONGITUDE_RESOLUTION_DEG,
                "{lon} came back as {}",
                raw.lon
            );
        }
    }

    /// A coordinate or altitude past its range has to saturate, not wrap into a plausible position
    /// on the other side of the planet.
    #[test]
    fn out_of_range_positions_clamp_rather_than_wrap() {
        for (lat, lon, alt, expected) in [
            (
                95.0f32,
                200.0f32,
                70_000.0f32,
                (90.0f64, 180.0f64, 64_534.0f32),
            ),
            (-95.0, -200.0, -3000.0, (-90.0, -180.0, -1000.0)),
        ] {
            let (raw, _) = round_trip(&with_fix(lat, lon, alt));
            let (want_lat, want_lon, want_alt) = expected;

            assert!(
                (f64::from(raw.lat) / 1e7 - want_lat).abs() <= LATITUDE_RESOLUTION_DEG,
                "{lat} came back as {}",
                raw.lat
            );
            assert!(
                (f64::from(raw.lon) / 1e7 - want_lon).abs() <= LONGITUDE_RESOLUTION_DEG,
                "{lon} came back as {}",
                raw.lon
            );
            assert!(
                ((raw.alt as f32) / 1000.0 - want_alt).abs() <= 1.0,
                "{alt} came back as {}",
                raw.alt
            );
        }
    }

    /// A regression that reports a missing fix as null island rather than as unknown fails here.
    #[test]
    fn a_missing_fix_stays_missing() {
        let (raw, _) = round_trip(&SnapshotParts::default());

        assert_eq!(raw.fix_type, GpsFixType::NoGps);
        assert_eq!(raw.lat, i32::MAX);
        assert_eq!(raw.lon, i32::MAX);
        assert_eq!(raw.alt, i32::MAX);
        assert_eq!(raw.eph, u16::MAX);
        assert_eq!(raw.satellites_visible, u8::MAX);
    }

    /// A receiver that sees the satellites but no position reports exactly that.
    #[test]
    fn a_receiver_without_a_position_still_reports_itself() {
        let mut parts = SnapshotParts::default();
        parts.readings.gps = Some(GpsDatum {
            hdop: 9999,
            num_satellites: 3,
            ..Default::default()
        });

        let (raw, _) = round_trip(&parts);

        assert_eq!(raw.fix_type, GpsFixType::NoFix);
        assert_eq!(raw.lat, i32::MAX);
        assert_eq!(raw.satellites_visible, 3);
    }

    /// The whole point of carrying the ground altitude: every AGL on the link becomes an AMSL.
    #[test]
    fn the_ground_altitude_reaches_the_context() {
        use super::super::HeartbeatMessage;

        let mut parts = with_fix(39.389_72, -8.288_58, 95.0);
        parts.estimator.altitude_ground = 92.0;
        parts.estimator.kalman.x[2] = 1092.0;

        let mut context = ConnectionContext::init(0);
        assert_eq!(context.altitude_ground_asl, None);

        let DownlinkMessage::Gps(gps) =
            through_packet(DownlinkMessage::Gps(GpsMessage::pack(&parts.snapshot())))
        else {
            panic!("decoded as the wrong message")
        };
        gps.unpack(&mut context);

        assert_eq!(context.altitude_ground_asl, Some(92.0));

        let DownlinkMessage::Heartbeat(heartbeat) = through_packet(DownlinkMessage::Heartbeat(
            HeartbeatMessage::pack(&parts.snapshot()),
        )) else {
            panic!("decoded as the wrong message")
        };
        let (.., altitude, _) = heartbeat.unpack(&mut context);

        assert!(
            (altitude.altitude_local - 1000.0).abs() <= 0.1,
            "{altitude:?}"
        );
        assert!(
            (altitude.altitude_amsl - 1092.0).abs() <= 0.1,
            "{altitude:?}"
        );
    }

    /// GLOBAL_POSITION_INT has no altitude of its own on this link, so it borrows the heartbeat's.
    #[test]
    fn the_global_position_borrows_the_heartbeats_agl() {
        let parts = with_fix(39.389_72, -8.288_58, 95.0);

        let mut context = ConnectionContext::init(0);
        context.altitude_agl = Some(1234.0);

        let (_, global) = GpsMessage::pack(&parts.snapshot()).unpack(&mut context);

        assert_eq!(global.relative_alt, 1_234_000);
        assert_eq!(global.hdg, u16::MAX);
    }
}
