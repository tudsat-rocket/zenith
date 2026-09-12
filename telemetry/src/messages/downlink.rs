use core::hash::Hasher;

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use mission::mavlink::VehicleSnapshot;
use rapid_dialect::Rapid;
use rapid_dialect::rapid::messages::RadioStatus;
use siphasher::sip::SipHasher;
use utils::anychannel::AnySender;

use crate::DOWNLINK_MESSAGE_INTERVAL_MS;
use crate::TelemetryError;
use crate::messages::TelemetryMessage;

mod components;
mod heartbeat;
mod pressures;
mod sensors;
mod status;

pub use components::ComponentsMessage;
pub use heartbeat::HeartbeatMessage;
pub use pressures::PressuresMessage;
pub use sensors::SensorsMessage;
pub use status::StatusMessage;

pub const DOWNLINK_PACKET_SIZE: usize = 16;
const DOWNLINK_PAYLOAD_SIZE: usize = DOWNLINK_PACKET_SIZE - 4;

const UNKNOWN: u8 = u8::MAX;
const FULL_SCALE: f32 = (u8::MAX - 1) as f32;

const TEMPERATURE_OFFSET_C: f32 = 60.0;
const TEMPERATURE_CODES_PER_C: f32 = 2.0;

/// Stuff the telemetry receiver can just "keep in mind" based on previously received messages.
/// These are used to enrich received data, especially when values can reasonably be converted or
/// reconstructed from other messages, such as conversion between different altitude reference
/// frames.
pub struct ConnectionContext {
    /// The receivers current best guess at the absolute time since boot in ms.
    /// Since the main time included in every packet frequently overflows, this may "jump" when the
    /// absolute time since boot is discovered to be higher than the previous estimate.
    pub time: u32,
    /// The last received ground altitude in meters above sea-level, if known
    pub altitude_ground_asl: Option<f32>,
    /// Received signal strength indicator (RSSI) of downlink packets received.
    /// Used to enrich RADIO_STATUS messages with ground-side information.
    pub rx_rssi: Option<u8>,
    /// Noise level on the ground; calculated from RSSI and signal-to-noise ratio (SNR).
    /// Used to enrich RADIO_STATUS messages with ground-side information.
    pub rx_noise: Option<u8>,
    /// Downlink packet loss (in percent).
    /// Used to enrich RADIO_STATUS messages with ground-side information.
    pub rx_packet_loss: Option<u16>,
}

impl ConnectionContext {
    /// The packet time covers bits 5..15 of the time since boot, so it wraps this often.
    const TIME_WRAP_MS: u32 = 1 << 15;

    pub fn init(time: u16) -> Self {
        Self {
            time: time as u32,
            altitude_ground_asl: None,
            rx_rssi: None,
            rx_noise: None,
            rx_packet_loss: None,
        }
    }

    /// Extend the coarse time of a freshly received packet into our absolute time estimate, by
    /// counting the wraps we have seen since the start of the connection. Must be called for
    /// every received packet, before unpacking it.
    pub fn advance(&mut self, packet_time: u16) {
        let wraps = self.time >> 15;
        let extended = wraps
            .saturating_mul(Self::TIME_WRAP_MS)
            .saturating_add(u32::from(packet_time));

        self.time = if extended < self.time {
            extended.saturating_add(Self::TIME_WRAP_MS)
        } else {
            extended
        };
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub enum DownlinkMessage {
    Heartbeat(HeartbeatMessage),
    Status(StatusMessage),
    Pressures(PressuresMessage),
    Components(ComponentsMessage),
    Sensors(SensorsMessage),
    // 0x06 is reserved for a GPS message.
}

impl DownlinkMessage {
    /// How many packets the pattern in [`Self::for_tick`] repeats over. One cycle is
    /// `SLOT_COUNT * DOWNLINK_MESSAGE_INTERVAL_MS`, or 256 ms.
    const SLOT_COUNT: u32 = 8;

    /// Which message goes out on this tick of the vehicle's main loop, or `None` on the ticks
    /// between packets. The RF counterpart of `VehicleSnapshot::send_telemetry`, which decides the
    /// same thing for the full-bandwidth links.
    ///
    /// Unlike that schedule the interval is fixed by the protocol rather than chosen per message:
    /// the receiver has to know when to expect a packet in order to follow the hopping sequence,
    /// and it infers packet loss from the packets that fail to arrive, so we leave no gaps.
    ///
    /// The pattern itself is fixed for now, may be dynamic based on flight mode later.
    pub fn for_tick(
        time_ms: u32,
        snapshot: &VehicleSnapshot<'_>,
        uplink: RadioStatus,
    ) -> Option<Self> {
        if time_ms % DOWNLINK_MESSAGE_INTERVAL_MS != 0 {
            return None;
        }

        #[allow(
            clippy::arithmetic_side_effects,
            reason = "DOWNLINK_MESSAGE_INTERVAL_MS and SLOT_COUNT are nonzero constants"
        )]
        let slot = (time_ms / DOWNLINK_MESSAGE_INTERVAL_MS) % Self::SLOT_COUNT;

        Some(match slot {
            1 => Self::Pressures(PressuresMessage::pack(snapshot)),
            3 => Self::Components(ComponentsMessage::pack(snapshot)),
            5 => Self::Status(StatusMessage::pack((snapshot, uplink))),
            7 => Self::Sensors(SensorsMessage::pack(snapshot)),
            // Every other slot, so the heartbeat keeps half the link to itself.
            _ => Self::Heartbeat(HeartbeatMessage::pack(snapshot)),
        })
    }
}

impl TelemetryMessage for DownlinkMessage {
    type Packet = [u8; DOWNLINK_PACKET_SIZE];

    type Input = Rapid;
    type Output = Rapid;

    fn encode(
        self,
        time: u16,
        hmac_key: &[u8; 16],
    ) -> Result<[u8; DOWNLINK_PACKET_SIZE], TelemetryError> {
        // TODO: some enum variant macro-fuckery?
        let (id, payload) = match self {
            Self::Heartbeat(inner) => (HeartbeatMessage::ID, inner.serialize()?),
            Self::Status(inner) => (StatusMessage::ID, inner.serialize()?),
            Self::Pressures(inner) => (PressuresMessage::ID, inner.serialize()?),
            Self::Components(inner) => (ComponentsMessage::ID, inner.serialize()?),
            Self::Sensors(inner) => (SensorsMessage::ID, inner.serialize()?),
        };

        let mut buffer = [0x00; DOWNLINK_PACKET_SIZE];
        buffer[0] = (time >> 7) as u8;
        buffer[1] = (time & 0b1110_0000) as u8 | (id & 0b11111);
        buffer[2..(DOWNLINK_PACKET_SIZE - 2)].copy_from_slice(&payload);

        let mut siphasher = SipHasher::new_with_key(hmac_key);
        siphasher.write(&buffer[..(DOWNLINK_PACKET_SIZE - 2)]);
        let hmac: u16 = siphasher.finish() as u16;
        let hmac_bytes = hmac.to_be_bytes();

        buffer[DOWNLINK_PACKET_SIZE - 2] = hmac_bytes[0];
        buffer[DOWNLINK_PACKET_SIZE - 1] = hmac_bytes[1];

        Ok(buffer)
    }

    fn decode(
        buffer: [u8; DOWNLINK_PACKET_SIZE],
        hmac_key: &[u8; 16],
    ) -> Result<(u16, Self), TelemetryError> {
        let mut siphasher = SipHasher::new_with_key(hmac_key);
        siphasher.write(&buffer[..(DOWNLINK_PACKET_SIZE - 2)]);

        let hmac = u16::from_be_bytes([
            buffer[DOWNLINK_PACKET_SIZE - 2],
            buffer[DOWNLINK_PACKET_SIZE - 1],
        ]);

        if hmac != (siphasher.finish() as u16) {
            return Err(TelemetryError::HmacMismatch);
        }

        let time = ((buffer[0] as u16) << 7) | (buffer[1] as u16 & 0b1110_0000);
        let payload = &buffer[2..(DOWNLINK_PACKET_SIZE - 2)];

        let msg_id = buffer[1] & 0b11111;
        let msg = match msg_id {
            HeartbeatMessage::ID => DownlinkMessage::Heartbeat(postcard::from_bytes(payload)?),
            StatusMessage::ID => DownlinkMessage::Status(postcard::from_bytes(payload)?),
            PressuresMessage::ID => DownlinkMessage::Pressures(postcard::from_bytes(payload)?),
            ComponentsMessage::ID => DownlinkMessage::Components(postcard::from_bytes(payload)?),
            SensorsMessage::ID => DownlinkMessage::Sensors(postcard::from_bytes(payload)?),
            id => {
                return Err(TelemetryError::UnknownMessageId(id));
            }
        };

        Ok((time, msg))
    }

    async fn unpack<S: AnySender<Self::Output>>(
        self,
        sender: &mut S,
        context: &mut ConnectionContext,
    ) {
        match self {
            Self::Heartbeat(inner) => {
                let (h, l, a, al, v) = inner.unpack(context);
                sender.anysend(Rapid::Heartbeat(h)).await;
                sender.anysend(Rapid::LocalPositionNed(l)).await;
                sender.anysend(Rapid::Attitude(a)).await;
                sender.anysend(Rapid::Altitude(al)).await;
                sender.anysend(Rapid::VfrHud(v)).await;
            }
            Self::Status(inner) => {
                let (sys, radio, time) = inner.unpack(context);
                sender.anysend(Rapid::SysStatus(sys)).await;
                sender.anysend(Rapid::RadioStatus(radio)).await;
                sender.anysend(Rapid::SystemTime(time)).await;
            }
            Self::Pressures(inner) => {
                for vessel in inner.unpack(context) {
                    sender.anysend(Rapid::PressureVessel(vessel)).await;
                }
            }
            Self::Components(inner) => {
                for valve in inner.unpack(context) {
                    sender.anysend(Rapid::Valve(valve)).await;
                }
            }
            Self::Sensors(inner) => {
                let (imu, pressure) = inner.unpack(context);
                sender.anysend(Rapid::ScaledImu(imu)).await;
                sender.anysend(Rapid::ScaledPressure(pressure)).await;
            }
        }
    }
}

pub trait DownlinkTelemetryMessage: Sized + Serialize + DeserializeOwned {
    const ID: u8;

    type Input<'a>;
    type Output;

    fn serialize(self) -> Result<[u8; DOWNLINK_PAYLOAD_SIZE], postcard::Error> {
        let mut buf = [0x00; DOWNLINK_PAYLOAD_SIZE];
        postcard::to_slice(&self, &mut buf)?;
        Ok(buf)
    }

    fn pack(input: Self::Input<'_>) -> Self;
    fn unpack(self, context: &mut ConnectionContext) -> Self::Output;
}

/// Scales a reading against the full-scale range of the sensor that produced it. Sensor ranges on
/// this vehicle span two orders of magnitude, so one shared scale would be useless at one end.
fn pack_full_scale(value: Option<f32>, full_scale: f32) -> u8 {
    match value.filter(|v| v.is_finite()) {
        Some(v) => (v / full_scale * FULL_SCALE).clamp(0.0, FULL_SCALE) as u8,
        None => UNKNOWN,
    }
}

fn unpack_full_scale(code: u8, full_scale: f32) -> Option<f32> {
    (code != UNKNOWN).then(|| f32::from(code) / FULL_SCALE * full_scale)
}

/// Packs a temperature into -60.0 - +67.0 C at 0.5 C.
fn pack_temperature(celsius: Option<f32>) -> u8 {
    match celsius.filter(|v| v.is_finite()) {
        Some(c) => {
            ((c + TEMPERATURE_OFFSET_C) * TEMPERATURE_CODES_PER_C).clamp(0.0, FULL_SCALE) as u8
        }
        None => UNKNOWN,
    }
}

fn unpack_temperature(code: u8) -> Option<f32> {
    (code != UNKNOWN).then(|| f32::from(code) / TEMPERATURE_CODES_PER_C - TEMPERATURE_OFFSET_C)
}

/// Serializes an [`half::f16`] as its fixed-width bit pattern, for `#[serde(with = ...)]`.
///
/// Postcard varint-encodes anything wider than a byte, including the `u16` behind an `f16`, which
/// would make the length of a payload depend on the values in it. Our packets are a fixed size, so
/// a payload that only fits for small values is a payload that overflows in flight.
mod f16_le {
    use half::f16;
    use serde::{Deserializer, Serializer};

    #[allow(
        clippy::trivially_copy_pass_by_ref,
        reason = "signature is fixed by serde's `with` contract"
    )]
    pub fn serialize<S: Serializer>(value: &f16, serializer: S) -> Result<S::Ok, S::Error> {
        postcard::fixint::le::serialize(&value.to_bits(), serializer)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<f16, D::Error> {
        postcard::fixint::le::deserialize(deserializer).map(f16::from_bits)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    use core::num::Wrapping;

    use mission::bus::{BusInputImage, BusOutputImage};
    use mission::{SensorReadings, StateMachineParams};
    use rapid_dialect::FlightMode;
    use state_estimator::{StateEstimator, StateEstimatorParams};

    /// Everything a [`VehicleSnapshot`] borrows, owned, so tests can build one without a
    /// `mission::Vehicle` and the four hardware traits it is generic over.
    pub(crate) struct SnapshotParts {
        pub time: Wrapping<u32>,
        pub mode: FlightMode,
        pub params: StateMachineParams,
        pub readings: SensorReadings,
        pub estimator: StateEstimator,
        pub inputs: BusInputImage,
        pub outputs: BusOutputImage,
    }

    impl Default for SnapshotParts {
        fn default() -> Self {
            Self {
                time: Wrapping(0),
                mode: FlightMode::Idle,
                params: StateMachineParams::default(),
                readings: SensorReadings::default(),
                estimator: StateEstimator::new(1000.0, StateEstimatorParams::default()),
                inputs: BusInputImage::default(),
                outputs: BusOutputImage::default(),
            }
        }
    }

    impl SnapshotParts {
        pub(crate) fn snapshot(&self) -> VehicleSnapshot<'_> {
            VehicleSnapshot {
                time: self.time,
                mode: self.mode,
                state_machine_params: &self.params,
                readings: &self.readings,
                state_estimator: &self.estimator,
                input_image: &self.inputs,
                output_image: &self.outputs,
            }
        }
    }

    pub(crate) fn encoded_len<M: Serialize>(msg: &M) -> usize {
        let mut buf = [0x00; DOWNLINK_PAYLOAD_SIZE];
        postcard::to_slice(msg, &mut buf)
            .expect("message did not fit its payload")
            .len()
    }

    /// Round-trips a message through a whole packet, so tests assert on what the receiver actually
    /// recovers rather than on the struct the vehicle built.
    pub(crate) fn through_packet(msg: DownlinkMessage) -> DownlinkMessage {
        const KEY: [u8; 16] = [0x42; 16];

        let packet = msg
            .encode(1234, &KEY)
            .expect("message did not fit its packet");
        DownlinkMessage::decode(packet, &KEY)
            .expect("packet did not decode")
            .1
    }

    /// The coarse packet time wraps every 32.768s; the receiver's estimate must not.
    #[test]
    fn absolute_time_survives_the_packet_time_wrapping() {
        let mut context = ConnectionContext::init(32_000);
        assert_eq!(context.time, 32_000);

        context.advance(32_700);
        assert_eq!(context.time, 32_700);

        // Wrapped once.
        context.advance(100);
        assert_eq!(context.time, 32_868);

        context.advance(32_000);
        assert_eq!(context.time, 64_768);

        // And again.
        context.advance(500);
        assert_eq!(context.time, 66_036);
    }

    #[test]
    fn full_scale_scaling_round_trips_within_one_code() {
        for full_scale in [2.0, 55.0, 60.0, 300.0] {
            for fraction in [0.0, 0.01, 0.5, 0.99, 1.0] {
                let value = full_scale * fraction;
                let recovered =
                    unpack_full_scale(pack_full_scale(Some(value), full_scale), full_scale)
                        .expect("a finite reading came back as unknown");
                assert!(
                    (recovered - value).abs() <= full_scale / FULL_SCALE,
                    "{value} of {full_scale} came back as {recovered}"
                );
            }
        }

        assert_eq!(unpack_full_scale(pack_full_scale(None, 55.0), 55.0), None);
        assert_eq!(
            unpack_full_scale(pack_full_scale(Some(f32::NAN), 55.0), 55.0),
            None
        );
        // Over-range saturates rather than wrapping into a plausible low reading.
        assert_eq!(pack_full_scale(Some(1000.0), 55.0), u8::MAX - 1);
        assert_eq!(pack_full_scale(Some(-1000.0), 55.0), 0);
    }

    #[test]
    fn temperature_scaling_round_trips_within_half_a_degree() {
        for celsius in [-60.0, -30.0, 0.0, 20.0, 36.4, 67.0] {
            let recovered = unpack_temperature(pack_temperature(Some(celsius)))
                .expect("a finite temperature came back as unknown");
            assert!(
                (recovered - celsius).abs() <= 0.5,
                "{celsius} C came back as {recovered}"
            );
        }

        assert_eq!(unpack_temperature(pack_temperature(None)), None);
    }

    /// One packet per interval, no gaps, and the slot pattern the module documents.
    #[test]
    fn the_slot_pattern_repeats_and_leaves_no_gaps() {
        let parts = SnapshotParts::default();
        let snapshot = parts.snapshot();

        let mut built = 0;
        for time_ms in 0..(DownlinkMessage::SLOT_COUNT * DOWNLINK_MESSAGE_INTERVAL_MS * 2) {
            let Some(msg) = DownlinkMessage::for_tick(time_ms, &snapshot, RadioStatus::default())
            else {
                continue;
            };
            built += 1;

            let slot = (time_ms / DOWNLINK_MESSAGE_INTERVAL_MS) % DownlinkMessage::SLOT_COUNT;
            let expected = match slot {
                1 => PressuresMessage::ID,
                3 => ComponentsMessage::ID,
                5 => StatusMessage::ID,
                7 => SensorsMessage::ID,
                _ => HeartbeatMessage::ID,
            };

            let packet = msg
                .encode(time_ms as u16, &[0x42; 16])
                .expect("message did not fit its packet");
            assert_eq!(packet[1] & 0b11111, expected, "slot {slot} at {time_ms} ms");
        }

        assert_eq!(built, DownlinkMessage::SLOT_COUNT * 2);
    }

    /// Every payload has to be the same length whatever the vehicle put in it: the packet is a
    /// fixed size, so a value-dependent length is a packet that overflows in flight rather than on
    /// the bench. This is what catches a field that lost its `fixint` annotation.
    #[test]
    fn no_payload_length_depends_on_its_values() {
        fn lengths(parts: &SnapshotParts) -> [usize; 5] {
            let s = parts.snapshot();
            [
                encoded_len(&HeartbeatMessage::pack(&s)),
                encoded_len(&StatusMessage::pack((&s, RadioStatus::default()))),
                encoded_len(&PressuresMessage::pack(&s)),
                encoded_len(&ComponentsMessage::pack(&s)),
                encoded_len(&SensorsMessage::pack(&s)),
            ]
        }

        let mut parts = SnapshotParts::default();
        let empty = lengths(&parts);

        // Every reading present and large, which is where varints would have grown.
        parts.readings = sensors::tests::saturated_readings();
        parts.inputs = pressures::tests::saturated_inputs();
        parts.outputs = components::tests::saturated_outputs();
        parts.estimator = heartbeat::tests::flying_estimator();
        let full = lengths(&parts);

        assert_eq!(empty, full, "a payload length depends on the values in it");
        for len in full {
            assert!(len <= DOWNLINK_PAYLOAD_SIZE);
        }
    }
}
