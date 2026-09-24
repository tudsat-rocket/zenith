use core::hash::Hasher;
use core::time::Duration;

use serde::{Deserialize, Serialize, de::DeserializeOwned};
use siphasher::sip::SipHasher;

use mission::inventory::{InventoryId, ValveId};
use rapid_dialect::rapid::enums::MavResult;
use rapid_dialect::{FlightMode, ValveCommand};

use crate::messages::{ParamEntry, TelemetryMessage};
use crate::{TelemetryError, UplinkCommand};

pub const UPLINK_PACKET_SIZE: usize = 16;
const UPLINK_PAYLOAD_SIZE: usize = UPLINK_PACKET_SIZE - 10;

/// Sequence numbers occupy just 11 bits of the packet header.
pub const UPLINK_SEQ_MODULO: u16 = 1 << 11;

#[derive(Debug, Clone)]
pub enum UplinkMessage {
    Heartbeat(()),
    SetFlightMode(SetFlightModeMessage),
    SetValve(SetValveMessage),
    ParamSet(ParamSetMessage),
    ParamRequest(ParamRequestMessage),
}

impl TelemetryMessage for UplinkMessage {
    type Packet = [u8; UPLINK_PACKET_SIZE];

    type Input = UplinkCommand;
    type Output = (u16, Result<UplinkCommand, MavResult>);

    fn encode(
        self,
        seq: u16,
        hmac_key: &[u8; 16],
    ) -> Result<[u8; UPLINK_PACKET_SIZE], TelemetryError> {
        let (id, payload) = match self {
            Self::Heartbeat(()) => (0x01, [0x00; UPLINK_PAYLOAD_SIZE]),
            Self::SetFlightMode(inner) => (SetFlightModeMessage::ID, inner.serialize()?),
            Self::SetValve(inner) => (SetValveMessage::ID, inner.serialize()?),
            Self::ParamSet(inner) => (ParamSetMessage::ID, inner.serialize()?),
            Self::ParamRequest(inner) => (ParamRequestMessage::ID, inner.serialize()?),
        };

        let mut buffer = [0x00; UPLINK_PACKET_SIZE];
        buffer[0] = (seq >> 3) as u8;
        buffer[1] = (seq << 5) as u8 | (id & 0b11111);
        buffer[2..(UPLINK_PACKET_SIZE - 8)].copy_from_slice(&payload);

        let mut siphasher = SipHasher::new_with_key(hmac_key);
        siphasher.write(&buffer[..(UPLINK_PACKET_SIZE - 8)]);
        let hmac = siphasher.finish();
        let hmac_bytes = hmac.to_be_bytes();

        buffer[UPLINK_PACKET_SIZE - 8] = hmac_bytes[0];
        buffer[UPLINK_PACKET_SIZE - 7] = hmac_bytes[1];
        buffer[UPLINK_PACKET_SIZE - 6] = hmac_bytes[2];
        buffer[UPLINK_PACKET_SIZE - 5] = hmac_bytes[3];
        buffer[UPLINK_PACKET_SIZE - 4] = hmac_bytes[4];
        buffer[UPLINK_PACKET_SIZE - 3] = hmac_bytes[5];
        buffer[UPLINK_PACKET_SIZE - 2] = hmac_bytes[6];
        buffer[UPLINK_PACKET_SIZE - 1] = hmac_bytes[7];

        Ok(buffer)
    }

    fn decode(
        buffer: [u8; UPLINK_PACKET_SIZE],
        hmac_key: &[u8; 16],
    ) -> Result<(u16, Self), TelemetryError> {
        let mut siphasher = SipHasher::new_with_key(hmac_key);
        siphasher.write(&buffer[..(UPLINK_PACKET_SIZE - 8)]);

        let hmac = u64::from_be_bytes([
            buffer[UPLINK_PACKET_SIZE - 8],
            buffer[UPLINK_PACKET_SIZE - 7],
            buffer[UPLINK_PACKET_SIZE - 6],
            buffer[UPLINK_PACKET_SIZE - 5],
            buffer[UPLINK_PACKET_SIZE - 4],
            buffer[UPLINK_PACKET_SIZE - 3],
            buffer[UPLINK_PACKET_SIZE - 2],
            buffer[UPLINK_PACKET_SIZE - 1],
        ]);

        if hmac != siphasher.finish() {
            return Err(TelemetryError::HmacMismatch);
        }

        let seq = ((buffer[0] as u16) << 3) | ((buffer[1] as u16) >> 5);
        let payload = &buffer[2..(UPLINK_PACKET_SIZE - 8)];

        let msg_id = buffer[1] & 0b11111;
        let msg = match msg_id {
            0x01 => UplinkMessage::Heartbeat(()),
            SetFlightModeMessage::ID => {
                UplinkMessage::SetFlightMode(postcard::from_bytes(payload)?)
            }
            SetValveMessage::ID => UplinkMessage::SetValve(postcard::from_bytes(payload)?),
            ParamSetMessage::ID => UplinkMessage::ParamSet(postcard::from_bytes(payload)?),
            ParamRequestMessage::ID => UplinkMessage::ParamRequest(postcard::from_bytes(payload)?),
            id => {
                return Err(TelemetryError::UnknownMessageId(id));
            }
        };

        Ok((seq, msg))
    }

    async fn unpack<S: utils::anychannel::AnySender<Self::Output>>(
        self,
        _sender: &mut S,
        _context: &mut super::ConnectionContext,
    ) {
    }
}

impl UplinkMessage {
    /// What this message asks the vehicle to do, or why it can't, or `None` for a heartbeat.
    pub fn command(self) -> Option<Result<UplinkCommand, MavResult>> {
        Some(match self {
            Self::Heartbeat(()) => return None,
            Self::SetFlightMode(inner) => match inner.mode.try_into() {
                Ok(mode) => Ok(UplinkCommand::SetFlightMode(mode)),
                Err(_) => {
                    defmt::warn!("Rejecting uplink command with invalid flight mode.");
                    Err(MavResult::Denied)
                }
            },
            Self::SetValve(inner) => match inner.command() {
                Some((valve, command)) => Ok(UplinkCommand::CommandValve(valve, command)),
                None => {
                    defmt::warn!("Rejecting uplink command for an unknown valve.");
                    Err(MavResult::Denied)
                }
            },
            Self::ParamSet(ParamSetMessage(ParamEntry { id, raw })) => {
                Ok(UplinkCommand::SetParam { id, raw })
            }
            Self::ParamRequest(inner) => Ok(UplinkCommand::RequestParams {
                first: inner.first,
                mask: inner.mask,
            }),
        })
    }
}

pub trait UplinkTelemetryMessage: Sized + Serialize + DeserializeOwned {
    const ID: u8;

    fn serialize(self) -> Result<[u8; UPLINK_PAYLOAD_SIZE], postcard::Error> {
        let mut buf = [0x00; UPLINK_PAYLOAD_SIZE];
        postcard::to_slice(&self, &mut buf)?;
        Ok(buf)
    }
}

/// 0x02: SetFlightMode
///
/// TODO
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SetFlightModeMessage {
    pub mode: u8,
    // TODO: include an armed bit here?
}

impl From<FlightMode> for UplinkMessage {
    fn from(flightmode: FlightMode) -> Self {
        UplinkMessage::SetFlightMode(SetFlightModeMessage {
            mode: flightmode as u8,
        })
    }
}

impl UplinkTelemetryMessage for SetFlightModeMessage {
    const ID: u8 = 0x02;
}

/// 0x03: SetValve
///
/// Commands one valve, the RF counterpart of MAV_CMD_COMMAND_VALVE.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SetValveMessage {
    /// [`ValveId`] discriminant. Values outside the vehicle's inventory are rejected on receipt.
    valve: u8,
    /// A [`ValveCommandKind`] discriminant.
    command: u8,
    /// Target position for [`ValveCommandKind::Partial`], 0 fully closed and 1000 fully open.
    #[serde(with = "postcard::fixint::le")]
    position: u16,
    /// How long to hold open for [`ValveCommandKind::PulseOpen`].
    #[serde(with = "postcard::fixint::le")]
    duration_ms: u16,
}

/// Which of the [`ValveCommand`] variants a [`SetValveMessage`] carries. `ValveCommand` holds an
/// `f32` and a `Duration`, neither of which belongs on the wire.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
enum ValveCommandKind {
    Close = 0,
    Open = 1,
    Partial = 2,
    PulseOpen = 3,
}

impl ValveCommandKind {
    fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Close),
            1 => Some(Self::Open),
            2 => Some(Self::Partial),
            3 => Some(Self::PulseOpen),
            _ => None,
        }
    }
}

impl UplinkTelemetryMessage for SetValveMessage {
    const ID: u8 = 0x03;
}

impl SetValveMessage {
    /// `ValveCommand::Partial` is a 0..=1 fraction, the bus speaks promille.
    const PROMILLE_PER_UNIT: f32 = 1000.0;

    pub fn new(valve: ValveId, command: ValveCommand) -> Self {
        let (kind, position, duration_ms) = match command {
            ValveCommand::Close => (ValveCommandKind::Close, 0, 0),
            ValveCommand::Open => (ValveCommandKind::Open, 0, 0),
            ValveCommand::Partial(p) => (
                ValveCommandKind::Partial,
                (p * Self::PROMILLE_PER_UNIT).clamp(0.0, Self::PROMILLE_PER_UNIT) as u16,
                0,
            ),
            ValveCommand::PulseOpen(d) => (
                ValveCommandKind::PulseOpen,
                0,
                d.as_millis().min(u128::from(u16::MAX)) as u16,
            ),
        };

        Self {
            valve: valve as u8,
            command: kind as u8,
            position,
            duration_ms,
        }
    }

    /// The command this packet asks for, or `None` if invalid.
    pub fn command(&self) -> Option<(ValveId, ValveCommand)> {
        let valve = ValveId::try_from(self.valve)
            .ok()
            .filter(|v| ValveId::ALL.contains(v))?;

        let command = match ValveCommandKind::from_u8(self.command)? {
            ValveCommandKind::Close => ValveCommand::Close,
            ValveCommandKind::Open => ValveCommand::Open,
            ValveCommandKind::Partial => {
                ValveCommand::Partial(f32::from(self.position) / Self::PROMILLE_PER_UNIT)
            }
            ValveCommandKind::PulseOpen => {
                ValveCommand::PulseOpen(Duration::from_millis(u64::from(self.duration_ms)))
            }
        };

        Some((valve, command))
    }
}

/// 0x04: ParamSet
///
/// The RF counterpart of PARAM_SET. The vehicle answers with the resulting value.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParamSetMessage(pub ParamEntry);

impl UplinkTelemetryMessage for ParamSetMessage {
    const ID: u8 = 0x04;
}

/// 0x05: ParamRequest
///
/// Asks for the values of the parameters at flat index `first + i` for every set bit `i`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParamRequestMessage {
    #[serde(with = "postcard::fixint::le")]
    pub first: u16,
    #[serde(with = "postcard::fixint::le")]
    pub mask: u32,
}

impl UplinkTelemetryMessage for ParamRequestMessage {
    const ID: u8 = 0x05;
}

// TODO: messages for:
//  - log/storage management
//  -
//  - radio control (control tx power)?

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: [u8; 16] = [0x42; 16];

    fn through_packet(msg: UplinkMessage) -> UplinkMessage {
        let packet = msg.encode(7, &KEY).expect("message did not fit its packet");
        UplinkMessage::decode(packet, &KEY)
            .expect("packet did not decode")
            .1
    }

    /// The payload is exactly six bytes, so a field that lost its `fixint` annotation would stop
    /// the ground station commanding valves at all.
    #[test]
    fn no_payload_length_depends_on_its_values() {
        fn payload_len(msg: &impl Serialize) -> usize {
            let mut buf = [0x00; UPLINK_PAYLOAD_SIZE];
            postcard::to_slice(msg, &mut buf)
                .expect("message did not fit its payload")
                .len()
        }

        let valves = [
            ValveCommand::Close,
            ValveCommand::Open,
            ValveCommand::Partial(0.0),
            ValveCommand::Partial(1.0),
            ValveCommand::PulseOpen(Duration::from_millis(0)),
            ValveCommand::PulseOpen(Duration::from_secs(30)),
        ]
        .map(|cmd| payload_len(&SetValveMessage::new(ValveId::Main, cmd)));
        let params = [(0, 0, 0, 0), (u16::MAX, u32::MAX, u16::MAX, u32::MAX)].map(
            |(id, raw, first, mask)| {
                [
                    payload_len(&ParamSetMessage(ParamEntry { id, raw })),
                    payload_len(&ParamRequestMessage { first, mask }),
                ]
            },
        );
        let lengths: heapless::Vec<usize, 10> = valves
            .into_iter()
            .chain(params.into_iter().flatten())
            .collect();

        assert!(
            lengths.iter().all(|l| *l == UPLINK_PAYLOAD_SIZE),
            "{lengths:?} should all be {UPLINK_PAYLOAD_SIZE}"
        );
    }

    /// The sequence number is 11 bits on the wire, so a receiver comparing two of them has to
    /// reduce the difference modulo [`UPLINK_SEQ_MODULO`] - on a `u16` it would otherwise read the
    /// rollover as tens of thousands of lost packets. Pin that the header layout agrees with the
    /// constant that math depends on.
    #[test]
    fn sequence_numbers_wrap_at_the_modulo() {
        for seq in 0..(UPLINK_SEQ_MODULO.wrapping_mul(3)) {
            let packet = UplinkMessage::Heartbeat(())
                .encode(seq, &KEY)
                .expect("message did not fit its packet");
            let (decoded, _) = UplinkMessage::decode(packet, &KEY).expect("packet did not decode");

            assert_eq!(decoded, seq % UPLINK_SEQ_MODULO, "seq {seq}");
        }
    }

    #[test]
    fn valve_commands_survive_the_packet() {
        for valve in ValveId::ALL {
            for command in [
                ValveCommand::Close,
                ValveCommand::Open,
                ValveCommand::Partial(0.4),
                ValveCommand::PulseOpen(Duration::from_millis(2500)),
            ] {
                let sent = UplinkMessage::SetValve(SetValveMessage::new(valve, command));
                let UplinkMessage::SetValve(decoded) = through_packet(sent) else {
                    panic!("decoded as the wrong message")
                };

                assert_eq!(decoded.command(), Some((valve, command)), "{valve:?}");
            }
        }
    }

    #[test]
    fn param_messages_survive_the_packet() {
        let UplinkMessage::ParamSet(set) =
            through_packet(UplinkMessage::ParamSet(ParamSetMessage(ParamEntry {
                id: 0x0201,
                raw: 0xdead_beef,
            })))
        else {
            panic!("decoded as the wrong message")
        };
        assert_eq!((set.0.id, set.0.raw), (0x0201, 0xdead_beef));

        let UplinkMessage::ParamRequest(request) =
            through_packet(UplinkMessage::ParamRequest(ParamRequestMessage {
                first: 32,
                mask: 0x8000_0001,
            }))
        else {
            panic!("decoded as the wrong message")
        };
        assert_eq!((request.first, request.mask), (32, 0x8000_0001));
    }

    /// A valve id or command kind we do not recognise must not reach the vehicle.
    #[test]
    fn unknown_valves_and_commands_are_rejected() {
        // Extra1 exists in the dialect but is not in this vehicle's inventory.
        let extra = SetValveMessage {
            valve: ValveId::Extra1 as u8,
            command: ValveCommandKind::Open as u8,
            position: 0,
            duration_ms: 0,
        };
        assert_eq!(extra.command(), None);

        let unmapped = SetValveMessage {
            valve: 200,
            command: ValveCommandKind::Open as u8,
            position: 0,
            duration_ms: 0,
        };
        assert_eq!(unmapped.command(), None);

        let bad_kind = SetValveMessage {
            valve: ValveId::Main as u8,
            command: 9,
            position: 0,
            duration_ms: 0,
        };
        assert_eq!(bad_kind.command(), None);
    }
}
