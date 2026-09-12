use core::hash::Hasher;
use core::time::Duration;

use serde::{Deserialize, Serialize, de::DeserializeOwned};
use siphasher::sip::SipHasher;

use mission::inventory::{InventoryId, ValveId};
use rapid_dialect::ValveCommand;

use crate::{TelemetryError, UplinkCommand, messages::TelemetryMessage};

pub const UPLINK_PACKET_SIZE: usize = 16;
const UPLINK_PAYLOAD_SIZE: usize = UPLINK_PACKET_SIZE - 10;

#[derive(Debug, Clone)]
pub enum UplinkMessage {
    Heartbeat(()),
    SetFlightMode(SetFlightModeMessage),
    SetValve(SetValveMessage),
}

impl TelemetryMessage for UplinkMessage {
    type Packet = [u8; UPLINK_PACKET_SIZE];

    type Input = UplinkCommand;
    type Output = UplinkCommand;

    fn encode(
        self,
        seq: u16,
        hmac_key: &[u8; 16],
    ) -> Result<[u8; UPLINK_PACKET_SIZE], TelemetryError> {
        let (id, payload) = match self {
            Self::Heartbeat(()) => (0x01, [0x00; UPLINK_PAYLOAD_SIZE]),
            Self::SetFlightMode(inner) => (SetFlightModeMessage::ID, inner.serialize()?),
            Self::SetValve(inner) => (SetValveMessage::ID, inner.serialize()?),
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

// TODO: messages for:
//  - parameters
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
        let lengths: heapless::Vec<usize, 8> = [
            ValveCommand::Close,
            ValveCommand::Open,
            ValveCommand::Partial(0.0),
            ValveCommand::Partial(1.0),
            ValveCommand::PulseOpen(Duration::from_millis(0)),
            ValveCommand::PulseOpen(Duration::from_secs(30)),
        ]
        .into_iter()
        .map(|cmd| {
            let mut buf = [0x00; UPLINK_PAYLOAD_SIZE];
            postcard::to_slice(&SetValveMessage::new(ValveId::Main, cmd), &mut buf)
                .expect("message did not fit its payload")
                .len()
        })
        .collect();

        assert!(
            lengths.iter().all(|l| *l == UPLINK_PAYLOAD_SIZE),
            "{lengths:?} should all be {UPLINK_PAYLOAD_SIZE}"
        );
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
