use core::hash::Hasher;

use serde::{Deserialize, Serialize, de::DeserializeOwned};
use siphasher::sip::SipHasher;

use crate::{TelemetryError, UplinkCommand, messages::TelemetryMessage};

pub const UPLINK_PACKET_SIZE: usize = 16;
const UPLINK_PAYLOAD_SIZE: usize = UPLINK_PACKET_SIZE - 10;

#[derive(Debug, Clone)]
pub enum UplinkMessage {
    Heartbeat(()),
    SetFlightMode(SetFlightModeMessage),
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

// TODO: messages for:
//  - parameters
//  - log/storage management
//  -
//  - radio control (control tx power)?
