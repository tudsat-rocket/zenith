pub mod downlink;
pub mod uplink;

pub use downlink::*;
pub use uplink::*;
use utils::anychannel::AnySender;

use crate::TelemetryError;

pub trait TelemetryMessage: Sized {
    type Packet: Default;

    type Input;
    type Output;

    // TODO: packet sizes

    fn encode(
        self,
        seq_or_time: u16,
        hmac_key: &[u8; 16],
    ) -> Result<[u8; DOWNLINK_PACKET_SIZE], TelemetryError>;

    fn decode(
        buffer: [u8; DOWNLINK_PACKET_SIZE],
        hmac_key: &[u8; 16],
    ) -> Result<(u16, Self), TelemetryError>;

    async fn unpack<S: AnySender<Self::Output>>(
        self,
        sender: &mut S,
        context: &mut ConnectionContext,
    );
}
