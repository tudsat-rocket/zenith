use core::marker::PhantomData;

use embassy_futures::select::{Either, select};
use embassy_time::{Delay, Duration, Instant, Timer};

use lora_phy::LoRa;
use lora_phy::mod_params::{ModulationParams, PacketParams, RadioError};
use lora_phy::mod_traits::RadioKind;

use utils::anychannel::AnyReceiver;

use crate::config::{FREQUENCIES, LinkConfig};
use crate::messages::{DOWNLINK_PACKET_SIZE, DownlinkMessage, TelemetryMessage, UplinkMessage};
use crate::trx::MAX_CONSECUTIVE_ERRORS;

pub struct HoppingTransmitter<RK: RadioKind, M: TelemetryMessage, R: AnyReceiver<(u16, M)>> {
    radio: LoRa<RK, Delay>,
    config: LinkConfig<'static>,
    receiver: R,
    _msg: PhantomData<M>,
}

impl<RK: RadioKind, M: TelemetryMessage, R: AnyReceiver<(u16, M)>> HoppingTransmitter<RK, M, R> {
    pub fn new(radio: LoRa<RK, Delay>, config: LinkConfig<'static>, receiver: R) -> Self {
        Self {
            radio,
            config,
            receiver,
            _msg: PhantomData,
        }
    }

    fn create_parameters(
        &mut self,
        frequency: u32,
    ) -> Result<(ModulationParams, PacketParams), RadioError> {
        let mod_params = self.radio.create_modulation_params(
            self.config.spreading_factor,
            self.config.bandwidth,
            self.config.coding_rate,
            frequency,
        )?;

        let pkt_params = self.radio.create_tx_packet_params(
            self.config.preamble_length,
            true,
            true,
            false,
            &mod_params,
        )?;

        Ok((mod_params, pkt_params))
    }

    async fn transmit_packet(
        &mut self,
        frequency: u32,
        data: &[u8; DOWNLINK_PACKET_SIZE],
        transmit_power: i32,
    ) -> Result<(), RadioError> {
        let (mod_params, mut pkt_params) = self.create_parameters(frequency)?;

        self.radio
            .prepare_for_tx(&mod_params, &mut pkt_params, transmit_power, data)
            .await?;
        self.radio.tx().await?;

        Ok(())
    }

    /// Pull the transceiver's reset line and reconfigure it from scratch.
    async fn reset_radio(&mut self) {
        defmt::warn!("Resetting transmitter radio.");
        if let Err(e) = self.radio.init().await {
            defmt::error!("Failed to reset radio: {:?}", defmt::Debug2Format(&e));
        }
    }

    /// Transmit a packet, hard-resetting the transceiver once transmissions have failed often
    /// enough in a row.
    async fn transmit_or_reset(
        &mut self,
        frequency: u32,
        data: &[u8; DOWNLINK_PACKET_SIZE],
        transmit_power: i32,
        consecutive_errors: &mut u32,
    ) {
        match self.transmit_packet(frequency, data, transmit_power).await {
            Ok(()) => *consecutive_errors = 0,
            Err(e) => {
                defmt::error!("Failed to transmit packet: {:?}", defmt::Debug2Format(&e));

                *consecutive_errors = u32::saturating_add(*consecutive_errors, 1);
                if *consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                    self.reset_radio().await;
                    *consecutive_errors = 0;
                }
            }
        }
    }
}

impl<RK: RadioKind, R: AnyReceiver<(u16, DownlinkMessage)>>
    HoppingTransmitter<RK, DownlinkMessage, R>
{
    pub async fn run_downlink(mut self) {
        const TX_POWER: i32 = 10;

        let mut consecutive_errors = 0;

        loop {
            let (time, msg) = self.receiver.anyreceive().await;
            #[allow(
                clippy::unwrap_used,
                reason = "message encodes into a buffer sized for the fixed packet layout, infallible"
            )]
            let bytes = msg.encode(time, &self.config.hmac_key).unwrap();

            let frequency = self.config.frequency(time);

            self.transmit_or_reset(frequency, &bytes, TX_POWER, &mut consecutive_errors)
                .await;
        }
    }
}

impl<RK: RadioKind, R: AnyReceiver<(u16, UplinkMessage)>> HoppingTransmitter<RK, UplinkMessage, R> {
    pub async fn run_uplink<CONN: AnyReceiver<Option<(Instant, u16)>>>(
        mut self,
        mut connection_receiver: CONN,
    ) {
        const TX_POWER: i32 = 22;

        let mut connection = None;
        let mut consecutive_errors = 0;

        loop {
            let (seq, message) =
                match select(self.receiver.anyreceive(), connection_receiver.anyreceive()).await {
                    Either::First(x) => x,
                    Either::Second(conn) => {
                        connection = conn;
                        continue;
                    }
                };

            let num_transmissions = if let UplinkMessage::Heartbeat(()) = message {
                1
            } else {
                3
            };

            #[allow(
                clippy::unwrap_used,
                reason = "message encodes into a buffer sized for the fixed packet layout, infallible"
            )]
            let bytes = message.encode(seq, &self.config.hmac_key).unwrap();

            if let Some((last_instant, last_t)) = connection {
                // If we have a good downlink connection, we use that information to figure out on
                // which frequency we have to transmit right now, and when best to do that to avoid
                // overlaps with downlink packets.
                for _i in 0..num_transmissions {
                    // Both the clock estimate and the frequency are taken per transmission, since
                    // the receiver could hop while we're retransmitting.
                    let t = last_t.wrapping_add(last_instant.elapsed().as_millis() as u16);
                    let target = self.config.next_transmission_time(t);
                    Timer::after(Duration::from_millis(target.wrapping_sub(t).into())).await;

                    let frequency = self.config.frequency(target);
                    self.transmit_or_reset(frequency, &bytes, TX_POWER, &mut consecutive_errors)
                        .await;
                }
            } else {
                // If we don't have a good connection, we send on all uplink frequencies.
                for frequency in FREQUENCIES
                    .iter()
                    .zip(self.config.frequency_mask)
                    .filter_map(|(f, m)| m.then_some(*f))
                {
                    self.transmit_or_reset(frequency, &bytes, TX_POWER, &mut consecutive_errors)
                        .await;
                }
            }
        }
    }
}
