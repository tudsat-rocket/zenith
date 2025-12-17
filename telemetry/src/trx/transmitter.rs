use core::marker::PhantomData;

use embassy_futures::select::{Either, select};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};

use lora_phy::LoRa;
use lora_phy::mod_params::{ModulationParams, PacketParams, RadioError};
use lora_phy::mod_traits::RadioKind;

use utils::anychannel::AnyReceiver;

use crate::config::{FREQUENCIES, LinkConfig};
use crate::messages::{DOWNLINK_PACKET_SIZE, DownlinkMessage, TelemetryMessage, UplinkMessage};
use crate::{DOWNLINK_MESSAGE_INTERVAL_MS, UPLINK_HOP_INTERVAL_MS};

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
}

impl<RK: RadioKind, R: AnyReceiver<(u16, DownlinkMessage)>>
    HoppingTransmitter<RK, DownlinkMessage, R>
{
    pub async fn run_downlink(mut self) {
        const TX_POWER: i32 = 10;

        loop {
            let (time, msg) = self.receiver.anyreceive().await;
            let bytes = msg.encode(time, &self.config.hmac_key).unwrap();

            let frequency = self.config.frequency(time);

            match with_timeout(
                Duration::from_millis((DOWNLINK_MESSAGE_INTERVAL_MS - 1).into()),
                self.transmit_packet(frequency, &bytes, TX_POWER),
            )
            .await
            {
                Ok(Ok(_)) => {}
                Ok(Err(e)) => {
                    defmt::error!("Failed to transmit packet: {:?}", defmt::Debug2Format(&e));
                }
                Err(_) => {
                    defmt::error!("Timed out while transmitting.");
                }
            }
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

            let bytes = message.encode(seq, &self.config.hmac_key).unwrap();

            if let Some((last_instant, last_t)) = connection {
                // If we have a good downlink connection, we use that information to figure out on
                // which frequency we have to transmit right now, and when best to do that to avoid
                // overlaps with downlink packets.

                // Note that this does not account for time-on-air or transmission latency. This is
                // effectively the timestamp a downlink packet would have if it arrived right now.
                // This means our clock is running slightly late, so we'll have to apply some extra
                // tolerance to our end-of-hop checks.
                let current_t = last_t.wrapping_add(last_instant.elapsed().as_millis() as u16);
                let frequency = self.config.frequency(current_t);

                // If we're close to the end of an uplink frequency hopping slot, we delay our
                // transmission so its reception does not get interrupted by the hop.
                let time_in_hopping_interval = current_t as u64 % UPLINK_HOP_INTERVAL_MS as u64;
                if (UPLINK_HOP_INTERVAL_MS as u64) - time_in_hopping_interval < 30 {
                    let remaining = UPLINK_HOP_INTERVAL_MS as u64 - time_in_hopping_interval;
                    Timer::after(Duration::from_millis(remaining + 10)).await;
                } else if time_in_hopping_interval < 10 {
                    Timer::after(Duration::from_millis(10 - time_in_hopping_interval)).await;
                }

                for _i in 0..num_transmissions {
                    // If we're close to another downlink message, we also delay our transmission, so
                    // it is sent and arrives while the airwaves are clear (assuming we're sending with
                    // less than 50% duty cycle).
                    let time_in_downlink_interval =
                        current_t as u64 % DOWNLINK_MESSAGE_INTERVAL_MS as u64;
                    //let target_time_in_downlink_interval =
                    //    5 * (DOWNLINK_MESSAGE_INTERVAL_MS as u64) / 16;
                    //if time_in_downlink_interval > target_time_in_downlink_interval {
                    if time_in_downlink_interval > 2 {
                        let remaining =
                            DOWNLINK_MESSAGE_INTERVAL_MS as u64 - time_in_downlink_interval;
                        Timer::after(Duration::from_millis(remaining + 1)).await;
                    }

                    match with_timeout(
                        Duration::from_millis(100),
                        self.transmit_packet(frequency, &bytes, TX_POWER),
                    )
                    .await
                    {
                        Ok(Ok(_)) => {}
                        Ok(Err(e)) => {
                            defmt::error!(
                                "Failed to transmit packet: {:?}",
                                defmt::Debug2Format(&e)
                            );
                        }
                        Err(_) => {
                            defmt::error!("Timed out while transmitting.");
                        }
                    }
                }
            } else {
                // If we don't have a good connection, we send on all uplink frequencies.
                for frequency in FREQUENCIES
                    .iter()
                    .zip(self.config.frequency_mask)
                    .filter_map(|(f, m)| m.then_some(*f))
                {
                    match with_timeout(
                        Duration::from_millis(100),
                        self.transmit_packet(frequency, &bytes, TX_POWER),
                    )
                    .await
                    {
                        Ok(Ok(_)) => {}
                        Ok(Err(e)) => {
                            defmt::error!(
                                "Failed to transmit packet: {:?}",
                                defmt::Debug2Format(&e)
                            );
                        }
                        Err(_) => {
                            defmt::error!("Timed out while transmitting.");
                        }
                    }
                }
            }
        }
    }
}
