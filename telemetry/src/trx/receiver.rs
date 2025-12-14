//! Receiver implementation for our frequency-hopping telemetry scheme, both for up and downlink.

use core::marker::PhantomData;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{
    Delay, Duration, Instant, Ticker, TimeoutError, Timer, with_deadline, with_timeout,
};

use lora_phy::mod_params::{ModulationParams, PacketParams, PacketStatus, RadioError};
use lora_phy::mod_traits::{IrqState, RadioKind};
use lora_phy::{LoRa, RxMode};

use rapid_dialect::Rapid;
use rapid_dialect::rapid::messages::RadioStatus;

use utils::anychannel::AnySender;

use crate::config::{FREQUENCIES, LinkConfig, SEQUENCE_LENGTH};
use crate::messages::{
    ConnectionContext, DOWNLINK_PACKET_SIZE, DownlinkMessage, TelemetryMessage, UplinkMessage,
};
use crate::{DOWNLINK_MESSAGE_INTERVAL_MS, UplinkCommand};

#[derive(Debug, thiserror::Error)]
pub enum ReceiveError {
    #[error("Radio error: {0:?}")]
    Radio(RadioError),
    #[error("Timeout Error")]
    Timeout(TimeoutError),
}

impl From<RadioError> for ReceiveError {
    fn from(value: RadioError) -> Self {
        Self::Radio(value)
    }
}

impl From<TimeoutError> for ReceiveError {
    fn from(value: TimeoutError) -> Self {
        Self::Timeout(value)
    }
}

pub struct HoppingReceiver<RK: RadioKind, M: TelemetryMessage, S: AnySender<M::Output>> {
    radio: LoRa<RK, Delay>,
    config: LinkConfig<'static>,
    sender: S,
    _msg: PhantomData<M>,
}

impl<RK: RadioKind, M: TelemetryMessage, S: AnySender<M::Output>> HoppingReceiver<RK, M, S> {
    pub fn new(radio: LoRa<RK, Delay>, config: LinkConfig<'static>, sender: S) -> Self {
        Self {
            radio,
            config,
            sender,
            _msg: PhantomData,
        }
    }

    fn create_parameters(
        &mut self,
        frequency: u32,
        packet_size: u8,
    ) -> Result<(ModulationParams, PacketParams), RadioError> {
        let mod_params = self.radio.create_modulation_params(
            self.config.spreading_factor,
            self.config.bandwidth,
            self.config.coding_rate,
            frequency,
        )?;

        let pkt_params = self.radio.create_rx_packet_params(
            self.config.preamble_length,
            true,
            packet_size,
            true,
            false,
            &mod_params,
        )?;

        Ok((mod_params, pkt_params))
    }

    async fn receive_until(
        &mut self,
        frequency: u32,
        deadline: Instant,
    ) -> Result<Option<(u16, M, PacketStatus)>, ReceiveError> {
        const N: u8 = DOWNLINK_PACKET_SIZE as u8; // TODO

        let (mod_params, pkt_params) = self.create_parameters(frequency, N).unwrap();

        loop {
            if Instant::now() > deadline {
                defmt::warn!("timed out packet reception");
                return Ok(None);
            }

            with_timeout(
                Duration::from_millis(10),
                self.radio
                    .prepare_for_rx(RxMode::Continuous, &mod_params, &pkt_params),
            )
            .await??;

            with_timeout(Duration::from_millis(10), self.radio.start_rx()).await??;

            let mut buffer = [00u8; N as usize]; // TODO

            loop {
                // This is the only part of our receive procedure that should actually timeout
                // during normal operation, and wait_for_irq is cancel-safe.
                match with_deadline(deadline, self.radio.wait_for_irq()).await {
                    Ok(Ok(())) => {}
                    Ok(Err(e)) => {
                        return Err(e.into());
                    }
                    Err(_) => {
                        return Ok(None);
                    }
                }

                let irq_state =
                    with_timeout(Duration::from_millis(10), self.radio.get_irq_state()).await??;
                if irq_state.is_some() {
                    with_timeout(Duration::from_millis(10), self.radio.clear_irq_status())
                        .await??;
                }

                match irq_state {
                    Some(IrqState::Done) => {
                        break;
                    }
                    Some(IrqState::PreambleReceived) | None => {}
                }
            }

            let (_len, status) = match with_timeout(
                Duration::from_millis(10),
                self.radio.get_rx_result(&pkt_params, &mut buffer),
            )
            .await
            {
                Ok(Ok((len, status))) => (len, status),
                Ok(Err(e)) => {
                    defmt::error!("RX read error: {}", defmt::Debug2Format(&e));

                    with_timeout(Duration::from_millis(1000), async {
                        let _ = self.radio.enter_standby().await;
                        let _ = self.radio.sleep(false).await;
                        let _ = self.radio.init().await;
                        let _ = self.radio.clear_irq_status().await;
                    })
                    .await?;

                    return Err(e.into());
                }
                Err(_e) => {
                    defmt::error!("get rx result timeout.");
                    return Ok(None);
                }
            };

            match M::decode(buffer, &self.config.hmac_key) {
                Ok((time_or_seq, msg)) => {
                    return Ok(Some((time_or_seq, msg, status)));
                }
                Err(e) => {
                    defmt::warn!("Failed to decode packet: {}", defmt::Display2Format(&e));
                }
            }
        }
    }

    async fn receive_slot_until(
        &mut self,
        time: u16,
        deadline: Instant,
    ) -> Result<Option<(u16, M, PacketStatus)>, ReceiveError> {
        let frequency = self.config.frequency(time);
        self.receive_until(frequency, deadline).await
    }
}

impl<RK: RadioKind, S: AnySender<Rapid>> HoppingReceiver<RK, DownlinkMessage, S> {
    const CONNECTION_LOST_TIMEOUT_MS: u64 = 2000;
    const SWEEP_DURATION_PER_FREQUENCY_MS: u64 =
        (SEQUENCE_LENGTH as u64) * (DOWNLINK_MESSAGE_INTERVAL_MS as u64);

    pub async fn run_downlink<CONN: AnySender<Option<(Instant, u16)>>>(
        mut self,
        mut connection_sender: CONN,
    ) -> ! {
        loop {
            defmt::info!("Sweeping downlink frequencies");

            for f in FREQUENCIES.iter().cycle() {
                defmt::info!("Listening on {}.", f);

                let timeout = Duration::from_millis(Self::SWEEP_DURATION_PER_FREQUENCY_MS);
                let deadline = Instant::now() + timeout;

                match self.receive_until(*f, deadline).await {
                    Ok(Some((time, msg, _status))) => {
                        connection_sender
                            .anysend(Some((Instant::now(), time)))
                            .await;

                        defmt::info!("Received first packet, initializing connection.");
                        self.handle_connection(time, msg, &mut connection_sender)
                            .await;

                        defmt::warn!("Connection lost.");
                        connection_sender.anysend(None).await;
                    }
                    Ok(None) => {}
                    Err(e) => {
                        defmt::error!("Failed to receive packet: {:?}", defmt::Debug2Format(&e));
                        Timer::after(Duration::from_millis(10)).await;
                    }
                }

                // Send a "100% packet loss" message after every RX period while we're sweeping.
                self.sender
                    .anysend(Rapid::RadioStatus(RadioStatus {
                        rssi: u8::MAX,
                        remrssi: u8::MAX,
                        txbuf: 0,
                        noise: u8::MAX,
                        remnoise: u8::MAX,
                        rxerrors: 100,
                        fixed: 100,
                    }))
                    .await;
            }
        }
    }

    /// Connection handler for a downlink connection.
    ///
    /// After we receive our first valid packet during our sweeps, this is responsible for keeping that
    /// connnection alive, following the hopping sequence of our vehicle. Once we stop receiving
    /// packets from the vehicle for a certain timeout, we return to our frequency sweep.
    async fn handle_connection<CONN: AnySender<Option<(Instant, u16)>>>(
        &mut self,
        mut time: u16,
        initial_msg: DownlinkMessage,
        connection_sender: &mut CONN,
    ) {
        let mut last_packet = Instant::now();

        let mut context = ConnectionContext::init(time);
        initial_msg.unpack(&mut self.sender, &mut context).await;

        // If we ever momentarily lose connection for a few packets, we need to be able to keep up with
        // the hopping sequence, so we use this ticker. The ticker is reset for every received message
        // to avoid this ticker drifting apart from the one driving transmissions on the vehicle side.
        let timeout = Duration::from_millis(DOWNLINK_MESSAGE_INTERVAL_MS as u64);
        let mut ticker = Ticker::every(timeout);

        let mut packet_history: heapless::Deque<(Instant, u16), 128> = heapless::Deque::new();

        loop {
            // Haven't heard anything in a while, return to our sweep.
            if last_packet.elapsed() > Duration::from_millis(Self::CONNECTION_LOST_TIMEOUT_MS) {
                return;
            }

            let t = Instant::now();
            let deadline = t + timeout;
            let next_time = time.wrapping_add(DOWNLINK_MESSAGE_INTERVAL_MS as u16);

            match self.receive_slot_until(next_time, deadline).await {
                Ok(Some((t, msg, status))) => {
                    // Since we just received a packet, if we reset all our timers right away, and
                    // wait for a full interval exactly on the next round, we might occasionally
                    // miss packets because of minor jitter causing the next packet to arrive only
                    // slightly after a full interval. For this reason, we shift everything by a
                    // small time to give us some wiggle room.
                    Timer::after(Duration::from_millis(1)).await;

                    last_packet = Instant::now();
                    ticker.reset();
                    time = t;

                    msg.unpack(&mut self.sender, &mut context).await;

                    connection_sender.anysend(Some((last_packet, t))).await;

                    while packet_history
                        .front()
                        .map(|(t, _)| t.elapsed() > Duration::from_millis(1000))
                        .unwrap_or(false)
                    {
                        let _ = packet_history.pop_front();
                    }

                    let _ = packet_history.push_back((last_packet, t / 16)); // TODO

                    let packet_loss = 1.0
                        - (packet_history.len() as f32)
                            / (1000.0 / DOWNLINK_MESSAGE_INTERVAL_MS as f32);
                    context.rx_rssi = Some((status.rssi as i8) as u8);
                    context.rx_noise = Some(((status.rssi - status.snr) as i8) as u8);
                    context.rx_packet_loss = Some((100.0 * packet_loss) as u16);
                }
                Ok(None) => {
                    defmt::warn!("Missed packet.");
                    time = next_time;
                }
                Err(e) => {
                    defmt::warn!("Failed receiving packet: {:?}.", defmt::Debug2Format(&e));
                    ticker.next().await;
                    time = next_time;
                }
            }
        }
    }
}

impl<RK: RadioKind, S: AnySender<UplinkCommand>> HoppingReceiver<RK, UplinkMessage, S> {
    pub async fn run_uplink<STATS: AnySender<(i8, i8, f32)>>(
        mut self,
        mut stat_sender: STATS,
        mut time_receiver: embassy_sync::watch::Receiver<
            'static,
            CriticalSectionRawMutex,
            (Instant, u16),
            3,
        >,
    ) -> ! {
        let mut last_seq = u16::MAX;
        let mut packet_history: heapless::Deque<(Instant, u16), 32> = heapless::Deque::new();

        loop {
            let Some((last_time_instant, last_time_counter)) = time_receiver.try_get() else {
                Timer::after(Duration::from_millis(10)).await;
                continue;
            };

            let frequency = self.config.frequency(last_time_counter);

            let interval = self.config.hopping_interval as u16;
            let next_hop =
                last_time_counter.wrapping_add(interval - (last_time_counter % interval));
            let deadline = last_time_instant
                + Duration::from_millis(next_hop.wrapping_sub(last_time_counter) as u64);

            let (seq, msg, status) = match self.receive_until(frequency, deadline).await {
                Ok(Some(x)) => x,
                Ok(None) => {
                    continue;
                }
                Err(e) => {
                    defmt::error!("Error receiving uplink: {}", defmt::Debug2Format(&e));
                    continue;
                }
            };

            if seq == last_seq {
                defmt::warn!("Discarding duplicate message.");
                continue;
            }

            while packet_history
                .front()
                .map(|(t, _)| t.elapsed() > Duration::from_millis(10_000))
                .unwrap_or(false)
            {
                let _ = packet_history.pop_front();
            }

            let _ = packet_history.push_back((Instant::now(), seq));

            let lost_packets = packet_history
                .iter()
                .fold((0u32, None), |(mut total, mut last_seq), (_t, seq)| {
                    if let Some(last) = last_seq
                        && last != *seq
                    {
                        total += (seq.wrapping_sub(last) - 1) as u32;
                    }
                    last_seq = Some(*seq);
                    (total, last_seq)
                })
                .0 as f32;

            let packet_loss = lost_packets / (lost_packets + packet_history.iter().count() as f32);

            stat_sender
                .anysend(((status.rssi as i8), (status.snr as i8), packet_loss))
                .await;

            last_seq = seq;

            let cmd = match msg {
                UplinkMessage::Heartbeat(()) => {
                    continue;
                }
                UplinkMessage::SetFlightMode(inner) => {
                    let mode = inner.mode.try_into().unwrap(); // TODO
                    UplinkCommand::SetFlightMode(mode)
                }
            };

            self.sender.anysend(cmd).await;
        }
    }
}
