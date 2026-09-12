#![allow(
    clippy::unwrap_used,
    reason = "boot-time radio setup; panic-on-failure is the embedded model"
)]
#![allow(
    clippy::arithmetic_side_effects,
    reason = "bounded radio timing/counter math"
)]
#![allow(clippy::unreachable, reason = "exhaustive radio-state match")]

use embassy_sync::watch::Watch;
use static_cell::StaticCell;

use embassy_executor::{SendSpawner, Spawner};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Delay, Duration, Instant, with_timeout};

use lora_phy::LoRa;
use lora_phy::mod_params::{Bandwidth, CodingRate, PacketStatus, RadioError, SpreadingFactor};

use rapid_dialect::Rapid;
use rapid_dialect::rapid::messages::RadioStatus;

use telemetry::config::{DEFAULT_DOWNLINK_CONFIG, DEFAULT_UPLINK_CONFIG, FREQUENCIES, LinkConfig};
use telemetry::messages::TelemetryMessage;
use telemetry::messages::UplinkMessage;
use telemetry::messages::{DOWNLINK_PACKET_SIZE, DownlinkMessage};
use telemetry::trx::receiver::HoppingReceiver;
use telemetry::trx::transmitter::HoppingTransmitter;

use crate::LoraTransceiver;
use crate::Vehicle;
use crate::can::{CanRxSubscriber, CanTxPublisher};
use crate::links::UplinkCommand;
use crate::links::interfaces::{
    InterfaceCommandSubscriber, InterfaceCommands, InterfaceRx, InterfaceRxPublisher,
    InterfaceRxSubscriber, InterfaceTx, InterfaceTxPublisher, InterfaceTxSubscriber,
};

/// Intentionally capacity of 1 for downlink to avoid stale messages backing up.
pub static DOWNLINK: StaticCell<Channel<CriticalSectionRawMutex, (u16, DownlinkMessage), 1>> =
    StaticCell::new();
pub static UPLINK: StaticCell<Channel<CriticalSectionRawMutex, UplinkCommand, 5>> =
    StaticCell::new();

static UPLINK_STATS: Watch<CriticalSectionRawMutex, (i8, i8, f32), 3> = Watch::new();
static TIME: Watch<CriticalSectionRawMutex, (Instant, u16), 3> = Watch::new();

pub struct LoraHandle {
    tx: Sender<'static, CriticalSectionRawMutex, (u16, DownlinkMessage), 1>,
    rx: Receiver<'static, CriticalSectionRawMutex, UplinkCommand, 5>,
    time_sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, (Instant, u16), 3>,
}

impl LoraHandle {
    pub fn init(
        lora1: LoRa<LoraTransceiver, Delay>,
        lora2: LoRa<LoraTransceiver, Delay>,
        spawner: SendSpawner,
    ) -> Self {
        let tx = DOWNLINK.init(Channel::new());
        let rx = UPLINK.init(Channel::new());

        let downlink = HoppingTransmitter::new(lora1, DEFAULT_DOWNLINK_CONFIG, tx.receiver());
        spawner.spawn(run_downlink(downlink)).unwrap();

        let uplink = HoppingReceiver::new(lora2, DEFAULT_UPLINK_CONFIG, rx.sender());
        spawner
            .spawn(run_uplink(
                uplink,
                UPLINK_STATS.sender(),
                TIME.receiver().unwrap(),
            ))
            .unwrap();

        Self {
            tx: tx.sender(),
            rx: rx.receiver(),
            time_sender: TIME.sender(),
        }
    }
}

impl LoraHandle {
    pub fn try_recv_command(&mut self) -> Option<UplinkCommand> {
        self.rx.try_receive().ok()
    }

    pub fn send_telemetry_messages(&mut self, vehicle: &Vehicle) {
        let t = vehicle.time.0;

        self.time_sender.send((Instant::now(), t as u16));

        let (rssi, snr, packet_loss) = UPLINK_STATS.try_get().unwrap_or_default();
        let uplink = RadioStatus {
            remrssi: rssi as u8,
            remnoise: rssi.saturating_sub(snr) as u8,
            fixed: (packet_loss * 100.0) as u16,
            ..Default::default()
        };

        let Some(msg) = DownlinkMessage::for_tick(t, &vehicle.snapshot(), uplink) else {
            return;
        };

        let _ = self.tx.try_send((t as u16, msg));
    }
}

#[embassy_executor::task]
async fn run_downlink(
    transmitter: HoppingTransmitter<
        LoraTransceiver,
        DownlinkMessage,
        Receiver<'static, CriticalSectionRawMutex, (u16, DownlinkMessage), 1>,
    >,
) {
    transmitter.run_downlink().await;
}

#[embassy_executor::task]
async fn run_uplink(
    receiver: HoppingReceiver<
        LoraTransceiver,
        UplinkMessage,
        Sender<'static, CriticalSectionRawMutex, UplinkCommand, 5>,
    >,
    stat_sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, (i8, i8, f32), 3>,
    time_receiver: embassy_sync::watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        (Instant, u16),
        3,
    >,
) {
    receiver.run_uplink(stat_sender, time_receiver).await;
}
