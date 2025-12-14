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
use rapid_dialect::rapid::messages::{
    Attitude, Heartbeat, LocalPositionNed, RadioStatus, SysStatus, SystemTime,
};

use telemetry::config::{DEFAULT_DOWNLINK_CONFIG, DEFAULT_UPLINK_CONFIG, FREQUENCIES, LinkConfig};
use telemetry::messages::TelemetryMessage;
use telemetry::messages::UplinkMessage;
use telemetry::messages::{
    DOWNLINK_PACKET_SIZE, DownlinkMessage, DownlinkTelemetryMessage, HeartbeatMessage,
    StatusMessage,
};
use telemetry::trx::receiver::HoppingReceiver;
use telemetry::trx::transmitter::HoppingTransmitter;

use crate::LoraTransceiver;
use crate::can::{CanRxSubscriber, CanTxPublisher};
use crate::links::interfaces::{
    InterfaceCommandSubscriber, InterfaceCommands, InterfaceRx, InterfaceRxPublisher,
    InterfaceRxSubscriber, InterfaceTx, InterfaceTxPublisher, InterfaceTxSubscriber,
};
use crate::links::{TelemetryLink, UplinkCommand, protocols};
use crate::vehicle::Vehicle;

pub static DOWNLINK: StaticCell<Channel<CriticalSectionRawMutex, (u16, DownlinkMessage), 5>> =
    StaticCell::new();
pub static UPLINK: StaticCell<Channel<CriticalSectionRawMutex, UplinkCommand, 5>> =
    StaticCell::new();

static UPLINK_STATS: Watch<CriticalSectionRawMutex, (i8, i8, f32), 3> = Watch::new();
static TIME: Watch<CriticalSectionRawMutex, (Instant, u16), 3> = Watch::new();

pub struct LoraHandle {
    tx: Sender<'static, CriticalSectionRawMutex, (u16, DownlinkMessage), 5>,
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

impl TelemetryLink for LoraHandle {
    fn send_message(&mut self, _message: Rapid) {
        // TODO: this is not called since we overwrite send_telemetry_messages
        //
        // we should refactor this
    }

    fn try_recv_command(&mut self) -> Option<UplinkCommand> {
        self.rx.try_receive().ok()
    }

    fn send_telemetry_messages(&mut self, vehicle: &Vehicle) {
        const MESSAGE_PATTERN_LENGTH: u32 = 8;

        // While we can freely choose the messages we wish to send, we have to respect the message
        // interval defined in our telemetry protocol since the timing and interval of messages has
        // to be known by the receiver in advance.
        //
        // We also send a message each interval, without leaving gaps. This allows the receiver to
        // infer packet loss and allows more chances to pick up the signal while connecting.
        let t = vehicle.time.0;

        self.time_sender.send((Instant::now(), t as u16));

        if t % telemetry::DOWNLINK_MESSAGE_INTERVAL_MS != 0 {
            return;
        }

        let i = t / telemetry::DOWNLINK_MESSAGE_INTERVAL_MS;

        // Right now the pattern of messages we send is simply a repeating sequence of this length.
        // In theory, we could choose the message to send dynamically based on vehicle state etc.
        let msg = match i % MESSAGE_PATTERN_LENGTH {
            0 | 2 | 4 | 6 => {
                let heartbeat: Heartbeat = vehicle.into();
                let local_position: LocalPositionNed = vehicle.into();
                let attitude: Attitude = vehicle.into();

                let inner = HeartbeatMessage::pack((heartbeat, local_position, attitude));
                DownlinkMessage::Heartbeat(inner)
            }
            1 | 3 | 5 | 7 => {
                let (rssi, snr, packet_loss) = UPLINK_STATS.try_get().unwrap_or_default();
                let radio_status = RadioStatus {
                    remrssi: rssi as u8,
                    remnoise: (rssi - snr) as u8,
                    fixed: (packet_loss * 100.0) as u16,
                    ..Default::default()
                };

                let sys_status = SysStatus::default(); // TODO
                let system_time = SystemTime::default(); // TODO

                let inner = StatusMessage::pack((sys_status, radio_status, system_time));
                DownlinkMessage::Status(inner)
            }
            MESSAGE_PATTERN_LENGTH..=u32::MAX => unreachable!(),
        };

        if let Err(e) = self.tx.try_send((t as u16, msg)) {
            defmt::error!("Failed to send downlink msg.");
        }
    }
}

#[embassy_executor::task]
async fn run_downlink(
    transmitter: HoppingTransmitter<
        LoraTransceiver,
        DownlinkMessage,
        Receiver<'static, CriticalSectionRawMutex, (u16, DownlinkMessage), 5>,
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
