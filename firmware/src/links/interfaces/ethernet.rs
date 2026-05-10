use embassy_sync::watch::Watch;
use rapid_dialect::Rapid;
use static_cell::StaticCell;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{DhcpConfig, StackResources};
use embassy_stm32::eth::Ethernet;
use embassy_stm32::eth::GenericPhy;
use embassy_stm32::peripherals::ETH;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};

use mavio::Frame;
use mavio::error::FrameError;
use mavio::prelude::V2;

use crate::can::{CanRxSubscriber, CanTxPublisher};
use crate::links::interfaces::{
    InterfaceCommandPublisher, InterfaceCommandSubscriber, InterfaceCommands, InterfaceRx,
    InterfaceRxPublisher, InterfaceRxSubscriber, InterfaceTx, InterfaceTxPublisher,
    InterfaceTxSubscriber,
};
use crate::links::protocols::link_quality::LinkQuality;
use mission::TelemetryLink;

use crate::links::{UplinkCommand, protocols};

#[cfg(not(feature = "gcs"))]
pub const ETHERNET_SYSTEM_ID: u8 = 0x04;
#[cfg(feature = "gcs")]
pub const ETHERNET_SYSTEM_ID: u8 = 0x06;

pub static DOWNLINK: StaticCell<InterfaceTx> = StaticCell::new();
pub static UPLINK: StaticCell<InterfaceRx> = StaticCell::new();
pub static COMMANDS: StaticCell<InterfaceCommands> = StaticCell::new();

static LINK_QUALITY: Watch<CriticalSectionRawMutex, LinkQuality, 3> = Watch::new();

static RX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
static RX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
static TX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
static TX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();

pub struct EthernetHandle {
    tx: InterfaceTxPublisher,
    cmd_rx: InterfaceCommandSubscriber,
}

impl EthernetHandle {
    #[allow(
        clippy::needless_pass_by_value,
        reason = "at the moment we always pass the CAN stuff, even for the GCS where we don't need it."
    )]
    pub fn init(
        device: Ethernet<'static, ETH, GenericPhy>,
        seed: u64,
        can: (CanTxPublisher, CanRxSubscriber),
        spawner: Spawner,
    ) -> Self {
        static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

        let tx = DOWNLINK.init(PubSubChannel::new());
        let rx = UPLINK.init(PubSubChannel::new());
        let commands = COMMANDS.init(PubSubChannel::new());

        // TODO
        let config = embassy_net::Config::dhcpv4(DhcpConfig::default());

        let (stack, runner) =
            embassy_net::new(device, config, RESOURCES.init(StackResources::new()), seed);

        spawner.spawn(run_network(runner)).unwrap();

        let socket = UdpSocket::new(
            stack,
            RX_META.init([PacketMetadata::EMPTY; 8]),
            RX_BUFFER.init([0; 1024]),
            TX_META.init([PacketMetadata::EMPTY; 8]),
            TX_BUFFER.init([0; 1024]),
        );

        spawner
            .spawn(run_socket(
                socket,
                tx.subscriber().unwrap(),
                rx.publisher().unwrap(),
            ))
            .unwrap();

        spawner
            .spawn(run_commands(
                ETHERNET_SYSTEM_ID,
                0x01,
                tx.publisher().unwrap(),
                rx.subscriber().unwrap(),
                commands.publisher().unwrap(),
                LINK_QUALITY.sender(),
            ))
            .unwrap();

        spawner
            .spawn(run_link_quality(
                tx.publisher().unwrap(),
                LINK_QUALITY.receiver().unwrap(),
            ))
            .unwrap();

        #[cfg(not(feature = "gcs"))]
        spawner
            .spawn(protocols::can_probe::run(
                can.0,
                can.1,
                commands.subscriber().unwrap(),
                tx.publisher().unwrap(),
                rx.subscriber().unwrap(),
            ))
            .unwrap();

        spawner
            .spawn(run_modes(
                tx.publisher().unwrap(),
                commands.subscriber().unwrap(),
            ))
            .unwrap();

        Self {
            tx: tx.publisher().unwrap(),
            cmd_rx: commands.subscriber().unwrap(),
        }
    }

    pub fn split(self) -> (InterfaceTxPublisher, InterfaceCommandSubscriber) {
        let Self { tx, cmd_rx } = self;
        (tx, cmd_rx)
    }
}

impl TelemetryLink for EthernetHandle {
    fn send_message(&mut self, message: Rapid) {
        self.tx.publish_immediate(message);
    }

    fn try_recv_command(&mut self) -> Option<UplinkCommand> {
        self.cmd_rx.try_next_message_pure()
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn run_commands(
    system_id: u8,
    component_id: u8,
    tx: InterfaceTxPublisher,
    rx: InterfaceRxSubscriber,
    cmd_tx: InterfaceCommandPublisher,
    link_quality_sender: embassy_sync::watch::Sender<
        'static,
        CriticalSectionRawMutex,
        LinkQuality,
        3,
    >,
) {
    protocols::commands::run(system_id, component_id, tx, rx, cmd_tx, link_quality_sender).await;
}

#[embassy_executor::task(pool_size = 2)]
async fn run_link_quality(
    tx: InterfaceTxPublisher,
    rx: embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, LinkQuality, 3>,
) {
    protocols::link_quality::run(tx, rx).await;
}

#[embassy_executor::task(pool_size = 2)]
async fn run_modes(tx: InterfaceTxPublisher, rx: InterfaceCommandSubscriber) {
    protocols::modes::run(tx, rx).await;
}

#[embassy_executor::task]
async fn run_network(
    mut runner: embassy_net::Runner<'static, Ethernet<'static, ETH, GenericPhy>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn run_socket(
    mut socket: UdpSocket<'static>,
    mut subscriber: InterfaceTxSubscriber,
    publisher: InterfaceRxPublisher,
) -> ! {
    let remote_endpoint = (embassy_net::Ipv4Address::BROADCAST, 14550);
    socket.bind(14551).unwrap();
    socket.set_hop_limit(Some(4));

    let endpoint = mavio::Endpoint::v2(mavio::MavLinkId::new(ETHERNET_SYSTEM_ID, 0x01));
    let mut mavlink_buffer = heapless::Vec::<u8, 1024>::new();

    loop {
        let mut recv_buffer = [0; 1024];
        match select(
            subscriber.next_message_pure(),
            socket.recv_from(&mut recv_buffer),
        )
        .await
        {
            Either::First(message) => {
                let frame = endpoint.next_frame(&message).unwrap();

                let mut transmit_buffer = [0; 1024];
                let n = frame.serialize(&mut transmit_buffer).unwrap();
                let serialized = &transmit_buffer[..n];

                socket.send_to(serialized, remote_endpoint).await.unwrap();
            }
            Either::Second(res) => {
                let Ok((len, _peer)) = res else {
                    continue;
                };

                defmt::debug!("received packet: len: {}", len);
                if let Err(..) = mavlink_buffer.extend_from_slice(&recv_buffer[..len]) {
                    defmt::error!("mavlink buffer overrun");
                    mavlink_buffer.truncate(0);
                    continue;
                }

                let frame_result = unsafe { mavio::Frame::deserialize(&mavlink_buffer) };
                match frame_result {
                    Ok(frame) => {
                        publisher.publish(frame).await;
                        mavlink_buffer.truncate(0);
                    }
                    Err(FrameError::FrameBufferIsTooSmall { .. }) => {}
                    Err(e) => {
                        defmt::error!("mavio error: {}", defmt::Debug2Format(&e));
                        mavlink_buffer.truncate(0);
                    }
                }
            }
        }
    }
}
