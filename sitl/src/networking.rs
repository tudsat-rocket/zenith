use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::StackResources;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net_tuntap::TunTapDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::Watch;

use mavio::error::FrameError;
use rapid_dialect::Rapid;
use static_cell::StaticCell;

use links::protocols::link_quality::LinkQuality;
use links::{
    InterfaceCommandPublisher, InterfaceCommandSubscriber, InterfaceCommands, InterfaceRx,
    InterfaceRxPublisher, InterfaceTx, InterfaceTxPublisher, InterfaceTxSubscriber, UplinkCommand,
};
use mission::TelemetryLink;

use crate::Vehicle;

pub const SYSTEM_ID: u8 = 0x14;
const COMPONENT_ID: u8 = 0x01;

static DOWNLINK: StaticCell<InterfaceTx> = StaticCell::new();
static UPLINK: StaticCell<InterfaceRx> = StaticCell::new();
static COMMANDS: StaticCell<InterfaceCommands> = StaticCell::new();

static LINK_QUALITY: Watch<CriticalSectionRawMutex, LinkQuality, 3> = Watch::new();

static RX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
static RX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
static TX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
static TX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();

pub struct Links {
    tx: InterfaceTxPublisher,
    cmd_rx: InterfaceCommandSubscriber,
}

impl Links {
    pub fn init(spawner: Spawner) -> Self {
        static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

        let tx = DOWNLINK.init(PubSubChannel::new());
        let rx = UPLINK.init(PubSubChannel::new());
        let commands = COMMANDS.init(PubSubChannel::new());

        let device = TunTapDevice::new("tap99").unwrap();

        let seed = rand::random();
        let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
            address: embassy_net::Ipv4Cidr::new(embassy_net::Ipv4Address::new(192, 168, 69, 2), 24),
            gateway: Some(embassy_net::Ipv4Address::new(192, 168, 69, 1)),
            dns_servers: heapless::Vec::new(),
        });
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
                SYSTEM_ID,
                COMPONENT_ID,
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

    pub fn send_telemetry_messages(&mut self, vehicle: &Vehicle) {
        vehicle.send_telemetry(self);
    }
}

impl TelemetryLink for Links {
    fn send_message(&mut self, message: Rapid) {
        self.tx.publish_immediate(message);
    }

    fn try_recv_command(&mut self) -> Option<UplinkCommand> {
        while let Some(cmd) = self.cmd_rx.try_next_message_pure() {
            match cmd {
                UplinkCommand::SetFlightMode(..) => return Some(cmd),
                _ => {}
            }
        }

        None
    }
}

#[embassy_executor::task]
async fn run_network(mut runner: embassy_net::Runner<'static, TunTapDevice>) -> ! {
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

    let endpoint = mavio::Endpoint::v2(mavio::MavLinkId::new(SYSTEM_ID, COMPONENT_ID));
    let mut mavlink_buffer = Vec::<u8>::new();

    loop {
        let mut recv_buffer = [0; 1024];
        match select(
            subscriber.next_message_pure(),
            socket.recv_from(&mut recv_buffer),
        )
        .await
        {
            Either::First(message) => {
                let frame = match message {
                    Rapid::Heartbeat(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::CommandAck(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::AvailableModes(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::CanFrame(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::Attitude(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::LocalPositionNed(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::ScaledImu(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::ScaledImu2(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::ScaledImu3(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::ScaledPressure(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::ScaledPressure2(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::ScaledPressure3(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::BatteryStatus(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::RadioStatus(m) => endpoint.next_frame(&m).unwrap(),
                    Rapid::LinkNodeStatus(m) => endpoint.next_frame(&m).unwrap(),
                    _ => continue,
                };

                let mut transmit_buffer = [0; 1024];
                let n = frame.serialize(&mut transmit_buffer).unwrap();
                let serialized = &transmit_buffer[..n];

                socket.send_to(serialized, remote_endpoint).await.unwrap();
            }
            Either::Second(res) => {
                let Ok((len, _peer)) = res else {
                    continue;
                };

                log::debug!("received packet: len: {len}");
                mavlink_buffer.extend_from_slice(&recv_buffer[..len]);

                let frame_result = unsafe { mavio::Frame::deserialize(&mavlink_buffer) };
                match frame_result {
                    Ok(frame) => {
                        publisher.publish(frame).await;
                        mavlink_buffer.clear();
                    }
                    Err(FrameError::FrameBufferIsTooSmall { .. }) => {}
                    Err(e) => {
                        log::error!("mavio error: {e:?}");
                        mavlink_buffer.clear();
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn run_commands(
    system_id: u8,
    component_id: u8,
    tx: InterfaceTxPublisher,
    rx: links::InterfaceRxSubscriber,
    cmd_tx: InterfaceCommandPublisher,
    link_quality_sender: embassy_sync::watch::Sender<
        'static,
        CriticalSectionRawMutex,
        LinkQuality,
        3,
    >,
) {
    links::protocols::commands::run(system_id, component_id, tx, rx, cmd_tx, link_quality_sender)
        .await;
}

#[embassy_executor::task]
async fn run_link_quality(
    tx: InterfaceTxPublisher,
    rx: embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, LinkQuality, 3>,
) {
    links::protocols::link_quality::run(tx, rx).await;
}

#[embassy_executor::task]
async fn run_modes(tx: InterfaceTxPublisher, rx: InterfaceCommandSubscriber) {
    links::protocols::modes::run(tx, rx).await;
}
