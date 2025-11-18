use defmt::Debug2Format;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use mavio::error::FrameError;
use static_cell::StaticCell;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::StackResources;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_stm32::eth::Ethernet;
use embassy_stm32::eth::GenericPhy;
use embassy_stm32::peripherals::ETH;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use mavio::Frame;
use mavio::prelude::V2;

type Dialect = mavio::dialects::Common;

pub const ETHERNET_DOWNLINK_QUEUE_SIZE: usize = 10;
pub const ETHERNET_UPLINK_QUEUE_SIZE: usize = 5;
pub const NUM_ETHERNET_PUBLISHERS: usize = 2;
pub const NUM_ETHERNET_SUBSCRIBERS: usize = 2;

pub type EthTxPublisher = Publisher<
    'static,
    CriticalSectionRawMutex,
    Dialect,
    ETHERNET_DOWNLINK_QUEUE_SIZE,
    1,
    NUM_ETHERNET_PUBLISHERS,
>;
pub type EthTxSubscriber = Subscriber<
    'static,
    CriticalSectionRawMutex,
    Dialect,
    ETHERNET_DOWNLINK_QUEUE_SIZE,
    1,
    NUM_ETHERNET_PUBLISHERS,
>;
pub type EthRxPublisher = Publisher<
    'static,
    CriticalSectionRawMutex,
    Frame<V2>,
    ETHERNET_UPLINK_QUEUE_SIZE,
    NUM_ETHERNET_SUBSCRIBERS,
    1,
>;
pub type EthRxSubscriber = Subscriber<
    'static,
    CriticalSectionRawMutex,
    Frame<V2>,
    ETHERNET_UPLINK_QUEUE_SIZE,
    NUM_ETHERNET_SUBSCRIBERS,
    1,
>;

pub static DOWNLINK: StaticCell<
    PubSubChannel<
        CriticalSectionRawMutex,
        Dialect,
        ETHERNET_DOWNLINK_QUEUE_SIZE,
        1,
        NUM_ETHERNET_PUBLISHERS,
    >,
> = StaticCell::new();
pub static UPLINK: StaticCell<
    PubSubChannel<
        CriticalSectionRawMutex,
        Frame<V2>,
        ETHERNET_UPLINK_QUEUE_SIZE,
        NUM_ETHERNET_SUBSCRIBERS,
        1,
    >,
> = StaticCell::new();

static RX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
static RX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
static TX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
static TX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();

pub fn start(
    device: Ethernet<'static, ETH, GenericPhy>,
    spawner: Spawner,
    seed: u64,
    downlink_subscriber: EthTxSubscriber,
    uplink_publisher: EthRxPublisher,
) {
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

    // TODO
    let config = embassy_net::Config::dhcpv4(Default::default());

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
        .spawn(run_socket(socket, downlink_subscriber, uplink_publisher))
        .unwrap();
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
    mut downlink_subscriber: EthTxSubscriber,
    uplink_publisher: EthRxPublisher,
) -> ! {
    let remote_endpoint = (embassy_net::Ipv4Address::new(255, 255, 255, 255), 14550);
    socket.bind(14550).unwrap();
    socket.set_hop_limit(Some(4));

    let endpoint = mavio::Endpoint::v2(mavio::MavLinkId::new(0x01, 0x01));

    let mut mavlink_buffer = heapless::Vec::<u8, 1024>::new();

    loop {
        let mut recv_buffer = [0; 1024];
        match select(
            downlink_subscriber.next_message_pure(),
            socket.recv_from(&mut recv_buffer),
        )
        .await
        {
            Either::First(message) => {
                // TODO: this is awful
                let frame = match message {
                    Dialect::Heartbeat(m) => endpoint.next_frame(&m).unwrap(),
                    Dialect::CommandAck(m) => endpoint.next_frame(&m).unwrap(),
                    Dialect::CanFrame(m) => endpoint.next_frame(&m).unwrap(),
                    Dialect::ScaledImu(m) => endpoint.next_frame(&m).unwrap(),
                    Dialect::ScaledImu2(m) => endpoint.next_frame(&m).unwrap(),
                    Dialect::ScaledImu3(m) => endpoint.next_frame(&m).unwrap(),
                    _ => continue,
                };

                let mut transmit_buffer = [0; 1024];
                let n = frame.serialize(&mut transmit_buffer).unwrap();
                let serialized = &transmit_buffer[..n];

                socket.send_to(serialized, remote_endpoint).await.unwrap();
            }
            Either::Second(res) => {
                let Ok((len, _peer)) = res else {
                    defmt::error!("Error receiving ethernet packet");
                    continue;
                };

                defmt::info!("eth: received packet: len: {}", len);
                if mavlink_buffer
                    .extend_from_slice(&recv_buffer[..len])
                    .is_err()
                {
                    defmt::error!(
                        "mavlink buffer of {} bytes was to short for message",
                        mavlink_buffer.len()
                    );
                    mavlink_buffer.truncate(0);
                    continue;
                }

                let frame_result = unsafe { mavio::Frame::deserialize(&mavlink_buffer) };
                match frame_result {
                    Ok(frame) => {
                        uplink_publisher.publish(frame).await;
                        mavlink_buffer.truncate(0);
                    }
                    // Assume the next part of the frame is sent in the next udp packet.
                    Err(FrameError::FrameBufferIsTooSmall { .. }) => {}
                    Err(e) => {
                        defmt::error!("eth: {}", Debug2Format(&e));
                        mavlink_buffer.truncate(0);
                    }
                }
            }
        }
    }
}
