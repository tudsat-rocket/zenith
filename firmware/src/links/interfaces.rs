use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use mavio::error::FrameError;
use rapid_dialect::Rapid;
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

use crate::links::UplinkCommand;

pub mod ethernet;
pub mod usb;

pub const DOWNLINK_N: usize = 10;
pub const DOWNLINK_PUBS: usize = 5;
pub type InterfaceTx = PubSubChannel<CriticalSectionRawMutex, Rapid, DOWNLINK_N, 1, DOWNLINK_PUBS>;
pub type InterfaceTxPublisher =
    Publisher<'static, CriticalSectionRawMutex, Rapid, DOWNLINK_N, 1, DOWNLINK_PUBS>;
pub type InterfaceTxSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, Rapid, DOWNLINK_N, 1, DOWNLINK_PUBS>;

pub const UPLINK_N: usize = 5;
pub const UPLINK_SUBS: usize = 5;
pub type InterfaceRx = PubSubChannel<CriticalSectionRawMutex, Frame<V2>, UPLINK_N, UPLINK_SUBS, 1>;
pub type InterfaceRxPublisher =
    Publisher<'static, CriticalSectionRawMutex, Frame<V2>, UPLINK_N, UPLINK_SUBS, 1>;
pub type InterfaceRxSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, Frame<V2>, UPLINK_N, UPLINK_SUBS, 1>;

pub const COMMAND_N: usize = 5;
pub const COMMAND_SUBS: usize = 5;
pub type InterfaceCommands =
    PubSubChannel<CriticalSectionRawMutex, UplinkCommand, COMMAND_N, COMMAND_SUBS, 1>;
pub type InterfaceCommandPublisher =
    Publisher<'static, CriticalSectionRawMutex, UplinkCommand, COMMAND_N, COMMAND_SUBS, 1>;
pub type InterfaceCommandSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, UplinkCommand, COMMAND_N, COMMAND_SUBS, 1>;
