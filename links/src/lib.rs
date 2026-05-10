#![no_std]

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};

use mavio::Frame;
use mavio::prelude::V2;

use rapid_dialect::rapid::enums::ValveId;
use rapid_dialect::{FlightMode, Rapid, ValveCommand};

pub mod protocols;

#[derive(Debug, Clone, PartialEq)]
pub enum UplinkCommand {
    SetFlightMode(FlightMode),
    RequestAvailableModes(usize),
    RequestCanForwarding,
    CommandValve(ValveId, ValveCommand),
}

pub const DOWNLINK_N: usize = 32;
pub const DOWNLINK_PUBS: usize = 5;
pub type InterfaceTx = PubSubChannel<CriticalSectionRawMutex, Rapid, DOWNLINK_N, 1, DOWNLINK_PUBS>;
pub type InterfaceTxPublisher =
    Publisher<'static, CriticalSectionRawMutex, Rapid, DOWNLINK_N, 1, DOWNLINK_PUBS>;
pub type InterfaceTxSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, Rapid, DOWNLINK_N, 1, DOWNLINK_PUBS>;

pub const UPLINK_N: usize = 32;
pub const UPLINK_SUBS: usize = 5;
pub type InterfaceRx = PubSubChannel<CriticalSectionRawMutex, Frame<V2>, UPLINK_N, UPLINK_SUBS, 1>;
pub type InterfaceRxPublisher =
    Publisher<'static, CriticalSectionRawMutex, Frame<V2>, UPLINK_N, UPLINK_SUBS, 1>;
pub type InterfaceRxSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, Frame<V2>, UPLINK_N, UPLINK_SUBS, 1>;

pub const COMMAND_N: usize = 32;
pub const COMMAND_SUBS: usize = 5;
pub type InterfaceCommands =
    PubSubChannel<CriticalSectionRawMutex, UplinkCommand, COMMAND_N, COMMAND_SUBS, 1>;
pub type InterfaceCommandPublisher =
    Publisher<'static, CriticalSectionRawMutex, UplinkCommand, COMMAND_N, COMMAND_SUBS, 1>;
pub type InterfaceCommandSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, UplinkCommand, COMMAND_N, COMMAND_SUBS, 1>;
