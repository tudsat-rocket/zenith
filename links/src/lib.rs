#![no_std]

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};

use mavio::Frame;
use mavio::prelude::{Endpoint, V2};

use rapid_dialect::rapid::enums::{MavCmd, MavResult, ValveId};
use rapid_dialect::rapid::messages::CommandAck;
use rapid_dialect::{FlightMode, Rapid, ValveCommand};

pub mod protocols;

/// The MAVLink component the flight computer speaks as. Also the io boards' `master_node_id`
/// default, so zenith is node 1 on CAN and component 1 on MAVLink.
pub const SELF_COMPONENT_ID: u8 = 0x01;

/// One message on its way out, plus the MAVLink component it speaks for.
///
/// Almost everything zenith sends is its own; the exception is the heartbeat per IO board node,
/// which carries the node's id so ground software sees one component per board.
#[derive(Debug, Clone, PartialEq)]
pub struct Downlink {
    pub component_id: u8,
    pub message: Rapid,
}

impl Downlink {
    pub fn new(component_id: u8, message: impl Into<Rapid>) -> Self {
        Self {
            component_id,
            message: message.into(),
        }
    }

    pub fn from_self(message: impl Into<Rapid>) -> Self {
        Self::new(SELF_COMPONENT_ID, message)
    }

    pub fn command_ack(command: MavCmd, result: MavResult) -> Self {
        Self::from_self(CommandAck {
            command,
            result,
            ..Default::default()
        })
    }

    /// Builds the outgoing frame with this message's component rather than the endpoint's own,
    /// which is what [`Endpoint::next_frame`] would use. The sequence stays one run per link, so
    /// packet loss reconstructed from sequence gaps stays meaningful.
    pub fn frame(&self, endpoint: &Endpoint<V2>) -> mavio::error::Result<Frame<V2>> {
        Ok(Frame::builder()
            .sequence(endpoint.next_sequence())
            .system_id(endpoint.system_id())
            .component_id(self.component_id)
            .version(V2)
            .message(&self.message)?
            .build())
    }
}

impl From<Rapid> for Downlink {
    fn from(message: Rapid) -> Self {
        Self::from_self(message)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum UplinkCommand {
    SetFlightMode(FlightMode),
    RequestAvailableModes(usize),
    RequestCanForwarding,
    CommandValve(ValveId, ValveCommand),
    SetParam {
        id: u16,
        raw: u32,
    },
    /// Asks for the PARAM_VALUE of every flat param index `first + i` whose bit `i` is set.
    RequestParams {
        first: u16,
        mask: u32,
    },
}

impl UplinkCommand {
    /// The command whose terminal ack is left to the vehicle.
    pub fn mav_cmd(&self) -> Option<MavCmd> {
        match self {
            Self::SetFlightMode(_) => Some(MavCmd::DoSetMode),
            Self::CommandValve(..) => Some(MavCmd::CommandValve),
            Self::RequestAvailableModes(_)
            | Self::RequestCanForwarding
            | Self::SetParam { .. }
            | Self::RequestParams { .. } => None,
        }
    }
}

pub const DOWNLINK_N: usize = 32;
pub const DOWNLINK_PUBS: usize = 6;
pub type InterfaceTx =
    PubSubChannel<CriticalSectionRawMutex, Downlink, DOWNLINK_N, 1, DOWNLINK_PUBS>;
pub type InterfaceTxPublisher =
    Publisher<'static, CriticalSectionRawMutex, Downlink, DOWNLINK_N, 1, DOWNLINK_PUBS>;
pub type InterfaceTxSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, Downlink, DOWNLINK_N, 1, DOWNLINK_PUBS>;

pub const UPLINK_N: usize = 32;
pub const UPLINK_SUBS: usize = 5;
pub type InterfaceRx = PubSubChannel<CriticalSectionRawMutex, Frame<V2>, UPLINK_N, UPLINK_SUBS, 1>;
pub type InterfaceRxPublisher =
    Publisher<'static, CriticalSectionRawMutex, Frame<V2>, UPLINK_N, UPLINK_SUBS, 1>;
pub type InterfaceRxSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, Frame<V2>, UPLINK_N, UPLINK_SUBS, 1>;

pub const COMMAND_N: usize = 32;
pub const COMMAND_SUBS: usize = 5;
pub const COMMAND_PUBS: usize = 2;
pub type InterfaceCommands =
    PubSubChannel<CriticalSectionRawMutex, UplinkCommand, COMMAND_N, COMMAND_SUBS, COMMAND_PUBS>;
pub type InterfaceCommandPublisher = Publisher<
    'static,
    CriticalSectionRawMutex,
    UplinkCommand,
    COMMAND_N,
    COMMAND_SUBS,
    COMMAND_PUBS,
>;
pub type InterfaceCommandSubscriber = Subscriber<
    'static,
    CriticalSectionRawMutex,
    UplinkCommand,
    COMMAND_N,
    COMMAND_SUBS,
    COMMAND_PUBS,
>;

/// Set by every link that hears from a ground station and taken by the main loop each tick.
static UPLINK_ACTIVITY: AtomicBool = AtomicBool::new(false);

/// Reports ground station traffic, on any link
pub fn note_uplink_activity() {
    UPLINK_ACTIVITY.store(true, Ordering::Relaxed);
}

/// Whether any link heard from a ground station since the last call, clearing the flag.
pub fn take_uplink_activity() -> bool {
    UPLINK_ACTIVITY.swap(false, Ordering::Relaxed)
}
