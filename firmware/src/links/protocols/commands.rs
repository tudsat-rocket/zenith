//! This task handles COMMAND_LONG and COMMAND_INT messages and is in charge of producing
//! acknowledgments for each requested command.
//!
//! Commands that are understood, valid and will be executed are forwarded on a separate
//! [`PubSubChannel`], to be handled both by other async tasks and the main loop.
//!
//! Executed for both Ethernet and USB links.

use static_cell::StaticCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver};

use rapid_dialect::rapid::{
    enums::{MavCmd, MavModeProperty, MavResult, MavStandardMode},
    messages::{AvailableModes, CommandAck},
};
use rapid_dialect::{FlightMode, Rapid};

use crate::links::UplinkCommand;
use crate::links::interfaces::{
    InterfaceCommandPublisher, InterfaceRxSubscriber, InterfaceTxPublisher,
};

#[embassy_executor::task(pool_size = 2)]
pub async fn run(
    system_id: u8,
    component_id: u8,
    tx: InterfaceTxPublisher,
    mut rx: InterfaceRxSubscriber,
    cmd_tx: InterfaceCommandPublisher,
) {
    loop {
        let frame = rx.next_message_pure().await;
        let Ok(msg) = frame.decode::<Rapid>() else {
            continue;
        };

        match msg {
            Rapid::CommandLong(cmd)
                if cmd.target_system == system_id && cmd.target_component == component_id =>
            {
                let result = match cmd.command {
                    MavCmd::DoSetMode => {
                        let custom_mode = cmd.param2 as u8;
                        if let Ok(mode) = FlightMode::try_from(custom_mode)
                            && (cmd.param1 as u32) == 0x01
                        {
                            cmd_tx.publish(UplinkCommand::SetFlightMode(mode)).await;
                            MavResult::Accepted
                        } else {
                            MavResult::Denied
                        }
                    }
                    MavCmd::RequestMessage => {
                        let cmd = match cmd.param1 as u32 {
                            AvailableModes::ID => {
                                Some(UplinkCommand::RequestAvailableModes(cmd.param2 as usize))
                            }
                            _ => None,
                        };

                        if let Some(cmd) = cmd {
                            cmd_tx.publish(cmd).await;
                            MavResult::Accepted
                        } else {
                            MavResult::Denied
                        }
                    }
                    MavCmd::CanForward => {
                        //let bus_enable = cmd.param1 as u8;
                        // For now we only support bus1
                        //can_forwarding_enabled = bus_enable > 0;
                        // TODO: enable/disable, bus number
                        cmd_tx.publish(UplinkCommand::RequestCanForwarding).await;
                        MavResult::Accepted
                    }
                    _ => MavResult::Unsupported,
                };

                let ack = CommandAck {
                    command: cmd.command,
                    result,
                    target_system: frame.system_id(),
                    target_component: frame.component_id(),
                    ..Default::default()
                };

                let _ = tx.publish(Rapid::CommandAck(ack)).await;
            }
            Rapid::CommandInt(cmd)
                if cmd.target_system == system_id && cmd.target_component == component_id =>
            {
                let ack = CommandAck {
                    command: cmd.command,
                    result: MavResult::CommandLongOnly,
                    target_system: frame.system_id(),
                    target_component: frame.component_id(),
                    ..Default::default()
                };

                let _ = tx.publish(Rapid::CommandAck(ack)).await;
            }
            _ => {}
        };
    }
}
