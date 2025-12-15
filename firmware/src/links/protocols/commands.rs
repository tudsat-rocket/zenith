//! This task handles COMMAND_LONG and COMMAND_INT messages and is in charge of producing
//! acknowledgments for each requested command.
//!
//! Commands that are understood, valid and will be executed are forwarded on a separate
//! [`PubSubChannel`], to be handled both by other async tasks and the main loop.
//!
//! Executed for both Ethernet and USB links.

use core::cmp::Ordering;

use embassy_sync::watch::Sender;
use static_cell::StaticCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver};
use embassy_time::{Duration, Instant};

use rapid_dialect::rapid::{
    enums::{MavCmd, MavModeProperty, MavResult, MavStandardMode},
    messages::{AvailableModes, CommandAck},
};
use rapid_dialect::{FlightMode, Rapid};

use crate::links::UplinkCommand;
use crate::links::interfaces::ethernet::ETHERNET_SYSTEM_ID;
use crate::links::interfaces::{
    InterfaceCommandPublisher, InterfaceRxSubscriber, InterfaceTxPublisher,
};
use crate::links::protocols::link_quality::LinkQuality;

#[embassy_executor::task(pool_size = 2)]
#[allow(clippy::too_many_lines, reason = "TODO")]
pub async fn run(
    system_id: u8,
    component_id: u8,
    tx: InterfaceTxPublisher,
    mut rx: InterfaceRxSubscriber,
    cmd_tx: InterfaceCommandPublisher,
    link_quality_sender: Sender<'static, CriticalSectionRawMutex, LinkQuality, 3>,
) {
    let mut received_queue: heapless::Deque<(Instant, u8, usize), 64> = heapless::Deque::new();

    loop {
        let frame = rx.next_message_pure().await;

        // This is likely not a packet intended for us. Ground stations tend to have high IDs.
        // This may be something like another flight computer on the same network.
        if frame.system_id() < 0x7f {
            continue;
        }

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
        }

        while received_queue
            .front()
            .map(|(t, _, _)| t.elapsed() > Duration::from_millis(5000))
            .unwrap_or(false)
        {
            let _ = received_queue.pop_front();
        }

        let _ = received_queue.push_back((Instant::now(), frame.sequence(), frame.body_length()));

        // Messages might arrive out of order, so to track packet loss we attempt to reorder minor
        // shuffles, otherwise we end up with big bursts of 255 "lost" packets.
        let mut received_sorted: heapless::Vec<u8, 64> = heapless::Vec::new();
        for (t, seq, _) in &received_queue {
            let mut i = received_sorted.len();

            // We look back into the past by at most 5 packets and insert the packet before any
            // previously added packets with a suspiciously slightly higher sequence number.
            let lookback = usize::min(received_sorted.len(), 5);
            for j in 0..lookback {
                let previous_i = received_sorted.len() - lookback + j;
                let seq_other = received_sorted[previous_i];
                let diff = (*seq as i16).wrapping_sub(seq_other as i16);
                if (diff < 0 && diff > -50) || diff > 205 {
                    i = previous_i;
                    break;
                }
            }

            let _ = received_sorted.insert(i, *seq);
        }

        let lq = LinkQuality {
            tx_rate: 0,
            rx_rate: received_queue.iter().map(|(_t, _seq, b)| *b as u32).sum(),
            messages_received: received_queue.len() as u32,
            messages_lost: received_sorted
                .iter()
                .fold((0u32, None), |(mut total, mut last_seq), seq| {
                    if let Some(last) = last_seq
                        && last != *seq
                    {
                        total += (seq.wrapping_sub(last) - 1) as u32;
                    }
                    last_seq = Some(*seq);
                    (total, last_seq)
                })
                .0,
        };

        link_quality_sender.send(lq);
    }
}
