//! This task handles COMMAND_LONG and COMMAND_INT messages and is in charge of producing
//! acknowledgments for each requested command.
//!
//! Commands that are understood, valid and will be executed are forwarded on a separate
//! [`PubSubChannel`], to be handled both by other async tasks and the main loop.
//!
//! Executed for both Ethernet and USB links.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Instant, Timer};

use rapid_dialect::rapid::enums::{MavCmd, MavResult, TuneFormat, ValveId};
use rapid_dialect::rapid::messages::{AvailableModes, CommandAck, SupportedTunes};
use rapid_dialect::{FlightMode, Rapid, ValveCommand};

use crate::protocols::link_quality::LinkQuality;
use crate::{
    InterfaceCommandPublisher, InterfaceRxSubscriber, InterfaceTxPublisher, TUNE_NAME_LEN,
    UplinkCommand,
};

/// The tune format we advertise in `SUPPORTED_TUNES`.
///
/// The field is documented as a bitfield, but `TUNE_FORMAT` is not declared as
/// a bitmask in the dialect, so it is generated as a plain enum and we can only
/// name a single format. `TUNE_FORMAT_QBASIC1_1` happens to be bit 0, so the
/// encoded value is a valid bitfield either way.
const SUPPORTED_TUNE_FORMAT: TuneFormat = TuneFormat::Qbasic11;

/// Read the NUL-terminated tune out of a `PLAY_TUNE_V2` message.
///
/// Returns `None` if it is not valid UTF-8 or is longer than a tune name can
/// be, which - until actual tune notation is supported - means it cannot name
/// one of the firmware's sounds.
fn tune_name(tune: &[u8; 248]) -> Option<heapless::String<TUNE_NAME_LEN>> {
    let end = tune.iter().position(|b| *b == 0).unwrap_or(tune.len());
    let name = core::str::from_utf8(tune.get(..end)?).ok()?.trim();

    heapless::String::try_from(name).ok()
}

#[allow(clippy::too_many_lines, reason = "TODO")]
// Packet-loss tracking below reorders sequence numbers with a bounded lookback:
// `previous_i` stays < len, and `wrapping_sub(last) - 1` is guarded by `last != seq`.
#[allow(
    clippy::arithmetic_side_effects,
    reason = "bounded sequence-number reorder/loss math, wrapping or length-guarded"
)]
#[allow(
    clippy::indexing_slicing,
    reason = "reorder index bounded by received_sorted.len()"
)]
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
            log::debug!(
                "commands: ignoring frame from sys_id={:#x} (< 0x7f)",
                frame.system_id()
            );
            continue;
        }

        let Ok(msg) = frame.decode::<Rapid>() else {
            log::debug!("commands: failed to decode frame");
            continue;
        };

        log::debug!(
            "commands: received message from sys={} comp={}",
            frame.system_id(),
            frame.component_id()
        );

        match msg {
            Rapid::CommandLong(cmd)
                if cmd.target_system == system_id && cmd.target_component == component_id =>
            {
                let mut reboot_requested = false;

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
                        log::debug!("commands: RequestMessage id={}", cmd.param1 as u32);
                        match cmd.param1 as u32 {
                            AvailableModes::ID => {
                                log::info!(
                                    "commands: RequestMessage for AvailableModes (index={})",
                                    cmd.param2 as usize
                                );
                                cmd_tx
                                    .publish(UplinkCommand::RequestAvailableModes(
                                        cmd.param2 as usize,
                                    ))
                                    .await;
                                MavResult::Accepted
                            }
                            SupportedTunes::ID => {
                                // A single message, so unlike AVAILABLE_MODES it is
                                // answered here instead of via the command channel.
                                log::info!("commands: RequestMessage for SupportedTunes");
                                tx.publish(Rapid::SupportedTunes(SupportedTunes {
                                    target_system: frame.system_id(),
                                    target_component: frame.component_id(),
                                    format: SUPPORTED_TUNE_FORMAT,
                                }))
                                .await;
                                MavResult::Accepted
                            }
                            _ => MavResult::Denied,
                        }
                    }
                    MavCmd::PreflightRebootShutdown => match cmd.param1 as u32 {
                        0 => MavResult::Accepted,
                        1 => {
                            reboot_requested = true;
                            MavResult::Accepted
                        }
                        _ => MavResult::Unsupported,
                    },
                    MavCmd::CanForward => {
                        cmd_tx.publish(UplinkCommand::RequestCanForwarding).await;
                        MavResult::Accepted
                    }
                    MavCmd::CommandValve => {
                        let valve_id = ValveId::try_from(cmd.param1 as u8).ok();
                        let target_pos = cmd.param2.is_finite().then(|| cmd.param2.clamp(0.0, 1.0));
                        let duration = core::time::Duration::try_from_secs_f32(cmd.param3)
                            .ok()
                            .filter(|dur| !dur.is_zero());

                        let valve_cmd = match (target_pos, duration) {
                            (Some(p), Some(dur)) if p > 0.99 => Some(ValveCommand::PulseOpen(dur)),
                            (Some(p), None) if p > 0.99 => Some(ValveCommand::Open),
                            (Some(p), None) if p < 0.01 => Some(ValveCommand::Close),
                            (Some(p), None) => Some(ValveCommand::Partial(p)),
                            _ => None,
                        };

                        if let (Some(vid), Some(vc)) = (valve_id, valve_cmd) {
                            cmd_tx.publish(UplinkCommand::CommandValve(vid, vc)).await;
                            // TODO: currently this is lying, because the command may be dropped
                            // in the vehicle main loop if the current flight mode does not permit
                            // it. rethink our command result handling.
                            MavResult::Accepted
                        } else {
                            MavResult::Denied
                        }
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

                if reboot_requested {
                    reboot().await;
                }
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
            Rapid::PlayTuneV2(msg)
                if msg.target_system == system_id && msg.target_component == component_id =>
            {
                // PLAY_TUNE_V2 is a message, not a command, so there is nothing to
                // acknowledge - a tune we cannot make sense of is just logged.
                if msg.format != SUPPORTED_TUNE_FORMAT {
                    log::warn!("play_tune: unexpected tune format {:?}", msg.format);
                }

                match tune_name(&msg.tune) {
                    Some(name) => {
                        log::info!("play_tune: requested tune {:?}", name.as_str());
                        cmd_tx.publish(UplinkCommand::PlayTune(name)).await;
                    }
                    None => log::warn!(
                        "play_tune: tune is not the name of a known sound, and tune notation is not supported yet"
                    ),
                }
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
        for (_t, seq, _) in &received_queue {
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

async fn reboot() {
    log::warn!("rebooting");
    Timer::after(Duration::from_millis(250)).await;

    #[cfg(target_os = "none")]
    cortex_m::peripheral::SCB::sys_reset();

    #[cfg(not(target_os = "none"))]
    log::error!("no MCU to reset on this platform, ignoring");
}
