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
use rapid_dialect::rapid::messages::{AvailableModes, CommandAck, CommandLong, SupportedTunes};
use rapid_dialect::{FlightMode, Rapid, ValveCommand};

use crate::protocols::link_quality::LinkQuality;
use crate::{
    InterfaceCommandPublisher, InterfaceRxSubscriber, InterfaceTxPublisher, SELF_COMPONENT_ID,
    TUNE_NAME_LEN, UplinkCommand,
};

/// The tune format we advertise in `SUPPORTED_TUNES`.
///
/// The field is documented as a bitfield, but `TUNE_FORMAT` is not declared as
/// a bitmask in the dialect, so it is generated as a plain enum and we can only
/// name a single format. `TUNE_FORMAT_QBASIC1_1` happens to be bit 0, so the
/// encoded value is a valid bitfield either way.
const SUPPORTED_TUNE_FORMAT: TuneFormat = TuneFormat::Qbasic11;

/// The 0-based servo each 1-based DO_WINCH instance drives. `mission` asserts these against its
/// `ServoId`s.
pub const WINCH_SERVOS: [u8; 2] = [2, 3];

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

        // Anything that got this far came from a ground station, whether or not it is a command.
        crate::note_uplink_activity();

        match msg {
            Rapid::CommandLong(cmd)
                if cmd.target_system == system_id && cmd.target_component == SELF_COMPONENT_ID =>
            {
                let mut reboot_requested = false;

                let result = match cmd.command {
                    MavCmd::DoSetMode => {
                        let custom_mode = cmd.param2 as u8;
                        if let Ok(mode) = FlightMode::try_from(custom_mode)
                            && (cmd.param1 as u32) == 0x01
                        {
                            cmd_tx.publish(UplinkCommand::SetFlightMode(mode)).await;
                            MavResult::InProgress
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
                                tx.publish(
                                    Rapid::SupportedTunes(SupportedTunes {
                                        target_system: frame.system_id(),
                                        target_component: frame.component_id(),
                                        format: SUPPORTED_TUNE_FORMAT,
                                    })
                                    .into(),
                                )
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
                            MavResult::InProgress
                        } else {
                            MavResult::Denied
                        }
                    }
                    MavCmd::DoGripper => match gripper_command(&cmd) {
                        Ok(gripper_cmd) => {
                            cmd_tx.publish(gripper_cmd).await;
                            MavResult::InProgress
                        }
                        Err(result) => result,
                    },
                    MavCmd::DoSetServo | MavCmd::DoSetActuator | MavCmd::DoWinch => {
                        match servo_command(&cmd) {
                            Ok(servo_cmd) => {
                                cmd_tx.publish(servo_cmd).await;
                                MavResult::InProgress
                            }
                            Err(result) => result,
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

                let _ = tx.publish(Rapid::CommandAck(ack).into()).await;

                if reboot_requested {
                    reboot().await;
                }
            }
            Rapid::CommandInt(cmd)
                if cmd.target_system == system_id && cmd.target_component == SELF_COMPONENT_ID =>
            {
                let ack = CommandAck {
                    command: cmd.command,
                    result: MavResult::CommandLongOnly,
                    target_system: frame.system_id(),
                    target_component: frame.component_id(),
                    ..Default::default()
                };

                let _ = tx.publish(Rapid::CommandAck(ack).into()).await;
            }
            Rapid::PlayTuneV2(msg)
                if msg.target_system == system_id && msg.target_component == SELF_COMPONENT_ID =>
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

/// DO_SET_SERVO addresses a servo by 1-based instance with a pulse width in microseconds.
/// DO_SET_ACTUATOR addresses six per set, with -1..1 values and NaN to skip one; only one of them
/// may be set. DO_WINCH addresses a retract servo by 1-based instance; only relative length
/// control is accepted, and its length is taken as the absolute position in promille.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "values are clamped to 0..=1000 first; float-to-int casts saturate"
)]
fn servo_command(cmd: &CommandLong) -> Result<UplinkCommand, MavResult> {
    let (servo, promille) = match cmd.command {
        MavCmd::DoSetServo => {
            if !cmd.param1.is_finite() || !cmd.param2.is_finite() {
                return Err(MavResult::Denied);
            }
            let servo = (cmd.param1 as u8).checked_sub(1).ok_or(MavResult::Denied)?;
            (servo, (cmd.param2 - 1000.0).clamp(0.0, 1000.0) as u16)
        }
        MavCmd::DoSetActuator => {
            let values = [
                cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6,
            ];
            let mut set = values.into_iter().zip(0u8..).filter(|(v, _)| !v.is_nan());
            let (value, offset) = set.next().ok_or(MavResult::Denied)?;
            if set.next().is_some() {
                return Err(MavResult::Unsupported);
            }
            if !cmd.param7.is_finite() {
                return Err(MavResult::Denied);
            }
            let servo = (cmd.param7 as u8)
                .checked_mul(6)
                .and_then(|base| base.checked_add(offset))
                .ok_or(MavResult::Denied)?;
            (servo, ((value.clamp(-1.0, 1.0) + 1.0) * 500.0) as u16)
        }
        MavCmd::DoWinch => {
            if !cmd.param1.is_finite() || !cmd.param3.is_finite() {
                return Err(MavResult::Denied);
            }
            #[allow(
                clippy::float_cmp,
                reason = "MAVLink enum values travel as exact integers in a float"
            )]
            // WINCH_RELATIVE_LENGTH_CONTROL
            if cmd.param2 != 1.0 {
                return Err(MavResult::Unsupported);
            }
            let servo = (cmd.param1 as u8)
                .checked_sub(1)
                .and_then(|instance| WINCH_SERVOS.get(usize::from(instance)))
                .ok_or(MavResult::Denied)?;
            (*servo, cmd.param3.clamp(0.0, 1000.0) as u16)
        }
        _ => return Err(MavResult::Unsupported),
    };

    Ok(UplinkCommand::SetServo {
        command: cmd.command,
        servo,
        promille,
    })
}

/// DO_GRIPPER addresses a quick disconnect by 1-based instance, with GRIPPER_ACTION_RELEASE (0)
/// or GRIPPER_ACTION_GRAB (1).
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "float-to-int casts saturate; the result is range-checked"
)]
fn gripper_command(cmd: &CommandLong) -> Result<UplinkCommand, MavResult> {
    if !cmd.param1.is_finite() {
        return Err(MavResult::Denied);
    }
    let instance = cmd.param1 as u8;
    if instance == 0 {
        return Err(MavResult::Denied);
    }
    let grab = match cmd.param2 {
        0.0 => false,
        1.0 => true,
        _ => return Err(MavResult::Denied),
    };

    Ok(UplinkCommand::Gripper { instance, grab })
}

async fn reboot() {
    log::warn!("rebooting");
    Timer::after(Duration::from_millis(250)).await;

    #[cfg(target_os = "none")]
    cortex_m::peripheral::SCB::sys_reset();

    #[cfg(not(target_os = "none"))]
    log::error!("no MCU to reset on this platform, ignoring");
}

#[cfg(test)]
mod tests {
    use super::*;

    fn servo(command: MavCmd, params: [f32; 7]) -> Result<UplinkCommand, MavResult> {
        let [param1, param2, param3, param4, param5, param6, param7] = params;
        servo_command(&CommandLong {
            command,
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7,
            ..Default::default()
        })
    }

    fn set_servo(command: MavCmd, servo: u8, promille: u16) -> Result<UplinkCommand, MavResult> {
        Ok(UplinkCommand::SetServo {
            command,
            servo,
            promille,
        })
    }

    #[test]
    fn set_servo_maps_pulse_width_onto_promille() {
        let cmd = MavCmd::DoSetServo;
        let at = |instance, pwm| servo(cmd, [instance, pwm, 0.0, 0.0, 0.0, 0.0, 0.0]);

        assert_eq!(at(1.0, 1000.0), set_servo(cmd, 0, 0));
        assert_eq!(at(3.0, 1500.0), set_servo(cmd, 2, 500));
        assert_eq!(at(2.0, 900.0), set_servo(cmd, 1, 0));
        assert_eq!(at(2.0, 2500.0), set_servo(cmd, 1, 1000));

        // Instances are 1-based.
        assert_eq!(at(0.0, 1500.0), Err(MavResult::Denied));
        assert_eq!(at(1.0, f32::NAN), Err(MavResult::Denied));
        assert_eq!(at(f32::NAN, 1500.0), Err(MavResult::Denied));
    }

    #[test]
    fn set_actuator_takes_the_one_value_set() {
        let cmd = MavCmd::DoSetActuator;
        let nan = f32::NAN;

        assert_eq!(
            servo(cmd, [nan, nan, -1.0, nan, nan, nan, 0.0]),
            set_servo(cmd, 2, 0)
        );
        assert_eq!(
            servo(cmd, [nan, 2.0, nan, nan, nan, nan, 0.0]),
            set_servo(cmd, 1, 1000)
        );
        assert_eq!(
            servo(cmd, [0.5, nan, nan, nan, nan, nan, 1.0]),
            set_servo(cmd, 6, 750)
        );

        assert_eq!(
            servo(cmd, [0.0, nan, 0.0, nan, nan, nan, 0.0]),
            Err(MavResult::Unsupported)
        );
        assert_eq!(servo(cmd, [nan; 7]), Err(MavResult::Denied));
        assert_eq!(
            servo(cmd, [0.0, nan, nan, nan, nan, nan, 100.0]),
            Err(MavResult::Denied)
        );
    }

    #[test]
    fn winch_takes_the_length_as_promille() {
        let cmd = MavCmd::DoWinch;
        let at =
            |instance, action, length| servo(cmd, [instance, action, length, 0.0, 0.0, 0.0, 0.0]);

        assert_eq!(at(1.0, 1.0, 250.0), set_servo(cmd, 2, 250));
        assert_eq!(at(2.0, 1.0, 1000.0), set_servo(cmd, 3, 1000));
        assert_eq!(at(1.0, 1.0, -5.0), set_servo(cmd, 2, 0));
        assert_eq!(at(1.0, 1.0, 5000.0), set_servo(cmd, 2, 1000));

        assert_eq!(at(0.0, 1.0, 500.0), Err(MavResult::Denied));
        assert_eq!(at(3.0, 1.0, 500.0), Err(MavResult::Denied));
        assert_eq!(at(1.0, 1.0, f32::NAN), Err(MavResult::Denied));
        // Only WINCH_RELATIVE_LENGTH_CONTROL.
        assert_eq!(at(1.0, 0.0, 500.0), Err(MavResult::Unsupported));
        assert_eq!(at(1.0, 2.0, 500.0), Err(MavResult::Unsupported));
    }

    #[test]
    fn gripper_takes_a_one_based_instance_and_an_action() {
        let at = |instance, action| {
            gripper_command(&CommandLong {
                command: MavCmd::DoGripper,
                param1: instance,
                param2: action,
                ..Default::default()
            })
        };

        assert_eq!(
            at(1.0, 0.0),
            Ok(UplinkCommand::Gripper {
                instance: 1,
                grab: false
            })
        );
        assert_eq!(
            at(2.0, 1.0),
            Ok(UplinkCommand::Gripper {
                instance: 2,
                grab: true
            })
        );

        assert_eq!(at(0.0, 0.0), Err(MavResult::Denied));
        assert_eq!(at(f32::NAN, 0.0), Err(MavResult::Denied));
        assert_eq!(at(1.0, 0.5), Err(MavResult::Denied));
        assert_eq!(at(1.0, f32::NAN), Err(MavResult::Denied));
    }
}
