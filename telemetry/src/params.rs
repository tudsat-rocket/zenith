//! The MAVLink parameter protocol over the telemetry link.
//!
//! The receiver knows the vehicle's parameter definitions, so only ids and raw values travel over
//! the air. It keeps no values of its own: every request is forwarded to the vehicle, which
//! answers with [`ParamValuesMessage`]s. Lost packets are left to ground software, which retries
//! reads of the indices it is missing and writes it saw no echo for.

use links::protocols::params::{ParamRequest, ParamStore};
use links::{InterfaceCommandPublisher, InterfaceRxSubscriber, UplinkCommand};
use mission::params::{ParameterGroup, Params};

use crate::messages::{DownlinkTelemetryMessage, ParamEntry, ParamValuesMessage};

const _: () = assert!(Params::PARAM_COUNT <= u64::BITS as usize);

/// A bit for every parameter index.
const ALL_PARAMS: u64 = !match u64::MAX.checked_shl(Params::PARAM_COUNT as u32) {
    Some(above) => above,
    None => 0,
};

/// The parameter indices whose values the vehicle still has to send.
#[derive(Debug, Default)]
pub struct PendingParams(u64);

impl PendingParams {
    /// Handles the parameter commands, returning the others.
    pub fn handle<P: ParamStore>(
        &mut self,
        store: &P,
        command: UplinkCommand,
    ) -> Option<UplinkCommand> {
        match command {
            UplinkCommand::RequestParams { first, mask } => {
                self.request(first, mask);
                None
            }
            UplinkCommand::SetParam { id, raw } => self.apply_set(store, id, raw),
            command => Some(command),
        }
    }

    /// Queues the indices `first + i` for every set bit `i` of `mask`. Unknown indices are ignored.
    fn request(&mut self, first: u16, mask: u32) {
        let requested = u64::from(mask).checked_shl(u32::from(first)).unwrap_or(0);
        self.0 |= requested & ALL_PARAMS;
    }

    /// Applies a PARAM_SET to the mirror and queues the resulting value, which is the unchanged
    /// one if the set was rejected. Returns the command that applies it to the vehicle.
    fn apply_set<P: ParamStore>(&mut self, store: &P, id: u16, raw: u32) -> Option<UplinkCommand> {
        let (index, descriptor) = Params::by_id(id)?;
        self.request(index, 1);

        let (id, raw, _) = store
            .set(
                &descriptor.mavlink_name(),
                f32::from_bits(raw),
                descriptor.ty.into(),
            )
            .ok()?;

        Some(UplinkCommand::SetParam { id, raw })
    }

    /// The next one or two queued values, lowest index first.
    pub fn take<P: ParamStore>(&mut self, store: &P) -> Option<ParamValuesMessage> {
        let first = self.pop(store)?;
        let second = self.pop(store).unwrap_or(first);
        Some(ParamValuesMessage::pack([first, second]))
    }

    fn pop<P: ParamStore>(&mut self, store: &P) -> Option<ParamEntry> {
        while self.0 != 0 {
            let index = u16::try_from(self.0.trailing_zeros()).ok()?;
            self.0 &= self.0.wrapping_sub(1);

            if let (Some(descriptor), Some(info)) =
                (Params::descriptor_by_index(index), store.by_index(index))
            {
                return Some(ParamEntry {
                    id: descriptor.id.get(),
                    raw: info.value.to_bits(),
                });
            }
        }

        None
    }
}

/// Receiver side: forwards the PARAM_* requests addressed to `system_id` over the uplink.
/// Replaces [`links::protocols::params::run`], which answers from a local store.
pub async fn forward(
    system_id: u8,
    mut rx: InterfaceRxSubscriber,
    cmd_tx: InterfaceCommandPublisher,
) -> ! {
    loop {
        let request = ParamRequest::receive(&mut rx, system_id).await;
        for command in Params::commands(&request) {
            cmd_tx.publish(command).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    extern crate std;
    use std::vec::Vec;

    use mission::params::SharedParams;
    use rapid_dialect::rapid::enums::MavParamType;

    use crate::messages::downlink::tests::through_packet;
    use crate::messages::{ConnectionContext, DownlinkMessage};

    fn store() -> SharedParams {
        let store = SharedParams::new();
        store.init(Params::default());
        store
    }

    fn drain(pending: &mut PendingParams, store: &SharedParams) -> Vec<(u16, f32)> {
        let mut values = Vec::new();
        while let Some(msg) = pending.take(store) {
            let DownlinkMessage::ParamValues(msg) =
                through_packet(DownlinkMessage::ParamValues(msg))
            else {
                panic!("decoded as the wrong message")
            };
            values.extend(
                msg.unpack(&mut ConnectionContext::init(0))
                    .into_iter()
                    .map(|v| (v.param_index, v.param_value)),
            );
        }
        values
    }

    /// Forwards `request` to a vehicle, returning the values it would send.
    fn forwarded(request: &ParamRequest, store: &SharedParams) -> Vec<(u16, f32)> {
        let mut pending = PendingParams::default();
        for command in Params::commands(request) {
            let _ = pending.handle(store, command);
        }
        drain(&mut pending, store)
    }

    fn name(id: u16) -> [u8; 16] {
        Params::by_id(id).expect("param exists").1.mavlink_name()
    }

    #[test]
    fn a_list_request_yields_every_value_once() {
        let store = store();

        let expected: Vec<_> = (0..Params::count())
            .map(|i| (i, store.by_index(i).expect("initialized").value))
            .collect();
        assert_eq!(forwarded(&ParamRequest::List, &store), expected);
    }

    #[test]
    fn reads_by_index_and_name_yield_that_value() {
        let store = store();
        let (index, _) = Params::by_id(0x0200).expect("param exists");

        for request in [
            ParamRequest::ReadIndex(index),
            ParamRequest::ReadName(name(0x0200)),
        ] {
            assert_eq!(forwarded(&request, &store), [(index, 400.0)]);
        }
    }

    #[test]
    fn requests_for_unknown_params_are_not_forwarded() {
        for request in [
            ParamRequest::ReadName(*b"NOT_A_PARAM\0\0\0\0\0"),
            ParamRequest::ReadIndex(Params::count()),
            ParamRequest::Set {
                name: *b"NOT_A_PARAM\0\0\0\0\0",
                value: 1.0,
                ty: MavParamType::Real32,
            },
        ] {
            assert!(Params::commands(&request).is_empty(), "{request:?}");
        }
    }

    #[test]
    fn a_set_is_applied_and_echoed() {
        let store = store();
        let (index, _) = Params::by_id(0x0201).expect("param exists");
        let request = ParamRequest::Set {
            name: name(0x0201),
            value: f32::from_bits(1500),
            ty: MavParamType::Uint32,
        };

        assert_eq!(
            Params::commands(&request).as_slice(),
            [UplinkCommand::SetParam {
                id: 0x0201,
                raw: 1500
            }]
        );

        let values = forwarded(&request, &store);
        assert_eq!(values.len(), 1);
        assert_eq!(values[0].0, index);
        assert_eq!(values[0].1.to_bits(), 1500);
    }

    /// A set of the wrong type never reaches the vehicle, but is still answered with the current
    /// value.
    #[test]
    fn a_rejected_set_echoes_the_current_value() {
        let store = store();
        let (index, _) = Params::by_id(0x0200).expect("param exists");
        let request = ParamRequest::Set {
            name: name(0x0200),
            value: 1.0,
            ty: MavParamType::Uint32,
        };

        assert_eq!(forwarded(&request, &store), [(index, 400.0)]);
    }

    #[test]
    fn the_vehicle_echoes_a_set_it_rejects() {
        let store = store();
        let (index, _) = Params::by_id(0x0200).expect("param exists");
        let mut pending = PendingParams::default();

        assert_eq!(
            pending.handle(
                &store,
                UplinkCommand::SetParam {
                    id: 0x0200,
                    raw: f32::NAN.to_bits()
                }
            ),
            None
        );
        assert_eq!(drain(&mut pending, &store), [(index, 400.0)]);

        assert_eq!(
            pending.handle(&store, UplinkCommand::SetParam { id: 0xffff, raw: 0 }),
            None
        );
        assert!(pending.take(&store).is_none());
    }

    #[test]
    fn requests_beyond_the_params_are_ignored() {
        let store = store();
        let mut pending = PendingParams::default();

        pending.request(Params::count(), u32::MAX);
        pending.request(u16::MAX, 1);

        assert!(pending.take(&store).is_none());
    }
}
