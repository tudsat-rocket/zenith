//! The MAVLink parameter protocol over the telemetry link.
//!
//! The receiver knows the vehicle's parameter definitions, so only ids and raw values travel over
//! the air. It keeps none of the vehicle's values: those requests are forwarded to the vehicle,
//! which answers with [`ParamValuesMessage`]s. Lost packets are left to ground software, which
//! retries reads of the indices it is missing and writes it saw no echo for.
//!
//! The receiver's own [`GcsParams`] are answered locally, appended to the vehicle's in one stream:
//! they take the indices after the vehicle's, and every value reports the total of both as its
//! count.

use embassy_time::{Duration, Timer};

use links::protocols::params::{ParamInfo, ParamRequest, ParamSetError, ParamStore};
use links::{
    Downlink, InterfaceCommandPublisher, InterfaceRxSubscriber, InterfaceTxPublisher, UplinkCommand,
};
use mission::params::{LoraParams, ParamTable, ParameterGroup, Params, SharedParams};
use rapid_dialect::rapid::messages::ParamValue;

use crate::messages::{DownlinkTelemetryMessage, ParamEntry, ParamValuesMessage};

const _: () = assert!(Params::PARAM_COUNT <= u128::BITS as usize);

/// The parameters a receiver serves: the vehicle's, then its own.
#[allow(
    clippy::cast_possible_truncation,
    reason = "the vehicle's alone are capped at 128"
)]
pub const RECEIVER_PARAM_COUNT: u16 = (Params::PARAM_COUNT + GcsParams::PARAM_COUNT) as u16;

/// A bit for every parameter index.
const ALL_PARAMS: u128 = !match u128::MAX.checked_shl(Params::PARAM_COUNT as u32) {
    Some(above) => above,
    None => 0,
};

/// The parameter indices whose values the vehicle still has to send.
#[derive(Debug, Default)]
pub struct PendingParams(u128);

/// The ground station modem's own link settings, exposed over MAVLink as `GCS_*`. They have to
/// match the vehicle's `LORA_*`, and like those are read once at boot.
#[derive(Debug, Clone, macros::ParameterGroup)]
#[param_group(prefix = "GCS")]
pub struct GcsParams {
    /// Seed of the frequency hopping sequence, low word first. See [`LoraParams::seed`].
    #[param(id = 0x0900..=0x0901, name = "BIND", default = [0, 0])]
    pub seed: [u32; 2],
    /// Key authenticating every packet. See [`LoraParams::key`].
    #[param(id = 0x0902..=0x0905, name = "KEY", default = [0, 0, 0, 0])]
    pub key: [u32; 4],
}

/// What a receiver does about one request.
#[derive(Debug, Default)]
struct Answer {
    /// Commands for the vehicle.
    forward: heapless::Vec<UplinkCommand, { Params::PARAM_COUNT.div_ceil(u32::BITS as usize) }>,
    /// Values answered from the receiver's own params.
    local: heapless::Vec<ParamInfo, { GcsParams::PARAM_COUNT }>,
}

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
        let requested = u128::from(mask).checked_shl(u32::from(first)).unwrap_or(0);
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
                let raw = if LoraParams::is_secret(descriptor.id) {
                    0
                } else {
                    info.value.to_bits()
                };
                return Some(ParamEntry {
                    id: descriptor.id.get(),
                    raw,
                });
            }
        }

        None
    }
}

impl GcsParams {
    pub fn hop_seed(&self) -> u64 {
        mission::params::hop_seed(self.seed)
    }

    pub fn hmac_key(&self) -> [u8; 16] {
        mission::params::hmac_key(self.key)
    }
}

impl Answer {
    /// Splits `request` between the vehicle and `store`, the receiver's own params. Accepted sets
    /// of the latter are handed to `persist` as (id, raw).
    fn new(
        request: &ParamRequest,
        store: &SharedParams<GcsParams>,
        persist: impl FnOnce(u16, u32),
    ) -> Self {
        let mut answer = Self::default();

        match *request {
            ParamRequest::List => {
                answer.forward = Params::commands(request);
                answer.local = (0..GcsParams::count())
                    .filter_map(|index| store.by_index(index))
                    .collect();
            }
            ParamRequest::ReadIndex(index) => match index.checked_sub(Params::count()) {
                None => answer.forward = Params::commands(request),
                Some(local) => answer.local.extend(store.by_index(local)),
            },
            ParamRequest::ReadName(name) => {
                answer.forward = Params::commands(request);
                answer.local.extend(store.by_name(&name));
            }
            ParamRequest::Set { name, value, ty } => {
                answer.forward = Params::commands(request);
                match store.set(&name, value, ty) {
                    Ok((id, raw, info)) => {
                        persist(id, raw);
                        answer.local.extend([info]);
                    }
                    // A rejected write is answered with the current value.
                    Err(ParamSetError::InvalidValue) => answer.local.extend(store.by_name(&name)),
                    Err(ParamSetError::UnknownParam) => {}
                }
            }
        }

        for info in &mut answer.local {
            info.index = info.index.saturating_add(Params::count());
            info.count = RECEIVER_PARAM_COUNT;
        }

        answer
    }
}

/// Receiver side: forwards the PARAM_* requests addressed to `system_id` over the uplink, and
/// answers those about its own params from `store`. Replaces [`links::protocols::params::run`],
/// which answers everything from a local store.
pub async fn forward(
    system_id: u8,
    tx: InterfaceTxPublisher,
    mut rx: InterfaceRxSubscriber,
    cmd_tx: InterfaceCommandPublisher,
    store: &SharedParams<GcsParams>,
    persist: fn(u16, u32),
) -> ! {
    loop {
        let request = ParamRequest::receive(&mut rx, system_id).await;
        let answer = Answer::new(&request, store, persist);

        for command in answer.forward {
            cmd_tx.publish(command).await;
        }
        for info in &answer.local {
            tx.publish(Downlink::from_self(ParamValue::from(info)))
                .await;
            // Same pacing as links::protocols::params::run, the downlink is shared with telemetry.
            Timer::after(Duration::from_millis(20)).await;
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

    fn gcs_store() -> SharedParams<GcsParams> {
        let store = SharedParams::new();
        store.init(GcsParams {
            key: [0x4242_4242; 4],
            ..GcsParams::default()
        });
        store
    }

    fn no_persist(id: u16, raw: u32) {
        panic!("persisted {id:#x} = {raw:#x}");
    }

    fn store() -> SharedParams {
        let store = SharedParams::<Params>::new();
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
            .map(|i| {
                let descriptor = Params::descriptor_by_index(i).expect("in range");
                let value = store.by_index(i).expect("initialized").value;
                (
                    i,
                    if LoraParams::is_secret(descriptor.id) {
                        0.0
                    } else {
                        value
                    },
                )
            })
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
    fn the_key_is_not_sent_over_the_air() {
        let store = store();
        let key_index = |id| Params::by_id(id).expect("param exists").0;

        let values = forwarded(&ParamRequest::List, &store);
        for id in 0x0802..=0x0805 {
            assert!(values.contains(&(key_index(id), 0.0)), "{id:#x} leaked");
        }

        let request = ParamRequest::Set {
            name: name(0x0802),
            value: f32::from_bits(0x1234_5678),
            ty: MavParamType::Uint32,
        };
        assert_eq!(forwarded(&request, &store), [(key_index(0x0802), 0.0)]);
        assert_eq!(
            store
                .by_name(&name(0x0802))
                .map(|info| info.value.to_bits()),
            Some(0x1234_5678),
            "the vehicle still applies it"
        );
    }

    #[test]
    fn a_receiver_lists_its_own_params_after_the_vehicles() {
        let answer = Answer::new(&ParamRequest::List, &gcs_store(), no_persist);

        assert_eq!(answer.forward, Params::commands(&ParamRequest::List));
        let indices: Vec<_> = answer.local.iter().map(|info| info.index).collect();
        let expected: Vec<_> = (Params::count()..RECEIVER_PARAM_COUNT).collect();
        assert_eq!(indices, expected);
        assert!(
            answer
                .local
                .iter()
                .all(|info| info.count == RECEIVER_PARAM_COUNT)
        );
    }

    #[test]
    fn a_receiver_answers_reads_of_its_own_params() {
        let store = gcs_store();
        let gcs_key = GcsParams::by_id(0x0902).expect("param exists").1;

        for request in [
            ParamRequest::ReadIndex(Params::count().saturating_add(2)),
            ParamRequest::ReadName(gcs_key.mavlink_name()),
        ] {
            let answer = Answer::new(&request, &store, no_persist);
            assert!(answer.forward.is_empty(), "{request:?}");
            assert_eq!(answer.local.len(), 1, "{request:?}");
            assert_eq!(answer.local[0].name, gcs_key.mavlink_name());
            assert_eq!(answer.local[0].value.to_bits(), 0x4242_4242);
        }

        let answer = Answer::new(
            &ParamRequest::ReadIndex(RECEIVER_PARAM_COUNT),
            &store,
            no_persist,
        );
        assert!(answer.forward.is_empty() && answer.local.is_empty());
    }

    #[test]
    fn a_receiver_applies_and_persists_sets_of_its_own_params() {
        let store = gcs_store();
        let name = GcsParams::by_id(0x0900)
            .expect("param exists")
            .1
            .mavlink_name();
        let mut persisted = None;

        let answer = Answer::new(
            &ParamRequest::Set {
                name,
                value: f32::from_bits(7),
                ty: MavParamType::Uint32,
            },
            &store,
            |id, raw| persisted = Some((id, raw)),
        );

        assert!(answer.forward.is_empty());
        assert_eq!(persisted, Some((0x0900, 7)));
        assert_eq!(answer.local.len(), 1);
        assert_eq!(answer.local[0].value.to_bits(), 7);
        assert_eq!(answer.local[0].index, Params::count());

        // The wrong type is rejected and answered with the current value.
        let answer = Answer::new(
            &ParamRequest::Set {
                name,
                value: 1.0,
                ty: MavParamType::Real32,
            },
            &store,
            no_persist,
        );
        assert_eq!(answer.local.len(), 1);
        assert_eq!(answer.local[0].value.to_bits(), 7);
    }

    #[test]
    fn a_receiver_forwards_requests_about_the_vehicles_params() {
        let request = ParamRequest::ReadName(name(0x0200));
        let answer = Answer::new(&request, &gcs_store(), no_persist);

        assert_eq!(answer.forward, Params::commands(&request));
        assert!(answer.local.is_empty());
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
