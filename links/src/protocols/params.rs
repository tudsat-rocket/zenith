//! This task implements the MAVLink parameter microservice (PARAM_REQUEST_LIST, PARAM_REQUEST_READ,
//! PARAM_SET, PARAM_VALUE).
//!
//! The actual parameter definitions live in the mission crate, which depends on this one, so the
//! task is generic over a [`ParamStore`].
//!
//! Parameter values are exchanged bytewise: `param_value` carries the raw bits rather than a
//! by-value cast, so integer params survive exactly. That is advertised in the `AutopilotVersion`
//! message.

use embassy_time::{Duration, Timer};

use rapid_dialect::Rapid;
use rapid_dialect::rapid::enums::MavParamType;
use rapid_dialect::rapid::messages::ParamValue;

use crate::{
    Downlink, InterfaceCommandPublisher, InterfaceRxSubscriber, InterfaceTxPublisher,
    SELF_COMPONENT_ID, UplinkCommand,
};

/// Everything needed to emit one PARAM_VALUE message.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamInfo {
    /// NUL-padded MAVLink parameter name.
    pub name: [u8; 16],
    /// Current value, already encoded into the `param_value` float.
    pub value: f32,
    pub ty: MavParamType,
    pub index: u16,
    pub count: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamSetError {
    UnknownParam,
    InvalidValue,
}

pub trait ParamStore: Sync {
    fn count(&self) -> u16;
    fn by_index(&self, index: u16) -> Option<ParamInfo>;
    fn by_name(&self, name: &[u8; 16]) -> Option<ParamInfo>;

    /// Validate and apply a PARAM_SET to the live mirror. Returns the storage id and raw encoding
    /// (for persistence) and the info to echo back.
    fn set(
        &self,
        name: &[u8; 16],
        value: f32,
        ty: MavParamType,
    ) -> Result<(u16, u32, ParamInfo), ParamSetError>;
}

/// A PARAM_* request addressed to us.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParamRequest {
    List,
    ReadIndex(u16),
    ReadName([u8; 16]),
    Set {
        name: [u8; 16],
        value: f32,
        ty: MavParamType,
    },
}

/// Whether a message targeting `(target_system, target_component)` is for us. A zero target is a
/// broadcast, which ground stations commonly use for parameter discovery.
fn addressed(target_system: u8, target_component: u8, system_id: u8) -> bool {
    (target_system == 0 || target_system == system_id)
        && (target_component == 0 || target_component == SELF_COMPONENT_ID)
}

impl From<&ParamInfo> for ParamValue {
    fn from(info: &ParamInfo) -> Self {
        Self {
            param_id: info.name,
            param_value: info.value,
            param_type: info.ty,
            param_count: info.count,
            param_index: info.index,
        }
    }
}

impl ParamRequest {
    /// The next request on `rx` addressed to `system_id`.
    pub async fn receive(rx: &mut InterfaceRxSubscriber, system_id: u8) -> Self {
        loop {
            let frame = rx.next_message_pure().await;

            // This is likely not a packet intended for us, see commands.rs.
            if frame.system_id() < 0x7f {
                continue;
            }

            if let Some(request) = frame
                .decode::<Rapid>()
                .ok()
                .and_then(|msg| Self::parse(&msg, system_id))
            {
                return request;
            }
        }
    }

    fn parse(msg: &Rapid, system_id: u8) -> Option<Self> {
        match msg {
            Rapid::ParamRequestList(req)
                if addressed(req.target_system, req.target_component, system_id) =>
            {
                Some(Self::List)
            }
            Rapid::ParamRequestRead(req)
                if addressed(req.target_system, req.target_component, system_id) =>
            {
                Some(match u16::try_from(req.param_index) {
                    Ok(index) => Self::ReadIndex(index),
                    Err(_) => Self::ReadName(req.param_id),
                })
            }
            Rapid::ParamSet(req)
                if addressed(req.target_system, req.target_component, system_id) =>
            {
                Some(Self::Set {
                    name: req.param_id,
                    value: req.param_value,
                    ty: req.param_type,
                })
            }
            _ => None,
        }
    }
}

fn param_value_msg(info: &ParamInfo) -> Downlink {
    Downlink::from_self(ParamValue::from(info))
}

pub async fn run<P: ParamStore>(
    system_id: u8,
    tx: InterfaceTxPublisher,
    mut rx: InterfaceRxSubscriber,
    cmd_tx: InterfaceCommandPublisher,
    store: &'static P,
) {
    log::info!("params: task started");
    loop {
        match ParamRequest::receive(&mut rx, system_id).await {
            ParamRequest::List => {
                log::info!("params: enumerating {} params", store.count());
                for index in 0..store.count() {
                    if let Some(info) = store.by_index(index) {
                        tx.publish(param_value_msg(&info)).await;
                        // Pace the stream so we don't crowd out telemetry on the shared downlink
                        // channel.
                        Timer::after(Duration::from_millis(20)).await;
                    }
                }
            }
            ParamRequest::ReadIndex(index) => {
                // Unknown params get no reply; the GCS request times out.
                if let Some(info) = store.by_index(index) {
                    tx.publish(param_value_msg(&info)).await;
                }
            }
            ParamRequest::ReadName(name) => {
                if let Some(info) = store.by_name(&name) {
                    tx.publish(param_value_msg(&info)).await;
                }
            }
            ParamRequest::Set { name, value, ty } => match store.set(&name, value, ty) {
                Ok((id, raw, info)) => {
                    log::info!("params: set id={id} raw={raw:#x}");
                    cmd_tx.publish(UplinkCommand::SetParam { id, raw }).await;
                    tx.publish(param_value_msg(&info)).await;
                }
                Err(ParamSetError::InvalidValue) => {
                    // Per spec, a rejected write is signaled by echoing the current (unchanged)
                    // value.
                    if let Some(info) = store.by_name(&name) {
                        tx.publish(param_value_msg(&info)).await;
                    }
                }
                Err(ParamSetError::UnknownParam) => {}
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use rapid_dialect::rapid::messages::{ParamRequestList, ParamRequestRead};

    const SYSTEM_ID: u8 = 0x06;

    #[test]
    fn reads_are_by_index_unless_it_is_negative() {
        let read = |param_index| {
            ParamRequest::parse(
                &Rapid::ParamRequestRead(ParamRequestRead {
                    target_system: SYSTEM_ID,
                    target_component: SELF_COMPONENT_ID,
                    param_id: *b"SM_MAIN_ALT\0\0\0\0\0",
                    param_index,
                }),
                SYSTEM_ID,
            )
        };

        assert_eq!(read(3), Some(ParamRequest::ReadIndex(3)));
        assert_eq!(
            read(-1),
            Some(ParamRequest::ReadName(*b"SM_MAIN_ALT\0\0\0\0\0"))
        );
    }

    #[test]
    fn only_requests_addressed_to_us_or_broadcast_are_parsed() {
        let list = |target_system, target_component| {
            ParamRequest::parse(
                &Rapid::ParamRequestList(ParamRequestList {
                    target_system,
                    target_component,
                }),
                SYSTEM_ID,
            )
        };

        assert_eq!(list(SYSTEM_ID, SELF_COMPONENT_ID), Some(ParamRequest::List));
        assert_eq!(list(0, 0), Some(ParamRequest::List));
        assert_eq!(list(SYSTEM_ID.wrapping_add(1), 0), None);
        assert_eq!(list(SYSTEM_ID, SELF_COMPONENT_ID.wrapping_add(1)), None);
    }
}
