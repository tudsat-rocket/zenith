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
    InterfaceCommandPublisher, InterfaceRxSubscriber, InterfaceTxPublisher, UplinkCommand,
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

/// Whether a message targeting `(target_system, target_component)` is for us. A zero target is a
/// broadcast, which ground stations commonly use for parameter discovery.
fn addressed(target_system: u8, target_component: u8, system_id: u8, component_id: u8) -> bool {
    (target_system == 0 || target_system == system_id)
        && (target_component == 0 || target_component == component_id)
}

fn param_value_msg(info: &ParamInfo) -> Rapid {
    Rapid::ParamValue(ParamValue {
        param_id: info.name,
        param_value: info.value,
        param_type: info.ty,
        param_count: info.count,
        param_index: info.index,
    })
}

pub async fn run<P: ParamStore>(
    system_id: u8,
    component_id: u8,
    tx: InterfaceTxPublisher,
    mut rx: InterfaceRxSubscriber,
    cmd_tx: InterfaceCommandPublisher,
    store: &'static P,
) {
    log::info!("params: task started");
    loop {
        let frame = rx.next_message_pure().await;

        // This is likely not a packet intended for us, see commands.rs.
        if frame.system_id() < 0x7f {
            continue;
        }

        let Ok(msg) = frame.decode::<Rapid>() else {
            continue;
        };

        match msg {
            Rapid::ParamRequestList(req)
                if addressed(
                    req.target_system,
                    req.target_component,
                    system_id,
                    component_id,
                ) =>
            {
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
            Rapid::ParamRequestRead(req)
                if addressed(
                    req.target_system,
                    req.target_component,
                    system_id,
                    component_id,
                ) =>
            {
                let info = if req.param_index >= 0 {
                    store.by_index(req.param_index as u16)
                } else {
                    store.by_name(&req.param_id)
                };

                // Unknown params get no reply; the GCS request times out.
                if let Some(info) = info {
                    tx.publish(param_value_msg(&info)).await;
                }
            }
            Rapid::ParamSet(req)
                if addressed(
                    req.target_system,
                    req.target_component,
                    system_id,
                    component_id,
                ) =>
            {
                match store.set(&req.param_id, req.param_value, req.param_type) {
                    Ok((id, raw, info)) => {
                        log::info!("params: set id={id} raw={raw:#x}");
                        cmd_tx.publish(UplinkCommand::SetParam { id, raw }).await;
                        tx.publish(param_value_msg(&info)).await;
                    }
                    Err(ParamSetError::InvalidValue) => {
                        // Per spec, a rejected write is signaled by echoing the current (unchanged)
                        // value.
                        if let Some(info) = store.by_name(&req.param_id) {
                            tx.publish(param_value_msg(&info)).await;
                        }
                    }
                    Err(ParamSetError::UnknownParam) => {}
                }
            }
            _ => {}
        }
    }
}
