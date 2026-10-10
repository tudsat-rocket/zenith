use links::{InterfaceCommandPublisher, InterfaceRxSubscriber, InterfaceTxPublisher};

/// The PARAM_* handler for a MAVLink interface.
pub async fn run(
    system_id: u8,
    tx: InterfaceTxPublisher,
    rx: InterfaceRxSubscriber,
    cmd_tx: InterfaceCommandPublisher,
) {
    #[cfg(not(feature = "gcs"))]
    links::protocols::params::run(system_id, tx, rx, cmd_tx, &crate::storage::PARAM_STORE).await;
    #[cfg(feature = "gcs")]
    telemetry::params::forward(
        system_id,
        tx,
        rx,
        cmd_tx,
        &crate::storage::GCS_PARAM_STORE,
        |id, raw| crate::storage::FlashHandle::new().write_param(id, raw),
    )
    .await;
}
