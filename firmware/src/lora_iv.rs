//! Interface variant wrapper that limits the LoRa driver's internal waits.
//!
//! `lora-phy` waits on the transceiver's BUSY and DIO1 lines without any timeout, so a transceiver
//! that stops responding hangs the calling task forever. The futures are not cancel-safe from the
//! outside, so we wrap our `InterfaceVariant` to insert those timeouts here.

use embassy_time::{Duration, with_timeout};
use embedded_hal_async::delay::DelayNs;

use lora_phy::mod_params::RadioError;
use lora_phy::mod_traits::InterfaceVariant;

pub struct BoundedWaits<IV> {
    inner: IV,
    busy_timeout: Duration,
    irq_timeout: Duration,
}

impl<IV: InterfaceVariant> BoundedWaits<IV> {
    pub fn new(inner: IV, busy_timeout: Duration, irq_timeout: Duration) -> Self {
        Self {
            inner,
            busy_timeout,
            irq_timeout,
        }
    }
}

impl<IV: InterfaceVariant> InterfaceVariant for BoundedWaits<IV> {
    async fn reset(&mut self, delay: &mut impl DelayNs) -> Result<(), RadioError> {
        self.inner.reset(delay).await
    }

    async fn wait_on_busy(&mut self) -> Result<(), RadioError> {
        with_timeout(self.busy_timeout, self.inner.wait_on_busy())
            .await
            .map_err(|_timeout| RadioError::Busy)?
    }

    async fn await_irq(&mut self) -> Result<(), RadioError> {
        with_timeout(self.irq_timeout, self.inner.await_irq())
            .await
            .map_err(|_timeout| RadioError::Irq)?
    }

    async fn enable_rf_switch_rx(&mut self) -> Result<(), RadioError> {
        self.inner.enable_rf_switch_rx().await
    }

    async fn enable_rf_switch_tx(&mut self) -> Result<(), RadioError> {
        self.inner.enable_rf_switch_tx().await
    }

    async fn disable_rf_switch(&mut self) -> Result<(), RadioError> {
        self.inner.disable_rf_switch().await
    }
}
