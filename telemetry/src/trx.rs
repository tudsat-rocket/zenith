pub mod receiver;
pub mod transmitter;

/// Number of consecutive radio errors after which we hard-reset the transceiver.
pub(crate) const MAX_CONSECUTIVE_ERRORS: u32 = 5;
