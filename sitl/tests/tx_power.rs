//! The LoRa downlink transmit power the vehicle asks for: what moves it, and what leaves it be.

#![allow(clippy::float_cmp, reason = "params round-trip bit-exactly")]

use common::{Harness, block_on};
use mission::Params;
use mission::params::{LORA_TX_POWER, LoraParams};
use rapid_dialect::FlightMode;

mod common;

/// `LORA_TX_PWR_HI`
const LORA_TX_POWER_ARMED: u16 = 0x0807;

#[test]
fn arming_boosts_and_lower_modes_do_not_undo_it() {
    block_on(async {
        let mut h = Harness::new(None).await;
        assert_eq!(h.vehicle.downlink_tx_power_dbm(), 10.0);

        h.arm();
        assert_eq!(h.vehicle.downlink_tx_power_dbm(), 22.0);

        // Whether the operator commands it or the uplink failsafe does, dropping back below
        // DetectLaunch must not take the radio down with it.
        h.vehicle.set_mode(FlightMode::Idle);
        assert_eq!(h.vehicle.downlink_tx_power_dbm(), 22.0);
    });
}

#[test]
fn a_tx_power_write_lowers_it_until_the_next_armed_mode() {
    block_on(async {
        let mut h = Harness::new(None).await;
        h.arm();

        h.vehicle
            .set_param(LORA_TX_POWER.get(), 12.0f32.to_bits())
            .await;
        assert_eq!(h.vehicle.downlink_tx_power_dbm(), 12.0);

        h.vehicle.set_mode(FlightMode::Vent);
        assert_eq!(h.vehicle.downlink_tx_power_dbm(), 12.0);

        h.vehicle.set_mode(FlightMode::Ignite);
        assert_eq!(h.vehicle.downlink_tx_power_dbm(), 22.0);
    });
}

#[test]
fn an_armed_tx_power_write_applies_while_armed() {
    block_on(async {
        let mut h = Harness::new(None).await;
        h.arm();

        h.vehicle
            .set_param(LORA_TX_POWER_ARMED, 16.0f32.to_bits())
            .await;
        assert_eq!(h.vehicle.downlink_tx_power_dbm(), 16.0);
    });
}

#[test]
fn both_levels_are_configurable() {
    block_on(async {
        let params = Params {
            lora: LoraParams {
                tx_power: 0.0,
                tx_power_armed: 14.0,
                ..LoraParams::default()
            },
            ..Params::default()
        };
        let mut h = Harness::new(Some(params)).await;
        assert_eq!(h.vehicle.downlink_tx_power_dbm(), 0.0);

        h.arm();
        assert_eq!(h.vehicle.downlink_tx_power_dbm(), 14.0);
    });
}
