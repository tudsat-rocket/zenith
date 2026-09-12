use core::hash::Hasher;

use rand::prelude::*;
use rand_chacha::ChaCha20Rng;
use siphasher::sip::SipHasher;

use lora_phy::mod_params::{Bandwidth, CodingRate, SpreadingFactor};

use crate::{DOWNLINK_MESSAGE_INTERVAL_MS, UPLINK_HOP_INTERVAL_MS};

pub const NUM_FREQUENCIES: usize = 14;
pub const FREQUENCIES: [u32; NUM_FREQUENCIES] = [
    863_250_000,
    863_750_000,
    864_250_000,
    864_750_000,
    865_250_000,
    865_750_000,
    866_250_000,
    866_750_000,
    867_250_000,
    867_750_000,
    868_250_000,
    868_750_000,
    869_250_000,
    869_750_000,
];

pub const SEQUENCE_LENGTH: usize = 64;

const DOWNLINK_FREQUENCY_MASK: [bool; NUM_FREQUENCIES] = [
    true, false, true, true, false, true, true, false, true, true, true, true, false, true,
];

const UPLINK_FREQUENCY_MASK: [bool; NUM_FREQUENCIES] = [
    false, true, false, false, true, false, false, true, false, false, false, false, true, false,
];

/// How far into a downlink message interval a transmission should start.
const DOWNLINK_OFFSET_MS: u16 = 1;

/// How much room a transmission needs before the end of a hopping slot: its own time-on-air,
/// plus slack for the ground station's estimate of the vehicle's clock.
const HOP_GUARD_MS: u16 = 40;

/// Skipping a hopping slot in [`LinkConfig::next_transmission_time`] adds the remainder of that
/// slot to a time already on the downlink grid, which lands back on the grid only if the two
/// intervals line up.
const _: () = assert!(DEFAULT_UPLINK_CONFIG.hopping_interval % DOWNLINK_MESSAGE_INTERVAL_MS == 0);

pub const DEFAULT_DOWNLINK_CONFIG: LinkConfig = LinkConfig {
    spreading_factor: SpreadingFactor::_7,
    bandwidth: Bandwidth::_500KHz,
    coding_rate: CodingRate::_4_5,
    preamble_length: 8,
    frequency_mask: DOWNLINK_FREQUENCY_MASK,
    binding_phrase: "schinken",
    hopping_interval: DOWNLINK_MESSAGE_INTERVAL_MS,
    hmac_key: [0x42; 16],
};

pub const DEFAULT_UPLINK_CONFIG: LinkConfig = LinkConfig {
    spreading_factor: SpreadingFactor::_7,
    bandwidth: Bandwidth::_500KHz,
    coding_rate: CodingRate::_4_5,
    preamble_length: 8,
    frequency_mask: UPLINK_FREQUENCY_MASK,
    binding_phrase: "schinken",
    hopping_interval: UPLINK_HOP_INTERVAL_MS,
    hmac_key: [0x42; 16],
};

pub struct LinkConfig<'a> {
    pub spreading_factor: SpreadingFactor,
    pub bandwidth: Bandwidth,
    pub coding_rate: CodingRate,
    pub preamble_length: u16,
    pub frequency_mask: [bool; NUM_FREQUENCIES],
    pub binding_phrase: &'a str,
    pub hopping_interval: u32,
    pub hmac_key: [u8; 16],
}

impl LinkConfig<'_> {
    #[allow(
        clippy::indexing_slicing,
        reason = "writes bounded by SEQUENCE_LENGTH / NUM_FREQUENCIES"
    )]
    pub fn sequence(&self) -> [usize; SEQUENCE_LENGTH] {
        let mut siphasher = SipHasher::new_with_key(&[0x00; 16]);
        siphasher.write(self.binding_phrase.as_bytes());
        let seed = siphasher.finish();

        let mut sequence: [usize; SEQUENCE_LENGTH] = [0; SEQUENCE_LENGTH];
        for (seq_i, freq_i) in (0..NUM_FREQUENCIES)
            .filter(|i| self.frequency_mask[*i])
            .cycle()
            .enumerate()
        {
            if seq_i >= SEQUENCE_LENGTH {
                break;
            }

            sequence[seq_i] = freq_i;
        }

        let mut rng = ChaCha20Rng::seed_from_u64(seed);
        sequence.shuffle(&mut rng);

        sequence
    }

    #[allow(
        clippy::arithmetic_side_effects,
        reason = "modular hop index, divisor is a nonzero const"
    )]
    #[allow(
        clippy::indexing_slicing,
        reason = "index is % SEQUENCE_LENGTH; freq_i from the fixed table"
    )]
    pub fn frequency(&self, t: u16) -> u32 {
        let i = (t as usize / self.hopping_interval as usize) % SEQUENCE_LENGTH;
        let freq_i = self.sequence()[i]; // TODO
        FREQUENCIES[freq_i]
    }

    /// When to start the next transmission, given the time `t` on the vehicle's clock.
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "bounded modular frequency-hop timing math"
    )]
    pub fn next_transmission_time(&self, t: u16) -> u16 {
        let interval = DOWNLINK_MESSAGE_INTERVAL_MS as u16;
        let hop = self.hopping_interval as u16;

        let aligned = t.wrapping_add((interval + DOWNLINK_OFFSET_MS - t % interval) % interval);

        let into_hop = aligned % hop;
        if into_hop + HOP_GUARD_MS > hop {
            aligned.wrapping_add(hop - into_hop + DOWNLINK_OFFSET_MS)
        } else {
            aligned
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Radio setup plus time-on-air of one uplink packet, i.e. how long a transmission occupies
    /// the ground station between two calls to [`LinkConfig::next_transmission_time`].
    const TRANSMISSION_MS: u16 = 15;

    /// Time-on-air alone (16 bytes, SF7, 500 kHz, CR 4/5, rounded up), measured from the moment
    /// the packet starts going out.
    const TIME_ON_AIR_MS: u16 = 12;

    #[test]
    fn transmissions_sit_on_the_downlink_grid_and_clear_of_the_next_hop() {
        let config = &DEFAULT_UPLINK_CONFIG;
        let interval = DOWNLINK_MESSAGE_INTERVAL_MS as u16;
        let hop = config.hopping_interval as u16;

        for t in 0..=u16::MAX {
            let target = config.next_transmission_time(t);

            // Never wait out a whole hopping slot to send one packet.
            assert!(target.wrapping_sub(t) < hop, "waited too long at {t}");
            assert_eq!(target % interval, DOWNLINK_OFFSET_MS, "at {t}");
            assert!(target % hop + HOP_GUARD_MS <= hop, "at {t}");
        }
    }

    /// Every packet of a retransmit burst has to go out on the channel the vehicle is listening
    /// on, for its whole time-on-air. Deriving one frequency for the whole burst up front put the
    /// later packets on a channel the vehicle had already hopped away from.
    #[test]
    fn a_retransmit_burst_stays_on_the_vehicles_channel() {
        let config = &DEFAULT_UPLINK_CONFIG;

        // Our clock estimate is the timestamp of the last downlink packet plus the time since it
        // arrived, so it lags the vehicle by that packet's time-on-air and by whatever the
        // vehicle's scheduler put in front of it. We do not know that lag, so cover a range.
        for lag in 0..=20u16 {
            for start in 0..(SEQUENCE_LENGTH as u16 * UPLINK_HOP_INTERVAL_MS as u16) {
                let mut t = start;

                for _ in 0..3 {
                    let target = config.next_transmission_time(t);
                    let frequency = config.frequency(target);
                    let on_air = target.wrapping_add(lag);

                    assert_eq!(
                        config.frequency(on_air),
                        frequency,
                        "start of packet at {t}, lag {lag}"
                    );
                    assert_eq!(
                        config.frequency(on_air.wrapping_add(TIME_ON_AIR_MS)),
                        frequency,
                        "end of packet at {t}, lag {lag}"
                    );

                    t = target.wrapping_add(TRANSMISSION_MS);
                }
            }
        }
    }
}
