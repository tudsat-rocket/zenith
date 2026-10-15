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

// TODO: replace with a more general default
const DOWNLINK_CHANNEL_MASK: u32 = 0b01_0001_0110_1000;

const UPLINK_CHANNEL_MASK: u32 = 0b00_0000_1001_0110;

/// Every channel a mask can select.
const ALL_CHANNELS: u32 = (1 << NUM_FREQUENCIES) - 1;

/// How far into a downlink message interval a transmission should start.
const DOWNLINK_OFFSET_MS: u16 = 1;

/// Time-on-air of one 16 byte packet at SF7, 500 kHz and CR 4/5, rounded up.
const TIME_ON_AIR_MS: u16 = 12;

/// How far behind the vehicle the ground station's estimate of its clock runs: the setup and
/// time-on-air of the downlink packet the estimate was taken from, plus the ground station's own
/// handling before it timestamps that packet.
const CLOCK_LAG_MS: u16 = 16;

/// How much room a transmission needs before the end of a hopping slot, measured from its start
/// on the estimated clock: the lag against that estimate, and then its own time-on-air.
const HOP_GUARD_MS: u16 = CLOCK_LAG_MS + TIME_ON_AIR_MS;

/// Skipping a hopping slot in [`LinkConfig::next_transmission_time`] adds the remainder of that
/// slot to a time already on the downlink grid, which lands back on the grid only if the two
/// intervals line up.
const _: () = assert!(DEFAULT_UPLINK_CONFIG.hopping_interval % DOWNLINK_MESSAGE_INTERVAL_MS == 0);

pub const DEFAULT_DOWNLINK_CONFIG: LinkConfig = LinkConfig {
    spreading_factor: SpreadingFactor::_7,
    bandwidth: Bandwidth::_500KHz,
    coding_rate: CodingRate::_4_5,
    preamble_length: 8,
    channel_mask: DOWNLINK_CHANNEL_MASK,
    hop_seed: 0,
    hopping_interval: DOWNLINK_MESSAGE_INTERVAL_MS,
    hmac_key: [0; 16],
};

pub const DEFAULT_UPLINK_CONFIG: LinkConfig = LinkConfig {
    spreading_factor: SpreadingFactor::_7,
    bandwidth: Bandwidth::_500KHz,
    coding_rate: CodingRate::_4_5,
    preamble_length: 8,
    channel_mask: UPLINK_CHANNEL_MASK,
    hop_seed: 0,
    hopping_interval: UPLINK_HOP_INTERVAL_MS,
    hmac_key: [0; 16],
};

pub struct LinkConfig {
    pub spreading_factor: SpreadingFactor,
    pub bandwidth: Bandwidth,
    pub coding_rate: CodingRate,
    pub preamble_length: u16,
    /// Bit `i` selects `FREQUENCIES[i]`.
    pub channel_mask: u32,
    /// See [`hop_seed`].
    pub hop_seed: u64,
    pub hopping_interval: u32,
    pub hmac_key: [u8; 16],
}

/// The seed of the hopping sequence for a binding phrase. Ground software derives it the same way,
/// so the phrase itself never has to be stored.
pub fn hop_seed(binding_phrase: &str) -> u64 {
    let mut siphasher = SipHasher::new_with_key(&[0x00; 16]);
    siphasher.write(binding_phrase.as_bytes());
    siphasher.finish()
}

impl LinkConfig {
    /// The downlink and uplink configs, or `None` if both masks are empty. Masks that cannot work
    /// (fewer than two channels, unknown channels, overlap) fall back to the defaults.
    pub fn links(
        downlink_mask: u32,
        uplink_mask: u32,
        hop_seed: u64,
        hmac_key: [u8; 16],
    ) -> Option<(Self, Self)> {
        if downlink_mask == 0 && uplink_mask == 0 {
            return None;
        }

        let valid = |mask: u32| mask & !ALL_CHANNELS == 0 && mask.count_ones() >= 2;
        let (downlink_mask, uplink_mask) =
            if valid(downlink_mask) && valid(uplink_mask) && downlink_mask & uplink_mask == 0 {
                (downlink_mask, uplink_mask)
            } else {
                // Host tests have no defmt logger to link against.
                #[cfg(not(test))]
                defmt::error!(
                    "Invalid LoRa channel masks {=u32:#x}/{=u32:#x}, using the defaults.",
                    downlink_mask,
                    uplink_mask
                );
                (DOWNLINK_CHANNEL_MASK, UPLINK_CHANNEL_MASK)
            };

        Some((
            Self {
                channel_mask: downlink_mask,
                hop_seed,
                hmac_key,
                ..DEFAULT_DOWNLINK_CONFIG
            },
            Self {
                channel_mask: uplink_mask,
                hop_seed,
                hmac_key,
                ..DEFAULT_UPLINK_CONFIG
            },
        ))
    }

    /// The hopping sequence, as channel indices into [`FREQUENCIES`].
    ///
    /// Built as back-to-back shuffles of the link's channels so channels don't repeat and are
    /// evenly spread across the whole sequence.
    #[allow(
        clippy::indexing_slicing,
        reason = "writes bounded by SEQUENCE_LENGTH, block indices by its own length"
    )]
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "the write cursor is bounded by SEQUENCE_LENGTH"
    )]
    pub fn sequence(&self) -> [usize; SEQUENCE_LENGTH] {
        let mut rng = ChaCha20Rng::seed_from_u64(self.hop_seed);

        // One block, one of each channel.
        let mut block: heapless::Vec<usize, NUM_FREQUENCIES> = (0..NUM_FREQUENCIES)
            .filter(|i| self.channel_mask >> i & 1 == 1)
            .collect();

        let mut sequence: [usize; SEQUENCE_LENGTH] = [0; SEQUENCE_LENGTH];
        let mut written = 0;

        while written < SEQUENCE_LENGTH {
            block.shuffle(&mut rng);

            // If the last channel of the last block matches the first of this block, swap this
            // block's first two around to fix that.
            if written > 0 && block.len() > 1 && block[0] == sequence[written - 1] {
                block.swap(0, 1);
            }

            for channel in &block {
                if written >= SEQUENCE_LENGTH {
                    break;
                }
                sequence[written] = *channel;
                written += 1;
            }
        }

        // Same as the block seam fixing, just for the first and last channel of the sequence.
        if SEQUENCE_LENGTH > 2 && sequence[SEQUENCE_LENGTH - 1] == sequence[0] {
            sequence.swap(0, 1);
        }

        sequence
    }

    /// The most slots that can pass between two appearances of the same channel, counted around
    /// the wrap. A receiver sweeping for a vehicle has to sit on a channel at least this long
    /// before it can conclude it would have heard one.
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "indices and distances bounded by SEQUENCE_LENGTH"
    )]
    #[allow(
        clippy::indexing_slicing,
        reason = "both indices are reduced modulo SEQUENCE_LENGTH"
    )]
    pub fn max_hop_gap(&self) -> usize {
        let sequence = self.sequence();

        (0..SEQUENCE_LENGTH)
            .map(|i| {
                (1..=SEQUENCE_LENGTH)
                    .find(|d| sequence[(i + d) % SEQUENCE_LENGTH] == sequence[i])
                    .unwrap_or(SEQUENCE_LENGTH)
            })
            .max()
            .unwrap_or(SEQUENCE_LENGTH)
    }

    /// The frequencies this link actually uses.
    pub fn channels(&self) -> heapless::Vec<u32, NUM_FREQUENCIES> {
        FREQUENCIES
            .iter()
            .enumerate()
            .filter_map(|(i, f)| (self.channel_mask >> i & 1 == 1).then_some(*f))
            .collect()
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

    extern crate std;
    use std::{format, string::String, vec::Vec};

    /// Every channel count and enough binding phrases that the properties below are claims about
    /// the construction rather than about the masks and the one phrase we happen to ship.
    fn sequences() -> Vec<(String, u32, [usize; SEQUENCE_LENGTH])> {
        let mut out = Vec::new();

        let every_count = (2..=NUM_FREQUENCIES).map(|n| (1 << n) - 1);
        for mask in [DOWNLINK_CHANNEL_MASK, UPLINK_CHANNEL_MASK]
            .into_iter()
            .chain(every_count)
        {
            for i in 0..200 {
                let binding_phrase = format!("phrase-{i}");
                let sequence = LinkConfig {
                    hop_seed: hop_seed(&binding_phrase),
                    channel_mask: mask,
                    ..DEFAULT_DOWNLINK_CONFIG
                }
                .sequence();
                out.push((binding_phrase, mask, sequence));
            }
        }

        out
    }

    fn channel_count(mask: u32) -> usize {
        mask.count_ones() as usize
    }

    /// A run parks the link on one frequency for as long as it lasts, which is what the hopping
    /// exists to avoid and what a dwell-time argument turns on. The sequence repeats, so its last
    /// slot is adjacent to its first.
    #[test]
    fn no_channel_ever_follows_itself() {
        for (phrase, _, sequence) in sequences() {
            for i in 0..SEQUENCE_LENGTH {
                assert_ne!(
                    sequence[i],
                    sequence[(i + 1) % SEQUENCE_LENGTH],
                    "{phrase} repeats at slot {i}: {sequence:?}"
                );
            }
        }
    }

    /// Every channel gets the same number of slots, or one more where the channel count does not
    /// divide [`SEQUENCE_LENGTH`].
    #[test]
    fn every_channel_gets_an_even_share() {
        for (phrase, mask, sequence) in sequences() {
            let fewest = SEQUENCE_LENGTH / channel_count(mask);

            for channel in (0..NUM_FREQUENCIES).filter(|i| mask >> i & 1 == 1) {
                let slots = sequence.iter().filter(|c| **c == channel).count();
                assert!(
                    slots == fewest || slots == fewest + 1,
                    "{phrase} gives channel {channel} of {mask:#x} {slots} slots"
                );
            }
        }
    }

    /// A receiver sweeping for a vehicle sits on each channel for a multiple of this, so it has to
    /// be bounded and small. One block can hold a channel's slot at its start and the next at its
    /// end, which is the worst a block construction allows. A cut-off last block can add one more
    /// across the wrap.
    #[test]
    fn the_gap_between_a_channels_slots_is_bounded() {
        for (phrase, mask, _) in sequences() {
            let config = LinkConfig {
                hop_seed: hop_seed(&phrase),
                channel_mask: mask,
                ..DEFAULT_DOWNLINK_CONFIG
            };

            let channels = channel_count(mask);
            let bound = if SEQUENCE_LENGTH % channels == 0 {
                2 * channels
            } else {
                3 * channels
            };
            let gap = config.max_hop_gap();
            assert!(
                gap <= bound,
                "{phrase} leaves a gap of {gap}, bound {bound}"
            );
        }
    }

    /// Both ends derive the sequence independently, so the construction *is* the wire protocol. A
    /// change here does not degrade a link, it stops two radios finding each other at all - which
    /// is worth being told about at review time rather than on a launch day with mixed firmware.
    #[test]
    fn the_shipped_sequence_does_not_change_by_accident() {
        #[rustfmt::skip]
        const DOWNLINK: [usize; SEQUENCE_LENGTH] = [
            3, 2, 6, 5, 8, 6, 3, 8, 2, 5, 8, 5, 6, 3, 2, 8,
            2, 5, 6, 3, 5, 3, 6, 2, 8, 6, 3, 5, 2, 8, 6, 8,
            2, 3, 5, 8, 5, 3, 6, 2, 5, 3, 8, 2, 6, 8, 2, 5,
            6, 3, 2, 3, 5, 8, 6, 8, 3, 5, 6, 2, 5, 3, 6, 2,
        ];
        #[rustfmt::skip]
        const UPLINK: [usize; SEQUENCE_LENGTH] = [
            1, 4, 12, 7, 12, 4, 7, 1, 7, 1, 12, 4, 7, 4, 1, 12,
            7, 1, 12, 4, 12, 1, 7, 4, 7, 12, 4, 1, 12, 4, 1, 7,
            4, 1, 12, 7, 12, 7, 4, 1, 7, 1, 4, 12, 1, 12, 4, 7,
            4, 12, 7, 1, 4, 12, 1, 7, 12, 1, 4, 7, 12, 1, 7, 4,
        ];

        assert_eq!(DEFAULT_DOWNLINK_CONFIG.sequence(), DOWNLINK);
        assert_eq!(DEFAULT_UPLINK_CONFIG.sequence(), UPLINK);
    }

    /// Ground software derives the seed from a phrase on its own, with a copy of this vector.
    #[test]
    fn the_phrase_hash_does_not_change_by_accident() {
        assert_eq!(hop_seed("schinken"), 0x53bd_d841_8658_7bcb);
    }

    /// The two directions divide the band up between them. An overlap would put uplink
    /// transmissions on a channel the vehicle is trying to be heard on.
    #[test]
    fn the_two_directions_do_not_share_a_channel() {
        assert_eq!(DOWNLINK_CHANNEL_MASK & UPLINK_CHANNEL_MASK, 0);

        let downlink = DEFAULT_DOWNLINK_CONFIG.channels();
        let uplink = DEFAULT_UPLINK_CONFIG.channels();

        assert!(downlink.iter().all(|f| !uplink.contains(f)));

        // They do not divide it up exhaustively, though: the band's outermost two channels are in
        // neither direction, since at 500 kHz each they reach past 863 and 870 MHz.
        for edge in [FREQUENCIES[0], FREQUENCIES[NUM_FREQUENCIES - 1]] {
            assert!(!downlink.contains(&edge) && !uplink.contains(&edge));
        }
    }

    #[test]
    fn channel_masks_that_cannot_work_fall_back_to_the_defaults() {
        let masks = |down, up| {
            LinkConfig::links(down, up, 0, [0; 16])
                .map(|(down, up)| (down.channel_mask, up.channel_mask))
        };
        let defaults = Some((DOWNLINK_CHANNEL_MASK, UPLINK_CHANNEL_MASK));

        assert_eq!(masks(0, 0), None);
        assert_eq!(masks(0b0011, 0b1100), Some((0b0011, 0b1100)));

        assert_eq!(masks(0, 0b1100), defaults);
        assert_eq!(masks(0b0001, 0b1100), defaults);
        assert_eq!(masks(0b0111, 0b1100), defaults);
        assert_eq!(masks(0b0011 | 1 << NUM_FREQUENCIES, 0b1100), defaults);
    }

    /// Radio setup plus time-on-air of one uplink packet, i.e. how long a transmission occupies
    /// the ground station between two calls to [`LinkConfig::next_transmission_time`].
    const TRANSMISSION_MS: u16 = TIME_ON_AIR_MS + 3;

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
        // arrived, so it lags the vehicle. This is what [`CLOCK_LAG_MS`] has to cover: the whole
        // scheme holds only as far as the real lag stays under it.
        for lag in 0..=CLOCK_LAG_MS {
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
