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

    pub fn frequency(&self, t: u16) -> u32 {
        let i = (t as usize / self.hopping_interval as usize) % SEQUENCE_LENGTH;
        let freq_i = self.sequence()[i]; // TODO
        FREQUENCIES[freq_i]
    }
}
