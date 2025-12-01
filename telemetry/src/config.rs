use siphasher::sip::SipHasher;

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

const SEQUENCE_LENGTH: usize = 64;

pub struct LinkConfig {
    sequence: [usize; SEQUENCE_LENGTH],
    hmac_key: [u8; 16],
    /// power of two
    message_interval_ms: u32,
    /// also power of two
    packets_per_hop: u32,
}

impl LinkConfig {
    fn init(
        frequency_mask: [bool; NUM_FREQUENCIES],
        binding_phrase: &str,
        hmac_key: [u8; 16],
    ) -> Self {
        let mut siphasher = SipHasher::new_with_key(&[0x00; 16]);
        siphasher.write(binding_phrase.as_bytes());
        let seed = siphasher.finish();

        let mut sequence: [usize; SEQUENCE_LENGTH] = [0; SEQUENCE_LENGTH];
        // TODO: fill with used frequencies

        let mut rng = ChaCha20Rng::seed_from_u64(seed);
        sequence.shuffle(&mut rng);

        Self {
            sequence,
            hmac_key,
            message_interval_ms: 32,
            packets_per_hop: 1,
        }
    }

    fn num_frequencies(&self) -> usize {
        self.frequency_mask.iter().map(|b| *b as usize).sum()
    }

    fn hop(&self, t: u16) -> u32 {
        let packet = t as u32 / self.message_interval_ms;
        packet / self.packets_per_hop
    }

    fn frequency(&self, t: u16) -> u32 {
        let hop = self.hop(t) as usize;
        let mask_i = hop % self.num_frequencies();
        let freq_i = self
            .frequency_mask
            .iter()
            .enumerate()
            .filter(|(_i, b)| **b)
            .map(|(i, _b)| i)
            .nth(mask_i)
            .unwrap();
        FREQUENCIES[freq_i]
    }
}
