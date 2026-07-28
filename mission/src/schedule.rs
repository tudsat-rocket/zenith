//! The schedule for telemetry downlink messages, or how it is built.
//!
//! We want to send different messages at different rates (e.g. fast moving sensors vs firmware
//! information), and we also want to spread out sending those messages, ideally limiting us to a
//! single message per tick.
//!
//! Because two message frequencies might be multiples of each other (e.g. sensor message every 10ms
//! and GPS every 1000ms), we need to phase offset some messages, preferably the slower ones, so the
//! fast sensor timeseries stay jitter-free.
//!
//! To avoid manually having to maintain phase offsets in a giant list of `t % 100 == 15`, a const
//! function ([`allocate`]) works these out at compile-time, with some macro magic to make the
//! schedule declaration conventient.

/// When one message goes out: on every tick `t` with `t % interval_ms == phase_ms`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Slot {
    pub interval_ms: u32,
    pub phase_ms: u32,
}

/// One slot per interval, in the order the intervals came in. See the module docs.
#[expect(
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    reason = "const-evaluated walk over fixed-length arrays: every index is bounded by its loop \
              condition and every tick counter by the cycle length"
)]
pub const fn allocate<const N: usize>(intervals: [u32; N]) -> [Slot; N] {
    /// The slowest interval a schedule may declare.
    const MAX_CYCLE_MS: usize = 8192;

    let mut cycle = 0;
    let mut i = 0;
    while i < N {
        assert!(intervals[i] > 0, "interval can't be 0");
        if intervals[i] > cycle {
            cycle = intervals[i];
        }
        i += 1;
    }
    assert!(cycle as usize <= MAX_CYCLE_MS, "slower than MAX_CYCLE_MS");

    let mut i = 0;
    while i < N {
        assert!(
            cycle % intervals[i] == 0,
            "every interval has to divide the slowest one."
        );
        i += 1;
    }

    // Visit the fastest messages first: they constrain more ticks than the slow ones and should
    // pick first. Going by interval rather than by position is what keeps the result independent
    // of the order the schedule happens to be written in. Insertion sort, so equal intervals stay
    // in declaration order - a selection sort would not.
    let mut order = [0usize; N];
    let mut i = 0;
    while i < N {
        let mut j = i;
        while j > 0 && intervals[order[j - 1]] > intervals[i] {
            order[j] = order[j - 1];
            j -= 1;
        }
        order[j] = i;
        i += 1;
    }

    // Which ticks of the cycle are spoken for. Only the first `cycle` entries are ever touched.
    let mut occupied = [false; MAX_CYCLE_MS];
    let mut slots = [Slot {
        interval_ms: 0,
        phase_ms: 0,
    }; N];

    let mut k = 0;
    while k < N {
        let index = order[k];
        let interval = intervals[index];

        // The lowest phase whose whole run of ticks through the cycle is still free.
        let mut phase = 0;
        while phase < interval {
            let mut free = true;
            let mut tick = phase;
            while tick < cycle {
                if occupied[tick as usize] {
                    free = false;
                    break;
                }
                tick += interval;
            }
            if free {
                break;
            }
            phase += 1;
        }

        assert!(
            phase < interval,
            "downlink schedule: one message has no tick left to itself - every phase of its \
             interval already carries a faster one. Drop a message, slow a rate down, or move \
             some of them onto a slower interval"
        );

        let mut tick = phase;
        while tick < cycle {
            occupied[tick as usize] = true;
            tick += interval;
        }

        slots[index] = Slot {
            interval_ms: interval,
            phase_ms: phase,
        };
        k += 1;
    }

    slots
}

/// How many slots one entry of the [`downlink_schedule!`] takes: one per message actually built, so
/// an instance message takes one per component. Must stay in step with [`downlink_entry!`] - the
/// n-th `due()` is answered with the n-th slot, so a disagreement drops messages.
macro_rules! downlink_entry_slots {
    ($message:ty) => {
        1
    };
    // `$ids` is expanded here in const position and again at runtime below, so it has to be a
    // const array, not just anything iterable.
    ($message:ty, $ids:expr) => {
        $ids.len()
    };
}

/// One entry of the [`downlink_schedule!`]. Asks `$due` exactly once per message it may build.
///
/// `$message` and the `InstanceMessage` impl behind it are resolved at the call site, which is
/// where the schedule and the conversions live.
macro_rules! downlink_entry {
    // regular message, built from snapshot
    ($due:ident, $snapshot:expr, $link:ident, $message:ty) => {
        if $due() {
            let message: $message = $snapshot.into();
            $link.send_message(message.into());
        }
    };
    // instance message, iterate through all ids
    ($due:ident, $snapshot:expr, $link:ident, $message:ty, $ids:expr) => {
        for id in $ids {
            if $due() {
                let message = <$message as InstanceMessage<_>>::build($snapshot, id);
                $link.send_message(message.into());
            }
        }
    };
}

/// Declares the downlink schedule: which messages the flight computer sends unprompted, and how
/// often. Adding one means naming its type under the rate it should go out at, and nothing else.
///
/// Expands to statements, so it goes in the body of whatever does the sending; it takes the tick
/// counter, the snapshot to build from and the link to send on. Writing `Message[Ids::ALL]` sends
/// one message per component, each taking a slot of its own.
macro_rules! downlink_schedule {
    ($time:expr, $snapshot:expr, $link:ident:
        $(every $interval_ms:literal ms => $($message:ty $([$ids:expr])?),+ $(,)? ;)+
    ) => {
        /// One slot per message built, not per entry declared.
        const SLOT_COUNT: usize =
            0 $($( + $crate::schedule::downlink_entry_slots!($message $(, $ids)?) )+)+;

        #[expect(
            clippy::indexing_slicing,
            reason = "const-evaluated fill of a fixed-length array; the cursor lands exactly on \
                      the array length, which the assert below pins"
        )]
        const INTERVALS: [u32; SLOT_COUNT] = {
            let mut intervals = [0; SLOT_COUNT];
            let mut i = 0;
            $($({
                let mut n = 0;
                while n < $crate::schedule::downlink_entry_slots!($message $(, $ids)?) {
                    intervals[i] = $interval_ms;
                    i += 1;
                    n += 1;
                }
            })+)+
            assert!(i == SLOT_COUNT);
            intervals
        };

        /// Fails to compile if the schedule cannot be spread out; the panic says why.
        const SLOTS: [$crate::schedule::Slot; SLOT_COUNT] =
            $crate::schedule::allocate(INTERVALS);

        let time = $time;

        // Slots come out in declaration order and the entries below ask for them in declaration
        // order, so the n-th `due()` is the n-th message's.
        let mut slots = SLOTS.into_iter();
        #[expect(
            clippy::arithmetic_side_effects,
            reason = "the intervals are nonzero, which `allocate` asserts at compile time"
        )]
        let mut due =
            || matches!(slots.next(), Some(slot) if time % slot.interval_ms == slot.phase_ms);

        $($(
            $crate::schedule::downlink_entry!(due, $snapshot, $link, $message $(, $ids)?);
        )+)+
    };
}

pub(crate) use {downlink_entry, downlink_entry_slots, downlink_schedule};

#[cfg(test)]
mod tests {
    use super::*;

    /// Both guarantees, re-derived the expensive and obvious way: walk every tick of the cycle and
    /// count, rather than trusting the occupancy map that produced the slots.
    #[expect(
        clippy::arithmetic_side_effects,
        reason = "hand-written test intervals, all small"
    )]
    fn check(intervals: &[u32], slots: &[Slot]) {
        let cycle = intervals.iter().copied().max().unwrap_or(0);

        for tick in 0..cycle {
            let due = slots
                .iter()
                .filter(|s| tick % s.interval_ms == s.phase_ms)
                .count();
            assert!(due <= 1, "tick {tick} of {cycle} has {due} messages due");
        }

        for (interval, slot) in intervals.iter().zip(slots) {
            assert_eq!(
                slot.interval_ms, *interval,
                "slots came back out of declaration order"
            );
            let sends = (0..cycle)
                .filter(|t| t % slot.interval_ms == slot.phase_ms)
                .count();
            assert_eq!(
                sends as u32,
                cycle / interval,
                "{interval} ms message sent {sends} times per {cycle} ms"
            );
        }
    }

    #[test]
    fn the_flight_schedule_fits() {
        // 2 at 10 ms, 7 sensor, 1 battery, 4 at 500, 2 at 1000, 6 tanks and 9 valves at 200.
        let intervals = [
            10, 10, 100, 100, 100, 100, 100, 100, 100, 200, 500, 500, 500, 500, 1000, 1000, 200,
            200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        ];
        check(&intervals, &allocate(intervals));
    }

    #[test]
    fn phases_do_not_depend_on_declaration_order() {
        let mut a = allocate([1000, 100, 200, 100, 500, 100]).map(|s| (s.interval_ms, s.phase_ms));
        let mut b = allocate([100, 100, 100, 200, 500, 1000]).map(|s| (s.interval_ms, s.phase_ms));
        a.sort_unstable();
        b.sort_unstable();
        assert_eq!(a, b);
    }

    /// Insertion sort rather than selection sort: equal intervals keep the order they came in.
    #[test]
    #[expect(
        clippy::indexing_slicing,
        reason = "fixed-length array, literal indices"
    )]
    fn equal_intervals_keep_declaration_order() {
        let slots = allocate([1000, 100, 100, 100]);
        assert!(slots[1].phase_ms < slots[2].phase_ms);
        assert!(slots[2].phase_ms < slots[3].phase_ms);
    }

    #[test]
    #[should_panic = "no tick left to itself"]
    fn an_oversubscribed_schedule_is_rejected() {
        let _ = allocate([1, 2]);
    }

    #[test]
    #[should_panic = "divide the slowest one"]
    fn an_interval_that_does_not_divide_the_slowest_is_rejected() {
        let _ = allocate([300, 1000]);
    }
}
