#![cfg(not(feature = "hybrid"))]
//! The flight log schedule over a whole simulated flight, through the on-board log codec: nothing
//! may be lost between record and `.tlog`, and each mode has to stay within its share of the
//! flash. The sizes are printed (`--nocapture`) to see what compression buys on real telemetry.

mod common;

use std::collections::HashMap;

use common::{Harness, block_on};
use links::log::{ChunkBuilder, ChunkHeader, Record, decompress_chunk, read_tlog};
use links::{Downlink, UplinkCommand};
use mavio::protocol::{IntoPayload, MavLinkVersion};
use mission::TelemetryLink;
use rapid_dialect::FlightMode;

const MAX_TICKS: u32 = 400_000;
/// A 4 KiB sequential-storage page minus its markers and one item header.
const MAX_CHUNK: usize = 4086;

/// Raw record bytes per second each mode may log at most. Every mode not listed logs nothing.
const BUDGETS: [(FlightMode, usize); 7] = [
    (FlightMode::DetectLaunch, 12_000),
    (FlightMode::Ignite, 12_000),
    (FlightMode::Burn, 22_000),
    (FlightMode::Coast, 22_000),
    (FlightMode::DeployDrogue, 22_000),
    (FlightMode::DeployMain, 22_000),
    (FlightMode::Landed, 1_000),
];

/// Ticks spent in a mode and the raw record bytes logged meanwhile.
#[derive(Default)]
struct ModeUsage {
    ticks: usize,
    bytes: usize,
}

struct LogSink {
    time_ms: u32,
    builder: Box<ChunkBuilder<16384, MAX_CHUNK>>,
    records: usize,
    raw_bytes: usize,
    chunk_bytes: usize,
    chunks: usize,
    tlog_bytes: usize,
    decoded_records: usize,
    mode: FlightMode,
    usage: HashMap<FlightMode, ModeUsage>,
}

impl LogSink {
    fn flush(&mut self, all: bool) {
        while self.builder.is_ready() || (all && !self.builder.is_empty()) {
            let mut chunk = [0; ChunkBuilder::<16384, MAX_CHUNK>::OUT_LEN];
            let len = self.builder.take_chunk(1, &mut chunk).unwrap();
            assert!(len <= MAX_CHUNK);
            let header = ChunkHeader::parse(&chunk).unwrap();

            let mut raw = [0; 16384];
            let raw = decompress_chunk(&chunk[..len], &mut raw).unwrap();
            let mut tlog = vec![0; usize::from(header.tlog_len)];
            let tlog_len = read_tlog(raw, 4, 0, &mut tlog).unwrap();
            assert_eq!(tlog_len, usize::from(header.tlog_len));

            self.chunks += 1;
            self.chunk_bytes += len;
            self.raw_bytes += raw.len();
            self.tlog_bytes += tlog_len;
            self.decoded_records += Record::iter(raw).count();
        }
    }
}

impl TelemetryLink for LogSink {
    fn send_message(&mut self, message: Downlink) {
        let payload = message.message.encode(MavLinkVersion::V2).unwrap();
        self.usage.entry(self.mode).or_default().bytes += 9 + payload.bytes().len();
        assert!(self.builder.push(self.time_ms, &message));
        self.records += 1;
        self.flush(false);
    }

    fn try_recv_command(&mut self) -> Option<UplinkCommand> {
        None
    }
}

#[test]
fn a_logged_flight_survives_the_codec() {
    block_on(async {
        let mut h = Harness::new(None).await;
        let mut sink = LogSink {
            time_ms: 0,
            builder: Box::new(ChunkBuilder::new()),
            records: 0,
            raw_bytes: 0,
            chunk_bytes: 0,
            chunks: 0,
            tlog_bytes: 0,
            decoded_records: 0,
            mode: FlightMode::Idle,
            usage: HashMap::new(),
        };

        // Some time on the pad, then the flight.
        for i in 0..MAX_TICKS {
            if i == 10_000 {
                h.arm();
            }
            h.tick().await;
            let snapshot = h.vehicle.snapshot();
            sink.time_ms = snapshot.time.0;
            sink.mode = snapshot.mode;
            sink.usage.entry(snapshot.mode).or_default().ticks += 1;
            snapshot.send_log(&mut sink);
            // A while on the ground to see the landed schedule.
            if h.mode() == FlightMode::Landed && sink.usage[&FlightMode::Landed].ticks >= 10_000 {
                break;
            }
        }
        sink.flush(true);

        assert_eq!(h.mode(), FlightMode::Landed);
        assert_eq!(sink.decoded_records, sink.records);
        println!(
            "{} records: {} B raw, {} B in {} chunks ({:.2}x), {} B as .tlog ({:.2}x)",
            sink.records,
            sink.raw_bytes,
            sink.chunk_bytes,
            sink.chunks,
            sink.raw_bytes as f64 / sink.chunk_bytes as f64,
            sink.tlog_bytes,
            sink.tlog_bytes as f64 / sink.chunk_bytes as f64,
        );

        for (mode, usage) in &sink.usage {
            let rate = usage.bytes * 1000 / usage.ticks;
            println!("{mode:?}: {} s, {rate} B/s raw", usage.ticks / 1000);
            let budget = BUDGETS
                .iter()
                .find(|(m, _)| m == mode)
                .map_or(0, |(_, budget)| *budget);
            assert!(
                rate <= budget,
                "{mode:?} logs {rate} B/s, over its {budget}"
            );
        }
    });
}
