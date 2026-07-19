target := "thumbv7em-none-eabihf"

default:
    @just --list

# Build and flash the rocket firmware onto the target MCU via probe-rs
flash-solid *args:
    cargo build -p firmware --bin rocket --release --target {{target}} {{args}}
    probe-rs run --chip STM32H743VITx --catch-hardfault --always-print-stacktrace --log-format '{L} {m:white} {s}' target/thumbv7em-none-eabihf/release/rocket

flash-hybrid *args:
    cargo build -p firmware --bin rocket --release --features hybrid --target {{target}} {{args}}
    probe-rs run --chip STM32H743VITx --catch-hardfault --always-print-stacktrace --log-format '{L} {m:white} {s}' target/thumbv7em-none-eabihf/release/rocket

# Build and flash the hardware selftest binary via probe-rs
flash-selftest *args:
    cargo build -p firmware --bin selftest --release --target {{target}} {{args}}
    probe-rs run --chip STM32H743VITx --catch-hardfault --always-print-stacktrace --log-format '{L} {m:white} {s}' target/thumbv7em-none-eabihf/release/selftest

# Build and flash the GCS firmware via probe-rs
flash-gcs *args:
    cargo build -p firmware --bin gcs --release --features="gcs" --target {{target}} {{args}}
    probe-rs run --chip STM32H743VITx --catch-hardfault --always-print-stacktrace --log-format '{L} {m:white} {s}' target/thumbv7em-none-eabihf/release/gcs

# Run the SITL on the host (solid-rocket build)
sitl-solid *args:
    ./sitl/tap.sh
    cargo run -p sitl --bin sitl --release --no-default-features {{args}}

# Run the SITL on the host (hybrid-rocket build)
sitl-hybrid *args:
    ./sitl/tap.sh
    cargo run -p sitl --bin sitl --release --features hybrid {{args}}

cargo-everywhere *args:
    cargo {{args}} -p firmware --bin rocket --target {{target}}
    cargo {{args}} -p firmware --bin rocket --features hybrid --target {{target}}
    cargo {{args}} -p firmware --bin selftest --target {{target}}
    cargo {{args}} -p firmware --bin gcs --features gcs --target {{target}}
    cargo {{args}} -p sitl --no-default-features
    cargo {{args}} -p sitl --features hybrid
    cargo {{args}} -p state_estimator -p telemetry -p utils -p links -p mission --all-features

check:
    @just cargo-everywhere check

clippy:
    @just cargo-everywhere clippy

# cargo test, but with release due to all the state estimator sitl number crunching
test:
    cargo test --release
    cargo test -p sitl --no-default-features --release
    cargo test -p sitl --features hybrid --release

fmt:
    cargo fmt --all

fmt-check:
    cargo fmt --all --check

suite:
    @just check
    @just test
    @just fmt-check
    @just clippy
