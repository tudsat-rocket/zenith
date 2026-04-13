target := "thumbv7em-none-eabihf"
chip := "STM32H743VITx"

export CARGO_TARGET_THUMBV7EM_NONE_EABIHF_RUNNER := "probe-rs run --chip " + chip + " --catch-hardfault --always-print-stacktrace --log-format '{L} {m:white} {s}'"
export CARGO_TARGET_THUMBV7EM_NONE_EABIHF_RUSTFLAGS := "-C linker=flip-link -C link-arg=-Tlink.x -C link-arg=-Tdefmt.x -C link-arg=--nmagic"
export DEFMT_LOG := "info"

default:
    @just --list

# Build and flash the rocket firmware onto the target MCU via probe-rs
flash *args:
    cargo run -p firmware --bin rocket --release --target {{target}} {{args}}

# Build and flash the hardware selftest binary via probe-rs
flash-selftest *args:
    cargo run -p firmware --bin selftest --release --target {{target}} {{args}}

# Build and flash the GCS firmware via probe-rs
flash-gcs *args:
    cargo run -p firmware --bin gcs --release --features gcs --target {{target}} {{args}}

# Run the SITL / std version of the firmware on the host
sitl *args:
    ./sitl/tap.sh
    cargo run -p sitl --bin sitl {{args}}

# Alias for `sitl`
std *args: (sitl args)

# cargo check across the workspace, with the right target per crate
check:
    cargo check -p firmware --all-features --target {{target}}
    cargo check -p sitl --all-features
    cargo check -p state_estimator -p telemetry -p utils -p links -p mission --all-features

# cargo clippy across the workspace, with the right target per crate
clippy:
    cargo clippy -p firmware --all-features --target {{target}}
    cargo clippy -p sitl --all-features
    cargo clippy -p state_estimator -p telemetry -p utils -p links -p mission --all-features

fmt:
    cargo fmt --all

fmt-check:
    cargo fmt --all --check
