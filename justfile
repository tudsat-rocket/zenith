target := "thumbv7em-none-eabihf"

default:
    @just --list

# Build and flash the rocket firmware onto the target MCU via probe-rs
flash *args:
    cargo run -p firmware --bin rocket --release --target {{target}} {{args}}

flash-hybrid *args:
    cargo run -p firmware --bin rocket --release --features hybrid --target {{target}} {{args}}

# Build and flash the hardware selftest binary via probe-rs
flash-selftest *args:
    cargo run -p firmware --bin selftest --release --target {{target}} {{args}}

# Build and flash the GCS firmware via probe-rs
flash-gcs *args:
    cargo run -p firmware --bin gcs --release --features gcs --target {{target}} {{args}}

# Run the SITL on the host (solid-rocket build)
sitl-solid *args:
    ./sitl/tap.sh
    cargo run -p sitl --bin sitl --release {{args}}

# Run the SITL on the host (hybrid-rocket build)
sitl-hybrid *args:
    ./sitl/tap.sh
    cargo run -p sitl --bin sitl --release --features hybrid {{args}}

# cargo check across the workspace, with the right target per crate
check:
    cargo check -p firmware --all-features --target {{target}}
    cargo check -p sitl
    cargo check -p sitl --features hybrid
    cargo check -p state_estimator -p telemetry -p utils -p links -p mission --all-features

# cargo test, but with release due to all the state estimator sitl number crunching
test:
    cargo test --release
    cargo test -p sitl --features hybrid --release

# cargo clippy across the workspace, with the right target per crate
clippy:
    cargo clippy -p firmware --all-features --target {{target}}
    cargo clippy -p sitl
    cargo clippy -p sitl --features hybrid
    cargo clippy -p state_estimator -p telemetry -p utils -p links -p mission --all-features

fmt:
    cargo fmt --all

fmt-check:
    cargo fmt --all --check
