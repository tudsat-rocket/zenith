#!/usr/bin/env bash
# use this as rust-analyzer's check command, else there will be a lot of project errros

# Check workspace (excluding firmware) for host target
cargo clippy \
  --message-format=json \
  --workspace \
  --exclude firmware \
  --all-features
  # --target x86_64-unknown-linux-gnu \

# Check firmware for cortex-m target
cargo clippy \
  --message-format=json \
  --target thumbv7em-none-eabihf \
  --manifest-path firmware/Cargo.toml \
  --all-features
