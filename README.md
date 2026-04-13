zenith
======

Embedded flight control firmware for high-powered rockets, using embassy on STM32H743VI. Speaks MAVLink.

# Building & Running

## Dependencies

- Install [Rust](https://rustup.rs/)
- Install [`just`](https://github.com/casey/just): `cargo install just`

## SITL (Software in the Loop)

Run the code (or as much as possible of it) on a regular (Linux for now) system:

```
just sitl
```

This will attempt to run `sitl/tap.sh` which uses sudo, so it may prompt for your password. This is required for an initial setup of a `tuntap` virtual network interface for the simulated firmware to use.

Once running, the SITL binary will broadcast MAVLink packets just like the firmware and you can use any MAVLink ground station software running on the same system and listening on UDP port 14550 to control it.

In the SITL, sensor values are simulated and the rocket flies a simple simulated trajectory (ignition is 5s after entering Armed mode).

## Flashing Hardware

In addition to the other requirements, you need:

- The correct target for Rust: `rustup target add thumbv7em-none-eabihf`
- [`probe-rs`](https://probe.rs/docs/getting-started/installation/)

To build and flash the firmware, run:

- `just flash` for the FC firmware
- `just flash-gcs` for the GCS firmware
- `just flash-selftest` for a hardware self-test firmware
