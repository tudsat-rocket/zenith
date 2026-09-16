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
just sitl-hybrid
```

This will attempt to run `sitl/tap.sh` which uses sudo, so it may prompt for your password. This is required for an initial setup of a `tuntap` virtual network interface for the simulated firmware to use.

Once running, the SITL binary will broadcast MAVLink packets just like the firmware and you can use any MAVLink ground station software running on the same system and listening on UDP port 14550 to control it.

In the SITL, sensor values are simulated and the rocket flies a simple simulated trajectory (ignition is 5s after entering Armed mode).

### Off-nominal scenarios

For operator training, the SITL can simulate failures. Each is a cargo feature and they can be combined, e.g. `just sitl-hybrid --features fault-drogue-failure,fault-downlink-loss`:

| Feature | Behaviour |
| --- | --- |
| `fault-engine-underperformance` | Engine only reaches 55% of its nominal propellant mass flow: lower chamber pressure and thrust, longer burn, lower apogee |
| `fault-sensor-dropout` | 1-3 random sensors (IMUs, baros, mag, GPS, power, and on hybrid builds tank pressure/temperature sensors) stop reporting at random times within 60s after arming |
| `fault-downlink-loss` | 50% of downlink packets are lost |
| `fault-drogue-failure` | Drogue deployment is commanded, but the chute never comes out; the vehicle falls ballistic until the main opens |
| `fault-random` | Rolls a random combination of the above at startup, 25% of the time a fully nominal flight |

The active scenario is logged at startup and stays fixed until the SITL is restarted.

## Flashing Hardware

In addition to the other requirements, you need:

- The correct target for Rust: `rustup target add thumbv7em-none-eabihf`
- [`probe-rs`](https://probe.rs/docs/getting-started/installation/)

To build and flash the firmware, run:

- `just flash-hybrid` for the FC firmware
- `just flash-gcs` for the GCS firmware
- `just flash-selftest` for a hardware self-test firmware
