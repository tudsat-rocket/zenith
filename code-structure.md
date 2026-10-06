# Zenith code structure

Embedded flight computer firmware for high-powered (hybrid or solid motor) rockets, built on
[embassy](https://embassy.dev/) (async embedded framework) for STM32H743VI, speaking MAVLink
(via a custom "rapid" dialect) both directly (Ethernet/USB) and over a compressed LoRa telemetry
protocol. A software-in-the-loop (SITL) build runs the same vehicle logic on Linux for simulation.

## Workspace layout

Cargo workspace, `default-members = ["sitl"]` (so a bare `cargo build/test` targets the host-runnable
simulator, not the embedded firmware).

```
firmware/       embassy/STM32H743 binaries: rocket, gcs, selftest (no_std, hardware-specific)
mission/        hardware-agnostic vehicle/flight logic (no_std, shared by firmware + sitl)
links/          MAVLink command/param/mode "microservices", transport-agnostic (no_std)
telemetry/      compressed LoRa downlink/uplink protocol (packing MAVLink into 16B packets)
state_estimator/complementary-filter (Mahony) + Kalman filter for attitude/position/velocity
params/         core parameter primitives (id, type, encode/decode) - no derive logic
macros/         proc-macro derives (`ParameterGroup`, `ParameterGroups`) used by mission/state_estimator
utils/          tiny shared trait glue (`AnySender`/`AnyReceiver` over channel/pubsub/watch)
sitl/           host build: Linux TAP networking + a hybrid-rocket physics simulation
```

Two build-time "personalities" cut across most of this:
- `hybrid` feature (mission, links, firmware, sitl): hybrid liquid/gas propulsion (fill, vent,
  pressurize, ignite modes; valve/CAN IO board control) vs. a plain solid-motor build.
- `gcs` feature (firmware only): builds the *ground-side* receiver firmware (talks LoRa + relays to
  Ethernet/USB) rather than the flight computer.

## Data flow (rocket firmware, `firmware/src/bin/rocket.rs`)

```
sensors (SPI: IMUs, baros, mag, GPS UART, ADC power)
        │
        ▼
mission::Vehicle::tick()  [1 kHz, high-priority interrupt executor]
  ├─ state_estimator.update()      (AHRS + Kalman, mode-aware gain switching)
  ├─ flight_logic.update()         (autonomous FlightMode transitions)
  ├─ outputs.set_*()               (recovery pyro channels)
  ├─ valves.resolve()              (mode + manual command + Hold-baseline -> ValveMap<ValveState>)
  └─ bus.set_output_image()        (valve/binary-output state -> CAN SDO writes to IO boards)
        │
        ▼
links (Ethernet MAVLink/UDP, USB MAVLink/CDC-ACM, LoRa compressed telemetry)
  each interface runs: commands, params, modes, link_quality "microservice" tasks
  (+ CAN forwarding microservice on Ethernet only)
        │
        ▼
UplinkCommand channel -> consumed once per tick by the main loop
  (SetFlightMode, CommandValve, SetParam; RequestAvailableModes/RequestCanForwarding
   are consumed directly by their own microservice tasks)
```

The CAN bus (`firmware/src/bus.rs` + `bus/mapping.rs` + `bus/pdo_mapping.rs`) is a CANopen-ish PDO
link to external IO boards that host the propellant valves, igniters, cameras and sensors for a
hybrid vehicle. `mapping.rs` is the single hand-maintained table from `mission`'s logical ids
(`ValveId`, `BinaryOutputId`, `PressSensId`, `TempSensId`) to physical `(node_id, index, subindex)`
CAN addresses; `pdo_mapping.rs` encodes/decodes the actual 8-byte CAN frames.

## `mission` crate (the hardware-agnostic core)

- `vehicle.rs` - `Vehicle<Sensors, Outputs, Storage, Bus>`: owns flight mode, `FlightLogic`,
  `ValveController`, `StateEstimator`, and per-tick orchestration (`tick()`).
- `flight_logic.rs` - pure state machine for *autonomous* `FlightMode` transitions (accel-based
  launch/burnout detection, apogee-by-falling detection, altitude-based main deploy, landing
  detection). Debounced via `true_since()`.
- `valves.rs` - `ValveController`: resolves the final commanded state of every valve each tick from
  three layered sources (manual command > mode policy > Hold baseline), including pulse-open
  commands with expiry/restore semantics. Extensively unit-tested.
  `mode_valve_state()` is the per-`FlightMode` truth table (e.g. `Ignite`: pressurization valve
  open immediately, main valve opens after `PropulsionParams::main_valve_delay`).
- `inventory.rs` - id enums for every valve/sensor/output (`ValveId`, `PressSensId`, `TempSensId`,
  `BinaryOutputId`, `TankId`) plus `InventoryMap<Id, T, N>`, a fixed-size array indexed by id with a
  compile-time-checked `ALL` ordering.
- `bus.rs` - the `Bus` trait (`get_input_image`/`set_output_image`) and the input/output "images"
  (`BusInputImage`/`BusOutputImage`) that decouple mission logic from the firmware's CAN
  implementation (and from `sitl`'s simulated bus).
- `mavlink.rs` - `VehicleSnapshot` and all `From`/`Into` conversions to MAVLink ("rapid dialect")
  messages, plus `send_telemetry()`, which drives the compile-time-checked downlink `schedule!`.
- `schedule.rs` - `allocate()`: a const-fn scheduler that phase-offsets a declared set of
  `(message, interval_ms)` pairs so at most one downlink message goes out per tick, computed and
  validated entirely at compile time (asserts if the schedule can't fit). Thoroughly unit-tested.
- `params.rs` - `Params` (aggregates `StateEstimatorParams`, `RecoveryParams`, `PropulsionParams`)
  and `SharedParams`, the live mirror the MAVLink parameter protocol reads/writes.
- `traits.rs` - the hardware-facing traits (`Sensors`, `Outputs`, `TelemetryLink`, `Storage`) that
  `firmware` and `sitl` each implement.

## `firmware` crate (STM32H743 specifics)

- `board.rs` - peripheral init (clocks, SPI buses + shared-bus devices, CAN, Ethernet, USB, LoRa
  radios, ADC, watchdog).
- `bus.rs`/`bus/*` - CAN transport + PDO encode/decode + the logical-id-to-CAN-address mapping
  table (see above).
- `can.rs` - CAN1/CAN2 RX/TX task plumbing over `embassy_sync::pubsub`.
- `links.rs`/`links/*` - per-interface (Ethernet, USB, LoRa) wiring of the shared `links::protocols`
  microservices (commands, params, modes, link_quality) plus CAN-forwarding (Ethernet only) and the
  LoRa-specific compressed telemetry pump.
- `sensors/*` - one driver module per physical sensor (3x IMU, high-G accel, mag, 3x baro, GPS,
  power/ADC).
- `storage.rs`/`storage/w25q.rs` - W25Q NOR flash driver + a `sequential-storage`-backed parameter
  map; a dedicated task owns the flash so the 1kHz main loop never blocks on flash I/O.
- `bin/rocket.rs`, `bin/gcs.rs`, `bin/selftest.rs` - the three firmware images (flight computer,
  ground receiver, hardware self-test).

## `links` and `telemetry` crates

- `links`: transport-agnostic MAVLink "microservices" (`commands`, `params`, `modes`,
  `link_quality`) shared verbatim between Ethernet and USB interfaces (and reused, in compressed
  form, over LoRa). Defines `UplinkCommand`, the internal command enum decoded from MAVLink and fed
  to `Vehicle`.
- `telemetry`: the LoRa link protocol - fixed-size (16B) HMAC'd packets, FHSS frequency hopping
  derived from a shared binding phrase, packing/unpacking between MAVLink messages and the compact
  wire format (`messages/downlink.rs`, `messages/uplink.rs`), plus the hopping
  transmitter/receiver state machines (`trx/`).

## `state_estimator`, `params`, `macros`

- `state_estimator`: Mahony AHRS for orientation + a 9-state (pos/vel/accel) Kalman filter fusing
  barometer, accelerometer and (when reliable) GPS, with mode-aware tuning (e.g. ignoring
  accelerometer for orientation during `Burn`, transonic barometer noise inflation, dual-IMU
  high-G switchover).
- `params`: the core `ParamValue`/`ParamId`/`ParameterField`/`ParameterGroup` types - bytewise
  MAVLink encoding so integer parameters survive exactly.
- `macros`: `#[derive(ParameterGroup)]`/`#[derive(ParameterGroups)]` proc-macros that generate the
  above from `#[param(id, name, default)]`-annotated struct fields (including multi-slot composites
  like `Vector3<f32>` -> `_X`/`_Y`/`_Z`).

## `sitl` crate

Host (Linux) build of the same `mission::Vehicle`, with `StdSensors`/`StdOutputs`/`MemoryStorage`
implementations and (under `hybrid`) a from-scratch propulsion physics model
(`simulation/hybrid/{tank,two_phase,fluid,valves}.rs`: N2 pressurant tank, two-phase N2O oxidizer
tank, valve conductance/travel-time model, chamber pressure response) plus generic 3-DOF flight
physics (`simulation/physics.rs`) and battery model. `networking.rs` broadcasts real MAVLink over
UDP so any ground station can connect. Has a real test suite (`tests/*.rs`) exercising ignition,
outputs, params, state machine and telemetry end-to-end.

## Cross-cutting conventions

- Time is `Wrapping<u32>` milliseconds almost everywhere outside `embassy_time::Instant`, since
  firmware code needs wraparound-safe arithmetic without embassy's `Instant` (host/no_std parity).
- `#[derive(ParameterGroup)]` + a stable `u16` id is the only way parameters are declared; the
  MAVLink name, defaults, storage key and flash round-trip all fall out of that one attribute.
- `InventoryMap<Id, T, N>` is the standard way to store "one value per component"; `Id::ALL`'s
  order is checked at compile time and again in unit tests.
- Clippy runs `pedantic` + `cargo` groups workspace-wide, with a deliberately curated allow-list in
  the root `Cargo.toml`, plus opt-in restriction lints (`unwrap_used`, `indexing_slicing`,
  `arithmetic_side_effects`, etc.) that are then locally re-allowed with a `reason = "..."` at each
  genuine boot-time/const-eval/bounded-math site - the codebase treats "why this lint doesn't apply
  here" as documentation, not noise.
