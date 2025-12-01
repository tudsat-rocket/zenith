//! Datatypes and protocol specification for our telemetry protocol
//!
//! # Overview
//!
//! The goal of this protocol is to present a MAVLink-compatible interface at the receiver,
//! comparable to a direct one, using a LoRa phy-based long range wireless link.
//!
//! The protocol should be usable for different vehicles or MAVLink systems, including things like
//! drones, stationary equipment or other systems. The focus is on airborne ones though (rockets
//! and drones).
//!
//!     ____________________________                      ____________________________
//!    |                            |                    |                            |
//!    |   Rocket                   |                    |                 Receiver   |
//!    |               ___________  | \|/    LoRa    \|/ |  ___________               |
//!    |              |           | |  |   ~~~  ~~~   |  | |           |              |
//!    |      +------>| Telemetry |----+   ~868 Mhz   +----| Telemetry |-------+      |
//!    |  ____|____   |___________| |       500 kHz      | |___________|   ____v____  |
//!    | |         |                |                    |                |         | |
//!    | | MAVLink |                |                    |                | MAVLink | |
//!    | |_________|   ___________  |                    |  ___________   |_________| |
//!    |      |       |           | |                    | |           |       |      |
//!    |      +------>| Ethernet  |------+          +------| Ethernet  |<------+      |
//!    |              |___________| |    |          |    | |___________|              |
//!    |____________________________|    |          |    |____________________________|
//!                                      |          |
//!                               _______v__________v_______
//!                              |                          |
//!                              | Ground Station Software/ |
//!                              |   Switches/Relays/etc.   |
//!                              |__________________________|
//!
//!
//! To accomplish this with our limited bandwidth, we restrict ourselves to a subset of MAVLink
//! messages and package them in bespoke packets of uniform size, optimized for RF transmission.
//!
//! We also rely on the telemetry receiver to implement parts of the MAVLink connection. For
//! instance, "metadata" on the vehicle, such as the type of the vehicle, the firmware used, the
//! available flight modes, how many cells the battery pack has, can all be known by the receiver
//! in advance, and the receiver can reply to queries for this data without involving the vehicle
//! or LoRa at all.
//!
//! In order to still have the ability to use the same receiver for multiple vehicles without
//! having to recompile constantly, we define "profiles" of vehicles, such as:
//!
//!     - Generic solid motor rocket, single parachute or dual deployment, no CAN bus stuff
//!         - This could be used for many smaller projects without modification
//!     - Super-special hybrid/biliquid EuRoC project
//!         - This gets its own profile, includes information on the expected component IDs, the
//!             available modes for these components
//!     - PX4 or ArduCopter quadcopter
//!     - ArduPlane fixed wing
//!     - Stationary equipment (such as a filling station)
//!
//! In theory, the profile (or the information contained therein) could be manually configured in
//! the ground station software or the receiver, but instead vehicles identity themselves in their
//! transmissions. This is less error-prone (there is no scenario where a receiver thinks it's
//! talking to a filling station, but it's actually talking to a rocket), less annoying to
//! configure and there is one less thing to worry about when using the receiver to talk to
//! multiple vehicles at the same time or in short succession.
//!
//! Since transmissions are translated to regular MAVLink on either hand, these profiles _can_
//! change without affecting past log files, and by giving special vehicles with frequent changes
//! their own profile, we can implement these changes without affecting other systems. The team
//! working on these profiles just have to update their own hardware, and everyone else does not
//! notice.
//!
//! # Downlink
//!
//! ## Phy
//!
//! We use LoRa transceivers in the 868MHz band, from 863 - 870 MHz. In order to maximize the data
//! rate we use the largest bandwidth possible (500kHz), giving us 14 channels. We also use a low
//! spreading factor (SF), 7. This limits the duration of our packets, giving us a higher data
//! rate.
//!
//! In order to avoid the need for a LoRa packet header, we need every message to have the same
//! length. Generally, the smaller the better, since the larger the packet the higher the chance
//! for a collision with another transmission.
//!
//! 16 bytes seems to be enough to transport a reasonable amount of data in each packet, with the
//! ability to translate the necessary information contained in various MAVLink message into a
//! single comparable compressed packet.
//!
//! For a given bandwidth, SF, and payload size we can calculate the packet time-on-air for each
//! coding rate:
//!
//!
//!    T_symbol = 2^SF / BW = 0.256ms
//!
//!    n_preamble = 8 TODO
//!    T_preamble = (n_preamble + 4.25) * T_symbol = 3.136ms
//!
//!    CR (coding rate) E {1,2,3,4}
//!    bytes_payload = 16
//!
//!    n_payload = 8 + max(ceil((8*bytes_payload - 4*SF + 24)/(4*SF)) * (CR+4), 0)
//!    T_payload = n_payload * T_symbol
//!
//!    T_packet = T_preamble + T_payload
//!
//!    | CR | n_payload | T_payload | T_packet  | duty cycle @ 16ms interval |
//!    +----+-----------+-----------+-----------+----------------------------+
//!    |  1 |        33 |     8.448 | 11.584 ms |                      72.8% |
//!    |  2 |        38 |     9.728 | 12.864 ms |                      80.4% |
//!    |  3 |        43 |    11.008 | 14.144 ms |                      88.4% |
//!    |  4 |        48 |    12.288 | 15.424 ms |                      96.4% |
//!
//!
//! ## FHSS
//!
//! To allow multiple flight computers or other devices in the same band without having to
//! coordinate frequencies, we use frequency hopping spread spectrum (FHSS), where both parties
//! know a predefined or calculated hopping sequence and we hop through the entire band.
//!
//!
//!   OO  Flight Computer 1    XX  Flight Computer 2
//!
//!   863                                                                   870 MHz
//!
//!    |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 |
//!    +----+----+----+----+----+----+----+----+----+----+----+----+----+----+
//!  t | OO |    |    |    |    |    |    |    |    |    | XX |    |    |    | ^
//!    |    |    | XX |    |    |    | OO |    |    |    |    |    |    |    | |
//!  | |    |    |    |    |    | XX |    |    |    |    |    |    |    | OO | | sequence
//!  v |    |    |    |    |    |    |    |    |    | OO |    | XX |    |    | | repeats
//!    ....................................................................... |
//!    |    |    |    |    | OO |    |    |    | XX |    |    |    |    |    | v
//!    | OO |    |    |    |    |    |    |    |    |    | XX |    |    |    |
//!    |    |    | XX |    |    |    | OO |    |    |    |    |    |    |    |
//!
//!
//! To enable this hopping, each downlink packet includes a time. After listening on random
//! frequencies until a single packet is received, the receiver can now determine the position in
//! the hopping sequence using the time contained in the received packet and start following along.
//! Since this time can overflow, the sequence must repeat, and it may repeat earlier.
//!
//! We derive our hopping sequence from a binding phrase, which is hashed and used to seed a PRNG.
//! This is similar to other comparable protocols, including e.g. ExpressLRS. The binding phrase is
//! not considered to be a secret.
//!
//! ## Downlink Packet Format
//!
//! Each downlink packet consists of:
//!
//!
//!   | time  | msg id |   payload   | HMAC |
//!
//!      |        |           |         |
//!      +--------|-----------|---------|-- 11b time
//!               +-----------|---------|--  5b message identifier
//!                           +---------|-- 14B message payload, depends on identifier
//!                                     +-- 16b HMAC calculated over everything else
//!
//!
//!   - time: time since boot and/or message counter, depending on your point of view.
//!
//!       Time is encoded as 11bits of time in 16*ms (time_in_ms >> 4). Since we assume our message
//!       interval to be a multiple of 16ms (see below), so we can recover full time in ms by
//!       assuming 4 bits of zeros, and the value should cleanly overflow every 32.8 seconds.
//!
//!   - message identifier: allows identifying the content of the message payload.
//!
//!       Since we have just 5 bits for this, we are limited to just 32 possible downlink messages.
//!       However, we have some options:
//!           - These telemetry messages are short-lived, since on reception they are turned into
//!               actual MAVLink messages. This means we can change these IDs fairly flexibly compared
//!               to MAVLink messages that may be stored in some telemetry log and will have to be
//!               parsable long into the future
//!           - Similar to actual MAVLink and dialects, different vehicles could use different sets
//!               of message IDs, as long as all vehicles transmit the heartbeat message,
//!               which allows identifying the profile of vehicle. This is not something we do at the
//!               moment.
//!
//!   - payload: 14 bytes of actual payload, depending on the message identifier.
//!
//!   - HMAC: 16 bits of HMAC calculated over the other 14 bytes using a key known by both the
//!       vehicle and the receiver. This provides some integrity and authenticity protection, and it
//!       also allows us to differentiate between multiple transmitters in the same band. If
//!       different keys are used, the HMAC will only match with one of them.
//!
//! ## The Heartbeat
//!
//! Similar to MAVLink, we also have a heartbeat message. The only requirement regarding which
//! messages to transmit is that vehicles should frequently (>=2Hz) broadcast heartbeat messages.
//! This message also contains the identifier of the profile of the vehicle, allowing the receiver
//! to identify the type of vehicle it's talking to.
//!
//! Since all our packets are the same size, simple mode and profile information would not be
//! enough to fill a full packet, so we also include altitude, velocity and attitude information in
//! this packet, since this is the majority of the data transmitted that is both very important and
//! frequently changes. For airborne systems, this should be the most common message transmitted.
//!
//! ## Packing & Unpacking
//!
//! Since we want constant packet sizes, we sometimes have to combine multiple MAVLink messages
//! into one LoRa packet, so we don't waste any of our payload bytes. This means we are effectively
//! converting tuples of MAVLink messages into a single packet ("packing"), and recovering multiple
//! MAVLink messages on reception ("unpacking").
//!
//! As an example, the MAVLink HEARTBEAT message mostly contains mode information, which we can
//! compress to way less than 14 bytes. So we combine it with some altitude and velocity
//! information from LOCAL_POSITION_NED as well as our attitude from the ATTITUDE message.
//!
//! The receiver may even recover more MAVLink messages than were used in the construction of the
//! message. For instance, our combined messages of (HEARTBEAT, LOCAL_POSITION_NED, ATTITUDE) are
//! enough to recover most of the information contained in the VFR_HUD message as well. This gives
//! the system more compatibility with MAVLink ground stations using different messages without any
//! additional RF bandwidth.
//!
//!   (HEARBEAT, LOCAL_POSITION_NED, ATTITUDE) -> [packet] -> (HB, L_P_N, ATT, ALTITUDE, VFR_HUD)
//!
//!
//! # Uplink
//!
//! TODO

#![no_std]

/// Interval between messages in ms.
///
/// Hardcoded right now, may become configurable.
///
/// This must be a power of two, meaning our message frequencies won't be round numbers:
///
///   | Interval | Packet Rate | Data Rate (with 16B packets) | Duty Cycle (CR=1) |
///   +----------+-------------+------------------------------+-------------------+
///   | 16       |    62.5  Hz |                   1000   B/s |             72.8% |
///   | 32       |    31.25 Hz |                    500   B/s |             36.4% |
///   | 64       |    15.63 Hz |                    250   B/s |             18.2% |
///   | 128      |     7.81 Hz |                    125   B/s |              9.1% |
///   | 256      |     3.91 Hz |                     62.5 B/s |              4.6% |
///
/// This ensures that our message sequence aligns to the overflows of our time value.
pub const DOWNLINK_MESSAGE_INTERVAL_MS: u16 = 32;

pub const UPLINK_HOP_INTERVAL_MS: u16 = 128;

pub mod config;
pub mod downlink;
pub mod uplink;
