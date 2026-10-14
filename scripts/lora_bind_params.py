#!/usr/bin/env python3
"""Configure the LoRa binding phrase and HMAC key on a vehicle or ground modem.

Talks directly to the wired MAVLink endpoint (Ethernet/UDP or USB-CDC) of a
`rocket` or `gcs` firmware binary and sets its LORA_*/GCS_* parameters by
name, bypassing PARAM_REQUEST_LIST (which never completes for the ground
modem alone -- it waits on the vehicle's own params too, see
telemetry/src/params.rs::forward).

Ethernet is asymmetric on the firmware side (firmware/src/links/interfaces/
ethernet.rs): a device listens for commands on UDP port 14551 but always
*broadcasts* its replies to port 14550, regardless of who asked and ignoring
the actual sender address. So this never uses a single bidirectional UDP
socket for ethernet -- it sends on one socket (to :14551) and listens on
another (0.0.0.0:14550) for the reply.

With --ethernet and no IP, it first listens briefly for the target device's
own broadcast traffic to learn its real IP (matched by MAVLink system id),
then unicasts to that instead of blindly broadcasting the command. This
matters because a plain broadcast send isn't reliable on a multi-homed
machine (e.g. Ethernet + Wi-Fi): pymavlink's UDP sender doesn't bind to a
specific local address, so the OS's routing table -- not necessarily the
interface actually wired to the device -- decides where the packet goes
out, while *receiving* a broadcast has no such ambiguity. If nothing is
heard from the device within a few seconds, it falls back to a genuine
broadcast send anyway. --ethernet <ip> skips discovery and unicasts there
directly.

USB/serial, in contrast, is a single bidirectional connection as usual --
pass a device path to --serial. (USB is known to have other firmware-side
issues independent of this script; prefer --ethernet.)

Values are exchanged "bytewise": the firmware reinterprets the IEEE-754
float wire field as raw bits for integer params rather than casting it, and
advertises this via MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE. This
script always does that encoding explicitly -- do not use a generic GUI
param editor for these parameters unless you've confirmed it respects that
capability flag.

LORA_BIND_*/LORA_KEY_*/GCS_BIND_*/GCS_KEY_* are all `reboot_required`: they
take effect only on the next boot. Power-cycle the device after applying.

Requires: pip install pymavlink

Examples:
  # Offline: just see what a phrase/key would produce
  lora_bind_params.py bind-words "my phrase"
  lora_bind_params.py gen-key

  # Set the binding phrase and a key on the vehicle, over Ethernet
  lora_bind_params.py apply --ethernet --target vehicle \\
      --phrase "my phrase" --key-hex 00112233445566778899aabbccddeeff

  # Same key/phrase on the ground modem, over Ethernet, targeting a known IP
  lora_bind_params.py apply --ethernet 192.168.1.50 --target gcs \\
      --phrase "my phrase" --key-hex 00112233445566778899aabbccddeeff

  # Over USB instead
  lora_bind_params.py apply --serial /dev/ttyACM0 --target vehicle \\
      --phrase "my phrase" --key-hex 00112233445566778899aabbccddeeff

  # Read a single parameter back (bytewise-decoded)
  lora_bind_params.py read --ethernet --target vehicle LORA_BIND_0

  # Diagnose "nothing comes back at all": is anything reaching this machine
  # on :14550, independent of any PARAM_SET round trip?
  lora_bind_params.py listen
"""

import argparse
import os
import secrets
import socket
import struct
import sys
import time

# pymavlink encodes outgoing messages as MAVLink v1 (0xFE) by default; the
# firmware only ever builds/expects v2 frames (0xFD -- every Frame::builder()
# in the Rust code is `.version(V2)`, no v1 path). This must be set before
# `from pymavlink import mavutil` is first imported anywhere below, since
# which dialect module (and wire version) it binds to is decided at import
# time. Without this, every PARAM_SET/PARAM_REQUEST_READ this script sends
# is in a format the firmware likely never recognizes, regardless of
# whether the packet even reaches it over the network.
os.environ["MAVLINK20"] = "1"

ROCKET_SYSTEM_ID = 0x04
GCS_SYSTEM_ID = 0x06
SELF_COMPONENT_ID = 0x01

PREFIX = {"vehicle": "LORA", "gcs": "GCS"}
TARGET_SYSTEM_ID = {"vehicle": ROCKET_SYSTEM_ID, "gcs": GCS_SYSTEM_ID}

_UNSET = object()  # "--ethernet wasn't passed at all", distinct from "--ethernet with no IP"


# ---------------------------------------------------------------------------
# SipHash-2-4 with an all-zero 16-byte key, matching telemetry::config::hop_seed
# ---------------------------------------------------------------------------

MASK64 = (1 << 64) - 1


def _rotl(x: int, b: int) -> int:
    return ((x << b) | (x >> (64 - b))) & MASK64


def _sipround(v0, v1, v2, v3):
    v0 = (v0 + v1) & MASK64
    v1 = _rotl(v1, 13) ^ v0
    v0 = _rotl(v0, 32)
    v2 = (v2 + v3) & MASK64
    v3 = _rotl(v3, 16) ^ v2
    v0 = (v0 + v3) & MASK64
    v3 = _rotl(v3, 21) ^ v0
    v2 = (v2 + v1) & MASK64
    v1 = _rotl(v1, 17) ^ v2
    v2 = _rotl(v2, 32)
    return v0, v1, v2, v3


def siphash24(data: bytes, key: bytes = b"\x00" * 16) -> int:
    k0 = int.from_bytes(key[0:8], "little")
    k1 = int.from_bytes(key[8:16], "little")
    v0 = 0x736F6D6570736575 ^ k0
    v1 = 0x646F72616E646F6D ^ k1
    v2 = 0x6C7967656E657261 ^ k0
    v3 = 0x7465646279746573 ^ k1

    length = len(data)
    nblocks = length // 8
    for i in range(nblocks):
        m = int.from_bytes(data[i * 8 : i * 8 + 8], "little")
        v3 ^= m
        for _ in range(2):
            v0, v1, v2, v3 = _sipround(v0, v1, v2, v3)
        v0 ^= m

    tail = data[nblocks * 8 :]
    b = (length & 0xFF) << 56 | int.from_bytes(tail, "little")
    v3 ^= b
    for _ in range(2):
        v0, v1, v2, v3 = _sipround(v0, v1, v2, v3)
    v0 ^= b

    v2 ^= 0xFF
    for _ in range(4):
        v0, v1, v2, v3 = _sipround(v0, v1, v2, v3)

    return v0 ^ v1 ^ v2 ^ v3


def _selftest() -> None:
    # telemetry/src/config.rs: assert_eq!(hop_seed("schinken"), 0x53bd_d841_8658_7bcb);
    expected = 0x53BDD84186587BCB
    got = siphash24(b"schinken")
    if got != expected:
        sys.exit(
            f"siphash24 self-test failed (got {got:#018x}, expected {expected:#018x}) "
            "-- the firmware's hop_seed() may use a different SipHash variant than "
            "this script implements. Do not trust the computed bind words until "
            "this is fixed."
        )


_selftest()


def hop_seed(phrase: str) -> int:
    """telemetry::config::hop_seed: SipHash-2-4 of the phrase, zero key."""
    return siphash24(phrase.encode("ascii"))


def bind_words(phrase: str) -> tuple[int, int]:
    """(BIND_0, BIND_1): low word first, per mission::params::hop_seed."""
    seed = hop_seed(phrase)
    return seed & 0xFFFFFFFF, (seed >> 32) & 0xFFFFFFFF


def key_words(key: bytes) -> tuple[int, int, int, int]:
    """KEY_0..3 from 16 raw bytes, little-endian per mission::params::hmac_key."""
    if len(key) != 16:
        raise ValueError("key must be exactly 16 bytes (32 hex chars)")
    return struct.unpack("<4I", key)


# ---------------------------------------------------------------------------
# Bytewise param wire encoding (MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE)
# ---------------------------------------------------------------------------


def encode_u32(value: int) -> float:
    return struct.unpack("<f", struct.pack("<I", value & 0xFFFFFFFF))[0]


def decode_u32(value: float) -> int:
    return struct.unpack("<I", struct.pack("<f", value))[0]


def _param_id(raw) -> str:
    if isinstance(raw, bytes):
        raw = raw.decode("ascii", errors="ignore")
    return raw.rstrip("\x00")


# ---------------------------------------------------------------------------
# MAVLink I/O
# ---------------------------------------------------------------------------

ETHERNET_SEND_PORT = 14551  # a device's own bind()
ETHERNET_RECV_PORT = 14550  # where every device broadcasts its replies


def _require_pymavlink():
    try:
        from pymavlink import mavutil

        return mavutil
    except ImportError:
        sys.exit("pymavlink is required for this command: pip install pymavlink")


class EthernetLink:
    """Pairs a sender (-> device:14551) with a listener (<- 0.0.0.0:14550),
    matching the firmware's asymmetric UDP socket. Exposes the same
    `.mav`/`.recv_match` surface a plain mavutil connection does, so the
    rest of the script doesn't need to care which transport it's using.
    """

    def __init__(self, ip: str | None):
        mavutil = _require_pymavlink()
        self._send = (
            mavutil.mavlink_connection(f"udpout:{ip}:{ETHERNET_SEND_PORT}")
            if ip
            else mavutil.mavlink_connection(f"udpbcast:255.255.255.255:{ETHERNET_SEND_PORT}")
        )
        self._recv = mavutil.mavlink_connection(f"udpin:0.0.0.0:{ETHERNET_RECV_PORT}")

    @property
    def mav(self):
        return self._send.mav

    def recv_match(self, **kwargs):
        return self._recv.recv_match(**kwargs)


def _discover_ip(target_system: int, timeout: float) -> str | None:
    """The real IP of `target_system`, learned from a raw MAVLink v2 frame
    it broadcasts on :14550, matched by the sysid byte in the frame header.

    Done with our own socket rather than through a pymavlink connection:
    pymavlink's `udpin:`/server mode only records peer addresses into an
    unordered `clients` set (`last_address` is set in broadcast-receive
    mode only, not server mode), so there's no reliable way to recover
    which address a given *parsed* message came from. Reading the raw
    datagram ourselves keeps the (data, addr) pairing intact. Runs before
    any pymavlink connection claims port 14550, so there's no conflict.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", ETHERNET_RECV_PORT))
    try:
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            sock.settimeout(remaining)
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                return None
            # MAVLink v2 frame: 0xFD magic (byte 0), sysid is byte 5.
            if len(data) > 5 and data[0] == 0xFD and data[5] == target_system:
                return addr[0]
    finally:
        sock.close()


def connect(args):
    if args.ethernet is not _UNSET:
        ip = args.ethernet
        if ip is None:
            target_system = TARGET_SYSTEM_ID[args.target]
            print(
                f"Ethernet: listening on 0.0.0.0:{ETHERNET_RECV_PORT} to find the "
                f"device's real IP (sysid={target_system}) before sending anything "
                "-- a blind broadcast send can go out the wrong interface on a "
                "multi-homed machine..."
            )
            ip = _discover_ip(target_system, timeout=5.0)
            if ip:
                print(f"  found it at {ip}; sending there instead of broadcasting")
            else:
                print(
                    f"  heard nothing from system {target_system} in 5s; falling "
                    "back to a broadcast send, which may well not reach the "
                    "device (run `listen` separately to confirm it's actually "
                    "on the air)"
                )
        else:
            print(f"Ethernet: sending to {ip}:{ETHERNET_SEND_PORT}")
        print(f"Listening on 0.0.0.0:{ETHERNET_RECV_PORT} for replies")
        return EthernetLink(ip)

    mavutil = _require_pymavlink()
    # No wait_heartbeat(): the gcs binary's wired ports aren't guaranteed to
    # emit MAVLink HEARTBEATs, and we don't need one to send PARAM_SET/READ.
    return mavutil.mavlink_connection(args.serial)


def param_set_u32(master, target_system: int, name: str, value: int, timeout: float = 3.0) -> bool:
    from pymavlink import mavutil

    master.mav.param_set_send(
        target_system,
        SELF_COMPONENT_ID,
        name.encode("ascii"),
        encode_u32(value),
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=deadline - time.time())
        if msg is None:
            break
        if _param_id(msg.param_id) != name:
            continue
        stored = decode_u32(msg.param_value)
        wanted = value & 0xFFFFFFFF
        if stored != wanted:
            print(f"  WARNING: {name} echoed {stored:#010x}, expected {wanted:#010x}")
        else:
            print(f"  {name} = {stored:#010x} (confirmed)")
        return True
    print(f"  WARNING: no PARAM_VALUE ack for {name} within {timeout}s")
    return False


def param_read_u32(master, target_system: int, name: str, timeout: float = 3.0):
    master.mav.param_request_read_send(target_system, SELF_COMPONENT_ID, name.encode("ascii"), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=deadline - time.time())
        if msg is None:
            break
        if _param_id(msg.param_id) == name:
            return decode_u32(msg.param_value)
    return None


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _add_transport_args(parser: argparse.ArgumentParser) -> None:
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--ethernet",
        nargs="?",
        const=None,
        default=_UNSET,
        metavar="IP",
        help=(
            "talk over Ethernet/UDP. With no IP, sends broadcast to :14551 "
            "(use when the device's DHCP-assigned IP is unknown); with an "
            "IP, unicasts the send side to it. Either way, replies are read "
            "by listening on 0.0.0.0:14550, since the device always "
            "broadcasts its replies there regardless of who asked."
        ),
    )
    group.add_argument(
        "--serial",
        metavar="PORT",
        help="talk over USB/serial, e.g. /dev/ttyACM0 (has known firmware-side issues)",
    )


def cmd_bind_words(args) -> None:
    low, high = bind_words(args.phrase)
    print(f"BIND_0 = {low:#010x} ({low})")
    print(f"BIND_1 = {high:#010x} ({high})")


def cmd_gen_key(args) -> None:
    key = secrets.token_bytes(16)
    print(f"key bytes (hex) = {key.hex()}")
    for i, word in enumerate(key_words(key)):
        print(f"KEY_{i} = {word:#010x} ({word})")


def cmd_apply(args) -> None:
    prefix = PREFIX[args.target]
    target_system = TARGET_SYSTEM_ID[args.target]
    if args.phrase is None and args.key_hex is None:
        sys.exit("nothing to do: pass --phrase and/or --key-hex")

    master = connect(args)
    print(f"Targeting system {target_system} ({args.target})")

    acked = []
    if args.phrase is not None:
        low, high = bind_words(args.phrase)
        print(f"Setting {prefix}_BIND_0/1 from phrase...")
        acked.append(param_set_u32(master, target_system, f"{prefix}_BIND_0", low))
        acked.append(param_set_u32(master, target_system, f"{prefix}_BIND_1", high))

    if args.key_hex is not None:
        key = bytes.fromhex(args.key_hex)
        print(f"Setting {prefix}_KEY_0..3...")
        for i, word in enumerate(key_words(key)):
            acked.append(param_set_u32(master, target_system, f"{prefix}_KEY_{i}", word))

    if acked and not any(acked):
        print(
            "\nNone of these were acknowledged at all -- that's not specific to "
            f"{prefix}_* params, so it's unlikely to be about whether a flight "
            "computer is connected over LoRa (a direct PARAM_SET to this "
            "device's own params doesn't need one). Run `lora_bind_params.py "
            "listen` to check whether any traffic from the device is reaching "
            "this machine at all; see its output for likely causes (no DHCP "
            "lease on this link, or a broadcast going out the wrong interface "
            "on a multi-homed machine)."
        )
    else:
        print(
            f"\nDone. These parameters only take effect on the next boot -- "
            f"power-cycle the {args.target} device now."
        )


def cmd_read(args) -> None:
    master = connect(args)
    target_system = TARGET_SYSTEM_ID[args.target]
    value = param_read_u32(master, target_system, args.name)
    if value is None:
        print(f"No response for {args.name}")
    else:
        print(f"{args.name} = {value:#010x} ({value})")


def cmd_listen(args) -> None:
    mavutil = _require_pymavlink()
    recv = mavutil.mavlink_connection(f"udpin:0.0.0.0:{ETHERNET_RECV_PORT}")
    print(f"Listening on 0.0.0.0:{ETHERNET_RECV_PORT} for {args.timeout:.0f}s "
          "(every device broadcasts its MAVLink traffic here)...")

    deadline = time.time() + args.timeout
    count = 0
    while time.time() < deadline:
        msg = recv.recv_match(blocking=True, timeout=deadline - time.time())
        if msg is None:
            continue
        count += 1
        print(f"  sysid={msg.get_srcSystem()} compid={msg.get_srcComponent()}  {msg.get_type()}")

    if count == 0:
        print(
            "\nNothing arrived at all -- this points at the network path, not at "
            "whether a flight computer is connected over LoRa (a direct PARAM_SET "
            "to the gcs device's own GCS_* params doesn't need one). Two likely "
            "causes:\n"
            "  1. No DHCP server on this link. The firmware only does DHCP, with "
            "no static/link-local fallback (firmware/src/links/interfaces/"
            "ethernet.rs) -- if you're cabled directly to the device with "
            "nothing handing out an address, it never gets an IP at all and "
            "nothing will get through regardless.\n"
            "  2. A broadcast send on a multi-homed machine (e.g. Ethernet + "
            "Wi-Fi) can go out the wrong interface, since pymavlink's UDP sender "
            "doesn't bind to a specific local address. If DHCP is working, check "
            "the device's leased IP (router's client list / DHCP leases) and "
            "retry with `--ethernet <that-ip>` instead of broadcast.\n"
            "Also sanity-check: does the Ethernet interface you're cabled "
            "through actually show a normal IPv4 address (not empty/169.254.x.x)?"
        )
    else:
        print(f"\n{count} message(s) received -- the network path works.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("bind-words", help="compute BIND_0/BIND_1 from a phrase, offline")
    p.add_argument("phrase")
    p.set_defaults(func=cmd_bind_words)

    p = sub.add_parser("gen-key", help="generate a random 16-byte HMAC key, offline")
    p.set_defaults(func=cmd_gen_key)

    p = sub.add_parser("apply", help="set BIND and/or KEY on a connected vehicle or gcs device")
    _add_transport_args(p)
    p.add_argument("--target", choices=["vehicle", "gcs"], required=True)
    p.add_argument("--phrase", help="binding phrase; sets BIND_0/BIND_1")
    p.add_argument("--key-hex", help="32 hex chars (16 bytes); sets KEY_0..3")
    p.set_defaults(func=cmd_apply)

    p = sub.add_parser("read", help="read back one parameter by name (bytewise u32)")
    _add_transport_args(p)
    p.add_argument("--target", choices=["vehicle", "gcs"], required=True)
    p.add_argument("name")
    p.set_defaults(func=cmd_read)

    p = sub.add_parser(
        "listen",
        help="diagnostic: passively listen on :14550 for any MAVLink traffic, no sends",
    )
    p.add_argument("--timeout", type=float, default=10.0, metavar="SECONDS")
    p.set_defaults(func=cmd_listen)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
