#!/usr/bin/env python3
"""Stand-in autonomy client: receives SimBridge UDP packets and decodes them.

Shows that simulation output can be consumed from another process and another
language without linking any simulator code. Decodes the layout documented in
include/simbridge/wire.hpp.

    python3 tools/autonomy_listener.py --port 47000
    ./build/simbridge_run scenarios/coastal_patrol.scn --udp 127.0.0.1:47000
"""
import argparse
import math
import socket
import struct
import sys

MAGIC = 0x47524253
VERSION = 1
HEADER = struct.Struct("<IHHI")
ENTITY = struct.Struct("<Id8dH")  # id, t, pos xyz, vel xyz, heading, speed, name_len
DETECTION = struct.Struct("<3I3d")


class WireError(ValueError):
    pass


def decode(packet: bytes):
    if len(packet) < HEADER.size:
        raise WireError("packet truncated")
    magic, version, msg_type, length = HEADER.unpack_from(packet)
    if magic != MAGIC:
        raise WireError("bad magic")
    if version != VERSION:
        raise WireError(f"unsupported version {version}")
    payload = packet[HEADER.size:]
    if len(payload) != length:
        raise WireError("payload length mismatch")

    if msg_type == 1:
        fields = ENTITY.unpack_from(payload)
        name_len = fields[-1]
        name = payload[ENTITY.size:ENTITY.size + name_len].decode("utf-8")
        eid, t, x, y, z, vx, vy, vz, heading, speed = fields[:-1]
        return "entity_state", {"id": eid, "name": name, "t": t, "pos": (x, y, z),
                                "heading_rad": heading, "speed_mps": speed}
    if msg_type == 2:
        sensor, host, target, t, rng, bearing = DETECTION.unpack(payload)
        return "detection", {"sensor_id": sensor, "host_id": host, "target_id": target,
                             "t": t, "range_m": rng, "bearing_rad": bearing}
    raise WireError(f"unknown message type {msg_type}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47000)
    parser.add_argument("--count", type=int, default=0, help="exit after N packets (0 = run until idle)")
    parser.add_argument("--idle-timeout", type=float, default=3.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(args.idle_timeout)
    print(f"listening on {args.host}:{args.port}", flush=True)

    counts = {"entity_state": 0, "detection": 0, "errors": 0}
    contacts = {}
    received = 0
    try:
        while args.count == 0 or received < args.count:
            try:
                packet, _ = sock.recvfrom(65535)
            except socket.timeout:
                break
            received += 1
            try:
                kind, msg = decode(packet)
            except WireError:
                counts["errors"] += 1
                continue
            counts[kind] += 1
            if kind == "detection":
                first = msg["target_id"] not in contacts
                contacts[msg["target_id"]] = msg
                if first:
                    print(f"t={msg['t']:7.2f}s  NEW CONTACT target {msg['target_id']} "
                          f"range {msg['range_m']:7.1f} m  bearing {math.degrees(msg['bearing_rad']):6.1f} deg",
                          flush=True)
    finally:
        sock.close()

    print(f"received {received} packets: {counts['entity_state']} entity states, "
          f"{counts['detection']} detections, {counts['errors']} decode errors; "
          f"{len(contacts)} distinct contacts")
    return 0


if __name__ == "__main__":
    sys.exit(main())
