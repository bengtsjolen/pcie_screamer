#!/usr/bin/env python3
"""Talk to the Versa the way LeechCore RawUDP talks to an FT601.

The payload is the FT601 byte pipe, not little-endian logical words. A
logical word is sent big-endian, which is what ``FT_WritePipe`` writes.
Replies are the bytes ``FT_ReadPipe`` would return, printed as
little-endian dwords. Default target is 192.168.1.50 port 28474.

    python3 tools/pcileech_udp.py
    python3 tools/pcileech_udp.py --read
"""

import argparse
import socket
import struct
import time

UDP_PORT = 28474
MAGIC = 0x77
TYPE_LOOP = 0b10
SYNC = 0x55556666
MARKER = (0xeffffff3, 0xdeceffff)
CORE_MAGIC = 0x89ab0000  # host view of ro[0] == 0xab89


def pipe(*logical):
    """FT_WritePipe bytes for logical FIFO words."""
    return b"".join((v & 0xffffffff).to_bytes(4, "big") for v in logical)


def dwords(blob):
    return [struct.unpack_from("<I", blob, i)[0] for i in range(0, len(blob) - 3, 4)]


def fmt(words):
    return " ".join(f"{w:08x}" for w in words)


class RawUDP:
    def __init__(self, ip, port, timeout):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("192.168.1.148", 0))
        self.sock.settimeout(timeout)
        self.dest = (ip, port)

    def send(self, payload):
        self.sock.sendto(payload, self.dest)

    def recv_until(self, seconds, stop=None):
        words = []
        deadline = time.time() + seconds
        while time.time() < deadline:
            self.sock.settimeout(max(0.01, deadline - time.time()))
            try:
                blob, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                break
            words.extend(dwords(blob))
            if stop and stop(words):
                break
        return words


def loopback_packet(data):
    flags = MAGIC | (TYPE_LOOP << 8) | (1 << 10)
    return pipe(0x66665555, data, flags)


def read_probe():
    """Core magic read, then the 1 ms inactivity arm FT_ReadPipe sends."""
    # resync, then CMD read of core ro[0] (flags 0x13, address 0).
    read = pipe(0x66665555, 0x66665555) + bytes([
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x77,
    ])
    # rw[64] = 0x000186a0, two masked writes at byte 0x08 and 0x0a.
    timer = bytes([
        0xa0, 0x86, 0xff, 0xff, 0x80, 0x08, 0x23, 0x77,
        0x01, 0x00, 0xff, 0xff, 0x80, 0x0a, 0x23, 0x77,
    ])
    # ReadPipe's prefix: arm rw[16], leave the tick count alone.
    arm = bytes([0x01, 0x00, 0x01, 0x00, 0x80, 0x02, 0x23, 0x77])
    return read, timer, arm


def has_marker(words):
    return any(words[i:i + 2] == list(MARKER) for i in range(len(words) - 1))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="192.168.1.50")
    parser.add_argument("--port", type=int, default=UDP_PORT)
    parser.add_argument("--data", default="0xaabbccdd")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--read", action="store_true",
                        help="core read plus the FT_ReadPipe inactivity marker")
    args = parser.parse_args()
    link = RawUDP(args.ip, args.port, args.timeout)

    if not args.read:
        data = int(args.data, 0)
        link.send(loopback_packet(data))
        words = link.recv_until(args.timeout)
        if not words:
            print(f"no reply from {args.ip}:{args.port} within {args.timeout:.1f}s")
            return 1
        echo = int.from_bytes(data.to_bytes(4, "little"), "big")
        print(fmt(words))
        print(f"sync={SYNC in words} echo={echo in words}")
        return 0 if SYNC in words and echo in words else 1

    read, timer, arm = read_probe()
    link.send(timer)
    time.sleep(0.01)
    link.send(read)
    time.sleep(0.02)
    link.send(arm)
    words = link.recv_until(0.2, stop=has_marker)
    print(fmt(words))
    print(f"sync={SYNC in words} magic={CORE_MAGIC in words} marker={has_marker(words)}")
    return 0 if SYNC in words and CORE_MAGIC in words and has_marker(words) else 1


if __name__ == "__main__":
    raise SystemExit(main())
