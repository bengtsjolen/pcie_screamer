#!/usr/bin/env python3
"""BAR0 MMIO soak for the ECP5 `--litepcie` build. Run on the host as root.

Maps BAR0 through sysfs and checks 32-bit writes/reads against a shadow copy: fixed
patterns over the whole BAR first, then random single-dword reads and writes until
`--seconds` is up. Only whole-dword accesses are used: LitePCIeWishboneMaster ignores
byte enables, so a sub-dword write would overwrite the full dword.

Memory space must be enabled first: setpci -s <bdf> COMMAND=0x02
"""

import argparse
import ctypes
import mmap
import os
import random
import sys
import time

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--bdf", default="0000:ad:00.0")
parser.add_argument("--seconds", type=float, default=60, help="Length of the random phase")
parser.add_argument("--seed", type=int, default=1)
args = parser.parse_args()

dev = f"/sys/bus/pci/devices/{args.bdf}"
with open(f"{dev}/config", "rb") as f:
    command = int.from_bytes(f.read(6)[4:6], "little")
if not command & 0x2:
    sys.exit(f"memory space is disabled (command=0x{command:04x}); run: setpci -s {args.bdf} COMMAND=0x02")

fd = os.open(f"{dev}/resource0", os.O_RDWR | os.O_SYNC)
size = os.fstat(fd).st_size
bar = (ctypes.c_uint32 * (size // 4)).from_buffer(mmap.mmap(fd, size))
words = len(bar)
rng = random.Random(args.seed)
shadow = [0] * words
errors = reads = writes = 0


def check(i, phase):
    global errors, reads
    got = bar[i]
    reads += 1
    if got != shadow[i]:
        errors += 1
        if errors <= 20:
            print(f"MISMATCH {phase} offset 0x{4 * i:03x}: read 0x{got:08x} expected 0x{shadow[i]:08x}")


def write(i, value):
    global writes
    bar[i] = value
    shadow[i] = value
    writes += 1


start = time.time()
patterns = {
    "zeros":   lambda i: 0x00000000,
    "ones":    lambda i: 0xffffffff,
    "aaaa":    lambda i: 0xaaaaaaaa,
    "5555":    lambda i: 0x55555555,
    "address": lambda i: (4 * i) | (~(4 * i) & 0xffff) << 16,
    "random":  lambda i: rng.getrandbits(32),
}
for name, pattern in patterns.items():
    before = errors
    for i in range(words):
        write(i, pattern(i))
    for i in range(words):
        check(i, name)
    print(f"pattern {name:<8}: {words} dwords, {errors - before} mismatches")

deadline = time.time() + args.seconds
report = time.time() + 10
while time.time() < deadline:
    for _ in range(2000):
        i = rng.randrange(words)
        if rng.random() < 0.5:
            write(i, rng.getrandbits(32))
        else:
            check(i, "random")
    if time.time() > report:
        print(f"  {time.time() - start:6.0f} s: {reads} reads, {writes} writes, {errors} mismatches", flush=True)
        report += 10

for i in range(words):
    check(i, "final")
elapsed = time.time() - start
print(f"{'FAIL' if errors else 'PASS'}: {reads} reads, {writes} writes, {errors} mismatches "
      f"in {elapsed:.0f} s ({(reads + writes) / elapsed:.0f} accesses/s)")
sys.exit(1 if errors else 0)
