# PCIe read-throughput measurements

Date: 2026-10-26 (Squirrel, updated 2026-10-27). ECP5 Versa added 2026-11-22.
LitePCIe example loopback added 2026-11-23. RawUDP dump added 2026-11-23,
follow-up through 2026-11-24.

This document collects memory-read throughput for two platforms. The ECP5
section is the on-chip search engine (`ReadBench`, four outstanding 4 KiB
reads, UART console). The Squirrel section is the FT601/PCILeech path.

## ECP5 Versa (LFE5UM-45F, Gen1 x1)

### TL;DR

* The search image keeps four 4 KiB Memory Reads in flight and scans each
  buffer from the on-chip CPU. A 1 GiB run from `0x100000000` measured
  **196.7 MiB/s (206 MB/s)** both as a raw `bench` and as a `scan` at
  stride `0x400`.
* That is the Gen1 x1 payload ceiling (raw line rate 250 MB/s; expected
  payload after TLP overhead about 200–225 MB/s). The Squirrel's 426.7
  MiB/s raw figure is the same test on Gen2 x1.
* At stride `0x400` the CPU kept up with the DMA: the scan time matched
  the raw benchmark. The link is the limit at that stride.
* The VESA option ROM at `0x3f000` completed as a 64-byte read and as a
  4 KiB read, and a scan of that 4 KiB matched at `0x3f000`.
* The LitePCIe example (`soc_ecp5_litepcie.py`, one DMA channel, on-chip
  loopback) measured **0.38 Gbps** of reader payload on 2026-11-23, about
  47.5 MB/s, with the writer sending the same buffers back and zero data
  errors. That is not this ReadBench number. The example posts one TLP at
  a time and splits each 8 KiB buffer into 512-byte requests. Build steps
  and the host commands are in [LITEPCIE_EXAMPLES.md](LITEPCIE_EXAMPLES.md).
* Stock `pcileech` 4.20 over RawUDP (`soc_ecp5_udp.py`) first dumped
  256 MiB + 4 KiB at **23 MB/s** with no failed pages. Reporting device
  id 5, clocking the stack at 70 MHz, and capping the keepalive at
  100 µs moved the printed speed to **32–36 MB/s**. A 512 KiB read batch
  in a patched LeechCore, preloaded under the same binary, finished at
  **36 MB/s** as well. The transfer itself is what sits there. Host steps
  are in [VARIANTS.md](VARIANTS.md). The patch is
  [contrib/leechcore-netv2-512kib.patch](contrib/leechcore-netv2-512kib.patch).

### Test setup

Hardware:

* Lattice ECP5 Versa Development Board, LFE5UM-45F (non-5G)
* PCIe Gen1 x1, on-board 156.25 MHz reference clock
* FT2232 JTAG/UART. Console:
  `/dev/serial/by-id/usb-Lattice_Lattice_ECP5_Versa_Board-if01-port0`
  at 115200
* No FT601. The UART only carries commands and results

Host:

* Rechner2. Root port `0000:ac:00.0`, endpoint `0000:ad:00.0`
* After rescan: `Mem+ BusMaster+`, BAR0 `e6600000` (32-bit,
  non-prefetchable, 4 KiB)
* IOMMU off, `nokaslr`, `pcie_aspm=off`

Gateware:

* `soc_ecp5_search.py --device LFE5UM --gen 1 --refclk-mhz 156 --seed 24`
* nextpnr seed 24 closed clk100 at 100.19 MHz
* Bitstream
  `build/soc_ecp5_search_lfe5um_gen1_ref156/gateware/soc_ecp5_search_lfe5um_gen1_ref156.bit`,
  flashed 2026-11-22
* Engine: `gateware/read_bench.py` (four tags, four 4 KiB buffers). The
  console is `software/soc_search_console`

Method: wall clock from sending the UART command until the `soc> ` prompt
returns. The reply is a few dozen bytes, so the 5.21 s figure is the DMA.
`1 MiB = 1024*1024` and `1 MB = 1000*1000`.

```text
bench 100000000 40000000
scan 100000000 40000000 deadbeef feedface cafebabe 12345678 400
```

The scan pattern is chosen so the run covers the whole range. Progress
lines every 256 MiB (`0x10000000`, `0x20000000`, `0x30000000`,
`0x40000000`) confirmed the full 1 GiB was scanned.

### Theoretical limit

| Stage | Calculation | Limit |
|-------|-------------|-------|
| PCIe Gen1 x1 raw | 2.5 GT/s × 1 lane × 8/10 | 250 MB/s |
| Payload after TLP overhead | 4 KiB MRd plus CplD framing | ~200–225 MB/s |
| Measured 1 GiB `bench` | wall clock 5.21 s, 196.7 MiB/s | **206 MB/s** |

Device Control reset still advertises a 512-byte max read request. The
4 KiB reads are larger than that advertisement, and the host completed
them. The 100 MHz, 64-bit on-chip path (800 MB/s) is above the link rate.

### Results

| Command | Range | Elapsed s | MiB/s | MB/s | Result |
|---------|-------|-----------|-------|------|--------|
| `bench` | `0x100000000` + 1 GiB | 5.21 | 196.7 | 206 | `bench done` |
| `scan` stride `0x400` | `0x100000000` + 1 GiB | 5.21 | 196.7 | 206 | `no match` |

Correctness checks on the same image, before the 1 GiB runs:

| Command | What it checked | Result |
|---------|-----------------|--------|
| `bench 3f000 40`, then `rpeek` | first 32 bytes of the VESA ROM | `56455341 000300f1 00300100 000022f0 00300001 029407f1 00301af1 003024f1` |
| `bench 3f000 1000` | one 4 KiB read (1024 dwords) | `bench done`, `cnt_count=0x400` |
| `scan 3f000 1000 41534556 f1000300 00013000 f0220000` | VESA signature in that buffer | `match 0x000000000003f000` |

Printed high byte first, the header is
`56 45 53 41 00 03 00 f1 00 30 01 00 00 00 22 f0`.

An earlier ChunkDMA-only image (one outstanding 4 KiB
read) also dumped `0x3f000` for `0x40` and `0x1000` and matched this ROM.
That image was not timed.

### Observations

* A 4 KiB `bench` is one Memory Read, so it does not fill the four tags.
  The 1 GiB length is 262144 chunks of 4 KiB, which is the run that keeps
  four reads in flight.
* The sparse scan and the raw bench took the same 5.21 s. On the Squirrel
  (Gen2 x1) the same stride was 330–338 MiB/s against a 426.7 MiB/s raw
  bench, so the CPU still had headroom there. Here both sit on the Gen1
  payload ceiling.
* Dense strides (`0x10`, `0x100`) were CPU-bound on the Squirrel and were
  not re-run on this link.
* Link replay/corruption was not visible in these runs: the 4 KiB VESA
  read completed with the expected dword count, the signature matched, and
  the 1 GiB runs returned `bench done` / `no match` with no error code.

### RawUDP, stock pcileech

`soc_ecp5_udp.py` on the same Versa and the same Rechner2 slot. The
Ethernet host ran `pcileech` 4.20 and the FPGA bus-mastered Rechner2.
Command and bus-master setup are in [VARIANTS.md](VARIANTS.md). No
separate wall clock was taken; the speed below is what `pcileech` printed.
`1 MiB = 1024*1024`. The span is `0x20040000 - 0x1003f000 = 0x10001000`
(256 MiB + 4 KiB), which is 65537 pages of 4 KiB. `pcileech` showed that
as progress `256 / 256`.

```text
pcileech dump -device RAWUDP://ip=192.168.1.50 -min 0x1003f000 -max 0x20040000 -out dump5.test
```

| Host SW | Range | pcileech speed | Pages read | Pages failed |
|---------|-------|----------------|------------|--------------|
| 4.20.0.67 | `0x1003f000` + 256 MiB + 4 KiB | 23 MB/s | 65537 / 65537 | 0 |

The same image's one-page read at `0x3f000` returned the VESA header
`56 45 53 41` and the `NVIDIA` string at offset `0x100`. The 256 MiB run
was not byte-checked.

Effective payload on these links, after framing, is about 10 MB/s at
100 Mbit and about 100 MB/s at 1000 Mbit. The raw line rates are 12.5 MB/s
and 125 MB/s. The MAC/IP/UDP stack that fills the store-and-forward TX
FIFO runs at 50 MHz (`cd_eth`); the PHY consumes one byte per cycle of
the 125 MHz gigabit clock. Scaling the gigabit payload ceiling by that
clock ratio gives the rate to expect before the stack can be clocked
higher and still meet timing:

| Stage | Calculation | Ceiling |
|-------|-------------|---------|
| 100 Mbit effective payload | ~80% of 12.5 MB/s | 10 MB/s |
| 1000 Mbit effective payload | ~80% of 125 MB/s | 100 MB/s |
| Stack at 50 MHz, PHY byte clock 125 MHz | 50/125 × 100 MB/s | **40 MB/s** |
| Measured RawUDP `dump` | `pcileech` 4.20, 65537 pages, 0 failed | **23 MB/s** |

23 MB/s is past the 10 MB/s 100 Mbit figure. It matches the operator's
FT2232H-56Q result on an Acorn CLE-215+, which was not re-measured for
this note. The on-chip `bench` on this same Gen1 x1 slot is 206 MB/s, so
the PCIe link is above the Ethernet path. That 23 MB/s run reported
device id 4, so LeechCore used the ScreamerM2 profile (`DELAY_READ` 300 µs,
`DELAY_WRITE` 25 µs) and the stock 1 ms keepalive.

### RawUDP follow-up, 2026-11-23 and 2026-11-24

Three host-visible changes, then one host-library change. Speeds are what
`pcileech` printed. `1 MB = 1000*1000`.

The image reports device id **5** (NeTV2 RawUDP). That profile has
`DELAY_READ = 0`, `DELAY_WRITE = 0`, and `MAX_SIZE_RX = 0x1c000` (112 KiB).
The connect line is `NeTV2 RawUDP ... [0,0,0]`. The MAC/IP/UDP stack clock
is **70 MHz**. Post-route `crg_clkout` reached 73.7 MHz against that
constraint. The gigabit RX clock on the same build reached 123.4 MHz
against 125 MHz. LeechCore still writes a 1 ms inactivity timer and arms
it on every `FT_ReadPipe`. The read returns when it sees the keepalive
marker, so the Versa caps the idle gap at **100 µs** (10 000 ticks at
100 MHz) and sends the same marker.

| Change | Range | Speed | Pages failed |
|--------|-------|-------|--------------|
| id 4, stack 50 MHz, keepalive 1 ms | `0x1003f000` + 256 MiB + 4 KiB | 23 MB/s | 0 / 65537 |
| id 5, stack 50 MHz, keepalive 1 ms | same class of dump | 28 MB/s | not recorded; meter often 32, sometimes higher |
| id 5, stack 70 MHz, keepalive 1 ms | same class of dump | 26 MB/s | not recorded |
| id 5, stack 70 MHz, keepalive 100 µs | `0x1003f000`–`0x30040000` | 36 MB/s | 822 / 131073 |
| same image, repeat | `0x1003f000`–`0x30040000` | 34 MB/s | 926 / 131073 |
| same image | `0x3003f000`–`0x50040000` | 34 MB/s | 886 / 131073 |
| same image | `0x1003f000`–`0x20040000` | 32 MB/s | 432 / 65537 |
| same image, LeechCore `MAX_SIZE_RX` 512 KiB | `0x1003f000`–`0x20040000`, `-out none` | 36 MB/s | 643 / 65537 |

The 512 KiB row used LeechCore 2.23.5 (`23f1919`, 2026-11-02) with
[contrib/leechcore-netv2-512kib.patch](contrib/leechcore-netv2-512kib.patch),
built as `contrib/LeechCore/files/leechcore.so` and loaded with
`LD_PRELOAD` in front of `pcileech` 4.20. The header matches the one
shipped with that binary, so MemProcFS and PciLeech were left as they
are. The connect line was `NeTV2 RawUDP PCIe gen1 x1 [0,0,0] [v4.14,ad00]
[SYNC,NORM]`. The meter moved between 32 and 41 MB/s. That library also
printed `protocol=403` for the run.

Raising the stack from 50 MHz to 70 MHz, with the 1 ms keepalive still in
place, went from 28 MB/s to 26 MB/s. The 8-bit stack clock was not what
held the dump at the high 20s. Capping the keepalive at 100 µs is what
moved the final number into the low-to-mid 30s. On a 112 KiB batch that
1 ms had been about a quarter of the wall time. At 100 µs it is a few
percent.

The failed pages showed up with the short keepalive and stayed at roughly
0.6–1% of pages. Two dumps of the same 512 MiB do not miss the same pages.
The holes are single pages and pairs, spread evenly across the 28 pages
of a 112 KiB batch. A keepalive that closed the read early would have
piled them on the last page of the batch. A dropped datagram throws away
the completions inside it, and the next datagram is a new TLP frame, so
the rest of the batch survives. That matches the gigabit RX clock still
missing 125 MHz. The speed figures above include those holes. The NeTV2
profile does not retry them.

A continuously busy 70 MHz stack would move about **54 MB/s** of memory.
The stack emits one frame byte per cycle, 70 MB/s, and a 1024-byte
datagram is 1066 of those bytes once the Ethernet, IP, and UDP headers
are counted, about 67 MB/s of FT601 payload. A 256-byte completion is
320 bytes on that pipe (status word plus header), and `67 / 1.25` is
54 MB/s. 50 MB/s of memory needs that shifter busy almost the whole time.

The 512 KiB batch was the test of the other suspect: stock LeechCore
stops every 112 KiB, waits for the marker, parses, and only then sends
the next reads. The tag field holds 128 pages, which is 512 KiB, and
`readsize` can only lower `MAX_SIZE_RX`. Quadrupling the batch left the
dump at 36 MB/s. The pause between batches is not the remaining gap. The
transfer itself runs near 36 MB/s, short of the 54 MB/s a full shifter
would do.

To try the 512 KiB profile again, from a fresh LeechCore 2.23.5 tree:

```sh
git clone --depth 1 https://github.com/ufrisk/LeechCore.git contrib/LeechCore
git -C contrib/LeechCore apply ../leechcore-netv2-512kib.patch
make -C contrib/LeechCore/leechcore
LD_PRELOAD=$PWD/contrib/LeechCore/files/leechcore.so \
  pcileech dump -device RAWUDP://ip=192.168.1.50 ...
```

Run that from this repository. `git -C` makes the apply path relative to
`contrib/LeechCore`. The patch is against commit `23f1919`.

## PCIe Squirrel read-throughput measurements

Date: 2026-10-26 (updated 2026-10-27)

This section collects the memory-read throughput measured on the PCIe
Squirrel with our LiteX/Migen gateware and with the original
`ufrisk/pcileech-fpga` PCIeSquirrel gateware v4.14.

## TL;DR

* The original 20 MB/s problem was the FT601 running at USB 2.0 (480 Mbps),
  not the PCIe/gateware path.
* On USB 3.0 the ufrisk v4.14 gateware reaches **~149 MiB/s (156.6 MB/s)**
  on a 1 GiB range and **~148 MiB/s (155 MB/s) of valid data** on a 2 GiB
  range on this machine.
* Increasing the TX FIFOs took our gateware from ~101 to ~139 MiB/s on
  1 GiB.
* Replacing the single-stream 64->32-bit RX path with an ufrisk-style
  128-bit RX path (64->128 packer, 128-bit FIFO, four parallel 32-bit mux
  ports) took it to **~147 MiB/s (154 MB/s)** with PCILeech 4.17.8.49 and
  **~149 MiB/s** with 4.19.5, within ~1.5% of ufrisk on 1 GiB.
* The claimed 190 MB/s figure for v4.14 was **not reproduced** here, even
  with PCILeech 4.19.5 (best 1 GiB result: 151.7 MiB/s / 159 MB/s).

## Test setup

Hardware:

* LambdaConcept PCIe Screamer Squirrel, XC7A35T-FGG484-2
* FTDI FT601, 32-bit FIFO
* PCIe Gen2 x1
* FT601 enumerated at **5000 Mbps** (USB 3.0 SuperSpeed)

Host/target:

* Same machine is host and DMA target (X11DPi-NT, 2x Intel Xeon Scalable,
  24 cores per socket, kernel 6.8.0-138-generic)
* Intel C620 xHCI USB 3.0 controller (`8086:a1af`)

Software:

* Primary: PCILeech 4.17.8.49 from the v4.17.5 Linux bundle
* Cross-check: PCILeech 4.19.5.63
* Gateware under test:
  * ufrisk PCIeSquirrel v4.14 (`pcileech_squirrel-4.14.bin`)
  * ours: LiteX/Migen tree at commit `0286dde` plus the TX-FIFO change
    (64 KiB `mux_out_fifo`, 32 KiB FT601 `write_fifo`) and the 128-bit RX
    path (`gateware/tlp128.py`, four parallel mux ports), flashed 2026-10-27

Method:

```sh
pcileech dump -min <start> -max <end> -out none
```

`-out none` avoids disk I/O. The ranges are read from low physical RAM:

| Name | Range | Requested bytes |
|------|-------|-----------------|
| 64M  | `0x01000000` - `0x05000000` | 64 MiB |
| 1G   | `0x01000000` - `0x41000000` | 1 GiB |
| 2G   | `0x01000000` - `0x81000000` | 2 GiB |

The 2 GiB range contains 40960 unreadable pages (160 MiB of memory-map
holes) on this machine, so both "raw" (requested range) and "valid"
(pages actually read) throughput are reported. 1 GiB had no failed pages.

"Raw" and "Valid" use `1 MiB = 1024*1024` and `1 MB = 1000*1000`.
PCILeech's own progress line is listed separately; it is not the same
measurement.

## Theoretical limits

These are interface/protocol ceilings, not expected benchmark results.

| Stage | Calculation | Limit |
|-------|-------------|-------|
| PCIe Gen2 x1 raw | 5 GT/s × 1 lane × 8/10 | 500 MB/s |
| PCIe TLP payload, 256 B CplD | 256 B / (256 + 12 B header + 4 B LCRC + 2 B seq + ~8 B framing) | ~455 MB/s |
| FT601 USB3 Gen1 raw | 5 Gbps × 8/10 | 500 MB/s |
| FT601 maximum | FTDI spec, 32-bit FIFO at 100 MHz | **400 MB/s** |
| PCILeech mux framing | 7 data words per 8-word / 256-bit frame | 400 × 7/8 = 350 MB/s |
| PCILeech + 256 B CplD overhead | 350 × ~0.91 | **~318 MB/s ≈ 303 MiB/s** |

* The observed CplDs carry 256 B of payload because the root complex uses
  a 256 B maximum payload size. With 4 KiB CplDs the TLP efficiency would
  be ~99% and the combined ceiling ~347 MB/s.
* The 100 MHz, 64-bit internal path (800 MB/s) and the PCIe user interface
  (64-bit at 125 MHz, 1 GB/s) are not the limiting interfaces.
* Sync words and keepalive frames are small and not included above.
* The measured 147-149 MiB/s is about 48-49% of the ~303 MiB/s
  protocol/framing ceiling.

### Memory bandwidth is not the limit

The host has tens to hundreds of GB/s of DRAM bandwidth. A ~150 MB/s DMA
read is well below 1% of that, so DRAM bandwidth and self-DMA congestion
are negligible. Self-DMA also does not copy the data twice: the FPGA reads
host DRAM over PCIe and the same data leaves over USB. The remaining gap
is dominated by completion latency, outstanding-request depth, and host
software pacing.

## Results

### Our gateware

USB 2.0 (480 Mbps), before any gateware changes:

| Host SW | Range | Elapsed s | Raw MiB/s | Valid MiB/s | Valid MB/s | PCILeech speed | Pages failed |
|---------|-------|-----------|-----------|-------------|------------|----------------|--------------|
| 4.17.8.49 | 64M | 2.760 | 23.19 | 23.18 | 24.31 | n/a | 4 |

USB 3.0 (5000 Mbps), before the TX FIFO increase:

| Host SW | Range | Elapsed s | Raw MiB/s | Valid MiB/s | Valid MB/s | PCILeech speed | Pages failed |
|---------|-------|-----------|-----------|-------------|------------|----------------|--------------|
| 4.17.8.49 | 64M  | 0.950 | 67.37 | 67.33 | 70.60 | 64 MB/s | 10 |
| 4.17.8.49 | 256M | 2.640 | 96.97 | 96.93 | 101.68 | 85 MB/s | 27 |
| 4.17.8.49 | 1G   | 10.150 | 100.89 | 100.83 | 105.73 | 93 MB/s | 137 |

USB 3.0 (5000 Mbps), after increasing the TX FIFOs:

| Host SW | Range | Elapsed s | Raw MiB/s | Valid MiB/s | Valid MB/s | PCILeech speed | Pages failed |
|---------|-------|-----------|-----------|-------------|------------|----------------|--------------|
| 4.17.8.49 | 64M | 0.646 | 99.07 | 99.07 | 103.88 | 64 MB/s | 0 |
| 4.17.8.49 | 1G  | 7.355 | 139.23 | 139.23 | 145.99 | 128 MB/s | 0 |
| 4.17.8.49 | 2G  | 13.868 | 147.68 | 136.14 | 142.75 | 146 MB/s | 40960 |

Measured with `./bench-pcileech.sh ours-final 64M 1G 2G`.

USB 3.0 (5000 Mbps), 128-bit RX path (final design):

| Host SW | Range | Elapsed s | Raw MiB/s | Valid MiB/s | Valid MB/s | PCILeech speed | Pages failed |
|---------|-------|-----------|-----------|-------------|------------|----------------|--------------|
| 4.17.8.49 | 64M | 0.651 | 98.31 | 98.31 | 103.09 | 64 MB/s | 0 |
| 4.17.8.49 | 1G  | 6.960 | 147.13 | 147.13 | 154.27 | 146 MB/s | 0 |
| 4.17.8.49 | 2G  | 13.170 | 155.50 | 143.36 | 150.32 | 146 MB/s | 40960 |
| 4.19.5.63 | 1G  | 6.857 | 149.34 | 149.34 | 156.59 | 146 MB/s | 0 |

Measured with `./bench-pcileech.sh ours-rx128 64M 1G 2G` plus the 4.19.5
bundle for the last row. A repeat 2 GiB run measured 144.31 MiB/s valid
(151.32 MB/s). BRAM usage is 41.5/50 tiles and LUT usage 22%.

Data validation: a 4 KiB dump of the stable VESA ROM region
(`0x3f000-0x40000`) with the 128-bit RX path is byte-identical to the old
ufrisk v4.14 dump, and the 1 GiB run had 0 failed pages. The 2 GiB
failures are the known memory-map holes (40960 pages), not data errors.

The changes were:

* `gateware/pcileech_fifo.py`: `mux_out_fifo` 512 -> 2048 frames
  (16 KiB -> 64 KiB)
* `gateware/ft601.py`: FT601 `write_fifo` 1024 -> 8192 words
  (4 KiB -> 32 KiB)
* `gateware/tlp128.py` plus the mux: 64->128 TLP packer, 128-bit
  Cpl/CplD filter and FIFO, and four parallel 32-bit mux ports

The FIFO change matches ufrisk's buffering. The 128-bit RX path mirrors
ufrisk's `pcileech_tlps128_src64`/`dst_fifo`/mux structure.

### ufrisk PCIeSquirrel v4.14

Measured with the same ranges and host software:

| Host SW | Range | Elapsed s | Raw MiB/s | Valid MiB/s | Valid MB/s | PCILeech speed | Pages failed |
|---------|-------|-----------|-----------|-------------|------------|----------------|--------------|
| 4.17.8.49 | 64M | 0.643 | 99.53 | 99.53 | 104.37 | 64 MB/s | 0 |
| 4.17.8.49 | 1G  | 6.856 | 149.36 | 149.36 | 156.61 | 146 MB/s | 0 |
| 4.17.8.49 | 2G  | 12.767 | 160.41 | 147.88 | 155.06 | 157 MB/s | 40960 |

Cross-check with a newer host stack, PCILeech 4.19.5.63:

| Host SW | Range | Elapsed s | Raw MiB/s | Valid MiB/s | Valid MB/s | PCILeech speed | Pages failed |
|---------|-------|-----------|-----------|-------------|------------|----------------|--------------|
| 4.19.5.63 | 64M | 0.647 | 98.92 | 98.92 | 103.72 | 64 MB/s | 0 |
| 4.19.5.63 | 1G  | 6.752 | 151.66 | 151.66 | 159.03 | 146 MB/s | 0 |
| 4.19.5.63 | 2G  | 12.866 | 159.18 | 146.74 | 153.87 | 157 MB/s | 40960 |

## Observations

* The USB 2.0 link caps throughput at ~23 MiB/s. The same gateware on
  USB 3.0 reached ~100 MiB/s before the FIFO fix, so the original 20 MB/s
  was entirely a USB-link problem.
* Increasing the TX buffering improved our 1 GiB result from 100.9 to
  139.2 MiB/s (+38%).
* The 128-bit RX path improved it further to 147.1 MiB/s with PCILeech
  4.17.8.49 and 149.3 MiB/s with 4.19.5, within ~1.5% of ufrisk on 1 GiB.
  On 2 GiB we now measure 143.4-144.3 MiB/s of valid data vs 146.7-147.9
  for ufrisk (~2-3%).
* After the RX path change, a 1 GiB USB trace has 19,712 full 64 KiB
  reads, 64 twenty-byte reads, and **zero** partial reads, versus
  19,584 / 64 / 64 for ufrisk. The mid-stream short reads are gone.
* The 64 MiB result is dominated by PCILeech process/device startup and
  is not a good steady-state indicator. Use 1 GiB or 2 GiB for comparisons.
* The 2 GiB "raw" number is inflated by 160 MiB of unreadable pages;
  "valid" is the more meaningful value. PCILeech's displayed speed tracks
  the requested range, which is why it reports 157 MB/s for the 2 GiB run.
* The published 190 MB/s for v4.14 was not reproduced on this host. The
  best observed result was 151.7 MiB/s / 159 MB/s (PCILeech 4.19.5,
  1 GiB). Differences may come from host CPU/memory-controller behavior,
  the read range, or the conditions used for the published figure.

## Remaining gap investigation

The first step was the two FIFO increases. The single-stream RX path still
left the output stream occasionally empty, so we then implemented the
ufrisk-style 128-bit RX path. The historical trace below shows why that
was needed; the final 128-bit path removes the starvation.

USB traces of a 1 GiB dump:

| Gateware | Full 64 KiB reads | 20-byte filler reads | Partial reads |
|----------|-------------------|----------------------|---------------|
| ufrisk v4.14 | 19,584 | 64 | 64 |
| ours, single-stream RX | ~19,300 | 3,136 | ~1,000 |
| ours, 128-bit RX | 19,712 | 64 | 0 |

With the single-stream path, our completion stream occasionally starved
the USB output, so the host issued roughly 19% more read commands; each
extra command costs a control transfer and a round trip. The short 20-byte
reads are FT601 sync-word filler packets emitted when the FPGA runs out of
completion data.

### Resolution: 128-bit RX path

The 64->128 packer, 128-bit Cpl/CplD filter/FIFO, and four parallel 32-bit
mux ports removed the mid-stream starvation. The 1 GiB USB trace now has
19,712 full 64 KiB reads, 64 twenty-byte reads, and no partial reads,
essentially matching ufrisk. 1 GiB throughput is 147.1 MiB/s with
PCILeech 4.17.8.49 and 149.3 MiB/s with 4.19.5; the remaining 1.5-3%
difference to ufrisk is within run-to-run variance and small wire
overhead. BRAM usage is 41.5/50 tiles and LUT usage 22%.

### What the trace says

Bulk transfer rate is not the difference. Full 64 KiB reads take about the
same time in both designs (median start-to-start 0.325 ms vs 0.327 ms).
The mean is 0.344 ms for ufrisk vs 0.386 ms for the old single-stream
design; the difference was in the tail, where that stream occasionally
went empty and produced short or partial reads.

* 3,136 extra 20-byte reads and ~1,000 partial reads are enough to cost
  several percent: at roughly 0.1-0.15 ms per extra control transfer and
  round trip, that is ~0.3-0.5 s out of the ~7.6 s run.
* Longer FIFOs are not adding meaningful latency. BRAM/FIFO read latency
  is a few clock cycles, negligible next to the ~0.3 ms host read cycle.
  Deeper buffers should reduce stalls, not create them.
* It is also not PCIe link turnaround: the link, root complex, MRd size,
  and CplD size are the same. It is completion-arrival jitter plus the
  host's extra read-command round trips.

This pointed to the PCIe RX forwarding architecture. ufrisk splits a
128-bit TLP stream into four parallel 32-bit ports before the mux; the old
design used a single 64->32-bit stream through a stride converter and BE
filter, which made completion delivery less smooth and let the host's
credit-based read loop amplify the jitter into more read commands. The
128-bit RX path described above replaced that old path and closed the gap.

### FT601 session/underrun mechanism

FTDI AN_412 describes the IN direction as a USB read **session** with a
requested length. The FIFO master must write maximum-sized packets
(1024 B on USB3) except for the last packet, which must be a short packet.
If the master stops writing before the FIFO buffer is full and the
requested length has not been reached, the FT601 detects a **session
underrun** and terminates the read with a short packet.

The FPGA does not know the host's requested read length: that length is
part of the FT601/USB read request, while the FPGA only sees MRd TLPs and
writes CplD data. The original ufrisk gateware does not track it either;
it keeps the output stream fed so underruns do not occur. Our 5-word
filler writes 20 B and stops, so FT601 ends the session with a 20-byte
short packet. With the old single-stream RX path this produced the 3,136
short reads; after the 128-bit RX fix only the expected 64 remain,
matching ufrisk.

A gateware-only workaround is possible without changing the standard
PCILeech host: parse the MRd length fields and CplD byte counts, keep a
running count of outstanding completion bytes, and only allow an
underrun/short packet when no completions are outstanding. While
completions are outstanding the output must stay fed with real data or
filler so the session survives. The FT601 also has a
`DISABLECANCELSESSIONUNDERRUN` option, but the standard PCILeech host
sets `OptionalFeatureSupport = 0`, so it is not used in these tests.

A larger FIFO or store-and-forward pacing does not remove the
session-length problem. A 0.1 ms gap at 300 MB/s is only ~30 KB, which
the existing 112 KB buffering can cover; the buffer is often near empty
because the source average is ~150 MB/s. Waiting for a full buffer adds
latency and stalls the host read pipeline (measured ~100 MiB/s), and a
much larger FIFO would only smooth jitter, not raise the average above
the source rate.

Host-visible gaps can be extracted from USB traces; gateware-level gaps
would need a cycle counter. In the 1 GiB trace, gaps after partial reads
are ~0.2 ms typical, while 20-byte events cluster and can stall for
several milliseconds.

These cheap fixes were tested and made no measurable difference:

* FT601 return-to-IDLE instead of the 100 us `drain_wait` (137.3 MiB/s).
* A second-stage TLP RX FIFO, +8 KiB, matching ufrisk's two-stage
  buffering (137.3 MiB/s).
* Limiting continuous TX to 64-word bursts and prioritizing RX in IDLE
  (137.3 MiB/s).
* Delaying the 5-word filler to a 1 ms true-idle timeout (~100 MiB/s;
  the host read waits and the pipeline stalls).
* Filling to full 1024-byte FT601 packets without a terminating short
  packet (the host read blocks; reverted).

All were reverted. The final design uses the 128-bit RX path, which keeps
the design simple and extensible and closes the gap to ~1.5-3%.

## FPGA resource use

See [RESOURCES.md](RESOURCES.md) for the current XC7A35T utilization, the
FT601 controller cost, and the measured FIFO-depth/performance trade-offs.

## Reproducing a run

Use the script in the repository root:

```sh
./bench-pcileech.sh <label> [sizes...]
```

Examples:

```sh
# quick 1 GiB test (default)
./bench-pcileech.sh current

# full comparison set
./bench-pcileech.sh ufrisk-4.14 64M 1G 2G

# use a different PCILeech bundle
PCILEECH_DIR=$HOME/Code/FPGA/PciLeech/binary/v4.19.5 \
    ./bench-pcileech.sh ufrisk-4.14-pcileech-4.19.5 1G 2G
```

The script records the FT601 link speed, PCILeech version, elapsed time,
computed raw/valid throughput, PCILeech's reported speed, and page
results. It writes per-run CSVs plus a combined `all.csv` under
`bench-results/`.

Environment overrides: `PCILEECH_DIR`, `OUTDIR`, `TIMEOUT`.
