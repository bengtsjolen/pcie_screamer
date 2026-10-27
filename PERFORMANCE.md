# PCIe Squirrel read-throughput measurements

Date: 2026-10-26 (updated 2026-10-27)

This document collects the memory-read throughput measured on the PCIe
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
