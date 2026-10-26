# PCIe Squirrel read-throughput measurements

Date: 2026-10-26

This document collects the memory-read throughput measured on the PCIe
Squirrel with our LiteX/Migen gateware and with the original
`ufrisk/pcileech-fpga` PCIeSquirrel gateware v4.14.

## TL;DR

* The original 20 MB/s problem was the FT601 running at USB 2.0 (480 Mbps),
  not the PCIe/gateware path.
* On USB 3.0 the ufrisk v4.14 gateware reaches **~149 MiB/s (156.6 MB/s)**
  on a 1 GiB range and **~148 MiB/s (155 MB/s) of valid data** on a 2 GiB
  range on this machine.
* Our gateware after increasing the TX FIFOs reaches **~139 MiB/s
  (146 MB/s)** on 1 GiB, i.e. about 7% below the ufrisk reference on this
  host.
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
    (64 KiB `mux_out_fifo`, 32 KiB FT601 `write_fifo`), flashed 2026-10-27

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

The FIFO change was:

* `gateware/pcileech_fifo.py`: `mux_out_fifo` 512 -> 2048 frames
  (16 KiB -> 64 KiB)
* `gateware/ft601.py`: FT601 `write_fifo` 1024 -> 8192 words
  (4 KiB -> 32 KiB)

This matches the buffering of the ufrisk reference design and removes
the USB starvation between PCIe completion bursts. BRAM usage after the
change is 42.5 / 50 tiles.

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
* The ufrisk v4.14 gateware is still faster than ours on sustained reads:
  ~149 MiB/s vs ~139 MiB/s on 1 GiB (~7% difference), and ~148 MiB/s vs
  ~136 MiB/s of valid data on 2 GiB (~8% difference).
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

The final design keeps only the two FIFO increases that produced the large
win: `mux_out_fifo` 512 -> 2048 frames and FT601 `write_fifo`
1024 -> 8192 words. BRAM usage is 42.5/50 tiles. No experimental FSM or
extra TLP-buffer changes were kept.

USB traces of a 1 GiB dump on both gatewares show where the remaining 7-8%
goes:

| Gateware | Full 64 KiB reads | 20-byte filler reads | Partial reads |
|----------|-------------------|----------------------|---------------|
| ufrisk v4.14 | 19,584 | 64 | 64 |
| ours | ~19,300 | 3,136 | ~1,000 |

Our completion stream occasionally starves the USB output, so the host
issues roughly 19% more read commands; each extra command costs a control
transfer and a round trip. The short 20-byte reads are FT601 sync-word
filler packets emitted when the FPGA runs out of completion data.

These cheap fixes were tested and made no measurable difference:

* FT601 return-to-IDLE instead of the 100 us `drain_wait` (137.3 MiB/s).
* A second-stage TLP RX FIFO, +8 KiB, matching ufrisk's two-stage
  buffering (137.3 MiB/s).
* Limiting continuous TX to 64-word bursts and prioritizing RX in IDLE
  (137.3 MiB/s).

All three were reverted. The remaining gap therefore appears to be in the
PCIe completion path or its interaction with the host read pipeline, not
in the FT601/USB side. Closing it would require a deeper RX-path redesign
(e.g. a different PCIe RX width/stream architecture or more invasive
pacing), which was deliberately not done to keep the design simple and
extensible.

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
