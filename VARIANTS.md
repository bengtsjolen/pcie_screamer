# PCIe Screamer bring-up variants

Date: 2026-10-31
Status: Five variants are functional on the PCIe Screamer Squirrel
(Xilinx Artix-7). The ECP5/Versa port has not been started.

## Purpose

These variants are not independent products. They are ordered stepping
stones that separate the hard parts of a PCIe memory-access tool:

1. a host-driven, high-throughput PCILeech-compatible path;
2. the smallest possible PCIe bus-master SoC;
3. the PCILeech wire protocol driven from an on-chip CPU instead of FT601;
4. a 4 KiB burst DMA engine with CPU-visible buffers;
5. multiple outstanding 4 KiB reads for high search throughput.

The long-term target is the Lattice ECP5 Versa board, which is already on
the PCIe bus and is programmed and controlled through an FT2232-family
JTAG/UART interface. The Versa has no FT601 yet. The variants are arranged
so that the first ECP5 steps do not depend on FT601, and so that an FT601
shield can later enable the same host-driven and high-bandwidth paths.

Nothing in this document is an ECP5 implementation plan. It records what
exists on the Squirrel, what each variant proves, and which dependencies
must be solved before the same variants can run on ECP5.

## Hardware baseline and target

### Current reference: PCIe Screamer Squirrel

- Xilinx XC7A35T-FGG484-2, PCIe Gen2 x1, 64-bit internal datapath.
- FTDI FT601, 32-bit USB 3.0 FIFO.
- FT4232H for JTAG and the serial console.
- Vendor/device ID `10ee:0666`, BAR0 `0x40000`.
- Shared system clock 100 MHz; the PCIe hard IP provides the user clock.

### Target: ECP5 Versa

- ECP5-5G device with the board's hard PCIe x1 edge connector.
- FT2232-family device for JTAG and UART.
- No FT601; a shield is planned but not designed or built.
- The ECP5 has a hard PCIe block, but this tree has no ECP5 platform file
  and the vendored LitePCIe has no ECP5 PHY. That is the largest open
  dependency for the port.

### Serial device naming

The Squirrel documentation uses `/dev/ttyUSB1`; the current handover used
`/dev/ttyUSB2`. Enumeration depends on which FTDI channels the host binds.
Use `/dev/serial/by-id` or `dmesg` rather than relying on the number.

## What each transport is for

| Transport | Carries | Required by |
|---|---|---|
| PCIe BAR0 | PCIe TLPs, bus-master reads, config/BME | All variants |
| FT601 USB 3.0 | PCILeech host command/data stream | V1 only |
| FT4232H / FT2232 UART | Console commands, progress, results, JTAG | All SoC variants (V2-V5) |

The important consequence for ECP5 is that V2-V5 use PCIe for the data
path and only use the serial port for control and results. A slow FT2232
UART does not limit their search throughput. FT601 is required for V1,
where the PCILeech host application owns the protocol, and it is useful
later for a high-speed host data path. It is not a prerequisite for the
single-word SoC or the on-chip search engines.

## Common PCIe mechanics

All SoC variants use the LitePCIe S7 PCIe PHY with a 64-bit datapath and
BAR0 `0x40000`. The common pieces are:

- requester ID from `pcie_id`;
- Bus Master Enable plus Memory Space Enable (Device Control `0x6`),
  issued once after link-up;
- MRd32 below 4 GiB and MRd64 at or above 4 GiB;
- completion parsing with tag, requester ID, lower-address, status, and
  byte-count checks;
- little-endian DWORD data as stored in host memory.

The search consoles compare the four 32-bit pattern words in host memory
order. A 32-bit word read from a DMA buffer is therefore byte-swapped with
`bswap32()` before comparison. For the VESA signature at `0x3f000`:

```text
bytes: 56 45 53 41 00 03 00 f1 00 30 01 00 00 00 22 f0
words: 0x41534556 0xf1000300 0x00013000 0xf0220000
```

## Variant summary

| # | Variant | Top level | Host/control | Data path | FT601 | Verified result |
|---|---|---|---|---|---|---|
| V1 | PCILeech over FT601 | `pcileech_squirrel.py` | `pcileech` binary over USB 3.0 | PCILeechFIFO, RX128, FT601 | Yes | 147-149 MiB/s on 1 GiB; VESA dump matches |
| V2 | SoC single-word reads | `soc_squirrel.py` | UART console | TLPDMA, one DWORD per MRd, 4 KiB page | No | VESA dump matches; high address simulation-only |
| V3 | CPU-fed PCILeech FIFO | `soc_pcileech.py` | UART console | PCILeechFIFO + CPU bridge, one DWORD per MRd | No | VESA dump matches; bridge tests pass |
| V4 | SoC double-buffer DMA | `soc_search.py`, ChunkDMA | UART console | 2 x 4 KiB buffers, one outstanding 4 KiB MRd | No | 2-buffer VESA/high scan verified; raw DMA ~319 MiB/s |
| V5 | SoC 4-outstanding search | `soc_search.py`, ReadBench | UART console | 4 x 4 KiB buffers, up to 4 outstanding 4 KiB MRds | No | VESA/high match; 330-338 MiB/s at 1/1024 stride; raw 426.7 MiB/s |

The originally requested four variants are V1, V2, V4, and V5. V3 is the
intermediate CPU-fed PCILeech variant described above.

V4 and V5 share the `soc_search.py` top level and one bitstream. The
`bench_select` CSR muxes the PCIe TX/RX between `ChunkDMA` and
`ReadBench`. The current console runs its `scan` and `bench` commands on
`ReadBench` (V5); its `peek` command reads the `ChunkDMA` buffers (V4).
The legacy 2-buffer `scan` console is the source at
`e2ba260:software/soc_search_console/main.c`; rebuilding the image with
that console gives a full V4 scan.

## V1 - PCILeech over FT601

### What it proves

The PCIe endpoint, TLP TX/RX paths, completion buffering, and USB 3.0
transport can run the unmodified PCILeech host application at the
reference throughput. This is the compatibility baseline: no custom host
software and no on-chip CPU are involved.

### Architecture

```text
pcileech host
    | FT601 USB 3.0, 32-bit FIFO
    v
FT601Sync -> PCILeechFIFO (RX128, mux, serializer) -> TLP32To64Packer
    |                                                    |
    +-------------------- S7PCIEPHY <--------------------+
                         PCIe Gen2 x1
```

### Components

- Top level: `pcileech_squirrel.py`.
- USB: `gateware/ft601.py`.
- PCILeech protocol: `gateware/pcileech_fifo.py`, `gateware/tlp128.py`,
  `gateware/tlp32.py`.
- Host: the ufrisk `pcileech` binary, no FPGA-side console.

### Build and run

```sh
source env.sh
venv/bin/python pcileech_squirrel.py --build
./program.sh build/pcie_squirrel/gateware/pcie_squirrel.bin
./rescan.sh
```

Optional FIFO presets: `--fifo-preset large|medium|mini`. The verified
reference image is `bitstreams/pcie_squirrel-cleanup-rx128.bin`
(SHA-256 `33848540d0efaacbd975321ff99ea14108051451be81c75fb5eb44f9e9e2297b`).

### Measured results

- 1 GiB: 147.1 MiB/s with PCILeech 4.17.8.49; 149.3 MiB/s with 4.19.5.
- 2 GiB: 143.4-144.3 MiB/s valid; the 160 MiB difference is known
  memory-map holes.
- Resource use (large RX128 build): 4,200 LUTs, 3,437 registers, 41.5/50
  BRAM tiles.

### Limitations

- Requires FT601 USB 3.0.
- No on-chip search; the host drives every operation.
- The design is close to the BRAM limit; the large FIFO preset is the
  performance configuration, while `medium` and `mini` trade throughput
  for BRAM.

### ECP5 implication

This is the last variant to port, not the first. It requires an FT601
shield, an ECP5-compatible FT601 controller port, and a ECP5 PCIe endpoint
that behaves like the Squirrel's S7 hard IP. It should follow, not precede,
the SoC bring-up.

## V2 - SoC with single-word reads

### What it proves

A minimal VexRiscv SoC can enumerate as a PCIe endpoint, enable bus
mastering, issue physical memory reads, collect completions into a local
buffer, and report the result over a serial console. It deliberately uses
one 4-byte MRd per DWORD. That is slow, but it isolates the PCIe, CPU,
BRAM, and UART integration without USB or a DMA engine.

### Architecture

```text
VexRiscv (100 MHz)
  |-- ROM / SRAM / UART (FT4232H or FT2232)
  |-- CSR bus
  v
TLPDMA -- one 4-byte MRd at a time --> S7PCIEPHY --> host memory
  |
  +-- 4 KiB page buffer -- CPU reads rd_index/rd_data
```

### Components

- Top level: `soc_squirrel.py`.
- DMA: `gateware/tlp_dma.py` (`TLPDMA`).
- Console: `software/soc_console/`.
- PCIe: LitePCIe S7 PHY; MRd32 below 4 GiB, MRd64 above.

### Build and run

```sh
source env.sh
venv/bin/python soc_squirrel.py --build
./program.sh build/soc_squirrel/gateware/soc_squirrel.bin
./rescan.sh
```

Serial console commands: `help`, `status`, `dump [addr] [len]`. The
default dump is `0x3f000`, length `0x1000`, and the console accepts
lengths from 4 to `0x1000` bytes in 4-byte steps.

The verified saved image is `bitstreams/soc_squirrel-console.bin`
(SHA-256 `25a439bed48e14ad0720a986678338cb51a147225da1f00172e36d29f57031df`).

### Measured results

- Hardware VESA dump matched SHA-256
  `5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db`.
- `dump 0x1000000 0x40` completed with no DMA error; the host stayed
  alive and the device stayed enumerated.
- Resource use: 2,050 LUTs, 2,470 registers, 16/50 BRAM tiles.

### Limitations

- One 4-byte read per request; the serial console and single-DW path are
  not throughput engines.
- MRd64 above 4 GiB is simulation-verified only in this variant; hardware
  used low-memory MRd32.
- The 8-bit request tag wraps after 256 requests.
- No independent ISSUE-state timeout.

### ECP5 implication

This is the recommended first ECP5 target. It needs only the hard PCIe
endpoint, a small CPU/SoC, and the FT2232 UART. The slow FT2232 does not
matter because the data path is PCIe; the UART carries only commands and
the final dump. Success here establishes the ECP5 PCIe PHY, BME, MRd32,
MRd64, completion parsing, and console before adding DMA complexity.

## V3 - CPU-fed PCILeech FIFO SoC

### What it proves

The PCILeech wire protocol and mux framing can be reused without FT601.
The VexRiscv CPU writes TLP data/flag words into `PCILeechFIFO` and parses
the mux response frames. This isolates the PCILeech protocol from the USB
transport and is useful if the ECP5 port wants to preserve PCILeech
compatibility before an FT601 shield exists.

### Architecture

```text
VexRiscv
  |-- PCILeechUsbBridge TX/RX FIFOs
  v
PCILeechFIFO (TLP TX, RX128, mux, serializer)
  |
  +-- S7PCIEPHY --> host memory
  |
  +-- CPU parses mux frames --> UART
```

### Components

- Top level: `soc_pcileech.py`.
- Protocol: `gateware/pcileech_fifo.py`, `gateware/tlp128.py`.
- CPU bridge: `gateware/pcileech_bridge.py`.
- Console: `software/soc_pcileech_console/`.
- No FT601; the CPU supplies the words that FT601 would normally carry.

### Build and run

```sh
source env.sh
venv/bin/python soc_pcileech.py --build
./program.sh build/soc_pcileech/gateware/soc_pcileech.bin
./rescan.sh
```

Serial console commands: `help`, `status`, `dump [addr] [len]`.
The verified saved image is `bitstreams/soc_pcileech-console.bin`
(SHA-256 `c5bd55ec75e618bcf11bfcb8ff9ed4f050fc3f52f576ad1bcc34ec159f236984`).

### Measured results

- Hardware VESA dump matched the same SHA-256 as V1 and V2.
- `dump 0x1000000 0x40` completed with no mux/TLP error.
- Bridge simulation and integration tests pass.
- Resource use: 6,208 LUTs, 4,213 registers, 37/50 BRAM tiles.

### Limitations

- One single-DWORD MRd at a time, driven and parsed by the CPU.
- Serial console is the bottleneck; this is a protocol bring-up path.
- No MSI, host writes, multi-DWORD MRds, or PCILeech host init.
- The mux/frame parser expects the PCILeech response framing and is more
  complex than the raw LitePCIe path used by V4/V5.

### ECP5 implication

This is an optional ECP5 step after V2. It answers the question "can the
ECP5 run the PCILeech protocol without FT601?" It does not need the FT601
shield, but it also does not provide the PCILeech host's throughput. Use
it to validate protocol reuse; use V4/V5 for search throughput.

## V4 - SoC with double-buffer DMA

### What it proves

A 4 KiB MRd64 burst plus a two-buffer ping-pong is enough to read host
memory at PCIe speed while the CPU scans the other buffer. This variant
introduced the CPU-visible buffer window and the optional scan stride.

### Architecture

```text
ChunkDMA
  |-- 2 x 4 KiB BRAM buffers
  |-- one outstanding 4 KiB MRd64
  |-- split-CplD completion parser
  v
S7PCIEPHY --> host memory

CPU reads buffer 0/1 through the read-only window at 0x20000000
```

### Components

- Engine: `gateware/chunk_dma.py`.
- Top level: `soc_search.py` (the same top level also contains V5).
- Legacy console: `e2ba260:software/soc_search_console/main.c`.
- CPU window: `0x20000000`, size `0x2000` (two buffers).

### Build and run

The current `soc_search.py --build` image contains both V4 and V5.
`bench_select=0` selects `ChunkDMA`; `bench_select=1` selects `ReadBench`.
The current console's `scan` uses `ReadBench`, so a full V4 scan needs the
legacy console above. The current console's `peek` command reads the
ChunkDMA buffers, which is enough for a buffer sanity check.

```sh
source env.sh
venv/bin/python soc_search.py --build
./program.sh build/soc_search/gateware/soc_search.bin
./rescan.sh
```

### Measured results

- The 2-buffer path is hardware-verified: low VESA scan matched at
  `0x3f000`, and the high region matched at `0x100000000`.
- Raw two-buffer DMA reached about 319 MiB/s.
- CPU scanning at 16-byte stride was about 8-9 MiB/s. The pre-window CSR
  path took 122.8 s for 1 GiB (about 8.3 MiB/s); the CPU-visible window
  removed the CSR read overhead but the 16-byte scan remains CPU-bound.
- One outstanding 4 KiB request is the throughput limit at sparse
  strides. That limitation motivated V5.

### Limitations

- One outstanding request; no tag pipelining.
- The current unified console does not run a full scan through ChunkDMA;
  use the legacy console or V5.
- The CPU is the bottleneck at dense strides.

### ECP5 implication

After V2, V4 is the natural second DMA step. It needs a correct 4 KiB
MRd64 path, split-completion handling, and a CPU-visible buffer. It does
not need FT601. If the ECP5 PCIe block cannot support four outstanding
tags reliably, V4 is the fallback search engine.

## V5 - SoC with 4-outstanding search

### What it proves

Four tags and four 4 KiB buffers can keep enough PCIe reads in flight to
reach the PCILeech-over-USB search target, while the CPU scans completed
buffers and the hardware reissues chunks into freed tags. This is the
highest-throughput on-chip search variant.

### Architecture

```text
ReadBench
  |-- tags 0..3, each with a 4 KiB BRAM buffer
  |-- up to 4 outstanding 4 KiB MRd64 requests
  |-- per-tag pending-DWORD and byte-enable handling
  |-- tag-ready / release ring for the CPU scanner
  v
S7PCIEPHY --> host memory

CPU scans each completed buffer through the 4-buffer read-only window
at 0x20008000 (16 KiB)
```

### Components

- Engine: `gateware/read_bench.py`.
- Top level: `soc_search.py` (shared with V4).
- Console: `software/soc_search_console/main.c`.
- CPU window: `0x20008000`, size `0x4000` (four buffers).

Recent fix: the 4-outstanding write/commit path was repaired in commit
`de0536f` (2026-10-31). The write stage is now driven into the per-tag
BRAMs, the completion tag is applied on the header cycle, CplD byte
enables are honored, and the CPU window is a full 12-bit word address so
all four buffers are visible. The earlier handover document describes the
failure as unresolved and is historical.

### Build and run

```sh
source env.sh
venv/bin/python soc_search.py --build
./program.sh build/soc_search/gateware/soc_search.bin
./rescan.sh
```

Console commands:

- `scan <addr64> <len64> <w0> <w1> <w2> <w3> [stride]`
- `bench <addr64> <len64>` (raw 4-outstanding read benchmark)
- `wtest` / `rpeek` (direct buffer write/readback diagnostics)
- `peek` (ChunkDMA/V4 buffer diagnostics)

The current verified image is
`build/soc_search/gateware/soc_search.bin`
(SHA-256 `1b4e739acb8318fbea3107ce3901d52e46a5231fde8ce0c692e4db3873c7eaee`).
The images under `bitstreams/soc_search-*.bin` are older bring-up
snapshots.

### Measured results (Artix-7 Squirrel)

- Raw 1 GiB benchmark: 2.4 s, 426.7 MiB/s.
- 1 GiB scan, stride `0x400` (1/1024): 3.1 s, about 330 MiB/s.
- 6 GiB scan, stride `0x400`: 18.2 s, about 338 MiB/s.
- 1 GiB scan, stride `0x100` (1/256): 8.9 s, about 115 MiB/s
  (CPU-bound).
- 1 GiB scan, stride `0x10` (1/16): about 117 s, about 8.8 MiB/s
  (CPU-bound).
- Low VESA scan matched at `0x3f000`; high 4 GiB scan matched at
  `0x100000000`; both still matched after the 6 GiB run.
- Resource use of the current unified `soc_search` build: 3,902 LUTs,
  3,820 registers, 22/50 BRAM tiles (20 RAMB36 + 4 RAMB18).

The sparse-stride search rate meets or exceeds the 149 MiB/s
PCILeech-over-USB reference. Dense strides are CPU-limited, not DMA-limited.

### Limitations

- Dense-stride scans are limited by the VexRiscv scan loop.
- The current console maps tags to chunks by release order; that assumes
  the CPU releases tags in ring order and the engine reissues into the
  lowest free tag. This is true for the current scanner, but it should be
  stated explicitly if the console is changed.
- The debug CSRs and `wtest`/`rpeek` are intentionally retained for
  bring-up; they are not part of the final product interface.
- The 4-outstanding path depends on the PCIe hard IP tolerating four
  tagged 4 KiB reads with split completions.

### ECP5 implication

V5 is the highest-value but not the first ECP5 step. It needs the ECP5 PCIe
endpoint to support multiple outstanding tagged reads and split
completions at least as well as the S7 hard IP. If that is not reliable,
V4 is the safe search fallback. It does not need FT601; the serial console
reports progress and matches while PCIe carries the data.

## ECP5 stepping-stone roadmap

This section is a plan, not an implementation. No ECP5 code exists in this
tree yet.

### Step 0 - ECP5 platform and PCIe PHY

Required before any variant can run:

- an ECP5 platform file with the Versa pinout, PCIe reference clock, and
  the FT2232 JTAG/UART pins;
- an ECP5 PCIe endpoint integration. The vendored LitePCIe in this tree
  supports S7, UltraScale+, Cyclone V, Gowin GW5A, and Lattice
  CertusPro-NX PHYs, but not ECP5. This needs either a new ECP5 hard-PCIe
  wrapper/LitePCIe PHY or another proven endpoint core;
- a constraints/timing strategy for the ECP5 hard PCIe block and the
  100 MHz system clock;
- a host-visible device ID and BAR layout, ideally matching the Squirrel
  (`10ee:0666`, BAR0 `0x40000`) so the existing console software and
  scripts need minimal changes.

This is the largest unknown. The SoC variants are only as portable as the
ECP5 PCIe endpoint underneath them.

### Step 1 - V2 single-word SoC

First functional target. It validates:

- PCIe enumeration and auto-BME on ECP5;
- MRd32 below 4 GiB and MRd64 above 4 GiB;
- completion parsing and byte order;
- the FT2232 UART console;
- the VexRiscv/ROM/SRAM/CSR integration.

No FT601 is involved. The slow FT2232 UART is sufficient because the data
path is PCIe.

### Step 2 - V3 and V4

Once V2 works:

- V3 tests whether the PCILeech FIFO/mux protocol can be reused without
  FT601. It is useful if the ECP5 port wants protocol compatibility before
  the FT601 shield exists.
- V4 adds 4 KiB MRd64 bursts, split-completion handling, two buffers, and
  the CPU-visible window. This is the first real search engine on ECP5.

Success criteria are the same as on the Squirrel: VESA match, high-address
match, no completion errors, and a stable 4 KiB chunk stream.

### Step 3 - V5 and the FT601 shield

V5 (4 outstanding) depends on the ECP5 hard PCIe block's tag/outstanding
behavior. Test V4 first; move to V5 only if the endpoint can keep multiple
tagged 4 KiB reads in flight without completion loss.

The FT601 shield is a separate track:

- it enables V1 and PCILeech-host compatibility on ECP5;
- it provides a high-speed host data path if the search SoC later needs to
  stream more than progress/results over USB;
- it is not required for V2-V5, whose data path is PCIe and whose UART
  only carries control and results.

The shield work includes the FT601 FIFO pinout and clock, USB 3.0
connector and power, signal integrity, and an ECP5 port of the FT601
controller. It should not block the SoC bring-up.

### ECP5 risk list

- No ECP5 LitePCIe PHY in the current tree.
- ECP5 hard-PCIe outstanding/tag capabilities may differ from the S7 IP.
- ECP5 BRAM/LUT budget and 100 MHz timing.
- FT2232 UART enumeration and baud-rate handling.
- FT601 shield electrical design, pin count, and USB 3.0 signal integrity.
- Host software assumes the Squirrel device ID/BAR layout unless changed.

## Reproduction appendix

### Common commands

```sh
source env.sh

# V1
venv/bin/python pcileech_squirrel.py --build

# V2
venv/bin/python soc_squirrel.py --build

# V3
venv/bin/python soc_pcileech.py --build

# V4/V5
venv/bin/python soc_search.py --build

# Flash the resulting .bin, then rescan
./program.sh build/<variant>/gateware/<variant>.bin
./rescan.sh
```

### Verified images and tests

| Variant | Saved image | Tests |
|---|---|---|
| V1 | `bitstreams/pcie_squirrel-cleanup-rx128.bin` | `test/test_pcie.py`, host PCILeech runs |
| V2 | `bitstreams/soc_squirrel-console.bin` | `test/test_tlp_dma.py` |
| V3 | `bitstreams/soc_pcileech-console.bin` | `test/test_pcileech_bridge.py`, `test/test_pcileech_bridge_integration.py` |
| V4 | current `soc_search` image | `test/test_chunk_dma.py` |
| V5 | `build/soc_search/gateware/soc_search.bin` | `test/test_read_bench.py` |

Run the unit tests before and after a rebuild:

```sh
venv/bin/python test/test_read_bench.py
venv/bin/python test/test_chunk_dma.py
venv/bin/python test/test_tlp_dma.py
venv/bin/python test/test_pcileech_bridge.py
venv/bin/python test/test_pcileech_bridge_integration.py
```

### VESA and high-address examples

```text
scan 0x3f000 0x1000 0x41534556 0xf1000300 0x00013000 0xf0220000 0x10
scan 0x100000000 0x1000 <w0> <w1> <w2> <w3> 0x10
```

The high-address pattern is host-memory dependent; re-derive it after a
reboot by reading 16 bytes with an unmatchable scan and inspecting the
buffer with `rpeek`.

## Related documents

- `SOC.md` - V2 single-word SoC design and verification.
- `SOC_PCILEECH.md` - V3 CPU-fed PCILeech FIFO design and verification.
- `PERFORMANCE.md` - V1 PCILeech throughput measurements and history.
- `RESOURCES.md` - FPGA resource tables and FIFO presets.
- `docs/superpowers/specs/2026-10-29-soc-search-design.md` - V4 design.
- `docs/superpowers/specs/2026-10-30-soc-search-throughput-design.md` -
  CPU-visible buffers and the V4/V5 decision point.
- `docs/superpowers/plans/2026-10-30-soc-search-throughput.md` - V4/V5
  implementation plan.
- `docs/HANDOVER-2026-10-31-soc-search-4outstanding.md` - historical
  record of the V5 write-path failure; resolved in `de0536f`.
