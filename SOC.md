# PCIe Squirrel VexRiscv Console SoC

Date: 2026-10-28
Status: Implemented and hardware-verified on the MRd32/VESA path

## Overview

`soc_squirrel.py` is a separate top level for the LambdaConcept PCIe
Screamer Squirrel. It keeps the PCIe endpoint but removes the FT601 USB
controller and the `PCILeechFIFO` USB/mux path. Instead:

- a small VexRiscv CPU boots a custom serial BIOS from on-chip ROM;
- the CPU controls a hardware TLP DMA through CSRs;
- the DMA issues 64-bit host physical memory reads and captures the
  completions into a 4 KiB page buffer;
- the BIOS prints the result on the FT4232H channel-B UART.

The existing PCILeech top level (`pcileech_squirrel.py`) is unchanged in
function and remains the normal high-throughput build. The SoC is selected
only by flashing its separate bitstream.

## Block Diagram

```text
host PCIe root complex
        │  PCIe Gen2 x1, 64-bit
        ▼
   S7PCIEPHY ── TLPDMA ── 4 KiB page buffer
        │          │
        │      Wishbone/CSR bus
        │          │
        │       VexRiscv (minimal)
        │          │
        │       UART / RS232PHY
        │          │
        │      FT4232H channel B
        │          │
        │       /dev/ttyUSB1 @ 115200
        └── BAR0 / config space
```

The CPU does not DMA into its 8 KiB SRAM. It starts the TLP DMA, waits for
`done`, then reads the 4 KiB DMA page buffer through `rd_index`/`rd_data`.

## Project Components Used

From this repository:

| Component | Role |
|---|---|
| `soc_squirrel.py` | SoC top level: 100 MHz CRG, `SoCCore`, PCIe PHY, auto-BME, TLPDMA, custom BIOS packaging |
| `platforms/pcie_squirrel.py` | Squirrel pinout/constraints: 100 MHz clock, PCIe x1, FT4232H serial on G22/G21, LEDs/buttons, FT601 pins (unused by the SoC) |
| `gateware/tlp_dma.py` | CSR-controlled sequential single-DWORD TLP DMA, MRd32 below 4 GiB and MRd64 above, 4 KiB page buffer |
| `gateware/tlp32.py` | `TLP32To64Packer`, moved from `pcileech_squirrel.py` and shared by both top levels |
| `software/soc_console/main.c` | No-libc BIOS: UART console, `help`, `status`, `dump [addr] [len]`, DMA driver |
| `software/soc_console/Makefile` | Builds the custom `bios.bin` with the LiteX common make rules |
| `software/soc_console/linker.ld` | LiteX BIOS linker layout (ROM/SRAM/CSR regions) |
| `test/test_tlp_dma.py` | Migen simulation coverage for TLPDMA |
| `pcileech_squirrel.py` | Existing PCILeech/FT601 top level; not instantiated by the SoC, but shares `gateware/tlp32.py` |

Not used by the SoC:

- `gateware/ft601.py` (`FT601Sync`)
- `gateware/pcileech_fifo.py` (`PCILeechFIFO`, mux, TLP RX128 path)
- `gateware/tlp128.py`
- `gateware/msi.py` (the SoC does not use MSI)
- `gateware/tlp.py`, `gateware/usb.py`, `gateware/etherbone.py`
- the old `pcie_squirrel.py` / `pcie_screamer.py` top levels

## LiteX Components Used

| LiteX component | Use |
|---|---|
| `litex.soc.integration.soc_core.SoCCore` | CPU bus, CSR map, UART, integrated ROM/SRAM regions |
| `litex.soc.cores.cpu.vexriscv` | VexRiscv `minimal` CPU (`VexRiscv_Min`, RV32I, 32-bit Wishbone) |
| `litex.soc.cores.clock.S7PLL` | 100 MHz sys clock from the board 100 MHz oscillator |
| `litex.soc.cores.uart` / `RS232PHY` | 115200 8N1 serial console |
| `litex.soc.interconnect.csr` | `CSRStorage`, `CSRStatus`, `AutoCSR`, generated `csr.h` |
| `litex.soc.interconnect.stream` | Stream endpoints between the PCIe PHY, packer, and DMA |
| `litex.soc.integration.builder.Builder` | Software/ROM/gateware build; a small local subclass installs the custom BIOS package and avoids the unused meson/libc path |
| LiteX software build files | `crt0.S`, `common.mak`, linker script, generated `soc.h`/`csr.h`/memory regions |

From LitePCIe:

| Component | Use |
|---|---|
| `litepcie.phy.s7pciephy.S7PCIEPHY` | Xilinx 7-series PCIe Gen2 x1 endpoint, 64-bit datapath, BAR0 0x40000 |
| `litepcie.common.phy_layout` | 64-bit PCIe TLP stream layout |

From Migen:

- `Module`, `Signal`, `FSM`, `Case`, `Memory`, `ResetInserter`, `Cat`,
  `Constant`, `Mux`; `migen.sim` for the TLPDMA test.

## CPU and Firmware

- CPU: VexRiscv `minimal` / `VexRiscv_Min`, RV32I, 32-bit Wishbone, 100 MHz.
- Firmware: custom no-libc BIOS, built as `software/soc_console`.
- ROM image: 2,728 bytes.
- Static SRAM: `.text` 2,728 B in ROM, `.data` 0 B, `.bss` 128 B.
- No heap; the stack starts at the top of SRAM.
- UART: 115200 8N1 on `/dev/ttyUSB1`; RX FIFO is read-strobe mode
  (`uart_rx_fifo_rx_we=True`) because the console polls and does not use the
  LiteX UART event manager.
- Console commands:
  - `help`
  - `status` — TLP status, PCIe link status, BME, requester ID
  - `dump [addr] [len]` — default `0x3f000`, `0x1000`; 64-bit hex address;
    length 4–0x1000 bytes, 4-byte aligned; prints 16 bytes/line in memory
    byte order; no writes.

## Memory Map and Usage

| Region | Base | Configured size | Used now |
|---|---:|---:|---:|
| ROM | `0x00000000` | `0x8000` (32 KiB) | 2,728 B image (8.3%) |
| SRAM | `0x10000000` | `0x2000` (8 KiB) | 128 B static `.bss`; stack grows down from `0x10002000` |
| CSR | `0xf0000000` | `0x10000` (64 KiB) | PCIe PHY, TLPDMA, `pcie_id`, UART registers |
| TLPDMA page buffer | internal | 4 KiB (1024 × 32) | one 4 KiB page per dump |

Key CSRs:

| Block | Base |
|---|---:|
| `pcie_phy` | `0xf0000000` |
| `tlp_dma` | `0xf0001000` |
| `pcie_id` | `0xf0001800` |
| `uart` | `0xf0002000` |

LiteX auto-resizes the physical ROM to the BIOS image (`0xAA8`, 2,728 B)
while the bus/linker ROM region remains 32 KiB. The 8 KiB SRAM is the CPU
scratchpad; the 4 KiB TLPDMA page buffer is separate BRAM and is not part of
the CPU SRAM.

## TLP DMA

The TLPDMA is a Wishbone CSR peripheral:

| CSR | Access | Meaning |
|---|---|---|
| `addr_lo` | RW | low 32 bits of host physical address |
| `addr_hi` | RW | high 32 bits |
| `length` | RW | DWORD count, 1–1024 |
| `start` | RW | write strobe; ignored while busy |
| `status` | RO | bit0 link-ready, bit1 busy, bit2 done, bit3 error |
| `error` | RO | 1 not-ready, 2 completion error, 3 malformed, 4 timeout, 5 invalid length |
| `rd_index` | RW | page-buffer DWORD index |
| `rd_data` | RO | page-buffer DWORD at `rd_index` |

Behavior:

- `link_ready = PCIe link up & Bus Master Enable & (pcie_phy.id != 0)`.
- 8-bit per-request tag; completion must match requester ID, tag, lower
  address, CplD type/length, status, and data byte enables.
- MRd32 for addresses below 4 GiB; MRd64 above.
- One outstanding request; 10 ms completion timeout; stale completions are
  dropped.
- The page buffer holds 1024 DWORDs (4 KiB); the CPU reads it after `done`.

## FPGA Resource Use

Authoritative table and history: `RESOURCES.md`.

| Resource | Used | Available | Util |
|---|---:|---:|---:|
| Slice LUTs | 2,050 | 20,800 | 9.86% |
| Slice registers | 2,470 | 41,600 | 5.94% |
| Block RAM tiles | 16 | 50 | 32.00% |
| — RAMB36 | 14 | 50 | 28.00% |
| — RAMB18 | 4 | 100 | 4.00% |
| DSP slices | 0 | 90 | 0.00% |
| Bonded IOB | 4 | 250 | 1.60% |
| BUFGCTRL | 8 | 32 | 25.00% |

Hierarchy split:

| Block | LUTs | FFs | BRAM |
|---|---:|---:|---:|
| VexRiscv | 1,109 | 748 | 2 × RAMB18 |
| PCIe hard IP (`pcie_s7`) | 395 | 752 | 8 × RAMB36 |
| Top level/other (CRG, UART, TLPDMA, ROM/SRAM) | 548 | 970 | 6 × RAMB36 + 2 × RAMB18 |

The PCIe hard IP is the largest BRAM consumer. The SoC fits with substantial
LUT/FF margin; BRAM is the tighter resource at 16/50.

Timing/build caveat: WNS +0.686 ns, WHS +0.032 ns, WPWS -0.007 ns on one
PCIe hard-IP `PIPECLK → USERCLK` endpoint; 5 DRC warnings (CFGBVS, RAMB async
control, unused nets), 0 errors. These are accepted bring-up caveats, not
SoC-logic failures.

## Build and Flash

```sh
source env.sh
venv/bin/python soc_squirrel.py --build
```

Outputs:

- `build/soc_squirrel/gateware/soc_squirrel.bin` — 822,836 bytes,
  SHA-256 `25a439bed48e14ad0720a986678338cb51a147225da1f00172e36d29f57031df`
- `build/soc_squirrel/software/bios/bios.bin` — 2,728 bytes
- saved image: `bitstreams/soc_squirrel-console.bin`

Flash and rescan:

```sh
./program.sh bitstreams/soc_squirrel-console.bin
./rescan.sh
```

Serial console:

```sh
picocom -b 115200 /dev/ttyUSB1
```

Restore the PCILeech build:

```sh
./program.sh bitstreams/pcie_squirrel-cleanup-rx128.bin
./rescan.sh
```

## Verification

- Default `dump` read the VESA page and matched SHA-256
  `5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db`.
- `dump 0x1000000 0x40` completed with no DMA error; the host remained alive
  and the PCIe device stayed enumerated.
- `status` after rescan: `link=0x163`, `bme=1`, `id=0x1600`, `tlp=0x01`.
- PCILeech RX128 fallback was reflashed and verified with a live VESA dump
  matching the same hash, 1/1 page and 0 failures.

Accepted limitations:

- no independent ISSUE-state timeout; a stalled PCIe TX path needs a reset;
- 8-bit request tag wraps after 256 requests;
- MRd64 above 4 GiB is simulation-verified only; hardware used MRd32 VESA/low
  memory;
- known hard-IP WPWS/DRC caveats above.

## Related Documents

- `docs/superpowers/specs/2026-10-28-soc-console-design.md` — design.
- `docs/superpowers/plans/2026-10-28-soc-console.md` — implementation plan and
  verification notes.
- `RESOURCES.md` — authoritative FPGA resource tables and FIFO/preset data.
- `PERFORMANCE.md` — PCILeech throughput results, not the SoC path.
