# PCIe Squirrel PCILeechFIFO CPU-Fed SoC

Date: 2026-10-28
Status: Implemented and hardware-verified on the MRd32/VESA path

## Overview

`soc_pcileech.py` is a separate top level for the LambdaConcept PCIe
Screamer Squirrel. It keeps the PCIe endpoint and the existing
`PCILeechFIFO`/mux path, but removes the FT601 USB controller and the normal
PCILeech host CMD/CFG initialization. Instead:

- a small VexRiscv CPU boots a custom serial BIOS from on-chip ROM;
- the CPU feeds PCILeech TLP data/flag words to `PCILeechFIFO.usb_rx`
  through a small CSR bridge;
- the CPU reads and parses the mux's 8-word response frames from
  `PCILeechFIFO.usb_tx`;
- the BIOS prints the completion data on the FT4232H channel-B UART.

This differs from the first SoC (`soc_squirrel.py`), which uses a dedicated
`TLPDMA` and bypasses `PCILeechFIFO`. The first SoC and the PCILeech top level
(`pcileech_squirrel.py`) remain unchanged and are selected only by flashing
their own bitstreams.

## Block Diagram

```text
CPU TX word
    │
    ▼
PCILeechUsbBridge TX FIFO (256 × 32)
    │ FT601-style byte swap
    ▼
PCILeechFIFO.usb_rx
    │ rx64 packer
    ▼
tlp_tx_fifo (256 × 32)
    │
    ▼
TLP32To64Packer
    │
    ▼
S7PCIEPHY.tlp_tx ── PCIe Gen2 x1, 64-bit ──► host root complex

S7PCIEPHY.tlp_rx
    │ TLP64To128Packer
    ▼
TLP128Filter
    │
    ▼
tlp_rx_fifo (2048 × 128)
    │
    ▼
PCILeechMux (2048 × 256)
    │
    ▼
MuxSerializer (8 × 32)
    │
    ▼
PCILeechFIFO.usb_tx
    │ FT601-style byte swap
    ▼
PCILeechUsbBridge RX FIFO (1024 × 32)
    │
    ▼
CPU mux/CplD parser ──► UART /dev/ttyUSB1 @ 115200
```

The CPU is not a DMA engine. Each DWORD is one single-DWORD MRd followed by a
synchronous wait for its completion.

## Project Components Used

| Component | Role |
|---|---|
| `soc_pcileech.py` | Variant top level: 100 MHz CRG, `SoCCore`, PCIe PHY, auto-BME, `PCILeechFIFO` large, CPU bridge, custom BIOS packaging |
| `platforms/pcie_squirrel.py` | Squirrel pinout/constraints: 100 MHz clock, PCIe x1, FT4232H serial on G22/G21, LEDs/buttons, unused FT601 pins |
| `gateware/pcileech_fifo.py` | Existing PCILeech wire protocol, TLP TX/RX, CplD filter, mux, serializer, timeout hooks |
| `gateware/pcileech_bridge.py` | New CPU-visible stream adapter: TX/RX FIFOs, byte swap, link status CSRs |
| `gateware/tlp128.py` | `TLP64To128Packer`, `TLP128Filter`, 128-bit TLP layout used by the PCILeech RX path |
| `gateware/tlp32.py` | `TLP32To64Packer`, shared with the first SoC and PCILeech top level |
| `software/soc_pcileech_console/main.c` | No-libc BIOS: UART console, `help`, `status`, `dump [addr] [len]`, PCILeech TX builder, mux/CplD parser |
| `software/soc_pcileech_console/Makefile` | Builds the custom `bios.bin` with the LiteX common make rules |
| `software/soc_pcileech_console/linker.ld` | LiteX BIOS linker layout (ROM/SRAM/CSR regions) |
| `test/test_pcileech_bridge.py` | Migen simulation of bridge FIFO ordering, byte swap, and status |
| `test/test_pcileech_bridge_integration.py` | Bridge-to-`PCILeechFIFO` TLP data/flag integration simulation |

Not used by this variant:

- `gateware/ft601.py` (`FT601Sync`);
- the PCILeech host CMD/CFG/loopback init sequence;
- `gateware/tlp_dma.py` (`TLPDMA`) and the first SoC TLPDMA page buffer;
- `gateware/msi.py` (no MSI);
- `gateware/tlp.py`, `gateware/usb.py`, `gateware/etherbone.py`;
- the old `pcie_squirrel.py` / `pcie_screamer.py` top levels.

## LiteX and LitePCIe Components

| LiteX component | Use |
|---|---|
| `litex.soc.integration.soc_core.SoCCore` | CPU bus, CSR map, UART, integrated ROM/SRAM regions |
| `litex.soc.cores.cpu.vexriscv` | VexRiscv `minimal` CPU (`VexRiscv_Min`, RV32I, 32-bit Wishbone) |
| `litex.soc.cores.clock.S7PLL` | 100 MHz sys clock from the board 100 MHz oscillator |
| `litex.soc.cores.uart` / `RS232PHY` | 115200 8N1 serial console |
| `litex.soc.interconnect.csr` | `CSRStorage`, `CSRStatus`, `AutoCSR`, generated `csr.h` |
| `litex.soc.interconnect.stream` | `Endpoint`, `SyncFIFO` for the CPU bridge, PCILeechFIFO, and TLP paths |
| `litex.soc.integration.builder.Builder` | Software/ROM/gateware build; local subclass installs the custom BIOS package and skips the unused meson/libc path |
| LiteX software build files | `crt0.S`, `common.mak`, linker script, generated `soc.h`/`csr.h`/memory regions |

| LitePCIe component | Use |
|---|---|
| `litepcie.phy.s7pciephy.S7PCIEPHY` | Xilinx 7-series PCIe Gen2 x1 endpoint, 64-bit datapath, BAR0 0x40000 |
| `litepcie.common.phy_layout` | 64-bit and 32-bit PCIe TLP stream layouts |

Migen supplies `Module`, `Signal`, `FSM`, `Case`, `Memory`, `Cat`, `Constant`,
`Mux`, and `migen.sim` for the bridge simulations.

## PCILeechFIFO and Bridge Architecture

`PCILeechFIFO` is instantiated in the large configuration:
`mux_out_fifo_depth=2048`, `tlp_rx_fifo_depth=2048`. Its internal
`tlp_tx_fifo` is 256 × 32. The CPU-facing bridge is a separate CSR
peripheral at `0xf0001000`; `pcie_id` remains at `0xf0001800`.

`PCILeechUsbBridge` is a stream adapter, not a DMA engine:

| CSR | Offset | Access | Meaning |
|---|---:|---|---|
| `tx_data` | `0x00` | write | Word toward `PCILeechFIFO.usb_rx`; the write strobe pushes the TX FIFO |
| `tx_full` | `0x04` | read | TX FIFO full |
| `rx_data` | `0x08` | read | Word from `PCILeechFIFO.usb_tx`; reading pops the RX FIFO |
| `rx_empty` | `0x0c` | read | RX FIFO empty |
| `status` | `0x10` | read | bit0 `link_ready`, bit1 `tx_full`, bit2 `rx_empty` |

- TX FIFO: 256 × 32-bit.
- RX FIFO: 1024 × 32-bit (34 bits internally).
- `link_ready = link_up & bus_master_enable & (pcie_id != 0)`.
- Both directions apply the FT601-style 32-bit byte swap, so the CPU sees the
  same word order the PCILeech wire protocol expects.
- The bridge exposes `link_ready`; the BIOS uses it to wait for enumeration and
  BME before sending TLPs.

Auto-BME reuses the first SoC's one-shot sequence: after link-up it waits
about 10 ms, then writes `0x6` to Device Control (bus master + memory space).
No host-side driver or CMD/CFG init is required. `PCILeechFIFO` retains its
reset values: `rw[200]=0` (core out of reset), `rw[202]=1` (CplD filter
enabled), `rw[16]=0` (inactivity keepalive disabled).

## TX Data/Flag Framing

The CPU writes one TLP DWORD followed by one flag word for every DWORD:

1. the TLP DWORD;
2. a flag word with magic `0x77` in the high byte, type bits `00` (TLP), and
   `last=1` on the final DWORD (`TLP_FLAG_LAST = 1 << 18`).

The FIFO's `rx64` packer takes the flag word as the upper half and the
preceding data word as `pkt_data`, so the data word must always be written
first. The CPU sends one MRd at a time and waits for its CplD before starting
the next DWORD.

| Request | DWORDs |
|---|---|
| MRd32, address below 4 GiB | DW0 `0x00000001` (Fmt=00, 3-DW header, length=1); DW1 `{requester_id, rotating 8-bit tag, first_be=0xF}`; DW2 `addr[31:0]`, 4-byte aligned, `last` on DW2 |
| MRd64, address at or above 4 GiB | DW0 `0x20000001` (Fmt=01, 4-DW header, length=1); DW1 `{requester_id, rotating 8-bit tag, first_be=0xF}`; DW2 `addr[31:0]`; DW3 `addr[63:32]`, `last` on DW3 |

The 8-bit tag rotates for every DWORD request (`mrd_tag++`). The requester ID
comes from the `pcie_id` CSR. Addresses must be 4-byte aligned; the console
rejects unaligned addresses before sending.

No CMD, CFG, or loopback frames are sent. The VESA and arbitrary-read tests
use MRd32; MRd64 is implemented but the hardware test exercised low memory.

## RX Sync, Frames, and Context Mapping

The mux serializer emits five `0x66665555` sync words at the start of a USB
burst, then 8-word frames:

- word 0: context-nibble header (nibble 7:4 is `0xE`);
- words 1–7: data words 0–6.

Context-nibble mapping:

```text
bits 31:28 ctx[1]   bits 27:24 ctx[0]
bits 23:20 ctx[3]   bits 19:16 ctx[2]
bits 15:12 ctx[5]   bits 11: 8 ctx[4]
bits  7: 4 0xE      bits  3: 0 ctx[6]
```

The BIOS parser:

- drains the bridge RX FIFO at dump start so stale burst data is discarded;
- skips sync words and idle filler (`ctx == 0xF`); a sync discards any partial
  CplD state;
- collects data words with TLP context (`ctx & 0x3 == 0`);
- uses `ctx & 0x4` as the final-DWORD flag (the Migen mux puts `tlp_last` in
  context bit 2);
- locates a 1-DWORD CplD (four DWORDs: three header DWORDs plus data);
- verifies CplD type `0x4A`, length 1, byte count 4, completion status, tag,
  requester ID, and lower address bits;
- handles a no-data Cpl (`0x0A`) by reporting its status, treating status 0 as
  malformed;
- extracts the data DWORD and prints it MSB-first so the serial output is host
  memory byte order;
- ignores CMD/CFG/loopback contexts (none are generated in this variant).

The parser keeps only a small line/frame buffer, not a full 4 KiB page. The
RX path has a 20,000,000-poll timeout and a 65,536-word frame budget; a
malformed frame, unexpected TLP, tag/ID/address/byte-count mismatch, or
completion error stops the dump and prints a reason.

## Memory Usage

| Region | Base | Configured size | Used now |
|---|---:|---:|---:|
| ROM | `0x00000000` | `0x8000` (32 KiB) | 3,935 B BIOS image (12.0%) |
| SRAM | `0x10000000` | `0x2000` (8 KiB) | 136 B `.bss`; `.data` 0 B; stack grows down from `0x10002000` |
| CSR | `0xf0000000` | `0x10000` (64 KiB) | PCIe PHY, CPU bridge, `pcie_id`, UART registers |
| `PCILeechFIFO` TLP RX | internal | 2048 × 128 | PCILeech CplD/TLP frame storage |
| `PCILeechFIFO` mux | internal | 2048 × 256 | 7-word response frames plus context |
| Bridge TX FIFO | internal | 256 × 32 | CPU-to-FIFO word staging |
| Bridge RX FIFO | internal | 1024 × 32 (34-bit internal) | FIFO-to-CPU word staging |

The physical ROM is sized to the BIOS image (984 × 32-bit words, 3,936 B)
while the bus/linker ROM region remains 32 KiB. There is no heap; the CPU SRAM
is used only for the stack and small static state.

## FPGA Resource Use

Authoritative table and history: `RESOURCES.md`.

| Resource | Used | Available | Util |
|---|---:|---:|---:|
| Slice LUTs | 6,208 | 20,800 | 29.85% |
| — LUT as logic | 4,881 | 20,800 | 23.47% |
| — LUT as memory | 1,327 | 9,600 | 13.82% |
| Slice registers | 4,213 | 41,600 | 10.13% |
| Block RAM tiles | 37 | 50 | 74.00% |
| — RAMB36 | 34 | 50 | 68.00% |
| — RAMB18 | 6 | 100 | 6.00% |
| DSP slices | 0 | 90 | 0.00% |
| Bonded IOB | 4 | 250 | 1.60% |
| BUFGCTRL | 8 | 32 | 25.00% |

The bridge FIFOs did not consume BRAM. In particular, the 1024 × 34 RX FIFO
was inferred as distributed LUTRAM (`RAM64M` × 176), not block RAM. This keeps
the BRAM total at 37/50 tiles (74%) while increasing LUT-as-memory use; the
1,316 distributed-RAM LUTs are mostly the two CPU bridge FIFOs. BRAM remains
the tighter resource, with 13 tiles spare.

Timing: WNS +0.525 ns, WHS +0.040 ns, WPWS -0.007 ns on the known PCIe
hard-IP `PIPECLK → USERCLK` skew; 4 DRC warnings (CFGBVS, two RAMB async
controls, one unroutable/unused net), 0 errors. The WPWS warning is a
hard-IP bring-up caveat, not a CPU/SoC-logic failure.

## Build, Flash, and Console

```sh
source env.sh
venv/bin/python soc_pcileech.py --build
```

Outputs:

- `build/soc_pcileech/gateware/soc_pcileech.bin` — 1,280,396 bytes,
  SHA-256 `c5bd55ec75e618bcf11bfcb8ff9ed4f050fc3f52f576ad1bcc34ec159f236984`
- `build/soc_pcileech/software/bios/bios.bin` — 3,935 bytes, no undefined
  symbols
- saved image: `bitstreams/soc_pcileech-console.bin`

Flash and rescan:

```sh
./program.sh bitstreams/soc_pcileech-console.bin
./rescan.sh
lspci -d 10ee:0666
```

Serial console:

```sh
picocom -b 115200 /dev/ttyUSB1
```

Restore the PCILeech RX128 fallback:

```sh
./program.sh bitstreams/pcie_squirrel-cleanup-rx128.bin
./rescan.sh
```

The first SoC can be restored with
`bitstreams/soc_squirrel-console.bin` in the same way.

## Verification

- `test/test_pcileech_bridge.py`: bridge FIFO ordering, byte swap, status, and
  read-strobe behavior pass.
- `test/test_pcileech_bridge_integration.py`: TLP data/flag words reach
  `PCILeechFIFO.tlp_tx` with the correct data and `last` flag.
- `test/test_tlp_dma.py`: first-SoC TLPDMA regression passes.
- `soc_pcileech.py` and `gateware/pcileech_bridge.py` compile; `git diff
  --check` is clean.
- Hardware `status`:
  `bridge link=1 tx_full=0 rx_empty=1`,
  `pcie link=0x163 bme=1 id=0x1600`.
- Default VESA `dump` matched SHA-256
  `5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db`.
- `dump 0x1000000 0x40` returned 64 bytes with no mux/TLP error; the host
  remained alive and the device stayed enumerated.
- The PCILeech RX128 fallback was reflashed and a live VESA dump matched the
  same hash.

## Accepted Risks

- No CPU DMA, host writes, multi-DWORD MRds, CMD/CFG/loopback host emulation,
  or MSI. The serial console is the bottleneck.
- The PCILeechFIFO burst gate adds warmup/settle latency before frames flow.
- Auto-BME is one-shot after link-up; a host that clears BME later is not
  automatically re-enabled.
- The known hard-IP WPWS -0.007 ns and four DRC warnings are accepted
  bring-up caveats.
- The bridge RX FIFO's LUTRAM inference increases LUT-as-memory use; it is
  accepted because BRAM remains the scarcer resource.
- MRd64 above 4 GiB is implemented but only low-memory MRd32 was exercised on
  hardware.
- The PCILeechFIFO timeout watchdog `force_pcie_reset` is not wired to the
  PCIe reset; a stalled TX path needs a manual reflash/reset.

## Related Documents

- `SOC.md` — first VexRiscv console SoC with `TLPDMA`.
- `docs/superpowers/specs/2026-10-28-soc-pcileech-design.md` — design and
  implementation result.
- `docs/superpowers/plans/2026-10-28-soc-pcileech.md` — implementation plan.
- `RESOURCES.md` — authoritative FPGA resource tables and FIFO/preset data.
- `PERFORMANCE.md` — PCILeech throughput results; not the CPU-fed SoC path.
