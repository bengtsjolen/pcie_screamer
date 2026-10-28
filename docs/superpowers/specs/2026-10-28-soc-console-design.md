# PCIe Squirrel VexRiscv Console SoC Design

Date: 2026-10-28
Status: Implemented; MRd32/VESA path hardware-verified

## Context

The existing `performance-tuning` branch has a verified PCILeech build with the
FT601 USB path and the 128-bit TLP RX path. This design adds a second,
independent top level for the same Squirrel board:

- a minimal LiteX SoC with a small VexRiscv CPU;
- a custom serial console BIOS;
- the existing PCIe Gen2 x1 endpoint;
- no FT601 controller and no `PCILeechFIFO` USB/mux path.

The CPU becomes the PCIe initiator. It uses a small hardware TLP DMA to read a
host physical page, then prints the data over the FT4232H channel-B serial
port. The default target is the known-safe VESA ROM page at `0x3f000`; an
explicit 64-bit physical address may also be supplied.

The existing PCILeech top level remains available. The SoC bitstream is a
separate build and replaces the PCILeech bitstream only when flashed.

## Goals

- Add a `soc_squirrel.py` top level with a minimal VexRiscv SoC and serial
  console.
- Keep the PCIe endpoint and auto-BME behavior, but remove FT601 and
  `PCILeechFIFO` from this build.
- Add a hardware page-burst TLP DMA controllable from the CPU.
- Provide a custom minimal BIOS console with `dump [addr] [len]`.
- Read the default VESA page and print it over serial.
- Fit comfortably on the XC7A35T-FGG484-2.
- Preserve the existing PCILeech build behavior.

## Non-Goals

- No FT601 controller or PCILeech USB wire protocol in this SoC.
- No host writes, no PCIe configuration writes beyond the existing auto-BME.
- No full LiteX BIOS, filesystem, networking, or shell.
- No multi-DWORD MRd, MSI, or interrupt-driven DMA in the first version.
- No single command larger than 4 KiB.
- No performance target; the exercise is functional bring-up.
- No timing-closure claim; the known PCIe/sys constraint caveat is tracked
  separately.

## Resource Feasibility

Estimated BRAM:

| Block | Tiles |
|---|---:|
| PCIe hard IP | 8 |
| 4 KiB TLP DMA page buffer | ~1 |
| 32 KiB BIOS ROM | 8 |
| 8 KiB SRAM | 2 |
| Small FIFOs/CDC | ~1–2 |
| **Total** | **~20–21 / 50** |

The current PCILeech large build uses 41.5 tiles; removing FT601 and the
`PCILeechFIFO` mux/TLP FIFOs leaves ample room for the CPU, ROM, SRAM, and DMA.
The `VexRiscv` `minimal` variant and a no-libc console keep LUT/FF use modest.
The built implementation uses 2,050 LUTs, 2,470 registers, and 16 BRAM tiles;
LiteX auto-resizes the physical ROM from 0x8000 to the 2.7 KiB BIOS.
If the design does not fit, reduce in this order:

1. ROM 32 KiB → 16 KiB and SRAM 8 KiB → 4 KiB.
2. VexRiscv `minimal` → an even smaller feature set only if already available.
3. Page buffer 4 KiB → 2 KiB, then one-DWORD DMA.

## Hardware Architecture

```text
soc_squirrel.py
  ├─ _CRG: 100 MHz sys clock
  ├─ SoCCore: VexRiscv minimal, 32 KiB ROM, 8 KiB SRAM, serial UART
  ├─ S7PCIEPHY: Gen2 x1, 64-bit data path, Device_ID 0666
  ├─ auto-BME after link-up
  └─ TLPDMA: CSR-controlled page-burst DMA
       ├─ TLP32To64Packer → pcie_phy.sink
       └─ pcie_phy.source → CplD parser
```

### SoC

- `SoCCore` with:
  - `cpu_type="vexriscv"`;
  - `cpu_variant="minimal"`;
  - `integrated_rom_size=0x8000`;
  - `integrated_sram_size=0x2000`;
  - `uart_name="serial"`, 115200 baud;
  - `uart_rx_fifo_rx_we=True` so each polling `uart_rxtx` read pops the RX
    FIFO (the custom console does not use the UART event manager);
  - `with_timer=False`, `with_ctrl=False`.
- `check_bios_requirements()` is overridden to require ROM/SRAM but not the
  unused timer.
- The custom console replaces the normal LiteX BIOS package and is built into
  the integrated ROM.
- The FT4232H channel-B serial pins are the ones already committed in
  `platforms/pcie_squirrel.py`.

### PCIe endpoint

- Reuse `S7PCIEPHY` with `data_width=64`, `bar0_size=0x40000`.
- Keep `Device_ID=0666`, class code `02/00`, extended tag settings, and the
  existing GTP placement command.
- Use the post-synthesis clock-group helper:
  `add_false_path_constraints_by_name("main_s7pciephy_clkout*",
  "main_crg_clkout")`; the old `main_clkout` names do not apply to this SoC.
- Do not override `pcie_phy.pcie_rst_n` from a PCILeech CMD register. The PHY
  default and board PERST# (`pads.rst_n`) control reset.
- Auto-BME after link-up remains: wait about `1<<20` cycles after link, then
  write `0x6` to config command register with byte enables `0x3`. The BME
  timer/pulse runs in the pcie domain.
- `link_ready` uses the host-visible `_bus_master_enable.status`, not the
  local `bme_done` pulse, and is qualified by link-up and a nonzero
  `pcie_phy.id`.
- Do not instantiate the MSI module or the PCILeech command/mux path.

### TLPDMA

New module `gateware/tlp_dma.py`.

CSRs:

| CSR | Direction | Purpose |
|---|---|---|
| `addr_lo` | CPU write | Low 32 bits of the host physical address |
| `addr_hi` | CPU write | High 32 bits of the host physical address |
| `length` | CPU write | Number of DWORDs, 1–1024 |
| `start` | CPU write | Self-clearing start trigger |
| `status` | CPU read | `link_ready`, `busy`, `done`, `error` fields |
| `error` | CPU read | Error code |
| `rd_index` | CPU write | Page-buffer read index |
| `rd_data` | CPU read | Page-buffer DWORD at `rd_index` |

Behavior:

- `link_ready = pcie_link_up & bus_master_enable & (pcie_phy.id != 0)`.
- On `start` when idle, latch the 64-bit address and length; `start` is
  ignored while busy. The start address is aligned to 4 bytes.
- A free-running 8-bit tag is latched for each request and consumed at
  request start; the next request uses the next tag.
- Issue sequential single-DWORD MRd TLPs:
  - For addresses below 4 GiB, use MRd32:
    - DW0 `0x00000001` (Fmt=00, 3-DW header, MRd, length=1);
    - DW1 `{requester_id, tag, first_be=0xF}`;
    - DW2 `addr[31:0]` with the low two bits cleared;
    - `last` on DW2.
  - For addresses at or above 4 GiB, use MRd64:
    - DW0 `0x20000001` (Fmt=01, 4-DW header, MRd, length=1);
    - DW1 `{requester_id, tag, first_be=0xF}`;
    - DW2 `addr[31:0]`, DW3 `addr[63:32]`;
    - `last` on DW3.
- Send the words through `TLP32To64Packer` to `pcie_phy.sink`.
- Wait for the completion on `pcie_phy.source`:
  - a 1-DWORD CplD is four DWORDs: three header DWORDs plus data;
  - validate the CplD type `dat[24:32] == 0x4A` and length 1;
  - check completion status bits in DW1 (`status == 0`);
  - validate the DW2 lower address, tag, and requester ID against the
    outstanding request; stale completions are dropped;
  - check the data beat is valid (`last`, `be[4:8] == 0xF`);
  - extract data from the final beat's upper 32 bits.
- Cpl (`0x0A`) no-data completions are drained and matched before reporting
  error code 2 (nonzero status) or 3 (unexpected Cpl).
- Store each completed DWORD into a 1024×32 page buffer, increment the
  address, and continue until `length` DWORDs are complete.
- Set `done` on success; DONE/ERROR/error code are sticky until the next
  start.
- Set `error` and stop on:
  - not ready/link down;
  - malformed or non-CplD completion;
  - completion status not Successful;
  - no completion within 10 ms.
- `rd_index`/`rd_data` expose the completed buffer to the CPU.
- The DMA accepts only one outstanding request at a time.

The page buffer is 1024×32 bits = 4 KiB. `rd_index` selects a DWORD; the
buffer read is registered, so firmware writes `rd_index` and reads `rd_data`
after the CSR access. The CPU reads all requested DWORDs after `done`.

### TLP packer sharing

Move `TLP32To64Packer` from `pcileech_squirrel.py` to
`gateware/tlp32.py`. Update the PCILeech top level to import it. The
implementation is unchanged; this is a behavior-preserving refactor.

## Firmware

New `software/soc_console/` package:

- `main.c` — console and DMA driver.
- `Makefile` — builds `bios.bin` using LiteX `common.mak`.
- `linker.ld` — LiteX BIOS linker script (or a local copy).
- Uses LiteX `crt0.S` from the VexRiscv CPU directory.
- Includes generated `csr.h`; no libc, no `printf`.
- Defines an empty `isr`.

Console:

- 115200 8N1, raw polling UART (`uart_txfull`, `uart_rxempty`,
  `uart_rxtx`); RX FIFO reads pop on the CSR read (`uart_rx_fifo_rx_we`).
- Banner and prompt on boot.
- Commands:
  - `status` — print TLPDMA status, PCIe link status, BME, and the assigned
    requester ID;
  - `dump [addr] [len]`:
    - default address `0x3f000`, default length `0x1000`;
    - address is 64-bit hexadecimal;
    - length is bytes, 4-byte aligned, maximum `0x1000`;
    - firmware converts the byte length to a DWORD count before programming
      the DMA `length` CSR;
    - waits for `link_ready` and reports status if not ready;
    - writes address/length, sets `start`, polls `status`;
    - on `error`, prints the error code and stops;
    - on `done`, reads `rd_index`/`rd_data` and prints 16 bytes per line
      with the physical address, MSB-first within each DWORD so the output
      matches memory byte order;
    - while link-ready is false, prints `status` every ~1M polling cycles;
  - `help` — one-screen summary and the arbitrary-read warning.
- The console warns that arbitrary physical reads may hang or crash the host;
  the VESA default is the safe path.
- No write command is provided.

Build integration:

- After constructing the LiteX `Builder`, replace the default `bios` package
  entry with `("bios", "software/soc_console")` so `BIOS_DIRECTORY` points at
  the custom package.
- The package Makefile outputs `bios.bin`; the normal Builder ROM
  initialization path embeds it.
- Build output lives under `build/soc_squirrel/`; CSR export can use
  `test/soc_csr.csv`.

## Data Flow

1. Board resets; PERST# controls the PCIe PHY; CPU boots from ROM.
2. Link trains; auto-BME writes the config command; `bme_done` asserts.
3. CPU waits for `link_ready`.
4. `dump` latches a 64-bit physical address and a DWORD count.
5. TLPDMA sends sequential 1-DWORD MRd32 TLPs below 4 GiB (MRd64 above).
6. Host completions return as CplDs; TLPDMA matches, validates, and stores
   data.
7. CPU reads the completed buffer and prints it over serial.

## Error Handling

- Console reports not-ready/link-down.
- Console reports DMA error codes: not-ready, completion error, malformed
  completion, timeout.
- A failed page leaves a partial buffer; the console does not print it as
  successful.
- A serial input line that is too long is discarded with an error prompt.
- Invalid addresses/arguments are rejected before starting the DMA.

## Verification

1. Static:
   - `py_compile` the new/changed Python modules.
   - Elaborate `soc_squirrel.py`.
   - Elaborate the existing `pcileech_squirrel.py` after the packer move.
2. Build the SoC bitstream; inspect utilization:
   - BRAM target ≤ ~25/50;
   - LUT/FF within the XC7A35T;
   - record the placed report.
3. Save the SoC bitstream under `bitstreams/` and flash it.
4. Rescan PCIe; confirm the `10ee:0666` device enumerates.
5. Connect to the FT4232H channel-B serial port at 115200 8N1; verify banner
   and prompt.
6. Run `dump` with defaults; capture the 4 KiB hex output, parse it, and
   compare SHA-256 with the VESA reference:
   `5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db`.
7. Run a small arbitrary-address dump (for example `dump 0x1000000 0x40`);
   verify no DMA error and that the host remains alive and the PCIe device
   still enumerates.
8. If resources do not fit, apply the fallback order above and rebuild.
9. Reflash `bitstreams/pcie_squirrel-cleanup-rx128.bin` and rescan to restore
   the PCILeech build after the exercise.

The build may retain the known PCIe/sys timing exception. Functional
acceptance is boot + VESA match + host alive, not timing closure.

## Files

- Create: `soc_squirrel.py`
- Create: `gateware/tlp_dma.py`
- Create: `gateware/tlp32.py` (moved packer)
- Modify: `pcileech_squirrel.py` (import packer from `gateware.tlp32`)
- Create: `software/soc_console/main.c`
- Create: `software/soc_console/Makefile`
- Create: `software/soc_console/linker.ld`
- Modify: `RESOURCES.md` (add the SoC placed-resource row/table and the VESA
  verification result)
- Do not change the existing PCILeech bitstream or `platforms/pcie_squirrel.py`

## Acceptance Criteria

- `soc_squirrel.py` builds a bitstream that fits the XC7A35T.
- The serial console boots and responds at 115200 8N1.
- Default `dump` returns 4 KiB of VESA data matching the known SHA-256.
- An explicit 64-bit address can be dumped without a DMA error.
- The host remains alive and the PCIe device remains enumerated.
- The PCILeech RX128 bitstream can be reflashed and still works.
- The existing PCILeech top level elaborates after the packer move.

## Implementation Result (2026-10-28)

- Built bitstream: `bitstreams/soc_squirrel-console.bin`, 822,836 bytes,
  SHA-256 `25a439bed48e14ad0720a986678338cb51a147225da1f00172e36d29f57031df`.
- BIOS: 2,728 bytes, SHA-256
  `ee620316f9b3ae5fe03e41780b975acd0b42dd83131941b901bb37d78db66cf3`,
  byte-identical to the embedded ROM init.
- Placed resources: 2,050 LUT, 2,470 registers, 16 BRAM, 4 IOB, 8 BUFG.
- Timing: WNS +0.686 ns, WHS +0.032 ns, WPWS -0.007 ns on one PCIe hard-IP
  PIPECLK→USERCLK endpoint; 5 DRC warnings, 0 errors.
- Serial port: `ttyUSB1` (FT4232H USB interface 1.2), 115200 8N1.
- `status` after enumeration reports `link=0x163`, `bme=1`, `id=0x1600`,
  `tlp=0x01`.
- Default `dump` returned 4,096 bytes matching VESA reference SHA-256
  `5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db`.
- `dump 0x1000000 0x40` returned 64 zero bytes with no DMA error; the host
  remained alive and the PCIe device still enumerated.
- The PCILeech RX128 fallback was reflashed after testing.

### Accepted limitations

- The ISSUE state has no independent timeout; if the PCIe TX stream stalls while
  link-ready stays high, the DMA stays busy and the console must be reset.
  The tested main path does not hit this.
- The 8-bit request tag wraps after 256 requests. A completion more than 10 ms
  late after a timeout could theoretically alias a later request; not observed.
- MRd64 (above 4 GiB) is simulation-verified; the hardware test used the MRd32
  VESA/low-memory path.
- The known PCIe hard-IP WPWS -0.007 ns and five DRC warnings are accepted
  bring-up caveats.
- The PCILeech RX128 fallback was reflashed; its final rescan is user-side.

## Future Work

- Multi-DWORD MRd and hardware queueing for higher throughput.
- Host-reserved buffer support and a safer arbitrary-read policy.
- MSI/interrupt-driven completion.
- Optional FT601/PCILeech protocol integration.
- Timing closure and false-path ordering cleanup.
