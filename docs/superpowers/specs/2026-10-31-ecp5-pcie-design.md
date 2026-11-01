# ECP5 PCIe Backend Design

Date: 2026-10-31
Status: Draft for review
Scope: ECP5 Versa (LFE5UM-45F, non-5G), vendor PCIe endpoint IP, local
LitePCIe-compatible PHY backend.

## Summary

LitePCIe currently supports several Xilinx and Intel FPGA PCIe hard IPs,
but no ECP5 endpoint. The target is the ECP5 Versa family in both
variants:

- non-5G ECP5 Versa: LFE5UM-45F, PCIe Gen1 x1, 16-bit vendor TLP path;
- ECP5-5G Versa: LFE5UM5G-45F, PCIe Gen2 x1, 64-bit vendor TLP path.

Both use an FT2232-family JTAG/UART interface and have no FT601.

This design adds a local ECP5 PHY backend that presents the same
software-facing contract as `S7PCIEPHY`, so the existing SoC variants
(V2 single-word, V4 double-buffer DMA, V5 four-outstanding search) can
run unchanged once the PHY is swapped. The wrapper is parameterized by
the vendor TLP width: the 16->64 gearbox is required for the non-5G
16-bit x1 core and bypassed for the 64-bit 5G x1 core.

The vendor backend uses Lattice Diamond and the Lattice PCI Express
Endpoint IP for the link and transaction layers. The vendor IP is used
only as an evaluation and bring-up stepping stone; the long-term goal
is an open-source ECP5 PCIe implementation that does not depend on
Lattice IP. The wrapper boundary is deliberately PHY-agnostic so the
open-source backend can replace the vendor IP later without changing
the SoC.

## Context

### Why this is not a LitePCIe PHY port in the usual sense

LitePCIe's current PHY modules wrap hard IP. The hard IP performs link
training, the data link layer (retry, ACK/NAK, flow control), config
space, BAR decoding, and error handling. LitePCIe sees a TLP
AXI-Stream plus link-status and config signals:

- 64-bit TLP `sink`/`source` streams with `dat`, `be`, and `last`;
- link status (`status`, `rate`, `width`, `ltssm`);
- bus-master, MSI, requester ID, Device Control, and config-management
  signals.

An ECP5 replacement therefore has two very different options:

1. wrap a complete ECP5 endpoint behind the same contract; or
2. implement the ECP5 PHY, LTSSM, DLL, TLP, config space, and BAR
   decode from scratch.

This design takes option 1 first and keeps option 2 as the long-term
open-source destination.

### ECP5 PCIe options

The ECP5 does not have a PCIe hard block. PCIe is built from the ECP5
DCU/SERDES plus Lattice's soft endpoint IP:

- non-5G ECP5 (LFE5UM-45F): 2.5 GT/s (PCIe Gen1), Lattice `PCIE_core`,
  ordering part `PCI-EXP1-E5` for the x1 endpoint;
- ECP5-5G (LFE5UM5G-45F, a different Versa board): 5 Gbps SERDES and
  Lattice `PCIE2_core`, ordering part `PCI-EXP2-E5G`.

Both variants are in scope. The non-5G board is the first hardware
milestone; the 5G variant uses the matching IP core.

| | LFE5UM-45F (Versa) | LFE5UM5G-45F (Versa 5G) |
|---|---|---|
| Family | ECP5 | ECP5-5G |
| Fabric | 45F class | 45F class |
| SERDES | up to 3.2 Gb/s | up to 5.0 Gb/s |
| PCIe | Gen1 2.5 GT/s | Gen2 5 GT/s |
| Vendor PCIe IP | `PCIE_core`, `PCI-EXP1-E5` | `PCIE2_core`, `PCI-EXP2-E5G` |
| Vendor x1 TLP path | 16-bit | 64-bit |
| Core voltage | 1.1 V | 1.2 V |
| Board ordering part | `LFE5UM-45F-VERSA-EVN` | `LFE5UM5G-45F-VERSA-EVN` |
| nextpnr-ecp5 architecture | `um-45k` | `um5g-45k` |
| LiteX device string | `LFE5UM-45F-8BG381C` | `LFE5UM5G-45F-8BG381C` |

Both device variants are supported by Yosys `synth_ecp5`,
`nextpnr-ecp5` (`um-45k` and `um5g-45k`), the prjtrellis database, and
`ecppack`. The same open-source flow is used with a different device and
architecture; the 5G SERDES/PCIe clocking, constraints, and vendor IP
core differ. LiteX-Boards already provides
`lattice_versa_ecp5` with the PCIe x1 pins and a
`--device LFE5UM|LFE5UM5G` option; use it as the starting point for
`platforms/pcie_versa.py`.

The vendor endpoint IP is a mix of hard and soft blocks, is placed in a
specific DCU location, and is delivered through Diamond/Clarity Designer.
The non-5G x1 native core has a 16-bit TLP data path (about 6,207 LUTs,
4 EBRs, 4,188 registers); 16 bits at 125 MHz is 250 MB/s, matching PCIe
Gen1 x1. The 5G x1 downgraded core uses a 64-bit TLP data path; the
wrapper can connect it directly, without the gearbox.

A production IP license is required. Without a license, Diamond embeds a
hardware timer in the bitstream that typically allows 30 minutes to 4
hours of operation (the PCIe core page says about 4 hours). The IP stops
when the timer expires, and reprogramming the FPGA resets the timer.
Full timing simulation is not available without a license. The
evaluation mode is suitable for bring-up and short test sessions, not
for shipping or long unattended runs.

### Open-source alternatives

Both open-source references are 5G-first; neither targets the non-5G
LFE5UM-45F Versa as its primary board.

- **Yumewatari** (whitequark) is written in Migen, not Amaranth. It
  implements the ECP5 DCU/SERDES and LTSSM up to L0 (PHY-level). It
  targets the ECP5-5G Versa (LFE5UM5G-45F): the testbench imports
  `migen.build.platforms.versaecp55g`, the OpenOCD config is
  `ecp5-versa5g.cfg`, and the ispClock images are `versa5g-pcie-*`.
  It runs the DCU at 2.5 Gbps, but on the 5G board. It has no DLL,
  TLP, config space, BAR decode, or DMA.
- **ECP5-PCIe** (ECP5-PCIe/ECP5-PCIe) is written in Amaranth and goes
  further: PHY, LTSSM, DLL/DLLP, and TLP. It is research-grade and
  incomplete; its README says "PHY mostly done", its TODO list is open.
  Its primary hardware is the ECP5-5G Versa plus a custom PCIe adapter
  for the ECP5 5G EVN (`LFE5UM5G-85F`). Most tests use
  `amaranth_boards.versa_ecp5_5g`; `Tests/pcie_adapter.py` defines an
  adapter platform with `device = "LFE5UM5G-85F"`. Some tests override
  the device to LFE5UM-45F/25F on the Versa 5G platform, so non-5G is
  not entirely absent, but it is not the reference configuration.

Neither project is a complete endpoint today. The non-5G LFE5UM-45F
Versa is not the tested board for either; using it would require a
board port and validation. The 5G Versa is the natural open-source
target and also gives Gen2 x1 with a 64-bit TLP path.

These facts affect the open-source backend only. They do not change the
vendor-IP phase ordering, which remains non-5G first until the order is
decided.

The vendor IP is the shortest path to a working ECP5 link and
transaction layer and is used only as an evaluation stepping stone.
Completing an open-source ECP5 PCIe stack is a separate, larger project;
the wrapper boundary in this design is what makes that replacement
possible without touching the SoC.

## Goals

1. Add an ECP5 Versa platform file with device selection for
   LFE5UM-45F and LFE5UM5G-45F, including the PCIe x1 pins, reference
   clock, PERST#, and FT2232 UART/JTAG.
2. Add an `ECP5PCIEPHY` Migen wrapper that exposes the same interface as
   `S7PCIEPHY` and is backed by the Lattice endpoint IP.
3. Prove the vendor endpoint on hardware before integrating LiteX:
   enumerate, BAR0, config space, and TLP loopback.
4. Reuse the existing SoC variants with only the platform and PHY
   swapped: V2 first, then V4/V5.
5. Keep the wrapper boundary stable so a future open-source ECP5 backend
   can replace the vendor IP without changing the SoC, DMA, or search
   code.
6. Measure the real Gen1/Gen2 x1 throughput on each variant and update
   the variant targets accordingly.
7. Support both device variants with a build-time vendor TLP width:
   16-bit non-5G with the gearbox, 64-bit 5G without it.
8. Deliver the vendor-IP path only as an evaluation/bring-up stepping
   stone. The long-term destination is a fully open-source ECP5 PCIe
   implementation, which is a separate project.

## Non-Goals

- Implementing a full open-source PCIe link and transaction stack in
  this spec. That is the long-term destination and a separate project.
- Shipping or production use of the evaluation IP. The evaluation
  bitstream is for bring-up only.
- Upstreaming the vendor IP backend to LitePCIe. The vendor netlist and
  Diamond flow are local to this project.
- PCILeech-over-FT601 on ECP5. That remains the V1 path and needs the
  planned FT601 shield.
- Changing the TLP DMA engines, search engines, console commands, or
  matching semantics.
- A 5G bring-up before the non-5G path is validated. Both variants are
  in scope; the non-5G board is the first hardware milestone.
- A production-ready ECP5 bitstream with full timing closure and
  long-term error recovery.

## Hardware target

### Boards

- Non-5G ECP5 Versa Development Board, LFE5UM-45F.
- ECP5-5G Versa Development Board, LFE5UM5G-45F (second target).
- Both have a PCIe x1 edge connector, 100 MHz-capable reference
  clocking, an FT2232-family JTAG/UART, and no FT601.

### Link expectations

Non-5G (LFE5UM-45F):

- PCIe Gen1 x1, 2.5 GT/s.
- Vendor x1 core: 16-bit TLP data path at 125 MHz; the wrapper uses the
  16<->64 gearbox.
- Raw 8b/10b line rate: 250 MB/s.
- After TLP framing and completion overhead, expect roughly
  200-225 MB/s of payload, not the Squirrel's 426.7 MiB/s.
- The 149 MiB/s search target is plausible on this link, but it must be
  measured. The V5 four-outstanding result of 330-338 MiB/s is not
  expected on Gen1 x1.

5G (LFE5UM5G-45F):

- PCIe Gen2 x1, 5 GT/s.
- Vendor x1 core: 64-bit TLP data path at 125 MHz; the wrapper bypasses
  the gearbox.
- Raw 8b/10b line rate: 500 MB/s.
- Expect roughly 400-450 MB/s of payload; measure rather than assume.
- V5 four-outstanding may be viable at Gen2; measure.

### Prerequisites

- Lattice Diamond installation. The evaluation mode for the matching IP
  core (`PCI-EXP1-E5` for non-5G, `PCI-EXP2-E5G` for 5G) is sufficient
  for Phase 0 and short Phase 1/2 test sessions; a production IP license
  is required for long unattended runs or shipping.
- Each target board's PCIe reference clock and PERST# wiring confirmed
  from the board documentation and the vendor reference design.
- FT2232 JTAG/UART enumeration and programming flow.
- For the open-source backend: Yosys, nextpnr-ecp5, prjtrellis, and
  ecppack. Both `um-45k` and `um5g-45k` are supported.

## Architecture

```text
LiteX SoC (VexRiscv, TLPDMA / ChunkDMA / ReadBench, UART)
        |
        |  sink/source TLP streams (64-bit dat/be/last),
        |  cfg_mgmt_*, link_status, bus_master_enable, id,
        |  dcommand, msi, pcie_rst_n
        v
ECP5PCIEPHY wrapper (new Migen module, vendor_data_width=16|64)
        |  16-bit path: gearbox; 64-bit path: direct
        v
Lattice PCIe x1 Endpoint IP:
  non-5G: PCI-EXP1-E5, 16-bit, Gen1
  5G:     PCI-EXP2-E5G, 64-bit, Gen2
        |
        v
ECP5 DCU/SERDES -> Versa PCIe x1 edge connector
```

The wrapper is the only new integration layer. Everything above it uses
the existing LitePCIe stream and CSR contract. Everything below it is
vendor-specific. The SoC-facing side is always 64-bit; the vendor-facing
width is a build-time parameter selected by the board variant.

## PHY contract

`ECP5PCIEPHY` must provide the subset of `S7PCIEPHY` used by the SoC
variants:

| Signal / attribute | Direction | Purpose |
|---|---|---|
| `sink` | SoC -> PHY | TLP TX stream: `dat`, `be`, `last` |
| `source` | PHY -> SoC | TLP RX stream: `dat`, `be`, `last` |
| `_link_status` | status | `status`, `rate`, `width`, `ltssm` |
| `_bus_master_enable` | status | Bus Master Enable |
| `_msi_enable`, `_msix_enable` | status | MSI/MSI-X enable |
| `_max_request_size`, `_max_payload_size` | status | negotiated sizes |
| `id` | status | requester ID (bus/device/function) |
| `dcommand` | status | Device Control |
| `cfg_mgmt_dwaddr`, `cfg_mgmt_di`, `cfg_mgmt_byte_en`, `cfg_mgmt_wr_en`, `cfg_mgmt_rd_wr_done` | SoC -> PHY / status | config writes, including BME |
| `pcie_rst_n` | SoC -> PHY | PCIe reset / PERST# |
| `msi` | SoC -> PHY | MSI endpoint (optional for V2-V5) |
| `bar0_size`, `bar0_mask`, `config` | parameters/status | BAR0 and ID compatibility |

The SoC variants use `pcie_phy.source`, `pcie_phy.sink`, `_link_status`,
`_bus_master_enable`, `id`, and the BME `cfg_mgmt_*` sequence. The
wrapper does not need to expose every `S7PCIEPHY` signal in the first
iteration, but the table above is the minimal compatible set.

The SoC-facing streams are always 64-bit. The vendor-facing width is a
constructor parameter: `vendor_data_width=16` for LFE5UM-45F and
`vendor_data_width=64` for LFE5UM5G-45F. The gearbox is instantiated
only for the 16-bit case.

## Vendor IP integration

### IP core

- Non-5G: Lattice PCI Express x1/x2/x4 Endpoint IP, x1 native 2.5 GT/s
  variant (`PCI-EXP1-E5`), 16-bit TLP data path at 125 MHz.
- 5G: Lattice PCI Express x1/x2/x4 Endpoint IP, x1 downgraded 5G variant
  (`PCI-EXP2-E5G`), 64-bit TLP data path, Gen2 5 GT/s.
- Both generated through Diamond/Clarity Designer with a matching vendor
  ID, device ID, BAR0 size, and class code matching the Squirrel
  (`10ee:0666`, BAR0 `0x40000`, class `02:00`) where the IP allows it.
- Both include the link and transaction layers, config space, and BAR
  decode. The user-facing TLP interface is the boundary the wrapper
  consumes.
- The non-5G x1 core is 16-bit; the 5G x1 core is 64-bit. The wrapper's
  gearbox is used only for the non-5G core.

### Evaluation-license constraints

- The evaluation IP is a netlist embedded in the user design through
  Diamond; it is not limited to the vendor demo.
- The non-5G (`PCI-EXP1-E5`) and 5G (`PCI-EXP2-E5G`) cores are separate
  IP cores with separate evaluation and license terms; evaluate each
  variant independently.
- Unlicensed bitstreams contain a hardware timer, typically 30 minutes
  to 4 hours (the PCIe core page says about 4 hours).
- When the timer expires, the IP stops. Reprogramming the FPGA resets
  the timer, so repeated short test sessions are possible.
- Full timing simulation is not available without a license.
- Evaluation use is for bring-up and validation only. Do not ship it,
  and do not rely on it for long unattended scans or soak testing.
- Practical plan: use the evaluation IP for Phase 0 and short Phase 1/2
  tests, reflashing between sessions. Long 1 GiB/6 GiB or V5 tests
  either fit inside the timer, use a temporary license, or wait for the
  open-source backend.

### Phase 0 questions to answer from the generated wrapper

The non-5G x1 core's 16-bit width and the 5G x1 core's 64-bit width are
known from the IP documentation. The remaining items are resolved in
Phase 0 for each variant before the wrapper is written:

1. What is the exact TLP handshake and timing (`trn_*`, AXI-Stream, or
   another local bus), and how are the end-of-TLP, byte count, and valid
   bytes encoded? For the non-5G core this is the 16-bit path; for the
   5G core it is the 64-bit path.
2. Does the TLP TX interface expect the requester ID in the user header,
   or does the IP insert it? This determines whether the wrapper's `id`
   is informational only or must be used by the SoC header builder.
3. What config/status signals expose link up, rate, width, LTSSM, BME,
   Device Control, and BAR0 size?
4. Does the IP expose a config-management write port equivalent to
   `cfg_mgmt_*`, or must the wrapper emulate the BME sequence through a
   vendor config interface?
5. What are the exact `.ngo`/Verilog files, placement constraints, and
   license requirements for a Diamond build?
6. Does the x1 core support the two-outstanding / four-outstanding
   completion behavior needed by V5, or only the single-request V2/V4
   behavior?
7. For the 5G x1 downgraded core, confirm that the user TLP interface is
   a direct 64-bit stream (no gearbox) and whether its config/status and
   handshake signals match the non-5G core.

## Wrapper design

### Stream adaptation

- Present a 64-bit LitePCIe `phy_layout` (`dat` 64, `be` 8) on both
  `sink` and `source`, regardless of the device variant.
- `vendor_data_width=16` (LFE5UM-45F): instantiate the 16<->64 gearbox:
  - TX: pack four 16-bit vendor words into one 64-bit LitePCIe word,
    with a partial final word and correct `be`/`last`.
  - RX: unpack each 64-bit LitePCIe word into four 16-bit vendor words,
    preserving the vendor end-of-TLP and valid-byte markers.
- `vendor_data_width=64` (LFE5UM5G-45F): connect the vendor 64-bit TLP
  stream directly, mapping `last`/`be` and preserving TLP order. No
  gearbox is instantiated.
- Map vendor byte enables to `be`; map the vendor end-of-TLP marker to
  `last`.
- Preserve TLP order. Do not merge or reorder TLPs; the gearbox is a
  pure width conversion.
- LitePCIe's `PHYTXDatapath`/`PHYRXDatapath` support 64/128-bit paths,
  not 16-bit. The 16<->64 gearbox is wrapper-specific; the 64-bit path
  can use the existing helpers.

### Status and configuration mapping

- `_link_status.status`: vendor link-up.
- `_link_status.rate`: Gen1 (0) on LFE5UM-45F; Gen2 (1) on LFE5UM5G-45F
  when the link trains at 5 GT/s.
- `_link_status.width`: x1 (0).
- `_link_status.ltssm`: vendor LTSSM state, truncated to six bits.
- `_bus_master_enable`: vendor Device Control bit 2.
- `id`: vendor bus/device/function, or the IP-assigned requester ID.
- `dcommand`: vendor Device Control.
- `_max_request_size`, `_max_payload_size`: from vendor negotiated
  values, or constants matching the IP configuration.
- `cfg_mgmt_*`: map to the vendor config port. If the vendor IP has no
  equivalent, implement the BME write (Device Control `0x6`) in the
  wrapper and expose `cfg_mgmt_rd_wr_done`.

### Clocking and reset

- The vendor user interface is 125 MHz: 16-bit on the non-5G core and
  64-bit on the 5G core.
- Use the vendor 125 MHz user clock for the vendor side, with a
  clock-domain crossing to the 100 MHz SoC side where needed.
- Drive `pcie_rst_n` from PERST# and the SoC reset policy used by the
  Squirrel variants.
- Hold the vendor IP in reset until the reference clock and PERST# are
  valid.
- Provide a `link_ready` equivalent: link up, bus-master enabled, and a
  valid requester ID.

### Optional MSI

The V2-V5 search variants do not use MSI. The wrapper can leave MSI
unconnected in the first iteration and add it only if a later host path
needs it.

## Platform and build flow

### Platform file

Add `platforms/pcie_versa.py` with
`device="LFE5UM"|"LFE5UM5G"`:

- device string: `LFE5UM-45F-8BG381C` or `LFE5UM5G-45F-8BG381C`;
- start from `litex_boards.platforms.lattice_versa_ecp5`, which already
  defines the PCIe x1 pins, serial port, and both device options;
- PCIe x1 TX/RX pins, reference clock, PERST#;
- FT2232 JTAG/UART pins and board clocking;
- LEDs/buttons as needed for bring-up;
- constraints for the selected vendor IP's DCU placement and the PCIe
  reference clock.

### Toolchain

The vendor IP is a placed hard/soft-block netlist and is expected to
require Diamond. Two integration options are acceptable:

1. **Diamond builds the whole design.** LiteX generates Verilog; the
   ECP5 platform uses the Diamond toolchain; the vendor IP wrapper and
   `.ngo` are added to the Diamond project.
2. **Hybrid.** Diamond generates the vendor IP and its constraints; a
   Diamond project wraps the LiteX-generated Verilog; LiteX handles the
   SoC/software side.

Phase 0 determines which is practical. The design does not require a
fully open-source ECP5 build.

Both device variants are supported by the open-source flow:
Yosys `synth_ecp5`, `nextpnr-ecp5` (`um-45k`/`um5g-45k`), prjtrellis,
and ecppack. LiteX's `LatticeTrellisToolchain` already maps
`LFE5UM-45F` to `um-45k` and `LFE5UM5G-45F` to `um5g-45k`. The local
environment currently needs a working `nextpnr-ecp5` and `ecppack`
(the installed `ecppack` is missing `libboost_filesystem.so.1.71.0`);
this is an installation issue, not a device-support gap.

### Generated IP files

- Keep the non-5G and 5G vendor files in separate local directories
  that are not upstreamed and not committed if the license prohibits
  redistribution.
- Record each IP version, configuration, and license state in the
  platform README or a local note.
- Add `.gitignore` entries for vendor netlists if needed.

## SoC reuse

The existing SoC variants should need only a PHY/platform swap:

- `soc_squirrel.py` (V2): replace `S7PCIEPHY` with `ECP5PCIEPHY`; keep
  `TLPDMA`, `soc_console`, and the memory map.
- `soc_search.py` (V4/V5): replace `S7PCIEPHY` with `ECP5PCIEPHY`; keep
  `ChunkDMA`, `ReadBench`, the CPU windows, and the search console.
- `soc_pcileech.py` (V3) can follow if the PCILeech FIFO path is useful
  on ECP5; it is not a first milestone.
- `pcileech_squirrel.py` (V1) is out of scope until the FT601 shield
  exists.

To avoid duplicating top-level code, introduce a build-time selector
(e.g. `--phy s7|ecp5` plus `--device LFE5UM|LFE5UM5G`) or a small
`soc_ecp5.py` that imports the same SoC cores with the ECP5 PHY. The
same SoC cores run on both ECP5 variants; the device and vendor TLP
width are build-time selections. The implementation plan will choose
the least invasive option after Phase 0.

## Phased implementation

### Phase 0: Vendor baseline

- Install Diamond and confirm the evaluation/license state for the
  matching IP core (`PCI-EXP1-E5` first, then `PCI-EXP2-E5G`).
- Non-5G first: build the Lattice x1 PCIe Basic Demo for the
  LFE5UM-45F Versa.
- 5G second, if the board is available: build the equivalent demo for
  the LFE5UM5G-45F with `PCI-EXP2-E5G`.
- Flash and verify each available variant:
  - PCIe enumeration and link rate;
  - BAR0 access and config space;
  - basic TLP loopback / vendor demo behavior.
- Record the evaluation timer behavior and license state per core.
- Inspect the generated wrapper and answer the seven Phase 0 questions
  for each variant.
- Deliverable: known-good vendor bitstreams and a written record of the
  16-bit and 64-bit TLP interfaces, config signals, evaluation timer,
  and build flow.

### Phase 1: Wrapper and V2

- Add `platforms/pcie_versa.py` with device selection.
- Implement `ECP5PCIEPHY(vendor_data_width=16|64)`.
- Build a minimal SoC with `ECP5PCIEPHY` and `TLPDMA`.
- Validate over the FT2232 UART on the non-5G board, then on the 5G
  board if available:
  - enumeration, BME, requester ID;
  - one MRd32 and one MRd64;
  - VESA dump matches;
  - no completion errors.
- Keep each test session within the evaluation timer and reflash
  between sessions.
- Acceptance: V2 behaves like the Squirrel V2 on low memory on each
  available variant, with the link reporting Gen1 x1 or Gen2 x1 as
  appropriate.

### Phase 2: V4 and V5

- Switch `soc_search.py` to `ECP5PCIEPHY`.
- Validate the 2-buffer V4 path (VESA, high address, 1 GiB) on both
  available variants.
- Run V5 four-outstanding only if the vendor IP supports the required
  outstanding/tag behavior. Otherwise keep V4. The 5G Gen2 link is the
  more likely V5 target; the non-5G Gen1 link may remain V4-only.
- Measure 1 GiB and 6 GiB scan rates at sparse strides.
- Long scans may exceed the evaluation timer. Fit them inside a session,
  use a temporary license, or defer them until the open-source backend.
- Acceptance: no completion errors, correct buffer contents, and a
  recorded throughput appropriate for the link (Gen1 x1 or Gen2 x1).

### Phase 3: Long-term open-source backend

- This is the destination, not an optional extra.
- Keep the wrapper interface unchanged for both device variants.
- Replace the vendor IP with an open-source ECP5 PCIe stack
  (ECP5-PCIe/Yumewatari plus the missing link and transaction layers).
- The open-source references are 5G-first. Start the open-source backend
  on the 5G Versa, or explicitly port and validate it for the non-5G
  LFE5UM-45F. Do not assume an untested non-5G path.
- This is a separate project and separate spec; it is not required for
  the vendor-IP stepping stone.

## Data flow and timing

- SoC TLP builders (TLPDMA, ChunkDMA, ReadBench) produce 64-bit TLPs
  with `dat`, `be`, and `last` on both variants.
- Non-5G: the 16<->64 gearbox converts between the 64-bit LitePCIe
  stream and the vendor's 16-bit TLP path at 125 MHz.
- 5G: the vendor's 64-bit TLP path connects directly at 125 MHz; no
  gearbox.
- The vendor IP handles link training, retry, flow control, config
  space, and completions.
- The SoC clock remains 100 MHz, so the wrapper may need a clock-domain
  crossing on either variant.
- At Gen1 x1, a 4 KiB MRd takes about 20-25 microseconds at the raw line
  rate plus completion latency. At Gen2 x1 it takes roughly half that.
  The four-outstanding V5 benefit depends on the vendor IP's
  outstanding-request support and the host root complex behavior.

## Error handling and reset

- PERST# resets the vendor IP and the wrapper state.
- Link down is reported through `_link_status.status`; the SoC's
  existing link-ready checks stop the DMA.
- Config/BME failures are reported through the existing SoC error
  paths; the wrapper does not invent new recovery behavior.
- Completion errors, timeouts, and malformed completions remain handled
  by the existing TLP DMA engines.
- The wrapper must not silently drop TLPs; if the vendor interface
  deasserts a handshake unexpectedly, surface it as an error or a
  status bit for bring-up.

## Verification

### Simulation

- Unit-test the wrapper against a mock vendor TLP interface:
  - 16<->64 gearbox, including partial final words (non-5G);
  - direct 64-bit path (5G);
  - `last` and `be` generation;
  - TLP ordering;
  - link-status and config mapping.
- Re-run the existing `test_tlp_dma.py`, `test_chunk_dma.py`, and
  `test_read_bench.py` after the PHY swap where the simulated PHY can
  be substituted.

### Hardware

Phase 0:
- Vendor demo enumerates; link is Gen1 x1 or Gen2 x1 as appropriate;
  BAR0/config works on each available variant.

Phase 1:
- V2 enumerates and reports `link=1`, BME, requester ID on each
  available variant.
- VESA dump matches the known SHA-256.
- MRd64 high-address read works.

Phase 2:
- V4 VESA/high scan matches on each available variant.
- 1 GiB scan completes with no completion errors.
- 6 GiB scan completes if stable and, if needed, within the evaluation
  timer or with a temporary license.
- Buffer contents are validated through the existing CPU window or
  `rpeek` diagnostics.

### Acceptance criteria

- The ECP5 path runs at least V2 correctly on each available variant;
  V4/V5 are conditional on the vendor IP's outstanding support and the
  link rate.
- No silent data corruption: a known VESA dump and a known high-address
  read must match.
- The link rate and measured throughput are recorded for each variant;
  no assumption of Squirrel Gen2 numbers.
- The evaluation timer is documented and the tests fit its window, or a
  license is present. The result is a stepping stone, not a shipping
  configuration.

## Risks and open questions

- Lattice endpoint IP licensing (evaluation timer) and Diamond
  integration for both cores.
- Vendor x1 TLP handshake semantics: 16-bit non-5G, 64-bit 5G; confirm
  end-of-TLP encoding and byte-count handling for each.
- Evaluation timer limits long test sessions and unattended scans; each
  IP core has its own timer/license.
- Requester-ID ownership (user header vs IP insertion).
- Config/BME signal mapping and whether each vendor IP exposes a
  `cfg_mgmt` equivalent.
- DCU placement and whether the vendor IP constraints coexist with a
  LiteX-generated design.
- Versa PCIe reference clock/PERST# wiring and FT2232 enumeration on
  both boards.
- Gen1 x1 throughput may not meet the 149 MiB/s search target without
  tuning; measure before committing to V5. Gen2 x1 has more headroom but
  still needs measurement.
- The 5G board is a different device/IP with different SERDES/PCIe
  clocking and constraints; do not transfer non-5G assumptions to it.
- Both open-source references are 5G-first; the non-5G LFE5UM-45F
  Versa is off their tested path and needs a board port and validation.
- The open-source backend depends on a working Yosys/nextpnr-ecp5/
  ecppack installation; the local environment currently needs one.

## Proposed files

| File | Purpose |
|---|---|
| `platforms/pcie_versa.py` | ECP5 Versa platform with `device=LFE5UM or LFE5UM5G`, PCIe x1 and FT2232 pins |
| `gateware/ecp5_pcie_phy.py` | LitePCIe-compatible ECP5 PHY wrapper with `vendor_data_width=16 or 64` |
| `gateware/ecp5_pcie_ip/non5g/` (local, not committed) | `PCI-EXP1-E5` wrapper/netlist and constraints |
| `gateware/ecp5_pcie_ip/5g/` (local, not committed) | `PCI-EXP2-E5G` wrapper/netlist and constraints |
| `soc_squirrel.py` / `soc_search.py` | PHY/device selection or a small ECP5 top-level |
| `software/soc_console`, `software/soc_search_console` | Reused unchanged |
| `test/test_ecp5_pcie_phy.py` | Wrapper simulation for both 16-bit and 64-bit paths |
| `docs/ecp5-pcie-notes.md` (optional) | Phase 0 findings: IP versions, TLP widths, config mapping, license/timer state |

## References

- `litex/litepcie/litepcie/phy/s7pciephy.py` - the PHY contract to match.
- `litex/litepcie/litepcie/phy/common.py` - TX/RX datapath helpers.
- `litex/litepcie/litepcie/common.py` - `phy_layout`, `msi_layout`,
  `get_bar_mask`.
- `litex/litex-boards/litex_boards/platforms/lattice_versa_ecp5.py` -
  Versa ECP5 platform with PCIe x1 pins and LFE5UM/LFE5UM5G selection.
- `litex/litex/litex/build/lattice/trellis.py` - nextpnr-ecp5 device and
  architecture mapping (`um-45k`, `um5g-45k`).
- `VARIANTS.md` - V1-V5 and the ECP5 stepping-stone plan.
- Lattice FPGA-IPUG-02009 - PCI Express x1/x2/x4 Endpoint IP Core.
- Lattice FPGA-UG-02006 - PCI Express Demos for the ECP5 and ECP5-5G
  Versa Development Boards.
- Lattice FPGA-TN-02045 - ECP5/ECP5-5G PCI Express Soft IP Ease of Use.
- Lattice IP Support - hardware evaluation timer behavior and licensing:
  https://www.latticesemi.com/Support/LatticeIPSupport
- Yumewatari: https://github.com/whitequark/Yumewatari (Migen,
  PHY/LTSSM reference).
- ECP5-PCIe: https://codeberg.org/ECP5-PCIe/ECP5-PCIe (Amaranth,
  incomplete full-stack reference).

## Out of scope

- Open-source ECP5 PCIe stack implementation in this spec. It is the
  long-term destination and a separate project.
- A production-ready 5G bring-up before the non-5G path is validated.
  Both variants are in scope; the non-5G board is the first milestone.
- FT601 shield and PCILeech V1 on ECP5.
- Production timing closure, long-term reliability, and full error
  recovery.
- Changes to the Squirrel bitstreams or the existing V1-V5 behavior.
