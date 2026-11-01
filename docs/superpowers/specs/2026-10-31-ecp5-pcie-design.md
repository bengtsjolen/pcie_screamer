# ECP5 PCIe Backend Design

Date: 2026-10-31
Status: Draft for review
Scope: ECP5 Versa (LFE5UM-45F, non-5G), vendor PCIe endpoint IP, local
LitePCIe-compatible PHY backend.

## Summary

LitePCIe currently supports several Xilinx and Intel FPGA PCIe hard IPs,
but no ECP5 endpoint. The ECP5 target is the non-5G ECP5 Versa
Development Board with an LFE5UM-45F, a PCIe x1 edge connector, and an
FT2232-family JTAG/UART interface. It has no FT601.

This design adds a local ECP5 PHY backend that presents the same
software-facing contract as `S7PCIEPHY`, so the existing SoC variants
(V2 single-word, V4 double-buffer DMA, V5 four-outstanding search) can
run unchanged once the PHY is swapped. The first backend uses Lattice
Diamond and the Lattice PCI Express Endpoint IP for the link and
transaction layers. The vendor IP is used only as an evaluation and
bring-up stepping stone; the long-term goal is an open-source ECP5 PCIe
implementation that does not depend on Lattice IP. The wrapper boundary
is deliberately PHY-agnostic so the open-source backend can replace the
vendor IP later without changing the SoC.

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

This design takes option 1 first and keeps option 2 as a fallback.

### ECP5 PCIe options

The ECP5 does not have a PCIe hard block. PCIe is built from the ECP5
DCU/SERDES plus Lattice's soft endpoint IP:

- non-5G ECP5 (LFE5UM-45F): 2.5 GT/s (PCIe Gen1), Lattice `PCIE_core`,
  ordering part `PCI-EXP1-E5` for the x1 endpoint;
- ECP5-5G (LFE5UM5G-45F, a different Versa board): 5 Gbps SERDES and
  Lattice `PCIE2_core`.

The vendor endpoint IP is a mix of hard and soft blocks, is placed in a
specific DCU location, and is delivered through Diamond/Clarity Designer.
The x1 native ECP5 core has a 16-bit TLP data path (about 6,207 LUTs,
4 EBRs, 4,188 registers); 16 bits at 125 MHz is 250 MB/s, matching PCIe
Gen1 x1.

A production IP license is required. Without a license, Diamond embeds a
hardware timer in the bitstream that typically allows 30 minutes to 4
hours of operation (the PCIe core page says about 4 hours). The IP stops
when the timer expires, and reprogramming the FPGA resets the timer.
Full timing simulation is not available without a license. The
evaluation mode is suitable for bring-up and short test sessions, not
for shipping or long unattended runs.

### Open-source alternatives

- **Yumewatari** (whitequark) is written in Migen, not Amaranth. It
  implements the ECP5 DCU/SERDES and LTSSM up to L0 (PHY-level) and
  targets the ECP5-5G Versa at 2.5 Gbps. It has no DLL, TLP, config
  space, BAR decode, or DMA.
- **ECP5-PCIe** (ECP5-PCIe/ECP5-PCIe) is written in Amaranth and goes
  further: PHY, LTSSM, DLL/DLLP, and TLP. It is research-grade and
  incomplete; its README says "PHY mostly done", its TODO list is open,
  and it targets a custom PCIe adapter for the ECP5 EVN. It is inspired
  by LitePCIe's layering, but it is not a drop-in LitePCIe PHY.

Neither project is a complete endpoint today. The vendor IP is the
shortest path to a working ECP5 link and transaction layer and is used
only as an evaluation stepping stone. Completing an open-source ECP5
PCIe stack is a separate, larger project; the wrapper boundary in this
design is what makes that replacement possible without touching the SoC.

## Goals

1. Add an ECP5 platform file for the Versa (LFE5UM-45F) with the PCIe x1
   pins, reference clock, PERST#, and FT2232 UART/JTAG.
2. Add an `ECP5PCIEPHY` Migen wrapper that exposes the same interface as
   `S7PCIEPHY` and is backed by the Lattice endpoint IP.
3. Prove the vendor endpoint on hardware before integrating LiteX:
   enumerate, BAR0, config space, and TLP loopback.
4. Reuse the existing SoC variants with only the platform and PHY
   swapped: V2 first, then V4/V5.
5. Keep the wrapper boundary stable so a future open-source ECP5 backend
   can replace the vendor IP without changing the SoC, DMA, or search
   code.
6. Measure the real Gen1 x1 throughput and update the variant targets
   accordingly.
7. Deliver the vendor-IP path only as an evaluation/bring-up stepping
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
- Supporting the ECP5-5G board in the first iteration. The 5G board is
  a different part and a different vendor IP core.
- A production-ready ECP5 bitstream with full timing closure and
  long-term error recovery.

## Hardware target

### Board

- Lattice ECP5 Versa Development Board, LFE5UM-45F (non-5G).
- PCIe x1 edge connector.
- 100 MHz system/reference clocking as required by the board and the
  vendor PCIe IP.
- FT2232-family device for JTAG and UART.
- No FT601.

### Link expectations

- PCIe Gen1 x1, 2.5 GT/s.
- Vendor x1 core: 16-bit TLP data path at 125 MHz; the wrapper converts
  16-bit to/from 64-bit LitePCIe words.
- Raw 8b/10b line rate: 250 MB/s.
- After TLP framing and completion overhead, expect roughly
  200-225 MB/s of payload, not the Squirrel's 426.7 MiB/s.
- The 149 MiB/s search target is plausible on this link, but it must be
  measured. The V5 four-outstanding result of 330-338 MiB/s is not
  expected on Gen1 x1.

### Prerequisites

- Lattice Diamond installation. The PCIe endpoint IP evaluation mode is
  sufficient for Phase 0 and short Phase 1/2 test sessions; a
  production IP license is required for long unattended runs or
  shipping.
- The Versa board's PCIe reference clock and PERST# wiring confirmed
  from the board documentation and the vendor reference design.
- FT2232 JTAG/UART enumeration and programming flow.

## Architecture

```text
LiteX SoC (VexRiscv, TLPDMA / ChunkDMA / ReadBench, UART)
        |
        |  sink/source TLP streams (64-bit dat/be/last),
        |  cfg_mgmt_*, link_status, bus_master_enable, id,
        |  dcommand, msi, pcie_rst_n
        v
ECP5PCIEPHY wrapper (new Migen module)
        |
        |  vendor TLP TX/RX, cfg_*, link/LTSSM status
        v
Lattice PCI Express x1 Endpoint IP (PCI-EXP1-E5, 2.5 GT/s)
        |
        v
ECP5 DCU/SERDES -> Versa PCIe x1 edge connector
```

The wrapper is the only new integration layer. Everything above it uses
the existing LitePCIe stream and CSR contract. Everything below it is
vendor-specific.

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

## Vendor IP integration

### IP core

- Lattice PCI Express x1/x2/x4 Endpoint IP, x1 native 2.5 GT/s variant
  (`PCI-EXP1-E5` for ECP5).
- Generated through Diamond/Clarity Designer with a fixed vendor ID,
  device ID, BAR0 size, and class code matching the Squirrel
  (`10ee:0666`, BAR0 `0x40000`, class `02:00`) where the IP allows it.
- The IP includes the link and transaction layers, config space, and
  BAR decode. The user-facing TLP interface is the boundary the wrapper
  consumes.
- The x1 native ECP5 core uses a 16-bit TLP data path at 125 MHz. This
  is the width the wrapper's gearbox must handle; it is not an open
  question.

### Evaluation-license constraints

- The evaluation IP is a netlist embedded in the user design through
  Diamond; it is not limited to the vendor demo.
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

The x1 core's 16-bit width is known from the IP documentation. The
remaining items are resolved in Phase 0 before the wrapper is written:

1. What is the exact 16-bit TLP handshake and timing (`trn_*`,
   AXI-Stream, or another local bus), and how are the end-of-TLP,
   byte count, and valid bytes encoded?
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

## Wrapper design

### Stream adaptation

- Present a 64-bit LitePCIe `phy_layout` (`dat` 64, `be` 8) on both
  `sink` and `source`.
- The vendor x1 core is 16-bit. Implement a 16<->64 gearbox:
  - TX: pack four 16-bit vendor words into one 64-bit LitePCIe word,
    with a partial final word and correct `be`/`last`.
  - RX: unpack each 64-bit LitePCIe word into four 16-bit vendor words,
    preserving the vendor end-of-TLP and valid-byte markers.
- Map vendor byte enables to `be`; map the vendor end-of-TLP marker to
  `last`.
- Preserve TLP order. Do not merge or reorder TLPs; the gearbox is a
  pure width conversion.
- LitePCIe's `PHYTXDatapath`/`PHYRXDatapath` support 64/128-bit paths,
  not 16-bit. The 16<->64 gearbox must be implemented in the wrapper.

### Status and configuration mapping

- `_link_status.status`: vendor link-up.
- `_link_status.rate`: Gen1 (0) for the non-5G board; expose Gen2 only
  if the board/IP supports it.
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

- The vendor IP's user interface is 16 bits at 125 MHz.
- Use that 125 MHz user clock for the vendor side of the gearbox, with a
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

Add `platforms/pcie_versa.py`:

- device: LFE5UM-45F, package/speed matching the Versa board;
- PCIe x1 TX/RX pins, reference clock, PERST#;
- FT2232 JTAG/UART pins and board clocking;
- LEDs/buttons as needed for bring-up;
- constraints for the vendor IP's DCU placement and the PCIe reference
  clock.

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

### Generated IP files

- Keep the generated vendor files in a local directory that is not
  upstreamed and not committed if the license prohibits redistribution.
- Record the exact IP version, configuration, and license state in the
  platform README or a local note.
- Add a `.gitignore` entry for vendor netlists if needed.

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

To avoid duplicating top-level code, introduce a build-time PHY selector
(e.g. `--phy s7|ecp5`) or a small `soc_ecp5.py` that imports the same
SoC cores with the ECP5 PHY. The implementation plan will choose the
least invasive option after Phase 0.

## Phased implementation

### Phase 0: Vendor baseline

- Install Diamond and the endpoint IP; confirm the license.
- Build the Lattice x1 PCIe Basic Demo for the LFE5UM-45F Versa.
- Flash and verify:
  - PCIe enumeration and link rate;
  - BAR0 access and config space;
  - basic TLP loopback / vendor demo behavior.
- Record the evaluation timer behavior and license state.
- Inspect the generated wrapper and answer the six Phase 0 questions.
- Deliverable: a known-good vendor bitstream and a written record of
  the 16-bit TLP interface, config signals, evaluation timer, and build
  flow.

### Phase 1: Wrapper and V2

- Add `platforms/pcie_versa.py`.
- Implement `ECP5PCIEPHY` for the vendor x1 core.
- Build a minimal SoC with `ECP5PCIEPHY` and `TLPDMA`.
- Validate over the FT2232 UART:
  - enumeration, BME, requester ID;
  - one MRd32 and one MRd64;
  - VESA dump matches;
  - no completion errors.
- Keep each test session within the evaluation timer and reflash
  between sessions.
- Acceptance: V2 behaves like the Squirrel V2 on low memory, with the
  ECP5 link reporting Gen1 x1.

### Phase 2: V4 and V5

- Switch `soc_search.py` to `ECP5PCIEPHY`.
- Validate the 2-buffer V4 path (VESA, high address, 1 GiB).
- Run V5 four-outstanding only if the vendor IP supports the required
  outstanding/tag behavior. Otherwise keep V4 on ECP5.
- Measure 1 GiB and 6 GiB scan rates at sparse strides.
- Long scans may exceed the evaluation timer. Fit them inside a session,
  use a temporary license, or defer them until the open-source backend.
- Acceptance: no completion errors, correct buffer contents, and a
  recorded throughput appropriate for Gen1 x1.

### Phase 3: Long-term open-source backend

- This is the destination, not an optional extra.
- Keep the wrapper interface unchanged.
- Replace the vendor IP with an open-source ECP5 PCIe stack
  (ECP5-PCIe/Yumewatari plus the missing link and transaction layers).
- This is a separate project and separate spec; it is not required for
  the vendor-IP stepping stone.

## Data flow and timing

- SoC TLP builders (TLPDMA, ChunkDMA, ReadBench) produce 64-bit TLPs
  with `dat`, `be`, and `last`.
- The wrapper's 16<->64 gearbox converts between the 64-bit LitePCIe
  stream and the vendor's 16-bit TLP path.
- The vendor IP handles link training, retry, flow control, config
  space, and completions.
- The vendor user interface is 16 bits at 125 MHz; the SoC clock
  remains 100 MHz, so the wrapper may need a clock-domain crossing.
- At Gen1 x1, a 4 KiB MRd takes about 20-25 microseconds at the raw
  line rate, plus completion latency. The four-outstanding V5 benefit
  depends on the vendor IP's outstanding-request support and the host
  root complex behavior.

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
  - 16<->64 gearbox, including partial final words;
  - `last` and `be` generation;
  - TLP ordering;
  - link-status and config mapping.
- Re-run the existing `test_tlp_dma.py`, `test_chunk_dma.py`, and
  `test_read_bench.py` after the PHY swap where the simulated PHY can
  be substituted.

### Hardware

Phase 0:
- vendor demo enumerates, link is Gen1 x1, BAR0/config works.

Phase 1:
- V2 enumerates and reports `link=1`, BME, requester ID.
- VESA dump matches the known SHA-256.
- MRd64 high-address read works.

Phase 2:
- V4 VESA/high scan matches.
- 1 GiB scan completes with no completion errors.
- 6 GiB scan completes if stable and, if needed, within the evaluation
  timer or with a temporary license.
- Buffer contents are validated through the existing CPU window or
  `rpeek` diagnostics.

### Acceptance criteria

- The ECP5 path runs at least V2 correctly; V4/V5 are conditional on
  the vendor IP's outstanding support.
- No silent data corruption: a known VESA dump and a known high-address
  read must match.
- The link rate and measured throughput are recorded; no assumption of
  Squirrel Gen2 numbers.
- The evaluation timer is documented and the tests fit its window, or a
  license is present. The result is a stepping stone, not a shipping
  configuration.

## Risks and open questions

- Lattice endpoint IP licensing (evaluation timer) and Diamond
  integration.
- Vendor x1 16-bit TLP handshake semantics, end-of-TLP encoding, and
  byte-count handling.
- Evaluation timer limits long test sessions and unattended scans.
- Requester-ID ownership (user header vs IP insertion).
- Config/BME signal mapping and whether the vendor IP exposes a
  `cfg_mgmt` equivalent.
- DCU placement and whether the vendor IP constraints coexist with a
  LiteX-generated design.
- Versa PCIe reference clock/PERST# wiring and FT2232 enumeration.
- Gen1 x1 throughput may not meet the 149 MiB/s search target without
  tuning; measure before committing to V5.
- The 5G board is a different target; do not assume its Gen2 behavior
  transfers to the LFE5UM-45F.

## Proposed files

| File | Purpose |
|---|---|
| `platforms/pcie_versa.py` | ECP5 Versa platform, PCIe x1 and FT2232 pins |
| `gateware/ecp5_pcie_phy.py` | LitePCIe-compatible ECP5 PHY wrapper |
| `gateware/ecp5_pcie_ip/` (local, not committed) | Generated vendor wrapper/netlist and constraints |
| `soc_squirrel.py` / `soc_search.py` | PHY selection or a small ECP5 top-level |
| `software/soc_console`, `software/soc_search_console` | Reused unchanged |
| `test/test_ecp5_pcie_phy.py` | Wrapper simulation against a mock vendor interface |
| `docs/ecp5-pcie-notes.md` (optional) | Phase 0 findings: IP version, TLP width, config mapping, license state |

## References

- `litex/litepcie/litepcie/phy/s7pciephy.py` - the PHY contract to match.
- `litex/litepcie/litepcie/phy/common.py` - TX/RX datapath helpers.
- `litex/litepcie/litepcie/common.py` - `phy_layout`, `msi_layout`,
  `get_bar_mask`.
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
- ECP5-5G support.
- FT601 shield and PCILeech V1 on ECP5.
- Production timing closure, long-term reliability, and full error
  recovery.
- Changes to the Squirrel bitstreams or the existing V1-V5 behavior.
