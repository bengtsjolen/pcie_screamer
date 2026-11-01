# ECP5 Open-Source PCIe Backend Design

Date: 2026-11-01
Status: Draft for review
Scope: open-source PCIe x1 endpoint for the ECP5 Versa (LFE5UM-45F, Gen1)
and the ECP5-5G Versa (LFE5UM5G-45F, Gen2), built with Yosys/nextpnr and
exposed to the SoC as a LitePCIe-compatible `ECP5PCIEPHY`.

## Summary

This spec is the open-source backend that
`2026-10-31-ecp5-pcie-design.md` defers to a separate project. It
replaces that spec's vendor-IP Phase 0-2 as the main path; the vendor
IP remains a fallback if the go/no-go gates below fail.

The backend reuses ECP5-PCIe (Amaranth) for the DCU, PHY, LTSSM and data
link layer, and writes everything above the data link layer in Migen:
stream adapter, TLP router, Type 0 config space, credit gate and status
mapping. The Amaranth part is generated to Verilog and instantiated as a
black box; it is ported to Migen layer by layer only after the endpoint
works on hardware.

The first hardware target is the non-5G Versa (LFE5UM-45F) at Gen1 x1,
because it is the only board available. Every interface is parameterized
so the same wrapper builds for the 5G Versa at Gen2 x1, and that build is
kept compiling from the start.

The first milestone with user value is V2 (`soc_squirrel.py`, TLPDMA)
reading host memory over the ECP5 link. That workload is device-initiated
MRd plus received CplD, with Bus Master Enable set by the SoC itself. MSI,
host MMIO to BAR0 and a host driver are not needed for V2-V5.

## Relationship to the vendor-IP spec

- The SoC-facing contract (`PHY contract` in the vendor spec) is shared.
  `ECP5PCIEPHY` implements it once; there is no vendor wrapper and no
  16<->64 gearbox in this design.
- The vendor spec's platform file (`platforms/pcie_versa.py`) is shared.
- Lattice's prebuilt PCIe Basic Demo bitstream for the Versa is used only
  as a hardware oracle (board, slot, refclk, host). Diamond is not
  installed for this, and no Lattice IP is built.
- If milestone M2 (L0) or M4 (enumeration) fails its gate, fall back to
  the vendor spec's Phase 0 and treat this spec as a separate project.

## Context

### What ECP5-PCIe provides

Verified against ECP5-PCIe commit `c511d2e` (2023-05-16), the last
commit. Files are under `Gateware/ecp5_pcie/`.

| Layer | Module | State |
|---|---|---|
| DCU wrapper | `ecp5_serdes.py`, `ecp5_serdes_geared_x4.py` | `CHx_PROTOCOL="PCIe"`, CTC enabled, 100 MHz refclk with `D_REFCK_MODE=0b100` (25x), `D_TX_MAX_RATE` 2.5 or 5.0 from `speed_5GTps` |
| x1 top | `ecp5_phy_x1.py` (`LatticeECP5PCIePhy`) | 4:1 gearing (four 9-bit symbols per `rx`/`tx` cycle), `clkfreq=100e6`, `fabric_clk=True` |
| PHY RX/TX, scrambler, aligner | `phy_rx.py`, `phy_tx.py`, `serdes.py` | Working on 5G Versa |
| LTSSM | `ltssm.py` | Detect through L0 and Recovery; `support_5GTps` sets TS rate bits; no Hot Reset, Loopback, Disabled |
| DLLP, CRC | `dllp.py`, `crc.py` | Working |
| DLL control | `dll.py` | InitFC1/InitFC2, UpdateFC parse into `credits_rx`; advertises `PH=2, PD=0x20, NPH=2, NPD=2, CPLH=0, CPLD=0` (Cpl infinite) |
| TLP TX | `dll_tlp.py` `PCIeDLLTLPTransmitter` | Seq/LCRC, replay buffer of `2**len(replay_num)` = 4 TLPs, `replay_timeout = 462` in `rx` cycles |
| TLP RX | `dll_tlp.py` `PCIeDLLTLPReceiver` | LCRC check, Ack/Nak, `TLPBuffer(max_tlps=4)` of 512-byte slots, no NAK on full buffer |
| Transaction layer | `tlp.py` `TLP` | Config RAM with VID/DID and PCIe cap; CfgRd0/CfgWr0 completions; memory TLPs ignored |
| Simulation | `virtual_serdes.py`, `virtual_phy_Gen1_x1.py`, `virtual_tlp_gen.py` | Upstream/downstream Gen1 pair in one `sync` domain; config-only stimulus |

Facts that shape this design:

- `PCIePhy` defaults to `support_5GTps=True`. Gen1 builds must pass
  `False` explicitly to both the PHY and the SerDes.
- `PCIePhy` instantiates `TLP` and connects it to the DLL TLP stream
  internally. Exposing the raw stream requires changing `phy.py` or
  providing a new top that omits `TLP`.
- The DLL RX buffer has no backpressure to the link partner: when its 4
  slots fill, TLPs are lost (`# TODO: Send NAK if buffer is full`).
- A 512-byte slot holds a 256-byte payload plus header, not a 512-byte
  payload. Device Capabilities must advertise Max Payload Size 256 or
  less.
- The hardware tests use `amaranth_boards.versa_ecp5_5g` with the device
  overridden to `LFE5UM-25F`. LFE5UM-45F on the non-5G Versa has never
  been the build target.
- The bundled ispCLOCK images (`ispCLOCK/versa5g-pcie-*.svf`) are for
  the 5G Versa only.
- `Gateware/setup.py` requires unpinned `amaranth`; the code uses
  `Record`, old `Memory` and `amaranth.hdl.ast`, so it needs Amaranth
  0.4 or older.

### What the SoC needs from the PHY

`soc_squirrel.py`, `soc_search.py` and `soc_pcileech.py` use
`S7PCIEPHY` as follows; `ECP5PCIEPHY` must match each item.

| Member | Use in the SoC |
|---|---|
| `sink`, `source` | `stream.Endpoint(phy_layout(64))` in `sys`, connected straight to TLPDMA/ChunkDMA/ReadBench (no LitePCIe packetizer) |
| `cd_pcie` | `self.sync.pcie` drives the BME pulse timer |
| `_link_status.fields.status` | Link-up gate for the BME pulse and `link_ready` |
| `_bus_master_enable.status` | Part of `link_ready` |
| `id` | Requester ID in MRd headers; `link_ready` requires `id != 0` |
| `cfg_mgmt_dwaddr/di/byte_en/wr_en` | One write of `0x00000006` to dword 1 (Command: MSE \| BME) |
| `config` dict | `Device_ID`, `Class_Code_*`, `Extended_Tag_*` |
| `bar0_size`, `bar0_mask`, `data_width`, `endianness`, `qword_aligned`, `max_*_size`, `msi`, `_msi_enable`, `_msix_enable` | Read by LitePCIe helpers and CSR software |

The stream byte layout is defined by the S7 PHY and pinned by
`test/test_tlp_dma.py`:

- `dat[31:0]` is TLP DW0 and `dat[63:32]` is DW1 of each beat.
- Each dword is an integer whose bits `[31:24]` are the first byte on the
  wire (DW0 `[31:24]` = Fmt/Type).
- Payload dwords use the same packing; this is why the scan comparator
  byte-swaps CplD words.
- `be` has one bit per byte lane: `0x0F` = DW0 only, `0xF0` = DW1 only,
  `0xFF` = both. Only the last beat may be partial.
- `first` is unused by the SoC cores; `last` marks the final beat.

### Workload

| Variant | TX TLPs | RX TLPs | Outstanding reads |
|---|---|---|---|
| V2 TLPDMA | MRd32/MRd64, 1 DW to 4 KiB | CplD, Cpl (UR/CA) | 1 |
| V4 ChunkDMA | MRd | CplD | 2 (double buffer) |
| V5 ReadBench / search | MRd | CplD | 4, extended tags |

All of these are non-posted requests from the endpoint and completions
to it. Under PCIe rules the endpoint advertises infinite completion
credits, which ECP5-PCIe already does, so the endpoint itself must
guarantee it can absorb every completion for every request it has issued.
This, not TX replay depth, is the first-order buffering constraint.

Gen1 x1 limits throughput to 250 MB/s raw and roughly 200 MB/s of
payload; the 4-symbol `rx`/`tx` domains run at 62.5 MHz with a 32-bit
TLP symbol path.

## Goals

1. `ECP5PCIEPHY(platform, pads, data_width=64, bar0_size=..., gen=1|2)`
   implementing the contract above, with no Lattice IP and no Diamond.
2. Non-5G Versa (LFE5UM-45F) Gen1 x1: enumerate on a real host, then run
   V2 with a matching VESA dump and a working MRd64 high-address read.
3. Then V4 and V5 on the same board with no completion errors, and a
   measured Gen1 throughput recorded in `PERFORMANCE.md`.
4. The 5G Versa (LFE5UM5G-45F) Gen2 build compiles and passes simulation
   in CI from M1 onward; hardware validation when the board is available.
5. All layers above the DLL written in Migen from the start and tested
   against LitePCIe's host model.
6. After M6, port the Amaranth layers to Migen with differential
   simulation against the generated Verilog.

## Non-Goals

- MSI/MSI-X, host MMIO to BAR0 and the LitePCIe kernel driver, until V5
  works. The config space reserves their place (see below).
- Gen2 hardware bring-up before the 5G board is available.
- x2/x4 links, power management states beyond L0, ASPM, AER, hot plug.
- Upstreaming to LitePCIe before the Migen port is complete.
- Changes to TLPDMA, ChunkDMA, ReadBench, the consoles or their matching
  semantics.

## Hardware target and bring-up setup

- Non-5G Versa, LFE5UM-45F-8BG381C, nextpnr `um-45k`. litex-boards'
  `lattice_versa_ecp5.py` defaults to `device="LFE5UM5G"`; every build
  entry point must pass the device explicitly and assert it.
- PCIe x1 pins from litex-boards: DCU0 CH0, `perst` on A6,
  `refclk_en` C12, `refclk_rst_n` R1.
- The slot refclk reaches the DCU through the onboard ispCLOCK. Before
  any LTSSM debugging, confirm the routing and the PRSNT strap with the
  Lattice demo bitstream and, if needed, the J4 jumper.
- Power the Versa from its own 12 V supply, not the slot, so the FPGA can
  be reprogrammed over JTAG with the host running. Re-enumerate with the
  root port's `remove`/`rescan` in sysfs.
- Use a dedicated test host or a slot behind a PCIe switch; a half-trained
  endpoint can hang some chipsets.
- Debug path: UARTBone over the FT2232 UART, LiteScope on the symbol and
  TLP streams, CSRs for LTSSM state, DLL state and error counters.

## Architecture

```text
SoC (TLPDMA / ChunkDMA / ReadBench, VexRiscv, UART), unchanged
   │  sys: sink/source phy_layout(64), cfg_mgmt_*, CSRs, id
   v
ECP5PCIEPHY (Migen)                                   gateware/ecp5pcie/
   ├─ TLPStreamAdapter   32-bit symbols (pcie) <-> 64-bit dat/be (sys), async FIFOs
   ├─ TLPRouter          RX: Cfg -> ConfigSpace, Msg -> drop, MRd/MWr -> UR,
   │                         Cpl/CplD -> SoC
   │                     TX: arbitrate SoC, ConfigSpace completions, UR completions
   ├─ ConfigSpace        Type 0 header, PCIe cap, BAR0, cfg_mgmt port, ID capture
   ├─ CreditGate         TX gating on P/NP/Cpl header+data credits
   ├─ CompletionBudget   RX buffer sized for max outstanding read bytes
   └─ ECP5PCIeCore       black box: DCU + PHY + LTSSM + DLL
                         (Amaranth 0.4 -> Verilog, `rx`/`tx` domains)
   v
DCU0 CH0 -> Versa PCIe x1 edge connector
```

The boundary between `ECP5PCIeCore` and the Migen layers is the DLL TLP
stream: bare TLPs, 4 bytes per `rx`/`tx` cycle, with start/end markers and
no STP/sequence/LCRC/END. Everything that the SoC can observe is defined
in Migen, so the later port of the core does not change any SoC-visible
behavior.

### Clock domains

| Domain | Source | Gen1 | Gen2 |
|---|---|---|---|
| `rx` | DCU recovered clock, 4-symbol gearing | 62.5 MHz | 125 MHz |
| `tx` | DCU TX clock, 4-symbol gearing | 62.5 MHz | 125 MHz |
| `pcie` | `ECP5PCIEPHY.cd_pcie`, driven from `tx` | 62.5 MHz | 125 MHz |
| `sys` | SoC CRG | 100 MHz (current SoCs) | same |

ECP5-PCIe runs its DLL TX and RX logic in `rx` and relies on CTC to keep
`rx` and `tx` matched. The Migen layers run in `pcie`. The RX path crosses
`rx` to `pcie` in a small async FIFO inside the core top; `pcie` to `sys`
uses the adapter's async FIFOs. `sys` at 100 MHz with 64 bits carries
800 MB/s, more than Gen2 x1 needs.

## ECP5PCIeCore (Amaranth, black box)

### Changes to ECP5-PCIe

Kept as a fork pinned as a git submodule at `third_party/ECP5-PCIe`,
based on `c511d2e`. Changes are small and each one is a separate commit:

1. A new top `ecp5_pcie_core.py` that instantiates the SerDes, aligner
   and `PCIePhy` without `TLP`, and exports:
   - `tlp_rx`: `symbol[4]`, `valid[4]`, `first`, `last`, plus an
     `overflow` sticky bit from the DLL RX buffer;
   - `tlp_tx`: `symbol[4]`, `valid[4]`, `first`, `last`, `ready`;
   - `credits_rx` (the link partner's limits) and an `fc_update` strobe;
   - `ltssm_state[6]`, `link_up`, `dll_up`, `speed`;
   - `perst_n` input; `rx_clk`, `tx_clk` outputs; DCU pads.
2. `PCIePhy` takes the transaction layer as an optional argument so the
   upstream ECP5-PCIe tests keep working.
3. DLL fixes required before M5 (each with a simulation test):
   - `sending_tlp` in `dll.py` connected to the TLP transmitter, so
     UpdateFC and Ack/Nak scheduling see it;
   - replay buffer depth decoupled from `REPLAY_NUM`: a `replay_tlps`
     parameter, default 8;
   - `replay_timeout` and the Ack/Nak latency limit derived from PCIe 1.1
     Table 3-4 for MPS 256 x1, in symbol times, converted with the gearing
     ratio and `speed`;
   - `accepts_tlps` compared against the 12-bit sequence window per
     spec (`next_transmit_seq - ackd_seq` modulo 4096 must be below 2048);
   - Ack processing checks the acknowledged sequence number before
     popping the replay buffer;
   - RX buffer full: do not accept a TLP that cannot be stored; send a
     Nak so the partner replays it, and set `overflow`.
4. Advertised posted/non-posted credits sized for what the router can
   absorb (they only carry config requests and stray MWr/MRd here):
   `PH=4, PD=8, NPH=4, NPD=4`, Cpl infinite.

The Amaranth layers keep their `rx`/`tx` domains; the generated module
has plain clock and reset ports.

### Generation

- `tools/gen_ecp5_pcie_core.py` runs in a dedicated venv
  (`.venv-amaranth04`, Amaranth 0.4.x pinned in
  `third_party/requirements-amaranth04.txt`) and writes
  `gateware/ecp5pcie/verilog/ecp5_pcie_core_gen{1,2}.v`.
- The generated Verilog is committed with a header recording the
  submodule commit and generator arguments, so the main Migen/LiteX venv
  never needs Amaranth. CI regenerates and fails on a diff.
- `ECP5PCIeCore` (Migen) does `platform.add_source()` and `Instance()`;
  Yosys reads both sources and nextpnr places one design.
- Gen1: `speed_5GTps=False` (SerDes) and `support_5GTps=False` (PHY and
  LTSSM). Gen2: both `True`.

## Migen layers

### TLPStreamAdapter

- RX: packs 4-byte `rx` beats into 64-bit `sys` beats in the S7 layout
  above. DW0 lands in `dat[31:0]`, wire byte 0 in `dat[31:24]`.
- `be` on the last beat is computed from the dword count: `0x0F` when the
  TLP has an odd number of dwords, `0xFF` otherwise. `first` is set on
  beat 0.
- TX: the inverse; `be` is ignored except to find the dword count of the
  last beat.
- Async FIFOs `pcie`<->`sys`, at least 2 maximum-size TLPs deep each.
- The RX side never applies backpressure to the core faster than the
  core can tolerate; `CompletionBudget` provides the storage.
- Golden vectors: a set of real MRd/CplD beats captured from the Squirrel
  with the existing LiteScope setup, replayed through
  `test/test_ecp5pcie_adapter.py` in both directions.

### TLPRouter

RX, by Fmt/Type of DW0:

| TLP | Destination |
|---|---|
| CfgRd0, CfgWr0 | `ConfigSpace` |
| CfgRd1, CfgWr1 | UR completion |
| Cpl, CplD | SoC `source` |
| MRd32/MRd64 | UR completion until BAR0 MMIO exists |
| MWr32/MWr64 | dropped, counted |
| Msg, MsgD | dropped, counted (no reply) |
| anything else | UR completion if non-posted, else dropped; counted |

TX: round-robin between `ConfigSpace` completions, UR completions and the
SoC `sink`, never interleaving inside a TLP. Config and UR completions
are short and take priority so a busy DMA cannot starve enumeration.

### ConfigSpace

A register file, not a RAM. Fields are read-only unless listed as
writable. Values come from `phy.config` with S7-compatible keys so the SoC
code is unchanged.

| Offset | Register | Behavior |
|---|---|---|
| 0x00 | Vendor/Device ID | `config["Vendor_ID"]` (default `10EE`), `config["Device_ID"]` |
| 0x04 | Command / Status | Command bits MSE (1), BME (2), SERR (8), INTx Disable (10) writable; Status RW1C error bits, Capabilities List = 1 |
| 0x08 | Revision / Class | `Class_Code_*` |
| 0x0C | Header type 0, cache line size RW | |
| 0x10 | BAR0 | 32-bit non-prefetchable memory BAR; low `log2(bar0_size)` bits read 0 |
| 0x14-0x24 | BAR1-5 | Read 0 (not implemented) |
| 0x2C | Subsystem IDs | From `config` |
| 0x34 | Capabilities Pointer | 0x40 |
| 0x3C | Interrupt Line RW, Pin 0 | |
| 0x40 | PCI Power Management cap | Minimal, D0 only, next = 0x50 |
| 0x50 | MSI cap | Present with Enable stuck at 0 until MSI is implemented, next = 0x60 |
| 0x60 | PCIe cap v2, endpoint | Device Cap: MPS 256, Extended Tag from `config["Extended_Tag_Field"]`; Device Control writable (MPS, MRRS, Extended Tag Enable, error reporting bits); Link Cap: x1, Gen1 or Gen2 from `gen`; Link Status from the core |
| 0x100+ | Extended config | Reads 0 (no extended capabilities) |

- Bus/device number captured from every CfgWr0 completer ID field into
  `phy.id` (function 0).
- `max_payload_size` and `max_request_size` decoded from Device Control
  into bytes, exposed as signals and CSRs.
- The `cfg_mgmt_*` port performs the same masked write as a CfgWr0 from
  the host, one cycle per write, and pulses `cfg_mgmt_rd_wr_done`. This
  is how the SoC sets MSE and BME.
- CplD for every CfgRd0 and Cpl for every CfgWr0, completer ID from the
  captured ID, byte count 4, lower address from the register address.

### CreditGate

- Tracks consumed P, NP and Cpl header and data credits for TX TLPs and
  compares them against `credits_rx` from the core, using the PCIe 12-bit
  and 8-bit modulo arithmetic. Infinite (0) limits are never gated.
- Stalls the TX arbiter at TLP boundaries when the next TLP would exceed
  a limit. For this workload only NP header credits are realistically
  hit (one per MRd).
- Exposes stall counters as CSRs.

### CompletionBudget

- Parameter `max_outstanding_read_bytes`, set by the SoC: 4 KiB for V2,
  8 KiB for V4, 16 KiB for V5.
- The RX async FIFO plus the SoC-side buffer must hold that many bytes of
  CplD payload plus 16 bytes per completion header at the minimum read
  completion boundary (64 bytes). The constructor asserts this at build
  time.
- No dynamic reservation in the first iteration. If later SoCs issue
  reads without a fixed bound, add a per-tag reservation in the TX path.
- `overflow` from the core and from the adapter FIFOs is a sticky CSR bit
  and a LiteScope trigger.

### Status and reset

- `_link_status.status` = `dll_up`; `rate` = core `speed`; `width` = x1;
  `ltssm` = core state index (ECP5-PCIe `State` enum, documented in a
  table in `gateware/ecp5pcie/README.md`).
- `_bus_master_enable`, `_msi_enable` (always 0), `_msix_enable` (always
  0), `_max_request_size`, `_max_payload_size` from `ConfigSpace`.
- PERST# (`pads.perst`, active low) resets the core, the DCU and all
  Migen layers, and holds the DCU in reset until refclk is present.
- DLL down (link loss, Hot Reset seen as LTSSM Detect) resets the
  Migen layers' TLP state, clears `id`, Command and Device Control, and
  drops `_link_status.status` so the SoC's `link_ready` falls and its DMA
  engines abort and flush.
- The platform's `refclk_en` and `refclk_rst_n` are driven to enable the
  ispCLOCK output.

## Platform and build

- `platforms/pcie_versa.py`: wraps `litex_boards.platforms.lattice_versa_ecp5`
  with a required `device` argument (`LFE5UM` or `LFE5UM5G`) and the PCIe
  constraints. Shared with the vendor spec.
- `soc_ecp5.py`: a top that builds V2 or V4/V5 with
  `ECP5PCIEPHY`, reusing the existing SoC cores, with
  `--device LFE5UM|LFE5UM5G` and `--variant v2|v4|v5`. The existing
  Squirrel tops are not modified.
- Toolchain: Yosys, nextpnr-ecp5 and ecppack via LiteX's
  `LatticeTrellisToolchain`. The installed nextpnr-ecp5 is 0.6; M1
  confirms it handles the DCU and timing, and upgrades if not.
- CI-equivalent `make` targets: regenerate the core, run all simulations,
  build both `um-45k` and `um5g-45k` bitstreams.

## Verification

### Simulation

1. **Migen unit tests** (`run_simulation`):
   - adapter: golden S7 vectors, odd/even dword counts, back-to-back TLPs,
     both directions;
   - router: every TLP type to the right destination, UR completions;
   - config space: read-only masks, RW1C, BAR0 sizing (write all ones,
     read back the mask), Device Control decode, ID capture,
     `cfg_mgmt` write;
   - credit gate: limits, wraparound, infinite credits;
   - status/reset: DLL down clears state and drops link status.
2. **Host-model integration**: `litex/litepcie/test/model/` (`host.py`,
   `chipset.py`, `tlp.py`) drives the Migen stack above a TLP-level stub
   of the core: enumeration sequence (BAR sizing, Command write,
   Device Control), then V2 TLPDMA reads with CplDs split at 64 and 128
   bytes.
3. **Core simulation** with ECP5-PCIe's `virtual_phy_Gen1_x1.py`: an
   upstream/downstream pair through the virtual SerDes, run in Verilator
   on the generated Verilog via LiteX sim, with new stimulus for
   MRd/CplD bursts, injected LCRC errors (Nak and replay), dropped Acks
   (replay timeout) and RX buffer full.
4. Existing `test_tlp_dma.py`, `test_chunk_dma.py` and
   `test_read_bench.py` keep passing unchanged.

### Hardware

| Milestone | Pass criterion |
|---|---|
| M2 | Root port `LnkSta` shows a trained x1 link at 2.5 GT/s; LTSSM CSR in L0; stays up for 10 minutes |
| M3 | DLL CSR `dll_up`; UpdateFC exchange visible in LiteScope; no replay storms |
| M4 | `lspci -vv` shows the configured VID/DID/class, BAR0 of `bar0_size`, PCIe cap with x1 Gen1; `setpci` reads/writes behave; no AER errors on the root port |
| M5 | V2: link, BME and `id` reported over UART; VESA dump matches the known SHA-256; MRd64 high-address read correct |
| M6 | V4 1 GiB scan with no completion errors; V5 four-outstanding reads correct; throughput recorded |

## Milestones

Each milestone has a gate; do not start the next one until it passes.

- **M0 Hardware oracle.** Flash the Lattice PCIe Basic Demo bitstream for
  the LFE5UM-45F Versa (if Lattice ships one; otherwise skip). Gate: the
  host enumerates the demo. A failure here is a board, slot or refclk
  problem, not ours.
- **M1 Environment and builds.** Submodule, Amaranth 0.4 venv, generator,
  `ECP5PCIeCore` black box, `platforms/pcie_versa.py`, a minimal
  `soc_ecp5.py` with UARTBone and status CSRs only. Gate: both `um-45k`
  and `um5g-45k` bitstreams build and meet timing on `rx`, `tx`, `pcie`
  and `sys`.
- **M2 Link to L0.** Gate as in the hardware table. This is the highest
  risk milestone: non-5G DCU, PLL and CTC settings, refclk path and
  LFE5UM-45F are all off ECP5-PCIe's tested path. Go/no-go for the
  open-source path.
- **M3 DL_Active.** DLL fixes from the core section that affect bring-up
  (`sending_tlp`, Ack sequence check). Gate as in the table.
- **M4 Enumeration.** `TLPRouter`, `ConfigSpace`, `TLPStreamAdapter`,
  status mapping; host-model simulation first. Gate as in the table.
- **M5 V2.** `CreditGate`, `CompletionBudget`, remaining DLL fixes,
  `soc_ecp5.py --variant v2`. Gate as in the table.
- **M6 V4/V5 and throughput.** Gate as in the table; update
  `PERFORMANCE.md` and `VARIANTS.md`.
- **M7 Migen port.** See below. Gate: the Migen core passes the same
  simulations and the M5/M6 hardware tests on the same bitstream flow.
- **M8 (when the board is available) 5G Versa Gen2.** Repeat M2-M6 on
  LFE5UM5G-45F with `gen=2`, including the Gen1-to-Gen2 speed change
  through Recovery.

Rough effort for a solo developer, from the vendor spec's assessment:
M0-M1 days, M2-M4 weeks with M2 the main unknown, M5-M6 one to several
months dominated by DLL corner cases.

## Migen port (M7)

- Port order follows the dependency graph and the existing tests: CRC
  and scrambler, PHY RX/TX and aligner, DLLP, DLL control, DLL TLP
  TX/RX with the buffers, LTSSM, and the DCU wrapper last.
- Each ported layer is checked by differential simulation: the same
  stimulus through the generated Amaranth Verilog and the Migen module in
  Verilator, comparing outputs cycle by cycle.
- Mechanical mappings: `m.d.rx +=` to `self.sync.rx +=`, `m.If` to
  `If/Elif/Else`, `m.FSM` to `FSM` with `NextState`/`NextValue`,
  `FFSynchronizer` to `MultiReg`, `Record` to Migen `Record`. Watch for
  memory read-port transparency defaults and `Part`/shift semantics.
- The DCU wrapper is compared against LiteICLink's
  `serdes_ecp5.py`, which is already Migen but uses `10BSER` with CTC
  bypassed; reuse its PLL and SCI helpers where the parameters match.
- When the port is complete the submodule and generated Verilog are
  removed, and `ECP5PCIEPHY` can be proposed upstream as
  `litepcie/phy/ecp5pciephy.py`.

## Risks and open questions

- **Refclk on the non-5G Versa.** Whether the factory ispCLOCK image
  routes the slot refclk to DCU0 with this host and slot, and whether
  PRSNT must be strapped. M0 answers this.
- **LFE5UM DCU settings.** CMU/PLL, CDR and CTC values in
  `ecp5_serdes.py` came from 5G bring-up. Trellis models the DCU the same
  for both families, but the non-5G silicon is only rated to 3.2 Gb/s and
  untested here.
- **LTSSM interoperability.** Missing Hot Reset, Disabled and Loopback,
  and timer values tuned for one host. Expect host-specific fixes.
- **DLL corner cases.** The replay and Ack/Nak paths have open TODOs;
  CplD bursts at V5 rates will exercise them. Budget most of the M5-M6
  time here.
- **No backpressure on the DLL RX path** until the full-buffer Nak fix
  lands; until then `CompletionBudget` and line-rate draining are the
  only protection.
- **Old Amaranth.** Amaranth 0.4 must stay installable for the life of
  the black-box phase; the pinned venv and committed Verilog limit the
  exposure.
- **nextpnr-ecp5 0.6** may be too old for clean timing on the DCU
  domains.
- **Gen2 is unvalidated** until the 5G board is available; keep its
  build and simulation green so it does not rot.
- **License.** ECP5-PCIe has no license declared (`# TODO license=''` in
  `setup.py`). Ask the authors before redistributing the fork or
  upstreaming a port.

## Proposed files

| File | Purpose |
|---|---|
| `third_party/ECP5-PCIe` (submodule) | Fork of ECP5-PCIe at `c511d2e` plus the changes above |
| `third_party/requirements-amaranth04.txt` | Pinned Amaranth 0.4 environment for generation |
| `tools/gen_ecp5_pcie_core.py` | Generates the core Verilog for Gen1 and Gen2 |
| `gateware/ecp5pcie/verilog/` | Committed generated Verilog |
| `gateware/ecp5pcie/core.py` | `ECP5PCIeCore` Migen black-box wrapper |
| `gateware/ecp5pcie/adapter.py` | `TLPStreamAdapter` |
| `gateware/ecp5pcie/router.py` | `TLPRouter` and UR completion generator |
| `gateware/ecp5pcie/config_space.py` | `ConfigSpace` |
| `gateware/ecp5pcie/credits.py` | `CreditGate` and `CompletionBudget` |
| `gateware/ecp5pcie/phy.py` | `ECP5PCIEPHY` |
| `gateware/ecp5pcie/README.md` | LTSSM state table, CSR map, build notes |
| `platforms/pcie_versa.py` | Versa platform with required `device` |
| `soc_ecp5.py` | ECP5 top for V2/V4/V5 |
| `test/test_ecp5pcie_*.py` | Unit and host-model simulations |

## References

- ECP5-PCIe: https://github.com/ECP5-PCIe/ECP5-PCIe (commit `c511d2e`)
  and its GSoC report:
  https://ecp5-pcie.github.io/gsoc/2020/09/23/development-report.html
- `docs/Open-source PCIe for ECP5 (non-5G) with LitePCIe.md`: layer
  comparison and gap list.
- `docs/superpowers/specs/2026-10-31-ecp5-pcie-design.md`: vendor-IP
  fallback, shared PHY contract and platform.
- `litex/litepcie/litepcie/phy/s7pciephy.py`: the contract to match.
- `litex/litepcie/test/model/`: host, chipset and TLP models.
- `litex/liteiclink/liteiclink/serdes/serdes_ecp5.py`: Migen ECP5 DCU
  reference.
- `litex/litex-boards/litex_boards/platforms/lattice_versa_ecp5.py`.
- PCI Express Base Specification 1.1: Table 3-4 (replay and Ack/Nak
  timers), section 2.6 (flow control), section 7 (configuration space).
- Lattice FPGA-UG-02006: PCI Express demos for the ECP5 and ECP5-5G Versa
  boards.
