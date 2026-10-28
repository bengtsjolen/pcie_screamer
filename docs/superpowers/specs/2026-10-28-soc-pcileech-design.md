# PCIe Squirrel PCILeechFIFO SoC Variant Design

Date: 2026-10-28
Status: Implemented and hardware-verified (2026-10-28)

## Context

The first VexRiscv SoC variant (`soc_squirrel.py`) uses a dedicated TLPDMA
and bypasses the PCILeech USB/mux path. This second variant keeps the VexRiscv
CPU and serial console but uses the existing `PCILeechFIFO`, its TLP path, and
its mux, with the CPU feeding and reading the FIFO directly instead of an
FT601.

The CPU speaks the subset of the PCILeech wire protocol needed for a TLP
memory read: it sends PCILeech TLP data/flag words to `usb_rx` and parses CplD
frames from `usb_tx`. No FT601 and no full PCILeech host CMD/CFG init are
required for the first version.

The existing first SoC and the PCILeech top level remain unchanged. This is a
separate top level, separate BIOS package, and separate bitstream.

## Goals

- Add a standalone `soc_pcileech.py` with the same VexRiscv console and serial
  behavior as the first SoC.
- Reuse `PCILeechFIFO`, its mux, TLP TX/RX, and CplD filter.
- Feed and read `PCILeechFIFO` from the CPU with small FIFOs; no FT601.
- Implement the raw TLP subset of the PCILeech protocol in the BIOS.
- Keep the same `dump [addr] [len]` console command and VESA default.
- Fit the XC7A35T and keep the first SoC/PCILeech builds untouched.

## Non-Goals

- No FT601 controller in this variant.
- No full PCILeech host emulation; no CMD/CFG/loopback/keepalive init.
- No CPU-side DMA in the first version.
- No MSI, host writes, or multi-DWORD TLP reads.
- No changes to the first SoC or the working PCILeech bitstream.

## Architecture

```text
CPU SRAM / BIOS
   │ 32-bit words
   ▼
PCILeechUsbBridge
   │ TX FIFO + FT601 byte swap
   ▼
PCILeechFIFO.usb_rx
   │ PCILeech TLP flag/data frames
   ▼
PCILeechFIFO.tlp_tx → TLP32To64Packer → S7PCIEPHY
   ▲
   └─ PCILeechFIFO.tlp_rx ← S7PCIEPHY
        │ 64→128 packer → CplD filter → mux → serializer
        │ FT601 byte swap
        ▼
PCILeechUsbBridge RX FIFO
   │
   ▼
BIOS mux parser
```

The PCILeechFIFO's `pcie_rst_core` resets to 0, so no CMD write is needed to
release the PCIe core. Board PERST# and the PHY default remain the reset path.
Auto-BME and `link_ready` qualification are reused from the first SoC:

```text
link_ready = link_up & bus_master_enable & (pcie_phy.id != 0)
```

The CPU reads the same `pcie_id` CSR for the requester ID.

## PCILeechFIFO Connections

Use `PCILeechFIFO(mux_out_fifo_depth=2048, tlp_rx_fifo_depth=2048)` (the large
configuration).

- `pcie_phy.source` → `pcileech_fifo.tlp_rx`
- `pcileech_fifo.tlp_tx` → `TLP32To64Packer` → `pcie_phy.sink`
- `pcileech_fifo.usb_rx` ← `pcileech_bridge.usb_tx_words`
- `pcileech_fifo.usb_tx` → `pcileech_bridge.usb_rx_words`
- `pcileech_fifo.pcie_rst_core` → `pcie_phy.pcie_rst_n` active-low path
- Wire `phy_lnk_up`, `phy_ltssm`, `phy_lnk_rate`, `phy_lnk_width`,
  `phy_id`, and `cfg_dcommand` for the FIFO's CFG readback, even though this
  variant does not issue CFG frames.
- Wire `tx_axis_seen`, `rx_axis_seen`, and `tx_err_drop` for the existing
  timeout inputs.
- Do not instantiate FT601Sync, MSI, or a PCILeech CMD/CFG host.
- Keep the PCILeechFIFO's reset values:
  - `rw[200]=0`: core out of reset;
  - `rw[202]=1`: CplD filter enabled;
  - `rw[16]=0`: inactivity keepalive disabled.

## CPU Bridge

New `gateware/pcileech_bridge.py`:

| CSR | Access | Purpose |
|---|---|---|
| `tx_data` | write | word toward `usb_rx` |
| `tx_full` | read | TX FIFO full |
| `rx_data` | read | word from `usb_tx` |
| `rx_empty` | read | RX FIFO empty |
| `status` | read | link-ready and optional FIFO/sync state |

- TX FIFO: 64–256 × 32-bit. The CPU writes a TLP data word followed by its flag
  word. `usb_rx.ready` is always high, so the CPU only needs `tx_full` as a
  safety check.
- RX FIFO: 1024 × 32-bit. The serializer/mux writes 32-bit words; the CPU
  reads them through `rx_data`.
- Apply the FT601 byte swap in both directions:
  - CPU TX word → byte-swapped `usb_rx.data`;
  - `usb_tx.data` → byte-swapped CPU RX word.
- Expose `link_ready` from the SoC top level.
- The bridge is a CPU-visible stream adapter, not a DMA engine.

## PCILeech TX Format

For each TLP DWORD, the CPU writes:

1. the TLP DWORD;
2. a flag word with:
   - high byte `0x77` (PCILeech magic),
   - type bits `00` (TLP),
   - `last=1` on the final DWORD.

The FIFO's `rx64` packs the data word as `pkt_data` and uses the flag word for
magic/type/last. The CPU sends one MRd at a time:

- Below 4 GiB: MRd32
  - DW0 `0x00000001` (Fmt=00, 3-DW header, length=1)
  - DW1 `{requester_id, rotating 8-bit tag, first_be=0xF}`
  - DW2 `addr[31:0]`, 4-byte aligned
  - `last` on DW2
- At or above 4 GiB: MRd64
  - DW0 `0x20000001` (Fmt=01, 4-DW header, length=1)
  - DW1 `{requester_id, rotating 8-bit tag, first_be=0xF}`
  - DW2 `addr[31:0]`, DW3 `addr[63:32]`
  - `last` on DW3

No CMD, CFG, or loopback frames are sent. The PCILeechFIFO's reset values
leave the PCIe core and CplD filter operational.

## PCILeech RX Format

The serializer emits five `0x66665555` sync words at burst start, then
8-word frames:

- word 0: context-nibble header;
- words 1–7: data words 0–6.

Context-nibble mapping:

```text
bits 31:28 ctx[1]   bits 27:24 ctx[0]
bits 23:20 ctx[3]   bits 19:16 ctx[2]
bits 15:12 ctx[5]   bits 11: 8 ctx[4]
bits  7: 4 0xE      bits  3: 0 ctx[6]
```

The BIOS parser:

- skips sync words and idle filler (`ctx == 0xF`); a sync discards any
  partial CplD state;
- collects data words with TLP context (`ctx & 0x3 == 0`);
- uses `ctx & 0x4` as the final-DWORD flag (the Migen mux puts
  `tlp_last` in context bit 2);
- locates a 1-DWORD CplD (four DWORDs: three header DWORDs plus data);
- verifies CplD type/length, byte count 4, completion status, tag, requester
  ID, and lower address;
- extracts the data DWORD;
- prints the data MSB-first so the serial output is host memory byte order;
- ignores CMD/CFG/loopback contexts (none are generated in this variant).

The BIOS does not keep a full 4 KiB page buffer. It parses each CplD from the
RX stream and prints it, keeping only a small frame/line buffer. The CPU SRAM
is therefore not exhausted by a full page.

## Firmware and Console

Use a standalone copy of the first SoC console:

- `software/soc_pcileech_console/main.c`
- `software/soc_pcileech_console/Makefile`
- `software/soc_pcileech_console/linker.ld`

Shared console behavior:

- 115200 8N1 on `/dev/ttyUSB1`;
- UART RX FIFO read-strobe mode (`uart_rx_fifo_rx_we=True`);
- no libc/printf; custom hex/parser;
- `help`;
- `status` — link, BME, ID, bridge FIFO state;
- `dump [addr] [len]` — default `0x3f000`, `0x1000`, 64-bit hex address,
  4-byte aligned, up to 4 KiB, 16 bytes/line, no writes;
- arbitrary-read warning.

PCILeech backend:

- wait for `link_ready`;
- for each DWORD, build one MRd32/MRd64, write data/flag pairs to the bridge;
- read mux words; parse the CplD; print the data;
- timeout and report if no matching CplD arrives;
- no DMA; the UART is the bottleneck.

## Build Integration

New `soc_pcileech.py`:

- copy the VexRiscv/SoCCore/UART/CRG/PCIe PHY/auto-BME structure from
  `soc_squirrel.py`;
- replace TLPDMA with PCILeechFIFO + `PCILeechUsbBridge`;
- expose `pcie_id`;
- use `Builder` with the local no-meson/no-libc BIOS package
  `software/soc_pcileech_console`;
- output `build/soc_pcileech/`, CSR `test/soc_pcileech_csr.csv`, and
  `soc.build_name="soc_pcileech"`.

The first SoC and PCILeech top level are not modified.

## Resource Estimate

| Block | BRAM tiles |
|---|---:|
| PCILeechFIFO large without FT601 | ~33.5 |
| CPU/ROM/SRAM | ~8 |
| CPU bridge FIFOs | ~2 |
| **Total** | **~43.5 / 50** |

Fallback if placement is tight: use `mux_out_fifo_depth=1024` (medium),
which reduces the total to about 36.5 tiles. Keep `tlp_rx_fifo_depth=2048`.

The bridge TX FIFO is 64–256 entries and the RX FIFO 1024 entries; both are
small relative to the PCILeech mux/TLP buffers.

## Verification

1. Simulation of `PCILeechUsbBridge` FIFO ordering/byte swap.
2. Static compile and elaboration of `soc_squirrel.py` (unchanged),
   `pcileech_squirrel.py` (unchanged), and `soc_pcileech.py`.
3. Build the PCILeechFIFO SoC and inspect placed resources; require <=50 BRAM.
4. Flash the bitstream and rescan; confirm `10ee:0666`.
5. Serial `help`/`status`; default `dump` must match the VESA SHA-256
   `5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db`.
6. `dump 0x1000000 0x40` must complete without mux/PCILeech error and the
   host must remain alive.
7. Restore `bitstreams/pcie_squirrel-cleanup-rx128.bin` and verify it.
8. Record the PCILeechFIFO SoC resource/verification result in
   `RESOURCES.md` and a `SOC_PCILEECH.md` variant note.

## Implementation Result

Implemented and hardware-verified on 2026-10-28. The shipped parser uses the
final behavior described above: rotating 8-bit request tag, CplD
type/length/byte-count/lower-address validation, sync/drain handling, and
`ctx & 0x4` as the last-DWORD flag.

| Item | Result |
|---|---|
| Bitstream | `bitstreams/soc_pcileech-console.bin`, 1,280,396 bytes, SHA-256 `c5bd55ec75e618bcf11bfcb8ff9ed4f050fc3f52f576ad1bcc34ec159f236984` |
| BIOS | `build/soc_pcileech/software/bios/bios.bin`, 3,935 bytes, no undefined symbols |
| Placed resources | 6,208 LUT / 4,213 FF / 37 BRAM tiles (34 RAMB36 + 6 RAMB18) / 4 IOB / 8 BUFG / 0 DSP |
| Timing | WNS +0.525 ns, WHS +0.040 ns, WPWS -0.007 ns on the known PCIe hard-IP PIPECLK→USERCLK skew |
| DRC | 4 warnings, 0 errors |
| Bridge FIFO | 1024 × 34 RX FIFO inferred as LUTRAM (`RAMD64E`/`RAMD32`/`RAMS32` primitives), not BRAM |
| Hardware `status` | `bridge link=1 tx_full=0 rx_empty=1`; `pcie link=0x163 bme=1 id=0x1600` |
| Default VESA dump | SHA-256 `5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db` |
| Arbitrary read | `dump 0x1000000 0x40` returned 64 bytes with no mux/TLP error; host remained alive |
| Fallback | PCILeech RX128 reflashed and a live VESA dump matched the same hash |

The bridge simulation, bridge/PCILeechFIFO integration simulation, and
first-SoC TLPDMA regression pass. The PCILeechFIFO burst gate adds
warmup/settle latency, and the CPU-fed path is intentionally not a throughput
path.

## Error Handling and Timeouts

- Link not ready: console reports status and waits.
- TX FIFO full: wait.
- RX frame sync loss: resynchronize on `0x66665555`; drain the bridge RX
  FIFO at dump start; timeout if no valid frame/CplD.
- CplD status/type/tag/requester/lower-address/byte-count mismatch: report
  and stop. A no-data Cpl with status 0 is malformed.
- No completion within a console-side timeout: report and prompt.
- No host writes; arbitrary reads use the same warning/default as the first
  SoC.

## Accepted Risks / Follow-ups

- The raw TLP subset assumes the PCILeechFIFO needs no CMD/CFG host init.
  If hardware proves otherwise, add a minimal CMD reset/status sequence.
- The mux context/frame parser is the main new firmware risk; hardware VESA
  verification is the acceptance gate.
- The PCILeechFIFO burst gate may add warmup/settle latency; serial output
  dominates.
- CPU polling is not a throughput path; a DMA can be added later.
- PCILeechFIFO large is resource-tight; medium mux depth is the fallback.
- Known hard-IP timing/DRC caveats are reported, not chased.

## Files

- Create: `soc_pcileech.py`
- Create: `gateware/pcileech_bridge.py`
- Create: `software/soc_pcileech_console/main.c`
- Create: `software/soc_pcileech_console/Makefile`
- Create: `software/soc_pcileech_console/linker.ld`
- Create: `test/test_pcileech_bridge.py`
- Create: `test/test_pcileech_bridge_integration.py`
- Create: `SOC_PCILEECH.md`
- Modify: `RESOURCES.md`
- Do not modify the first SoC/PCILeech source or bitstreams.

## Acceptance Criteria

- PCILeechFIFO SoC builds and fits the XC7A35T.
- Serial console boots and `help`/`status` work.
- Default VESA dump matches the known hash.
- Arbitrary low-memory read works without a mux/TLP error.
- Host remains alive and PCIe stays enumerated.
- PCILeech RX128 fallback is restorable and verified.
- First SoC and PCILeech top level remain untouched.

## Future Work

- CPU-side RX/TX DMA.
- Minimal PCILeech CMD/CFG/loopback host subset.
- Multi-DWORD MRds and tag pipelining.
- Performance measurement of the CPU-fed PCILeechFIFO path.
- Resource reduction using the medium mux depth.
