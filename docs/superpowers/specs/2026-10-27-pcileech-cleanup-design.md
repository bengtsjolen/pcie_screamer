# PCIe Squirrel PCILeech Cleanup Design

Date: 2026-10-27
Status: Approved for implementation planning

## Context

The current `performance-tuning` branch has a working `large` RX128 build that
matches ufrisk throughput: 149.4 MiB/s on a 1 GiB read, zero failed pages, and a
byte-identical VESA ROM dump. The active top level has accumulated dead
converters, duplicate files, unreachable serializer alternatives, and
always-on diagnostic logic from the performance investigation.

Before adding a second top level for a VexRiscv/BIOS/serial SoC, remove the
dead code and diagnostic machinery from the active PCILeech path. The cleanup
must preserve the working data path, FT601 behavior, PCILeech wire protocol, and
measured performance.

## Goals

- Remove unused and unreachable code from the active PCILeech build.
- Remove diagnostic-only counters, latches, CDC paths, and custom readbacks.
- Keep functional PCIe timeout/recovery, FT601 filler/drain behavior, FIFO
  presets, MSI, link/BME setup, loopback, and analyzer support unchanged.
- Produce a cleaned bitstream that passes 1 GiB and 2 GiB dumps and the VESA
  comparison.
- Record any material resource change in `RESOURCES.md`.

## Non-Goals

- Do not add the VexRiscv/BIOS/serial SoC top level in this change.
- Do not retune FIFO sizes or change FT601 FSM behavior.
- Do not remove the old `pcie_squirrel.py`, `pcie_screamer.py`, or legacy
  gateware (`tlp.py`, `usb.py`, `etherbone.py`).
- Do not include the unrelated `platforms/pcie_squirrel.py` serial-pin edit.
- Do not change the PCILeech host-visible command/CFG/loopback protocol.

## Current Dead Code

The following are unused or unreachable in the active build:

- `pcileech_squirrel.py`: `TLP64To32Unpacker`,
  `StrideConverterBEFilter`, and the `StrideConverter` import.
- `gateware/pcileech_fifo.py`: `MuxSerializer2`; `MuxWordQueueTX` and the
  unreachable `elif 0`/`else` serializer branch.
- Root `befilter.py`: an untracked duplicate of `StrideConverterBEFilter`.
- `pcileech_squirrel.py`: the always-on `txsink_dbg`/`rxsink_dbg` capture
  blocks, flow/TLP/FT601 diagnostic counters, source/raw-RX seen latches, and
  dead `if 0` LED branches.
- `gateware/ft601.py`: `diag_filler_emit`, `diag_wrn0_accept`, and
  `diag_txen_high`.

## Cleanup Changes

### `pcileech_squirrel.py`

- Delete `TLP64To32Unpacker` and `StrideConverterBEFilter`.
- Remove the unused `StrideConverter` import.
- Remove the `txsink_dbg` and `rxsink_dbg` capture state and all wiring to
  `PCILeechFIFO`.
- Remove the flow counters, PCIe TLP counters, FT601 diagnostic CDC
  synchronizers, and their `diag_*` connections.
- Remove the `source_seen` and raw-RX-seen latches and their `PCILeechFIFO`
  connections.
- Remove the `if 0`/`elif 0` LED experiment branches.
- Keep `TLP32To64Packer`, `_CRG`, `PCIeSquirrel`, `FIFO_PRESETS`, the analyzer
  option, the loopback option, auto-BME, PCIe link-status wiring, and the
  PCIe timeout/recovery inputs.
- Remove obsolete `if 1:` wrappers around logic that remains functional.

### `gateware/pcileech_fifo.py`

- Delete `MuxSerializer2`.
- Delete `MuxWordQueueTX` and its unreachable branch; keep the active
  `MuxSerializer` plus burst-gate path.
- Remove diagnostic-only fields and logic:
  - `diag_tx_conv_seen`
  - `diag_rx64_seen`, `diag_rx32_seen`, `diag_ser_out_seen`,
    `diag_usbtx_seen`
  - `diag_rxfifo_in_seen`, `diag_rxfifo_out_seen`, `diag_mux_p3_wr_seen`
  - `diag_tx_tlp_seen`, `diag_rx_tlp_seen`, `diag_tx_err_drop_cnt`
  - `diag_tlp_rx_cpl_count`, `diag_tlp_rx_other_count`,
    `diag_tlp_rx_fifo_peak`, `diag_tlp_rx_stall_cnt`
  - `diag_ft601_filler_emit`, `diag_ft601_wrn0_accept`, `diag_ft601_txen_high`
  - debug capture arrays and flags (`tlp_tx_dbg*`, `tx64_dbg*`,
    `txsink_dbg*`, `rxsink_dbg*`, `rxsink_be`, `rxsink_lasts`,
    `rxsink_flags`)
  - `phy_source_seen`, `phy_raw_rx_seen`, and `tlp_rx_level`
- Remove custom readback cases above the protocol-required magic/version/
  device-ID registers. Preserve the magic, version, and device-ID responses.
- Keep and rename the functional timeout/recovery inputs:
  - `diag_tx_axis_seen` -> `tx_axis_seen`
  - `diag_rx_axis_seen` -> `rx_axis_seen`
  - `diag_tx_err_drop` -> `tx_err_drop`
  - `diag_force_pcie_reset` -> `force_pcie_reset`
- Keep the timeout/recovery state machine and the automatic PCIe reset pulse.
- Remove the timeout snapshot counters and readbacks (`snap_*`); keep the
  timeout detection and automatic PCIe reset pulse.

### `gateware/ft601.py`

- Remove the three diagnostic counters and their `usb`-domain update logic.
- Do not change the FSM, drain wait, sync filler, FIFO depths, or timeout.

### Repository

- Delete the untracked root `befilter.py`.
- Do not delete or modify old top levels or legacy gateware.
- Do not stage bitstreams, build output, the program image, or the unrelated
  platform serial-pin edit.

## Preserved Behavior

- FT601 sync filler and drain-wait behavior.
- FIFO presets and individual FIFO overrides.
- 64-to-128-bit TLP RX packing/filtering and 32-to-64-bit TX packing.
- PCILeech mux, serializer, CFG/loopback/CMD handling, and wire protocol.
- MSI generation and PCIe link/BME setup.
- PCIe timeout detection and the existing watchdog pulse. The pulse is not
  wired to `pcie_rst_n` in this build; automatic recovery is inactive.
- Optional loopback and LiteScope analyzer builds.

The only externally visible cleanup effect is that custom diagnostic readbacks
are absent or return zero. Standard PCILeech operation does not use them.

## Verification

1. Static checks:
   - `python3 -m py_compile pcileech_squirrel.py gateware/ft601.py gateware/pcileech_fifo.py`
   - Elaborate the default `large` build without building.
   - Elaborate with `--fifo-preset mini` to confirm preset paths still work.
2. Build the cleaned `large` bitstream and save it under `bitstreams/`.
3. Flash the cleaned bitstream and rescan the PCIe device.
4. Run the existing 1 GiB PCILeech benchmark:
   - Expect about 149 MiB/s and zero failed pages.
5. Run the 2 GiB benchmark:
   - Expect the previous valid-page rate (about 143-144 MiB/s).
   - Expect the known 40960-page hole count and no new failure pattern.
6. Compare the VESA ROM region `0x3f000-0x40000` byte-for-byte against the
   reference dump.
7. Inspect the placed utilization report:
   - BRAM should be unchanged.
   - LUT/FF should be equal or lower.
   - Update `RESOURCES.md` if the numbers move materially.
8. Keep the current known-good RX128 bitstream as a fallback. If the cleaned
   build regresses, reflash it and revert the cleanup.

## Commit Boundary

One focused cleanup commit containing only tracked active source changes and
any justified `RESOURCES.md` update. The untracked `befilter.py` is removed from
the working tree but does not appear in the commit. The design document is
committed separately. Exclude unrelated working-tree changes.

## Risks and Mitigations

- Removing a functional signal mistaken for diagnostics:
  - Keep the timeout/recovery inputs and verify elaboration and link behavior.
- Removing a custom readback used by a local script:
  - Standard PCILeech does not use the removed diagnostic registers; document
    the loss of custom diagnostics.
- Resource or timing regression:
  - Build, flash, and benchmark; retain the previous bitstream for rollback.
- Accidental scope expansion:
  - Do not touch legacy top levels, platform pins, or the future SoC.

## Future Work

After this cleanup is verified, design a separate top level that reuses the
PCIe/PCILeech core with VexRiscv, a small BIOS, and the serial console. The CPU
interface and host-safe memory-dump strategy are intentionally out of scope for
this cleanup.
