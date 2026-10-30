# Search SoC DMA Throughput Measurement and CPU-Visible Buffers

**Date:** 2026-10-30
**Status:** Design approved; spec awaiting user review; implementation not started
**Scope:** Search SoC only. First SoC and PCILeech variants are unchanged.

## Context

The search SoC currently reads host memory in 4 KiB MRd32/MRd64 chunks using
`gateware/chunk_dma.py`. Two 4 KiB buffers are ping-ponged: the next DMA starts
before the CPU scans the current buffer.

A 1 GiB high-address scan completed successfully at `0x100000000` and took
122.8 s, about 8.3 MiB/s. The scan was performed by `scan_buffer()` in
`software/soc_search_console/main.c`, which reads each 64-bit pair through the
`chunk_dma_rd_data` CSR. The measured rate includes both the DMA transfer and
the CPU scan, so it does not establish whether the PCIe read path or the CPU is
the limiting factor.

The target is to establish a PCIe read rate at least as fast as PCILeech over
USB, approximately 149 MiB/s. CPU instruction-level optimization is out of
scope for this change.

## Goal

1. Make the two DMA buffers directly readable by the CPU as one 8 KiB
   read-only memory region.
2. Replace per-16-byte CSR reads in the scan path with direct 32-bit loads.
3. Add an optional scan stride so CPU scan load can be varied without changing
   the DMA engine.
4. Measure 1 GiB high-address scans at stride 16 and stride 1024.
5. Accept the read path only if the stride-1024 scan reaches at least
   149 MiB/s and completes without errors.

The scan at stride 16 remains the normal search mode. The stride-1024 mode is
a diagnostic that checks only 4 offsets per 4 KiB, making CPU work negligible.

## Non-Goals

- Hardware pattern matching.
- Multiple outstanding DMA requests or PCIe tag pipelining.
- Pattern masking or variable-length matching.
- CPU instruction scheduling or assembly optimization.
- Changes to the first SoC or PCILeech variants.

## Architecture

### CPU-visible DMA buffers

`ChunkDMA` continues to own two 4 KiB buffers and their DMA write ports. It
adds a read-only CPU-facing interface covering 8 KiB:

- buffer 0 at CPU address `base + 0x0000`
- buffer 1 at CPU address `base + 0x1000`

The exact `base` is selected during implementation from an unused SoC address
region. The CPU bus is 32-bit; each 64-bit DMA word is exposed as two 32-bit
CPU words in little-endian order. The CPU cannot write the buffers.

The existing `peek` CSR path remains available as a debug fallback. The CPU
window and the CSR `peek` path are not used concurrently by the console.

### Scan command

`scan` gains an optional stride argument:

```text
scan <addr64> <len64> <w0> <w1> <w2> <w3> [stride]
```

- Default stride: 16 bytes.
- Stride must be nonzero, a multiple of 16, and no larger than the scan
  length.
- The scanner checks 16-byte-aligned offsets `0, stride, 2*stride, ...` within
  each DMA buffer.
- Match addresses remain `chunk_base + offset`.
- Existing validation for address alignment, length alignment, overflow, and
  pattern width remains unchanged.

At stride 1024 only four offsets per 4 KiB buffer are checked. This is a
measurement mode, not a replacement for normal scanning.

### Scanner implementation

`scan_buffer()` reads the selected buffer through the new CPU-visible region.
For each checked offset it reads four consecutive 32-bit words, applies the
existing `bswap32()` conversion to each word, and compares against the four
pattern words. The DMA data byte order and pattern semantics do not change.

The scanner remains a straightforward C loop. No assembly or CPU-specific
optimization is part of this change.

## Data Flow and Timing

The existing loop is retained:

1. Wait for the current DMA buffer.
2. Start the next DMA into the other buffer.
3. Scan the current buffer.

With two buffers, one DMA transfer overlaps one CPU scan. The loop rate is
`max(T_dma, T_scan)`:

- If `T_scan <= T_dma`, the loop is DMA-bound and hides one transfer's latency.
- If `T_dma <= T_scan`, the loop is CPU-bound.

Two buffers provide only one outstanding DMA request. They hide request
latency only while the CPU is scanning. If a single 4 KiB request cannot
sustain 149 MiB/s, additional outstanding requests are required.

The stride-1024 measurement makes `T_scan` negligible and therefore measures
the DMA path directly. The stride-16 measurement shows the combined path.

## Error Handling

- Invalid stride values produce a console error and do not start a scan.
- DMA completion, link loss, timeout, and malformed-completion handling are
  unchanged.
- A DMA error stops the scan and reports the existing error code.
- The CPU-visible region is read-only and must not overlap CSR, ROM, or SRAM
  regions.
- No new error recovery behavior is introduced.

## Verification

### Simulation and build

- Existing `test_chunk_dma.py` and `test_tlp_dma.py` simulations must continue
  to pass.
- Build `soc_search` and verify timing and resource reports.
- Verify the CPU-visible buffer address appears in the generated CSR/address
  map without overlaps.

### Hardware

1. Low VESA scan at stride 16 matches at `0x3f000`.
2. High scan at `0x100000000` with the known 16-byte pattern matches at
   `0x100000000`.
3. 1 GiB high scan at stride 16 completes; record elapsed time and MiB/s.
4. 1 GiB high scan at stride 1024 completes; record elapsed time and MiB/s.
5. If stride 1024 is at least 149 MiB/s, run the 6 GiB high scan and verify
   completion and range.

Elapsed time is measured on the host from command send to prompt return. The
UART progress output is part of the measured time but is emitted only every
256 MiB.

## Decision Point

- If stride 1024 is below 149 MiB/s, stop and design multiple outstanding 4 KiB
  reads or equivalent pipelining. Do not tune the CPU scanner in response.
- If stride 1024 passes but stride 16 is materially slower, the PCIe read path
  is accepted and the CPU scan rate becomes a separate follow-up decision.

## Affected Files

- `gateware/chunk_dma.py`: CPU-visible buffer interface, existing CSR retained.
- `soc_search.py`: bus mapping for the CPU-visible buffer region.
- `software/soc_search_console/main.c`: direct buffer scanning and optional
  stride argument.
- `test/test_chunk_dma.py`: regression check that completion data is readable
  through the CPU-visible buffer port.
- `docs/superpowers/specs/2026-10-30-soc-search-throughput-design.md`: this spec.
