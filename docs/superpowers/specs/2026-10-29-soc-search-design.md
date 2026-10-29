# Standalone Raw-PCIe Search SoC Design

Date: 2026-10-29
Status: Approved for implementation planning
Branch: `soc`

## Summary

Add a third standalone SoC variant that searches a 64-bit physical address
range for an exact 16-byte-aligned 128-bit pattern. The design uses raw
LitePCIe bus-master reads and a new 4 KiB chunk DMA engine with two buffers,
then scans those buffers from plain C on the VexRiscv CPU. It deliberately
avoids `PCILeechFIFO`, the PCILeech mux, FT601, and the USB pipe.

The existing first SoC (`soc_squirrel.py`) and PCILeechFIFO variant
(`soc_pcileech.py`) remain unchanged.

## Goals

- Search up to a 6 GB range using 64-bit physical addresses and a 64-bit
  length. No 32-bit-address special case anywhere in the search path.
- Match an exact aligned 128-bit value at 16-byte-aligned candidate addresses.
- Use 4 KiB PCIe reads (MRd64) and a double 4 KiB buffer so DMA and CPU
  scanning overlap.
- Report the first match and stop. A caller can search again starting at
  `match + 16`.
- Use plain C for the scan loop on the existing VexRiscv CPU.
- Emit progress only occasionally; never print per chunk or per candidate.
- Target throughput comparable to the measured PCILeech path (~149 MiB/s),
  i.e. roughly 41 seconds for 6 GiB (about 38 seconds for 6 GB decimal),
  subject to hardware measurement.

## Non-Goals

- Masked/ignore-bit patterns. The compare function will carry a mask internally
  with all bits set; an eight-word mask command is future work.
- Unaligned or arbitrary-byte-offset pattern matching.
- A hardware signature engine. The DMA/buffer shape must leave room for one
  later, but this design uses the CPU for matching.
- Multiple outstanding PCIe requests in the first version. One 4 KiB MRd at a
  time with double buffering is the initial shape.
- PCILeech compatibility, FT601, USB, CMD/CFG/loopback, writes, or host-app
  integration.
- Reusing the existing `TLPDMA` module for search.

## Important Implementation Finding

The current `gateware/tlp_dma.py` is not a 4 KiB burst engine. Despite its
name, it issues one 4-byte MRd per word and stores each completed word in a
4 KiB buffer. That is the slow pattern this design exists to avoid.

The search SoC therefore gets a **new `ChunkDMA` module** that issues one 4 KiB
MRd64 per chunk and captures all returned completion data into the active
buffer. The first SoC's `TLPDMA` remains untouched for its existing `dump`
path and regression history.

## Architecture

```text
VexRiscv CPU
  ├── ROM (BIOS + scan console)
  ├── 8 KiB SRAM (stack/static state)
  ├── UART /dev/ttyUSB1
  └── CSR bus
        ├── ChunkDMA control/status
        ├── ChunkDMA buffer read port
        └── PCIe ID/link

ChunkDMA
  ├── MRd64 request generator
  ├── CplD completion parser
  ├── 2 × 4 KiB BRAM buffers
  └── LitePCIe PHY endpoints

S7PCIEPHY.tlp_tx/tlp_rx ── PCIe Gen2 x1 ── host root complex
```

### ChunkDMA

Required behavior:

- Always generate a 64-bit MRd. There is no 32-bit MRd mode.
- One request covers up to 1024 DWORDs (4 KiB). The first version uses 4 KiB
  chunks; the final chunk may be shorter.
- Use a rotating tag, the CPU/reference requester ID, and first/last byte
  enables set for the full requested length.
- Accept completion data for the active tag, and check completion status,
  requester ID, lower address, byte count consistency, and completion length.
  Completer ID is not required for the first version.
- A 4 KiB read may be returned as multiple CplD TLPs. The DMA writes returned
  payload DWORDs sequentially into the active buffer and completes only when
  the expected number of bytes is present.
- On any timeout, malformed completion, unexpected tag/requester, link loss,
  or short/long completion, stop, mark an error code, and leave the CPU bus
  idle. No silent partial success.
- Provide CSRs for address low/high, length in DWORDs, start, busy/done/error,
  error code, active buffer select, buffer read index, and buffer read data.
- The DMA writes into the inactive buffer; the CPU reads the completed buffer
  through the CSR read port. The two buffers alternate. The DMA must never
  overwrite the buffer the CPU is still scanning.

The exact CSR names and internal FSM are implementation details for the plan.

### Buffer and scan flow

1. Validate the command: address 16-byte aligned, length multiple of 16,
   no 64-bit overflow.
2. Initialize the scan range and issue the first chunk into buffer 0.
3. On completion, issue the next chunk into buffer 1 and scan buffer 0.
4. When the next completion arrives, scan buffer 1 while issuing into buffer 0.
5. For each 16-byte candidate, load the first 32-bit word. Only if it matches
   the filter word does the CPU load the remaining three words and compare the
   full 128-bit value.
6. On a match, stop issuing chunks, wait for any in-flight DMA to finish, print
   the match address, and return to the prompt.
7. If the range is exhausted, print `no match`.
8. Emit progress every 256 MB scanned.

The mask is all ones in this version. The compare is written as a masked
comparison so a later command can accept explicit value/mask words.

### CPU and scan cost

- 100 MHz VexRiscv; no data cache is assumed.
- A 4 KiB buffer contains 256 16-byte candidates.
- The filter path is one CSR read per candidate; the full four-word compare
  runs only on a filter hit.
- Expected filter cost is a few microseconds per 4 KiB, below the ~27 µs
  transfer time at 149 MiB/s. This is a design estimate, not a hardware
  guarantee.

## PCIe Protocol Details

### Request

- MRd64 TLP, 4 DWORD header.
- Length field 0 encodes 1024 DWORDs (4 KiB); a shorter final chunk uses its
  DWORD count directly.
- Requester ID is the SoC's PCIe ID; tag rotates per chunk.
- Address is the 64-bit physical address, 4-byte aligned.
- First and last byte enables cover the full requested payload.

### Completion

- Accept CplD only; status 0 is required for data.
- Check type, length/byte count consistency, tag, requester ID, lower address,
  and completion status.
- CplD data may be split across multiple completions. Data is consumed in
  memory order (little-endian DWORDs as stored in host memory).
- Completion data is written to the active 4 KiB buffer at the next DWORD
  offset.
- A no-data completion or completion error aborts the chunk with a diagnostic.

### Byte order

The CPU and host memory are little-endian. The scan pattern words are entered
in memory order. For the VESA signature at `0x3f000`:

```text
bytes: 56 45 53 41 00 03 00 f1 00 30 01 00 00 00 22 f0
words: 0x41534556 0xf1000300 0x00013000 0xf0220000
```

## Console Command

```text
scan <addr64> <len64> <w0> <w1> <w2> <w3>
```

- `addr64` and `len64` accept 64-bit hexadecimal values.
- `w0..w3` are the 128-bit pattern in little-endian memory order.
- Address must be 16-byte aligned; length must be a multiple of 16 and nonzero.
- Length is a 64-bit value; a full 6 GB scan is supported.
- Start address plus length must not overflow 64 bits.
- The scan stops at the first match.
- No arguments prints usage.

VESA verification command:

```text
scan 0x3f000 0x1000 0x41534556 0xf1000300 0x00013000 0xf0220000
```

Expected output: match at `0x3f000`.

A second search after a match can use `match + 16` as the new start address.

## Progress and Output

- Progress every 256 MB: percentage or scanned bytes and elapsed indication.
- No per-chunk or per-candidate output.
- On match: one line with the 64-bit address.
- On no match: one line.
- On failure: one concise error line with the DMA error code.
- The UART must not be used inside the per-candidate loop.

## Performance

Reference: the PCILeech path measured ~149 MiB/s in `PERFORMANCE.md`.

- 4 KiB at 149 MiB/s ≈ 27 µs.
- 6 GiB at 149 MiB/s ≈ 41 s; 6 GB decimal ≈ 38 s.
- The CPU filter cost should fit under the DMA time, giving overlap rather than
  addition.
- Initial acceptance target: record the 6 GB measured rate and aim for at least
  ~140 MiB/s. If the raw path is materially slower, profile the MRd/CPLD path
  before adding complexity.
- If a single 4 KiB MRd is inefficient, the chunk size can be reduced to 2 KiB
  without changing the command or buffer model.
- Multiple outstanding MRds are explicitly future work.

## Resource Estimate

- First SoC baseline: 2,050 LUT, 2,470 FF, 16 BRAM.
- Replacing the DMA buffer with two 4 KiB buffers and adding `ChunkDMA` is
  expected to add roughly one BRAM tile and a modest amount of logic.
- Target: fit comfortably within 50 BRAM tiles and keep 100 MHz timing clean.
- No fallback to a larger FPGA is planned.

## Error Handling

Abort the scan and report an error for:

- invalid command/address/length/alignment;
- PCIe link not ready;
- DMA timeout;
- invalid or unexpected completion;
- CplD error status;
- tag/requester/lower-address/byte-count mismatch;
- link loss during a chunk.

The scan must not report `no match` after a failed chunk. Do not return partial
results as success.

## Testing

### Simulation / unit tests

- `ChunkDMA` request generation: MRd64, 4 KiB length encoding, tag rotation,
  requester ID, address.
- Completion handling: one 4 KiB CplD, split CplDs, partial final chunk,
  data ordering, byte count/lower-address checks.
- Error paths: timeout, bad status, wrong tag, wrong requester, short
  completion, link loss.
- Ping-pong buffer selection: DMA writes only the inactive buffer; CPU read
  port sees the completed buffer.
- Scan compare: 16-byte alignment, first-word filter, exact 128-bit match,
  no match, mask-all-ones behavior.
- 64-bit address/length parsing and overflow rejection.

### Hardware verification

1. Flash the search SoC image; rescan PCIe; confirm status/link/BME.
2. Run the VESA scan command above; expect `0x3f000`.
3. Run a scan starting at `0x3f010` with the same pattern; expect `no match`.
4. Run a scan over a longer nonmatching range with a pattern not present;
   confirm progress and `no match`.
5. Run a full-range scan (e.g. 6 GB starting after the VESA region) to measure
   rate; confirm no per-candidate UART output and host stays alive.
6. Verify the requested byte range/address and 64-bit length handling.
7. Confirm the existing first SoC and PCILeechFIFO bitstreams remain unaffected.

## Acceptance Criteria

- `scan 0x3f000 0x1000 0x41534556 0xf1000300 0x00013000 0xf0220000`
  returns match `0x3f000`.
- A known-absent pattern reports `no match` only after the requested range was
  scanned or a real error was reported.
- A 6 GB scan uses 64-bit addresses/lengths, shows progress, and does not print
  per candidate.
- Measured throughput is recorded; target is at least ~140 MiB/s, with the
  raw-PCIe result compared to the PCILeech reference.
- The design fits the XC7A35T with positive setup/hold timing at 100 MHz, apart
  from any already-accepted PCIe hard-IP pulse-width caveat.
- Existing SoC variants and PCILeech gateware are not modified.
- No PCILeechFIFO, FT601, USB pipe, or PCILeech mux is instantiated in the
  search SoC.

## Future Work

- Masked 128-bit patterns with explicit ignore bits.
- Hardware streaming matcher fed directly by the DMA path.
- Multiple outstanding 4 KiB/2 KiB requests.
- Optional result streaming instead of stop-at-first.
- PCILeech-compatible variant only if host-app integration becomes a goal.

## File Layout (planned)

```text
soc_search.py
gateware/chunk_dma.py
software/soc_search_console/{main.c,Makefile,linker.ld}
test/test_chunk_dma.py
test/test_search_compare.py
docs/superpowers/specs/2026-10-29-soc-search-design.md
docs/superpowers/plans/2026-10-29-soc-search.md
```

The implementation plan will define exact module interfaces, CSR names, FSM
states, and verification commands.
