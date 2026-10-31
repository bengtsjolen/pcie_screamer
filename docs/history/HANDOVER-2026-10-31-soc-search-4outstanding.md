# Handover: 4-Outstanding Search DMA Buffer Bug

> **Resolved 2026-10-31.** The write/commit path was fixed in commit
> `de0536f` ("fix: deliver four-outstanding completion data to search
> buffers") and merged to `master` in `c542474`. The four root causes were:
>
> 1. the `wr2_*` second write stage drove the BRAM ports but was never
>    loaded, so `wp*.we` never asserted;
> 2. `cpl_tag` was registered one cycle after the data capture indexed
>    the pending/write state with it, so data landed in the previous
>    tag's slot;
> 3. CplD byte enables were ignored, producing an extra zero word at the
>    end of even-length CplDs;
> 4. the ReadBench Wishbone window was only 11-bit word-addressed
>    (`address_width=13` is interpreted as bytes), so buffers 2/3
>    aliased buffers 0/1.
>
> The body below is the historical pre-fix failure record. See
> [VARIANTS.md](../../VARIANTS.md) for current status and verification.

**Date:** 2026-10-31
**Branch:** `soc`
**Last commit:** `5c30303 feat: add four-outstanding PCIe read benchmark`
**Status (at time of writing):** Blocked on a hardware write-path bug. The 2-buffer search is verified working; the 4-outstanding search is not.

## Goal

The desired search path is:

- Four outstanding 4 KiB PCIe MRd requests.
- Four 4 KiB buffers exposed to the CPU.
- The CPU scans each completed buffer at a sparse stride (1/16, 1/256, 1/1024).
- No hardware pattern comparator.
- Target: beat the 149 MiB/s PCILeech-over-USB rate; raw 4-outstanding DMA has already reached 426.7 MiB/s.

## Repository state

```text
branch: soc
HEAD:   5c30303
```

Uncommitted files at the time of this handover:

```text
M gateware/read_bench.py
M soc_search.py
M software/soc_search_console/main.c
M test/test_read_bench.py
```

Current bitstream:

```text
build/soc_search/gateware/soc_search.bin
SHA-256: 222285baa46dd7269ebcaa57aee5512b337e146c135a8c7e50efc74c9607bb51
```

This image is **unverified**: it was built and flashed after the unit test had already failed. The board may need a rescan or full power cycle.

## Verified working reference

The original 2-buffer search is known-good.

- Implementation: `gateware/chunk_dma.py` with two `Memory(64, 512)` buffers and one outstanding request.
- Console source at `e2ba260:software/soc_search_console/main.c`.
- Hardware verification:
  - Low VESA scan matched at `0x3f000`.
  - `peek` returned `pair0=0x000300f156455341`, `pair1=0x000022f000300100`.
- That console uses the `ChunkDMA` path by default (`bench_select=0`).
- This path was last verified after the 4-outstanding work began, so the 2-buffer path was not broken.

Known-good 2-buffer console source:

```bash
git show e2ba260:software/soc_search_console/main.c
```

The 2-buffer bitstream was overwritten during debugging; rebuild it from the legacy console if a known-good image is needed.

## Verified 4-outstanding facts

- Raw benchmark works:
  - 1 GiB at `0x40000000` in 2.4 s.
  - 426.7 MiB/s.
- The 4-outstanding request/tag engine can run 1 GiB scans at `stride=0x10` and `0x100` without completion errors:
  - `0x10`: 116.8 s, 8.8 MiB/s (CPU-limited).
  - `0x100`: 8.0 s, 127.8 MiB/s.
- These scans were not a correctness proof because the pattern was intentionally unmatchable and the buffer contents were not validated.
- A later `bench`/`stride=0x400` run returned `error 2` (completion status), pointing to stale completion/stop state. Stale completion handling and a SKIP state were added after that.

## Current 4-outstanding design

`gateware/read_bench.py`:

- Four tags (`tag_busy`, `tag_ready`, `release`, `auto_release`, `stop`).
- Four `Memory(64, 512)` buffers: `bmem0` .. `bmem3`.
- CPU read-only window at `0x20008000`, size `0x4000`, 32-bit CPU bus.
- Per-tag write index and pending data state (`wr0..3`, `pending0..3`, `pending_valid0..3`).
- Per-tag received DWORD accounting (`tag_data0..3`) and requested count (`tag_count0..3`).
- Completion parser states: `WAIT`, `CPL_H`, `CPL`, `CPL_END`, `COMMIT`, `SKIP`.
- `COMMIT` was added to delay tag-ready/free until after the final write.
- `wr2_en/wr2_addr/wr2_data/wr2_tag` is a second write stage.
- Debug CSRs:
  - `mem0`
  - `first_addr/first_data/first_tag/first_wp_we/first_test_we`
  - `fifo_seen/fifo_tag/fifo_addr/fifo_data/fifo_wp_we`
  - `cnt_seen/cnt_tag/cnt_count/cnt_data/cnt_len/cnt_new`
  - `test_addr/test_data/test_we` for the direct CPU write test.

`soc_search.py`:

- `ChunkDMA` remains instantiated.
- `ReadBench` is instantiated.
- `bench_select` muxes PCIe TX/RX between `ChunkDMA` and `ReadBench`.
- `ChunkDMA` CPU window: `0x20000000`, `0x2000`.
- `ReadBench` CPU window: `0x20008000`, `0x4000`.

`software/soc_search_console/main.c`:

- Current console is the 4-outstanding ring version.
- `scan` uses `ReadBench` with tag-ready/release.
- `bench` uses `ReadBench` in auto-release mode.
- `wtest` performs a direct CPU write to `bmem0[0]`.
- `rpeek` prints the first 32 bytes of `ReadBench` buffer 0 plus debug CSRs.

## The failure

Hardware low VESA scan:

```text
scan 0x3f000 0x1000 0x41534556 0xf1000300 0x00013000 0xf0220000 0x10
no match
```

`rpeek` after the scan:

```text
afafafaf afafafaf afafafaf afafafaf afafafaf afafafaf afafafaf afafafaf
```

At different points the engine also returned:

```text
error: ring dma 0x00000005   # completion tag not busy
error: ring dma 0x00000006   # unexpected TLP type
error: ring dma 0x00000002   # completion status
```

The low scan uses only one outstanding request/tag (`tag 0`), so this is **not** a four-outstanding concurrency problem.

## Evidence that rules out several theories

- **PCIe/request path is fine:** the raw bench reaches 426.7 MiB/s.
- **The tag/address/data capture is correct:**
  - `first_addr=0x000`
  - `first_data=0x000300f156455341` (sometimes the debug sampled before the staged write, showing stale/zero)
  - `first_tag=0x00`
  - `fifo_tag=0x0`
  - `fifo_addr=0x000`
  - `fifo_data=0x000300f156455341`
  - `fifo_wp_we=1`
- **The completion accounting is sane for the first CplD:**
  - `cnt_tag=0`
  - `cnt_count=0x400` (1024 DWORDs requested)
  - `cnt_data=0`
  - `cnt_len=0x040` (this CplD carries 64 DWORDs)
  - `cnt_new=0x040`
- **The CPU window and buffer readback work:**
  - Direct `wtest` writes `0x1122334455667788` to `bmem0[0]`.
  - `rpeek` returns `55667788 11223344 00000000 ...`.
- **The memory kind is the same as the working path:** both use `Memory(64, 512)`, one write port and one read port per 4 KiB buffer.
- **No two-cycle write is inherently required:** the 2-buffer ChunkDMA performs the same one-cycle write successfully.
- **The 4-outstanding count is not inherently the issue:** the failing low test has only tag 0 active.

## What remains

The write **request** is correct, but the actual `bmem0` contents remain BRAM initialization (`afaf…`). The failure is inside the `ReadBench` write/commit path, not PCIe or the CPU window.

The strongest structural difference from the working `ChunkDMA` is:

- `ChunkDMA` uses a **static buffer select** (`wr_buf`) and a `data_count`-derived write address.
- `ReadBench` uses a **dynamic completion tag** (`wr_tag`) plus per-tag `wr0..3` indexing and tag dispatch.
- `ReadBench` also maintains write state across several places: the completion FSM, a separate pending/write-index sync block, and the second `wr2` write stage. A one-clock skew between the tag/address and write-enable is plausible.

The one-clock `wr2` stage and `COMMIT` state were attempts to address that. The unit test still fails at `cpu_read` after `COMMIT`, so this hypothesis is not yet confirmed or fixed.

## Current unit test failure

```bash
venv/bin/python test/test_read_bench.py
```

Failure:

```text
File "test/test_read_bench.py", line 170, in tb
    assert (yield from cpu_read(dut, 0)) == 0xdeadbeef
AssertionError
```

The test waits for `STATUS_DONE`, waits four additional cycles, then reads `bmem0[0]`. It still gets 0. The `ReadBench` simulation must pass before further hardware work.

## Recommended next steps

1. **Fix the `ReadBench` unit test first.**
   - Instrument the actual BRAM write port: `wp0.we`, `wp0.adr`, `wp0.dat_w`, `wr2_en`, `wr2_addr`, `wr2_data`, `wr2_tag`, and `bmem0[0]`.
   - Capture the first actual `wp0.we`, not just the FSM request (`fifo_*` currently captures the request).
   - Verify the address and data at the exact edge the write occurs.

2. **Compare the generated write path with `ChunkDMA` line by line.**
   - `ChunkDMA` writes with `wr_en`, a static `wr_buf`, and `wr_addr=data_count>>1`.
   - Use the same structure per tag rather than the separate `wr0..3` array and tag dispatch, if possible.

3. **Prefer reusing the actual `ChunkDMA` module or its exact write submodule.**
   - The 2-buffer path is proven on hardware.
   - Copying only the timing has not fixed this.

4. **After the unit test passes, test hardware in order:**
   - `wtest` and `rpeek` (direct buffer path).
   - Low VESA `scan` with current pattern.
   - High scan; re-derive the high pattern from `rpeek` because host memory changes across reboots.
   - 1 GiB scan; then 6 GiB.
   - Remove debug CSRs, run all tests, commit.

5. **If the 4-outstanding path remains blocked, keep the verified 2-buffer search.**
   - It works at the cost of lower DMA throughput (~319 MiB/s raw, ~9 MiB/s CPU scan at stride 16).
   - The 4-outstanding raw benchmark can remain as a separate diagnostic.

## Hardware and recovery notes

- SoC console: `/dev/ttyUSB2`, 115200 baud.
- Build and flash:

```bash
source env.sh
venv/bin/python soc_search.py --build
./program.sh build/soc_search/gateware/soc_search.bin
sudo ./rescan.sh
```

- If `lspci -d 10ee:0666` shows no FPGA after reconfiguration, rescan first; if still absent, the PCIe link is wedged and needs a full machine power cycle. The FPGA slot has no per-slot software power control.
- The FT601 (`0403:601f`) is USB-only and not the PCIe link.
- The board has been observed to disappear from PCIe after repeated reconfiguration; a full power cycle restores it.

## Useful commands

```bash
# Unit tests
venv/bin/python test/test_read_bench.py
venv/bin/python test/test_chunk_dma.py
venv/bin/python test/test_tlp_dma.py

# Console
screen /dev/ttyUSB2 115200

# Raw 4-outstanding benchmark
bench 0x100000000 0x40000000

# Low VESA scan
scan 0x3f000 0x1000 0x41534556 0xf1000300 0x00013000 0xf0220000 0x10

# Direct buffer write/readback
wtest
rpeek

# 1 GiB sparse scans
scan 0x100000000 0x40000000 0xdeadbeef 0xfeedface 0xcafebabe 0x12345678 0x10
scan 0x100000000 0x40000000 0xdeadbeef 0xfeedface 0xcafebabe 0x12345678 0x100
scan 0x100000000 0x40000000 0xdeadbeef 0xfeedface 0xcafebabe 0x12345678 0x400
```

## Key files

- `gateware/chunk_dma.py` — verified 2-buffer DMA path.
- `gateware/read_bench.py` — current 4-outstanding engine and write path.
- `soc_search.py` — SoC integration, mux, CPU windows.
- `software/soc_search_console/main.c` — current 4-outstanding console.
- `test/test_read_bench.py` — failing unit test.
- `docs/superpowers/specs/2026-10-30-soc-search-throughput-design.md` — throughput spec.
- `docs/superpowers/plans/2026-10-30-soc-search-throughput.md` — original plan.
- `git show e2ba260:software/soc_search_console/main.c` — verified 2-buffer console.
- `/tmp/opencode/soc_search_main_4out.c` — saved 4-outstanding console (volatile; current `main.c` is also 4-outstanding).
- `build/soc_search/gateware/soc_search.bin` — current unverified image.

## Bottom line

The 2-buffer search works. The 4-outstanding raw engine works. The 4-outstanding search fails because the completion data is not reaching the per-tag BRAM buffers, even though the request/tag/address/data debug values are correct and the CPU-visible buffer path works. Fixing the `ReadBench` write/commit path is the remaining task.
