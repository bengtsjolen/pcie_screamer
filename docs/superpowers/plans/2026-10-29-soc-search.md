# Standalone Raw-PCIe Search SoC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a standalone raw-LitePCIe search SoC that DMAs 4 KiB MRd64 chunks into two buffers and scans them on the VexRiscv for an exact 16-byte-aligned 128-bit pattern.

**Architecture:** New `soc_search.py` reused from `soc_squirrel.py`; a new `gateware/chunk_dma.py` that issues one 4 KiB MRd64 and writes 64-bit packed pairs into two 4 KiB buffers; a custom no-libc BIOS with a `scan` command. No `PCILeechFIFO`, FT601, mux, bridge, or USB path.

**Tech Stack:** Migen/LiteX/LitePCIe, VexRiscv `minimal`, Xilinx 7-series, Python simulations, C BIOS.

---

## File Structure

- `gateware/chunk_dma.py` — new 4 KiB MRd64 DMA engine and 2 × 4 KiB buffers.
- `soc_search.py` — new SoC top level; copies the verified first SoC plumbing.
- `software/soc_search_console/{main.c,Makefile,linker.ld}` — scan console.
- `test/test_chunk_dma.py` — simulation coverage for request/completion/error/ping-pong.
- `SOC_SEARCH.md` — user-facing variant documentation.
- `RESOURCES.md` — search SoC resource/verification row.
- `docs/superpowers/specs/2026-10-29-soc-search-design.md` — approved design.

Do not modify `soc_squirrel.py`, `soc_pcileech.py`, `gateware/tlp_dma.py`, or the existing PCILeech gateware.

## Task 1: ChunkDMA request generation

**Files:**
- Create: `gateware/chunk_dma.py`
- Create: `test/test_chunk_dma.py`

- [ ] **Step 1: Write the failing request test**

Create `test/test_chunk_dma.py`:

```python
#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from migen import *
from migen.sim import run_simulation

from gateware.chunk_dma import ChunkDMA


REQ_ID = 0x1600
STATUS_LINK_READY = 1 << 0
STATUS_DONE = 1 << 2
STATUS_ERROR = 1 << 3


def expect_mrd64(dut, addr, length, tag):
    beats = []
    for _ in range(2):
        for _ in range(200):
            if (yield dut.tlp_tx.valid):
                break
            yield
        else:
            raise AssertionError("TLP beat not emitted")
        beats.append(((yield dut.tlp_tx.dat), (yield dut.tlp_tx.last),
                      (yield dut.tlp_tx.be)))
        yield

    length_field = 0 if length == 1024 else length
    last_be = 0x0f if length == 1 else 0xff
    dw0 = 0x20000000 | length_field
    dw1 = (REQ_ID << 16) | (tag << 8) | last_be
    expected0 = (dw1 << 32) | dw0
    expected1 = ((addr >> 32) << 32) | (addr & 0xfffffffc)

    assert beats[0][0] == expected0, hex(beats[0][0])
    assert beats[0][1] == 0
    assert beats[0][2] == 0xff
    assert beats[1][0] == expected1, hex(beats[1][0])
    assert beats[1][1] == 1
    assert beats[1][2] == 0xff


def tb_request(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield dut.addr_lo.write(0x0003f000)
    yield dut.addr_hi.write(1)
    yield dut.length.write(1024)
    yield dut.buffer.write(0)
    yield dut.start.write(1)
    yield from expect_mrd64(dut, 0x1_0003_f000, 1024, 0)


def main():
    dut = ChunkDMA()
    run_simulation(dut, tb_request(dut))
    print("ChunkDMA request simulation OK")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run it and confirm it fails**

Run:

```bash
venv/bin/python test/test_chunk_dma.py
```

Expected: `ModuleNotFoundError: No module named 'gateware.chunk_dma'`.

- [ ] **Step 3: Add the request/CSR skeleton**

Create `gateware/chunk_dma.py`:

```python
from migen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

from litepcie.common import phy_layout

from gateware.tlp32 import TLP32To64Packer


class ChunkDMA(Module, AutoCSR):
    def __init__(self, timeout_cycles=1_000_000):
        self.addr_lo = CSRStorage(32)
        self.addr_hi = CSRStorage(32)
        self.length = CSRStorage(11)
        self.buffer = CSRStorage(1)
        self.start = CSRStorage(1)
        self.status = CSRStatus(4)
        self.error = CSRStatus(4)
        self.rd_buffer = CSRStorage(1)
        self.rd_index = CSRStorage(9)
        self.rd_data = CSRStatus(64)

        self.link_ready = Signal()
        self.requester_id = Signal(16)
        self.tlp_rx = stream.Endpoint(phy_layout(64))
        self.submodules.packer = packer = ResetInserter()(TLP32To64Packer())
        self.tlp_tx = packer.source
        sink = packer.sink

        mem0 = Memory(64, 512)
        mem1 = Memory(64, 512)
        wp0 = mem0.get_port(write_capable=True)
        wp1 = mem1.get_port(write_capable=True)
        rp0 = mem0.get_port()
        rp1 = mem1.get_port()
        self.specials += mem0, mem1, wp0, wp1, rp0, rp1

        busy = Signal()
        done = Signal()
        error = Signal()
        error_code = Signal(4)
        tag = Signal(8)
        req_tag = Signal(8)
        addr = Signal(64)
        count = Signal(11)
        word = Signal(2)
        timeout = Signal(max=timeout_cycles)

        wr_buf = Signal()
        wr_en = Signal()
        wr_addr = Signal(9)
        wr_data = Signal(64)

        self.comb += [
            self.status.status.eq(Cat(self.link_ready, busy, done, error)),
            self.error.status.eq(error_code),
            rp0.adr.eq(self.rd_index.storage),
            rp1.adr.eq(self.rd_index.storage),
            self.rd_data.status.eq(Mux(self.rd_buffer.storage,
                                       rp1.dat_r, rp0.dat_r)),
            wp0.adr.eq(wr_addr),
            wp1.adr.eq(wr_addr),
            wp0.dat_w.eq(wr_data),
            wp1.dat_w.eq(wr_data),
            wp0.we.eq(wr_en & ~wr_buf),
            wp1.we.eq(wr_en & wr_buf),
            sink.valid.eq(0),
            sink.dat.eq(0),
            sink.be.eq(0),
            sink.last.eq(0),
        ]

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            If(self.start.re & self.start.storage & ~busy,
                NextValue(done, 0),
                NextValue(error, 0),
                NextValue(error_code, 0),
                If(~self.link_ready,
                    NextValue(error, 1),
                    NextValue(error_code, 1),
                    NextState("ERROR"),
                ).Elif((self.length.storage == 0) |
                       (self.length.storage > 1024),
                    NextValue(error, 1),
                    NextValue(error_code, 5),
                    NextState("ERROR"),
                ).Else(
                    NextValue(busy, 1),
                    NextValue(req_tag, tag),
                    NextValue(tag, tag + 1),
                    NextValue(addr, Cat(self.addr_lo.storage,
                                        self.addr_hi.storage) &
                                      0xFFFFFFFFFFFFFFFC),
                    NextValue(count, self.length.storage),
                    NextValue(wr_buf, self.buffer.storage),
                    NextValue(word, 0),
                    NextState("ISSUE"),
                ),
            )
        )

        fsm.act("ISSUE",
            sink.valid.eq(self.link_ready),
            sink.be.eq(0xf),
            Case(word, {
                0: sink.dat.eq(0x20000000 |
                               Mux(count == 1024, 0, count)),
                1: sink.dat.eq((self.requester_id << 16) |
                               (req_tag << 8) |
                               Mux(count == 1, 0x0f, 0xff)),
                2: sink.dat.eq(addr[0:32]),
                3: sink.dat.eq(addr[32:64]),
            }),
            If(word == 3, sink.last.eq(1)),
            If(~self.link_ready,
                packer.reset.eq(1),
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 1),
                NextState("ERROR"),
            ).Elif(sink.ready,
                If(word == 3,
                    NextValue(timeout, 0),
                    NextState("CPL_H0"),
                ).Else(
                    NextValue(word, word + 1),
                ),
            ),
        )

        # Completion parsing is added in Task 2.
        fsm.act("CPL_H0", NextState("CPL_H0"))

        fsm.act("ERROR",
            packer.reset.eq(1),
            NextValue(busy, 0),
            NextValue(error, 1),
            NextState("IDLE"),
        )
```

- [ ] **Step 4: Run the test and confirm it passes**

Run:

```bash
venv/bin/python test/test_chunk_dma.py
```

Expected: `ChunkDMA request simulation OK`.

- [ ] **Step 5: Commit**

```bash
git add gateware/chunk_dma.py test/test_chunk_dma.py
git commit -m "add ChunkDMA request generation"
```

## Task 2: Single-chunk completion and 64-bit buffer

**Files:**
- Modify: `gateware/chunk_dma.py`
- Modify: `test/test_chunk_dma.py`

- [ ] **Step 1: Add the completion test helpers and 4 KiB test**

Append these helpers and test to `test/test_chunk_dma.py`:

```python
def beat(dut, dw0, dw1, be=0xff, last=0):
    yield dut.tlp_rx.dat.eq((dw1 << 32) | dw0)
    yield dut.tlp_rx.be.eq(be)
    yield dut.tlp_rx.last.eq(last)
    yield dut.tlp_rx.valid.eq(1)
    yield
    for _ in range(100):
        if (yield dut.tlp_rx.ready):
            break
        yield
    else:
        raise AssertionError("tlp_rx.ready not asserted")
    yield dut.tlp_rx.valid.eq(0)


def cpl_chunk(dut, data, addr, tag):
    assert 1 <= len(data) <= 1024
    length = len(data) & 0x3ff
    dw0 = (0x4a << 24) | length
    dw1 = (len(data) * 4) & 0xfff
    dw2 = (REQ_ID << 16) | ((tag & 0xff) << 8) | (addr & 0x7f)

    beats = [(dw0, dw1, 0xff, 0)]
    beats.append((dw2, data[0], 0xff, len(data) == 1))
    i = 1
    while i + 1 < len(data):
        last = int(i + 2 == len(data))
        beats.append((data[i], data[i + 1], 0xff, last))
        i += 2
    if i < len(data):
        beats.append((data[i], 0, 0x0f, 1))

    for dw0, dw1, be, last in beats:
        yield from beat(dut, dw0, dw1, be, last)


def wait_status(dut, mask, timeout=4000):
    for _ in range(timeout):
        status = yield from dut.status.read()
        if status & mask:
            return status
        yield
    raise AssertionError("status mask not reached")


def read64(dut, buffer, index):
    yield dut.rd_buffer.write(buffer)
    yield from dut.rd_index.write(index)
    yield
    yield
    return (yield from dut.rd_data.read())


def tb_single(dut):
    data = [0x10000000 + i for i in range(1024)]

    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x0003f000)
    yield from dut.addr_hi.write(1)
    yield from dut.length.write(1024)
    yield from dut.buffer.write(0)
    yield from dut.start.write(1)

    yield from expect_mrd64(dut, 0x1_0003_f000, 1024, 0)
    yield from cpl_chunk(dut, data, 0x3f000, 0)

    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    for i in range(512):
        value = yield from read64(dut, 0, i)
        assert (value & 0xffffffff) == data[2 * i]
        assert (value >> 32) == data[2 * i + 1]
```

Update `main()`:

```python
def main():
    dut = ChunkDMA()
    run_simulation(dut, tb_request(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_single(dut))

    print("ChunkDMA simulation OK")
```

- [ ] **Step 2: Run it and confirm it fails**

Run:

```bash
venv/bin/python test/test_chunk_dma.py
```

Expected: FAIL — the test hangs in `CPL_H0` and never reports `DONE`.

- [ ] **Step 3: Replace the temporary completion state with the parser**

In `gateware/chunk_dma.py`, add these signals next to `count`:

```python
        data_count = Signal(11)
        cpl_len = Signal(11)
        cpl_count = Signal(11)
        pending = Signal(32)
        pending_valid = Signal()
```

Replace the temporary `fsm.act("CPL_H0", NextState("CPL_H0"))` with:

```python
        fsm.act("CPL_H0",
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid,
                If(self.tlp_rx.dat[24:32] != 0x4a,
                    NextValue(busy, 0),
                    NextValue(error, 1),
                    NextValue(error_code, 3),
                    NextState("ERROR"),
                ).Elif(self.tlp_rx.dat[45:48] != 0,
                    NextValue(busy, 0),
                    NextValue(error, 1),
                    NextValue(error_code, 2),
                    NextState("ERROR"),
                ).Elif(self.tlp_rx.dat[32:44] !=
                       Mux(self.tlp_rx.dat[0:10] == 0, 0,
                           self.tlp_rx.dat[0:10] * 4),
                    NextValue(busy, 0),
                    NextValue(error, 1),
                    NextValue(error_code, 3),
                    NextState("ERROR"),
                ).Elif(Mux(self.tlp_rx.dat[0:10] == 0, 1024,
                           self.tlp_rx.dat[0:10]) > (count - data_count),
                    NextValue(busy, 0),
                    NextValue(error, 1),
                    NextValue(error_code, 3),
                    NextState("ERROR"),
                ).Else(
                    NextValue(cpl_len, Mux(self.tlp_rx.dat[0:10] == 0,
                                           1024, self.tlp_rx.dat[0:10])),
                    NextValue(cpl_count, 0),
                    NextValue(timeout, 0),
                    NextState("CPL_H1"),
                )
            ).Elif(~self.link_ready,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 1),
                NextState("ERROR"),
            ).Elif(timeout == timeout_cycles - 1,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 4),
                NextState("ERROR"),
            ).Else(
                NextValue(timeout, timeout + 1),
            ),
        )

        fsm.act("CPL_H1",
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid,
                If((self.tlp_rx.dat[0:7] !=
                    (addr + (data_count << 2))[0:7]) |
                   (self.tlp_rx.dat[8:16] != req_tag) |
                   (self.tlp_rx.dat[16:32] != self.requester_id),
                    NextValue(busy, 0),
                    NextValue(error, 1),
                    NextValue(error_code, 3),
                    NextState("ERROR"),
                ).Else(
                    NextValue(data_count, data_count + 1),
                    NextValue(cpl_count, 1),
                    If(pending_valid,
                        NextValue(wr_en, 1),
                        NextValue(wr_addr, data_count >> 1),
                        NextValue(wr_data, Cat(pending,
                                               self.tlp_rx.dat[32:64])),
                        NextValue(pending_valid, 0),
                    ).Else(
                        NextValue(pending, self.tlp_rx.dat[32:64]),
                        NextValue(pending_valid, 1),
                    ),
                    If(self.tlp_rx.last,
                        NextState("CPL_END"),
                        NextState("CPL_D"),
                    ),
                )
            ).Elif(~self.link_ready,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 1),
                NextState("ERROR"),
            ).Elif(timeout == timeout_cycles - 1,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 4),
                NextState("ERROR"),
            ).Else(
                NextValue(timeout, timeout + 1),
            ),
        )

        fsm.act("CPL_D",
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid,
                If((self.tlp_rx.be[0:4] != 0) & (self.tlp_rx.be[4:8] != 0),
                    If(pending_valid,
                        NextValue(wr_en, 1),
                        NextValue(wr_addr, data_count >> 1),
                        NextValue(wr_data, Cat(pending,
                                               self.tlp_rx.dat[0:32])),
                        NextValue(pending, self.tlp_rx.dat[32:64]),
                        NextValue(pending_valid, 1),
                    ).Else(
                        NextValue(wr_en, 1),
                        NextValue(wr_addr, data_count >> 1),
                        NextValue(wr_data, Cat(self.tlp_rx.dat[0:32],
                                               self.tlp_rx.dat[32:64])),
                        NextValue(pending_valid, 0),
                    ),
                    NextValue(data_count, data_count + 2),
                    NextValue(cpl_count, cpl_count + 2),
                ).Elif(self.tlp_rx.be[0:4] != 0,
                    If(pending_valid,
                        NextValue(wr_en, 1),
                        NextValue(wr_addr, data_count >> 1),
                        NextValue(wr_data, Cat(pending,
                                               self.tlp_rx.dat[0:32])),
                        NextValue(pending_valid, 0),
                    ).Else(
                        NextValue(pending, self.tlp_rx.dat[0:32]),
                        NextValue(pending_valid, 1),
                    ),
                    NextValue(data_count, data_count + 1),
                    NextValue(cpl_count, cpl_count + 1),
                ).Elif(self.tlp_rx.be[4:8] != 0,
                    If(pending_valid,
                        NextValue(wr_en, 1),
                        NextValue(wr_addr, data_count >> 1),
                        NextValue(wr_data, Cat(pending,
                                               self.tlp_rx.dat[32:64])),
                        NextValue(pending_valid, 0),
                    ).Else(
                        NextValue(pending, self.tlp_rx.dat[32:64]),
                        NextValue(pending_valid, 1),
                    ),
                    NextValue(data_count, data_count + 1),
                    NextValue(cpl_count, cpl_count + 1),
                ),
                If(self.tlp_rx.last,
                    NextState("CPL_END"),
                ),
            ).Elif(~self.link_ready,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 1),
                NextState("ERROR"),
            ).Elif(timeout == timeout_cycles - 1,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 4),
                NextState("ERROR"),
            ).Else(
                NextValue(timeout, timeout + 1),
            ),
        )

        fsm.act("CPL_END",
            If(cpl_count != cpl_len,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 3),
                NextState("ERROR"),
            ).Elif(data_count > count,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 3),
                NextState("ERROR"),
            ).Elif(data_count == count,
                If(pending_valid,
                    NextValue(wr_en, 1),
                    NextValue(wr_addr, data_count >> 1),
                    NextValue(wr_data, Cat(pending, Constant(0, 32))),
                ),
                NextValue(busy, 0),
                NextValue(done, 1),
                NextState("IDLE"),
            ).Else(
                NextValue(timeout, 0),
                NextState("CPL_H0"),
            ),
        )
```

Notes for the implementer:

- `cpl_count` counts DWORDs accepted in the current CplD TLP; it must
  equal `cpl_len` when the TLP ends.
- `data_count` counts DWORDs across all CplDs for the chunk.
- `pending`/`pending_valid` pack the first data DWORD of a CplD with the next
  DWORD into a 64-bit buffer word.
- `wr_en` must be a one-cycle write strobe. The `NextValue(wr_en, 1)` snippets
  below are illustrative; implement `wr_en` as a combinational strobe that is
  low by default and asserted only in the cycle that writes a buffer word, or
  explicitly clear it in every parser state. Do not leave it asserted after
  the write; the test reads the complete buffer and will catch repeated writes.
- Task 1 review fixed Last DW BE to `Mux(count == 1, 0x0f, 0xff)` and masks
  the captured address to 4-byte alignment. Do not regress those fixes.
- Handle lower+upper byte enables together in `CPL_D`; do not evaluate the
  upper half using the old `pending_valid` after the lower half has already
  consumed the pending word.
- Every state must have the link-loss and timeout checks shown.
- `CPL_END` writes a final padded word only if `pending_valid` is set. The
  search command always scans a multiple of 16 bytes, so the normal chunk is
  even, but the guard keeps the buffer deterministic.

- [ ] **Step 4: Run the test and confirm it passes**

Run:

```bash
venv/bin/python test/test_chunk_dma.py
```

Expected: `ChunkDMA simulation OK`.

- [ ] **Step 5: Commit**

```bash
git add gateware/chunk_dma.py test/test_chunk_dma.py
git commit -m "add ChunkDMA completion parser and buffers"
```

## Task 3: Split completions, ping-pong, and errors

**Files:**
- Modify: `gateware/chunk_dma.py`
- Modify: `test/test_chunk_dma.py`

**Required carry-over from Task 2 review:**

- Reset `data_count`, `pending`, `pending_valid`, `cpl_len`, and `cpl_count`
  in the IDLE start branch before ISSUE. Without this, the second transfer
  fails because `data_count` is already at `count`.
- Make the `CPL_H0` remaining-length guard underflow-safe:
  reject when `data_count >= count` or
  `cpl_len > count - data_count`.
- Restore the single-DW regression in `main()` (already done in `438e80e`).
- Add restart coverage: the ping-pong test is the primary multi-start test;
  also add a second start after a valid failure.
- Add negative completions for wrong lower address, wrong byte count, wrong
  requester ID, and a CplD length larger than the remaining chunk.

- [ ] **Step 1: Add failing split/ping-pong/error tests**

Append:

```python
def tb_split(dut):
    data = [0x20000000 + i for i in range(1024)]

    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x0003f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1024)
    yield from dut.buffer.write(0)
    yield from dut.start.write(1)

    yield from expect_mrd64(dut, 0x3f000, 1024, 0)
    yield from cpl_chunk(dut, data[:512], 0x3f000, 0)
    yield from cpl_chunk(dut, data[512:], 0x3f800, 0)

    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    for i in range(512):
        value = yield from read64(dut, 0, i)
        assert (value & 0xffffffff) == data[2 * i]
        assert (value >> 32) == data[2 * i + 1]


def tb_pingpong(dut):
    first = [0x30000000 + i for i in range(1024)]
    second = [0x40000000 + i for i in range(1024)]

    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1024)
    yield from dut.buffer.write(0)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 1024, 0)
    yield from cpl_chunk(dut, first, 0x3f000, 0)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE

    yield from dut.addr_lo.write(0x40000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1024)
    yield from dut.buffer.write(1)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x40000, 1024, 1)
    yield from cpl_chunk(dut, second, 0x40000, 1)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE

    assert (yield from read64(dut, 0, 0)) == (
        (first[1] << 32) | first[0])
    assert (yield from read64(dut, 1, 0)) == (
        (second[1] << 32) | second[0])


def tb_partial(dut):
    data = [0x50000000 + i for i in range(16)]

    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x0003f010)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(16)
    yield from dut.buffer.write(0)
    yield from dut.start.write(1)

    yield from expect_mrd64(dut, 0x3f010, 16, 0)
    yield from cpl_chunk(dut, data, 0x3f010, 0)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE

    for i in range(8):
        value = yield from read64(dut, 0, i)
        assert (value & 0xffffffff) == data[2 * i]
        assert (value >> 32) == data[2 * i + 1]


def tb_error(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(0)
    yield from dut.start.write(1)
    status = yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 5


def tb_cpl_error(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)

    yield from beat(dut, (0x4a << 24) | 1, 1 << 13, be=0xff, last=0)
    status = yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 2


def tb_tag_error(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)

    yield from cpl_chunk(dut, [0x12345678], 0x3f000, 1)
    status = yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 3
```

Update `main()`:

```python
    dut = ChunkDMA()
    run_simulation(dut, tb_split(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_pingpong(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_partial(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_error(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_cpl_error(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_tag_error(dut))
```

- [ ] **Step 2: Run the new tests**

Run:

```bash
venv/bin/python test/test_chunk_dma.py
```

Expected: `tb_split` and `tb_partial` may pass because the Task 2 parser
already returns to `CPL_H0` when `data_count < count`. `tb_pingpong` should
fail on the second start until the IDLE lifecycle reset is added in Step 3.
`tb_error`/`tb_cpl_error`/`tb_tag_error` should pass. Do not weaken tests.

- [ ] **Step 3: Fix lifecycle, underflow, and add negative coverage**

First add the resets to the IDLE start branch:

```python
                    NextValue(data_count, 0),
                    NextValue(pending, 0),
                    NextValue(pending_valid, 0),
                    NextValue(cpl_len, 0),
                    NextValue(cpl_count, 0),
```

Then make the `CPL_H0` remaining guard underflow-safe:

```python
                ).Elif((data_count >= count) |
                       (Mux(self.tlp_rx.dat[0:10] == 0, 1024,
                            self.tlp_rx.dat[0:10]) >
                        (count - data_count)),
```

The Task 2 parser already returns to `CPL_H0` when `data_count < count`, so a
split CplD should work. Verify the lower-address check uses the *total* data
count, not the per-CplD count.

Add these negative/restart tests:

```python
def start_one(dut, length=1, buffer=0):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(length)
    yield from dut.buffer.write(buffer)
    yield from dut.start.write(1)


def tb_lower_error(dut):
    yield from start_one(dut)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    yield from beat(dut, (0x4a << 24) | 1, 4, be=0xff, last=0)
    yield from beat(dut, (REQ_ID << 16) | 4, 0x11111111,
                    be=0xf0, last=1)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 3


def tb_byte_error(dut):
    yield from start_one(dut)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    yield from beat(dut, (0x4a << 24) | 1, 8, be=0xff, last=0)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 3


def tb_requester_error(dut):
    yield from start_one(dut)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    yield from beat(dut, (0x4a << 24) | 1, 4, be=0xff, last=0)
    yield from beat(dut, (0x1111 << 16) | 0, 0x11111111,
                    be=0xf0, last=1)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 3


def tb_long_error(dut):
    yield from start_one(dut)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    yield from beat(dut, (0x4a << 24) | 2, 8, be=0xff, last=0)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 3


def tb_retry(dut):
    yield from start_one(dut, length=0)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 5

    yield from start_one(dut, length=16)
    yield from expect_mrd64(dut, 0x3f000, 16, 0)
    yield from cpl_chunk(dut, [0x60000000 + i for i in range(16)],
                         0x3f000, 0)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
```

Add these to `main()`:

```python
    dut = ChunkDMA()
    run_simulation(dut, tb_lower_error(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_byte_error(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_requester_error(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_long_error(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_retry(dut))
```

If `tb_split` fails, inspect:

```bash
venv/bin/python - <<'PY'
from gateware.chunk_dma import ChunkDMA
from migen.fhdl import verilog
verilog.convert(ChunkDMA())
print("ChunkDMA Verilog OK")
PY
```

Then adjust the parser transition so `CPL_END` goes to `CPL_H0` for the next
CplD and only marks `done` when `data_count == count`.

Add a timeout test with a short timeout instance:

```python
def tb_timeout(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    status = yield from wait_status(dut, STATUS_ERROR, timeout=200)
    assert (yield from dut.error.read()) == 4
```

Add to `main()`:

```python
    dut = ChunkDMA(timeout_cycles=20)
    run_simulation(dut, tb_timeout(dut))
```

- [ ] **Step 4: Run all ChunkDMA tests**

Run:

```bash
venv/bin/python test/test_chunk_dma.py
```

Expected: `ChunkDMA simulation OK`.

- [ ] **Step 5: Commit**

```bash
git add gateware/chunk_dma.py test/test_chunk_dma.py
git commit -m "handle split ChunkDMA completions and ping-pong buffers"
```

## Task 4: Search SoC top level

**Files:**
- Create: `soc_search.py`
- Read: `soc_squirrel.py`
- Test: `test/test_chunk_dma.py` (no new top-level test file required)

- [ ] **Step 1: Copy the verified first SoC**

```bash
cp soc_squirrel.py soc_search.py
```

- [ ] **Step 2: Replace the DMA wiring**

In `soc_search.py`, remove `from gateware.tlp_dma import TLPDMA` and add:

```python
from gateware.chunk_dma import ChunkDMA
```

Replace the `TLPDMA` block with:

```python
        self.submodules.chunk_dma = chunk_dma = ChunkDMA()
        self.add_csr("chunk_dma")

        self.comb += [
            self.pcie_phy.source.connect(chunk_dma.tlp_rx),
            chunk_dma.tlp_tx.connect(self.pcie_phy.sink),
            chunk_dma.link_ready.eq(
                self.pcie_phy._link_status.fields.status
                & self.pcie_phy._bus_master_enable.status
                & (self.pcie_phy.id != 0)
            ),
            chunk_dma.requester_id.eq(self.pcie_phy.id),
        ]
```

Change the class name to `SoCSearch`, the build name to
`"soc_search"`, the output directory to `"build/soc_search"`, the CSR CSV to
`"test/soc_search_csr.csv"`, and the software directory to
`"software/soc_search_console"`.

- [ ] **Step 3: Verify gateware elaboration**

The scan console does not exist until Task 5, so do not run the full builder
yet. Verify the gateware top level directly:

```bash
python3 -m py_compile soc_search.py gateware/chunk_dma.py
venv/bin/python - <<'PY'
from platforms.pcie_squirrel import Platform
from soc_search import SoCSearch

soc = SoCSearch(Platform())
soc.finalize()
print("SoCSearch elaboration OK")
PY
```

Expected: `SoCSearch elaboration OK`; no BIOS build yet. Task 5 creates the
console and Task 6 runs the full build.

- [ ] **Step 4: Commit**

```bash
git add soc_search.py
git commit -m "add search SoC top level"
```

## Task 5: Scan console

**Files:**
- Create: `software/soc_search_console/main.c`
- Create: `software/soc_search_console/Makefile`
- Create: `software/soc_search_console/linker.ld`
- Read: `software/soc_console/main.c`

- [ ] **Step 1: Copy the first console package**

```bash
mkdir -p software/soc_search_console
cp software/soc_console/main.c software/soc_search_console/main.c
cp software/soc_console/Makefile software/soc_search_console/Makefile
cp software/soc_console/linker.ld software/soc_search_console/linker.ld
```

Remove the copied `dump_memory()` function, its `dump` command branch, and
its help line. Change `print_status()` to read
`chunk_dma_status_read()` instead of `tlp_dma_status_read()`. This console
only needs `help`, `status`, and `scan`.

- [ ] **Step 2: Add 64-bit helpers and the scan function**

In `software/soc_search_console/main.c`, add:

```c
static void put_hex64(uint64_t value)
{
	put_hex32((uint32_t)(value >> 32), 8);
	put_hex32((uint32_t)value, 8);
}

static int wait_chunk(void)
{
	uint32_t status;

	do {
		status = chunk_dma_status_read();
	} while (!(status & (STATUS_DONE | STATUS_ERROR)));

	if (status & STATUS_ERROR) {
		uart_puts("error: chunk dma 0x");
		put_hex32(chunk_dma_error_read(), 8);
		uart_puts("\r\n");
		return -1;
	}
	return 0;
}

static int start_chunk(uint64_t addr, uint32_t len, uint32_t buffer)
{
	if (len == 0 || len > 4096 || (len & 15u))
		return -1;
	chunk_dma_buffer_write(buffer);
	chunk_dma_addr_lo_write((uint32_t)addr);
	chunk_dma_addr_hi_write((uint32_t)(addr >> 32));
	chunk_dma_length_write(len / 4);
	chunk_dma_start_write(1);
	return 0;
}

static int scan_buffer(uint32_t buffer, uint64_t addr, uint32_t len,
		       const uint32_t pattern[4], uint64_t *match)
{
	uint32_t off;

	chunk_dma_rd_buffer_write(buffer);
	for (off = 0; off < len; off += 16) {
		uint64_t pair0;
		uint64_t pair1;

		chunk_dma_rd_index_write(off / 8);
		asm volatile("nop; nop; nop; nop;" ::: "memory");
		pair0 = chunk_dma_rd_data_read();
		if ((uint32_t)pair0 != pattern[0])
			continue;

		chunk_dma_rd_index_write(off / 8 + 1);
		asm volatile("nop; nop; nop; nop;" ::: "memory");
		pair1 = chunk_dma_rd_data_read();
		if ((uint32_t)(pair0 >> 32) != pattern[1] ||
		    (uint32_t)pair1 != pattern[2] ||
		    (uint32_t)(pair1 >> 32) != pattern[3])
			continue;

		*match = addr + off;
		return 1;
	}
	return 0;
}

static void scan_memory(uint64_t addr, uint64_t len, const uint32_t pattern[4])
{
	uint64_t scanned = 0;
	uint64_t next_progress = 256ull * 1024 * 1024;
	uint64_t match;
	uint32_t current_len;
	uint32_t next_len;
	uint32_t current_buf = 0;
	uint32_t next_buf;
	uint32_t wait_count = 0;
	int have_next;

	if (!(chunk_dma_status_read() & STATUS_LINK_READY))
		uart_puts("waiting for PCIe link...\r\n");
	while (!(chunk_dma_status_read() & STATUS_LINK_READY)) {
		if ((wait_count++ & 0xfffff) == 0)
			print_status();
	}

	current_len = (len > 4096) ? 4096 : (uint32_t)len;
	if (start_chunk(addr, current_len, current_buf) != 0) {
		uart_puts("error: bad chunk\r\n");
		return;
	}

	for (;;) {
		uint64_t current_addr;

		if (wait_chunk() != 0)
			return;

		current_addr = addr + scanned;
		scanned += current_len;
		have_next = scanned < len;
		if (have_next) {
			next_len = ((len - scanned) > 4096) ? 4096
				: (uint32_t)(len - scanned);
			next_buf = current_buf ^ 1;
			if (start_chunk(addr + scanned, next_len, next_buf) != 0) {
				uart_puts("error: bad chunk\r\n");
				return;
			}
		}

		if (scan_buffer(current_buf, current_addr, current_len,
				pattern, &match)) {
			if (have_next)
				(void)wait_chunk();
			uart_puts("match 0x");
			put_hex64(match);
			uart_puts("\r\n");
			return;
		}

		if (!have_next)
			break;

		if (scanned >= next_progress) {
			uart_puts("scan ");
			put_hex64(scanned);
			uart_puts("\r\n");
			next_progress += 256ull * 1024 * 1024;
		}

		current_buf = next_buf;
		current_len = next_len;
	}

	uart_puts("no match\r\n");
}
```

- [ ] **Step 3: Add the scan command**

Change the help text:

```c
	uart_puts("  scan <addr64> <len64> <w0> <w1> <w2> <w3>\r\n");
	uart_puts("                      find a 16-byte-aligned 128-bit pattern\r\n");
```

Extend the tokenizer call to allow eight arguments:

```c
		char *argv[8];
		...
		argc = tokenize(line_buffer, argv, 8);
```

Replace the removed `dump` branch with this `scan` branch. The trailing
`status` line in the snippet is the existing branch, not a second copy:

```c
		} else if (str_eq(argv[0], "scan")) {
			uint32_t addr_hi;
			uint32_t addr_lo;
			uint32_t len_hi;
			uint32_t len_lo;
			uint64_t addr;
			uint64_t len;
			uint32_t pattern[4];
			int bad = 0;
			int i;

			if (argc != 7) {
				uart_puts("error: scan needs addr, len, and four pattern words\r\n");
				continue;
			}
			if (parse_hex64(argv[1], &addr_hi, &addr_lo) != 0 ||
			    parse_hex64(argv[2], &len_hi, &len_lo) != 0) {
				uart_puts("error: invalid address or length\r\n");
				continue;
			}
			addr = ((uint64_t)addr_hi << 32) | addr_lo;
			len = ((uint64_t)len_hi << 32) | len_lo;
			for (i = 0; i < 4; i++) {
				uint32_t hi;
				uint32_t lo;

				if (parse_hex64(argv[3 + i], &hi, &lo) != 0 || hi != 0) {
					bad = 1;
					break;
				}
				pattern[i] = lo;
			}
			if (bad) {
				uart_puts("error: invalid pattern word\r\n");
				continue;
			}
			if (addr & 15ull) {
				uart_puts("error: address must be 16-byte aligned\r\n");
				continue;
			}
			if (len == 0 || (len & 15ull)) {
				uart_puts("error: length must be nonzero and 16-byte aligned\r\n");
				continue;
			}
			if (addr + len < addr) {
				uart_puts("error: address range overflow\r\n");
				continue;
			}
			scan_memory(addr, len, pattern);
		} else if (str_eq(argv[0], "status")) {
```

- [ ] **Step 4: Build the BIOS and verify generated CSRs**

Run:

```bash
venv/bin/python soc_search.py
```

Expected: exits 0; `build/soc_search/software/bios/bios.bin` exists;
`grep chunk_dma build/soc_search/software/include/generated/csr.h` shows
`chunk_dma_addr_lo`, `chunk_dma_rd_data`, etc.

- [ ] **Step 5: Run the static tests**

Run:

```bash
venv/bin/python test/test_chunk_dma.py
venv/bin/python test/test_tlp_dma.py
venv/bin/python soc_squirrel.py
venv/bin/python soc_pcileech.py
venv/bin/python soc_search.py
python3 -m py_compile soc_search.py gateware/chunk_dma.py
```

Expected: all exit 0.

- [ ] **Step 6: Commit**

```bash
git add software/soc_search_console soc_search.py
git commit -m "add search console and scan command"
```

## Task 6: Build and resource check

**Files:**
- Read: `build/soc_search/gateware/*.rpt`
- Read: `build/soc_search/software/bios/bios.bin`

- [ ] **Step 1: Clean build**

```bash
rm -rf build/soc_search
venv/bin/python soc_search.py --build
```

Expected: build succeeds; `build/soc_search/gateware/soc_search.bin` exists.

- [ ] **Step 2: Verify BIOS size and symbols**

```bash
ls -l build/soc_search/software/bios/bios.bin
riscv64-unknown-elf-nm -u build/soc_search/software/bios/bios.elf
```

Expected: BIOS under 32 KiB; no undefined symbols.

- [ ] **Step 3: Capture resources and timing**

```bash
grep -E "Slice LUTs|Slice Registers|Block RAM Tile|RAMB36|RAMB18|DSPs|Bonded IOB|BUFGCTRL" \
    build/soc_search/gateware/soc_search_utilization_place.rpt
grep -E "WNS|WHS|WPWS|Timing constraints" \
    build/soc_search/gateware/soc_search_timing_summary_routed.rpt
```

Expected: BRAM ≤50; setup/hold timing positive at 100 MHz apart from any
known PCIe hard-IP pulse-width caveat; record exact numbers for Task 8.

- [ ] **Step 4: Commit build metadata only if the repo tracks it**

Do not commit `build/` or the bitstream. If the build reveals a required
source fix, commit that fix separately.

## Task 7: Hardware VESA and throughput verification

**Files:**
- Save: `bitstreams/soc_search-console.bin`
- Read: `bitstreams/pcie_squirrel-cleanup-rx128.bin`

- [ ] **Step 1: Save the bitstream**

```bash
mkdir -p bitstreams
cp build/soc_search/gateware/soc_search.bin bitstreams/soc_search-console.bin
sha256sum bitstreams/soc_search-console.bin
```

- [ ] **Step 2: Flash and rescan**

```bash
./program.sh bitstreams/soc_search-console.bin
./rescan.sh
lspci -d 10ee:0666
```

If sudo needs a terminal, stop and ask the user to run `./rescan.sh`.

- [ ] **Step 3: Verify VESA match**

On `/dev/ttyUSB1`, 115200:

```text
scan 0x3f000 0x1000 0x41534556 0xf1000300 0x00013000 0xf0220000
```

Expected: `match 0x000000000003f000`.

- [ ] **Step 4: Verify no-match and post-match behavior**

```text
scan 0x3f010 0x1000 0x41534556 0xf1000300 0x00013000 0xf0220000
```

Expected: `no match` (or a later real match if the pattern truly repeats;
inspect the returned address).

- [ ] **Step 5: Measure a long scan**

Use a pattern unlikely to occur and a long range, for example:

```text
scan 0x40000 0x17ffc0000 0xdeadbeef 0x01234567 0x89abcdef 0xfedcba98
```

Expected: progress lines every 256 MB, then `no match`; no per-candidate
UART output; host remains alive. Record elapsed time and calculate MB/s.

- [ ] **Step 6: Restore the PCILeech fallback**

```bash
./program.sh bitstreams/pcie_squirrel-cleanup-rx128.bin
./rescan.sh
```

Ask the user to rescan; verify a live VESA dump matches
`5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db`.

## Task 8: Documentation and commit

**Files:**
- Create: `SOC_SEARCH.md`
- Modify: `RESOURCES.md`
- Modify: `docs/superpowers/plans/2026-10-29-soc-search.md` only to tick boxes

- [ ] **Step 1: Write `SOC_SEARCH.md`**

Include:

- overview and architecture;
- 64-bit-only MRd64 and 4 KiB chunk DMA;
- two 4 KiB buffers and ping-pong scan;
- `scan` syntax and VESA example;
- CPU first-word filter and 16-byte alignment;
- progress/no-UART-per-candidate behavior;
- resources/timing/bitstream hash;
- hardware VESA, no-match, post-match, and long-scan results;
- accepted limits: exact pattern only, one outstanding MRd, 16-byte
  alignment, no PCILeech/USB mux.

- [ ] **Step 2: Update `RESOURCES.md`**

Add a `Search SoC variant (2026-10-29)` table with exact resource, timing,
bitstream hash, and VESA/throughput results.

- [ ] **Step 3: Run final checks**

```bash
venv/bin/python test/test_chunk_dma.py
venv/bin/python test/test_tlp_dma.py
venv/bin/python soc_squirrel.py
venv/bin/python soc_pcileech.py
venv/bin/python soc_search.py
python3 -m py_compile soc_search.py gateware/chunk_dma.py
git diff --check
```

- [ ] **Step 4: Commit**

```bash
git add soc_search.py gateware/chunk_dma.py \
    software/soc_search_console test/test_chunk_dma.py \
    SOC_SEARCH.md RESOURCES.md \
    docs/superpowers/plans/2026-10-29-soc-search.md
git commit -m "add raw-PCIe search SoC variant"
```

Do not push.

## Self-Review

**Spec coverage:**

- 64-bit-only MRd64: Task 1 request test.
- 4 KiB chunk/double buffer: Tasks 2–3.
- 16-byte-aligned exact 128-bit match: Task 5 scan buffer.
- Command form and VESA example: Task 5 and Task 7.
- Stop at first match: Task 5.
- Progress and no per-candidate UART: Task 5.
- DMA errors/timeouts/link loss: Task 3.
- Raw path/no PCILeech: Task 4.
- Build/resources: Task 6.
- Hardware verification/fallback: Task 7.
- Docs: Task 8.

**Known implementation risks:**

- The `CPL_D` two-DWORD-per-beat packing is the highest-risk logic; the
  `tb_single` and `tb_split` data checks catch ordering/pending errors.
- The 4 KiB MRd may be split by the root complex; `tb_split` covers this.
- The 64-bit `CSRStatus` read must generate a `uint64_t` C accessor; verify
  `csr.h` in Task 5.
- The final 4 KiB chunk may be shorter; `tb_partial` covers a 64-byte chunk,
  and Task 5 caps each command chunk at the remaining length.
