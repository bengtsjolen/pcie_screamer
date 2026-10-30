# Search SoC Throughput Measurement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose the search DMA buffers to the CPU and measure whether the raw PCIe read path sustains at least 149 MiB/s.

**Architecture:** `ChunkDMA` gains a read-only 32-bit Wishbone window over its two 4 KiB buffers. The console scans that window directly and accepts an optional stride so a 1 GiB scan can be run with negligible CPU work.

**Tech Stack:** Migen/LiteX, LitePCIe, VexRiscv, bare-metal C, Python/Migen simulation, hardware serial console.

---

## Preconditions

The working tree currently contains the verified MRd64 high/low address-order fix. Commit it before starting this plan:

```bash
git add gateware/chunk_dma.py gateware/tlp_dma.py test/test_chunk_dma.py test/test_tlp_dma.py
git commit -m "fix: use high/low address order for MRd64"
```

The search SoC is currently flashed and the console is on `/dev/ttyUSB2`. The host must be rescanned after every flash before running console commands.

## File Structure

- `gateware/chunk_dma.py`: add the CPU-visible Wishbone window and preserve the existing CSR peek path.
- `test/test_chunk_dma.py`: verify completion data through the CPU window.
- `soc_search.py`: map the window at `0x20000000`, size `0x2000`, read-only.
- `software/soc_search_console/main.c`: scan the CPU window directly and add the optional stride.
- `docs/superpowers/specs/2026-10-30-soc-search-throughput-design.md`: approved design.

## Task 1: Add CPU-visible DMA buffer window

**Files:**
- Modify: `gateware/chunk_dma.py`
- Test: `test/test_chunk_dma.py`

- [ ] **Step 1: Add the failing CPU-window test**

Add these helpers after `expect_no_tx()` in `test/test_chunk_dma.py`:

```python
def cpu_read(dut, adr):
    yield dut.bus.sel.eq(0xf)
    yield dut.bus.adr.eq(adr)
    yield dut.bus.cyc.eq(1)
    yield dut.bus.stb.eq(1)
    for _ in range(20):
        yield
        if (yield dut.bus.ack):
            break
    else:
        raise AssertionError("CPU buffer read did not ack")
    data = yield dut.bus.dat_r
    yield dut.bus.cyc.eq(0)
    yield dut.bus.stb.eq(0)
    yield
    return data


def tb_cpu_buffer(dut):
    data = [0x60000000 + i for i in range(16)]

    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(16)
    yield from dut.buffer.write(0)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 16, 0)
    yield from cpl_chunk(dut, data, 0x3f000, 0)

    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)

    for i in range(16):
        value = yield from cpu_read(dut, i)
        assert value == data[i], (i, hex(value), hex(data[i]))
```

Add this call near the other simulations in `main()`:

```python
    dut = ChunkDMA()
    run_simulation(dut, tb_cpu_buffer(dut))
```

- [ ] **Step 2: Run the test and verify it fails**

Run:

```bash
venv/bin/python test/test_chunk_dma.py
```

Expected: `AttributeError: 'ChunkDMA' object has no attribute 'bus'`.

- [ ] **Step 3: Add the CPU-visible Wishbone window**

In `gateware/chunk_dma.py`, add the import:

```python
from litex.soc.interconnect import wishbone
```

Add the bus after the `self.debug` declaration:

```python
        self.bus = bus = wishbone.Interface(
            data_width=32, address_width=13, addressing="word")
```

In the existing `self.comb +=` block, replace the two `rp` address assignments:

```python
            rp0.adr.eq(self.rd_index.storage),
            rp1.adr.eq(self.rd_index.storage),
```

with:

```python
            rp0.adr.eq(Mux(bus.cyc & bus.stb & ~bus.adr[10],
                           bus.adr[1:10], self.rd_index.storage)),
            rp1.adr.eq(Mux(bus.cyc & bus.stb & bus.adr[10],
                           bus.adr[1:10], self.rd_index.storage)),
            bus.dat_r.eq(Mux(bus.adr[10],
                             Mux(bus.adr[0], rp1.dat_r[32:64],
                                 rp1.dat_r[0:32]),
                             Mux(bus.adr[0], rp0.dat_r[32:64],
                                 rp0.dat_r[0:32]))),
            bus.err.eq(0),
```

Add the one-cycle acknowledge after the `self.comb +=` block:

```python
        self.sync += [
            bus.ack.eq(0),
            If(bus.cyc & bus.stb & ~bus.ack,
                bus.ack.eq(1),
            ),
        ]
```

- [ ] **Step 4: Run the test and verify it passes**

Run:

```bash
venv/bin/python test/test_chunk_dma.py
venv/bin/python test/test_tlp_dma.py
```

Expected: `ChunkDMA simulation OK` and `TLPDMA simulation OK`.

- [ ] **Step 5: Commit**

```bash
git add gateware/chunk_dma.py test/test_chunk_dma.py
git commit -m "feat: expose chunk DMA buffers to the CPU"
```

## Task 2: Map the CPU window into the SoC

**Files:**
- Modify: `soc_search.py`

- [ ] **Step 1: Add the read-only region**

Add the import near the other LiteX imports:

```python
from litex.soc.integration.soc import SoCRegion
```

Add this after `self.add_csr("chunk_dma")`:

```python
        self.bus.add_slave(
            name="dma_buf",
            slave=chunk_dma.bus,
            region=SoCRegion(origin=0x20000000, size=0x2000, mode="r"),
        )
```

- [ ] **Step 2: Build and verify the address map**

Run:

```bash
source env.sh
venv/bin/python soc_search.py --build
grep -n "DMA_BUF" build/soc_search/software/include/generated/mem.h
```

Expected: build succeeds and `mem.h` contains:

```text
#define DMA_BUF_BASE 0x20000000L
#define DMA_BUF_SIZE 0x00002000
```

- [ ] **Step 3: Check timing and resources**

Run:

```bash
grep -E "Slice LUTs|Slice Registers|Block RAM Tile" build/soc_search/gateware/soc_search_utilization_place.rpt
grep -E "WNS|WHS|WPWS|All user specified timing constraints are met" build/soc_search/gateware/soc_search_timing.rpt
```

Expected: the design builds with no new critical warnings; record the LUT, FF, BRAM, WNS, WHS, and WPWS values.

- [ ] **Step 4: Commit**

```bash
git add soc_search.py
git commit -m "feat: map DMA buffers into the SoC address space"
```

## Task 3: Scan the CPU window directly with an optional stride

**Files:**
- Modify: `software/soc_search_console/main.c`

- [ ] **Step 1: Include the generated memory map**

Change:

```c
#include <generated/csr.h>
```

to:

```c
#include <generated/csr.h>
#include <generated/mem.h>
```

- [ ] **Step 2: Replace `scan_buffer()`**

Replace the entire existing `scan_buffer()` with:

```c
static int scan_buffer(uint32_t buffer, uint64_t addr, uint32_t len,
		       const uint32_t pattern[4], uint32_t stride,
		       uint64_t *match)
{
	const volatile uint32_t *buf =
		(const volatile uint32_t *)(DMA_BUF_BASE +
					    (buffer ? 0x1000u : 0u));
	uint32_t off;

	for (off = 0; off + 16u <= len; off += stride) {
		uint32_t w0 = bswap32(buf[off / 4u + 0u]);
		uint32_t w1 = bswap32(buf[off / 4u + 1u]);
		uint32_t w2;
		uint32_t w3;

		if (w0 != pattern[0] || w1 != pattern[1])
			continue;
		w2 = bswap32(buf[off / 4u + 2u]);
		w3 = bswap32(buf[off / 4u + 3u]);
		if (w2 != pattern[2] || w3 != pattern[3])
			continue;

		*match = addr + off;
		return 1;
	}
	return 0;
}
```

- [ ] **Step 3: Pass the stride through `scan_memory()`**

Change the signature:

```c
static void scan_memory(uint64_t addr, uint64_t len, const uint32_t pattern[4])
```

to:

```c
static void scan_memory(uint64_t addr, uint64_t len, const uint32_t pattern[4],
			uint32_t stride)
```

Change the `scan_buffer()` call:

```c
		if (scan_buffer(current_buf, current_addr, current_len,
				pattern, &match)) {
```

to:

```c
		if (scan_buffer(current_buf, current_addr, current_len,
				pattern, stride, &match)) {
```

- [ ] **Step 4: Add optional stride parsing**

In the `scan` command branch, replace:

```c
			uint32_t pattern[4];
			int bad = 0;
			int i;

			if (argc != 7) {
				uart_puts("error: scan needs addr, len, and four pattern words\r\n");
				continue;
			}
```

with:

```c
			uint32_t pattern[4];
			uint32_t stride = 16;
			int bad = 0;
			int i;

			if (argc != 7 && argc != 8) {
				uart_puts("error: scan needs addr, len, four pattern words, and optional stride\r\n");
				continue;
			}
```

After the overflow check, before `scan_memory()`:

```c
			if (addr + len < addr) {
				uart_puts("error: address range overflow\r\n");
				continue;
			}
```

insert:

```c
			if (argc == 8) {
				uint32_t stride_hi;
				uint32_t stride_lo;

				if (parse_hex64(argv[7], &stride_hi,
						&stride_lo) != 0 ||
				    stride_hi != 0 || stride_lo == 0 ||
				    (stride_lo & 15u) || stride_lo > 4096u ||
				    stride_lo > len) {
					uart_puts("error: invalid stride\r\n");
					continue;
				}
				stride = stride_lo;
			}
```

Change the final call:

```c
			scan_memory(addr, len, pattern);
```

to:

```c
			scan_memory(addr, len, pattern, stride);
```

- [ ] **Step 5: Update help text**

Change:

```c
	uart_puts("  scan <addr64> <len64> <w0> <w1> <w2> <w3>\r\n");
```

to:

```c
	uart_puts("  scan <addr64> <len64> <w0> <w1> <w2> <w3> [stride]\r\n");
	uart_puts("                      stride defaults to 16 and is a 16-byte multiple\r\n");
```

- [ ] **Step 6: Rebuild**

Run:

```bash
source env.sh
venv/bin/python soc_search.py --build
```

Expected: build succeeds and `build/soc_search/gateware/soc_search.bin` exists.

- [ ] **Step 7: Commit**

```bash
git add software/soc_search_console/main.c
git commit -m "feat: scan DMA buffers directly with optional stride"
```

## Task 4: Hardware verification and throughput measurement

**Files:**
- No source changes.
- Artifact: `build/soc_search/gateware/soc_search.bin`

- [ ] **Step 1: Flash and rescan**

Run:

```bash
./program.sh build/soc_search/gateware/soc_search.bin
```

Then, in a terminal with sudo available:

```bash
./rescan.sh
```

Expected: `lspci -d 10ee:0666` shows the FPGA and the console is on `/dev/ttyUSB2`.

- [ ] **Step 2: Verify the low VESA pattern**

Run:

```bash
python3 - <<'PY'
import serial, time
s = serial.Serial('/dev/ttyUSB2', 115200, timeout=0.2)
s.reset_input_buffer()
s.write(b'scan 0x3f000 0x1000 0x41534556 0xf1000300 0x00013000 0xf0220000 0x10\r')
end = time.time() + 10
out = b''
while time.time() < end:
    out += s.read(4096)
    if out.endswith(b'soc> '):
        break
print(out.decode('latin1'))
s.close()
PY
```

Expected: `match 0x000000000003f000`.

- [ ] **Step 3: Verify the high-address pattern**

Run:

```bash
python3 - <<'PY'
import serial, time
s = serial.Serial('/dev/ttyUSB2', 115200, timeout=0.2)
s.reset_input_buffer()
s.write(b'scan 0x100000000 0x1000 0x00323067 0x00000001 0xcb77e067 0x0000000b 0x10\r')
end = time.time() + 10
out = b''
while time.time() < end:
    out += s.read(4096)
    if out.endswith(b'soc> '):
        break
print(out.decode('latin1'))
s.close()
PY
```

Expected: `match 0x0000000100000000`.

If the high host memory changed, run a high scan with any pattern, then `peek` and derive the first four words using the same byte order as the low VESA example.

- [ ] **Step 4: Measure 1 GiB at stride 16 and stride 1024**

Run:

```bash
python3 - <<'PY'
import serial, time
s = serial.Serial('/dev/ttyUSB2', 115200, timeout=0.2)
for stride in ('0x10', '0x400'):
    cmd = ('scan 0x100000000 0x40000000 '
           '0xdeadbeef 0xfeedface 0xcafebabe 0x12345678 '
           f'{stride}\r')
    s.reset_input_buffer()
    t = time.time()
    s.write(cmd.encode())
    out = b''
    while time.time() - t < 300:
        out += s.read(4096)
        if out.endswith(b'soc> '):
            break
    elapsed = time.time() - t
    print(f'stride={stride} elapsed={elapsed:.1f}s rate={1024/elapsed:.1f}MiB/s')
    print(out.decode('latin1'))
s.close()
PY
```

Expected: both scans print `no match`; stride 1024 must be at least 149 MiB/s. Record both rates. If stride 16 is materially lower, report that the CPU scanner is the limit; do not change the CPU scanner in this plan.

- [ ] **Step 5: Apply the decision gate**

If stride 1024 is below 149 MiB/s, stop. Record the measured rate and report that two buffers/one outstanding request are insufficient. Do not run the 6 GiB scan and do not tune the CPU scanner yet.

If stride 1024 passes, continue to Step 6.

- [ ] **Step 6: Run the 6 GiB scan**

Run:

```bash
python3 - <<'PY'
import serial, time
s = serial.Serial('/dev/ttyUSB2', 115200, timeout=0.2)
cmd = ('scan 0x100000000 0x180000000 '
       '0xdeadbeef 0xfeedface 0xcafebabe 0x12345678 0x10\r')
s.reset_input_buffer()
t = time.time()
s.write(cmd.encode())
out = b''
while time.time() - t < 1800:
    out += s.read(4096)
    if out.endswith(b'soc> '):
        break
print(out.decode('latin1'))
print(f'elapsed={time.time()-t:.1f}s')
s.close()
PY
```

Expected: progress lines every 256 MiB, then `no match`; no DMA errors.

- [ ] **Step 7: Commit the verified state**

```bash
git add gateware/chunk_dma.py soc_search.py software/soc_search_console/main.c test/test_chunk_dma.py
git commit -m "perf: measure search DMA throughput with CPU-visible buffers"
```

## Self-Review

- Spec coverage: CPU-visible buffers (Tasks 1–2), direct scanner and optional stride (Task 3), 1 GiB and 6 GiB hardware measurements (Task 4).
- No hardware matcher, masking, or multi-outstanding work is included.
- The two-buffer decision gate is explicit: stop if stride 1024 is below 149 MiB/s.
- All changed files and commands are named.
- No placeholder code remains in the implementation tasks.
