# PCIe Squirrel VexRiscv Console SoC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a separate `soc_squirrel.py` build with a minimal VexRiscv, custom serial BIOS, PCIe endpoint, and page-burst TLP DMA that dumps host memory over serial.

**Architecture:** Keep the existing PCILeech top level. Move the 32→64-bit TLP packer to `gateware/tlp32.py`, add a CSR-controlled `TLPDMA` in `gateware/tlp_dma.py`, and add a `SoCCore` top level using a custom no-libc BIOS package. The CPU starts a 64-bit MRd page burst and reads the completed 4 KiB buffer through CSRs.

**Tech Stack:** Python 3, Migen, LiteX, LitePCIe, VexRiscv `minimal`, RISC-V GCC, Vivado 2021.2, PCILeech hardware, FT4232H serial.

---

## Task 1: Move the TLP packer into gateware

**Files:**
- Create: `gateware/tlp32.py`
- Modify: `pcileech_squirrel.py`

- [ ] **Step 1: Create `gateware/tlp32.py`**

Move the complete `TLP32To64Packer` class from `pcileech_squirrel.py` into
`gateware/tlp32.py` unchanged. Add this header:

```python
# 32-bit TLP stream to 64-bit PCIe AXI-Stream packer.

from migen import *

from litex.soc.interconnect import stream

from litepcie.common import phy_layout


```

The class body is exactly the current `TLP32To64Packer` implementation,
including `swap_words`, `first_dat`, `first_be`, the `IDLE`/`HAVE_FIRST` FSM,
and the `source.ready` / `sink.ready` handshake fixes.

- [ ] **Step 2: Update the PCILeech top level**

Remove the `TLP32To64Packer` class from `pcileech_squirrel.py` and add:

```python
from gateware.tlp32 import TLP32To64Packer
```

Keep the existing `self.submodules.tlp_tx_conv = TLP32To64Packer(swap_words=False)`
and its connections unchanged.

- [ ] **Step 3: Verify the existing top level still elaborates**

Run:

```bash
venv/bin/python pcileech_squirrel.py
```

Expected: exit 0, no traceback.

- [ ] **Step 4: Check imports and syntax**

Run:

```bash
python3 -m py_compile gateware/tlp32.py pcileech_squirrel.py
git diff --check
```

Expected: exit 0 and no diff-check output.

---

## Task 2: Write the failing TLPDMA simulation test

**Files:**
- Create: `test/test_tlp_dma.py`

- [ ] **Step 1: Create the simulation test**

The test must import the not-yet-written `TLPDMA`, so it fails first with
`ModuleNotFoundError`. Create `test/test_tlp_dma.py` with:

```python
#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from migen import *
from migen.sim import run_simulation

from gateware.tlp_dma import TLPDMA


STATUS_LINK_READY = 1 << 0
STATUS_BUSY       = 1 << 1
STATUS_DONE       = 1 << 2
STATUS_ERROR      = 1 << 3


def beat(dut, dw0, dw1, be=0xFF, last=0):
    yield dut.tlp_rx.dat.eq((dw1 << 32) | dw0)
    yield dut.tlp_rx.be.eq(be)
    yield dut.tlp_rx.last.eq(last)
    yield dut.tlp_rx.valid.eq(1)
    yield


def cpl(dut, data, status=0):
    yield from beat(dut, 0, status << 13, last=0)
    yield from beat(dut, 0, data, last=1)
    yield dut.tlp_rx.valid.eq(0)
    yield


def wait_for_status(dut, mask, timeout=200):
    for _ in range(timeout):
        status = yield from dut.status.read()
        if status & mask:
            return status
        yield
    raise AssertionError(f"status mask 0x{mask:x} not reached")


def tb(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(0x1600)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)

    yield from dut.addr_lo.write(0x3F000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(2)
    yield from dut.start.write(1)

    # First request: 4 DW header -> 2 x 64-bit beats.
    for _ in range(2):
        while not (yield dut.tlp_tx.valid):
            yield
        yield
    yield from cpl(dut, 0x11111111)

    # Second request.
    for _ in range(2):
        while not (yield dut.tlp_tx.valid):
            yield
        yield
    yield from cpl(dut, 0x22222222)

    status = yield from wait_for_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE, f"unexpected status 0x{status:x}"
    assert not (status & STATUS_ERROR)

    yield from dut.rd_index.write(0)
    assert (yield from dut.rd_data.read()) == 0x11111111
    yield from dut.rd_index.write(1)
    assert (yield from dut.rd_data.read()) == 0x22222222


def tb_timeout(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(0x1600)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3F000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    yield from wait_for_status(dut, STATUS_ERROR, timeout=200)
    assert (yield from dut.error.read()) == 4


def tb_not_ready(dut):
    yield dut.link_ready.eq(0)
    yield dut.requester_id.eq(0x1600)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    yield from wait_for_status(dut, STATUS_ERROR, timeout=20)
    assert (yield from dut.error.read()) == 1


def main():
    dut = TLPDMA()
    run_simulation(dut, tb(dut))

    dut = TLPDMA(timeout_cycles=20)
    run_simulation(dut, tb_timeout(dut))

    dut = TLPDMA()
    run_simulation(dut, tb_not_ready(dut))

    print("TLPDMA simulation OK")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the test and verify it fails**

Run:

```bash
venv/bin/python test/test_tlp_dma.py
```

Expected: `ModuleNotFoundError: No module named 'gateware.tlp_dma'`.

---

## Task 3: Implement `gateware/tlp_dma.py`

**Files:**
- Create: `gateware/tlp_dma.py`

- [ ] **Step 1: Implement the CSR/FSM module**

Implement the module with this interface and behavior:

- Streams: `tlp_tx` (64-bit output to the PCIe PHY), `tlp_rx` (64-bit input
  from the PCIe PHY).
- Inputs: `link_ready`, `requester_id` (16-bit).
- CSRs: `addr_lo`, `addr_hi`, `length` (11 bits), `start`, `status` (4 bits),
  `error` (4 bits), `rd_index` (10 bits), `rd_data` (32 bits).
- Status bits: 0 link-ready, 1 busy, 2 done, 3 error.
- Error codes: 0 none, 1 not ready, 2 completion error/malformed, 3 malformed, 4 timeout, 5 invalid length.
- Internal `TLP32To64Packer` drives `tlp_tx`; drive `packer.sink.be = 0xF`
  for every header DWORD.
- A 1024×32 `Memory` page buffer; write port at `index`, read port at
  `rd_index.storage`; the sync read port plus one read register means `rd_data`
  is stable after two cycles.
- Sequential MRd64 requests, one DWORD per request:
  - DW0 `0x20000001`
  - DW1 `(requester_id << 16) | 0xF` (Requester ID at bits 31:16, tag 0,
    last BE 0, first BE 0xF)
  - DW2 `Cat(Constant(0, 2), addr[2:32])`
  - DW3 `addr[32:64]`
  - `last` on DW3.
- Completion: first beat checks the CplD fmt/type `dat[24:31] == 0x4A` and
  latches status `dat[45:48]`; second beat checks `last`, `status == 0`, and
  `be[4:8] == 0xF`; data is `dat[32:64]`.
- On success, write the buffer and increment `index`/`addr`; when
  `index + 1 == count`, set done and return to IDLE. DONE is sticky until the
  next start.
- On any error, set `error`/`error_code`, clear busy, return to IDLE. ERROR
  and the error code are sticky until the next start; the error code must be
  stable whenever ERROR is set.
- Timeout on no completion after `timeout_cycles` (default 1,000,000).
- `start` is a write strobe (`start.re & start.storage`); ignore while busy.
- `length` 0 or >1024 is an invalid-length error.

Use the exact header:

```python
from migen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

from litepcie.common import phy_layout

from gateware.tlp32 import TLP32To64Packer


class TLPDMA(Module, AutoCSR):
    def __init__(self, max_dwords=1024, timeout_cycles=1_000_000):
```

The FSM states are `IDLE`, `ISSUE`, `WAIT`, `ERROR`.

- [ ] **Step 2: Run the TLPDMA test**

Run:

```bash
venv/bin/python test/test_tlp_dma.py
```

Expected: `TLPDMA simulation OK`.

- [ ] **Step 3: Compile and check**

Run:

```bash
python3 -m py_compile gateware/tlp_dma.py
git diff --check
```

Expected: exit 0 and no output.

---

## Task 4: Add the SoC top level

**Files:**
- Create: `soc_squirrel.py`

- [ ] **Step 1: Implement the top level**

Create `soc_squirrel.py` with:

- `_CRG`: 100 MHz `sys` clock only, using `S7PLL`.
- `SoCSquirrel(SoCCore)`:
  - `cpu_type="vexriscv"`, `cpu_variant="minimal"`;
  - `integrated_rom_size=0x8000`, `integrated_sram_size=0x2000`;
  - `uart_name="serial"`, 115200 baud;
  - `with_timer=False`, `with_ctrl=False`.
- `S7PCIEPHY` with `data_width=64`, `bar0_size=0x40000`,
  Device_ID `0666`, class code `02/00`, extended tags.
- Copy the existing GTP placement command and PCIe/sys false-path platform
  commands from `pcileech_squirrel.py`.
- Do not override `pcie_phy.pcie_rst_n`.
- Auto-BME after link-up using the existing `bme_done`/`bme_timer`/`bme_pulse`
  pattern.
- `TLPDMA` instance and CSRs.
- Connections:

```python
self.comb += [
    self.pcie_phy.source.connect(self.tlp_dma.tlp_rx),
    self.tlp_dma.tlp_tx.connect(self.pcie_phy.sink),
    self.tlp_dma.link_ready.eq(
        self.pcie_phy._link_status.fields.status
        & bme_done
        & (self.pcie_phy.id != 0)
    ),
    self.tlp_dma.requester_id.eq(self.pcie_phy.id),
]
```

- `main()` parses `--build`, `--load`, `--flash`.
- Use `Builder(soc, output_dir="build/soc_squirrel",
  csr_csv="test/soc_csr.csv")`.
- After constructing the builder, replace the default `bios` package with the
  local package:

```python
bios_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "software", "soc_console")
builder.software_packages = [
    ("bios", bios_dir) if name == "bios" else (name, src_dir)
    for name, src_dir in builder.software_packages
]
```

- Set `soc.build_name = "soc_squirrel"` so `--load`/`--flash` use the right
  artifact names.

- [ ] **Step 2: Elaborate the SoC**

Run:

```bash
venv/bin/python soc_squirrel.py
```

Expected: exit 0, no traceback. The custom BIOS package may not build yet, but
elaboration should finish.

- [ ] **Step 3: Compile and check**

Run:

```bash
python3 -m py_compile soc_squirrel.py
git diff --check
```

Expected: exit 0 and no output.

---

## Task 5: Add the custom serial BIOS

**Files:**
- Create: `software/soc_console/main.c`
- Create: `software/soc_console/Makefile`
- Create: `software/soc_console/linker.ld`

- [ ] **Step 1: Copy the LiteX linker script**

Copy the LiteX BIOS linker script into `software/soc_console/linker.ld`:

```bash
cp litex/litex/litex/soc/software/bios/linker.ld software/soc_console/linker.ld
```

- [ ] **Step 2: Add the Makefile**

Create `software/soc_console/Makefile`:

```make
include ../include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

OBJECTS = main.o crt0.o
LSCRIPT = linker.ld

all: bios.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

bios.elf: $(BIOS_DIRECTORY)/$(LSCRIPT) $(OBJECTS)
	$(CC) $(LDFLAGS) -T $(BIOS_DIRECTORY)/$(LSCRIPT) -N -o $@ $(OBJECTS)

VPATH = $(BIOS_DIRECTORY):$(CPU_DIRECTORY)

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) bios.elf bios.bin

.PHONY: all clean
```

- [ ] **Step 3: Add `main.c`**

Implement a no-libc console with:

- polling UART helpers using generated `csr.h`;
- line input with echo/backspace;
- `help`;
- `dump [addr] [len]` with defaults `0x3f000`, `0x1000`;
- 64-bit hex parse and print;
- DMA start/status/error/buffer reads;
- 16 bytes per line output;
- empty `isr`;
- warnings for arbitrary physical reads.

Required CSR access pattern:

```c
tlp_dma_addr_lo_write((uint32_t)addr);
tlp_dma_addr_hi_write((uint32_t)(addr >> 32));
tlp_dma_length_write(len / 4);
tlp_dma_start_write(1);
while (!(tlp_dma_status_read() & (STATUS_DONE | STATUS_ERROR)))
    ;
if (tlp_dma_status_read() & STATUS_ERROR) {
    print_hex32(tlp_dma_error_read());
    return;
}
for (i = 0; i < len; i += 16) {
    for (j = 0; j < 16 && i + j < len; j += 4) {
        tlp_dma_rd_index_write((i + j) / 4);
        d = tlp_dma_rd_data_read();
        print_hex8(d & 0xff); /* etc. */
    }
}
```

Wait for `STATUS_LINK_READY` before starting, print a message if it is not
ready, and never issue a write TLP.

- [ ] **Step 4: Compile the custom BIOS**

Run:

```bash
venv/bin/python soc_squirrel.py
```

Expected: exit 0 and `build/soc_squirrel/software/bios/bios.bin` exists after
the Builder runs. If the Builder is not invoked without `--build`, use:

```bash
venv/bin/python soc_squirrel.py --build
```

Expected: the BIOS compiles and the gateware build starts.

- [ ] **Step 5: Check the firmware size**

Run:

```bash
ls -lh build/soc_squirrel/software/bios/bios.bin
```

Expected: non-empty and below 32 KiB.

---

## Task 6: Static checks and SoC resource build

**Files:**
- Read: `build/soc_squirrel/gateware/*.rpt`

- [ ] **Step 1: Run the TLPDMA test**

```bash
venv/bin/python test/test_tlp_dma.py
```

Expected: `TLPDMA simulation OK`.

- [ ] **Step 2: Elaborate both top levels**

```bash
venv/bin/python pcileech_squirrel.py
venv/bin/python soc_squirrel.py
```

Expected: both exit 0.

- [ ] **Step 3: Build the SoC bitstream**

```bash
source env.sh
rm -rf build/soc_squirrel
venv/bin/python soc_squirrel.py --build
```

Expected: Vivado completes and produces
`build/soc_squirrel/gateware/soc_squirrel.bin`.

- [ ] **Step 4: Inspect resources**

```bash
grep -E "Slice LUTs|Slice Registers|Block RAM Tile|RAMB36|RAMB18|DSPs|Bonded IOB|BUFGCTRL" build/soc_squirrel/gateware/soc_squirrel_utilization_place.rpt
```

Expected: BRAM no more than ~25/50, LUT/FF comfortably within the XC7A35T.
If it does not fit, apply the fallback order from the design spec and rebuild.

---

## Task 7: Flash and verify the SoC

**Files:**
- Create: `bitstreams/soc_squirrel-<date>.bin`
- Read: serial console output

- [ ] **Step 1: Save the bitstream**

```bash
mkdir -p bitstreams
cp build/soc_squirrel/gateware/soc_squirrel.bin bitstreams/soc_squirrel-console.bin
ls -lh bitstreams/soc_squirrel-console.bin
```

- [ ] **Step 2: Flash and rescan**

```bash
./program.sh bitstreams/soc_squirrel-console.bin
./rescan.sh
lspci -d 10ee:0666
```

Expected: flash succeeds and the device enumerates.

- [ ] **Step 3: Find the serial port**

Check the FT4232H channel-B serial device:

```bash
ls -l /dev/serial/by-id 2>/dev/null
ls -l /dev/ttyUSB* 2>/dev/null
```

Use `picocom -b 115200 /dev/ttyUSB<N>` or the corresponding by-id path.

- [ ] **Step 4: Boot and dump VESA**

At the prompt:

```text
dump
```

Capture the hex output, parse the 4096 bytes, and compare the SHA-256 with:

```text
5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db
```

- [ ] **Step 5: Test an explicit address**

At the prompt:

```text
dump 0x1000000 0x40
```

Expected: no DMA error, plausible data, host remains alive, and
`lspci -d 10ee:0666` still shows the device.

- [ ] **Step 6: Restore the PCILeech build**

```bash
./program.sh bitstreams/pcie_squirrel-cleanup-rx128.bin
./rescan.sh
```

Expected: the PCILeech device returns. Optionally verify a short PCILeech dump.

---

## Task 8: Document and commit

**Files:**
- Modify: `RESOURCES.md` (SoC resource row and VESA result)
- Add: source/test/BIOS files

- [ ] **Step 1: Add the SoC resource/verification note**

Record the placed SoC LUT/FF/BRAM and the VESA hash result in `RESOURCES.md`.
Do not change the existing PCILeech benchmark tables except to point to the new
SoC note.

- [ ] **Step 2: Check the full diff**

```bash
git diff --check
git status --short
```

Confirm only intended SoC files, the packer move/import, tests, BIOS sources,
and `RESOURCES.md` are included. Do not include bitstreams, build output,
vendored `litex/`, or unrelated files.

- [ ] **Step 3: Commit**

```bash
git add soc_squirrel.py gateware/tlp32.py gateware/tlp_dma.py \
    pcileech_squirrel.py software/soc_console test/test_tlp_dma.py RESOURCES.md
git commit -m "add VexRiscv serial console and TLP DMA SoC"
```

Do not push.

---

## Rollback

If the SoC does not boot or the VESA hash mismatches:

1. Reflash the PCILeech fallback:

```bash
./program.sh bitstreams/pcie_squirrel-cleanup-rx128.bin
./rescan.sh
```

2. If the changes are uncommitted, restore the touched files:

```bash
git restore soc_squirrel.py gateware/tlp32.py gateware/tlp_dma.py \
    pcileech_squirrel.py software/soc_console test/test_tlp_dma.py RESOURCES.md
```

3. If committed, revert the SoC commit by hash.

Do not amend or revert the design commit.

---

## Implementation Notes / Deviations (2026-10-28)

- The final `test/test_tlp_dma.py` is authoritative; it includes per-request
  8-bit tags, page-aligned stale-completion rejection, Cpl/CplD error matching,
  unaligned address coverage, link-drop tag tests, and Verilog conversion.
- TLPDMA uses MRd32 below 4 GiB and MRd64 above; the VESA path uses MRd32.
- Completion matching checks requester ID, tag, lower address, CplD type,
  status, length, `last`, and data BE. Stale completions are dropped.
- `uart_rx_fifo_rx_we=True` is required: the polling console does not clear
  the LiteX UART RX event, so without it the RX FIFO never pops.
- The console prints DMA bytes MSB-first so the serial output matches host
  memory byte order.
- The SoC exposes `main_pcie_id` and a `status` command for link/BME/ID
  diagnostics. `link_ready` uses `_bus_master_enable.status`.
- The actual placed resources are 2,050 LUT / 2,470 FF / 16 BRAM; the ROM
  auto-resizes to the 2,728-byte BIOS.
- Hardware verification: VESA SHA-256
  `5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db`,
  arbitrary `0x1000000` 64-byte read with no DMA error, host alive.
- The PCILeech RX128 fallback was reflashed after testing.
