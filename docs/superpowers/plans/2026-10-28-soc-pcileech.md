# PCILeechFIFO SoC Variant Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a standalone VexRiscv SoC variant that feeds and reads the existing `PCILeechFIFO` and mux directly from the CPU, with no FT601 and no host CMD/CFG init.

**Architecture:** Copy the verified VexRiscv/SoC/UART/PCIe/BME structure into `soc_pcileech.py`. Replace TLPDMA with `PCILeechFIFO` large plus a small CPU-facing `PCILeechUsbBridge`. The BIOS sends PCILeech TLP data/flag words and parses 8-word mux frames for CplD data.

**Tech Stack:** Python 3, Migen, LiteX, LitePCIe, VexRiscv `minimal`, RISC-V GCC, Vivado 2021.2, FT4232H serial.

---

## Task 1: Write the failing PCILeech bridge test

**Files:**
- Create: `test/test_pcileech_bridge.py`

- [ ] **Step 1: Create the test**

```python
#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from migen import *
from migen.sim import run_simulation

from gateware.pcileech_bridge import PCILeechUsbBridge


def wait_for(dut, signal, value, timeout=100):
    for _ in range(timeout):
        if (yield signal) == value:
            return
        yield
    raise AssertionError(f"signal did not become {value}")


def tb(dut):
    yield dut.link_ready.eq(1)
    yield dut.fifo_tx.ready.eq(0)
    yield dut.fifo_rx.valid.eq(0)
    yield

    assert not (yield dut.fifo_tx.valid)
    assert (yield dut.tx_full.status) == 0
    assert (yield dut.rx_empty.status) == 1
    status = yield from dut.status.read()
    assert status == 0b0101, hex(status)

    # TX: hold the sink stalled, verify stability, then consume.
    yield from dut.tx_data.write(0x11223344)
    yield from wait_for(dut, dut.fifo_tx.valid, 1)
    assert (yield dut.fifo_tx.data) == 0x44332211
    for _ in range(5):
        assert (yield dut.fifo_tx.valid)
        assert (yield dut.fifo_tx.data) == 0x44332211
        yield
    yield dut.fifo_tx.ready.eq(1)
    yield
    yield
    assert not (yield dut.fifo_tx.valid)

    # Second TX word, ordering/consumption.
    yield from dut.tx_data.write(0xDEADBEEF)
    yield from wait_for(dut, dut.fifo_tx.valid, 1)
    assert (yield dut.fifo_tx.data) == 0xEFBEADDE
    yield
    assert not (yield dut.fifo_tx.valid)

    # RX: hold valid until ready, poison data after the handshake.
    yield dut.fifo_rx.data.eq(0x55667788)
    yield dut.fifo_rx.valid.eq(1)
    yield from wait_for(dut, dut.fifo_rx.ready, 1)
    yield
    yield dut.fifo_rx.valid.eq(0)
    yield dut.fifo_rx.data.eq(0xDEADBEEF)
    for _ in range(20):
        if not (yield dut.rx_empty.status):
            break
        yield
    else:
        raise AssertionError("RX FIFO did not fill")
    assert (yield from dut.rx_data.read()) == 0x88776655
    yield
    assert (yield dut.rx_empty.status) == 1

    # Second RX word, ordering/consumption.
    yield dut.fifo_rx.data.eq(0xAABBCCDD)
    yield dut.fifo_rx.valid.eq(1)
    yield from wait_for(dut, dut.fifo_rx.ready, 1)
    yield
    yield dut.fifo_rx.valid.eq(0)
    for _ in range(20):
        if not (yield dut.rx_empty.status):
            break
        yield
    else:
        raise AssertionError("second RX word not captured")
    assert (yield from dut.rx_data.read()) == 0xDDCCBBAA
    yield
    assert (yield dut.rx_empty.status) == 1


def main():
    dut = PCILeechUsbBridge()
    run_simulation(dut, tb(dut))
    print("PCILeech bridge simulation OK")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run and confirm the import failure**

```bash
venv/bin/python test/test_pcileech_bridge.py
```

Expected: `ModuleNotFoundError: No module named 'gateware.pcileech_bridge'`.

Do not create the module yet; do not commit.

---

## Task 2: Implement the PCILeech USB bridge

**Files:**
- Create: `gateware/pcileech_bridge.py`

- [ ] **Step 1: Implement the bridge**

Implement `PCILeechUsbBridge(Module, AutoCSR)`:

- `fifo_tx` endpoint (32-bit) toward `PCILeechFIFO.usb_rx`.
- `fifo_rx` endpoint (32-bit) from `PCILeechFIFO.usb_tx`.
- `link_ready` input.
- CSRs: `tx_data` (CSRStorage 32), `tx_full` (CSRStatus), `rx_data`
  (CSRStatus 32), `rx_empty` (CSRStatus), `status` (CSRStatus 4).
- TX FIFO 256 × 32; RX FIFO 1024 × 32.
- TX CSR write pushes `tx_data.storage`; `tx_full.status = ~tx_fifo.sink.ready`.
- `fifo_tx.data = byteswap(tx_fifo.source.data)`; `fifo_tx.valid` from FIFO.
- `fifo_rx` → `rx_fifo` with `byteswap(fifo_rx.data)`.
- `rx_data.status = rx_fifo.source.data`; `rx_fifo.source.ready = rx_data.we`
  (the CSRStatus read strobe; LiteX uses `.we` for a status read).
- `status = Cat(link_ready, tx_full, rx_empty, 0)`.

Use the Migen byte swap:

```python
def swap32(value):
    return Cat(value[24:32], value[16:24], value[8:16], value[0:8])
```

- [ ] **Step 2: Run the bridge test**

```bash
venv/bin/python test/test_pcileech_bridge.py
```

Expected: `PCILeech bridge simulation OK`.

- [ ] **Step 3: Compile and diff-check**

```bash
python3 -m py_compile gateware/pcileech_bridge.py
git diff --check
```

Do not commit.

---

## Task 3: Add the PCILeechFIFO SoC top level

**Files:**
- Create: `soc_pcileech.py`

- [ ] **Step 1: Copy the SoC structure**

Copy `soc_squirrel.py` to `soc_pcileech.py`. Keep:

- `_CRG`, `SoCCore` parameters, `uart_rx_fifo_rx_we=True`;
- `check_bios_requirements()` ROM/SRAM override;
- `S7PCIEPHY` config, GTP placement, `add_false_path_constraints_by_name`;
- pcie-domain auto-BME (as in the first SoC) and `link_ready = link_status & bus_master_enable & (id != 0)`;
- `pcie_id` CSR;
- Builder subclass/no-meson logic and custom BIOS packaging;
- `--build/--load/--flash`.

- [ ] **Step 2: Replace TLPDMA with PCILeechFIFO and bridge**

Add imports:

```python
from gateware.pcileech_fifo import PCILeechFIFO
from gateware.pcileech_bridge import PCILeechUsbBridge
from gateware.tlp32 import TLP32To64Packer
```

Instantiate and connect:

```python
self.submodules.pcileech_fifo = pcileech_fifo = PCILeechFIFO(
    mux_out_fifo_depth=2048,
    tlp_rx_fifo_depth=2048,
)
self.submodules.pcileech_bridge = pcileech_bridge = PCILeechUsbBridge()
self.add_csr("pcileech_bridge")
self.submodules.tlp_tx_conv = TLP32To64Packer(swap_words=False)

self.comb += [
    self.pcie_phy.source.connect(pcileech_fifo.tlp_rx),
    pcileech_fifo.tlp_tx.connect(self.tlp_tx_conv.sink),
    self.tlp_tx_conv.source.connect(self.pcie_phy.sink),
    pcileech_bridge.fifo_tx.connect(pcileech_fifo.usb_rx),
    pcileech_fifo.usb_tx.connect(pcileech_bridge.fifo_rx),
    pcileech_bridge.link_ready.eq(link_ready),
    self.pcie_phy.pcie_rst_n.eq(~pcileech_fifo.pcie_rst_core),
    pcileech_fifo.phy_lnk_up.eq(self.pcie_phy._link_status.fields.status),
    pcileech_fifo.phy_ltssm.eq(self.pcie_phy._link_status.fields.ltssm),
    pcileech_fifo.phy_lnk_rate.eq(self.pcie_phy._link_status.fields.rate),
    pcileech_fifo.phy_lnk_width.eq(self.pcie_phy._link_status.fields.width),
    pcileech_fifo.phy_id.eq(self.pcie_phy.id),
    pcileech_fifo.cfg_dcommand.eq(self.pcie_phy.dcommand),
    pcileech_fifo.tx_axis_seen.eq(
        self.pcie_phy.s_axis_tx_tvalid & self.pcie_phy.s_axis_tx_tready),
    pcileech_fifo.rx_axis_seen.eq(self.pcie_phy.m_axis_rx_tvalid),
    pcileech_fifo.tx_err_drop.eq(self.pcie_phy.tx_err_drop),
]
```

- [ ] **Step 3: Change build naming/package**

- Output directory `build/soc_pcileech`;
- CSR `test/soc_pcileech_csr.csv`;
- custom BIOS package `software/soc_pcileech_console`;
- `soc.build_name = "soc_pcileech"`;
- no MSI, no FT601, no TLPDMA.

- [ ] **Step 4: Direct finalize**

```bash
venv/bin/python - <<'PY'
from soc_pcileech import SoCPCILeech
from platforms.pcie_squirrel import Platform
soc = SoCPCILeech(Platform())
soc.finalize()
print("SOC_PCILEECH_FINALIZE_OK")
PY
python3 -m py_compile soc_pcileech.py
git diff --check
```

Do not build/flash/commit.

---

## Task 4: Create the PCILeech BIOS console

**Files:**
- Create: `software/soc_pcileech_console/main.c`
- Create: `software/soc_pcileech_console/Makefile`
- Create: `software/soc_pcileech_console/linker.ld`

- [ ] **Step 1: Copy the console package**

Copy `software/soc_console/Makefile` and `linker.ld` unchanged. Copy
`main.c`, then replace the TLPDMA backend and `status` implementation with the
PCILeech bridge/Mux backend. Keep the same `help`, `readline`, hex parser,
`dump [addr] [len]` UI, link warning, UART polling, and no-libc constraints.

- [ ] **Step 2: Add bridge reads/writes**

Use generated CSRs:

```c
static void bridge_write(uint32_t word) {
    while (pcileech_bridge_tx_full_read())
        ;
    pcileech_bridge_tx_data_write(word);
}

static uint32_t bridge_read(void) {
    while (pcileech_bridge_rx_empty_read())
        ;
    return pcileech_bridge_rx_data_read();
}
```

The C macro names follow the `self.add_csr("pcileech_bridge")` block name.

- [ ] **Step 3: Send one MRd**

For a low address:

```c
bridge_write(0x00000001);          /* MRd32 DW0 */
bridge_write(0x77000000);          /* flag: magic TLP, not last */

bridge_write((requester_id << 16) | ((tag & 0xFF) << 8) | 0xF);
bridge_write(0x77000000);

bridge_write(addr_lo & 0xFFFFFFFCu);
bridge_write(0x77040000);          /* magic + last (bit 18) */
```

For a high address, send DW2/DW3 and set `last` on DW3. The data word always
precedes its flag word. The bridge performs the FT601 byte swap.

- [ ] **Step 4: Parse the mux response**

Read `0x66665555` sync words, then 8-word frames. Parse word 0:

```text
ctx[0] = bits 27:24   data0 = word1
ctx[1] = bits 31:28   data1 = word2
ctx[2] = bits 19:16   data2 = word3
ctx[3] = bits 23:20   data3 = word4
ctx[4] = bits 11: 8   data4 = word5
ctx[5] = bits 15:12   data5 = word6
ctx[6] = bits  3: 0   data6 = word7
```

- Ignore `ctx == 0xF` idle filler.
- Collect words with `ctx & 0x3 == 0` (TLP).
- Use `ctx & 0x4` as the last-DWORD flag (the Migen mux puts
  `tlp_last` in context bit 2).
- Locate the 1-DWORD CplD: three header DWORDs plus data; verify type
  `0x4A`, length 1, byte count 4, status 0, rotating tag, requester ID, and
  lower address.
- Print data MSB-first; stop/report on timeout, sync loss, early `last`,
  partial CplD, or completion error. Drain the bridge RX FIFO at dump start.

- [ ] **Step 5: `status` command**

Report:

- `pcileech_bridge_status_read()` link bit;
- `pcie_phy_link_status_read()`;
- `pcie_phy_bus_master_enable_read()`;
- `main_pcie_id_read()`;
- bridge `tx_full`/`rx_empty`.

- [ ] **Step 6: Build the BIOS**

```bash
venv/bin/python soc_pcileech.py
```

Expected: BIOS builds; `build/soc_pcileech/software/bios/bios.bin` exists,
below 32 KiB; `riscv64-unknown-elf-nm -u` empty; py_compile/diff clean.

Do not build/flash/commit.

---

## Task 5: Static and resource build

**Files:**
- Read: `build/soc_pcileech/gateware/*.rpt`

- [ ] **Step 1: Run bridge simulation and elaborate both SoCs**

```bash
venv/bin/python test/test_pcileech_bridge.py
venv/bin/python test/test_pcileech_bridge_integration.py
venv/bin/python soc_squirrel.py
venv/bin/python soc_pcileech.py
venv/bin/python pcileech_squirrel.py
```

Expected: all exit 0.

- [ ] **Step 2: Build the PCILeechFIFO SoC**

```bash
source env.sh
rm -rf build/soc_pcileech
venv/bin/python soc_pcileech.py --build
```

Use a long timeout. Expected: bitstream
`build/soc_pcileech/gateware/soc_pcileech.bin`.

- [ ] **Step 3: Inspect resources and timing**

```bash
grep -E "Slice LUTs|Slice Registers|Block RAM Tile|RAMB36|RAMB18|DSPs|Bonded IOB|BUFGCTRL" build/soc_pcileech/gateware/soc_pcileech_utilization_place.rpt
```

Expected: <=50 BRAM; target ~43.5. If it does not fit, change the PCILeechFIFO
mux depth to 1024 and rebuild. Report WNS/WHS/DRC.

---

## Task 6: Flash and verify

**Files:**
- Create: `bitstreams/soc_pcileech-console.bin`
- Read: serial console on `/dev/ttyUSB1`

- [ ] **Step 1: Save and flash**

```bash
mkdir -p bitstreams
cp build/soc_pcileech/gateware/soc_pcileech.bin bitstreams/soc_pcileech-console.bin
./program.sh bitstreams/soc_pcileech-console.bin
./rescan.sh
lspci -d 10ee:0666
```

- [ ] **Step 2: Serial checks**

At 115200 8N1 on `/dev/ttyUSB1`:

```text
help
status
dump
```

Expected: `status` shows link/BME/id and bridge FIFOs; default dump reads the
VESA page and matches SHA-256
`5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db`.

- [ ] **Step 3: Arbitrary read**

```text
dump 0x1000000 0x40
```

Expected: 64 bytes, no mux/TLP error, host alive, PCIe still enumerated.

- [ ] **Step 4: Restore fallback**

```bash
./program.sh bitstreams/pcie_squirrel-cleanup-rx128.bin
./rescan.sh
```

Confirm PCILeech enumerates and a live VESA dump matches.

---

## Task 7: Document and commit

**Files:**
- Create: `SOC_PCILEECH.md`
- Modify: `RESOURCES.md`

- [ ] **Step 1: Write `SOC_PCILEECH.md`**

Document the variant’s components, PCILeechFIFO/mux path, CPU bridge, TLP/mux
protocol, FIFO sizes, resource use, build/flash, console, verification, and
accepted risks. Link to the design spec and `SOC.md`.

- [ ] **Step 2: Record resources**

Add the PCILeechFIFO SoC LUT/FF/BRAM/timing and VESA result to
`RESOURCES.md`; leave the first SoC/PCILeech tables intact.

- [ ] **Step 3: Check/stage/commit**

```bash
git diff --check
git add soc_pcileech.py gateware/pcileech_bridge.py \
    software/soc_pcileech_console test/test_pcileech_bridge.py \
    test/test_pcileech_bridge_integration.py \
    SOC_PCILEECH.md RESOURCES.md \
    docs/superpowers/specs/2026-10-28-soc-pcileech-design.md \
    docs/superpowers/plans/2026-10-28-soc-pcileech.md
git diff --cached --stat
git commit -m "add PCILeechFIFO CPU-fed SoC variant"
```

Do not include bitstreams/build artifacts. Do not push.

---

## Rollback

If the PCILeechFIFO variant does not boot or the VESA hash mismatches:

1. Reflash `bitstreams/pcie_squirrel-cleanup-rx128.bin` and rescan.
2. Reflash `bitstreams/soc_squirrel-console.bin` if you want the first SoC.
3. If uncommitted, restore the new files; if committed, revert the variant
   commit by hash.
