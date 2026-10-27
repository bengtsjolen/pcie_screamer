# PCIe Squirrel FPGA resource use

Date: 2026-10-27

Device: Xilinx XC7A35T-FGG484-2, Vivado 2021.2, placed utilization.
The default (large FIFO) design is the committed 128-bit RX path.

## Current utilization (large FIFOs, RX128)

| Resource | Used | Available | Util |
|---|---:|---:|---:|
| Slice LUTs | 4,674 | 20,800 | 22.47% |
| — LUT as logic | 4,221 | 20,800 | 20.29% |
| — LUT as memory | 453 | 9,600 | 4.72% |
| Slice registers | 4,944 | 41,600 | 11.88% |
| F7 muxes | 81 | 16,300 | 0.50% |
| F8 muxes | 8 | 8,150 | 0.10% |
| Block RAM tiles | **41.5** | **50** | **83.00%** |
| — RAMB36/FIFO | 39 | 50 | 78.00% |
| — RAMB18 | 5 | 100 | 5.00% |
| DSP slices | 0 | 90 | 0.00% |
| Bonded IOB | 46 | 250 | 18.40% |
| BUFGCTRL | 9 | 32 | 28.13% |
| MMCM / PLL | 1 / 1 | 5 / 5 | 20.00% |

BRAM is the limiting resource: ~8.5 tiles (~38 KiB) remain. LUT and
register headroom is large; a small VexRiscv or control state machine fits
without touching the FIFOs, but a sizeable local RAM would need BRAM
reclaimed from the FIFOs.

## Where the BRAM goes (approximate)

| Block | Approx. tiles | Notes |
|---|---:|---|
| `mux_out_fifo` | 15 | 2048 × 256-bit = 64 KiB |
| 128-bit TLP RX FIFO | 8 | 2048 × 134-bit ≈ 34 KiB |
| FT601 write FIFO | 8 | 8192 × 32-bit = 32 KiB |
| PCIe hard IP | 8 | Xilinx `pcie_s7` internal buffers |
| Small FIFOs | 2.5 | USB read, TLP TX, CMD/CFG/loopback |
| **Total** | **41.5** | RAMB36 + RAMB18 tile equivalents |

## FT601 controller cost

Measured with a standalone build containing only the minimal SoC/CRG
wrapper and `FT601Sync`, default 8192-word write FIFO:

| Resource | FT601 build | Empty wrapper | FT601 contribution |
|---|---:|---:|---:|
| LUTs | 176 | 0 | **176** |
| Registers | 319 | 8 | **311** |
| Block RAM tiles | 8 | 0 | **8** |
| DSPs | 0 | 0 | **0** |
| Bonded IOB | 45 | 1 | **44** |
| BUFGCTRL | 2 | 1 | **1** |

The controller logic is tiny; almost all of its BRAM is the 32 KiB write
FIFO. Reducing `--ft601-write-fifo-depth` reduces the 8 tiles roughly
proportionally (4096 -> ~4 tiles, 1024 -> ~1 tile).

Removing the FT601 for an on-chip SoC stream would therefore free
**~8 BRAM tiles, ~176 LUTs, ~311 registers, and 44 I/O pins**, plus one
BUFG. That is enough headroom for a small VexRiscv plus a modest local
RAM, especially if the write FIFO is not needed at full depth.

## FIFO depth sweep

All measurements use PCILeech 4.17.8.49, USB 3.0 (5000 Mbps), the same
1 GiB range and the same 2 GiB range as `PERFORMANCE.md`. “Functional”
means PCILeech could connect, the 4 KiB VESA ROM region matched the old
ufrisk dump, and a 1 GiB dump completed with 0 failed pages. 2 GiB always
has 40960 known memory-map holes.

The default/large configuration is **mux_out 2048, TLP RX 2048,
FT601 write 8192**. Every other row changes one or more of those values
relative to this default.

| Name | mux_out | TLP RX | FT601 write | BRAM | Δ BRAM | LUTs | FFs | 1G MiB/s | 2G valid MiB/s | 1G failed | Functional |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Large (default) | 2048 | 2048 | 8192 | 41.5 | 0 | 4,674 | 4,944 | **147.1** | **143.4** | 0 | yes |
| Hybrid B | 1024 | 2048 | 4096 | 30.5 | -11.0 | 4,645 | 4,933 | **137.4** | **133.3** | 0 | yes |
| Medium | 1024 | 1024 | 4096 | 27.0 | -14.5 | 4,638 | 4,929 | 109.4 | 106.2 | 2 | degraded |
| Hybrid A | 2048 | 512 | 8192 | 36.0 | -5.5 | 4,688 | 4,936 | invalid | invalid | 236,443 | no |
| Hybrid C | 512 | 2048 | 2048 | 25.0 | -16.5 | 4,645 | 4,923 | 90.0 | 90.9 | 9 | degraded |
| Small | 512 | 512 | 1024 | 18.5 | -23.0 | 4,694 | 4,907 | n/a | n/a | n/a | no (connect fails) |

Notes:

* **Hybrid A** (TLP RX 512) connected and passed the single-page VESA
  test, but under load it lost most completions: 1 GiB read 25,701 of
  262,144 pages and 2 GiB read 40,147 of 524,288. The PCILeech speed
  display is meaningless when pages fail.
* **Small** (all FIFOs minimum) responds on the CMD path, but PCILeech
  cannot complete initialization. 18.5 tiles is too small for the init
  burst.
* **Medium** is the slowest working configuration (109 MiB/s) and had 2
  failed pages on 1 GiB. Reducing the TLP RX FIFO to 1024 is the main
  cause: Hybrid B keeps the same output FIFOs as Medium but restores the
  RX FIFO to 2048 and recovers to 137 MiB/s with 0 failed pages.
* **Hybrid B** is the best low-BRAM option: 30.5/50 tiles (61%), ~137
  MiB/s on 1 GiB, 0 failed pages, VESA match. It saves 11 BRAM tiles at
  a cost of ~7% steady-state throughput.
* **Hybrid C** (512-frame mux FIFO + 2048-word write FIFO, RX 2048)
  connects and passes VESA, but is too aggressive: 1 GiB drops to
  90.0 MiB/s with 9 failed pages and 2 GiB has 40979 failed (19 beyond
  the known holes). Both output FIFOs need more depth than this.
* The output FIFOs can be reduced to 1024 frames + 4096 words as long as
  the 128-bit TLP RX FIFO stays at 2048. The RX FIFO is the first thing
  that breaks when reduced; below Hybrid B, the output FIFOs also start
  losing data.

## Building a custom FIFO configuration

The top level exposes the FIFO depths directly:

```sh
./venv/bin/python pcileech_squirrel.py --build \
    --mux-fifo-depth 1024 \
    --tlp-rx-fifo-depth 2048 \
    --ft601-write-fifo-depth 4096
```

`--ft601-read-fifo-depth` is also available (default 128). The defaults
are the large configuration:

* `--mux-fifo-depth 2048`
* `--tlp-rx-fifo-depth 2048`
* `--ft601-write-fifo-depth 8192`
* `--ft601-read-fifo-depth 128`

Use power-of-two depths for predictable BRAM inference. After changing
FIFO depths, rebuild, flash, and rescan the PCIe device. The benchmark
script records the performance:

```sh
./bench-pcileech.sh <label> 1G 2G
```

Resource reports are generated by the build in
`build/pcie_squirrel/gateware/pcie_squirrel_utilization_place.rpt`.

## Future on-chip SoC / no-FT601 build

The FT601 path is already isolated behind `FT601Sync` and the
`usb_rx`/`usb_tx` 32-bit endpoints in `PCILeechFIFO`. Removing the FT601
and connecting a VexRiscv/SoC stream there would free ~8 BRAM tiles,
~176 LUTs, ~311 registers and 44 I/O pins. The 128-bit TLP RX path and
the mux/output FIFOs are independent of the FT601 and can remain
unchanged. The main BRAM budget decision is then the output FIFO depth
(`mux_out_fifo`) and the TLP RX FIFO depth, not the FT601 controller.
