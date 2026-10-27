# PCIe Squirrel FPGA resource use

Date: 2026-10-27

Device: Xilinx XC7A35T-FGG484-2, Vivado 2021.2, placed utilization.
The default (large FIFO) design is the committed 128-bit RX path.

## Current utilization (large FIFOs, RX128)

| Resource | Used | Available | Util |
|---|---:|---:|---:|
| Slice LUTs | 4,200 | 20,800 | 20.19% |
| — LUT as logic | 3,747 | 20,800 | 18.01% |
| — LUT as memory | 453 | 9,600 | 4.72% |
| Slice registers | 3,437 | 41,600 | 8.26% |
| F7 muxes | 30 | 16,300 | 0.18% |
| F8 muxes | 2 | 8,150 | 0.02% |
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

## Cleanup verification (2026-10-27)

The cleaned large build was verified with PCILeech 4.17.8.49:

- 1 GiB: 151.48 MiB/s, 262144/262144 pages, 0 failed.
- 2 GiB: 147.83 MiB/s valid, 483328/524288 pages, 40960 known holes.
- VESA ROM `0x3f000-0x40000`: SHA-256
  `5c091e71f7c3190d947f78769b0cc226d1c340c66c5e955244ff40e0f4a4c8db`,
  matching the ufrisk reference.
- Bitstream: `pcie_squirrel-cleanup-rx128.bin`, SHA-256
  `33848540d0efaacbd975321ff99ea14108051451be81c75fb5eb44f9e9e2297b`.

The build reports WNS -1.421 ns / 103 failing endpoints on PCIe
clock-domain reset paths. The existing false-path commands are emitted
before clock creation and appear to be discarded; this constraint issue is
outside the cleanup scope and is not timing closure.

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

## Presets

These are the named configurations exposed by `--fifo-preset`:

| Preset | mux_out | TLP RX | FT601 write | FT601 read | BRAM | 1G MiB/s | 2G valid MiB/s | Use |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| `large` | 2048 | 2048 | 8192 | 128 | 41.5 | 147-149 | 143.4 | maximum performance |
| `medium` | 1024 | 2048 | 4096 | 128 | 30.5 | 137.4 | 133.3 | best reduced-BRAM balance |
| `mini` | 1024 | 2048 | 1024 | 128 | 27.5 | 100.8 | 102.2 | smallest fully functional |

`mini` uses the 1024-word FT601 write FIFO. The 512-word variant
(Hybrid F below) is marginally smaller but fails the 2 GiB dump.

## FIFO depth sweep

All measurements use PCILeech 4.17.8.49, USB 3.0 (5000 Mbps), the same
1 GiB range and the same 2 GiB range as `PERFORMANCE.md`. “Functional”
means PCILeech could connect, the 4 KiB VESA ROM region matched the old
ufrisk dump, and a 1 GiB dump completed with 0 failed pages. The 2 GiB
range always has 40960 known memory-map holes, so a working run reports
about 40960 failed pages there.

The default/large configuration is **mux_out 2048, TLP RX 2048,
FT601 write 8192**. The depths are independent; there is no fixed ratio
between the 256-bit mux FIFO and the 32-bit FT601 FIFO (the 4x entry
difference in the defaults is just 64 KiB vs 32 KiB). “Half” and
“Quarter” below scale all default parameters by 2x and 4x respectively.

The LUT/FF columns in this sweep were measured before the diagnostic
cleanup; use the Current utilization table for the cleaned build.

| Name | mux_out | TLP RX | FT601 write | BRAM | Δ BRAM | LUTs | FFs | 1G MiB/s | 2G valid MiB/s | 1G failed | 2G failed | Functional |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Large (default) | 2048 | 2048 | 8192 | 41.5 | 0 | 4,674 | 4,944 | **147.1** | **143.4** | 0 | 40960 | yes |
| Hybrid A | 2048 | 512 | 8192 | 36.0 | -5.5 | 4,688 | 4,936 | invalid | invalid | 236,443 | 484,141 | no |
| Hybrid B | 1024 | 2048 | 4096 | 30.5 | -11.0 | 4,645 | 4,933 | **137.4** | **133.3** | 0 | 40960 | yes (`medium`) |
| Hybrid D | 1024 | 2048 | 2048 | 28.5 | -13.0 | 4,657 | 4,925 | 119.6 | 111.3 | 0 | 40960 | yes |
| Hybrid E | 1024 | 2048 | 1024 | 27.5 | -14.0 | 4,673 | 4,917 | 100.8 | 102.2 | 0 | 40960 | yes (`mini`) |
| Hybrid F | 1024 | 2048 | 512 | 27.0 | -14.5 | 4,667 | 4,909 | 94.3 | 74.5 | 0 | 406,871 | no (2G fails) |
| Half | 1024 | 1024 | 4096 | 27.0 | -14.5 | 4,638 | 4,929 | 109.4 | 106.2 | 2 | 40961 | degraded |
| Hybrid C | 512 | 2048 | 2048 | 25.0 | -16.5 | 4,645 | 4,923 | 90.0 | 90.9 | 9 | 40979 | degraded |
| Quarter | 512 | 512 | 1024 | 18.5 | -23.0 | 4,694 | 4,907 | n/a | n/a | n/a | n/a | no (connect fails) |

Notes:

* **Hybrid A** (TLP RX 512) connected and passed the single-page VESA
  test, but under load it lost most completions: 1 GiB read 25,701 of
  262,144 pages and 2 GiB read 40,147 of 524,288. The PCILeech speed
  display is meaningless when pages fail.
* **Quarter** (all FIFOs minimum) responds on the CMD path, but PCILeech
  cannot complete initialization. 18.5 tiles is too small for the init
  burst.
* **Half** is the slowest 1 GiB working configuration (109 MiB/s) and had
  2 failed pages. Reducing the TLP RX FIFO to 1024 is the main cause:
  Hybrid B keeps the same output FIFOs as Half but restores the RX FIFO to
  2048 and recovers to 137 MiB/s with 0 failed pages.
* **Hybrid B** is the `medium` preset: 30.5/50 tiles (61%), ~137 MiB/s
  on 1 GiB, 0 failed pages, VESA match. It saves 11 BRAM tiles at a cost
  of ~7% steady-state throughput.
* **Hybrid E** is the `mini` preset: 27.5/50 tiles (55%), ~101 MiB/s,
  0 failed pages. This is the smallest fully functional configuration;
  the extra 0.5 tile for the 1024-word write FIFO is worth it over the
  512-word Hybrid F.
* **Hybrid F** (512-word write FIFO) works on 1 GiB but loses most of the
  2 GiB dump (117,417 pages read, 406,871 failed). 512 words is not a
  safe minimum.
* At a fixed mux 1024 / RX 2048, the FT601 write FIFO dominates
  performance: 512 words = 94.3 MiB/s (fails 2G), 1024 = 100.8 MiB/s,
  2048 = 119.6 MiB/s, 4096 = 137.4 MiB/s. The original 1024-word FIFO
  was functional but was indeed a major throughput bottleneck; 4096 is
  the best reduced-BRAM value and 8192 remains best for full performance.
* **Hybrid D** (1024-frame mux FIFO + 2048-word write FIFO, RX 2048)
  works and has 0 failed pages, but drops to 119.6 MiB/s on 1 GiB. The
  write FIFO can be 2048 words, but 4096 is the better performance/BRAM
  trade-off.
* **Hybrid C** (512-frame mux FIFO + 2048-word write FIFO, RX 2048)
  connects and passes VESA, but is too aggressive: 1 GiB drops to
  90.0 MiB/s with 9 failed pages and 2 GiB has 40979 failed (19 beyond
  the known holes). Since Hybrid D works with the same 2048-word write
  FIFO but a 1024-frame mux FIFO, the 512-frame mux FIFO is the main
  cause of the C failures.
* The output FIFOs can be reduced to 1024 frames + 4096 words as long as
  the 128-bit TLP RX FIFO stays at 2048. The RX FIFO is the first thing
  that breaks when reduced; below Hybrid B, the output FIFOs also start
  losing data.

## Building a custom FIFO configuration

Select a named preset:

```sh
./venv/bin/python pcileech_squirrel.py --build --fifo-preset medium
./venv/bin/python pcileech_squirrel.py --build --fifo-preset mini
```

The preset values are listed in the table above; the default is `large`.
Individual depth options override the preset for experimentation:

```sh
./venv/bin/python pcileech_squirrel.py --build \
    --fifo-preset medium \
    --mux-fifo-depth 512 \
    --ft601-write-fifo-depth 2048
```

The individual options are `--mux-fifo-depth`, `--tlp-rx-fifo-depth`,
`--ft601-write-fifo-depth` and `--ft601-read-fifo-depth`. Use
power-of-two depths for predictable BRAM inference. After changing FIFO
depths, rebuild, flash, and rescan the PCIe device. The benchmark script
records the performance:

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
