# Dependencies and project structure

Date: 2026-11-22

This is a map of the tree that produces the working ECP5 Gen1 endpoint,
and of what would move if that endpoint were proposed to LitePCIe.
[VARIANTS.md](VARIANTS.md) records the bring-up steps.
[PERFORMANCE.md](PERFORMANCE.md) records the measured rates.

## Answer on where the ECP5 PHY belongs

The piece LitePCIe would take is the Migen PHY: `sink` / `source` in
`phy_layout(64)`, plus the config-space and stream adaptation that feed
those ports. That is the same shape as `S7PCIEPHY`. `LitePCIeEndpoint`
already talks to a PHY only through those streams.

The SerDes, LTSSM, and DLL stay in ECP5-PCIe. That core is Amaranth.
LitePCIe is Migen, and its other PHYs do not import a second HDL. A
LitePCIe build can still be self-contained by shipping the generated
Verilog next to the PHY, the way this repo already builds: Yosys reads
`gateware/ecp5pcie/verilog/ecp5_pcie_core_gen1_ref156.v` and does not
import Amaranth unless that file is missing. Regenerating it is a
separate step (`tools/gen_ecp5_pcie_core.py`) that needs Amaranth 0.4
and the ECP5-PCIe checkout.

Making LitePCIe import ECP5-PCIe on every ECP5 build is possible as an
optional generate step. It is a worse default. Every DLL fix would flow
from one repo, and every ECP5 user would need Amaranth plus that
checkout. The S7, UltraScale, Gowin GW5A, and CertusPro-NX PHYs are
wrappers around an external PCIe implementation too; the external part
is vendor IP rather than an Amaranth package.

Search engines, consoles, and the Versa pinout stay outside LitePCIe.
`ChunkDMA`, `ReadBench`, and `TLPDMA` are this project's application.
The board file belongs with litex-boards.

## How one bitstream is assembled

`soc_ecp5_search.py` is the image that was measured. It does not use
`LitePCIeEndpoint`. It connects `ChunkDMA` and `ReadBench` straight to
the PHY streams. `soc_ecp5.py --litepcie` is the path that does use the
LitePCIe endpoint and a BAR0 wishbone window.

```text
soc_ecp5_search.py          this repo, LiteX SoCCore
  ECP5PCIePHY               gateware/ecp5pcie/phy.py          Migen
    TLPSysRouter            gateware/ecp5pcie/sys_router.py   Migen, sys clock
    ConfigSpace             gateware/ecp5pcie/config_space.py Migen, sys clock
    TLPStreamAdapter        gateware/ecp5pcie/adapter.py      32-bit pcie_rx <-> 64-bit sys
    ECP5PCIeCore            gateware/ecp5pcie/core.py         Migen wrapper, adds the .v
      ecp5_pcie_core_gen1_ref156.v
                            gateware/ecp5pcie/verilog/        generated, checked in
        ECP5-PCIe           third_party/ECP5-PCIe             Amaranth source of that .v
  ChunkDMA, ReadBench       gateware/                         this repo
  phy_layout(64)            litepcie.common                   stream format only
  SoC, CPU, UART, CSR       litex                             untracked local clone
```

The PHY presents the ports `LitePCIeEndpoint` expects (`sink`, `source`,
`data_width`, `bar0_size`, `bar0_mask`, `id`, `max_request_size`,
`max_payload_size`) and, on top of those, the control ports this SoC
uses: `_link_status`, `_bus_master_enable`, `cfg_mgmt_*`, and `cd_pcie`
following `sys`. Dword order matches the S7 PHY: `dat[31:0]` is DW0.
The Gowin and CertusPro-NX PHYs are wider and big-endian, so the S7 PHY
is the closer template.

`cd_pcie` is the 100 MHz `sys` clock, not the 62.5 MHz DCU word clock.
The soft core's recovered clocks stay inside the generated Verilog.

## Where each tree lives

| Tree | How this repo holds it | Remote | State on 2026-11-22 |
|---|---|---|---|
| This repo | git | `ecp5-opus` | SoCs, DMA engines, Migen PHY, generated core Verilog |
| `litex/litex`, `litex/litex-boards`, and the other `litex/*` clones | not tracked here | enjoy-digital / litex-hub | `requirements.txt` editable-installs them. `litex` and `litex-boards` match `origin/master` |
| `litex/litepcie` | not tracked here | `enjoy-digital/litepcie` `master` | clean tree, **2 local commits** not on `origin/master` |
| `third_party/ECP5-PCIe` | git submodule | `bengtsjolen/ECP5-PCIe`, branch `pcie-screamer` | parent pins `d7d5fb5`. The fork's `origin` is `6d92448`. The six commits after that are only in this checkout |

`litex/` is a `litex_setup.py` checkout sitting in the working tree.
Nothing under it is committed by this repo. A second machine needs those
clones, including the two LitePCIe commits, or the editable install
builds upstream LitePCIe.

The submodule pin `d7d5fb5` cannot be fetched from the fork until those
six commits are pushed. A normal bitstream build does not need the
submodule: `ECP5PCIeCore` uses the checked-in Verilog when the file is
already there.

## Layer by layer

### ECP5-PCIe (Amaranth)

Fork point is upstream `c511d2e`. On the fork, `6d92448` adds the
`ECP5PCIeCore` black box used by the generator. After that, only in this
checkout:

- `633b0f5` Recovery, lost Ack, 156.25 MHz Gen1 refclk
- `d82251b` DLL TX/RX data paths
- `d6dc535` cumulative flow-control credits
- `2c08d06` DLL RX reorder
- `03ee03c` unacknowledged-TLP FIFO clock domain
- `d7d5fb5` replay timer, per-symbol scramble, TX scramble after SKP

`tools/gen_ecp5_pcie_core.py` elaborates that Amaranth core to RTLIL and
Yosys writes flat Verilog. The file the Gen1 Versa build actually
compiles is `gateware/ecp5pcie/verilog/ecp5_pcie_core_gen1_ref156.v`.
Its header says `ECP5-PCIe 03ee03c-dirty`. The submodule HEAD has since
moved to `d7d5fb5` and the Verilog has not been regenerated, so the
source checkout and the snapshot inside the bitstream can differ.

### Migen PHY in this repo

| File | Role |
|---|---|
| `gateware/ecp5pcie/phy.py` | LitePCIe-facing PHY, Squirrel control CSRs |
| `gateware/ecp5pcie/core.py` | clock domains, adds the generated Verilog, bring-up CSRs |
| `gateware/ecp5pcie/adapter.py` | 32-bit core TLP words to 64-bit `sys` beats |
| `gateware/ecp5pcie/config_space.py` | config space answered in `sys` |
| `gateware/ecp5pcie/sys_router.py` | config completions, memory requests, completions to the DMA engines |
| `gateware/ecp5pcie/router.py` | earlier router, used by the in-core transaction-layer path |
| `gateware/ecp5pcie/capture.py` | TLP capture FIFO for bring-up |
| `gateware/ecp5pcie/host_dma.py` | small bus-master engine used before ChunkDMA |

These are the files a LitePCIe PHY would be built from. `capture.py` and
`host_dma.py` are bring-up tools and can stay here.

### LitePCIe local commits

Neither commit is the ECP5 core.

| Commit | What it changes | Who needs it |
|---|---|---|
| `33defbe` | `tlp/depacketizer.py`: an error completion ends the request | `soc_ecp5.py --litepcie`, any `LitePCIeEndpoint` on this PHY |
| `7245ec3` | `phy/s7pciephy.py`: `cfg_mgmt` and AXIS taps | Squirrel SoCs. The ECP5 PHY implements `cfg_mgmt` itself |

The search image does not instantiate `LitePCIeEndpoint`, so it does not
exercise the depacketizer. The two commits are still local: they are
not in this repo's git history. They can be proposed to
`enjoy-digital/litepcie` on their own, ahead of any ECP5 PHY.

`litex/litex` has no local commits.

### This repo, apart from the PHY

| Piece | Lives in | Pair |
|---|---|---|
| Squirrel PCILeech gateware | `gateware/tlp.py`, `tlp128.py`, `pcileech_fifo.py`, `pcileech_squirrel.py` | Artix-7, FT601 |
| One-dword DMA | `gateware/tlp_dma.py`, `soc_squirrel.py`, `soc_ecp5_v2.py` | V2 |
| One outstanding 4 KiB read | `gateware/chunk_dma.py`, `soc_ecp5_v4.py` | V4 |
| Four outstanding reads | `gateware/read_bench.py`, `soc_squirrel_search.py`, `soc_ecp5_search.py` | V5 |
| Consoles | `software/soc_console`, `soc_chunk_console`, `soc_search_console` | UART |
| Versa platform | `platforms/pcie_versa.py` | wraps litex-boards `lattice_versa_ecp5`, device chosen explicitly (`LFE5UM` or `LFE5UM5G`) |

## What a LitePCIe ECP5 PHY would contain

In `litepcie/phy/`, next to `s7pciephy.py`:

- the Migen PHY class (`ECP5PCIePHY`)
- the adapter, config space, and sys router it needs
- the generated Verilog for the configurations that are actually tested
  (today: Gen1, 156.25 MHz refclk, transaction layer left outside the core)

A LiteX target would then do what `soc_ecp5.py --litepcie` does now:
construct the PHY, pass it to `LitePCIeEndpoint`, and add a wishbone
master. The Versa target itself would be a litex-boards platform plus a
small target script, not part of the PHY.

Kept here, and not part of that PHY:

- `ChunkDMA`, `ReadBench`, `TLPDMA`, the consoles, and the search SoCs
- the Squirrel FT601 design
- TLP capture and the early host-DMA scratch engine

The extra control CSRs (`_link_status`, `_bus_master_enable`,
`cfg_mgmt_*`) are what the search SoC drives. They can travel with the
PHY, since they are how software brings the link up, or this repo can
keep a thin subclass if upstream wants the PHY limited to the endpoint
ports.

## Drift to fix before that move

- Push `633b0f5`..`d7d5fb5` to `bengtsjolen/ECP5-PCIe`, or the submodule
  pin in this repo does not exist on the remote.
- Regenerate `ecp5_pcie_core_gen1_ref156.v` from a clean `d7d5fb5` (or
  from whatever revision is chosen) so the header matches the source.
  The current header is `03ee03c-dirty`.
- The two LitePCIe commits need a home: a branch of
  `enjoy-digital/litepcie`, or a note that the ECP5 endpoint path
  requires them. They are invisible to anyone who only clones this repo.
